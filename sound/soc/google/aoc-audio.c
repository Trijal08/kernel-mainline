// SPDX-License-Identifier: GPL-2.0
/*
 * Google AOC (Always-On Compute) ASoC platform + card.
 *
 * The AOC does the audio DSP and mixing and drives the physical amplifiers over
 * its own TDM; the AP streams PCM to it through the audio_playbackN IPC rings.
 * This is the ASoC platform component that presents such a ring as a PCM device
 * (the ring is the DMA buffer: the AP produces, the AOC consumes and advances
 * the read pointer), plus a minimal card so one PCM enumerates.
 *
 * The stream reaches the speakers as a front-end/back-end pair.  The front-end
 * is the ring the AP writes; the back-end is the AOC's TDM_0 port, which the
 * AOC clocks itself and which both CS35L41 amplifiers listen to.  The two are
 * separate because the AOC resamples and mixes: what the AP streams and what
 * comes out of the TDM are unrelated, and only the latter describes the wire.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/soc/google/aoc.h>

#include <sound/cs35l41.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AOC_PCM_BUFFER_BYTES	(512 * 1024)
/*
 * Entry point 2.  Not the first one: that is the AOC's ultra-low-latency
 * endpoint, which expects its buffer filled ahead of the reader rather than
 * streamed to, so writing to it leaves the speaker mixer starved and the TDM
 * clocking out an empty buffer.
 */
#define AOC_PLAYBACK_SERVICE	"audio_playback1"
#define AOC_PLAYBACK_SOURCE	1
#define AOC_OUTPUT_CTRL_SERVICE	"audio_output_control"
#define AOC_CMD_TIMEOUT_MS	200

/*
 * The AOC's TDM_0 port, which the speakers hang off.  Its frame is fixed in AOC
 * firmware and is not negotiable from here, so it is spelled out rather than
 * derived from the stream: four 32-bit slots at 48kHz, i.e. a 6.144MHz bit
 * clock.  The two amplifiers take a slot each and ignore the rest.
 */
#define AOC_TDM0_SLOTS		4
#define AOC_TDM0_SLOT_WIDTH	32
#define AOC_TDM0_RATE		48000
#define AOC_TDM0_CHANNELS	2
/* 24 bits of audio in each 32-bit slot: the amplifiers are 24-bit parts. */
#define AOC_TDM0_FORMAT		SNDRV_PCM_FORMAT_S24_LE

/*
 * AOC control-command framing (from the vendor aoc-interface): a CONTAINER_HDR
 * (type/counter/length) followed by a command id and reply field.
 */
#define AOC_CMD_TYPE_CMD		0
#define AOC_CMD_AUDIO_OUTPUT_SOURCE_ID	201

struct aoc_cmd_hdr {
	u8 type;
	u8 cntr;
	__le16 len;
	__le16 id;
	__le16 reply;
} __packed;

/* Enable/disable a playback source (entry point) in the AOC. */
struct cmd_audio_output_source {
	struct aoc_cmd_hdr hdr;
	u8 source;
	u8 on;
} __packed;

/*
 * The endpoint setup that must precede source-on: it hands the AOC the stream
 * format and the ring geometry so it knows how (and how fast) to consume.
 */
#define AOC_CMD_AUDIO_OUTPUT_EP_SETUP_ID	206
#define AOC_CMD_AUDIO_OUTPUT_EP_SETUP2_ID	298
#define AOC_EP_MODE_PLAYBACK			0
#define AOC_WATERMARK_DEFAULT			48000
#define AOC_WIDTH_16_BIT			1	/* enum BitsPerSample */
#define AOC_WIDTH_32_BIT			3
#define AOC_FRMT_FIXED_POINT			1	/* enum Format */
#define AOC_SR_44K1HZ				4	/* enum SampleRate */
#define AOC_SR_48KHZ				5

struct aoc_chan_metadata {
	u8 chan;
	u8 bits;
	u8 sr;
	u8 format;
	u8 offset;
} __packed;

struct aoc_ep_descriptor {
	__le32 address;
	__le32 watermark;
	__le32 length;
	struct aoc_chan_metadata metadata;
	u8 channel;
	u8 wraparound;
} __packed;

struct cmd_audio_output_ep_setup {
	struct aoc_cmd_hdr hdr;
	struct aoc_ep_descriptor d;
	u8 mode;
} __packed;

struct cmd_audio_output_ep_setup2 {
	struct aoc_cmd_hdr hdr;
	u8 channel;
	__le32 buffer_size;
	__le32 period_size;
	u8 mode;
} __packed;

/* Route a source to a sink; without this the source has nowhere to play. */
#define AOC_CMD_AUDIO_OUTPUT_BIND_ID		271	/* the "bind2" form */
#define AOC_SINK_SPEAKER			0	/* enum AUDIO_SINKS_ID */

struct cmd_audio_output_bind {
	struct aoc_cmd_hdr hdr;
	u8 src;
	u8 dst;
	u8 bind;
} __packed;

struct aoc_audio {
	struct device *aoc_dev;			/* the AOC platform device */
	struct snd_soc_card card;
	struct aoc_service *ctrl;		/* audio_output_control channel */
	struct mutex cmd_lock;			/* serialises control commands */
	struct completion cmd_done;		/* a control reply arrived */
};

/* Per-open stream state. */
struct aoc_audio_stream {
	struct aoc_service *svc;
	bool playback;
	u8 source;				/* AOC entry point = PCM device */
	u32 base;				/* AOC byte position at start */
	snd_pcm_uframes_t pushed;		/* frames handed to the ring */
};

/*
 * Recover our private data from a substream.  Not from the platform device's
 * drvdata: snd_soc_register_card() overwrites that with the card pointer, so we
 * go through the card, which is embedded in struct aoc_audio.
 */
static struct aoc_audio *substream_to_aud(struct snd_pcm_substream *substream)
{
	return container_of(snd_soc_substream_to_rtd(substream)->card,
			    struct aoc_audio, card);
}

/*
 * This component provides the TDM back-end's CPU DAI as well as the front-end's
 * ring, so its PCM callbacks are invoked for the back-end too.  The back-end has
 * no ring and no AOC endpoint of its own; only the front-end moves data.
 */
static bool aoc_pcm_is_be(struct snd_pcm_substream *substream)
{
	return snd_soc_substream_to_rtd(substream)->dai_link->no_pcm;
}

/* The AOC replied on the control channel. */
static void aoc_audio_ctrl_isr(struct aoc_service *svc, void *priv)
{
	struct aoc_audio *aud = priv;

	complete(&aud->cmd_done);
}

/*
 * Send a control command and wait for its reply.  Runs in process context (the
 * dai link is nonatomic), and is woken by the control channel's doorbell rather
 * than by polling.
 */
static int aoc_audio_cmd(struct aoc_audio *aud, const void *req, size_t req_len,
			 void *rsp, size_t rsp_len)
{
	struct device *dev = aud->card.dev;
	char drain[64];
	int ret;

	if (!aud->ctrl) {
		dev_err(dev, "no audio control channel\n");
		return -ENODEV;
	}

	mutex_lock(&aud->cmd_lock);

	/* Discard any stale replies left in the channel. */
	while (aoc_service_can_read(aud->ctrl))
		aoc_service_read(aud->ctrl, drain, sizeof(drain));

	reinit_completion(&aud->cmd_done);
	ret = aoc_service_write(aud->ctrl, req, req_len);
	if (ret < 0) {
		dev_err(dev, "control write failed: %d\n", ret);
		goto out;
	}

	if (!wait_for_completion_timeout(&aud->cmd_done,
					 msecs_to_jiffies(AOC_CMD_TIMEOUT_MS))) {
		dev_err(dev, "control command timed out (no reply)\n");
		ret = -ETIMEDOUT;
		goto out;
	}
	ret = aoc_service_read(aud->ctrl, rsp, rsp_len);
	if (ret < 0)
		dev_err(dev, "control read failed: %d\n", ret);
out:
	mutex_unlock(&aud->cmd_lock);
	return ret < 0 ? ret : 0;
}

/* Turn a playback source (entry point) on or off. */
static int aoc_audio_source(struct aoc_audio *aud, u8 source, bool on)
{
	struct cmd_audio_output_source cmd = { };
	char rsp[64];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_SOURCE_ID);
	cmd.source = source;
	cmd.on = on ? 1 : 0;
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/* Configure the playback endpoint's format and ring geometry (bytes). */
static int aoc_audio_ep_setup(struct aoc_audio *aud, u8 source,
			      unsigned int channels, unsigned int rate,
			      unsigned int width, u32 buffer_bytes,
			      u32 period_bytes)
{
	struct cmd_audio_output_ep_setup cmd = { };
	struct cmd_audio_output_ep_setup2 cmd2 = { };
	char rsp[64];
	int ret;

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_EP_SETUP_ID);
	cmd.d.watermark = cpu_to_le32(AOC_WATERMARK_DEFAULT);
	cmd.d.channel = source;
	cmd.d.wraparound = 1;
	cmd.d.metadata.chan = channels;
	cmd.d.metadata.bits = (width == 32) ? AOC_WIDTH_32_BIT : AOC_WIDTH_16_BIT;
	cmd.d.metadata.sr = (rate == 48000) ? AOC_SR_48KHZ : AOC_SR_44K1HZ;
	cmd.d.metadata.format = AOC_FRMT_FIXED_POINT;
	cmd.mode = AOC_EP_MODE_PLAYBACK;
	ret = aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (ret)
		return ret;

	cmd2.hdr.type = AOC_CMD_TYPE_CMD;
	cmd2.hdr.len = cpu_to_le16(sizeof(cmd2));
	cmd2.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_EP_SETUP2_ID);
	cmd2.channel = source;
	cmd2.buffer_size = cpu_to_le32(buffer_bytes);
	cmd2.period_size = cpu_to_le32(period_bytes);
	cmd2.mode = AOC_EP_MODE_PLAYBACK;
	return aoc_audio_cmd(aud, &cmd2, sizeof(cmd2), rsp, sizeof(rsp));
}

/* Bind (or unbind) a source to a sink. */
static int aoc_audio_bind(struct aoc_audio *aud, u8 src, u8 dst, bool on)
{
	struct cmd_audio_output_bind cmd = { };
	char rsp[64];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_BIND_ID);
	cmd.src = src;
	cmd.dst = dst;
	cmd.bind = on ? 1 : 0;
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

static const struct snd_pcm_hardware aoc_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
	.rate_min = 44100,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = AOC_PCM_BUFFER_BYTES,
	.period_bytes_min = 256,
	.period_bytes_max = 65536,
	.periods_min = 2,
	.periods_max = 64,
};

/* The AOC signalled this service: a period has been consumed/produced. */
static void aoc_pcm_isr(struct aoc_service *svc, void *priv)
{
	struct snd_pcm_substream *substream = priv;

	snd_pcm_period_elapsed(substream);
}

static int aoc_pcm_open(struct snd_soc_component *comp,
			struct snd_pcm_substream *substream)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	struct aoc_audio_stream *s;
	struct aoc_service *svc;

	if (aoc_pcm_is_be(substream))
		return 0;

	svc = aoc_service_find(aud->aoc_dev, AOC_PLAYBACK_SERVICE);
	if (!svc)
		return -ENODEV;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->svc = svc;
	s->playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	s->source = AOC_PLAYBACK_SOURCE;	/* the entry point the ring feeds */
	substream->runtime->private_data = s;

	snd_soc_set_runtime_hwparams(substream, &aoc_pcm_hw);
	aoc_service_set_handler(svc, aoc_pcm_isr, substream);
	return 0;
}

static int aoc_pcm_close(struct snd_soc_component *comp,
			 struct snd_pcm_substream *substream)
{
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (s) {
		aoc_service_set_handler(s->svc, NULL, NULL);
		kfree(s);
		substream->runtime->private_data = NULL;
	}
	return 0;
}

static int aoc_pcm_new(struct snd_soc_component *comp,
		       struct snd_soc_pcm_runtime *rtd)
{
	if (rtd->dai_link->no_pcm)
		return 0;

	/* The PCM buffer is a plain staging buffer; .ack copies it to the ring. */
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC,
				       comp->dev, AOC_PCM_BUFFER_BYTES,
				       AOC_PCM_BUFFER_BYTES);
	return 0;
}

static int aoc_pcm_prepare(struct snd_soc_component *comp,
			   struct snd_pcm_substream *substream)
{
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (aoc_pcm_is_be(substream))
		return 0;

	s->base = aoc_service_progress(s->svc, s->playback);
	s->pushed = 0;
	return 0;
}

/* The application committed frames up to appl_ptr; hand the new ones to the ring. */
static int aoc_pcm_ack(struct snd_soc_component *comp,
		       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *rt = substream->runtime;
	struct aoc_audio_stream *s = rt->private_data;
	snd_pcm_uframes_t appl = READ_ONCE(rt->control->appl_ptr);

	if (!s->playback)
		return 0;

	while (s->pushed < appl) {
		snd_pcm_uframes_t off = s->pushed % rt->buffer_size;
		snd_pcm_uframes_t n = min_t(snd_pcm_uframes_t, appl - s->pushed,
					    rt->buffer_size - off);
		int nbytes = frames_to_bytes(rt, n);
		int w = aoc_service_write(s->svc,
					  rt->dma_area + frames_to_bytes(rt, off),
					  nbytes);

		if (w <= 0)			/* ring full: the AOC is not draining yet */
			break;
		s->pushed += bytes_to_frames(rt, w);
		if (w < nbytes)
			break;
	}
	return 0;
}

static snd_pcm_uframes_t aoc_pcm_pointer(struct snd_soc_component *comp,
					 struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *rt = substream->runtime;
	struct aoc_audio_stream *s = rt->private_data;
	u32 buf_bytes = frames_to_bytes(rt, rt->buffer_size);
	u32 pos = aoc_service_progress(s->svc, s->playback) - s->base;

	return bytes_to_frames(rt, pos % buf_bytes);
}

/* Program the AOC endpoint before it is started, from the negotiated format. */
static int aoc_pcm_hw_params(struct snd_soc_component *comp,
			     struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	struct aoc_audio_stream *s = substream->runtime->private_data;
	int ret;

	if (aoc_pcm_is_be(substream))
		return 0;

	ret = aoc_audio_ep_setup(aud, s->source, params_channels(params),
				 params_rate(params), params_width(params),
				 params_buffer_bytes(params),
				 params_period_bytes(params));
	if (ret) {
		dev_err(comp->dev, "endpoint setup failed: %d\n", ret);
		return ret;
	}

	/*
	 * Route the source to the speaker here rather than at trigger: this is
	 * what makes the AOC clock its TDM, and the amplifiers are powered up
	 * before the trigger and will not complete their own power-up until
	 * they can lock to that clock.
	 */
	ret = aoc_audio_bind(aud, s->source, AOC_SINK_SPEAKER, true);
	if (ret)
		dev_err(comp->dev, "speaker bind failed: %d\n", ret);
	return ret;
}

static int aoc_pcm_hw_free(struct snd_soc_component *comp,
			   struct snd_pcm_substream *substream)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (aoc_pcm_is_be(substream))
		return 0;

	return aoc_audio_bind(aud, s->source, AOC_SINK_SPEAKER, false);
}

/* Start/stop the AOC pulling from the ring (the dai link is nonatomic). */
static int aoc_pcm_trigger(struct snd_soc_component *comp,
			   struct snd_pcm_substream *substream, int cmd)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	struct aoc_audio_stream *s = substream->runtime->private_data;
	bool on;
	int ret;

	if (aoc_pcm_is_be(substream))
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		on = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		on = false;
		break;
	default:
		return -EINVAL;
	}

	/* The route is already up; this only starts and stops the flow. */
	ret = aoc_audio_source(aud, s->source, on);
	dev_dbg(comp->dev, "playback %s source %u: %d\n",
		on ? "start" : "stop", s->source, ret);
	return ret;
}

static const struct snd_soc_component_driver aoc_component = {
	.name = "aoc-pcm",
	.open = aoc_pcm_open,
	.close = aoc_pcm_close,
	.pcm_new = aoc_pcm_new,
	.hw_params = aoc_pcm_hw_params,
	.hw_free = aoc_pcm_hw_free,
	.prepare = aoc_pcm_prepare,
	.trigger = aoc_pcm_trigger,
	.ack = aoc_pcm_ack,
	.pointer = aoc_pcm_pointer,
};

static struct snd_soc_dai_driver aoc_dais[] = {
	{
		.name = "aoc-fe",
		.playback = {
			.stream_name = "AOC Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
	},
	{
		/*
		 * The AOC's TDM_0 port.  The AOC drives the wire from its own
		 * firmware, so this DAI carries no data and needs no ops; it
		 * exists so the amplifiers have a back-end to hang off and get
		 * configured against.
		 */
		.name = "aoc-tdm0",
		.playback = {
			.stream_name = "TDM_0_RX Playback",
			.channels_min = AOC_TDM0_CHANNELS,
			.channels_max = AOC_TDM0_CHANNELS,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S24_LE,
		},
	},
};

/* The front-end feeds the TDM the AOC clocks out. */
static const struct snd_soc_dapm_route aoc_dapm_routes[] = {
	{ "TDM_0_RX Playback", NULL, "AOC Playback" },
};

/*
 * Pin the back-end to the wire, not to whatever the application opened: the AOC
 * mixes and resamples into a TDM frame that does not change.
 */
static int aoc_tdm0_fixup(struct snd_soc_pcm_runtime *rtd,
			  struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = AOC_TDM0_RATE;
	channels->min = channels->max = AOC_TDM0_CHANNELS;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, AOC_TDM0_FORMAT);
	return 0;
}

static int aoc_tdm0_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	unsigned int bclk = params_rate(params) * AOC_TDM0_SLOTS *
			    AOC_TDM0_SLOT_WIDTH;
	struct snd_soc_dai *codec_dai;
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		/*
		 * Slot 0 carries the left channel and slot 1 the right, so an
		 * amplifier's slot follows from the side it drives rather than
		 * from its place in the list.  The other slot goes to its second
		 * receive channel, which is what its firmware wants for
		 * cross-channel work.
		 */
		bool right = of_property_read_bool(codec_dai->dev->of_node,
						   "cirrus,right-channel-amp");
		unsigned int rx_slot[2] = { right, !right };

		/*
		 * The AOC owns the clocks, so the amplifiers recover theirs from
		 * the bit clock.  The component call points the PLL at it; the
		 * DAI call tells the clock monitor what to expect.
		 */
		ret = snd_soc_component_set_sysclk(codec_dai->component,
						   CS35L41_CLKID_SCLK, 0, bclk,
						   SND_SOC_CLOCK_IN);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_sysclk(codec_dai, CS35L41_CLKID_SCLK, bclk,
					     SND_SOC_CLOCK_IN);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_channel_map(codec_dai, 0, NULL,
						  ARRAY_SIZE(rx_slot), rx_slot);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct snd_soc_ops aoc_tdm0_ops = {
	.hw_params = aoc_tdm0_hw_params,
};

static struct snd_soc_dai_link_component aoc_fe_cpu = { .dai_name = "aoc-fe" };
static struct snd_soc_dai_link_component aoc_be_cpu = { .dai_name = "aoc-tdm0" };
static struct snd_soc_dai_link_component aoc_platform;

static struct snd_soc_dai_link aoc_dai_links[] = {
	{
		.name = "aoc-playback0",
		.stream_name = "aoc-playback0",
		.cpus = &aoc_fe_cpu,
		.num_cpus = 1,
		.codecs = &snd_soc_dummy_dlc,
		.num_codecs = 1,
		.platforms = &aoc_platform,
		.num_platforms = 1,
		.dynamic = 1,
		.playback_only = 1,
		/* control commands round-trip to the AOC, so the trigger must sleep */
		.nonatomic = true,
	},
	{
		.name = "aoc-tdm0",
		.stream_name = "TDM_0_RX Playback",
		.cpus = &aoc_be_cpu,
		.num_cpus = 1,
		/* .codecs comes from the device tree: the amplifiers */
		.no_pcm = 1,
		.playback_only = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.be_hw_params_fixup = aoc_tdm0_fixup,
		.ops = &aoc_tdm0_ops,
		/* the amplifiers are on SPI, so configuring them sleeps */
		.nonatomic = true,
	},
};

static int aoc_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *aoc_pdev;
	struct aoc_audio *aud;
	struct device_node *np;
	int ret;

	aud = devm_kzalloc(dev, sizeof(*aud), GFP_KERNEL);
	if (!aud)
		return -ENOMEM;

	/* The AOC that owns the audio services; defer until it is up. */
	np = of_parse_phandle(dev->of_node, "aoc", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no aoc phandle\n");
	aoc_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!aoc_pdev)
		return -EPROBE_DEFER;
	aud->aoc_dev = &aoc_pdev->dev;
	if (!aud->aoc_dev->driver) {
		put_device(aud->aoc_dev);
		return -EPROBE_DEFER;
	}
	mutex_init(&aud->cmd_lock);
	init_completion(&aud->cmd_done);
	aud->ctrl = aoc_service_find(aud->aoc_dev, AOC_OUTPUT_CTRL_SERVICE);
	if (aud->ctrl)
		aoc_service_set_handler(aud->ctrl, aoc_audio_ctrl_isr, aud);
	else
		dev_warn(dev, "no %s service; playback trigger will fail\n",
			 AOC_OUTPUT_CTRL_SERVICE);

	ret = devm_snd_soc_register_component(dev, &aoc_component, aoc_dais,
					      ARRAY_SIZE(aoc_dais));
	if (ret) {
		dev_err_probe(dev, ret, "cannot register component\n");
		goto err_put;
	}

	/* Resolve the CPU DAIs and platform to our own component (by DT node). */
	aoc_fe_cpu.of_node = dev->of_node;
	aoc_be_cpu.of_node = dev->of_node;
	aoc_platform.of_node = dev->of_node;

	/* The amplifiers listening to the TDM. */
	np = of_get_child_by_name(dev->of_node, "speakers");
	if (!np) {
		ret = dev_err_probe(dev, -EINVAL, "no speakers node\n");
		goto err_put;
	}
	ret = snd_soc_of_get_dai_link_codecs(dev, np, &aoc_dai_links[1]);
	of_node_put(np);
	if (ret) {
		dev_err_probe(dev, ret, "cannot resolve the speaker codecs\n");
		goto err_put;
	}

	aud->card.name = "aoc-audio";
	aud->card.owner = THIS_MODULE;
	aud->card.dev = dev;
	aud->card.dai_link = aoc_dai_links;
	aud->card.num_links = ARRAY_SIZE(aoc_dai_links);
	aud->card.dapm_routes = aoc_dapm_routes;
	aud->card.num_dapm_routes = ARRAY_SIZE(aoc_dapm_routes);

	ret = devm_snd_soc_register_card(dev, &aud->card);
	if (ret) {
		dev_err_probe(dev, ret, "cannot register card\n");
		goto err_put;
	}
	return 0;

err_put:
	put_device(aud->aoc_dev);
	return ret;
}

static void aoc_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct aoc_audio *aud = container_of(card, struct aoc_audio, card);

	if (aud->ctrl)
		aoc_service_set_handler(aud->ctrl, NULL, NULL);
	put_device(aud->aoc_dev);
}

static const struct of_device_id aoc_audio_of_match[] = {
	{ .compatible = "google,aoc-sound" },
	{}
};
MODULE_DEVICE_TABLE(of, aoc_audio_of_match);

static struct platform_driver aoc_audio_driver = {
	.probe = aoc_audio_probe,
	.remove = aoc_audio_remove,
	.driver = {
		.name = "aoc-audio",
		.of_match_table = aoc_audio_of_match,
	},
};
module_platform_driver(aoc_audio_driver);

MODULE_DESCRIPTION("Google AOC ASoC platform and card");
MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_LICENSE("GPL");
