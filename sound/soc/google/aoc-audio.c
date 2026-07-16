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
 * This is the transport skeleton.  Making the AOC actually route the stream to
 * a speaker needs the audio control protocol (source enable, source->sink bind)
 * and the CS35L41 amplifiers; both are separate, later steps.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/soc/google/aoc.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#define AOC_PCM_BUFFER_BYTES	(512 * 1024)
#define AOC_PLAYBACK_SERVICE	"audio_playback0"

struct aoc_audio {
	struct device *aoc_dev;			/* the AOC platform device */
	struct snd_soc_card card;
};

/* Per-open stream state. */
struct aoc_audio_stream {
	struct aoc_service *svc;
	bool playback;
	u32 base;				/* AOC byte position at start */
	snd_pcm_uframes_t pushed;		/* frames handed to the ring */
};

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
	struct aoc_audio *aud = dev_get_drvdata(comp->dev);
	struct aoc_audio_stream *s;
	struct aoc_service *svc;

	svc = aoc_service_find(aud->aoc_dev, AOC_PLAYBACK_SERVICE);
	if (!svc)
		return -ENODEV;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->svc = svc;
	s->playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
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

static const struct snd_soc_component_driver aoc_component = {
	.name = "aoc-pcm",
	.open = aoc_pcm_open,
	.close = aoc_pcm_close,
	.pcm_new = aoc_pcm_new,
	.prepare = aoc_pcm_prepare,
	.ack = aoc_pcm_ack,
	.pointer = aoc_pcm_pointer,
};

static struct snd_soc_dai_driver aoc_dai = {
	.name = "aoc-fe",
	.playback = {
		.stream_name = "AOC Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
};

static struct snd_soc_dai_link_component aoc_cpu = { .dai_name = "aoc-fe" };
static struct snd_soc_dai_link_component aoc_platform;

static struct snd_soc_dai_link aoc_dai_link = {
	.name = "aoc-playback0",
	.stream_name = "aoc-playback0",
	.cpus = &aoc_cpu,
	.num_cpus = 1,
	.codecs = &snd_soc_dummy_dlc,
	.num_codecs = 1,
	.platforms = &aoc_platform,
	.num_platforms = 1,
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
	dev_set_drvdata(dev, aud);

	ret = devm_snd_soc_register_component(dev, &aoc_component, &aoc_dai, 1);
	if (ret) {
		dev_err_probe(dev, ret, "cannot register component\n");
		goto err_put;
	}

	/* Resolve the CPU DAI and platform to our own component (by DT node). */
	aoc_cpu.of_node = dev->of_node;
	aoc_platform.of_node = dev->of_node;

	aud->card.name = "aoc-audio";
	aud->card.owner = THIS_MODULE;
	aud->card.dev = dev;
	aud->card.dai_link = &aoc_dai_link;
	aud->card.num_links = 1;

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
	struct aoc_audio *aud = platform_get_drvdata(pdev);

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
