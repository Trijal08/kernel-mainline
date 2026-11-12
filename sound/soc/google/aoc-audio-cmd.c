// SPDX-License-Identifier: GPL-2.0
/*
 * The commands this driver sends the AOC's audio firmware, and the reply
 * handling they share.  Everything that speaks the interface in
 * aoc-audio-ipc.h lives here; the ASoC side calls it and does not frame a
 * command of its own.
 *
 * Copyright 2025 Trijal Saha - <trijalsaha2012@gmail.com>
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/soc/google/aoc.h>
#include <linux/string.h>
#include <sound/soc.h>

#include "aoc-audio.h"
#include "aoc-audio-ipc.h"

#define AOC_CMD_TIMEOUT_MS	200
#define AOC_CMD_REPLY_TRIES	50	/* ~10ms of 200us polls for a lagging reply */
/* Switching the DSP's mode reloads blocks, well past that reply window. */
#define AOC_CMD_DSP_MODE_TIMEOUT_MS	700

/*
 * Replies are read only to drain the channel -- nothing here inspects one --
 * so this is just a bound on what a reply can be, not a described layout.
 */
#define AOC_CMD_RSP_MAX		64

/* The AOC replied on the control channel. */
void aoc_audio_ctrl_isr(struct aoc_service *svc, void *priv)
{
	struct aoc_audio *aud = priv;

	complete(&aud->cmd_done);
}

/*
 * Send a control command and wait for its reply.  Runs in process context (the
 * dai link is nonatomic), and is woken by the control channel's doorbell rather
 * than by polling.
 */
static int aoc_audio_cmd_to(struct aoc_audio *aud, struct aoc_service *ctrl,
			    const void *req, size_t req_len,
			    void *rsp, size_t rsp_len, unsigned int timeout_ms)
{
	struct device *dev = aud->card.dev;
	char drain[AOC_CMD_RSP_MAX];
	unsigned int i;
	int ret;

	if (!ctrl) {
		dev_err(dev, "no audio control channel\n");
		return -ENODEV;
	}

	mutex_lock(&aud->cmd_lock);

	/* Discard any stale replies left in the channel. */
	while (aoc_service_can_read(ctrl))
		aoc_service_read(ctrl, drain, sizeof(drain));

	reinit_completion(&aud->cmd_done);
	ret = aoc_service_write(ctrl, req, req_len);
	if (ret < 0) {
		dev_err(dev, "control write failed: %d\n", ret);
		goto out;
	}

	if (!wait_for_completion_timeout(&aud->cmd_done,
					 msecs_to_jiffies(timeout_ms))) {
		dev_err(dev, "control command timed out (no reply)\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	/*
	 * The reply lags the doorbell, so poll for it briefly rather than
	 * reading once.  Its contents are not used -- draining the channel is
	 * what matters -- so a command that never replies is not an error; the
	 * next command clears any late reply above.
	 */
	for (i = 0; i < AOC_CMD_REPLY_TRIES; i++) {
		if (aoc_service_can_read(ctrl)) {
			aoc_service_read(ctrl, rsp, rsp_len);
			break;
		}
		usleep_range(200, 400);
	}
	ret = 0;
out:
	mutex_unlock(&aud->cmd_lock);
	return ret < 0 ? ret : 0;
}

static int aoc_audio_cmd_on(struct aoc_audio *aud, struct aoc_service *ctrl,
			    const void *req, size_t req_len,
			    void *rsp, size_t rsp_len)
{
	return aoc_audio_cmd_to(aud, ctrl, req, req_len, rsp, rsp_len,
				AOC_CMD_TIMEOUT_MS);
}

/* The output control channel, for the playback commands. */
static int aoc_audio_cmd(struct aoc_audio *aud, const void *req, size_t req_len,
			 void *rsp, size_t rsp_len)
{
	return aoc_audio_cmd_on(aud, aud->ctrl, req, req_len, rsp, rsp_len);
}

/*
 * The AP->AOC telephony prologue, replayed byte for byte from this phone
 * running Android with the vendor driver's command log unratelimited
 * (google-modules/aoc/alsa/aoc_alsa_hw.c, pr_notice_ratelimited -> pr_notice
 * plus a print_hex_dump of the command).  The capture is gap-free: its cntr
 * field runs 176..226 with no holes.  Payloads are exactly as sent, so that a
 * stream identical to Android's can be tested and then bisected; anything we
 * can explain gets its own helper, the rest stays opaque on purpose.
 *
 *   cntr 176  SET_PARAMETER   block 19 (telephony sink), component 0, key 16
 *   cntr 177  DSP_MODE_SET    mode 2 (TELEPHONY)
 *   cntr 178-182  sidetone EQ, five stages (1.0f/1.0f coefficients, stage last)
 *   cntr 183-186  sidetone cfg get/set pairs (volume -96 then -90 dB)
 *   cntr 187  OUTPUT_SOURCE   source 11 (Sidetone) on
 *   cntr 188  OUTPUT_BIND     source 11 -> sink 0
 * then mic start (cntr 189) and the sink bind (cntr 190) from their helpers,
 * then:
 *   cntr 191  [input] 0x00f9  mic/EQ parameter
 *   cntr 192  [input] 0x00fe  mic high-power HW gain, 0
 * then MODEM_START (cntr 193) and the block-19 component-30 write.
 */
struct aoc_voice_raw_cmd {
	bool	input;		/* true: the input control channel */
	u16	id;
	u8	len;		/* payload bytes after the 8-byte header */
	u8	pay[25];
};

static const struct aoc_voice_raw_cmd aoc_voice_pre[] = {
	{ false, 0x00d1, 10, { 0x13, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ false, 0x0115,  1, { 0x02 } },
	/*
	 * Android's sidetone chain (0x00f9 x5, 0x00f8/0x00f7 pairs) and its
	 * source 11 (Sidetone) bind are deliberately NOT replayed.  Sidetone
	 * is fed from a mic path we do not set up, so binding it to the
	 * speaker sink only starves it: the AOC logs "[AHWSinkSPKR] SDT
	 * underrun" once a second, and PCM_VALID stops advancing -- the CP
	 * delivers a short burst of voice frames and then nothing, where
	 * before it ticked once per 20 ms for the whole call.  It is also
	 * route-specific on Android (earpiece only), so it is not part of the
	 * modem downlink.
	 */
};

static const struct aoc_voice_raw_cmd aoc_voice_post_bind[] = {
	{ true,  0x00f9,  4, { 0x00, 0x00, 0x00, 0x00 } },
	{ true,  0x00fe,  4, { 0x00, 0x00, 0x00, 0x00 } },
};

/* Undo of aoc_voice_pre's lasting effects, in reverse order. */
static const struct aoc_voice_raw_cmd aoc_voice_teardown[] = {
	{ false, 0x0115,  1, { 0x00 } },		/* DSP mode back to AMBIENT */
};

/* Replay a captured command table verbatim on its own control channel. */
static int aoc_voice_replay(struct aoc_audio *aud,
			    const struct aoc_voice_raw_cmd *tbl, size_t n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		struct {
			struct aoc_cmd_hdr hdr;
			u8 pay[25];
		} __packed cmd = { };
		char rsp[AOC_CMD_RSP_MAX];
		size_t len = sizeof(cmd.hdr) + tbl[i].len;

		cmd.hdr.type = AOC_CMD_TYPE_CMD;
		cmd.hdr.len = cpu_to_le16(len);
		cmd.hdr.id = cpu_to_le16(tbl[i].id);
		memcpy(cmd.pay, tbl[i].pay, tbl[i].len);

		ret = aoc_audio_cmd_on(aud,
				       tbl[i].input ? aud->ctrl_in : aud->ctrl,
				       &cmd, len, rsp, sizeof(rsp));
		if (ret) {
			dev_err(aud->card.dev,
				"voice replay %#06x (%zu bytes) failed: %d\n",
				tbl[i].id, len, ret);
			return ret;
		}
	}
	return 0;
}

/* Turn a playback source (entry point) on or off. */
int aoc_audio_source(struct aoc_audio *aud, u8 source, bool on)
{
	struct cmd_audio_output_source cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_SOURCE_ID);
	cmd.source = source;
	cmd.on = on ? 1 : 0;
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/* Configure the playback endpoint's format and ring geometry (bytes). */
int aoc_audio_ep_setup(struct aoc_audio *aud, u8 source,
		       unsigned int channels, unsigned int rate,
		       unsigned int width, u32 buffer_bytes,
		       u32 period_bytes)
{
	struct cmd_audio_output_ep_setup cmd = { };
	struct cmd_audio_output_ep_setup2 cmd2 = { };
	char rsp[AOC_CMD_RSP_MAX];
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

struct cmd_audio_input_modem_input_start2 {
	struct aoc_cmd_hdr hdr;
	u8 mic_input_source;
	u8 ref_input_source;
} __packed;

/* Start or stop the mic feeding the modem, with itself as the AEC reference. */
int aoc_voice_mic_set(struct aoc_audio *aud, bool on)
{
	struct cmd_audio_input_modem_input_start2 start = { };
	struct aoc_cmd_hdr stop = { };
	char rsp[AOC_CMD_RSP_MAX];

	if (!aud->ctrl_in)
		return -ENODEV;

	if (!on) {
		stop.type = AOC_CMD_TYPE_CMD;
		stop.len = cpu_to_le16(sizeof(stop));
		stop.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_MODEM_INPUT_STOP_ID);
		return aoc_audio_cmd_on(aud, aud->ctrl_in, &stop, sizeof(stop),
					rsp, sizeof(rsp));
	}

	start.hdr.type = AOC_CMD_TYPE_CMD;
	start.hdr.len = cpu_to_le16(sizeof(start));
	start.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_MODEM_INPUT_START2_ID);
	/*
	 * Back to MODEM_MIC_INPUT_INDEX.  The vendor opens the telephony sink
	 * with MODEM_INCALL_INPUT_INDEX (3, PORT_INCALL_TX) but switches to 0
	 * moments later and runs the rest of the call there; we were sending 3
	 * and leaving it, and PCM_VALID stopped advancing -- the CP delivered
	 * no voice frames at all, where before it ticked once per 20 ms.
	 */
	start.mic_input_source = AOC_MODEM_MIC_INPUT_INDEX;
	start.ref_input_source = AOC_MODEM_MIC_INPUT_INDEX;
	return aoc_audio_cmd_on(aud, aud->ctrl_in, &start, sizeof(start), rsp,
				sizeof(rsp));
}

struct cmd_audio_output_dsp_mode {
	struct aoc_cmd_hdr hdr;
	u8 mode;
} __packed;

/* Put the AOC's audio DSP into one of the AudioDSPMode processing modes. */
int aoc_audio_dsp_mode(struct aoc_audio *aud, u8 mode)
{
	struct cmd_audio_output_dsp_mode cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_DSP_MODE_SET_ID);
	cmd.mode = mode;
	return aoc_audio_cmd_to(aud, aud->ctrl, &cmd, sizeof(cmd), rsp,
				sizeof(rsp), AOC_CMD_DSP_MODE_TIMEOUT_MS);
}

/*
 * Open or close the modem's audio flow.  A bare header: the command carries no
 * payload, the AOC knows which endpoint it means.
 */
int aoc_audio_modem(struct aoc_audio *aud, bool on)
{
	struct aoc_cmd_hdr cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.type = AOC_CMD_TYPE_CMD;
	cmd.len = cpu_to_le16(sizeof(cmd));
	cmd.id = cpu_to_le16(on ? AOC_CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_START_ID
				: AOC_CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_STOP_ID);
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/* Bind (or unbind) a source to a sink. */
int aoc_audio_bind(struct aoc_audio *aud, u8 src, u8 dst, bool on)
{
	struct cmd_audio_output_bind cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_BIND_ID);
	cmd.src = src;
	cmd.dst = dst;
	cmd.bind = on ? 1 : 0;
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/* Set one block/component parameter on the AOC's output graph. */
int aoc_audio_set_param(struct aoc_audio *aud, u8 block, u8 component,
			u32 key, u32 val)
{
	struct cmd_audio_output_set_param cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_SET_PARAM_ID);
	cmd.block = block;
	cmd.component = component;
	cmd.key = cpu_to_le32(key);
	cmd.val = cpu_to_le32(val);
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/* Set the mixer volume of a source into a sink (0..AOC_SPEAKER_VOLUME_MAX). */
int aoc_audio_out_volume(struct aoc_audio *aud, u8 src, u8 dst, u32 vol)
{
	struct cmd_audio_output_set_param cmd = { };
	char rsp[AOC_CMD_RSP_MAX];

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_OUTPUT_SET_PARAM_ID);
	cmd.block = dst + AOC_AUDIO_SINK_BLOCK_ID_BASE;
	cmd.component = 0;
	cmd.key = cpu_to_le32(src);
	cmd.val = cpu_to_le32(vol);
	return aoc_audio_cmd(aud, &cmd, sizeof(cmd), rsp, sizeof(rsp));
}

/*
 * Lift the mic off its muted defaults: the analog preamp (HW gain, centibels)
 * and the record soft gain (a digital dB boost the stock path always applies).
 */
int aoc_audio_mic_gain(struct aoc_audio *aud)
{
	struct cmd_audio_input_hw_gain hw = { };
	struct cmd_audio_input_set_param soft = { };
	char rsp[AOC_CMD_RSP_MAX];
	int ret;

	if (!aud->ctrl_in)
		return 0;

	hw.hdr.type = AOC_CMD_TYPE_CMD;
	hw.hdr.len = cpu_to_le16(sizeof(hw));
	hw.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_SET_MIC_HP_GAIN_ID);
	hw.gain_cb = cpu_to_le32(aud->mic_muted ? 0 : aud->mic_hw_gain_cb);
	ret = aoc_audio_cmd_on(aud, aud->ctrl_in, &hw, sizeof(hw), rsp,
			       sizeof(rsp));
	if (ret)
		return ret;

	soft.hdr.type = AOC_CMD_TYPE_CMD;
	soft.hdr.len = cpu_to_le16(sizeof(soft));
	soft.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_SET_PARAM_ID);
	soft.block = AOC_PARAM_BLOCK_MIC;
	soft.component = AOC_PARAM_COMP_REC_GAIN;
	soft.key = cpu_to_le32(AOC_PARAM_KEY_DB);
	soft.val = cpu_to_le32(aud->mic_muted ? 0 : aud->mic_soft_gain_db);
	return aoc_audio_cmd_on(aud, aud->ctrl_in, &soft, sizeof(soft), rsp,
				sizeof(rsp));
}

/* Configure the mics' format and the capture ring geometry (bytes). */
int aoc_audio_mic_setup(struct aoc_audio *aud, u8 channel,
			unsigned int channels, unsigned int rate,
			unsigned int width, u32 buffer_bytes,
			u32 period_bytes)
{
	struct cmd_audio_input_mic_params cmd = { };
	struct cmd_audio_input_mic_params2 cmd2 = { };
	char rsp[AOC_CMD_RSP_MAX];
	unsigned int i;
	int ret;

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_MIC_PARAMS_ID);
	cmd.format.chan = channels;
	cmd.format.bits = (width == 32) ? AOC_WIDTH_32_BIT : AOC_WIDTH_16_BIT;
	cmd.format.sr = (rate == 48000) ? AOC_SR_48KHZ : AOC_SR_44K1HZ;
	cmd.format.format = AOC_FRMT_FIXED_POINT;
	/* One built-in mic per requested channel, on consecutive PDM lanes. */
	for (i = 0; i < AOC_MIC_PDM_MAX; i++)
		cmd.interleaving[i] = (i < channels) ? i : 0xff;
	cmd.pdm_mask = (1u << channels) - 1;
	cmd.period_ms = 10;
	cmd.num_periods = 4;
	cmd.sample_rate = cmd.format.sr;
	cmd.mic_process_index = AOC_MIC_PROCESS_RAW;
	ret = aoc_audio_cmd_on(aud, aud->ctrl_in, &cmd, sizeof(cmd), rsp,
			       sizeof(rsp));
	if (ret)
		return ret;

	cmd2.hdr.type = AOC_CMD_TYPE_CMD;
	cmd2.hdr.len = cpu_to_le16(sizeof(cmd2));
	cmd2.hdr.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_MIC_PARAMS2_ID);
	cmd2.channel = channel;
	cmd2.buffer_size = cpu_to_le32(buffer_bytes);
	cmd2.period_size = cpu_to_le32(period_bytes);
	ret = aoc_audio_cmd_on(aud, aud->ctrl_in, &cmd2, sizeof(cmd2), rsp,
			       sizeof(rsp));
	if (ret)
		return ret;

	return aoc_audio_mic_gain(aud);
}

/* Start or stop the AP mic capture. */
int aoc_audio_mic(struct aoc_audio *aud, bool on)
{
	struct cmd_audio_input_start cmd = { };
	struct aoc_cmd_hdr data = { };
	char rsp[AOC_CMD_RSP_MAX];
	int ret;

	cmd.hdr.type = AOC_CMD_TYPE_CMD;
	cmd.hdr.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(on ? AOC_CMD_AUDIO_INPUT_START_ID :
				      AOC_CMD_AUDIO_INPUT_STOP_ID);
	cmd.mic_input_source = AOC_MIC_SRC_AP;
	/* the stop command is just the header */
	ret = aoc_audio_cmd_on(aud, aud->ctrl_in, &cmd,
			       on ? sizeof(cmd) : sizeof(cmd.hdr),
			       rsp, sizeof(rsp));
	if (ret || !on)
		return ret;

	/*
	 * Starting the mic processor is not enough: this second command begins
	 * the flow of recorded data into the capture ring.  Stopping the input
	 * above ends it, so there is no matching stop here.
	 */
	data.type = AOC_CMD_TYPE_CMD;
	data.len = cpu_to_le16(sizeof(data));
	data.id = cpu_to_le16(AOC_CMD_AUDIO_INPUT_START_DATA_ID);
	return aoc_audio_cmd_on(aud, aud->ctrl_in, &data, sizeof(data),
				rsp, sizeof(rsp));
}

/*
 * The three points in a call's life the raw tables above describe.  The
 * tables stay here so that their lengths do too.
 */
int aoc_voice_replay_pre(struct aoc_audio *aud)
{
	return aoc_voice_replay(aud, aoc_voice_pre, ARRAY_SIZE(aoc_voice_pre));
}

int aoc_voice_replay_post_bind(struct aoc_audio *aud)
{
	return aoc_voice_replay(aud, aoc_voice_post_bind,
				ARRAY_SIZE(aoc_voice_post_bind));
}

void aoc_voice_replay_teardown(struct aoc_audio *aud)
{
	aoc_voice_replay(aud, aoc_voice_teardown,
			 ARRAY_SIZE(aoc_voice_teardown));
}
