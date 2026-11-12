/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The AOC's audio control interface: the commands the coprocessor's firmware
 * accepts and the structures they are framed in.  This is firmware ABI, not
 * this driver's own interface, and is kept apart from the driver for that
 * reason.  The names follow the vendor's aoc-interface headers so the two can
 * be compared.
 *
 * Copyright 2025 Trijal Saha - <trijalsaha2012@gmail.com>
 */

#ifndef __AOC_AUDIO_IPC_H__
#define __AOC_AUDIO_IPC_H__

#include <linux/types.h>

/*
 * AOC control-command framing (from the vendor aoc-interface): a CONTAINER_HDR
 * (type/counter/length) followed by a command id and reply field.
 */
#define AOC_CMD_TYPE_CMD		0
#define AOC_CMD_AUDIO_OUTPUT_SOURCE_ID	201
/*
 * Binding the telephony endpoint only routes it; nothing runs it.  Starting the
 * modem's audio flow is a separate bare-header command, which the vendor stack
 * issues from its telephony PCM's prepare ("Binding modem to start audio
 * flow").  Without it a call connects, the wire is clocked, the amplifiers are
 * powered, and both directions are silent.
 */
#define AOC_CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_START_ID	256
#define AOC_CMD_AUDIO_OUTPUT_TELEPHONY_MODEM_STOP_ID	257
/*
 * The firmware has two further telephony starts, VOIP (258/259) and RTP
 * (289/290), for bearers it decodes itself.  MODEM is the one that works here,
 * including for VoLTE: the AOC logs "PCIe Modem Configuration: Unknown codec
 * type (0xffffffff)" alongside it and carries the call regardless, so the
 * complaint is not what it looks like.  Neither alternative is used.
 */
/*
 * The AOC's audio DSP runs one processing mode at a time, and a call needs
 * TELEPHONY: in the boot default the modem mixer comes up with its
 * post-processing bypassed and the DSP never pulls the downlink, which reads
 * as "AMixMODEM PostProcessing Config: BYPASS" plus a modem underrun.  Set
 * from the vendor driver's "Audio DSP Mode" control (aoc_set_audio_dsp_mode).
 */
#define AOC_CMD_AUDIO_OUTPUT_DSP_MODE_SET_ID	277
/*
 * Mode 2 is the telephony mode the comment above describes.  It is not set
 * here: calls carry correctly with the DSP left in AMBIENT, so the AOC is
 * evidently switching itself when the modem flow starts.
 */
#define AOC_DSP_MODE_AMBIENT		0

/*
 * The call's uplink, and the downlink's precondition.
 *
 * The mic feeds the modem directly rather than a ring the AP reads, so it has
 * its own command, which carries the echo-cancel reference alongside the mic:
 * the AOC can only subtract what is being played if it is told what that is.
 *
 * It is not an optional extra to the downlink.  The DSP will not bind a sink
 * with no mic source live - the bind simply goes unanswered - and the vendor
 * stack goes as far as opening a default mic when none is up.  So the mic
 * starts before the sink is bound, and stops after it is unbound.
 */
#define AOC_CMD_AUDIO_INPUT_MODEM_INPUT_START2_ID	324
#define AOC_CMD_AUDIO_INPUT_MODEM_INPUT_STOP_ID		211
#define AOC_MODEM_MIC_INPUT_INDEX			0	/* enum ModemInputIndex */
/*
 * The telephony mic source.  The vendor stack starts the modem's input with
 * hw_id_to_phone_mic_source(DEFAULT_TELEPHONY_MIC), and DEFAULT_TELEPHONY_MIC
 * is PORT_INCALL_TX (hw id 16), which maps to MODEM_INCALL_INPUT_INDEX -- not
 * to MODEM_MIC_INPUT_INDEX.  Captured from this phone running Android, at the
 * point the telephony sink is bound:
 *
 *   aoc_telephony_sink_open: use default mic source: 3 - 16
 *   Modem mic input source: 3, aec ref source: 0
 *   open telephony sink id: 0 - 6
 *   bind: src:8 - sink:0!
 *
 * The AEC reference stays MODEM_MIC_INPUT_INDEX (the speaker-playback case in
 * the vendor's ft_aec_ref_source switch), so the two are NOT the same value:
 * the in-call input index is 3, which nothing here needs to send.
 */

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

/*
 * Per-(source,sink) playback volume. The AOC boots each sink's mixer gain at a
 * low default, so without this the speaker plays far quieter than Android --
 * the stock HAL sets this on every stream (dhd aoc_audio_volume_set). The value
 * is a 0..AOC_SPEAKER_VOLUME_MAX scale where MAX is unity (no attenuation); the
 * sink's block id is the sink index + AOC_AUDIO_SINK_BLOCK_ID_BASE, the key is
 * the source (entry point), component 0.
 */
#define AOC_CMD_AUDIO_OUTPUT_SET_PARAM_ID	209
#define AOC_AUDIO_SINK_BLOCK_ID_BASE		16
/*
 * The telephony sink's processing block (sink 3).  Android configures it twice
 * around a call, captured from this phone with the AoC command log
 * unratelimited (CMD_AUDIO_OUTPUT_SET_PARAMETER, block/component/key/val):
 *
 *   19, 0,  16, 0    before the telephony path is built
 *   19, 30, 16, 0    immediately after TELEPHONY_MODEM_START
 *
 * Nothing else in the capture touches block 19, and we were never sending it.
 */
#define AOC_TELEPHONY_SINK_BLOCK			19
#define AOC_TELEPHONY_BLOCK_KEY			16
/* The "before" value in the capture above is 0; only POST is ever sent. */
#define AOC_TELEPHONY_COMPONENT_POST		30
#define AOC_SPEAKER_VOLUME_MAX			1000	/* stock chip->volume */

struct cmd_audio_output_set_param {
	struct aoc_cmd_hdr hdr;
	u8 block;
	u8 component;
	__le32 key;
	__le32 val;
} __packed;

/*
 * Capture is the mirror of playback: the AOC records from the mics into an
 * audio_captureN ring (AOC_UP: the AOC writes, the AP reads), configured and
 * started over its own control channel, audio_input_control.
 */
#define AOC_CAPTURE_SERVICE	"audio_capture0"
#define AOC_INPUT_CTRL_SERVICE	"audio_input_control"
#define AOC_CAPTURE_SOURCE	0	/* the entry point audio_capture0 feeds */

#define AOC_CMD_AUDIO_INPUT_START_ID		208
#define AOC_CMD_AUDIO_INPUT_STOP_ID		209
#define AOC_CMD_AUDIO_INPUT_START_DATA_ID	293
#define AOC_CMD_AUDIO_INPUT_MIC_PARAMS_ID	224
#define AOC_CMD_AUDIO_INPUT_MIC_PARAMS2_ID	359
#define AOC_MIC_SRC_AP				1	/* APInputProcessorInputIndex */
#define AOC_MIC_PROCESS_RAW			0	/* APMicProcessIndex */
#define AOC_MIC_PDM_MAX				4

/*
 * The mic boots muted: both gain stages sit at zero, so a recording is barely
 * above the noise floor.  These are the values the stock record paths use.
 */
#define AOC_CMD_AUDIO_INPUT_SET_MIC_HP_GAIN_ID	254	/* high-power HW gain */
#define AOC_CMD_AUDIO_INPUT_SET_PARAM_ID	233	/* generic input parameter */
#define AOC_MIC_HW_GAIN_CB			200	/* mic preamp, centibels (20 dB) */
#define AOC_MIC_REC_SOFT_GAIN_DB		22	/* record soft gain, dB */
#define AOC_PARAM_BLOCK_MIC			136
#define AOC_PARAM_COMP_REC_GAIN			30
#define AOC_PARAM_KEY_DB			16

/* Turn the AP mic source on or off (the "on" is the START command). */
struct cmd_audio_input_start {
	struct aoc_cmd_hdr hdr;
	u8 mic_input_source;
} __packed;

/* The mics' format and lane mapping. */
struct cmd_audio_input_mic_params {
	struct aoc_cmd_hdr hdr;
	struct aoc_chan_metadata format;
	u8 pdm_mask;
	u8 period_ms;
	u8 num_periods;
	u8 interleaving[AOC_MIC_PDM_MAX];
	u8 sample_rate;
	u8 mic_process_index;
} __packed;

/* The capture ring geometry (bytes), like the playback endpoint's setup2. */
struct cmd_audio_input_mic_params2 {
	struct aoc_cmd_hdr hdr;
	u8 channel;
	__le32 buffer_size;
	__le32 period_size;
} __packed;

/* The mic preamp gain, in centibels. */
struct cmd_audio_input_hw_gain {
	struct aoc_cmd_hdr hdr;
	__le32 gain_cb;
} __packed;

/* A generic input-processor parameter (here, the record soft gain). */
struct cmd_audio_input_set_param {
	struct aoc_cmd_hdr hdr;
	u8 block;
	u8 component;
	__le32 key;
	__le32 val;
} __packed;

#endif /* __AOC_AUDIO_IPC_H__ */
