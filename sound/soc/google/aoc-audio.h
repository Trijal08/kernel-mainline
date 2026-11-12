/* SPDX-License-Identifier: GPL-2.0 */
/*
 * State the AOC audio card shares between the ASoC side and the commands it
 * sends the coprocessor, and the interface between the two.
 *
 * Copyright 2025 Trijal Saha - <trijalsaha2012@gmail.com>
 */

#ifndef __AOC_AUDIO_H__
#define __AOC_AUDIO_H__

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/soc/google/aoc.h>
#include <linux/types.h>
#include <sound/soc.h>

/* The card's links, in the order aoc_dai_link_tmpl[] declares them. */
enum {
	AOC_LINK_PLAYBACK,
	AOC_LINK_CAPTURE,
	AOC_LINK_VOICE,
	AOC_LINK_TDM0,
	AOC_LINK_COUNT,
};

struct aoc_audio {
	struct device *aoc_dev;			/* the AOC platform device */
	struct snd_soc_card card;
	struct aoc_service *ctrl;		/* audio_output_control channel */
	struct aoc_service *ctrl_in;		/* audio_input_control channel */
	struct mutex cmd_lock;			/* serialises control commands */
	struct completion cmd_done;		/* a control reply arrived */
	int mic_hw_gain_cb;			/* mic preamp gain, centibels */
	int mic_soft_gain_db;			/* record soft gain, dB */
	bool mic_muted;				/* capture switch, off */
	bool voice_mic_on;			/* the modem's mic input is running */
	/*
	 * The links and the components they name, per card rather than per
	 * driver.  Probe fills in the device tree node and the codecs it finds
	 * there, so a table shared between instances would carry one card's
	 * resolution into the next -- including from a probe that deferred into
	 * the one that succeeds.
	 */
	struct snd_soc_dai_link links[AOC_LINK_COUNT];
	struct snd_soc_dai_link_component cpus[AOC_LINK_COUNT];
	struct snd_soc_dai_link_component platform;
};

/*
 * What the ASoC side asks of the coprocessor.  Implemented in
 * aoc-audio-cmd.c, which owns the framing in aoc-audio-ipc.h.
 */
void aoc_audio_ctrl_isr(struct aoc_service *svc, void *priv);
int aoc_audio_source(struct aoc_audio *aud, u8 source, bool on);
int aoc_audio_ep_setup(struct aoc_audio *aud, u8 source, unsigned int channels,
		       unsigned int rate, unsigned int width, u32 buffer_bytes,
		       u32 period_bytes);
int aoc_audio_mic_setup(struct aoc_audio *aud, u8 channel,
			unsigned int channels, unsigned int rate,
			unsigned int width, u32 buffer_bytes, u32 period_bytes);
int aoc_audio_mic(struct aoc_audio *aud, bool on);
int aoc_audio_mic_gain(struct aoc_audio *aud);
int aoc_audio_bind(struct aoc_audio *aud, u8 src, u8 dst, bool on);
int aoc_audio_set_param(struct aoc_audio *aud, u8 block, u8 component, u32 key,
			u32 val);
int aoc_audio_out_volume(struct aoc_audio *aud, u8 src, u8 dst, u32 vol);
int aoc_audio_dsp_mode(struct aoc_audio *aud, u8 mode);
int aoc_audio_modem(struct aoc_audio *aud, bool on);
int aoc_voice_mic_set(struct aoc_audio *aud, bool on);
int aoc_voice_replay_pre(struct aoc_audio *aud);
int aoc_voice_replay_post_bind(struct aoc_audio *aud);
void aoc_voice_replay_teardown(struct aoc_audio *aud);

#endif /* __AOC_AUDIO_H__ */
