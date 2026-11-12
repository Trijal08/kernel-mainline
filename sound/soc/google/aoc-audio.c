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
 * A call does not stream at all -- its audio stays inside the coprocessor -- so
 * its route is a codec-to-codec link straight to the amplifiers, which DAPM
 * brings up from a mixer switch.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/workqueue.h>
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
#include <sound/tlv.h>

#include "aoc-audio.h"
#include "aoc-audio-ipc.h"

/* The ring the AOC drains is 16K; keep the staging buffer a few of those. */
/*
 * The staging buffer the application writes into, from which .ack and the
 * refill tick copy on to the AOC's ring.  Six rings' worth: enough that a
 * writer can get far enough ahead to ride out a scheduling delay, since the
 * ring itself only holds about 42ms.
 */
#define AOC_PCM_BUFFER_BYTES	(16384 * 6)
#define AOC_PCM_TICK_NS		(10 * NSEC_PER_MSEC)
/*
 * Entry point 2.  Not the first one: that is the AOC's ultra-low-latency
 * endpoint, which expects its buffer filled ahead of the reader rather than
 * streamed to, so writing to it leaves the speaker mixer starved and the TDM
 * clocking out an empty buffer.
 */
#define AOC_PLAYBACK_SERVICE	"audio_playback1"
#define AOC_PLAYBACK_SOURCE	1
/*
 * The source that carries a call.
 *
 * A call's audio never crosses the AP: the modem hands its downlink to the AOC
 * and takes its uplink back from it, both inside the coprocessor.  The AP's
 * part is to bind that downlink to the speaker sink and to start the modem's
 * flow, which is what the telephony link does.
 *
 * For a stream the entry point and the source are the same number: the ring
 * the AP writes is what the sink is bound to.  Telephony has neither.  Its
 * downlink is a source of its own, bound to the sink by this fixed number,
 * with no entry point and no ring behind it -- bind an endpoint instead and
 * every command still succeeds while the speaker is wired to an empty ring,
 * which is silence.
 */
#define AOC_SRC_TELEPHONY_DOWNLINK	8
#define AOC_OUTPUT_CTRL_SERVICE	"audio_output_control"

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
#define AOC_TDM0_FMTBIT		SNDRV_PCM_FMTBIT_S24_LE

/* Per-open stream state. */
struct aoc_audio_stream {
	struct aoc_service *svc;
	struct snd_pcm_substream *substream;
	struct hrtimer tick;			/* the AOC does not interrupt us */
	u64 tick_ns;				/* half a period; see hw_params */
	struct work_struct period_work;		/* the tick cannot enter the core */
	bool playback;
	u32 ring;				/* AOC ring size in bytes */
	struct device *dev;			/* for the debug lines below */
	u8 source;				/* AOC entry point = PCM device */
	u32 base;				/* AOC byte position at start */
	snd_pcm_uframes_t pushed;		/* frames handed to the ring */
	spinlock_t ring_lock;			/* serialises ring fill: .ack vs tick */
	/*
	 * Cleared before the tick is cancelled on STOP.  hrtimer_try_to_cancel()
	 * cannot wait -- .trigger runs atomically -- so a tick already past its
	 * check still runs after the stream has stopped, and would hand the ring
	 * another period.  This keeps the refill out of a stopped stream and ends
	 * the timer; the position report needs no such flag, because the core
	 * drops it under the stream lock if the stream is no longer running.
	 */
	bool running;
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

/*
 * Both mic gains are exposed as mixer controls, the way the stock stack drives
 * them: the values are latched here and (re)sent to a running mic, so they can
 * be swept live while recording.
 */
#define AOC_MIC_HW_GAIN_CB_MAX		400	/* generous headroom over stock's 130 */
#define AOC_MIC_SOFT_GAIN_DB_MAX	40

/*
 * Both gains are real decibels, so describe them: userspace can then show a
 * scale rather than a raw number, and a mixer can map a percentage onto the
 * control sensibly.  The record gain counts whole dB and its zero is silence
 * rather than 0 dB -- say so, so a volume taken to the bottom is understood
 * as a mute.  The preamp counts centibels and its zero is unity.
 */
static const DECLARE_TLV_DB_SCALE(aoc_mic_capture_tlv, 0, 100, 1);
static const DECLARE_TLV_DB_SCALE(aoc_mic_boost_tlv, 0, 10, 0);

static int aoc_mic_gain_info(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_info *ui)
{
	ui->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	ui->count = 1;
	ui->value.integer.min = 0;
	ui->value.integer.max = (kc->private_value == AOC_PARAM_KEY_DB) ?
				AOC_MIC_SOFT_GAIN_DB_MAX : AOC_MIC_HW_GAIN_CB_MAX;
	return 0;
}

static int aoc_mic_gain_get(struct snd_kcontrol *kc,
			    struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	struct aoc_audio *aud = container_of(card, struct aoc_audio, card);

	uc->value.integer.value[0] = (kc->private_value == AOC_PARAM_KEY_DB) ?
				     aud->mic_soft_gain_db : aud->mic_hw_gain_cb;
	return 0;
}

static int aoc_mic_gain_put(struct snd_kcontrol *kc,
			    struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	struct aoc_audio *aud = container_of(card, struct aoc_audio, card);
	long val = uc->value.integer.value[0];
	int *slot, max;

	if (kc->private_value == AOC_PARAM_KEY_DB) {
		slot = &aud->mic_soft_gain_db;
		max = AOC_MIC_SOFT_GAIN_DB_MAX;
	} else {
		slot = &aud->mic_hw_gain_cb;
		max = AOC_MIC_HW_GAIN_CB_MAX;
	}
	val = clamp(val, 0L, (long)max);
	if (val == *slot)
		return 0;

	*slot = val;
	aoc_audio_mic_gain(aud);	/* a no-op until the AOC is up */
	return 1;
}

/*
 * Capture switch.  The AOC has no mute of its own and the card has no other
 * boolean control on the capture side, so there is nothing for UCM's
 * CaptureSwitch to name and nothing a mute applied in userspace can reach:
 * on this part the microphone path is only silenced by taking its gains to
 * zero, which is real silence rather than 0 dB.
 *
 * Named "Mic Capture Switch" so alsa-lib's simple mixer presents element
 * "Mic" carrying a capture switch; UCM binds to it as CaptureSwitch "Mic",
 * and a mute then reaches the hardware instead of being applied to a stream
 * that carries no call audio.  ALSA's sense is inverted from the flag: 1 is
 * capture enabled.
 */
static int aoc_mic_switch_get(struct snd_kcontrol *kc,
			      struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	struct aoc_audio *aud = container_of(card, struct aoc_audio, card);

	uc->value.integer.value[0] = !aud->mic_muted;
	return 0;
}

static int aoc_mic_switch_put(struct snd_kcontrol *kc,
			      struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	struct aoc_audio *aud = container_of(card, struct aoc_audio, card);
	bool muted = !uc->value.integer.value[0];

	if (muted == aud->mic_muted)
		return 0;

	aud->mic_muted = muted;
	aoc_audio_mic_gain(aud);	/* a no-op until the AOC is up */

	/*
	 * The gains above are the record chain. A call does not travel it: the
	 * uplink is the AOC's modem input, taken from the microphones and
	 * handed to the modem, and the AOC offers no gain or mute on it. Its
	 * only lever is whether it runs at all, so muting a call means not
	 * running it. Nothing to do when no call is up -- hw_params starts it
	 * muted or not according to the flag.
	 */
	if (aud->voice_mic_on)
		aoc_voice_mic_set(aud, !muted);
	return 1;
}

/*
 * The routes' own volume.
 *
 * A call's audio never reaches the AP, so there is no stream for user space to
 * attenuate: the only gain left in the path is the amplifiers' own, and there
 * is one control per amplifier.  A pair of mono controls cannot be driven in
 * step from user space -- a volume merge spends the whole attenuation on the
 * first element it finds and leaves the other alone, which is an imbalance
 * rather than a volume -- so present each route the way it uses the pair: the
 * loudspeaker as one stereo control over both amplifiers, and the earpiece as
 * a mono one over the left, with the right kept silent.
 *
 * Each tops out where its route was tuned to sit rather than where the
 * hardware ends, so the amplifiers' +12 dB is not reachable through either.
 *
 * The earpiece's ceiling is 8 dB above the -22 dB this started at, which was
 * inherited from tegu's tuning and measured too quiet to hold a call outdoors.
 * There is no separate receiver on this phone -- the "earpiece" is the top
 * loudspeaker driven quietly -- so the backoff is a tuning choice rather than
 * a limit of the part, and the remaining 14 dB still keeps it well short of
 * what the same amplifier does for media. The scale is 0.125 dB a step from
 * 817 = 0 dB, so each 8 counts is 1 dB if this wants moving again; the TLV
 * maximum below has to move with it or the reported dB stops matching.
 * UCM names whichever control belongs to the device, and the dialler's volume
 * keys then land on hardware that is in the path.
 */
#define AOC_AMP_VOL_EARPIECE_MAX	705	/* -14 dB, the near-field backoff */
#define AOC_AMP_VOL_SPEAKER_MAX		817	/* 0 dB, unity */

static const char * const aoc_amp_vol_ctl[] = {
	"Digital PCM Volume",		/* the left amplifier */
	"R Digital PCM Volume",		/* and the right */
};

/* Both mirror the amplifiers' own scale, which is 0 to mute and 0.125 dB a step. */
static const DECLARE_TLV_DB_RANGE(aoc_earpiece_tlv,
		0, 0, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 1),
		1, AOC_AMP_VOL_EARPIECE_MAX, TLV_DB_MINMAX_ITEM(-10200, -1400));
static const DECLARE_TLV_DB_RANGE(aoc_speaker_tlv,
		0, 0, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 1),
		1, AOC_AMP_VOL_SPEAKER_MAX, TLV_DB_MINMAX_ITEM(-10200, 0));

struct aoc_route_vol {
	unsigned int channels;		/* amplifiers this route drives */
	unsigned int max;
};

static const struct aoc_route_vol aoc_vol_earpiece = {
	.channels = 1,
	.max = AOC_AMP_VOL_EARPIECE_MAX,
};

static const struct aoc_route_vol aoc_vol_speaker = {
	.channels = 2,
	.max = AOC_AMP_VOL_SPEAKER_MAX,
};

/* An amplifier's volume is its own control's; go through it rather than behind it. */
static int aoc_amp_vol_rw(struct snd_soc_card *card, unsigned int amp,
			  long *val, bool write)
{
	struct snd_ctl_elem_value uv = { };
	struct snd_kcontrol *kctl;
	int ret;

	kctl = snd_soc_card_get_kcontrol(card, aoc_amp_vol_ctl[amp]);
	if (!kctl)
		return -ENODEV;

	if (write) {
		uv.value.integer.value[0] = *val;
		return kctl->put(kctl, &uv);
	}

	ret = kctl->get(kctl, &uv);
	if (ret < 0)
		return ret;
	*val = uv.value.integer.value[0];
	return 0;
}

static int aoc_route_vol_info(struct snd_kcontrol *kc,
			      struct snd_ctl_elem_info *uinfo)
{
	const struct aoc_route_vol *v = (const void *)kc->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = v->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = v->max;
	return 0;
}

static int aoc_route_vol_get(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	const struct aoc_route_vol *v = (const void *)kc->private_value;
	unsigned int i;
	int ret;

	for (i = 0; i < v->channels; i++) {
		long val;

		ret = aoc_amp_vol_rw(card, i, &val, false);
		if (ret)
			return ret;

		/* The amplifier can be louder than this route is allowed to be. */
		uc->value.integer.value[i] = min_t(long, val, v->max);
	}
	return 0;
}

static int aoc_route_vol_put(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *uc)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kc);
	const struct aoc_route_vol *v = (const void *)kc->private_value;
	unsigned int i;
	int ret, changed = 0;
	long val;

	for (i = 0; i < v->channels; i++) {
		val = uc->value.integer.value[i];
		if (val < 0 || val > v->max)
			return -EINVAL;

		ret = aoc_amp_vol_rw(card, i, &val, true);
		if (ret < 0)
			return ret;
		changed |= ret;
	}

	/* The earpiece is the left amplifier alone: silence the right. */
	if (v->channels == 1) {
		val = 0;
		ret = aoc_amp_vol_rw(card, 1, &val, true);
		if (ret < 0)
			return ret;
		changed |= ret;
	}
	return changed;
}

static const struct snd_kcontrol_new aoc_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Capture Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = aoc_mic_switch_get,
		.put = aoc_mic_switch_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Boost Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = aoc_mic_gain_info,
		.get = aoc_mic_gain_get,
		.put = aoc_mic_gain_put,
		.tlv.p = aoc_mic_boost_tlv,
		.private_value = AOC_PARAM_KEY_DB + 1,	/* != the soft-gain tag */
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Mic Capture Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = aoc_mic_gain_info,
		.get = aoc_mic_gain_get,
		.put = aoc_mic_gain_put,
		.tlv.p = aoc_mic_capture_tlv,
		.private_value = AOC_PARAM_KEY_DB,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Earpiece Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = aoc_route_vol_info,
		.get = aoc_route_vol_get,
		.put = aoc_route_vol_put,
		.tlv.p = aoc_earpiece_tlv,
		.private_value = (unsigned long)&aoc_vol_earpiece,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Speaker Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = aoc_route_vol_info,
		.get = aoc_route_vol_get,
		.put = aoc_route_vol_put,
		.tlv.p = aoc_speaker_tlv,
		.private_value = (unsigned long)&aoc_vol_speaker,
	},
};

/* What a stream may ask of the AOC's ring. */
static const struct snd_pcm_hardware aoc_pcm_hw = {
	/*
	 * BATCH is not decoration: the AOC never interrupts, so the position
	 * only moves when the 10 ms tick below reads it, and between ticks it
	 * is stale by up to a tick.  Clients that believe a pointer is exact to
	 * the frame -- PipeWire among them -- otherwise run with no headroom at
	 * all and take an XRUN on the first cycle.  Declaring BATCH is what
	 * makes them allow for the granularity, and it keeps doing so for
	 * clients this driver has never met.
	 */
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
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

/*
 * The AOC does not interrupt as it drains a playback ring: it just moves the
 * read pointer.  Poll it, and tell the core how far it has got.
 */
static void aoc_pcm_push(struct aoc_audio_stream *s);
static void aoc_pcm_pull(struct aoc_audio_stream *s);

/*
 * Reporting a period is all this does.  The links are nonatomic, because the
 * AOC's control commands round-trip and .trigger has to wait for them, and on
 * a nonatomic PCM snd_pcm_period_elapsed() takes a mutex -- the same one an
 * application holds across its write.  So it cannot be called from the timer,
 * and it cannot be what moves the data either: it can be kept waiting for as
 * long as a write takes.  See aoc_pcm_tick().
 *
 * A stop needs no handshake with this.  snd_pcm_period_elapsed() rechecks
 * snd_pcm_running() under the stream lock and does nothing if the stream has
 * gone, which is the only place that check is not already stale.
 */
static void aoc_pcm_period_work(struct work_struct *w)
{
	struct aoc_audio_stream *s = container_of(w, struct aoc_audio_stream,
						  period_work);

	snd_pcm_period_elapsed(s->substream);
}

static enum hrtimer_restart aoc_pcm_tick(struct hrtimer *t)
{
	struct aoc_audio_stream *s = container_of(t, struct aoc_audio_stream,
						  tick);

	if (!READ_ONCE(s->running))
		return HRTIMER_NORESTART;

	/*
	 * Move the data from here, not from the work.  aoc_service_write() only
	 * copies into the ring and rings a doorbell, so it is safe under the
	 * ring lock this takes, and keeping it here is what makes the refill
	 * punctual: the work can be held off for as long as an application's
	 * write holds the PCM mutex, and a refill behind that lock starves the
	 * AOC.  Its renderer waits for data rather than underrunning, so a
	 * starved ring comes out as playback that runs slow rather than as a
	 * gap -- measured at 72% of nominal with a 21 ms period, and exact only
	 * once the period grew past how long the work could be delayed.
	 *
	 * Report the period from the work, which is the only part that has to
	 * enter the core.  Dropping one of those to coalescing costs an
	 * application a late wakeup; dropping a refill costs it the stream.
	 */
	if (s->playback)
		aoc_pcm_push(s);
	else
		aoc_pcm_pull(s);

	queue_work(system_highpri_wq, &s->period_work);

	hrtimer_forward_now(t, ns_to_ktime(s->tick_ns));

	return HRTIMER_RESTART;
}

static int aoc_pcm_open(struct snd_soc_component *comp,
			struct snd_pcm_substream *substream)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	const char *name;
	struct aoc_audio_stream *s;
	struct aoc_service *svc;

	if (aoc_pcm_is_be(substream))
		return 0;

	name = playback ? AOC_PLAYBACK_SERVICE : AOC_CAPTURE_SERVICE;

	svc = aoc_service_find(aud->aoc_dev, name);
	if (!svc)
		return dev_err_probe(comp->dev, -ENODEV, "no %s service\n", name);

	s = kzalloc_obj(*s);
	if (!s)
		return -ENOMEM;
	s->svc = svc;
	s->substream = substream;
	s->playback = playback;
	s->ring = aoc_service_ring_size(svc, playback);
	s->tick_ns = AOC_PCM_TICK_NS;
	s->dev = comp->dev;
	spin_lock_init(&s->ring_lock);
	/*
	 * A hard timer, so that it runs from the interrupt and not from the
	 * timer softirq: a soft one is allowed to be deferred and batched with
	 * whatever else is due, which on an otherwise idle phone means waiting
	 * for the next convenient tick.  This one carries the ring refill and
	 * has a deadline of one period, so it cannot be moved.
	 */
	hrtimer_setup(&s->tick, aoc_pcm_tick, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_HARD);
	INIT_WORK(&s->period_work, aoc_pcm_period_work);
	s->source = playback ? AOC_PLAYBACK_SOURCE : AOC_CAPTURE_SOURCE;
	substream->runtime->private_data = s;

	snd_soc_set_runtime_hwparams(substream, &aoc_pcm_hw);

	/*
	 * The position comes from a poll, not an interrupt, so a period shorter
	 * than the poll would have the core expect an update there is no way to
	 * produce.  The tick is set from the period that gets negotiated --
	 * see aoc_pcm_hw_params() -- so this only has to rule out a period the
	 * timer could not resolve at all.
	 */
	return snd_pcm_hw_constraint_minmax(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIOD_TIME,
					    AOC_PCM_TICK_NS / NSEC_PER_USEC,
					    UINT_MAX);
}

static int aoc_pcm_close(struct snd_soc_component *comp,
			 struct snd_pcm_substream *substream)
{
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (s) {
		hrtimer_cancel(&s->tick);
		disable_work_sync(&s->period_work);
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

	/*
	 * Wait out the tick and its refill work before re-basing the position.
	 *
	 * .trigger(STOP) cannot do this.  It runs atomically, so
	 * hrtimer_try_to_cancel() can return with the callback still in flight
	 * and a work item already queued behind it; this runs in process
	 * context with no stream lock held (snd_pcm_action_nonatomic), and is
	 * the only place that can wait.
	 *
	 * Letting that work land after the re-base is what kept a recovered
	 * stream broken, and it is the one case the core cannot drop for us:
	 * the stream is running again by the time the work gets in, so its
	 * snd_pcm_running() check passes.  The position the stale work then
	 * reports means nothing yet, the core reads it as the hardware having
	 * overrun the application, and the XRUN it raises re-prepares and arms
	 * the very same race again.  Measured on komodo: PipeWire
	 * recovering the speaker sink about sixteen times a second, silent, for
	 * as long as the stream stayed open, and a suspend/resume of the sink
	 * -- which closes the PCM, where this wait already happens -- clearing
	 * it every time.
	 */
	hrtimer_cancel(&s->tick);
	cancel_work_sync(&s->period_work);

	s->base = aoc_service_progress(s->svc, s->playback);
	s->pushed = 0;
	dev_dbg(s->dev, "prepare: base %u buffer %lu period %lu\n",
		s->base, substream->runtime->buffer_size,
		substream->runtime->period_size);
	return 0;
}

/* The application committed frames up to appl_ptr; hand the new ones to the ring. */
/*
 * Hand the ring whatever the application has committed and the AOC has already
 * read past.  Called both when userspace writes and from the poll: the ring
 * holds well under a period, so waiting for the next write would starve it.
 */
static void aoc_pcm_push(struct aoc_audio_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	snd_pcm_uframes_t appl;
	unsigned long flags;

	if (!s->playback)
		return;

	/* .ack (a userspace write) and the refill work both call this.  Without
	 * serialisation they race: both read the same s->pushed, write the same
	 * staging frames into the ring twice, and double-advance the pointer,
	 * which the AOC replays as garbled ("robotic") audio with buzzes under
	 * load.  aoc_service_write only memcpys into the ring and rings a
	 * doorbell (no sleep), so holding a spinlock across it is fine.
	 */
	spin_lock_irqsave(&s->ring_lock, flags);
	appl = READ_ONCE(rt->control->appl_ptr);
	while (s->pushed < appl) {
		snd_pcm_uframes_t off = s->pushed % rt->buffer_size;
		snd_pcm_uframes_t n = min_t(snd_pcm_uframes_t, appl - s->pushed,
					    rt->buffer_size - off);
		int nbytes = frames_to_bytes(rt, n);
		int w = aoc_service_write(s->svc,
					  rt->dma_area + frames_to_bytes(rt, off),
					  nbytes);

		if (w <= 0)			/* full: the AOC has not read on */
			break;
		s->pushed += bytes_to_frames(rt, w);
		if (w < nbytes)
			break;
	}
	spin_unlock_irqrestore(&s->ring_lock, flags);
	dev_dbg(s->dev, "push: appl %lu pushed %lu\n",
		(unsigned long)appl, s->pushed);
}

/*
 * Drain whatever the AOC has recorded into the ring, into the PCM buffer, and
 * report how far the capture has got.  Runs from the poll only; there is no
 * application ack for capture.
 */
static void aoc_pcm_pull(struct aoc_audio_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;

	if (s->playback)
		return;

	while (aoc_service_can_read(s->svc)) {
		snd_pcm_uframes_t off = s->pushed % rt->buffer_size;
		int room = frames_to_bytes(rt, rt->buffer_size - off);
		int r = aoc_service_read(s->svc,
					 rt->dma_area + frames_to_bytes(rt, off),
					 room);

		if (r <= 0)
			break;
		s->pushed += bytes_to_frames(rt, r);
	}
}

static int aoc_pcm_ack(struct snd_soc_component *comp,
		       struct snd_pcm_substream *substream)
{
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (aoc_pcm_is_be(substream))
		return 0;

	aoc_pcm_push(s);
	return 0;
}

static snd_pcm_uframes_t aoc_pcm_pointer(struct snd_soc_component *comp,
					 struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *rt = substream->runtime;
	struct aoc_audio_stream *s = rt->private_data;
	u32 buf_bytes = frames_to_bytes(rt, rt->buffer_size);
	u32 taken, pos, hold;

	/* Capture position is what we have moved into the buffer ourselves. */
	if (!s->playback)
		return s->pushed % rt->buffer_size;

	taken = aoc_service_progress(s->svc, s->playback) - s->base;
	/*
	 * What the AOC has taken is not what it has rendered.  On start it
	 * swallows up to a whole ring at once -- hardware-observed: the writes
	 * saturate at 2048 frames and the read counter jumps that far within
	 * half a millisecond, against the 42 ms those frames actually last --
	 * and only then does it pace itself at the sample rate.
	 *
	 * Reported as-is, that start-up gulp lands the hardware pointer exactly
	 * on appl_ptr whenever the application committed less than a ring
	 * before starting, and the core reads an empty buffer and raises an
	 * XRUN on the very first pointer read.
	 *
	 * So hold the pointer back -- but by a period, not by the ring.  The
	 * holdback is a delay before the core sees any progress at all, and an
	 * application only writes once it does, so a holdback worth several
	 * periods starves the stream it is meant to protect: it waits, the ring
	 * runs dry, and the AOC's renderer waits for data rather than
	 * underrunning, so the result is playback that runs slow.  Measured
	 * against a 48 kHz stream, by how many periods the ring came to:
	 *
	 *	holdback 1 period   47990 pushed / 47977 consumed frames/s
	 *	holdback 2 periods  34796 pushed / 34905 consumed frames/s
	 *	holdback 8 periods      0 pushed /     0 consumed frames/s
	 *
	 * One period is all the core needs: it keeps the hardware pointer off
	 * appl_ptr, which is what the XRUN came from, and costs a period of
	 * reported latency, which the stream genuinely has.
	 */
	hold = frames_to_bytes(rt, rt->period_size);
	pos = taken > hold ? taken - hold : 0;
	dev_dbg(s->dev,
		"pointer: taken %u ring %u played %u -> %lu of %lu, hw %lu pushed %lu\n",
		taken, s->ring, pos,
		(unsigned long)bytes_to_frames(rt, pos % buf_bytes),
		rt->buffer_size, (unsigned long)READ_ONCE(rt->status->hw_ptr),
		s->pushed);
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

	if (!s->playback) {
		ret = aoc_audio_mic_setup(aud, s->source, params_channels(params),
					  params_rate(params), params_width(params),
					  params_buffer_bytes(params),
					  params_period_bytes(params));
		if (ret)
			dev_err(comp->dev, "mic setup failed: %d\n", ret);
		return ret;
	}

	/*
	 * Beat on the period that was negotiated, not on an interval of this
	 * driver's choosing.  The tick is the only thing that reports a period
	 * elapsed, and a client sleeps until it hears one, so a tick that does
	 * not divide the period reports every boundary late by a margin that
	 * walks -- asking for 1024 frames at 48 kHz wants 21.33 ms, which a
	 * fixed 10 ms tick can only answer 4 to 9 ms after the fact, and that
	 * is most of the slack a two-period buffer has.
	 *
	 * One tick per period, which is what an interrupt-driven endpoint would
	 * give: a boundary always coincides with a tick, and the fixed cost of
	 * reporting one is paid once per period rather than twice.
	 */
	s->tick_ns = max_t(u64, div_u64((u64)params_period_size(params) *
					NSEC_PER_SEC, params_rate(params)),
			   NSEC_PER_MSEC);

	/* A stream describes its ring to the AOC before anything is bound to it. */
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
	if (ret) {
		dev_err(comp->dev, "speaker bind failed: %d\n", ret);
		return ret;
	}

	/*
	 * Lift the speaker off the AOC's quiet boot default to unity, matching
	 * the stock HAL -- otherwise full-scale ALSA output is a fraction of the
	 * loudness Android reaches.
	 */
	ret = aoc_audio_out_volume(aud, s->source, AOC_SINK_SPEAKER,
				   AOC_SPEAKER_VOLUME_MAX);
	if (ret)
		dev_err(comp->dev, "speaker volume set failed: %d\n", ret);
	return ret;
}

static int aoc_pcm_hw_free(struct snd_soc_component *comp,
			   struct snd_pcm_substream *substream)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	struct aoc_audio_stream *s = substream->runtime->private_data;

	if (aoc_pcm_is_be(substream) || !s->playback)
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

	if (on) {
		WRITE_ONCE(s->running, true);
		hrtimer_start(&s->tick, ns_to_ktime(s->tick_ns),
			      HRTIMER_MODE_REL_HARD);
	} else {
		WRITE_ONCE(s->running, false);
		hrtimer_try_to_cancel(&s->tick);
	}

	/* Capture starts/stops the mics; playback's route is already up. */
	if (!s->playback)
		ret = aoc_audio_mic(aud, on);
	else
		ret = aoc_audio_source(aud, s->source, on);

	dev_dbg(comp->dev, "%s %s: %d\n", s->playback ? "playback" : "capture",
		on ? "start" : "stop", ret);
	return ret;
}

static const struct snd_soc_component_driver aoc_component = {
	.name = "aoc-pcm",
	/* Honour each dai_link's .id as its PCM device number. */
	.use_dai_pcm_id = 1,
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

/*
 * Configure the amplifiers for the frame the AOC puts on the wire.  Shared:
 * both the back end a stream routes through and the call's own link end up
 * driving the same two parts over the same TDM.
 */
static int aoc_amps_hw_params(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_hw_params *params)
{
	unsigned int bclk = params_rate(params) * AOC_TDM0_SLOTS *
			    AOC_TDM0_SLOT_WIDTH;
	struct snd_soc_dai *codec_dai;
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		/*
		 * Slot 0 carries the left channel and slot 1 the right, and the
		 * device tree lists the amplifiers in that order, so an
		 * amplifier's slot is its place in the list.  The other slot
		 * goes to its second receive channel, which is what its
		 * firmware wants for cross-channel work.
		 *
		 * Taking it from the list rather than from a property of the
		 * codec's own node keeps this out of another driver's binding.
		 */
		unsigned int slot = i % AOC_TDM0_CHANNELS;
		unsigned int rx_slot[2] = { slot, !slot };

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

/*
 * The call's bring-up, which the link runs for us.
 *
 * Nothing here moves samples: the modem's downlink and its uplink both stay
 * inside the coprocessor, and the AP's whole part is to bind that downlink to
 * the speaker sink, keep the modem's mic input running and start the modem's
 * flow.  They are the link's callbacks, so ASoC runs them when DAPM completes
 * the path to the amplifiers -- there is no PCM to open, and so nothing in user
 * space has to hold one for the length of the call.
 */
static int aoc_voice_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	int ret;

	/* The amplifiers hear what they hear for a stream. */
	ret = aoc_amps_hw_params(snd_soc_substream_to_rtd(substream), params);
	if (ret)
		return ret;

	ret = aoc_voice_replay_pre(aud);
	if (ret)
		return ret;

	/* The DSP wants a mic source live before it will bind the sink. */
	aud->voice_mic_on = true;
	ret = aoc_voice_mic_set(aud, !aud->mic_muted);
	if (ret) {
		dev_err(dai->dev, "call mic start failed: %d\n", ret);
		goto err_mic;
	}

	ret = aoc_audio_bind(aud, AOC_SRC_TELEPHONY_DOWNLINK, AOC_SINK_SPEAKER,
			     true);
	if (ret) {
		dev_err(dai->dev, "speaker bind failed: %d\n", ret);
		goto err_bind;
	}

	/* cntr 191-192: the input-channel pair Android sends after the bind. */
	ret = aoc_voice_replay_post_bind(aud);
	if (ret)
		goto err_post;

	/*
	 * And last, the modem's flow, now that the sink it feeds is bound.
	 *
	 * This belongs in .prepare, which is where the link's power-up reaches
	 * after hw_params and which is where the vendor stack starts it too.
	 * It cannot go there: a codec-to-codec link's playback runs from its
	 * CPU end's *capture* direction, so this DAI declares capture only,
	 * and snd_soc_dai_prepare() skips a DAI whose stream the substream's
	 * direction is not valid for -- the link hands it PLAYBACK.  Nothing
	 * reports that, and everything else here is on the ungated hw_params
	 * path, so the symptom is a call with its route up, its sink bound and
	 * no audio at all.
	 */
	ret = aoc_audio_modem(aud, true);
	if (ret) {
		dev_err(dai->dev, "modem start failed: %d\n", ret);
		goto err_post;
	}

	ret = aoc_audio_set_param(aud, AOC_TELEPHONY_SINK_BLOCK,
				  AOC_TELEPHONY_COMPONENT_POST,
				  AOC_TELEPHONY_BLOCK_KEY, 0);
	if (ret)
		goto err_modem;

	return 0;

err_modem:
	aoc_audio_modem(aud, false);
err_post:
	aoc_audio_bind(aud, AOC_SRC_TELEPHONY_DOWNLINK, AOC_SINK_SPEAKER, false);
err_bind:
	aud->voice_mic_on = false;
	aoc_voice_mic_set(aud, false);
	goto err_teardown;
err_mic:
	aud->voice_mic_on = false;
err_teardown:
	aoc_voice_replay_teardown(aud);
	aoc_audio_dsp_mode(aud, AOC_DSP_MODE_AMBIENT);
	return ret;
}

/* Reverse of the bring-up: the modem first, then the sink, then its mic. */
static int aoc_voice_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct aoc_audio *aud = substream_to_aud(substream);
	int ret, err;

	ret = aoc_audio_modem(aud, false);

	err = aoc_audio_bind(aud, AOC_SRC_TELEPHONY_DOWNLINK, AOC_SINK_SPEAKER,
			     false);
	if (err && !ret)
		ret = err;

	aud->voice_mic_on = false;
	err = aoc_voice_mic_set(aud, false);
	if (err && !ret)
		ret = err;

	aoc_voice_replay_teardown(aud);
	aoc_audio_dsp_mode(aud, AOC_DSP_MODE_AMBIENT);
	return ret;
}

static const struct snd_soc_dai_ops aoc_voice_dai_ops = {
	.hw_params = aoc_voice_hw_params,
	.hw_free = aoc_voice_hw_free,
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
		 * Capture's own DAI.  One DAI cannot serve two links: it holds a
		 * single widget per direction and a single runtime's worth of
		 * state, so naming it from both the playback and the capture
		 * link has the second bind overwrite the first.
		 */
		.name = "aoc-fe-cap",
		.capture = {
			.stream_name = "AOC Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
	},
	{
		/*
		 * The telephony route's own DAI, standing in for the modem.  A
		 * codec-to-codec link's CPU end carries the capture of what the
		 * codec end plays back, and that is what this is: what the AOC
		 * hands the amplifiers once the modem's downlink is bound to them.
		 * Nothing is captured here -- the AP never sees a call's audio.
		 */
		.name = "aoc-voice",
		.capture = {
			.stream_name = "AOC Voice",
			/* The wire's frame, not what the CP decodes to. */
			.channels_min = AOC_TDM0_CHANNELS,
			.channels_max = AOC_TDM0_CHANNELS,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = AOC_TDM0_FMTBIT,
		},
		.ops = &aoc_voice_dai_ops,
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

/* The switch that opens a call, which is all user space has to set. */
static const struct snd_kcontrol_new aoc_voice_enable_ctl =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

/*
 * A call has no stream to power its route up, so the modem's downlink is given
 * a source of its own and a switch in front of it.  Turning the switch on
 * completes the path to the amplifiers, and a complete path is what has ASoC
 * bring the link up -- the whole of what a call asks of the AP, from one cset.
 */
static const struct snd_soc_dapm_widget aoc_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Modem Downlink", NULL),
	SND_SOC_DAPM_SWITCH("Call Downlink", SND_SOC_NOPM, 0, 0,
			    &aoc_voice_enable_ctl),
};

static const struct snd_soc_dapm_route aoc_dapm_routes[] = {
	/* The ring the AP writes, out over the TDM the AOC clocks. */
	{ "TDM_0_RX Playback", NULL, "AOC Playback" },
	/* A call, to the same amplifiers over the same wire, with no ring. */
	{ "Call Downlink", "Switch", "Modem Downlink" },
	{ "AOC Voice", NULL, "Call Downlink" },
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

	rate->min = AOC_TDM0_RATE;
	rate->max = AOC_TDM0_RATE;
	channels->min = AOC_TDM0_CHANNELS;
	channels->max = AOC_TDM0_CHANNELS;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, AOC_TDM0_FORMAT);
	return 0;
}

static int aoc_tdm0_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	return aoc_amps_hw_params(snd_soc_substream_to_rtd(substream), params);
}

static const struct snd_soc_ops aoc_tdm0_ops = {
	.hw_params = aoc_tdm0_hw_params,
};

/* The CPU DAI each link names.  Resolved onto aoc_audio::cpus at probe. */
static const char * const aoc_cpu_dai_names[AOC_LINK_COUNT] = {
	[AOC_LINK_PLAYBACK]	= "aoc-fe",
	[AOC_LINK_CAPTURE]	= "aoc-fe-cap",
	[AOC_LINK_VOICE]	= "aoc-voice",
	[AOC_LINK_TDM0]		= "aoc-tdm0",
};

/*
 * What the AOC puts on the TDM for a call.  The link configures the amplifiers
 * with it, as the back end's fixup does for a stream: the frame is the wire's
 * and is the same either way.
 */
static const struct snd_soc_pcm_stream aoc_voice_c2c_params = {
	.formats = AOC_TDM0_FMTBIT,
	.rate_min = AOC_TDM0_RATE,
	.rate_max = AOC_TDM0_RATE,
	.channels_min = AOC_TDM0_CHANNELS,
	.channels_max = AOC_TDM0_CHANNELS,
};

static const struct snd_soc_dai_link aoc_dai_link_tmpl[AOC_LINK_COUNT] = {
	{
		.name = "aoc-playback0",
		.stream_name = "aoc-playback0",
		.id = 0,
		.num_cpus = 1,
		.codecs = &snd_soc_dummy_dlc,
		.num_codecs = 1,
		.num_platforms = 1,
		.dynamic = 1,
		.playback_only = 1,
		/* control commands round-trip to the AOC, so the trigger must sleep */
		.nonatomic = true,
	},
	{
		/*
		 * Capture is a plain PCM: the AOC records the mics into the ring
		 * on its own once started, with no back-end to route.
		 */
		.name = "aoc-capture0",
		.stream_name = "aoc-capture0",
		.id = 1,
		.num_cpus = 1,
		.codecs = &snd_soc_dummy_dlc,
		.num_codecs = 1,
		.num_platforms = 1,
		.capture_only = 1,
		.nonatomic = true,
	},
	{
		/*
		 * The telephony route: the amplifiers listening to the modem's
		 * downlink, with the AOC in between and the AP outside the data
		 * path altogether.  A codec-to-codec link is exactly that -- two
		 * ends ASoC configures and runs itself, powered by DAPM -- so a
		 * call has no PCM device of its own, which is what lets it be
		 * opened by a mixer switch instead of by a process.
		 */
		.name = "aoc-voice",
		.stream_name = "aoc-voice",
		/* Internal now, but keep the vendor's telephony device number. */
		.id = 4,
		.num_cpus = 1,
		/* .codecs comes from the device tree: the amplifiers */
		.num_platforms = 1,
		.c2c_params = &aoc_voice_c2c_params,
		.num_c2c_params = 1,
		.playback_only = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		/* A call is not a stream and does not stop for a suspend. */
		.ignore_suspend = 1,
		/* the AOC's commands round-trip, and the amplifiers are on SPI */
		.nonatomic = true,
	},
	{
		.name = "aoc-tdm0",
		.stream_name = "TDM_0_RX Playback",
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

/*
 * Refuse a card whose links did not all bind. The DPCM open path reaches the
 * card lock through dai->component->card without checking it, so a link whose
 * component never bound panics the machine the first time anything opens the
 * PCM rather than failing the open. Better to have no card at all.
 */
static int aoc_card_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *dai;
	int i, ret = 0;

	for_each_card_rtds(card, rtd) {
		bool unbound = false;

		/*
		 * A dummy DAI's component is never bound to the card: the core
		 * returns early for it before taking that reference. Its NULL
		 * card is not evidence of anything, so skip it.
		 */
		for_each_rtd_cpu_dais(rtd, i, dai)
			if (!snd_soc_dai_is_dummy(dai) &&
			    (!dai->component || !dai->component->card))
				unbound = true;
		for_each_rtd_codec_dais(rtd, i, dai)
			if (!snd_soc_dai_is_dummy(dai) &&
			    (!dai->component || !dai->component->card))
				unbound = true;

		if (unbound) {
			dev_err(card->dev, "%s: bound to no card\n",
				rtd->dai_link->name);
			ret = -ENODEV;
		}
	}

	return ret;
}

static int aoc_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *aoc_pdev;
	struct aoc_audio *aud;
	struct device_node *np;
	unsigned int i;
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
	aud->mic_hw_gain_cb = AOC_MIC_HW_GAIN_CB;
	aud->mic_soft_gain_db = AOC_MIC_REC_SOFT_GAIN_DB;
	aud->ctrl = aoc_service_find(aud->aoc_dev, AOC_OUTPUT_CTRL_SERVICE);
	if (aud->ctrl)
		aoc_service_set_handler(aud->ctrl, aoc_audio_ctrl_isr, aud);
	else
		dev_warn(dev, "no %s service; playback trigger will fail\n",
			 AOC_OUTPUT_CTRL_SERVICE);

	aud->ctrl_in = aoc_service_find(aud->aoc_dev, AOC_INPUT_CTRL_SERVICE);
	if (aud->ctrl_in)
		aoc_service_set_handler(aud->ctrl_in, aoc_audio_ctrl_isr, aud);
	else
		dev_warn(dev, "no %s service; capture will fail\n",
			 AOC_INPUT_CTRL_SERVICE);

	ret = devm_snd_soc_register_component(dev, &aoc_component, aoc_dais,
					      ARRAY_SIZE(aoc_dais));
	if (ret) {
		dev_err_probe(dev, ret, "cannot register component\n");
		goto err_put;
	}

	/*
	 * Take a copy of the link template for this card and point every link
	 * at this card's own components, all of which live in the AOC's own
	 * device tree node.
	 */
	memcpy(aud->links, aoc_dai_link_tmpl, sizeof(aud->links));
	aud->platform.of_node = dev->of_node;
	for (i = 0; i < AOC_LINK_COUNT; i++) {
		aud->cpus[i].dai_name = aoc_cpu_dai_names[i];
		aud->cpus[i].of_node = dev->of_node;
		aud->links[i].cpus = &aud->cpus[i];
		if (aud->links[i].num_platforms)
			aud->links[i].platforms = &aud->platform;
	}

	/*
	 * The amplifiers listening to the TDM.  They are the codecs of both the
	 * links that reach the wire: the back end a stream routes through, and
	 * the call, which has the AOC drive them with no stream at all.
	 */
	np = of_get_child_by_name(dev->of_node, "speakers");
	if (!np) {
		ret = dev_err_probe(dev, -EINVAL, "no speakers node\n");
		goto err_put;
	}
	for (i = 0; i < AOC_LINK_COUNT; i++) {
		if (i != AOC_LINK_VOICE && i != AOC_LINK_TDM0)
			continue;
		ret = snd_soc_of_get_dai_link_codecs(dev, np, &aud->links[i]);
		if (ret)
			break;
	}
	of_node_put(np);
	if (ret) {
		dev_err_probe(dev, ret, "cannot resolve the speaker codecs\n");
		goto err_put;
	}

	aud->card.name = "aoc-audio";
	aud->card.owner = THIS_MODULE;
	aud->card.dev = dev;
	aud->card.dai_link = aud->links;
	aud->card.num_links = AOC_LINK_COUNT;
	aud->card.dapm_widgets = aoc_dapm_widgets;
	aud->card.num_dapm_widgets = ARRAY_SIZE(aoc_dapm_widgets);
	aud->card.dapm_routes = aoc_dapm_routes;
	aud->card.num_dapm_routes = ARRAY_SIZE(aoc_dapm_routes);
	aud->card.controls = aoc_controls;
	aud->card.num_controls = ARRAY_SIZE(aoc_controls);
	aud->card.late_probe = aoc_card_late_probe;

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
	if (aud->ctrl_in)
		aoc_service_set_handler(aud->ctrl_in, NULL, NULL);
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
