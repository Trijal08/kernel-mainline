// SPDX-License-Identifier: GPL-2.0-only
/*
 * AoC USF sensor framework -> IIO bridge.
 *
 * Bridges the sensors hosted by the AoC USF (Unified Sensor Framework)
 * firmware to Linux IIO. The AoC sensor registry is loaded out of band by a
 * userspace daemon; once it is loaded this driver enumerates the sensors over
 * the com.google.usf AOCC channel and exposes them as IIO devices. This is the
 * data-plane half described in docs/subsystems/sensors/reference/usf-iio-bridge.md.
 *
 * The registry-load daemon "pokes" this driver once the registry is ready by
 * writing the "enumerate" sysfs attribute; the driver stays inert until then.
 * The autopoll module parameter is a kernel-only bring-up escape hatch that
 * enumerates at probe instead.
 */

#define pr_fmt(fmt) "usf-iio: " fmt

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#include <linux/soc/google/aoc.h>

#include <linux/soc/google/usf.h>

#define USF_RETRY_MS		1000	/* bootstrap retry cadence */
#define USF_MAX_RETRIES		15	/* ~15 s before giving up */

#define USF_LIST_MAX		64	/* enumerated sensor handles */
#define USF_MAX_DEV		12	/* max IIO devices registered */

/*
 * The bridge asks for USF_MODE_FUSED for every sensor it exposes. The
 * downstream HAL distinguishes USF_MODE_PHYSICAL for on-chip parts, but the
 * fused mode streams them all correctly and is what this driver was validated
 * with; the distinction only matters for wake-up gestures, which are not IIO.
 */
#define USF_IIO_MODE		USF_MODE_FUSED

static bool autopoll;
module_param(autopoll, bool, 0644);
MODULE_PARM_DESC(autopoll,
		 "enumerate sensors at probe instead of waiting for the sysfs poke");

static char *sensor = "";
module_param(sensor, charp, 0644);
MODULE_PARM_DESC(sensor,
		 "if set, only expose USF sensors whose name contains this substring (default: all supported)");

static int rate = 50;
module_param(rate, int, 0644);
MODULE_PARM_DESC(rate,
		 "initial sampling rate in Hz; per-device sampling_frequency overrides it (AoC caps per sensor)");

struct usf_sensor;

struct usf_iio {
	struct device *dev;
	struct device *aoc_dev;
	struct delayed_work enum_work;
	unsigned int retries;
	bool enumerated;

	/* AOCC transport + USF request/response (drivers/soc/google/aoc). */
	struct usf_session *usf;

	/*
	 * Serialises this driver's stream transitions (start / stop / retune)
	 * so a retune cannot race a stop into leaving the AoC streaming a
	 * sampling_id nothing stops again. The session has its own lock for the
	 * wire, which says nothing about the order of *our* state changes.
	 */
	struct mutex stream_lock;

	/* Registered IIO devices, one per exposed sensor. */
	struct usf_sensor *sensors[USF_MAX_DEV];
	int nsensors;
	spinlock_t sample_lock;	/* fences sample push/lookup vs buffer teardown */
	bool ts_logged;		/* one-shot log of the first timestamp mapping */
};

/* Per-IIO-device state (in iio_priv). */
struct usf_sensor {
	struct usf_iio *usf;
	struct iio_dev *indio;	/* owning IIO device */
	u32 handle;		/* USF sensor handle (dst for Create/Reconfig) */
	u32 client_id;		/* AP-chosen opaque id echoed in samples */
	u32 sampling_id;	/* live stream id (0 = not streaming) */
	s64 period_ns;
	int samp_freq;		/* requested rate in Hz (sampling_frequency) */
	u8 ndata;		/* data channels (1 scalar, 3 vector) */

	/*
	 * Scale: samples quantise to raw = round(value / resolution), so raw is
	 * a native-unit LSB count and IIO SCALE carries the resolution (folded
	 * with the Android->IIO unit factor). res_q = resolution << USF_SCALE_Q;
	 * 0 means the firmware gave no resolution -> micro-unit fallback.
	 */
	u64 res_q;
	u64 scale_nano;		/* IIO_CHAN_INFO_SCALE in nano units */
};

/* Fixed-point fraction bits for the resolution divisor (res_q). */
#define USF_SCALE_Q	30
#define USF_NANO	1000000000ULL

/* Forced scan masks (all data channels; timestamp is tracked separately). */
static const unsigned long usf_scan_masks_3axis[] = { GENMASK(2, 0), 0 };
static const unsigned long usf_scan_masks_scalar[] = { BIT(0), 0 };

/*
 * Sample axes arrive as f32 in Android sensor units. The data path quantises
 * each to raw = round(value / resolution) as an s32, and IIO SCALE = resolution
 * (times the Android->IIO unit factor), so the value is recovered as raw*SCALE
 * without the fixed-1e-6 saturation that clipped high-range sensors (ALS, mag).
 */
#define USF_AXIS(_type, _mod, _idx) {				\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _idx,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
}
#define USF_3AXIS_CHANNELS(_type)		\
	USF_AXIS(_type, X, 0),			\
	USF_AXIS(_type, Y, 1),			\
	USF_AXIS(_type, Z, 2),			\
	IIO_CHAN_SOFT_TIMESTAMP(3)

static const struct iio_chan_spec usf_accel_channels[] = {
	USF_3AXIS_CHANNELS(IIO_ACCEL),
};
static const struct iio_chan_spec usf_gyro_channels[] = {
	USF_3AXIS_CHANNELS(IIO_ANGL_VEL),
};
static const struct iio_chan_spec usf_magn_channels[] = {
	USF_3AXIS_CHANNELS(IIO_MAGN),
};

#define USF_SCALAR_CHANNEL(_type) {				\
	.type = _type,						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = 0,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
}
#define USF_SCALAR_CHANNELS(_type)		\
	USF_SCALAR_CHANNEL(_type),		\
	IIO_CHAN_SOFT_TIMESTAMP(1)

static const struct iio_chan_spec usf_light_channels[] = {
	USF_SCALAR_CHANNELS(IIO_LIGHT),
};
static const struct iio_chan_spec usf_prox_channels[] = {
	USF_SCALAR_CHANNELS(IIO_PROXIMITY),
};
static const struct iio_chan_spec usf_pressure_channels[] = {
	USF_SCALAR_CHANNELS(IIO_PRESSURE),
};

/* Map a USF sensor name (substring) to an IIO device type + channels. */
static const struct usf_type_map {
	const char *match;	/* case-insensitive substring of the USF name */
	const char *iio_name;
	const struct iio_chan_spec *channels;
	int num_channels;
	int ndata;		/* data channels (1 scalar, 3 vector) */
	const unsigned long *scan_masks;
	/*
	 * Android-native -> IIO unit factor, times 1e9: SCALE(nano) =
	 * resolution * unit_nano. 1e9 = 1:1 (accel m/s^2, gyro rad/s, light
	 * lux, prox count); mag uT -> Gauss is /100; pressure hPa -> kPa is /10.
	 */
	u64 unit_nano;
} usf_type_maps[] = {
	{ "Accelerometer", "usf_accel", usf_accel_channels,
	  ARRAY_SIZE(usf_accel_channels), 3, usf_scan_masks_3axis, 1000000000ULL },
	{ "Gyroscope", "usf_gyro", usf_gyro_channels,
	  ARRAY_SIZE(usf_gyro_channels), 3, usf_scan_masks_3axis, 1000000000ULL },
	{ "Magnetometer", "usf_magn", usf_magn_channels,
	  ARRAY_SIZE(usf_magn_channels), 3, usf_scan_masks_3axis, 10000000ULL },
	{ "Ambient Light", "usf_als", usf_light_channels,
	  ARRAY_SIZE(usf_light_channels), 1, usf_scan_masks_scalar, 1000000000ULL },
	/* "Proximity" alone also matches gesture/voice/AAD sensors; be specific */
	{ "TMD3733 Proximity", "usf_prox", usf_prox_channels,
	  ARRAY_SIZE(usf_prox_channels), 1, usf_scan_masks_scalar, 1000000000ULL },
	{ "Barometer", "usf_baro", usf_pressure_channels,
	  ARRAY_SIZE(usf_pressure_channels), 1, usf_scan_masks_scalar, 100000000ULL },
};

static int usf_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct usf_sensor *s = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/* SCALE = resolution * unit factor, discovered per sensor. */
		*val = s->scale_nano / USF_NANO;
		*val2 = s->scale_nano % USF_NANO;
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = s->samp_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/*
 * Set the sampling rate. The AoC picks the nearest supported ODR, so the value
 * read back is what was requested, not necessarily what streams. A live stream
 * is retuned in place (ReconfigSampling keeps the sampling_id); otherwise the
 * rate takes effect at the next buffer enable.
 */
static int usf_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask)
{
	struct usf_sensor *s = iio_priv(indio_dev);
	struct usf_iio *usf = s->usf;
	s64 period_ns;
	u32 sid;
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val <= 0 || val > 1000000)		/* keep period_ns >= 1000 ns */
		return -EINVAL;

	s->samp_freq = val;
	period_ns = 1000000000LL / val;

	/*
	 * Read sampling_id *inside* stream_lock, and hold it across the retune.
	 * usf_stop_sampling() (buffer disable) clears sampling_id under the same
	 * lock before it sends its StopSampling, so if we observe a live sid here
	 * a concurrent stop cannot have disabled yet and its disable is
	 * serialised after our enable. Otherwise we would risk leaving the AoC
	 * streaming a sampling_id that nothing stops again.
	 */
	ret = 0;
	mutex_lock(&usf->stream_lock);
	spin_lock(&usf->sample_lock);
	sid = s->sampling_id;
	spin_unlock(&usf->sample_lock);
	if (sid) {
		ret = usf_session_reconfig(usf->usf, s->handle, sid, period_ns,
					   0, true);
		if (ret >= 0)
			s->period_ns = period_ns;
	}
	mutex_unlock(&usf->stream_lock);

	if (ret < 0) {
		dev_warn(usf->dev, "ReconfigSampling(rate) failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct iio_info usf_iio_info = {
	.read_raw = usf_read_raw,
	.write_raw = usf_write_raw,
};

/* Case-insensitive substring test. */
static bool usf_name_has(const char *name, const char *sub)
{
	size_t nl = strlen(name), sl = strlen(sub), i, j;

	if (!sl)
		return true;
	for (i = 0; i + sl <= nl; i++) {
		for (j = 0; j < sl; j++)
			if (tolower(name[i + j]) != tolower(sub[j]))
				break;
		if (j == sl)
			return true;
	}
	return false;
}

/*
 * IEEE-754 binary32 -> round(|value| * mul * 2^q) as a u64 magnitude, integer
 * math only (the kernel must not touch the FPU). @neg (if given) gets the sign.
 * Saturates to U64_MAX on overflow / non-finite so callers can clamp.
 */
static u64 usf_f32_scaled(u32 bits, u64 mul, unsigned int q, bool *neg)
{
	u32 mant = bits & 0x7fffff;
	int exp = (bits >> 23) & 0xff;
	u64 num;
	int e;

	if (neg)
		*neg = bits & 0x80000000u;
	if (exp == 0)
		return 0;			/* zero / subnormal ~= 0 */
	if (exp == 0xff)
		return U64_MAX;			/* inf / NaN */

	num = ((u64)1 << 23) | mant;		/* value = num * 2^(exp-127-23) */
	if (mul > 1) {
		if (num > U64_MAX / mul)
			return U64_MAX;
		num *= mul;
	}
	e = exp - 127 - 23 + (int)q;		/* fold the 2^q into the exponent */
	if (e >= 0) {
		if (e >= 64 || num > (U64_MAX >> e))
			return U64_MAX;
		return num << e;
	}
	e = -e;
	if (e >= 64)
		return 0;
	return (num + (1ULL << (e - 1))) >> e;	/* round-nearest */
}

static s32 usf_clamp_s32(u64 mag, bool neg)
{
	if (mag > S32_MAX)
		return neg ? S32_MIN : S32_MAX;
	return neg ? -(s32)mag : (s32)mag;
}

/* Quantise one sample axis to a raw scan value for its device's scale. */
static s32 usf_scale_sample(const struct usf_sensor *s, u32 val_bits)
{
	bool neg;
	u64 val_q;

	if (!s->res_q)		/* no resolution reported: micro-unit fallback */
		return usf_clamp_s32(usf_f32_scaled(val_bits, 1000000, 0, &neg),
				     neg);

	val_q = usf_f32_scaled(val_bits, 1, USF_SCALE_Q, &neg);
	if (val_q == U64_MAX)
		return neg ? S32_MIN : S32_MAX;
	return usf_clamp_s32((val_q + s->res_q / 2) / s->res_q, neg);
}

/*
 * Map a sample's AoC timestamp to CLOCK_BOOTTIME. The AoC stamps samples in ns
 * off the architected counter it shares with the AP, so aoc_ts_to_boottime_ns()
 * recovers the exact boottime; on the rare miss (firmware not ready, or a stamp
 * too far from now to trust) fall back to the current boottime. The first
 * mapping is logged once so the mapping can be checked on HW (a healthy sample
 * is a few ms old).
 */
static s64 usf_sample_boottime(struct usf_iio *usf, u64 aoc_ts)
{
	u64 boot = aoc_ts_to_boottime_ns(usf->aoc_dev, aoc_ts);

	if (!usf->ts_logged) {
		u64 now = ktime_get_boottime_ns();

		usf->ts_logged = true;
		dev_info(usf->dev,
			 "timestamp map: aoc_ts=%llu -> boottime=%llu (now=%llu, age=%lld ns)%s\n",
			 aoc_ts, boot, now, (s64)(now - boot),
			 boot ? "" : " [untrusted, substituting now]");
	}
	return (s64)(boot ? boot : ktime_get_boottime_ns());
}

/* Parse a type-9 compact sample batch and push each record to the IIO buffer. */
static void usf_handle_sample(struct usf_iio *usf, const u8 *pay, u32 plen)
{
	struct usf_sensor *match = NULL;
	struct iio_dev *indio;
	struct usf_sample_hdr hdr;
	u32 scount, dcount, sid, i, d;
	size_t stride, off;
	int k, n;
	/*
	 * The device scan is n data s32 at offsets 0.. then an 8-aligned s64
	 * timestamp. One 3-axis-sized buffer serves both scalar and vector
	 * devices: iio_push writes the timestamp using the device's scan_bytes,
	 * so for a scalar device it lands at offset 8 (over the unused axes).
	 */
	struct {
		s32 chan[3];
		aligned_s64 timestamp;
	} scan;

	if (plen < sizeof(hdr))
		return;
	memcpy(&hdr, pay, sizeof(hdr));
	sid = hdr.sampling_id;
	scount = usf_sample_count(&hdr);
	dcount = usf_sample_dcount(&hdr);
	stride = 8 + (size_t)dcount * 4;

	/*
	 * Hold sample_lock across the sampling_id lookup and the pushes:
	 * usf_stop_sampling() clears sampling_id under the same lock on buffer
	 * disable, so once it returns no push can be in flight to race the IIO
	 * buffer teardown. iio_push_to_buffers is non-sleeping (kfifo), so this
	 * is safe under a spinlock.
	 */
	spin_lock(&usf->sample_lock);
	for (k = 0; k < usf->nsensors; k++) {
		if (usf->sensors[k]->sampling_id &&
		    usf->sensors[k]->sampling_id == sid) {
			match = usf->sensors[k];
			break;
		}
	}
	if (!match) {
		spin_unlock(&usf->sample_lock);
		return;			/* not one of our active streams */
	}
	indio = match->indio;
	n = min_t(int, match->ndata, dcount);

	for (i = 0; i < scount; i++) {
		u64 tsp;
		s64 ts;

		off = 16 + (size_t)i * stride;
		if (off + stride > plen)
			break;
		tsp = get_unaligned_le64(pay + off);
		ts = usf_sample_boottime(usf, tsp & USF_SAMPLE_TS_MASK);

		memset(&scan, 0, sizeof(scan));
		for (d = 0; d < (u32)n; d++)
			scan.chan[d] = usf_scale_sample(match,
				get_unaligned_le32(pay + off + 8 + d * 4));

		iio_push_to_buffers_with_ts(indio, &scan, sizeof(scan), ts);
	}
	spin_unlock(&usf->sample_lock);
}

/*
 * AOCC rx callback. Runs in the per-service demux kthread; must not sleep.
 * Responses (type 2) matching the awaited txn wake the control path; async
 * samples (type 9) are demuxed to the IIO buffer by sampling_id.
 */
/* Sample batches from either channel (session rx thread; must not sleep). */
static void usf_iio_sample(void *ctx, const u8 *pay, u32 plen)
{
	usf_handle_sample(ctx, pay, plen);
}

/* CreateSampling: start the AoC stream for this sensor. */
static int usf_start_sampling(struct usf_sensor *s)
{
	struct usf_iio *usf = s->usf;
	s64 period_ns;
	u32 sid;
	int ret;

	period_ns = 1000000000LL / (s->samp_freq > 0 ? s->samp_freq : 50);
	s->client_id = 0xC0DE0000u | s->handle;

	mutex_lock(&usf->stream_lock);
	ret = usf_session_start_sampling(usf->usf, s->handle, USF_IIO_MODE,
					 period_ns, s->client_id, &sid);
	if (!ret) {
		s->period_ns = period_ns;
		spin_lock(&usf->sample_lock);
		s->sampling_id = sid;
		spin_unlock(&usf->sample_lock);
	}
	mutex_unlock(&usf->stream_lock);

	if (ret)
		return ret;

	dev_info(usf->dev, "%s streaming: sampling_id=%u period=%lld ns\n",
		 s->indio->name, sid, period_ns);
	return 0;
}

/* StopSampling: end the AoC stream for this sensor. */
static int usf_stop_sampling(struct usf_sensor *s)
{
	struct usf_iio *usf = s->usf;
	u32 sid;

	mutex_lock(&usf->stream_lock);
	spin_lock(&usf->sample_lock);
	sid = s->sampling_id;
	s->sampling_id = 0;		/* stop accepting samples immediately */
	spin_unlock(&usf->sample_lock);
	if (sid)
		usf_session_stop_sampling(usf->usf, s->handle, sid);
	mutex_unlock(&usf->stream_lock);

	return 0;
}

static int usf_buffer_postenable(struct iio_dev *indio_dev)
{
	return usf_start_sampling(iio_priv(indio_dev));
}

static int usf_buffer_predisable(struct iio_dev *indio_dev)
{
	return usf_stop_sampling(iio_priv(indio_dev));
}

static const struct iio_buffer_setup_ops usf_buffer_ops = {
	.postenable = usf_buffer_postenable,
	.predisable = usf_buffer_predisable,
};

/*
 * Derive the IIO scale from the sensor's reported resolution (id9). raw =
 * round(value / resolution) [res_q = resolution << USF_SCALE_Q] and SCALE =
 * resolution * unit factor. If the firmware gives no usable resolution, fall
 * back to fixed micro-units (SCALE = 1e-6 * unit factor), matching the original
 * behaviour for that sensor.
 */
static void usf_set_scale(struct usf_sensor *s, const struct usf_type_map *map,
			  u32 res_bits)
{
	u64 res_q = usf_f32_scaled(res_bits, 1, USF_SCALE_Q, NULL);
	u64 scale = usf_f32_scaled(res_bits, map->unit_nano, 0, NULL);

	if (res_q && res_q != U64_MAX && scale && scale != U64_MAX) {
		s->res_q = res_q;
		s->scale_nano = scale;
	} else {
		s->res_q = 0;			/* micro-unit fallback */
		s->scale_nano = map->unit_nano / 1000000;
	}
}

static int usf_register_sensor(struct usf_iio *usf, u32 handle,
			       const char *name,
			       const struct usf_type_map *map, u32 res_bits)
{
	struct iio_dev *indio;
	struct usf_sensor *s;
	int ret;

	if (usf->nsensors >= USF_MAX_DEV)
		return -ENOSPC;

	indio = devm_iio_device_alloc(usf->dev, sizeof(*s));
	if (!indio)
		return -ENOMEM;
	s = iio_priv(indio);
	s->usf = usf;
	s->indio = indio;
	s->handle = handle;
	s->ndata = map->ndata;
	s->samp_freq = rate > 0 ? rate : 50;
	usf_set_scale(s, map, res_bits);

	indio->name = map->iio_name;
	indio->info = &usf_iio_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = map->channels;
	indio->num_channels = map->num_channels;
	indio->available_scan_masks = map->scan_masks;

	ret = devm_iio_kfifo_buffer_setup(usf->dev, indio, &usf_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(usf->dev, indio);
	if (ret)
		return ret;

	usf->sensors[usf->nsensors++] = s;
	dev_info(usf->dev,
		 "registered %s for USF '%s' (handle 0x%x) scale=%llu.%09llu%s\n",
		 map->iio_name, name, handle, s->scale_nano / USF_NANO,
		 s->scale_nano % USF_NANO, s->res_q ? "" : " (micro fallback)");
	return 0;
}

/*
 * Enumerate the AoC sensors and register an IIO device for each supported one
 * (optionally filtered by the sensor= module param).
 */
static void usf_enumerate(struct usf_iio *usf)
{
	u32 handles[USF_LIST_MAX];
	char name[USF_NAME_MAX];
	bool filtered = sensor && sensor[0];
	int count, i, m;
	u32 res_bits;

	count = usf_session_sensor_list(usf->usf, handles, USF_LIST_MAX);
	if (count <= 0) {
		dev_warn(usf->dev, "GetSensorList returned %d\n", count);
		return;
	}
	dev_info(usf->dev, "enumerated %d USF sensors\n", count);

	for (i = 0; i < count; i++) {
		if (usf_session_sensor_info(usf->usf, handles[i], name,
					    sizeof(name), &res_bits))
			continue;
		dev_info(usf->dev, "  handle 0x%02x : %s\n", handles[i], name);

		if (filtered && !usf_name_has(name, sensor))
			continue;
		for (m = 0; m < (int)ARRAY_SIZE(usf_type_maps); m++) {
			if (usf_name_has(name, usf_type_maps[m].match)) {
				usf_register_sensor(usf, handles[i], name,
						    &usf_type_maps[m], res_bits);
				break;
			}
		}
	}
	if (!usf->nsensors)
		dev_warn(usf->dev, "no registerable sensors%s%s\n",
			 filtered ? " matching " : "", filtered ? sensor : "");
}

/* Reschedule the bootstrap if we have retries left; otherwise give up. */
static bool usf_retry(struct usf_iio *usf, const char *why)
{
	if (usf->retries++ < USF_MAX_RETRIES) {
		dev_dbg(usf->dev, "%s; retry %u/%u\n", why, usf->retries,
			USF_MAX_RETRIES);
		schedule_delayed_work(&usf->enum_work,
				      msecs_to_jiffies(USF_RETRY_MS));
		return true;
	}
	dev_warn(usf->dev, "giving up after %u retries: %s\n", usf->retries, why);
	return false;
}

static void usf_iio_enumerate_work(struct work_struct *work)
{
	struct usf_iio *usf = container_of(to_delayed_work(work),
					   struct usf_iio, enum_work);
	int ret;

	if (usf->enumerated)
		return;

	ret = usf_session_open(usf->usf);
	if (ret) {
		/* AoC / AOCC not up yet: transient, keep retrying. */
		usf_retry(usf, "AOCC channels not ready");
		return;
	}

	/*
	 * The servers stay dormant until the registry is up, so load it before
	 * bootstrapping rather than waiting for something else to do it.
	 * Re-uploading an already-loaded registry is what the retry path would
	 * do anyway, and the AoC tolerates it.
	 */
	ret = usf_registry_load(usf->usf, usf->dev->of_node);
	if (ret) {
		usf_session_close(usf->usf);
		usf_retry(usf, "sensor registry not loaded");
		return;
	}

	ret = usf_session_bootstrap(usf->usf);
	if (ret) {
		/* -EAGAIN: the servers did not come up after the registry. */
		usf_session_close(usf->usf);
		usf_retry(usf, "USF servers not responding after registry load");
		return;
	}

	usf->enumerated = true;

	usf_enumerate(usf);
}

static ssize_t enumerate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct usf_iio *usf = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	if (val && !usf->enumerated) {
		usf->retries = 0;
		schedule_delayed_work(&usf->enum_work, 0);
	}
	return count;
}
static DEVICE_ATTR_WO(enumerate);

static struct attribute *usf_iio_attrs[] = {
	&dev_attr_enumerate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(usf_iio);

/*
 * The AOC carries the USF services and the clock domain the samples are
 * stamped in, so the bridge needs its device, not just its services.
 */
static struct device *usf_iio_get_aoc(struct device *dev)
{
	struct platform_device *aoc_pdev;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "google,aoc", 0);
	if (!np) {
		dev_err(dev, "no google,aoc phandle\n");
		return ERR_PTR(-EINVAL);
	}

	aoc_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!aoc_pdev)
		return ERR_PTR(-EPROBE_DEFER);

	if (!platform_get_drvdata(aoc_pdev)) {
		put_device(&aoc_pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return &aoc_pdev->dev;
}

static int usf_iio_probe(struct platform_device *pdev)
{
	struct usf_iio *usf;

	usf = devm_kzalloc(&pdev->dev, sizeof(*usf), GFP_KERNEL);
	if (!usf)
		return -ENOMEM;

	usf->dev = &pdev->dev;

	usf->aoc_dev = usf_iio_get_aoc(&pdev->dev);
	if (IS_ERR(usf->aoc_dev))
		return PTR_ERR(usf->aoc_dev);

	usf->usf = usf_session_alloc(&pdev->dev, usf->aoc_dev, usf_iio_sample,
				     usf);
	if (IS_ERR(usf->usf)) {
		put_device(usf->aoc_dev);
		return PTR_ERR(usf->usf);
	}

	mutex_init(&usf->stream_lock);
	spin_lock_init(&usf->sample_lock);
	INIT_DELAYED_WORK(&usf->enum_work, usf_iio_enumerate_work);
	platform_set_drvdata(pdev, usf);

	if (autopoll)
		schedule_delayed_work(&usf->enum_work, 0);

	return 0;
}

static void usf_iio_remove(struct platform_device *pdev)
{
	struct usf_iio *usf = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&usf->enum_work);
	usf_session_close(usf->usf);
	put_device(usf->aoc_dev);
}

static const struct of_device_id usf_iio_of_match[] = {
	{ .compatible = "google,aoc-usf" },
	{ }
};
MODULE_DEVICE_TABLE(of, usf_iio_of_match);

static struct platform_driver usf_iio_driver = {
	.driver = {
		.name = "aoc-usf-iio",
		.of_match_table = usf_iio_of_match,
		.dev_groups = usf_iio_groups,
	},
	.probe = usf_iio_probe,
	.remove = usf_iio_remove,
};

module_platform_driver(usf_iio_driver);

MODULE_DESCRIPTION("AoC USF sensor framework to IIO bridge");
MODULE_LICENSE("GPL");
