/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * USF (AoC Unified Sensor Framework) wire-protocol codec.
 *
 * Hand-rolled FlatBuffer encoder/decoder for the handful of USF message types
 * the mainline USF drivers need. The layouts are reverse-engineered from the
 * unstripped libusf.so and validated byte-for-byte against on-device captures.
 *
 * Transport is packet/datagram: one message == one complete outer FlatBuffer,
 * little-endian, fields matched by FlatBuffer field-id (stable), not byte
 * offset (builder-assigned).
 */
#ifndef _AOC_USF_PROTO_H_
#define _AOC_USF_PROTO_H_

#include <linux/types.h>

/* Framework-stable 16-byte service UUIDs (raw bytes, not endian-swapped). */
extern const u8 usf_uuid_sensor_mgr[16];
extern const u8 usf_uuid_sample_channel[16];
extern const u8 usf_uuid_registry[16];

/* UsfMsg.body operation code (msg_id). */
enum usf_msg_id {
	USF_MSG_GET_SERVER      = 3,
	USF_MSG_GET_SENSOR_INFO = 4,
	USF_MSG_GET_SENSOR_LIST = 5,
	USF_MSG_CREATE_SAMPLING = 8,
	USF_MSG_STOP_SAMPLING   = 9,
	USF_MSG_RECONFIG        = 10,
	USF_MSG_REGISTRY_GET    = 21,
};

/* Outer envelope type (UsfMsgOuter.type). */
enum usf_env_type {
	USF_T_REQUEST  = 1,
	USF_T_RESPONSE = 2,
	USF_T_FRAGMENT = 3,
	USF_T_BATCHED  = 5,
	USF_T_SAMPLE   = 9,
};

#define USF_SERVER_MGR_HANDLE 1	/* fixed dst_handle for GetServer */

/*
 * CreateSampling fid0. The firmware validates this: 0 and 4 are rejected (the
 * response carries sampling_id 0). It also decides delivery -- a
 * USF_MODE_WAKE_GESTURE stream arrives on the wake service and can resume a
 * suspended AP, while the streaming modes arrive on com.google.usf.non_wake_up,
 * whose mailbox doorbell is masked across suspend.
 */
enum usf_sampling_mode {
	USF_MODE_PHYSICAL     = 1,	/* on-chip sensor (accel, gyro, ...) */
	USF_MODE_FUSED        = 2,	/* AoC-fused / VSC sensor */
	USF_MODE_WAKE_GESTURE = 3,	/* wake-up gesture (tap, lift-to-wake) */
};

/*
 * Builder scratch. Control messages are tiny; 512 B per region is ample. A
 * single fbb is reused across calls under the caller's serialization. buf holds
 * the message under construction; scratch_body/scratch_inner hold the two
 * intermediate copies needed to nest body -> UsfMsg -> outer envelope.
 */
#define USF_FBB_CAP 512

struct usf_fbb {
	u8 buf[USF_FBB_CAP];
	u8 scratch_body[USF_FBB_CAP];
	u8 scratch_inner[USF_FBB_CAP];
	size_t used;
	u16 field_loc[16];
	int max_field;
	u32 table_start_used;
	bool overflow;
};

/*
 * Request builders. Each encodes a complete outer FlatBuffer into @b and
 * returns a pointer/length INTO b->buf (valid until @b is next used). Return 0
 * on success or -EOVERFLOW.
 */
int usf_build_get_server(struct usf_fbb *b, u32 txn, const u8 uuid[16],
			 const u8 **out, size_t *out_len);
int usf_build_no_body(struct usf_fbb *b, u32 msg_id, u32 txn, u32 dst,
		      const u8 **out, size_t *out_len);
int usf_build_create_sampling(struct usf_fbb *b, u32 txn, u32 sensor_handle,
			      enum usf_sampling_mode mode, s64 period_ns,
			      u32 client_id, const u8 **out, size_t *out_len);
int usf_build_reconfig(struct usf_fbb *b, u32 txn, u32 sensor_handle,
		       u32 sampling_id, s64 period_ns, s64 max_latency_ns,
		       bool enable, const u8 **out, size_t *out_len);
int usf_build_stop_sampling(struct usf_fbb *b, u32 txn, u32 sensor_handle,
			    u32 sampling_id, const u8 **out, size_t *out_len);

/* Envelope/response readers (schemaless, field-id based). */

/* Split an outer envelope into type + inner payload (a pointer into @b). */
bool usf_parse_outer(const u8 *b, size_t len, u32 *type,
		     const u8 **payload, u32 *plen);
/* txn id of a response payload (inner UsfMsg fid1), or U32_MAX on error. */
u32 usf_resp_txn(const u8 *pay, u32 plen);
/* body ([ubyte] at inner UsfMsg fid3) of a response payload, NULL if absent. */
const u8 *usf_resp_body(const u8 *pay, u32 plen, u32 *blen);

/* Field accessors on a body FlatBuffer (its root is resolved internally). */
u32 usf_fb_u32(const u8 *body, u32 blen, int fid, u32 def);
const u8 *usf_fb_vec(const u8 *body, u32 blen, int fid, u32 *count);
/* Copy a string field into @out (NUL-terminated); returns length or -1. */
int usf_fb_string(const u8 *body, u32 blen, int fid, char *out, size_t outsz);

/* ---- Compact sample batch (outer type-9 payload) --------------------- */

struct usf_sample_hdr {		/* 16 bytes */
	u32 tag;		/* +0x00 = 1 (compact-small) */
	u32 client_id;		/* +0x04 echoes CreateSampling client_id */
	u32 sampling_id;	/* +0x08 echoes CreateSampling sampling_id */
	u32 packed;		/* +0x0c: sensor_type[15:0], scount[25:16]&0x3ff, dcount[31:26] */
} __packed;

static inline u16 usf_sample_type(const struct usf_sample_hdr *h)
{
	return h->packed & 0xffff;
}
static inline u32 usf_sample_count(const struct usf_sample_hdr *h)
{
	return (h->packed >> 16) & 0x3ff;
}
static inline u32 usf_sample_dcount(const struct usf_sample_hdr *h)
{
	return (h->packed >> 26) & 0x3f;
}

/* Per-sample record: u64 ts_packed, then dcount little-endian float32 axes. */
#define USF_SAMPLE_TS_MASK	0x0fffffffffffffffULL	/* 60-bit AoC ns */

#endif /* _AOC_USF_PROTO_H_ */
