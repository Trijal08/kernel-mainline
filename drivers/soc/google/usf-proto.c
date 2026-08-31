// SPDX-License-Identifier: GPL-2.0-only
/*
 * USF (AoC Unified Sensor Framework) wire-protocol codec.
 *
 * Kernel port of the FlatBuffer encoder/decoder in tools/usf-client.c, whose
 * layouts were validated byte-for-byte against on-device HAL captures. The
 * builder emits spec-valid FlatBuffers; the AoC reader resolves fields by
 * vtable field-id, so builder-assigned voffsets need not match the HAL's.
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <linux/soc/google/usf-proto.h>

/* 16-byte service UUIDs, on the wire as raw bytes (NOT endian-swapped). */
const u8 usf_uuid_sensor_mgr[16] = {
	0x6b, 0x31, 0xcb, 0xf0, 0x74, 0x4a, 0x14, 0xa1,
	0xd4, 0xe8, 0xae, 0x8a, 0x3e, 0x09, 0xfc, 0x4d,
};
const u8 usf_uuid_sample_channel[16] = {
	0xca, 0x18, 0x5f, 0x2c, 0x64, 0x42, 0x6f, 0x95,
	0x43, 0xd2, 0x27, 0x83, 0xb3, 0x6e, 0xe1, 0x39,
};
const u8 usf_uuid_registry[16] = {
	0xd7, 0x6a, 0x14, 0xcc, 0x26, 0x4d, 0xee, 0xa1,
	0xc6, 0x40, 0x6f, 0x8b, 0x46, 0x16, 0x82, 0xc0,
};

/* ------------------------------------------------------------------ *
 * FlatBuffer builder (back-to-front, per the flatbuffers reference)   *
 * ------------------------------------------------------------------ */

static void fbb_init(struct usf_fbb *b)
{
	b->used = 0;
	memset(b->field_loc, 0, sizeof(b->field_loc));
	b->max_field = -1;
	b->table_start_used = 0;
	b->overflow = false;
}

struct usf_fbb *usf_fbb_alloc(size_t cap)
{
	struct usf_fbb *b;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return NULL;

	b->buf = kzalloc(cap, GFP_KERNEL);
	b->scratch_body = kzalloc(cap, GFP_KERNEL);
	b->scratch_inner = kzalloc(cap, GFP_KERNEL);
	if (!b->buf || !b->scratch_body || !b->scratch_inner) {
		usf_fbb_free(b);
		return NULL;
	}
	b->cap = cap;

	return b;
}

void usf_fbb_free(struct usf_fbb *b)
{
	if (!b)
		return;
	kfree(b->scratch_inner);
	kfree(b->scratch_body);
	kfree(b->buf);
	kfree(b);
}

static u8 *fbb_head(struct usf_fbb *b)
{
	return b->buf + b->cap - b->used;
}

static void fbb_pre(struct usf_fbb *b, const void *src, size_t n)
{
	if (b->overflow)
		return;
	if (b->used + n > b->cap) {
		b->overflow = true;
		return;
	}
	b->used += n;
	if (src)
		memcpy(fbb_head(b), src, n);
	else
		memset(fbb_head(b), 0, n);
}

/* Pad so the NEXT pushed object of the given size is aligned to it. */
static void fbb_align(struct usf_fbb *b, size_t size)
{
	size_t pad = (~b->used + 1) & (size - 1);

	if (pad)
		fbb_pre(b, NULL, pad);
}

static void fbb_push_u8(struct usf_fbb *b, u8 v)
{
	fbb_pre(b, &v, 1);
}

static void fbb_push_u16_raw(struct usf_fbb *b, u16 v)
{
	u8 t[2];

	put_unaligned_le16(v, t);
	fbb_pre(b, t, 2);
}

static void fbb_push_u32(struct usf_fbb *b, u32 v)
{
	u8 t[4];

	put_unaligned_le32(v, t);
	fbb_align(b, 4);
	fbb_pre(b, t, 4);
}

static void fbb_push_u64(struct usf_fbb *b, u64 v)
{
	u8 t[8];

	put_unaligned_le64(v, t);
	fbb_align(b, 8);
	fbb_pre(b, t, 8);
}

static void fbb_start_table(struct usf_fbb *b)
{
	memset(b->field_loc, 0, sizeof(b->field_loc));
	b->max_field = -1;
	b->table_start_used = (u32)b->used;
}

static void fbb_track(struct usf_fbb *b, int field_id)
{
	if (field_id >= (int)ARRAY_SIZE(b->field_loc)) {
		b->overflow = true;
		return;
	}
	b->field_loc[field_id] = (u16)b->used; /* location == used after push */
	if (field_id > b->max_field)
		b->max_field = field_id;
}

static void fbb_add_i32(struct usf_fbb *b, int field_id, u32 v)
{
	fbb_push_u32(b, v);
	fbb_track(b, field_id);
}

static void fbb_add_i64(struct usf_fbb *b, int field_id, s64 v)
{
	fbb_push_u64(b, (u64)v);
	fbb_track(b, field_id);
}

static void fbb_add_bool(struct usf_fbb *b, int field_id, u8 v)
{
	fbb_push_u8(b, v);
	fbb_track(b, field_id);
}

/*
 * uoffset value referring to an object at absolute location @off (== used at
 * that object's start), computed just before the referring u32 is pushed.
 */
static u32 fbb_refer(struct usf_fbb *b, u32 off)
{
	fbb_align(b, 4);
	return (u32)b->used - off + 4;
}

static void fbb_add_offset(struct usf_fbb *b, int field_id, u32 off)
{
	u8 t[4];
	u32 v;

	if (!off)
		return;
	v = fbb_refer(b, off);
	/* One contiguous 4-byte push: the builder grows back-to-front, so
	 * splitting into two u16 pushes would swap the halves.
	 */
	put_unaligned_le32(v, t);
	fbb_pre(b, t, 4);
	fbb_track(b, field_id);
}

/*
 * Create a [ubyte] vector; returns its location (used at the count word).
 * PreAlign(n,4): pad so that (used + n) is 4-aligned, i.e. after pushing the n
 * data bytes `used` is 4-aligned and the count word lands contiguous with the
 * data (no gap between count and data[0]).
 */
static u32 fbb_create_bytes(struct usf_fbb *b, const void *data, u32 n)
{
	size_t pad = (~(b->used + n) + 1) & 3;

	if (pad)
		fbb_pre(b, NULL, pad);
	fbb_pre(b, data, n);	/* data[0] ends up at the lowest address */
	fbb_push_u32(b, n);	/* count precedes the data, now 4-aligned */
	return (u32)b->used;
}


/*
 * Create a vector of table offsets; returns its location.  Elements are pushed
 * last-to-first so element 0 lands at the lowest address, and each uoffset is
 * resolved against its own position, as fbb_refer() does for a scalar field.
 */
static u32 fbb_create_offset_vector(struct usf_fbb *b, const u32 *offs, u32 n)
{
	u32 i;

	for (i = n; i > 0; i--) {
		u8 t[4];
		u32 v = fbb_refer(b, offs[i - 1]);

		put_unaligned_le32(v, t);
		fbb_pre(b, t, 4);
	}
	fbb_push_u32(b, n);

	return (u32)b->used;
}

/*
 * Finish the current table; returns its location (used at the soffset word).
 * soffset = vtable_loc - table_loc (positive: vtable at higher `used`). Field
 * voffsets point forward from the table start: voffset = table_loc - field_loc.
 */
static u32 fbb_end_table(struct usf_fbb *b)
{
	int nfields, fid;
	u32 table_loc, vtable_loc;
	u16 vt_len, tbl_size;
	s32 soffset;

	fbb_align(b, 4);
	fbb_pre(b, NULL, 4);
	table_loc = (u32)b->used;

	nfields = b->max_field + 1;
	if (nfields < 0)
		nfields = 0;
	vt_len = (u16)(4 + nfields * 2);
	tbl_size = (u16)(table_loc - b->table_start_used);

	/* vtable body: voffset per field-id, high id first (pushed back-to-front) */
	for (fid = nfields - 1; fid >= 0; fid--) {
		u16 voff = b->field_loc[fid] ?
			(u16)(table_loc - b->field_loc[fid]) : 0;
		fbb_push_u16_raw(b, voff);
	}
	/* vtable header: [vt_len][tbl_size] */
	fbb_push_u16_raw(b, tbl_size);
	fbb_push_u16_raw(b, vt_len);
	vtable_loc = (u32)b->used;

	soffset = (s32)vtable_loc - (s32)table_loc;
	if (!b->overflow)
		put_unaligned_le32((u32)soffset, b->buf + b->cap - table_loc);
	return table_loc;
}

/* Finish the buffer with @root table; returns pointer/len of the message. */
static void fbb_finish(struct usf_fbb *b, u32 root, const u8 **out, size_t *len)
{
	u8 t[4];
	u32 v;

	fbb_align(b, 4);
	v = fbb_refer(b, root);
	put_unaligned_le32(v, t);
	fbb_pre(b, t, 4);
	*out = fbb_head(b);
	*len = b->used;
}

/* Wrap a UsfMsg + outer envelope around an already-encoded body ([ubyte]). */
static int build_request(struct usf_fbb *b, u32 msg_id, u32 txn, u32 dst,
			 const u8 *body, size_t body_len,
			 const u8 **out, size_t *out_len)
{
	const u8 *inner;
	size_t inner_len;
	u32 body_vec = 0, usfmsg, pay, outer;

	/* 1) UsfMsg table wrapping the caller's body bytes. */
	fbb_init(b);
	if (body && body_len)
		body_vec = fbb_create_bytes(b, body, (u32)body_len);
	fbb_start_table(b);
	fbb_add_i32(b, 0, msg_id);	/* fid0 msg_id */
	fbb_add_i32(b, 1, txn);		/* fid1 txn_id */
	fbb_add_i32(b, 2, dst);		/* fid2 dst_handle */
	if (body_vec)
		fbb_add_offset(b, 3, body_vec); /* fid3 body */
	usfmsg = fbb_end_table(b);
	fbb_finish(b, usfmsg, &inner, &inner_len);
	if (b->overflow || inner_len > b->cap)
		return -EOVERFLOW;

	/* 2) Outer envelope around a copy of the UsfMsg bytes. */
	memcpy(b->scratch_inner, inner, inner_len);
	fbb_init(b);
	pay = fbb_create_bytes(b, b->scratch_inner, (u32)inner_len);
	fbb_start_table(b);
	fbb_add_i32(b, 0, USF_T_REQUEST);	/* fid0 type */
	fbb_add_offset(b, 1, pay);		/* fid1 payload */
	outer = fbb_end_table(b);
	fbb_finish(b, outer, out, out_len);
	return b->overflow ? -EOVERFLOW : 0;
}

int usf_build_get_server(struct usf_fbb *b, u32 txn, const u8 uuid[16],
			 const u8 **out, size_t *out_len)
{
	const u8 *body;
	size_t blen;
	u32 uv, bt;

	fbb_init(b);
	uv = fbb_create_bytes(b, uuid, 16);
	fbb_start_table(b);
	fbb_add_offset(b, 0, uv);	/* fid0 uuid:[ubyte] */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;
	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_GET_SERVER, txn, USF_SERVER_MGR_HANDLE,
			     b->scratch_body, blen, out, out_len);
}

int usf_build_no_body(struct usf_fbb *b, u32 msg_id, u32 txn, u32 dst,
		      const u8 **out, size_t *out_len)
{
	return build_request(b, msg_id, txn, dst, NULL, 0, out, out_len);
}

int usf_build_create_sampling(struct usf_fbb *b, u32 txn, u32 sensor_handle,
			      enum usf_sampling_mode mode, s64 period_ns,
			      u32 client_id, const u8 **out, size_t *out_len)
{
	const u8 *body;
	size_t blen;
	u32 bt;

	fbb_init(b);
	fbb_start_table(b);
	fbb_add_i32(b, 0, mode);			/* fid0 sampling mode */
	fbb_add_i64(b, 1, period_ns);		/* fid1 period_ns:int64 */
	/*
	 * fid5 accompanies the streaming modes only. The firmware routes a
	 * USF_MODE_WAKE_GESTURE stream to the wake service (com.google.usf)
	 * instead of com.google.usf.non_wake_up, whose doorbell is masked while
	 * the AP is suspended -- that routing is what makes a gesture able to
	 * wake the AP at all.
	 */
	if (mode != USF_MODE_WAKE_GESTURE)
		fbb_add_bool(b, 5, 1);		/* fid5 = 1 */
	fbb_add_i32(b, 10, client_id);		/* fid10 client_id */
	fbb_add_bool(b, 11, 1);			/* fid11 = 1 */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;
	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_CREATE_SAMPLING, txn, sensor_handle,
			     b->scratch_body, blen, out, out_len);
}

int usf_build_reconfig(struct usf_fbb *b, u32 txn, u32 sensor_handle,
		       u32 sampling_id, s64 period_ns, s64 max_latency_ns,
		       bool enable, const u8 **out, size_t *out_len)
{
	const u8 *body;
	size_t blen;
	u32 bt;

	fbb_init(b);
	fbb_start_table(b);
	fbb_add_i32(b, 0, sampling_id);		/* fid0 sampling_id */
	fbb_add_i64(b, 1, period_ns);		/* fid1 period_ns */
	fbb_add_i64(b, 2, max_latency_ns);	/* fid2 max_latency_ns */
	fbb_add_bool(b, 3, enable ? 1 : 0);	/* fid3 enable */
	fbb_add_bool(b, 4, 1);			/* fid4 present */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;
	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_RECONFIG, txn, sensor_handle,
			     b->scratch_body, blen, out, out_len);
}

int usf_build_stop_sampling(struct usf_fbb *b, u32 txn, u32 sensor_handle,
			    u32 sampling_id, const u8 **out, size_t *out_len)
{
	const u8 *body;
	size_t blen;
	u32 bt;

	fbb_init(b);
	fbb_start_table(b);
	fbb_add_i32(b, 0, sampling_id);		/* fid0 sampling_id */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;
	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_STOP_SAMPLING, txn, sensor_handle,
			     b->scratch_body, blen, out, out_len);
}

/* ------------------------------------------------------------------ *
 * FlatBuffer reader (schemaless, field-id based)                      *
 * ------------------------------------------------------------------ */


int usf_build_load_script(struct usf_fbb *b, u32 txn, u32 reg_handle,
			  const u8 *script, size_t script_len, u32 cdt,
			  const u8 **out, size_t *out_len)
{
	const u8 *body;
	size_t blen;
	u32 sv, bt;

	fbb_init(b);
	sv = fbb_create_bytes(b, script, (u32)script_len);
	fbb_start_table(b);
	fbb_add_offset(b, 0, sv);	/* fid0 script:[ubyte] */
	fbb_add_i32(b, 1, cdt);		/* fid1 cdt:uint */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;

	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_REGISTRY_LOAD_SCRIPT, txn, reg_handle,
			     b->scratch_body, blen, out, out_len);
}

int usf_build_registry_set(struct usf_fbb *b, u32 txn, u32 reg_handle,
			   const u8 *path, size_t path_len,
			   const struct usf_prop *props, unsigned int nprops,
			   const u8 **out, size_t *out_len)
{
	u32 prop_off[USF_MAX_PROPS];
	const u8 *body;
	size_t blen;
	u32 pathv, propv, bt;
	unsigned int i;

	if (nprops > USF_MAX_PROPS)
		return -EINVAL;

	fbb_init(b);

	/* Every sub-object has to be complete before the parent table opens. */
	for (i = 0; i < nprops; i++) {
		u32 nv, vv;

		nv = fbb_create_bytes(b, props[i].name,
				      (u32)props[i].name_len);
		vv = fbb_create_bytes(b, props[i].value,
				      (u32)props[i].value_len);
		fbb_start_table(b);
		fbb_add_offset(b, 0, nv);	/* fid0 name:[ubyte] */
		fbb_add_offset(b, 1, vv);	/* fid1 value:[ubyte] */
		prop_off[i] = fbb_end_table(b);
	}
	propv = fbb_create_offset_vector(b, prop_off, nprops);
	pathv = fbb_create_bytes(b, path, (u32)path_len);

	fbb_start_table(b);
	fbb_add_offset(b, 0, pathv);	/* fid0 path:[ubyte] */
	fbb_add_offset(b, 1, propv);	/* fid1 props:[Prop] */
	bt = fbb_end_table(b);
	fbb_finish(b, bt, &body, &blen);
	if (b->overflow || blen > b->cap)
		return -EOVERFLOW;

	memcpy(b->scratch_body, body, blen);
	return build_request(b, USF_MSG_REGISTRY_SET, txn, reg_handle,
			     b->scratch_body, blen, out, out_len);
}

int usf_build_fragment(struct usf_fbb *b, u32 total_len, u32 offset,
		       const u8 *chunk, size_t chunk_len,
		       const u8 **out, size_t *out_len)
{
	const u8 *inner;
	size_t ilen;
	u32 cv, ft, pay, outer;

	fbb_init(b);
	cv = fbb_create_bytes(b, chunk, (u32)chunk_len);
	fbb_start_table(b);
	fbb_add_i32(b, 0, USF_FRAG_MSG_ID);	/* fid0 frag_msg_id */
	fbb_add_i32(b, 1, total_len);		/* fid1 total_len */
	/*
	 * fid2 offset is omitted when zero, as a default scalar: that is what
	 * the vendor stack emits, and the first fragment on the wire has no
	 * offset field at all.
	 */
	if (offset)
		fbb_add_i32(b, 2, offset);
	fbb_add_offset(b, 3, cv);		/* fid3 chunk:[ubyte] */
	ft = fbb_end_table(b);
	fbb_finish(b, ft, &inner, &ilen);
	if (b->overflow || ilen > b->cap)
		return -EOVERFLOW;

	/* Envelope(type=3) around the fragment. */
	memcpy(b->scratch_inner, inner, ilen);
	fbb_init(b);
	pay = fbb_create_bytes(b, b->scratch_inner, (u32)ilen);
	fbb_start_table(b);
	fbb_add_i32(b, 0, USF_T_FRAGMENT);	/* fid0 type */
	fbb_add_offset(b, 1, pay);		/* fid1 payload */
	outer = fbb_end_table(b);
	fbb_finish(b, outer, out, out_len);

	return b->overflow ? -EOVERFLOW : 0;
}

static u32 rd32(const u8 *b, size_t o) { return get_unaligned_le32(b + o); }
static u16 rd16(const u8 *b, size_t o) { return get_unaligned_le16(b + o); }
static s32 rdi32(const u8 *b, size_t o) { return (s32)get_unaligned_le32(b + o); }

/* Absolute offset of field @fid in the table at @tpos, or 0 if absent. */
static size_t fb_field(const u8 *b, size_t len, size_t tpos, int fid)
{
	s32 soff;
	long vt;
	u16 vt_len, voff;
	int nf;
	size_t fpos;

	if (tpos + 4 > len)
		return 0;
	soff = rdi32(b, tpos);
	vt = (long)tpos - soff;
	if (vt < 0 || (size_t)vt + 4 > len)
		return 0;
	vt_len = rd16(b, vt);
	nf = (vt_len - 4) / 2;
	if (fid >= nf)
		return 0;
	if ((size_t)vt + 4 + fid * 2 + 2 > len)
		return 0;
	voff = rd16(b, vt + 4 + fid * 2);
	if (!voff)
		return 0;
	fpos = tpos + voff;
	return fpos < len ? fpos : 0;
}

static bool fb_root(const u8 *b, size_t len, size_t *root)
{
	u32 r;

	if (len < 8)
		return false;
	r = rd32(b, 0);
	if (r == 0 || r + 4 > len)
		return false;
	*root = r;
	return true;
}

static u32 fb_u32_at(const u8 *b, size_t len, size_t tpos, int fid, u32 def)
{
	size_t p = fb_field(b, len, tpos, fid);

	return p ? rd32(b, p) : def;
}

/* Follow a uoffset field to a vector/string; returns data ptr + count. */
static const u8 *fb_vec_at(const u8 *b, size_t len, size_t tpos, int fid,
			   u32 *count)
{
	size_t p = fb_field(b, len, tpos, fid);
	size_t tgt;
	u32 n;

	if (!p)
		return NULL;
	tgt = p + rd32(b, p);
	if (tgt + 4 > len)
		return NULL;
	n = rd32(b, tgt);
	if (tgt + 4 + n > len)
		return NULL;
	*count = n;
	return b + tgt + 4;
}

bool usf_parse_outer(const u8 *b, size_t len, u32 *type,
		     const u8 **payload, u32 *plen)
{
	size_t root;
	u32 n = 0;

	if (!fb_root(b, len, &root))
		return false;
	*type = fb_u32_at(b, len, root, 0, 0);
	*payload = fb_vec_at(b, len, root, 1, &n);
	*plen = n;
	return true;
}

u32 usf_resp_txn(const u8 *pay, u32 plen)
{
	size_t proot;

	if (!fb_root(pay, plen, &proot))
		return U32_MAX;
	return fb_u32_at(pay, plen, proot, 1, U32_MAX);
}

const u8 *usf_resp_body(const u8 *pay, u32 plen, u32 *blen)
{
	size_t proot;

	*blen = 0;
	if (!fb_root(pay, plen, &proot))
		return NULL;
	return fb_vec_at(pay, plen, proot, 3, blen);
}

u32 usf_fb_u32(const u8 *body, u32 blen, int fid, u32 def)
{
	size_t broot;

	if (!fb_root(body, blen, &broot))
		return def;
	return fb_u32_at(body, blen, broot, fid, def);
}

const u8 *usf_fb_vec(const u8 *body, u32 blen, int fid, u32 *count)
{
	size_t broot;

	*count = 0;
	if (!fb_root(body, blen, &broot))
		return NULL;
	return fb_vec_at(body, blen, broot, fid, count);
}

int usf_fb_string(const u8 *body, u32 blen, int fid, char *out, size_t outsz)
{
	size_t broot;
	u32 n = 0;
	const u8 *s;

	if (!outsz)
		return -1;
	out[0] = 0;
	if (!fb_root(body, blen, &broot))
		return -1;
	s = fb_vec_at(body, blen, broot, fid, &n);
	if (!s)
		return -1;
	if (n >= outsz)
		n = outsz - 1;
	memcpy(out, s, n);
	out[n] = 0;
	return n;
}
