// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2026 Broadcom
 *
 * Fine Timing Measurement (802.11mc ranging) exposed as cfg80211 peer
 * measurement. The firmware carries the measurement engine behind the
 * "proxd" iovar; this translates a cfg80211 pmsr request into proxd
 * sessions and turns the results the firmware reports back into
 * cfg80211 pmsr results.
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/cfg80211.h>

#include "core.h"
#include "debug.h"
#include "fwil.h"
#include "fwil_types.h"
#include "cfg80211.h"
#include "xtlv.h"
#include "ftm.h"

/* proxd iovar interface version reported by a BCM4390 */
#define BRCMF_PROXD_API_VERSION		0x0300

/* wl_proxd_cmd_t */
enum brcmf_proxd_cmd {
	BRCMF_PROXD_CMD_GET_VERSION	= 1,
	BRCMF_PROXD_CMD_ENABLE		= 2,
	BRCMF_PROXD_CMD_DISABLE		= 3,
	BRCMF_PROXD_CMD_CONFIG		= 4,
	BRCMF_PROXD_CMD_START_SESSION	= 5,
	BRCMF_PROXD_CMD_BURST_REQUEST	= 6,
	BRCMF_PROXD_CMD_STOP_SESSION	= 7,
	BRCMF_PROXD_CMD_DELETE_SESSION	= 8,
	BRCMF_PROXD_CMD_GET_RESULT	= 9,
};

/* wl_proxd_method_t */
#define BRCMF_PROXD_METHOD_FTM		4

/* session flags (WL_PROXD_SESSION_FLAG_*) */
#define BRCMF_PROXD_SESSION_FLAG_INITIATOR	0x00000001

/* TLV ids (wl_proxd_tlv_types_t) used here */
enum brcmf_proxd_tlv_id {
	BRCMF_PROXD_TLV_ID_CHANSPEC	= 3,	/* u32 */
	BRCMF_PROXD_TLV_ID_BURST_NUM_FTM = 9,	/* u16, ftms per burst */
	BRCMF_PROXD_TLV_ID_NUM_BURST	= 10,	/* u16 */
	BRCMF_PROXD_TLV_ID_EVENT_MASK	= 16,	/* u32 */
	BRCMF_PROXD_TLV_ID_PEER_MAC	= 18,	/* ether_addr */
	BRCMF_PROXD_TLV_ID_SESSION_FLAGS = 25,	/* u32 */
	BRCMF_PROXD_TLV_ID_SESSION_FLAGS_MASK = 26, /* u32, which flag bits apply */
	BRCMF_PROXD_TLV_ID_CUR_ETHER_ADDR = 38,	/* ether_addr, our source */
	BRCMF_PROXD_TLV_ID_RTT_RESULT	= 515,	/* struct brcmf_proxd_rtt_result_le */
};

/* wl_proxd_event_type_t */
enum brcmf_proxd_event_type {
	BRCMF_PROXD_EVENT_BURST_END	= 5,
	BRCMF_PROXD_EVENT_SESSION_END	= 6,
};

/* firmware status, WL_FTM_E_OK is zero, errors are negative */
#define BRCMF_PROXD_E_OK		0

/* a measurement that did not produce a distance reports this */
#define BRCMF_PROXD_DIST_INVALID	0xffffffff

/**
 * struct brcmf_proxd_iov_le - header of every "proxd" iovar exchange
 * @version: BRCMF_PROXD_API_VERSION
 * @len: total length including this header and any TLVs
 * @cmd: &enum brcmf_proxd_cmd
 * @method: BRCMF_PROXD_METHOD_FTM
 * @sid: session id, zero for the global session
 * @ftm_type: 11az ftm type, zero for 11mc
 * @pad: alignment
 * @tlvs: variable-length TLVs
 */
struct brcmf_proxd_iov_le {
	__le16 version;
	__le16 len;
	__le16 cmd;
	__le16 method;
	__le16 sid;
	u8 ftm_type;
	u8 pad;
	u8 tlvs[];
};

#define BRCMF_PROXD_IOV_HDR_SIZE	offsetofend(struct brcmf_proxd_iov_le, pad)

/**
 * struct brcmf_proxd_tlv_le - one proxd TLV
 * @id: &enum brcmf_proxd_tlv_id
 * @len: length of @data
 * @data: payload
 */
struct brcmf_proxd_tlv_le {
	__le16 id;
	__le16 len;
	u8 data[];
};

/**
 * struct brcmf_proxd_event_le - header delivered with a BRCMF_E_PROXD event
 * @version: proxd api version
 * @len: total length
 * @type: &enum brcmf_proxd_event_type
 * @method: measurement method
 * @sid: session the event belongs to
 * @pad: alignment
 * @tlvs: variable-length TLVs; a result event carries a RTT_RESULT TLV
 */
struct brcmf_proxd_event_le {
	__le16 version;
	__le16 len;
	__le16 type;
	__le16 method;
	__le16 sid;
	u8 pad[2];
	u8 tlvs[];
};

/* wl_proxd_intvl_t -- a time span and its unit */
struct brcmf_proxd_intvl_le {
	__le32 intvl;
	__le16 tmu;
	u8 pad[2];
};

/**
 * struct brcmf_proxd_rtt_result_le - RTT_RESULT TLV payload (v2)
 *
 * Only the leading fields this driver reads are described; the firmware may
 * append per-frame samples after them, which are not used here.
 *
 * @version: result structure version
 * @length: length up to the samples
 * @sid: session id
 * @flags: WL_PROXD_RESULT_FLAG_*
 * @status: firmware status, BRCMF_PROXD_E_OK on success
 * @peer: address measured
 * @state: session state
 * @u: burst duration or retry hint
 * @avg_dist: average distance, 1/256 m units, 0xffffffff if invalid
 * @sd_rtt: rtt standard deviation
 * @num_valid_rtt: number of valid measurements
 * @num_ftm: configured ftm count
 * @burst_num: burst index within the session
 * @num_rtt: number of appended samples, zero without RTT detail
 */
struct brcmf_proxd_rtt_result_le {
	__le16 version;
	__le16 length;
	__le16 sid;
	__le16 flags;
	__le32 status;
	u8 peer[ETH_ALEN];
	__le16 state;
	struct brcmf_proxd_intvl_le u;
	__le32 avg_dist;
	__le16 sd_rtt;
	u8 num_valid_rtt;
	u8 num_ftm;
	__le16 burst_num;
	__le16 num_rtt;
};

/* Advertised measurement bounds. The firmware negotiates the rest with the
 * responder; these only bound what userspace may ask for.
 */
#define BRCMF_FTM_MAX_PEERS		8
#define BRCMF_FTM_MAX_BURSTS_EXP	2
#define BRCMF_FTM_MAX_FTMS_PER_BURST	5

static const struct cfg80211_pmsr_capabilities brcmf_ftm_capa = {
	.max_peers = BRCMF_FTM_MAX_PEERS,
	.ftm = {
		.supported = 1,
		.asap = 1,
		.non_asap = 1,
		.max_bursts_exponent = BRCMF_FTM_MAX_BURSTS_EXP,
		.max_ftms_per_burst = BRCMF_FTM_MAX_FTMS_PER_BURST,
		.preambles = BIT(NL80211_PREAMBLE_LEGACY) |
			     BIT(NL80211_PREAMBLE_HT) |
			     BIT(NL80211_PREAMBLE_VHT) |
			     BIT(NL80211_PREAMBLE_HE),
		.bandwidths = BIT(NL80211_CHAN_WIDTH_20) |
			      BIT(NL80211_CHAN_WIDTH_40) |
			      BIT(NL80211_CHAN_WIDTH_80) |
			      BIT(NL80211_CHAN_WIDTH_160),
	},
};

/* one proxd session per peer; session id is the peer index plus one so that
 * zero stays the global session the firmware reserves
 */
static u16 brcmf_ftm_peer_sid(unsigned int peer)
{
	return peer + 1;
}

/* append one TLV to a config buffer and advance the cursor. The firmware
 * reads these 32-bit aligned, as the vendor driver packs them, so each entry
 * is padded out to a four-byte boundary.
 */
static void brcmf_ftm_put_tlv(u8 **cursor, u16 id, u16 len, const void *data)
{
	brcmf_xtlv_pack_header((struct brcmf_xtlv *)*cursor, id, len, data,
			       BRCMF_XTLV_OPTION_ALIGN32);
	*cursor += brcmf_xtlv_data_size(len, BRCMF_XTLV_OPTION_ALIGN32);
}

/* send a proxd command carrying the already-built TLV block */
static int brcmf_ftm_cmd(struct brcmf_if *ifp, u16 cmd, u16 sid,
			 const u8 *tlvs, u16 tlvs_len)
{
	struct brcmf_proxd_iov_le *iov;
	u16 len = BRCMF_PROXD_IOV_HDR_SIZE + tlvs_len;
	int err;

	iov = kzalloc(len, GFP_KERNEL);
	if (!iov)
		return -ENOMEM;

	iov->version = cpu_to_le16(BRCMF_PROXD_API_VERSION);
	iov->len = cpu_to_le16(len);
	iov->cmd = cpu_to_le16(cmd);
	iov->method = cpu_to_le16(BRCMF_PROXD_METHOD_FTM);
	iov->sid = cpu_to_le16(sid);
	if (tlvs_len)
		memcpy(iov->tlvs, tlvs, tlvs_len);

	err = brcmf_fil_iovar_data_set(ifp, "proxd", iov, len);
	kfree(iov);
	return err;
}

/* configure and start one peer's session */
static int brcmf_ftm_start_peer(struct brcmf_cfg80211_info *cfg,
				struct brcmf_if *ifp, u16 sid,
				const struct cfg80211_pmsr_request_peer *peer)
{
	/* room for the aligned TLVs built below */
	u8 tlvs[8 * (sizeof(struct brcmf_proxd_tlv_le) + ETH_ALEN + 2)];
	u8 *cursor = tlvs;
	struct cfg80211_chan_def chandef = peer->chandef;
	__le64 flags;
	__le32 chanspec, event_mask;
	__le16 num_burst, num_ftm;
	int err;

	/* a mask says which of the session flag bits this config sets, then
	 * the flags themselves -- as the vendor driver pairs them. The session
	 * flags are a 64-bit field on this firmware generation; a 32-bit one
	 * is the wrong length and the whole config is rejected.
	 */
	flags = cpu_to_le64(BRCMF_PROXD_SESSION_FLAG_INITIATOR);
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_SESSION_FLAGS_MASK,
			  sizeof(flags), &flags);
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_SESSION_FLAGS,
			  sizeof(flags), &flags);

	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_CUR_ETHER_ADDR, ETH_ALEN,
			  ifp->mac_addr);

	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_PEER_MAC, ETH_ALEN,
			  peer->addr);

	chanspec = cpu_to_le32(chandef_to_chanspec(&cfg->d11inf, &chandef));
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_CHANSPEC,
			  sizeof(chanspec), &chanspec);

	num_burst = cpu_to_le16(1 << peer->ftm.num_bursts_exp);
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_NUM_BURST,
			  sizeof(num_burst), &num_burst);

	num_ftm = cpu_to_le16(peer->ftm.ftms_per_burst);
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_BURST_NUM_FTM,
			  sizeof(num_ftm), &num_ftm);

	/* ask to be told when each burst and the session finish */
	event_mask = cpu_to_le32(BIT(BRCMF_PROXD_EVENT_BURST_END) |
				 BIT(BRCMF_PROXD_EVENT_SESSION_END));
	brcmf_ftm_put_tlv(&cursor, BRCMF_PROXD_TLV_ID_EVENT_MASK,
			  sizeof(event_mask), &event_mask);

	err = brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_CONFIG, sid, tlvs,
			    cursor - tlvs);
	if (err)
		return err;

	return brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_START_SESSION, sid, NULL, 0);
}

int brcmf_ftm_start(struct wiphy *wiphy, struct wireless_dev *wdev,
		    struct cfg80211_pmsr_request *req)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = netdev_priv(wdev->netdev);
	struct brcmf_pub *drvr = cfg->pub;
	unsigned int i;
	int err;

	if (cfg->ftm.req)
		return -EBUSY;

	/* enable the engine before the first session, idempotent in firmware */
	err = brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_ENABLE, 0, NULL, 0);
	if (err) {
		bphy_err(drvr, "FTM enable failed: %d\n", err);
		return err;
	}

	for (i = 0; i < req->n_peers; i++) {
		err = brcmf_ftm_start_peer(cfg, ifp, brcmf_ftm_peer_sid(i),
					   &req->peers[i]);
		if (err) {
			bphy_err(drvr, "FTM peer %u start failed: %d\n", i, err);
			goto unwind;
		}
	}

	cfg->ftm.req = req;
	cfg->ftm.reported = 0;
	return 0;

unwind:
	while (i--) {
		u16 sid = brcmf_ftm_peer_sid(i);

		brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_STOP_SESSION, sid, NULL, 0);
		brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_DELETE_SESSION, sid, NULL, 0);
	}
	return err;
}

void brcmf_ftm_abort(struct wiphy *wiphy, struct wireless_dev *wdev,
		     struct cfg80211_pmsr_request *req)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = netdev_priv(wdev->netdev);
	unsigned int i;

	if (cfg->ftm.req != req)
		return;

	for (i = 0; i < req->n_peers; i++) {
		u16 sid = brcmf_ftm_peer_sid(i);

		brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_STOP_SESSION, sid, NULL, 0);
		brcmf_ftm_cmd(ifp, BRCMF_PROXD_CMD_DELETE_SESSION, sid, NULL, 0);
	}
	cfg->ftm.req = NULL;
}

/* find a TLV by id within a bounded buffer */
static const struct brcmf_proxd_tlv_le *
brcmf_ftm_find_tlv(const u8 *buf, u32 buflen, u16 id)
{
	while (buflen >= sizeof(struct brcmf_proxd_tlv_le)) {
		const struct brcmf_proxd_tlv_le *tlv = (const void *)buf;
		u16 len = le16_to_cpu(tlv->len);

		if (sizeof(*tlv) + len > buflen)
			break;
		if (le16_to_cpu(tlv->id) == id)
			return tlv;
		buf += sizeof(*tlv) + len;
		buflen -= sizeof(*tlv) + len;
	}
	return NULL;
}

/* turn one firmware RTT result into a cfg80211 pmsr result and report it */
static void brcmf_ftm_report(struct brcmf_cfg80211_info *cfg,
			     struct wireless_dev *wdev,
			     const struct brcmf_proxd_rtt_result_le *r)
{
	struct cfg80211_pmsr_result res = {};
	u32 dist = le32_to_cpu(r->avg_dist);

	memcpy(res.addr, r->peer, ETH_ALEN);
	res.host_time = ktime_get_boottime_ns();
	res.type = NL80211_PMSR_TYPE_FTM;
	res.final = 1;

	if (le32_to_cpu(r->status) != BRCMF_PROXD_E_OK || !r->num_valid_rtt) {
		res.status = NL80211_PMSR_STATUS_FAILURE;
		res.ftm.failure_reason =
			NL80211_PMSR_FTM_FAILURE_NO_RESPONSE;
	} else {
		res.status = NL80211_PMSR_STATUS_SUCCESS;
		res.ftm.burst_index = le16_to_cpu(r->burst_num);
		res.ftm.num_ftmr_attempts = r->num_ftm;
		res.ftm.num_ftmr_attempts_valid = 1;
		res.ftm.num_ftmr_successes = r->num_valid_rtt;
		res.ftm.num_ftmr_successes_valid = 1;
		/* avg_dist is 1/256 m, cfg80211 wants millimetres. A value
		 * is only reported when the firmware marked it valid.
		 */
		if (dist != BRCMF_PROXD_DIST_INVALID) {
			res.ftm.dist_avg = (s64)dist * 1000 / 256;
			res.ftm.dist_avg_valid = 1;
		}
	}

	cfg80211_pmsr_report(wdev, cfg->ftm.req, &res, GFP_KERNEL);
}

int brcmf_ftm_event(struct brcmf_if *ifp,
		    const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct wireless_dev *wdev = &ifp->vif->wdev;
	const struct brcmf_proxd_event_le *ev = data;
	const struct brcmf_proxd_tlv_le *tlv;
	const struct brcmf_proxd_rtt_result_le *rtt;
	u32 datalen = e->datalen;
	u16 type;

	if (!cfg->ftm.req)
		return 0;
	if (datalen < sizeof(*ev))
		return 0;

	type = le16_to_cpu(ev->type);
	if (type != BRCMF_PROXD_EVENT_BURST_END &&
	    type != BRCMF_PROXD_EVENT_SESSION_END)
		return 0;

	tlv = brcmf_ftm_find_tlv(ev->tlvs, datalen - sizeof(*ev),
				 BRCMF_PROXD_TLV_ID_RTT_RESULT);
	if (!tlv || le16_to_cpu(tlv->len) < sizeof(*rtt))
		return 0;

	rtt = (const struct brcmf_proxd_rtt_result_le *)tlv->data;
	brcmf_ftm_report(cfg, wdev, rtt);

	/* a result per peer completes the request */
	if (++cfg->ftm.reported >= cfg->ftm.req->n_peers) {
		cfg80211_pmsr_complete(wdev, cfg->ftm.req, GFP_KERNEL);
		cfg->ftm.req = NULL;
	}
	return 0;
}

void brcmf_ftm_attach(struct brcmf_cfg80211_info *cfg, struct brcmf_if *ifp)
{
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	u8 buf[64] = {};
	struct brcmf_proxd_iov_le *iov = (void *)buf;

	/* A version GET the firmware answers is proof the proxd engine is
	 * present. It has to be a read: the same command sent as a set is not
	 * a query and the firmware rejects it, which would hide a working
	 * engine.
	 */
	iov->version = cpu_to_le16(BRCMF_PROXD_API_VERSION);
	iov->len = cpu_to_le16(BRCMF_PROXD_IOV_HDR_SIZE);
	iov->cmd = cpu_to_le16(BRCMF_PROXD_CMD_GET_VERSION);
	iov->method = cpu_to_le16(BRCMF_PROXD_METHOD_FTM);

	if (brcmf_fil_iovar_data_get(ifp, "proxd", buf, sizeof(buf))) {
		brcmf_dbg(INFO, "no FTM: proxd version query failed\n");
		return;
	}

	wiphy->pmsr_capa = &brcmf_ftm_capa;
	brcmf_dbg(INFO, "FTM peer measurement enabled\n");
}
