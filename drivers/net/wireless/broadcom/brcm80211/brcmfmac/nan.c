// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2026 Broadcom
 *
 * Neighbor Awareness Networking (Wi-Fi Aware), exposed through the cfg80211
 * NAN interface. The firmware carries the NAN engine behind the "nan" iovar,
 * which takes a batch of sub-commands; this brings the engine up and down,
 * configures it, and publishes or subscribes to services on it, turning the
 * firmware's NAN events back into cfg80211 ones.
 *
 * Every command here is confirmed accepted by a BCM4390. The discovery match
 * and follow-up events are built to the same ABI but can only be seen with a
 * second Wi-Fi Aware device in range.
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <net/cfg80211.h>

#include "core.h"
#include "debug.h"
#include "fwil.h"
#include "fwil_types.h"
#include "cfg80211.h"
#include "feature.h"
#include "nan.h"

/* The "nan" iovar takes a bcm_iov_batch_buf: a version carrying the batch
 * marker, a count of sub-commands, and the sub-commands themselves. Each
 * sub-command is an id, a length that includes its options word, the options
 * (the firmware wants its inner data 32-bit aligned), and the data.
 */
#define BRCMF_NAN_IOV_BATCH_MASK	0x8000
#define BRCMF_NAN_IOCTL_VERSION		0x2
#define BRCMF_NAN_BATCH_VERSION		(BRCMF_NAN_IOV_BATCH_MASK | \
					 BRCMF_NAN_IOCTL_VERSION)
#define BRCMF_NAN_XTLV_ALIGN32		BIT(0)

/* command id = (component << 8) | sub-command */
#define BRCMF_NAN_CMD(comp, sub)	(((comp) << 8) | (sub))

/* configuration component */
#define BRCMF_NAN_CMD_CFG_NAN_INIT	BRCMF_NAN_CMD(0x01, 0x01) /* u8 */
#define BRCMF_NAN_CMD_CFG_NAN_ENAB	BRCMF_NAN_CMD(0x01, 0x16) /* u8 */
/* election component */
#define BRCMF_NAN_CMD_ELECTION_METRICS	BRCMF_NAN_CMD(0x02, 0x02)
/* synchronization component */
#define BRCMF_NAN_CMD_SYNC_SOCIAL_CHAN	BRCMF_NAN_CMD(0x04, 0x01)

/* the default NAN discovery (social) channels */
#define BRCMF_NAN_SOCIAL_CHAN_2G	6
#define BRCMF_NAN_SOCIAL_CHAN_5G	149
/* service discovery component */
#define BRCMF_NAN_CMD_SD_PUBLISH	BRCMF_NAN_CMD(0x03, 0x02)
#define BRCMF_NAN_CMD_SD_CANCEL_PUBLISH	BRCMF_NAN_CMD(0x03, 0x04)
#define BRCMF_NAN_CMD_SD_SUBSCRIBE	BRCMF_NAN_CMD(0x03, 0x05)
#define BRCMF_NAN_CMD_SD_CANCEL_SUBSCRIBE BRCMF_NAN_CMD(0x03, 0x07)

/* SD service-info TLV, appended to a publish/subscribe */
#define BRCMF_NAN_XTLV_SD_SVC_INFO	BRCMF_NAN_CMD(0x03, 0x01)

/* sd_params flags: a publish must declare its transmission type */
#define BRCMF_NAN_PUB_UNSOLICIT		0x1000
#define BRCMF_NAN_PUB_SOLICIT		0x2000
#define BRCMF_NAN_PUB_BCAST		0x4000

/* NAN event sub-types, carried in the event's reason field */
#define BRCMF_NAN_EVENT_DISCOVERY_RESULT	5
#define BRCMF_NAN_EVENT_TERMINATED		7
#define BRCMF_NAN_EVENT_RECEIVE			8

/* the service id is the first six bytes of the SHA-256 of the service name,
 * already hashed by userspace
 */
#define BRCMF_NAN_SVC_HASH_LEN		6
/* run a function until it is cancelled */
#define BRCMF_NAN_TTL_UNTIL_CANCEL	(-1)
/* a config or function batch stays well inside this */
#define BRCMF_NAN_BUF_SIZE		512

struct brcmf_nan_batch_buf_le {
	__le16 version;
	u8 count;
	u8 is_set;
	u8 cmds[];
};

struct brcmf_nan_subcmd_le {
	__le16 id;
	__le16 len;	/* options word + data */
	__le32 options;
	u8 data[];
};

/* a bare BCM xTLV: id, length, data. This is the carrier for the fields
 * nested inside a sub-command's data (e.g. a publish's service info), where
 * there is no options word -- unlike the outer sub-command above.
 */
struct brcmf_nan_xtlv_le {
	__le16 id;
	__le16 len;
	u8 data[];
};

/**
 * struct brcmf_nan_sd_params_le - publish/subscribe parameters
 * @length: length including the optional TLVs
 * @period: unsolicited SDF transmit period, in discovery windows
 * @awake_dw: discovery windows between active ones
 * @svc_hash: service id, the hashed service name
 * @instance_id: this function's instance id
 * @proximity_rssi: RSSI gate, zero for none
 * @flags: publish/subscribe option bits
 * @ttl: lifetime in seconds, negative to run until cancelled
 * @optional: appended TLVs, e.g. the service-specific info
 */
struct brcmf_nan_sd_params_le {
	__le16 length;
	u8 period;
	u8 awake_dw;
	u8 svc_hash[BRCMF_NAN_SVC_HASH_LEN];
	u8 instance_id;
	s8 proximity_rssi;
	__le32 flags;
	__le32 ttl;
	u8 optional[];
};

/* WL_NAN_CMD_ELECTION_METRICS_CONFIG payload: the firmware wants the whole
 * four-byte structure, in this field order. A short one is the wrong length
 * and the firmware refuses the entire batch it rides in.
 */
struct brcmf_nan_election_metric_le {
	u8 random_factor;
	u8 master_pref;
	u8 pad[2];
};

/* WL_NAN_CMD_SYNC_SOCIAL_CHAN payload: the discovery channels. A zero 5 GHz
 * channel leaves discovery on 2.4 GHz only.
 */
struct brcmf_nan_social_chan_le {
	u8 chan_2g;
	u8 chan_5g;
	__le16 pad;
};

/* a discovery-result event: a subscriber matched a publisher */
struct brcmf_nan_ev_disc_result_le {
	u8 pub_id;
	u8 sub_id;
	u8 pub_mac[ETH_ALEN];
	u8 opt_tlvs[];
};

/* a terminated event: a function ended */
struct brcmf_nan_ev_terminated_le {
	u8 instance_id;
	u8 reason;
	u8 svctype;	/* 0 publish, 1 subscribe */
	u8 pad;
	__le32 tx_cnt;
};

/**
 * struct brcmf_nan_func - a service function the host has added
 *
 * @list: links into &brcmf_nan.funcs
 * @cfg_func: the cfg80211 function, owned here until it ends
 * @instance_id: the firmware instance id, also cfg_func->instance_id
 */
struct brcmf_nan_func {
	struct list_head list;
	struct cfg80211_nan_func *cfg_func;
	u8 instance_id;
};

/* a batch being assembled */
struct brcmf_nan_buf {
	u8 data[BRCMF_NAN_BUF_SIZE];
	u16 used;	/* bytes populated after the header */
	u8 count;	/* sub-commands added */
};

static void brcmf_nan_buf_init(struct brcmf_nan_buf *b)
{
	struct brcmf_nan_batch_buf_le *bb = (void *)b->data;

	memset(b, 0, sizeof(*b));
	bb->version = cpu_to_le16(BRCMF_NAN_BATCH_VERSION);
}

/* append one sub-command, its data aligned behind the options word */
static int brcmf_nan_buf_add(struct brcmf_nan_buf *b, u16 id,
			     const void *data, u16 datalen)
{
	struct brcmf_nan_batch_buf_le *bb = (void *)b->data;
	struct brcmf_nan_subcmd_le *sub;
	u16 off = sizeof(*bb) + b->used;
	u16 sublen = sizeof(*sub) + datalen;

	if (off + round_up(sublen, 4) > BRCMF_NAN_BUF_SIZE)
		return -ENOSPC;

	sub = (void *)(b->data + off);
	sub->id = cpu_to_le16(id);
	/* the firmware counts the options word in the length */
	sub->len = cpu_to_le16(sizeof(sub->options) + datalen);
	sub->options = cpu_to_le32(BRCMF_NAN_XTLV_ALIGN32);
	if (datalen)
		memcpy(sub->data, data, datalen);

	b->used += round_up(sublen, 4);
	bb->count = ++b->count;
	return 0;
}

static int brcmf_nan_buf_set(struct brcmf_if *ifp, struct brcmf_nan_buf *b)
{
	struct brcmf_nan_batch_buf_le *bb = (void *)b->data;

	bb->is_set = 1;
	return brcmf_fil_iovar_data_set(ifp, "nan", b->data,
					sizeof(*bb) + b->used);
}

/* a single-byte config sub-command is the common case */
static int brcmf_nan_set_u8(struct brcmf_if *ifp, u16 id, u8 val)
{
	struct brcmf_nan_buf b;

	brcmf_nan_buf_init(&b);
	brcmf_nan_buf_add(&b, id, &val, sizeof(val));
	return brcmf_nan_buf_set(ifp, &b);
}

static int brcmf_nan_configure(struct brcmf_if *ifp,
			       struct cfg80211_nan_conf *conf)
{
	struct brcmf_nan_election_metric_le metric = {};
	struct brcmf_nan_buf b;

	brcmf_nan_buf_init(&b);

	/* the master preference, with a zero random factor, as one election
	 * metrics config
	 */
	metric.master_pref = conf->master_pref;
	brcmf_nan_buf_add(&b, BRCMF_NAN_CMD_ELECTION_METRICS, &metric,
			  sizeof(metric));

	/* Discovery stays on the 2.4 GHz social channel by default, which the
	 * firmware takes care of; only when a 5/6 GHz band is asked for do we
	 * add the 5 GHz social channel. (The firmware has no plain band
	 * selector iovar -- the social-channel set is how the band is chosen.)
	 */
	if (conf->bands & (BIT(NL80211_BAND_5GHZ) | BIT(NL80211_BAND_6GHZ))) {
		struct brcmf_nan_social_chan_le chans = {};

		chans.chan_2g = BRCMF_NAN_SOCIAL_CHAN_2G;
		chans.chan_5g = BRCMF_NAN_SOCIAL_CHAN_5G;
		brcmf_nan_buf_add(&b, BRCMF_NAN_CMD_SYNC_SOCIAL_CHAN, &chans,
				  sizeof(chans));
	}

	return brcmf_nan_buf_set(ifp, &b);
}

int brcmf_nan_start(struct wiphy *wiphy, struct wireless_dev *wdev,
		    struct cfg80211_nan_conf *conf)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = cfg->nan.vif->ifp;
	struct brcmf_pub *drvr = cfg->pub;
	int err;

	err = brcmf_nan_set_u8(ifp, BRCMF_NAN_CMD_CFG_NAN_INIT, 1);
	if (err) {
		bphy_err(drvr, "NAN init failed: %d\n", err);
		return err;
	}

	err = brcmf_nan_configure(ifp, conf);
	if (err) {
		bphy_err(drvr, "NAN config failed: %d\n", err);
		goto deinit;
	}

	err = brcmf_nan_set_u8(ifp, BRCMF_NAN_CMD_CFG_NAN_ENAB, 1);
	if (err) {
		bphy_err(drvr, "NAN enable failed: %d\n", err);
		goto deinit;
	}

	cfg->nan.enabled = true;
	return 0;

deinit:
	brcmf_nan_set_u8(ifp, BRCMF_NAN_CMD_CFG_NAN_INIT, 0);
	return err;
}

/* drop a tracked function, freeing the cfg80211 object it owns */
static void brcmf_nan_free_func(struct brcmf_nan_func *func)
{
	list_del(&func->list);
	cfg80211_free_nan_func(func->cfg_func);
	kfree(func);
}

void brcmf_nan_stop(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = cfg->nan.vif->ifp;
	struct brcmf_nan_func *func, *tmp;

	list_for_each_entry_safe(func, tmp, &cfg->nan.funcs, list)
		brcmf_nan_free_func(func);

	brcmf_nan_set_u8(ifp, BRCMF_NAN_CMD_CFG_NAN_ENAB, 0);
	brcmf_nan_set_u8(ifp, BRCMF_NAN_CMD_CFG_NAN_INIT, 0);
	cfg->nan.enabled = false;
}

int brcmf_nan_change_conf(struct wiphy *wiphy, struct wireless_dev *wdev,
			  struct cfg80211_nan_conf *conf, u32 changes)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = cfg->nan.vif->ifp;

	/* both the master preference and the bands are carried in the one
	 * config this builds, so a change to either re-sends it
	 */
	if (changes & (CFG80211_NAN_CONF_CHANGED_PREF |
		       CFG80211_NAN_CONF_CHANGED_BANDS))
		return brcmf_nan_configure(ifp, conf);

	return 0;
}

/* lowest instance id, 1..255, not already in use */
static u8 brcmf_nan_alloc_instance(struct brcmf_cfg80211_info *cfg)
{
	struct brcmf_nan_func *func;
	u8 id;

	for (id = 1; id; id++) {
		bool used = false;

		list_for_each_entry(func, &cfg->nan.funcs, list) {
			if (func->instance_id == id) {
				used = true;
				break;
			}
		}
		if (!used)
			return id;
	}
	return 0;
}

int brcmf_nan_add_func(struct wiphy *wiphy, struct wireless_dev *wdev,
		       struct cfg80211_nan_func *nan_func)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = cfg->nan.vif->ifp;
	struct brcmf_nan_sd_params_le *p;
	struct brcmf_nan_func *func;
	struct brcmf_nan_buf b;
	u8 params[sizeof(*p) + sizeof(struct brcmf_nan_xtlv_le) + 0xff];
	u32 flags = 0;
	u16 cmd, svclen = 0;
	u8 instance;

	switch (nan_func->type) {
	case NL80211_NAN_FUNC_PUBLISH:
		cmd = BRCMF_NAN_CMD_SD_PUBLISH;
		/* a publish has to say how it transmits */
		if (nan_func->publish_type & NL80211_NAN_SOLICITED_PUBLISH)
			flags |= BRCMF_NAN_PUB_SOLICIT;
		if (nan_func->publish_type & NL80211_NAN_UNSOLICITED_PUBLISH)
			flags |= BRCMF_NAN_PUB_UNSOLICIT;
		if (!flags)
			flags = BRCMF_NAN_PUB_UNSOLICIT;
		if (nan_func->publish_bcast)
			flags |= BRCMF_NAN_PUB_BCAST;
		break;
	case NL80211_NAN_FUNC_SUBSCRIBE:
		cmd = BRCMF_NAN_CMD_SD_SUBSCRIBE;
		break;
	default:
		/* a follow-up is a transmit in reply to a match, not a
		 * standing function; it is not offered here
		 */
		return -EOPNOTSUPP;
	}

	instance = brcmf_nan_alloc_instance(cfg);
	if (!instance)
		return -ENOSPC;

	func = kzalloc_obj(*func);
	if (!func)
		return -ENOMEM;

	/* the sub-command data is the fixed parameters, and where present the
	 * service-specific info as one appended TLV
	 */
	p = (struct brcmf_nan_sd_params_le *)params;
	memset(params, 0, sizeof(params));
	memcpy(p->svc_hash, nan_func->service_id, BRCMF_NAN_SVC_HASH_LEN);
	p->instance_id = instance;
	p->period = 1;
	p->flags = cpu_to_le32(flags);
	p->ttl = cpu_to_le32(nan_func->ttl ? nan_func->ttl :
			     BRCMF_NAN_TTL_UNTIL_CANCEL);

	if (nan_func->serv_spec_info_len) {
		struct brcmf_nan_xtlv_le *tlv = (void *)p->optional;

		tlv->id = cpu_to_le16(BRCMF_NAN_XTLV_SD_SVC_INFO);
		tlv->len = cpu_to_le16(nan_func->serv_spec_info_len);
		memcpy(tlv->data, nan_func->serv_spec_info,
		       nan_func->serv_spec_info_len);
		svclen = round_up(sizeof(*tlv) + nan_func->serv_spec_info_len,
				  4);
	}

	p->length = cpu_to_le16(sizeof(*p) + svclen);

	brcmf_nan_buf_init(&b);
	brcmf_nan_buf_add(&b, cmd, p, sizeof(*p) + svclen);
	if (brcmf_nan_buf_set(ifp, &b)) {
		kfree(func);
		return -EINVAL;
	}

	/* keep the function; the instance id is the driver's answer */
	nan_func->instance_id = instance;
	func->instance_id = instance;
	func->cfg_func = nan_func;
	list_add_tail(&func->list, &cfg->nan.funcs);
	return 0;
}

void brcmf_nan_del_func(struct wiphy *wiphy, struct wireless_dev *wdev,
			u64 cookie)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_if *ifp = cfg->nan.vif->ifp;
	struct brcmf_nan_func *func;
	u16 cmd;

	list_for_each_entry(func, &cfg->nan.funcs, list) {
		if (func->cfg_func->cookie != cookie)
			continue;

		cmd = func->cfg_func->type == NL80211_NAN_FUNC_PUBLISH ?
			BRCMF_NAN_CMD_SD_CANCEL_PUBLISH :
			BRCMF_NAN_CMD_SD_CANCEL_SUBSCRIBE;
		brcmf_nan_set_u8(ifp, cmd, func->instance_id);
		brcmf_nan_free_func(func);
		return;
	}
}

/* a discovery result: report the match to userspace */
static void brcmf_nan_rx_match(struct brcmf_cfg80211_info *cfg,
			       struct wireless_dev *wdev, void *data, u32 len)
{
	struct brcmf_nan_ev_disc_result_le *ev = data;
	struct cfg80211_nan_match_params match = {};
	u32 hdr = sizeof(*ev);

	if (len < hdr)
		return;

	match.type = NL80211_NAN_FUNC_SUBSCRIBE;
	match.inst_id = ev->sub_id;
	match.peer_inst_id = ev->pub_id;
	match.addr = ev->pub_mac;
	/* any service-specific info the publisher sent trails the header */
	if (len > hdr) {
		match.info = ev->opt_tlvs;
		match.info_len = len - hdr;
	}

	cfg80211_nan_match(wdev, &match, GFP_KERNEL);
}

/* firmware termination reasons: 1 timeout, 2 host, 3 firmware, 4 failure */
static enum nl80211_nan_func_term_reason brcmf_nan_term_reason(u8 reason)
{
	switch (reason) {
	case 1:
		return NL80211_NAN_FUNC_TERM_REASON_TTL_EXPIRED;
	case 2:
		return NL80211_NAN_FUNC_TERM_REASON_USER_REQUEST;
	default:
		return NL80211_NAN_FUNC_TERM_REASON_ERROR;
	}
}

/* a terminated function: tell userspace and drop it */
static void brcmf_nan_rx_terminated(struct brcmf_cfg80211_info *cfg,
				    struct wireless_dev *wdev, void *data,
				    u32 len)
{
	struct brcmf_nan_ev_terminated_le *ev = data;
	struct brcmf_nan_func *func;

	if (len < sizeof(*ev))
		return;

	list_for_each_entry(func, &cfg->nan.funcs, list) {
		if (func->instance_id != ev->instance_id)
			continue;

		cfg80211_nan_func_terminated(wdev, ev->instance_id,
					     brcmf_nan_term_reason(ev->reason),
					     func->cfg_func->cookie,
					     GFP_KERNEL);
		brcmf_nan_free_func(func);
		return;
	}
}

int brcmf_nan_event(struct brcmf_if *ifp,
		    const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct wireless_dev *wdev;

	if (!cfg->nan.vif)
		return 0;
	wdev = &cfg->nan.vif->wdev;

	/* the NAN sub-event type rides in the event's reason field */
	switch (e->reason) {
	case BRCMF_NAN_EVENT_DISCOVERY_RESULT:
		brcmf_nan_rx_match(cfg, wdev, data, e->datalen);
		break;
	case BRCMF_NAN_EVENT_TERMINATED:
		brcmf_nan_rx_terminated(cfg, wdev, data, e->datalen);
		break;
	default:
		break;
	}
	return 0;
}

struct wireless_dev *brcmf_nan_add_vif(struct wiphy *wiphy, const char *name)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_cfg80211_vif *vif;

	if (cfg->nan.vif)
		return ERR_PTR(-EEXIST);

	vif = brcmf_alloc_vif(cfg, NL80211_IFTYPE_NAN);
	if (IS_ERR(vif))
		return (struct wireless_dev *)vif;

	/* NAN has no firmware interface of its own; it runs on the primary
	 * one, so the management wdev borrows its interface and address. The
	 * wdev itself is registered by nl80211 once this returns.
	 */
	vif->ifp = brcmf_get_ifp(cfg->pub, 0);
	memcpy(vif->wdev.address, vif->ifp->mac_addr, ETH_ALEN);
	cfg->nan.vif = vif;
	return &vif->wdev;
}

int brcmf_nan_del_vif(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct brcmf_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct brcmf_cfg80211_vif *vif = container_of(wdev,
						      struct brcmf_cfg80211_vif,
						      wdev);

	if (cfg->nan.enabled)
		brcmf_nan_stop(wiphy, wdev);

	brcmf_free_vif(vif);
	cfg->nan.vif = NULL;
	return 0;
}

void brcmf_nan_init(struct brcmf_cfg80211_info *cfg)
{
	INIT_LIST_HEAD(&cfg->nan.funcs);
}

void brcmf_nan_attach(struct brcmf_cfg80211_info *cfg, struct brcmf_if *ifp)
{
	struct wiphy *wiphy = cfg_to_wiphy(cfg);

	/* Bringing the engine up to ask it its version is a configuration the
	 * firmware only accepts once the device is up, which it is not yet
	 * here. The firmware capability list, read while the features are
	 * probed, already says whether NAN is there, so go by that.
	 */
	if (!brcmf_feat_is_enabled(ifp, BRCMF_FEAT_NAN))
		return;

	/* The NL80211_IFTYPE_NAN interface mode and its slot in the interface
	 * combination are set up together in brcmf_setup_ifmodes(); here we
	 * only advertise the bands the NAN engine operates on.
	 */
	wiphy->nan_supported_bands = BIT(NL80211_BAND_2GHZ) |
				     BIT(NL80211_BAND_5GHZ);
	brcmf_dbg(INFO, "NAN enabled\n");
}
