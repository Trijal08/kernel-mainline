/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2026 Broadcom
 *
 * Neighbor Awareness Networking (Wi-Fi Aware), exposed through the cfg80211
 * NAN interface. The firmware "nan" iovar ABI it is built on is private to
 * nan.c.
 */
#ifndef BRCMFMAC_NAN_H
#define BRCMFMAC_NAN_H

#include <linux/list.h>

struct brcmf_cfg80211_info;
struct brcmf_cfg80211_vif;
struct brcmf_if;
struct brcmf_event_msg;
struct cfg80211_nan_conf;
struct cfg80211_nan_func;
struct wiphy;
struct wireless_dev;

/**
 * struct brcmf_nan - NAN interface state
 *
 * @vif: the NAN management virtual interface, NULL when there is none
 * @funcs: the service functions currently added, &struct brcmf_nan_func
 * @enabled: the firmware NAN engine is up
 */
struct brcmf_nan {
	struct brcmf_cfg80211_vif *vif;
	struct list_head funcs;
	bool enabled;
};

void brcmf_nan_attach(struct brcmf_cfg80211_info *cfg, struct brcmf_if *ifp);
void brcmf_nan_init(struct brcmf_cfg80211_info *cfg);
struct wireless_dev *brcmf_nan_add_vif(struct wiphy *wiphy, const char *name);
int brcmf_nan_del_vif(struct wiphy *wiphy, struct wireless_dev *wdev);
int brcmf_nan_start(struct wiphy *wiphy, struct wireless_dev *wdev,
		    struct cfg80211_nan_conf *conf);
void brcmf_nan_stop(struct wiphy *wiphy, struct wireless_dev *wdev);
int brcmf_nan_change_conf(struct wiphy *wiphy, struct wireless_dev *wdev,
			  struct cfg80211_nan_conf *conf, u32 changes);
int brcmf_nan_add_func(struct wiphy *wiphy, struct wireless_dev *wdev,
		       struct cfg80211_nan_func *nan_func);
void brcmf_nan_del_func(struct wiphy *wiphy, struct wireless_dev *wdev,
			u64 cookie);
int brcmf_nan_event(struct brcmf_if *ifp,
		    const struct brcmf_event_msg *e, void *data);

#endif /* BRCMFMAC_NAN_H */
