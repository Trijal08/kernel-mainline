/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2026 Broadcom
 *
 * Fine Timing Measurement (802.11mc) exposed to userspace as cfg80211 peer
 * measurement (NL80211_PMSR_TYPE_FTM). The wire ABI of the firmware "proxd"
 * iovar it is built on is private to ftm.c.
 */
#ifndef BRCMFMAC_FTM_H
#define BRCMFMAC_FTM_H

struct brcmf_cfg80211_info;
struct brcmf_if;
struct brcmf_event_msg;
struct cfg80211_pmsr_request;
struct wiphy;
struct wireless_dev;

/**
 * struct brcmf_ftm - in-flight peer measurement state
 *
 * @req: the cfg80211 request currently being measured, NULL when idle
 * @reported: how many peers have reported, the request completes at n_peers
 */
struct brcmf_ftm {
	struct cfg80211_pmsr_request *req;
	unsigned int reported;
};

void brcmf_ftm_attach(struct brcmf_cfg80211_info *cfg, struct brcmf_if *ifp);
int brcmf_ftm_start(struct wiphy *wiphy, struct wireless_dev *wdev,
		    struct cfg80211_pmsr_request *req);
void brcmf_ftm_abort(struct wiphy *wiphy, struct wireless_dev *wdev,
		     struct cfg80211_pmsr_request *req);
int brcmf_ftm_event(struct brcmf_if *ifp,
		    const struct brcmf_event_msg *e, void *data);

#endif /* BRCMFMAC_FTM_H */
