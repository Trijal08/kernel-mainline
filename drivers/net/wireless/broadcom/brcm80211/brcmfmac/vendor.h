// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */

#ifndef _vendor_h_
#define _vendor_h_

#define BROADCOM_OUI	0x001018
#define GOOGLE_OUI	0x001a11

#define BRCMF_APF_SUBCMD_SET_FILTER	0x1801

enum brcmf_vndr_cmds {
	BRCMF_VNDR_CMDS_UNSPEC,
	BRCMF_VNDR_CMDS_DCMD,
	BRCMF_VNDR_CMDS_LAST
};

/**
 * enum brcmf_nlattrs - nl80211 message attributes
 *
 * @BRCMF_NLATTR_LEN: message body length
 * @BRCMF_NLATTR_DATA: message body
 */
enum brcmf_nlattrs {
	BRCMF_NLATTR_UNSPEC,

	BRCMF_NLATTR_LEN,
	BRCMF_NLATTR_DATA,

	__BRCMF_NLATTR_AFTER_LAST,
	BRCMF_NLATTR_MAX = __BRCMF_NLATTR_AFTER_LAST - 1
};

/* Android APF vendor attributes. */
enum brcmf_apf_attrs {
	BRCMF_APF_ATTR_VERSION,
	BRCMF_APF_ATTR_MAX_LEN,
	BRCMF_APF_ATTR_PROGRAM,
	BRCMF_APF_ATTR_PROGRAM_LEN,

	__BRCMF_APF_ATTR_AFTER_LAST,
	BRCMF_APF_ATTR_MAX = __BRCMF_APF_ATTR_AFTER_LAST - 1
};

/**
 * struct brcmf_vndr_dcmd_hdr - message header for cfg80211 vendor command dcmd
 *				support
 *
 * @cmd: common dongle cmd definition
 * @len: length of expecting return buffer
 * @offset: offset of data buffer
 * @set: get or set request(optional)
 * @magic: magic number for verification
 */
struct brcmf_vndr_dcmd_hdr {
	uint cmd;
	int len;
	uint offset;
	uint set;
	uint magic;
};

extern const struct wiphy_vendor_command brcmf_vendor_cmds[];
extern const unsigned int brcmf_vendor_cmds_count;

#endif /* _vendor_h_ */
