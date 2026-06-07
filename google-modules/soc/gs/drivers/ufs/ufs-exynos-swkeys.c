// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Exynos FMP (Flash Memory Protector) UFS crypto support
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC
 *
 * Authors: Boojin Kim <boojin.kim@samsung.com>
 *	    Eric Biggers <ebiggers@google.com>
 *
 * This is the support for using FMP in the software keys mode
 * (also called the traditional FMP mode or legacy FMP mode).
 */

#include <linux/unaligned.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <core/ufshcd-crypto.h>
#include "ufs-exynos-gs.h"
#include "ufs-pixel-crypto.h"
#include "ufs-pixel-fips.h"

enum fmp_crypto_algo_mode {
	FMP_BYPASS_MODE = 0,
	FMP_ALGO_MODE_AES_CBC = 1,
	FMP_ALGO_MODE_AES_XTS = 2,
};

#define FMP_DATA_UNIT_SIZE 4096

struct fmp_sg_entry {
	/* The first four fields correspond to those of ufshcd_sg_entry. */
	__le32 des0;
	__le32 des1;
	__le32 des2;
	/*
	 * The algorithm and key length are configured in the high bits of des3,
	 * whose low bits already contain ufshcd_sg_entry::size.
	 */
	__le32 des3;
#define FKL			(1 << 26)
#define SET_KEYLEN(ent, v)	((ent)->des3 |= cpu_to_le32(v))
#define SET_FAS(ent, v)		((ent)->des3 |= cpu_to_le32((v) << 28))

	/* The IV with all bytes reversed */
	__be32 file_iv[4];

	/*
	 * The key with all bytes reversed.  For XTS, the two halves of the key
	 * are given separately and are byte-reversed separately.
	 */
	__be32 file_enckey[8];
	__be32 file_twkey[8];

	/* Not used */
	__be32 disk_iv[4];
	__le32 reserved[4];
};

int exynos_ufs_crypto_init_sw_keys_mode(struct ufs_hba *hba)
{
	long ret;

	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			ret);
		return 0;
	}

	ret = exynos_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			ret);
		return 0;
	}

	/*
	 * TODO(mainline): software-keys FMP inline encryption was wired via the
	 * Android UFS vendor hooks android_vh_ufs_fill_prdt (per-request PRDT
	 * key/IV programming) and android_rvh_ufs_complete_init (FIPS CMVP self
	 * test).  Neither hook exists in mainline.  The PRDT fill needs to be
	 * reimplemented on top of ufshcd crypto ops / blk-crypto (KEYS_IN_PRDT)
	 * before sw-keys inline encryption can be re-enabled.
	 */

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;
	ufshcd_set_sg_entry_size(hba, sizeof(struct fmp_sg_entry));

	/* Advertise crypto capabilities to the block layer. */
	devm_blk_crypto_profile_init(hba->dev, &hba->crypto_profile, 0);
	hba->crypto_profile.max_dun_bytes_supported = AES_BLOCK_SIZE;
	hba->crypto_profile.key_types_supported = BLK_CRYPTO_KEY_TYPE_RAW;
	hba->crypto_profile.dev = hba->dev;
	hba->crypto_profile.modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		FMP_DATA_UNIT_SIZE;
	return 0;
}
