// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2013 Broadcom Corporation
 */
/*********************channel spec common functions*********************/

#include <linux/module.h>

#include <brcmu_utils.h>
#include <brcmu_wifi.h>
#include <brcmu_d11.h>

static u16 d11n_sb(enum brcmu_chan_sb sb)
{
	switch (sb) {
	case BRCMU_CHAN_SB_NONE:
		return BRCMU_CHSPEC_D11N_SB_N;
	case BRCMU_CHAN_SB_L:
		return BRCMU_CHSPEC_D11N_SB_L;
	case BRCMU_CHAN_SB_U:
		return BRCMU_CHSPEC_D11N_SB_U;
	default:
		WARN_ON(1);
	}
	return 0;
}

static u16 d11n_bw(enum brcmu_chan_bw bw)
{
	switch (bw) {
	case BRCMU_CHAN_BW_20:
		return BRCMU_CHSPEC_D11N_BW_20;
	case BRCMU_CHAN_BW_40:
		return BRCMU_CHSPEC_D11N_BW_40;
	default:
		WARN_ON(1);
	}
	return 0;
}

static void brcmu_d11n_encchspec(struct brcmu_chan *ch)
{
	if (ch->bw == BRCMU_CHAN_BW_20)
		ch->sb = BRCMU_CHAN_SB_NONE;

	ch->chspec = 0;
	brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_CH_MASK,
			BRCMU_CHSPEC_CH_SHIFT, ch->chnum);
	brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11N_SB_MASK,
			0, d11n_sb(ch->sb));
	brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11N_BW_MASK,
			0, d11n_bw(ch->bw));

	if (ch->chnum <= CH_MAX_2G_CHANNEL)
		ch->chspec |= BRCMU_CHSPEC_D11N_BND_2G;
	else
		ch->chspec |= BRCMU_CHSPEC_D11N_BND_5G;
}

static u16 d11ac_bw(enum brcmu_chan_bw bw)
{
	switch (bw) {
	case BRCMU_CHAN_BW_20:
		return BRCMU_CHSPEC_D11AC_BW_20;
	case BRCMU_CHAN_BW_40:
		return BRCMU_CHSPEC_D11AC_BW_40;
	case BRCMU_CHAN_BW_80:
		return BRCMU_CHSPEC_D11AC_BW_80;
	case BRCMU_CHAN_BW_160:
		return BRCMU_CHSPEC_D11AC_BW_160;
	case BRCMU_CHAN_BW_320:
		return BRCMU_CHSPEC_D11AC_BW_320;
	default:
		WARN_ON(1);
	}
	return 0;
}

/* The 6G centre channels a three-bit 320 MHz index stands for. Index 3 is not
 * a channel, and the firmware does not use it.
 */
static const u8 d11ac_320_cch[] = { 31, 95, 159, 0, 63, 127, 191 };

static u8 d11ac_320_id2cch(u16 chspec)
{
	u8 id = brcmu_maskget16(chspec, BRCMU_CHSPEC_D11AC_320_CH_MASK,
				BRCMU_CHSPEC_D11AC_320_CH_SHIFT);

	if (id >= ARRAY_SIZE(d11ac_320_cch))
		return 0;

	return d11ac_320_cch[id];
}

static u8 d11ac_320_cch2id(u8 cch)
{
	u8 id;

	for (id = 0; id < ARRAY_SIZE(d11ac_320_cch); id++)
		if (d11ac_320_cch[id] && d11ac_320_cch[id] == cch)
			return id;

	WARN_ONCE(1, "Invalid 320 MHz centre channel %u\n", cch);
	return 0;
}

static void brcmu_d11ac_encchspec(struct brcmu_chan *ch)
{
	if (ch->bw == BRCMU_CHAN_BW_20 || ch->sb == BRCMU_CHAN_SB_NONE)
		ch->sb = BRCMU_CHAN_SB_L;

	if (ch->bw == BRCMU_CHAN_BW_320) {
		/* Neither field is where the narrower bandwidths keep it, and
		 * the index leaves bits 3~6 over, so start from a clean low
		 * half rather than masking each field in place.
		 */
		ch->chspec &= ~(BRCMU_CHSPEC_CH_MASK |
				BRCMU_CHSPEC_D11AC_320_SB_MASK);
		brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11AC_320_CH_MASK,
				BRCMU_CHSPEC_D11AC_320_CH_SHIFT,
				d11ac_320_cch2id(ch->chnum));
		brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11AC_320_SB_MASK,
				BRCMU_CHSPEC_D11AC_320_SB_SHIFT, ch->sb);
	} else {
		brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_CH_MASK,
				BRCMU_CHSPEC_CH_SHIFT, ch->chnum);
		brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11AC_SB_MASK,
				BRCMU_CHSPEC_D11AC_SB_SHIFT, ch->sb);
	}
	brcmu_maskset16(&ch->chspec, BRCMU_CHSPEC_D11AC_BW_MASK,
			0, d11ac_bw(ch->bw));

	ch->chspec &= ~BRCMU_CHSPEC_D11AC_BND_MASK;
	switch (ch->band) {
	case BRCMU_CHAN_BAND_6G:
		ch->chspec |= BRCMU_CHSPEC_D11AC_BND_6G;
		break;
	case BRCMU_CHAN_BAND_5G:
		ch->chspec |= BRCMU_CHSPEC_D11AC_BND_5G;
		break;
	case BRCMU_CHAN_BAND_2G:
		ch->chspec |= BRCMU_CHSPEC_D11AC_BND_2G;
		break;
	default:
		WARN_ONCE(1, "Invalid band 0x%04x\n", ch->band);
		break;
	}
}

static void brcmu_d11n_decchspec(struct brcmu_chan *ch)
{
	u16 val;

	ch->chnum = (u8)(ch->chspec & BRCMU_CHSPEC_CH_MASK);
	ch->control_ch_num = ch->chnum;

	switch (ch->chspec & BRCMU_CHSPEC_D11N_BW_MASK) {
	case BRCMU_CHSPEC_D11N_BW_20:
		ch->bw = BRCMU_CHAN_BW_20;
		ch->sb = BRCMU_CHAN_SB_NONE;
		break;
	case BRCMU_CHSPEC_D11N_BW_40:
		ch->bw = BRCMU_CHAN_BW_40;
		val = ch->chspec & BRCMU_CHSPEC_D11N_SB_MASK;
		if (val == BRCMU_CHSPEC_D11N_SB_L) {
			ch->sb = BRCMU_CHAN_SB_L;
			ch->control_ch_num -= CH_10MHZ_APART;
		} else {
			ch->sb = BRCMU_CHAN_SB_U;
			ch->control_ch_num += CH_10MHZ_APART;
		}
		break;
	default:
		WARN_ONCE(1,
			  "Invalid chanspec - unknown 11n bandwidth 0x%04x\n",
			  ch->chspec);
		break;
	}

	switch (ch->chspec & BRCMU_CHSPEC_D11N_BND_MASK) {
	case BRCMU_CHSPEC_D11N_BND_5G:
		ch->band = BRCMU_CHAN_BAND_5G;
		break;
	case BRCMU_CHSPEC_D11N_BND_2G:
		ch->band = BRCMU_CHAN_BAND_2G;
		break;
	default:
		WARN_ONCE(1, "Invalid chanspec - unknown 11n band 0x%04x\n",
			  ch->chspec);
		break;
	}
}

static void brcmu_d11ac_decchspec(struct brcmu_chan *ch)
{
	u16 val;

	ch->chnum = (u8)(ch->chspec & BRCMU_CHSPEC_CH_MASK);
	ch->control_ch_num = ch->chnum;

	switch (ch->chspec & BRCMU_CHSPEC_D11AC_BW_MASK) {
	case BRCMU_CHSPEC_D11AC_BW_20:
		ch->bw = BRCMU_CHAN_BW_20;
		ch->sb = BRCMU_CHAN_SB_NONE;
		break;
	case BRCMU_CHSPEC_D11AC_BW_40:
		ch->bw = BRCMU_CHAN_BW_40;
		val = ch->chspec & BRCMU_CHSPEC_D11AC_SB_MASK;
		if (val == BRCMU_CHSPEC_D11AC_SB_L) {
			ch->sb = BRCMU_CHAN_SB_L;
			ch->control_ch_num -= CH_10MHZ_APART;
		} else if (val == BRCMU_CHSPEC_D11AC_SB_U) {
			ch->sb = BRCMU_CHAN_SB_U;
			ch->control_ch_num += CH_10MHZ_APART;
		} else {
			WARN_ONCE(1,
				  "Invalid chanspec - unknown 11ac channel distance 0x%04x\n",
				  ch->chspec);
		}
		break;
	case BRCMU_CHSPEC_D11AC_BW_80:
		ch->bw = BRCMU_CHAN_BW_80;
		ch->sb = brcmu_maskget16(ch->chspec, BRCMU_CHSPEC_D11AC_SB_MASK,
					 BRCMU_CHSPEC_D11AC_SB_SHIFT);
		switch (ch->sb) {
		case BRCMU_CHAN_SB_LL:
			ch->control_ch_num -= CH_30MHZ_APART;
			break;
		case BRCMU_CHAN_SB_LU:
			ch->control_ch_num -= CH_10MHZ_APART;
			break;
		case BRCMU_CHAN_SB_UL:
			ch->control_ch_num += CH_10MHZ_APART;
			break;
		case BRCMU_CHAN_SB_UU:
			ch->control_ch_num += CH_30MHZ_APART;
			break;
		default:
			WARN_ONCE(1,
				  "Invalid chanspec - unknown 11ac channel distance 0x%04x\n",
				  ch->chspec);
			break;
		}
		break;
	case BRCMU_CHSPEC_D11AC_BW_160:
		ch->bw = BRCMU_CHAN_BW_160;
		ch->sb = brcmu_maskget16(ch->chspec, BRCMU_CHSPEC_D11AC_SB_MASK,
					 BRCMU_CHSPEC_D11AC_SB_SHIFT);
		switch (ch->sb) {
		case BRCMU_CHAN_SB_LLL:
			ch->control_ch_num -= CH_70MHZ_APART;
			break;
		case BRCMU_CHAN_SB_LLU:
			ch->control_ch_num -= CH_50MHZ_APART;
			break;
		case BRCMU_CHAN_SB_LUL:
			ch->control_ch_num -= CH_30MHZ_APART;
			break;
		case BRCMU_CHAN_SB_LUU:
			ch->control_ch_num -= CH_10MHZ_APART;
			break;
		case BRCMU_CHAN_SB_ULL:
			ch->control_ch_num += CH_10MHZ_APART;
			break;
		case BRCMU_CHAN_SB_ULU:
			ch->control_ch_num += CH_30MHZ_APART;
			break;
		case BRCMU_CHAN_SB_UUL:
			ch->control_ch_num += CH_50MHZ_APART;
			break;
		case BRCMU_CHAN_SB_UUU:
			ch->control_ch_num += CH_70MHZ_APART;
			break;
		default:
			WARN_ONCE(1,
				  "Invalid chanspec - unknown 11ac channel distance 0x%04x\n",
				  ch->chspec);
			break;
		}
		break;
	case BRCMU_CHSPEC_D11AC_BW_320:
		ch->bw = BRCMU_CHAN_BW_320;
		ch->chnum = d11ac_320_id2cch(ch->chspec);
		ch->sb = brcmu_maskget16(ch->chspec,
					 BRCMU_CHSPEC_D11AC_320_SB_MASK,
					 BRCMU_CHSPEC_D11AC_320_SB_SHIFT);
		if (!ch->chnum) {
			WARN_ONCE(1,
				  "Invalid chanspec - unknown 320 MHz centre channel 0x%04x\n",
				  ch->chspec);
			ch->control_ch_num = 0;
			break;
		}
		/* Sixteen 20 MHz sub-channels, so the control one sits
		 * (2 * sb - 15) * 10 MHz away from the centre.
		 */
		ch->control_ch_num = ch->chnum +
				     (2 * (int)ch->sb - 15) * CH_10MHZ_APART;
		break;
	case BRCMU_CHSPEC_D11AC_BW_8080:
	default:
		WARN_ONCE(1,
			  "Invalid chanspec - unknown 11ac channel bandwidth 0x%04x\n",
			  ch->chspec);
		break;
	}

	switch (ch->chspec & BRCMU_CHSPEC_D11AC_BND_MASK) {
	case BRCMU_CHSPEC_D11AC_BND_6G:
		ch->band = BRCMU_CHAN_BAND_6G;
		break;
	case BRCMU_CHSPEC_D11AC_BND_5G:
		ch->band = BRCMU_CHAN_BAND_5G;
		break;
	case BRCMU_CHSPEC_D11AC_BND_2G:
		ch->band = BRCMU_CHAN_BAND_2G;
		break;
	default:
		WARN_ONCE(1,
			  "Invalid chanspec - unknown 11ac channel band 0x%04x\n",
			  ch->chspec);
		break;
	}
}

void brcmu_d11_attach(struct brcmu_d11inf *d11inf)
{
	if (d11inf->io_type == BRCMU_D11N_IOTYPE) {
		d11inf->encchspec = brcmu_d11n_encchspec;
		d11inf->decchspec = brcmu_d11n_decchspec;
	} else {
		d11inf->encchspec = brcmu_d11ac_encchspec;
		d11inf->decchspec = brcmu_d11ac_decchspec;
	}
}
EXPORT_SYMBOL(brcmu_d11_attach);
