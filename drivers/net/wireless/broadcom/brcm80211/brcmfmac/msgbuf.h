// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMFMAC_MSGBUF_H
#define BRCMFMAC_MSGBUF_H

#ifdef CONFIG_BRCMFMAC_PROTO_MSGBUF

#define BRCMF_H2D_MSGRING_CONTROL_SUBMIT_MAX_ITEM	64
/* The D2H completion rings must match the sizes the firmware wraps at (the
 * BCM4390 firmware reports a fixed 1024-item TX-completion ring); oversizing
 * them makes the host read stale ring slots past the firmware's wrap point.
 */
#define BRCMF_H2D_MSGRING_RXPOST_SUBMIT_MAX_ITEM	2048
#define BRCMF_D2H_MSGRING_CONTROL_COMPLETE_MAX_ITEM	64
#define BRCMF_D2H_MSGRING_TX_COMPLETE_MAX_ITEM		1024
#define BRCMF_D2H_MSGRING_RX_COMPLETE_MAX_ITEM		2048
/* The BCM4390 (msgbuf v3/v4) sizes its TX flowrings at 768 to hold two 256
 * block-ack windows; the vendor driver uses H2DRING_TXPOST_SIZE_V3 == 768.
 */
#define BRCMF_H2D_TXFLOWRING_MAX_ITEM			768

#define BRCMF_H2D_MSGRING_CONTROL_SUBMIT_ITEMSIZE	40
#define BRCMF_H2D_MSGRING_RXPOST_SUBMIT_ITEMSIZE	32
#define BRCMF_D2H_MSGRING_CONTROL_COMPLETE_ITEMSIZE	24
#define BRCMF_D2H_MSGRING_TX_COMPLETE_ITEMSIZE_PRE_V7	16
#define BRCMF_D2H_MSGRING_TX_COMPLETE_ITEMSIZE		24
#define BRCMF_D2H_MSGRING_RX_COMPLETE_ITEMSIZE_PRE_V7	32
#define BRCMF_D2H_MSGRING_RX_COMPLETE_ITEMSIZE		40
#define BRCMF_H2D_TXFLOWRING_ITEMSIZE			80

struct msgbuf_buf_addr {
	__le32		low_addr;
	__le32		high_addr;
};

int brcmf_proto_msgbuf_rx_trigger(struct device *dev);
void brcmf_msgbuf_delete_flowring(struct brcmf_pub *drvr, u16 flowid);
int brcmf_proto_msgbuf_attach(struct brcmf_pub *drvr);
void brcmf_proto_msgbuf_detach(struct brcmf_pub *drvr);
int brcmf_msgbuf_h2d_mb_write(struct brcmf_pub *drvr, u32 data);
#else
static inline int brcmf_proto_msgbuf_attach(struct brcmf_pub *drvr)
{
	return 0;
}
static inline void brcmf_proto_msgbuf_detach(struct brcmf_pub *drvr) {}
#endif

#endif /* BRCMFMAC_MSGBUF_H */
