// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos Modem 5300 PCIe boot driver (Google Tensor G4 boards).
 *
 * Bring-up scope: boots the CP to its ONLINE state and answers the shared
 * memory command handshake.  No IPC ports or data path yet -- those follow
 * once the boot handshake is hardware-proven.  The protocol is ported from
 * the downstream GPL cpif driver (google-modules/radio/samsung/s5300,
 * modem_ctrl_s5100.c start_normal_boot()/link_device.c command handlers).
 *
 * Boot flow (all steps observed on downstream hardware traces):
 *  1. The RC driver has already rail-cycled the CP and trained the link; the
 *     mask ROM enumerates as 144d:a5a5 and parks in WAIT_DOORBELL.
 *  2. Move the RC's MSI target into the MSI carveout, allocate the EP's MSI
 *     vectors (the ROM reads the MSI capability address and DMA-writes its
 *     boot progress at fixed offsets above it).
 *  3. Stage the boot image ("BOOT" TOC entry of the factory modem.bin) at
 *     IPC-carveout+0x10000 and drive it in through the two s5400 mask-ROM
 *     doorbell stages: BL1 (first 0xb400 bytes) to boot_stage BL1_DONE, then
 *     the bootloader remainder to DONE.  The CP verifies each image internally
 *     -- no AP-side security call.
 *  4. Bounce the link (vendor s5100_poweroff_pcie/poweron_pcie): drop it,
 *     wait for CP2AP_WAKEUP, relink at Gen3 and restore EP config.
 *  5. Stream the remaining firmware sections (TOC, PSP, MAIN, APM, VSS,
 *     DBGCORE, then device NV/RF) into the NORM_RAW ring as SIT-framed std_dl
 *     frames; the cold CP's bootloader drains them into its DRAM.  (A warm CP
 *     that kept MAIN resident skips this and announces itself immediately.)
 *  6. Answer the shared-memory command handshake the CP then raises:
 *     INIT_START -> PIF_INIT_DONE, PHONE_START -> (queue init) INIT_END, ONLINE.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-zumapro.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/sprintf.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>
#include <linux/zlib.h>

#define S5300_PCI_VENDOR_ID		0x144d
#define S5300_PCI_DEVICE_ID		0xa5a5

/*
 * MSI capability offset in the mask ROM's config space (DesignWare EP
 * default; downstream hardcodes it, print_msi_register()).
 */
#define S5300_ROM_MSI_CAP		0x50

/*
 * The 4K MSI carveout doubles as the boot status block (downstream
 * modem_ctrl.h struct msi_reg_type).  Offset 0 is the MSI termination
 * address; the fields above it are plain DMA targets.
 */
#define S5300_MSI_ERR_REPORT		0x08
#define S5300_MSI_BOOT_STAGE		0x10
#define S5300_MSI_IMG_ADDR_LO		0x14
#define S5300_MSI_IMG_ADDR_HI		0x18
#define S5300_MSI_IMG_SIZE		0x1c
#define S5300_MSI_OTP_VERSION		0x20
/* s5400 diagnostic fields (vendor struct msi_reg_type). */
#define S5300_MSI_FLAG_CAFE		0x30	/* CP-alive marker */
#define S5300_MSI_SUB_BOOT_STAGE	0x34
#define S5300_MSI_DB_LOOP_CNT		0x38
#define S5300_MSI_DB_RECEIVED		0x3c
#define S5300_MSI_BOOT_SIZE		0x40

/*
 * The mask ROM only tolerates MME=2 (exactly four vectors); MAIN raises
 * INIT_START on MSI message-data base 4, so the RC reserves vectors 0-3 first
 * (zumapro_pcie_reserve_msi_base) and the modem's four land at base 4.
 */
#define S5300_MSI_VECTORS		4

/*
 * boot_stage completion masks (vendor modem_ctrl.h).  The s5400 mask ROM
 * boots in two doorbell stages: it reports BL1_DOWNLOAD_DONE (bits 0..13,
 * 0x3fff) after the first boot image and DONE (bits 0..15, 0xffff) after
 * the second.  On the s5300 0x3fff is the single-stage DONE value.
 */
#define S5300_BOOT_STAGE_BL1_DONE	0x3fff
#define S5300_BOOT_STAGE_DONE		0xffff

/*
 * The BOOT section is split into the two doorbell images at a fixed point
 * (vendor cbd: BL1 = first 0xb400 bytes, bootloader = the remainder).  Both
 * are staged into the same boot slot; only the published size differs.
 */
#define S5300_BL1_IMG_SIZE		0xb400

/*
 * IPC region layout (downstream create_legacy_link_device() with tegu's DT
 * offsets, and the DRAM_V1 control-message words).
 */
#define S5300_IPC_MAGIC			0x00
#define S5300_IPC_ACCESS		0x04
#define S5300_IPC_Q_HEAD_TAIL		0x08	/* 8 words: FMT/RAW head+tail */
#define S5300_IPC_Q_WORDS		8
#define S5300_IPC_SRINFO_OFS_PTR	0x64
#define S5300_IPC_CAP_OFS_PTR		0x70
#define S5300_IPC_CAP_BASE		0xa0
#define S5300_IPC_CAP_WORDS		4	/* AP cap x2, CP cap x2 */
#define S5300_IPC_CAP_AP0		(S5300_IPC_CAP_BASE + 0x0)
#define S5300_IPC_CAP_AP1		(S5300_IPC_CAP_BASE + 0x8)
/*
 * AP capability part 0 (downstream set_ap_capabilities()), published at
 * INIT_START before PIF_INIT_DONE.  Bit0 PKTPROC_UL, bit1 CH_EXTENSION
 * (modem_v1.h).  CH_EXTENSION is a no-op on the AP side (pure protocol-format
 * flag); PKTPROC_UL is the only bit that arms real memory (pktproc_init_ul) --
 * setting it makes the CP DMA into UL descriptor memory that does not exist yet
 * and drops the link.  So keep bit0 clear.  The gate that actually lets MAIN arm
 * its runtime IPC is the DL pktproc info region (below), NOT this word.
 */
#define S5300_AP_CAPABILITY_0		0x2

/*
 * Downlink pktproc info region (downstream pktproc_init(), link_rx_pktproc.c).
 * This is a pktproc-class CP: at INIT_START the vendor publishes a
 * struct pktproc_info_v2 at pktproc_base + info_rgn_offset describing where the
 * CP DMAs downlink packets.  MAIN reads it to finish its runtime-IPC bring-up;
 * with the region left zero MAIN never arms and never services the legacy
 * FMT/RAW control rings (AT/SIT/RFS) -- the "reaches ONLINE but the rings stay
 * dead" symptom.  We publish a minimal-but-valid single-queue DL region pointing
 * into the reserved pktproc carveout so MAIN arms; we do not consume DL data
 * (control channels ride the legacy rings, only rmnet uses pktproc).
 *
 * All CP-visible pointers use the CP aperture base (pktproc_cp_base), which the
 * CP's outbound path maps back onto the same physical carveout the AP maps at
 * pktproc_phys.  Values/offsets are the komodo s5400 DT (fdt.dts modem node).
 */
/*
 * Geometry taken verbatim from the vendor cpif boot log (cpif_full.txt
 * pktproc_create / pktproc_create_ul).  DL info at pktproc base, UL info at
 * base + 0x1c00000 (the cp_rmem_1 carveout holds both).  All *_PBASE values are
 * the CP aperture view (pktproc_cp_base + offset).
 */
#define S5300_PKTPROC_CP_BASE		0x20000000	/* pktproc_cp_base */
#define S5300_PKTPROC_DESC_SKTBUF_SZ	16		/* sizeof(pktproc_desc_sktbuf) */
#define S5300_PKTPROC_DESC_UL_SZ	32		/* sizeof(pktproc_desc_ul) */

/* DL: 4 queues, sktbuf, true_packet_size 2048, 3456 desc/queue */
#define S5300_PKTPROC_DL_INFO_OFS	0x0
#define S5300_PKTPROC_DL_DESC_OFS	0x1000
#define S5300_PKTPROC_DL_BUFF_OFS	0x100000
#define S5300_PKTPROC_DL_MAX_PKT	0x630		/* 1584 */
#define S5300_PKTPROC_DL_TRUE_PKT	2048
#define S5300_PKTPROC_DL_DESC_MODE	1		/* DESC_MODE_SKTBUF */
#define S5300_PKTPROC_DL_NUM_QUEUE	4
#define S5300_PKTPROC_DL_NUM_DESC	3456
#define S5300_PKTPROC_DL_Q_DESC_SZ	(S5300_PKTPROC_DL_NUM_DESC * \
					 S5300_PKTPROC_DESC_SKTBUF_SZ)	/* 0xd800 */
#define S5300_PKTPROC_DL_Q_BUFF_SZ	(S5300_PKTPROC_DL_NUM_DESC * \
					 S5300_PKTPROC_DL_TRUE_PKT)	/* 0x6c0000 */

/* UL: 2 queues, 32-byte descriptors, into the second carveout at base+0x1c00000 */
#define S5300_PKTPROC_UL_INFO_OFS	0x1c00000
#define S5300_PKTPROC_UL_DESC_OFS	0x1c01000
#define S5300_PKTPROC_UL_BUFF_OFS	0x1c90000
#define S5300_PKTPROC_UL_MAX_PKT	2048
#define S5300_PKTPROC_UL_NUM_QUEUE	2
#define S5300_PKTPROC_UL_Q0_NUM_DESC	1760
#define S5300_PKTPROC_UL_Q1_NUM_DESC	880
#define S5300_PKTPROC_UL_Q0_MAX_PKT	1024	/* q0 (HIPRIO) buffer stride */
#define S5300_PKTPROC_UL_TXQ		1	/* NORM queue (q1) carries PS data */

/* DL sktbuf control byte (desc byte 5): ring-boundary markers. */
#define S5300_PKTPROC_CTRL_HEAD		0x80
#define S5300_PKTPROC_CTRL_RINGEND	0x08

/*
 * UL descriptor: last_desc ownership (UL info word0 bit24, CP-written) and the
 * CP DMA padding the komodo DT mandates (pktproc_ul_padding_required=1).
 */
#define S5300_PKTPROC_END_BIT_AP	0
#define S5300_PKTPROC_UL_CP_PADDING	76

/*
 * First PDP (raw-IP data) channel.  With CH_EXTENSION advertised (ap_cap bit1)
 * the CP keys PS data on the extended id EXYNOS_CH_EX_ID_PDP_0 (181); the legacy
 * id would be 1.  Keep ap_cap bit1 set or this must change.
 */
#define S5300_CH_PDP_FIRST		0xb5
#define S5300_IPC_AP2CP_MSG		0x800
#define S5300_IPC_CP2AP_MSG		0x804
#define S5300_IPC_AP2CP_STATUS		0x808
#define S5300_IPC_CP2AP_STATUS		0x80c
/*
 * Handover block (downstream ap2cp_handover_block_info = <0x02 0x82c>): a
 * 161-byte struct t_handover_block_info the AP stages before BL1, carrying the
 * modem's HW/RF configuration (project/revision/rf_config/rf_sub...), the two
 * IMEIs and the CP signature.  MAIN reads it to configure itself; with the
 * region left zero it reaches ONLINE but never arms its runtime IPC.  Loaded
 * from firmware (device-specific: IMEIs + signature), staged in
 * s5300_init_control_messages().  cbd sends it via IOCTL_HANDOVER_BLOCK_INFO.
 */
#define S5300_IPC_HANDOVER_OFS		0x82c
#define S5300_HANDOVER_SIZE		161
#define S5300_HANDOVER_FW		"google/s5400/cp_handover.bin"
/*
 * The handover block is assembled entirely from device sources so nothing
 * device-specific is staged as a blob and it works on any caimito s5400 device:
 * the HW/RF config words (bytes 0..63) are derived from androidboot.cdt_hwid
 * (bootloader-provided, read from /proc/bootconfig) plus a hardware_config.json
 * row, exactly the way the vendor cbd does it; the two IMEIs (bytes 64..95) come
 * from the bootloader-populated /chosen/config; and the 64-byte CP signature
 * (bytes 96..159) from the cpsha file (a trailing NUL completes cpsig[65]).
 * cp_handover.bin is the fallback if any source is unavailable.
 */
#define S5300_HANDOVER_IMEI_OFS		64
#define S5300_HANDOVER_CPSIG_OFS	96
#define S5300_CPSHA_FW			"google/s5400/persist/modem/cpsha"
/*
 * RF_CFG auto-selection.  The modem image directory holds one RF_CFG_<sha1> per
 * HW/RF variant plus hardware_config.json, which maps a set of identifiers to
 * the right file.  Those identifiers are exactly the ones the handover block
 * already carries -- json rfid = handover rf_config, hwinfo = revision, and
 * rf_sub/rf_sku(=modem_sku)/modem_hw/major/minor verbatim -- so we read them
 * back from the staged block and pick the file (verified on-device: rf_config
 * 225 / revision 321 -> RF_CFG_ea3a795e...).  Falls back to a pre-staged
 * rf_cfg.bin if the block or the json is absent.
 */
#define S5300_RF_CFG_DIR		"google/s5400/modem/images/default"
#define S5300_HWCFG_FW			S5300_RF_CFG_DIR "/hardware_config.json"
#define S5300_RF_CFG_FALLBACK		"google/s5400/rf_cfg.bin"
/*
 * ap2cp_united_status ds_det field (downstream sbi_ds_det_pos=14, mask 0x3;
 * get_ds_detect() returns 1 on this device, so the live modem reads 0x4000).
 * Load-bearing for runtime IPC: with ds_det=0 the CP never runs its deep-sleep
 * link handshake, so after MAIN loads it never takes a link-up ISR to arm its
 * runtime IPC and the FMT/RAW control queues are never drained.  With ds_det=1
 * the CP cycles CP2AP_WAKEUP; each wake relink (s5300_pm_work) re-arms MAIN's
 * IPC.  Published in s5300_init_control_messages().
 */
#define S5300_IPC_DS_DET		(1u << 14)		/* 0x4000 */

#define S5300_IPC_SRINFO_OFFSET		0x400000
#define S5300_IPC_MAGIC_VALUE		0xaa	/* SIT protocol magic */

/* Interrupt-word encoding (downstream link_device_memory.h). */
#define S5300_INT_VALID			BIT(7)
#define S5300_CMD_VALID			BIT(6)
#define S5300_CMD_MASK			GENMASK(5, 0)
#define S5300_CMD(x)			(S5300_INT_VALID | S5300_CMD_VALID | (x))
#define S5300_CMD_INIT_START		0x1
#define S5300_CMD_INIT_END		0x2
#define S5300_CMD_CRASH_RESET		0x7
#define S5300_CMD_PHONE_START		0x8
#define S5300_CMD_CRASH_EXIT		0x9
#define S5300_CMD_PIF_INIT_DONE		0xd

/*
 * Runtime data-notification masks in ap2cp/cp2ap_msg (downstream
 * link_device_memory.h): once ONLINE the CP raises these (with INT_VALID, no
 * CMD_VALID) to say a ring has data, and the AP ORs them when it sends a frame.
 */
#define S5300_INT_SEND_FMT		BIT(1)	/* MASK_SEND_FMT: FMT ring */
#define S5300_INT_SEND_RAW		BIT(0)	/* MASK_SEND_RAW: NORM_RAW ring */
#define S5300_INT_REQ_ACK_FMT		0x0020	/* CP wants the AP to ack an FMT rx */
#define S5300_INT_RES_ACK_FMT		0x0008	/* AP's ack that it drained an FMT rx */
#define S5300_INT_REQ_ACK_RAW		0x0010	/* CP wants the AP to ack a RAW rx */
#define S5300_INT_RES_ACK_RAW		0x0004	/* AP's ack that it drained a RAW rx */

/*
 * Exynos link-header channel IDs (downstream exynos-cpif.h).  Runtime control
 * channels are multiplexed over the shared rings and demuxed by the ch_id byte
 * in the 12-byte header: FMT (245) = umts_ipc0 RIL control on the FMT ring;
 * AT (21, EXYNOS_CH_ID_BT_DUN) = umts_router AT commands on the NORM_RAW ring.
 */
#define S5300_CH_FMT			245
#define S5300_CH_AT			21
/*
 * RFS file channel (EXYNOS_CH_ID_RFS_0, umts_rfs0) on the NORM_RAW ring: once
 * ONLINE the modem pulls its carrier config and persists NV over it, and rejects
 * SETUP_DATA_CALL ("no carrier config") until an AP-side RFS server answers.
 */
#define S5300_CH_RFS			0x29
#define S5300_RFS_MAX			SZ_4K

/*
 * In-kernel RFS server: after attach the CP OPENs + READs its carrier config
 * (path-whitelisted to carrierconfig/) and its NV files over the RFS channel,
 * chunk by chunk with an ack ping-pong.  We answer them from firmware (the
 * carrierconfig tree + the efs NV) rather than forwarding to a userspace daemon.
 * Framing (RE'd from downstream rfsd): u16 cmd, u16 token, u32 payload_len, then
 * the payload.  Every request MUST be answered or the CP stalls its IPC.
 */
#define S5300_RFS_MAXFID		32
#define S5300_RFS_CHUNK			2012	/* downstream rfsd chunk size */
#define S5300_CC_DIR			"google/s5400/carrierconfig"
#define S5300_RFS_C_READ		0x0001	/* AP->CP chunk / CP->AP next-ack */
#define S5300_RFS_C_WRITE		0x0002	/* AP->CP req / CP->AP data */
#define S5300_RFS_C_STATUS		0x0003	/* status + fid + file-size */
#define S5300_RFS_C_OPEN		0x0004	/* open a path */
#define S5300_RFS_C_CLOSE		0x0005
#define S5300_RFS_C_IO			0x0006	/* start a read or write */
#define S5300_RFS_C_OPEN_FID		0x0007	/* open an NV file by id */
#define S5300_RFS_IO_READ		1
#define S5300_RFS_IO_WRITE		2

/* Doorbell values: bit 16 triggers, low bits select the mailbox index. */
#define S5300_DB_TRIGGER		BIT(16)
#define S5300_DB_MSG			(S5300_DB_TRIGGER | 0x0)	/* int_ap2cp_msg */
#define S5300_DB_LINK_ACK		(S5300_DB_TRIGGER | 0xe)	/* pcie_link_ack */

/* PBL lands at IPC base + this offset (round_up(raw buffer offset, 64K)). */
#define S5300_BOOT_IMG_OFFSET		0x10000

/* boot_stage / CP2AP_WAKEUP polling, downstream check_cp_status(). */
#define S5300_POLL_INTERVAL_MS		20
#define S5300_POLL_COUNT		200

/* PHONE_START handshake timeout, downstream MIF_INIT_TIMEOUT. */
#define S5300_INIT_TIMEOUT		(15 * HZ)

/*
 * Shared-memory IPC arming + the NORM_RAW boot-download ring (vendor
 * init_legacy_link / xmit_to_legacy_link).  The CP polls the SIT boot magic
 * at IPC+0 to recognise the region and MEM_ACCESS gates AP visibility.  On a
 * cold CP (DRAM wiped by the rail-cycle) the bootloader has no resident MAIN,
 * so after the boot bounce the AP streams the remaining firmware sections into
 * this ring; the CP polls txq.head and drains autonomously -- no doorbell.
 * Offsets are the komodo DT legacy_raw_* values (fdt.dts cpif node).
 */
#define S5300_IPC_MEM_ACCESS		0x04
#define S5300_SHM_BOOT_MAGIC		0xbdbd		/* SIT boot magic at IPC+0 */
#define S5300_RAW_TXQ_HEAD		0x18		/* AP advances */
#define S5300_RAW_TXQ_TAIL		0x1c		/* CP advances (drain) */
#define S5300_RAW_RXQ_HEAD		0x20		/* CP advances (ack) */
#define S5300_RAW_RXQ_TAIL		0x24		/* AP advances */
#define S5300_RAW_TXQ_BUFF		0x3000
#define S5300_RAW_TXQ_SIZE		0x1fd000
#define S5300_RAW_RXQ_BUFF		0x200000
#define S5300_RAW_RXQ_SIZE		0x200000
/*
 * Legacy FMT queue (DT legacy_fmt_*): the runtime SIT control channel
 * (umts_ipc0, ch 0xF5).  head/tail block at IPC+0x08 (TXQ head/tail then RXQ
 * head/tail); the FMT buffers start at IPC+0x1000 (legacy_fmt_buffer_offset) --
 * TX buffer at 0x1000, RX buffer at 0x1000+txq_size=0x2000, both 4K.  The CP
 * gates its secondary channels (umts_router AT on the RAW ring) on this control
 * plane being alive and its REQ_ACK_FMT round-trips answered.
 */
#define S5300_FMT_TXQ_HEAD		0x08		/* AP advances */
#define S5300_FMT_TXQ_TAIL		0x0c		/* CP advances (drain) */
#define S5300_FMT_RXQ_HEAD		0x10		/* CP advances */
#define S5300_FMT_RXQ_TAIL		0x14		/* AP advances */
#define S5300_FMT_TXQ_BUFF		0x1000
#define S5300_FMT_TXQ_SIZE		0x1000
#define S5300_FMT_RXQ_BUFF		0x2000
#define S5300_FMT_RXQ_SIZE		0x1000

/* Exynos SIT link header (12B) + Samsung std_dl download header (12B). */
#define S5300_SIT_HDR			12
#define S5300_SIT_SYNC			0xabcd
#define S5300_SIT_CFG_SINGLE		0xc000		/* EXYNOS_SINGLE_MASK<<8 */

/*
 * OEM/GEMS channel (downstream io-device oem_ipc, an IPC_FMT multi-channel iod
 * based at EXYNOS_CH_ID_OEM_0 = 0x81, so oem_ipc1 = 0x82) transported on the
 * legacy FMT ring post-ONLINE -- same frame shape as the SIT control channel,
 * only the channel byte differs.  The CP streams UE-capability-config file
 * requests here; left unanswered its internal LTE-RRC message queue overflows
 * and the modem asserts (PAL_QUEUE_FULL).  A userspace daemon answers it via
 * /dev/umts_oem1.  Payloads are capped so header+payload+pad still fit one
 * ring slot with the circ one-slot gap; both FMT writers share the cap.
 */
#define S5300_CH_OEM			0x82	/* EXYNOS_CH_ID_OEM_0 + 1 (oem_ipc1) */
#define S5300_FMT_MAX			(S5300_FMT_TXQ_SIZE - S5300_SIT_HDR - 8)
#define S5300_OEM_RXQ_MAX		64	/* bound the un-drained rx backlog */
#define S5300_SIT_CH_BOOT		0xf1		/* EXYNOS_CH_ID_BOOT */
#define S5300_DL_HDR			12
#define S5300_DL_CHUNK			0xc000		/* 49152-byte payload cap */
/*
 * std_dl control words: per-section start/end carry a tag; the standalone
 * finalise word (cbd emits 0xa400, CP acks 0xc400) tells the CP the image is
 * complete and to boot MAIN.
 */
#define S5300_DL_SEC_START(tag)		(0xa100 | ((tag) << 4))
#define S5300_DL_SEC_END(tag)		(0xa10d | ((tag) << 4))
#define S5300_DL_SEC_VERIFY(tag)	(0xa301 | ((tag) << 4))	/* CRC-verify */
#define S5300_DL_FINISH			0xa400

/* Per-open state for the in-kernel RFS server (indexed by the CP's fid). */
struct s5300_rfs_file {
	bool			used;
	const struct firmware	*fw;	/* cached read source, or NULL */
	size_t			size;	/* file size reported at OPEN */
	loff_t			roff;	/* read cursor */
	size_t			rremain;	/* bytes left to serve */
	loff_t			woff;	/* write cursor */
	size_t			wremain;	/* bytes left to sink */
	u16			token;	/* active IO token, echoed in chunks */
};

struct s5300_modem;

/*
 * A generic cpif channel exposed as a misc chardev.  Received frames (link
 * header already stripped by the ring drain) are queued as skbs, one delivered
 * per read(); write() wraps one app message in the 12-byte EXYNOS link header
 * for @channel and pushes it onto the FMT transport ring.  The first (only)
 * user is oem_ipc1 (ch 0x82); the header is staged on the stack in the shared
 * s5300_fmt_ring_tx(), so no per-chardev tx buffer is needed.
 */
struct s5300_chardev {
	struct s5300_modem	*sm;
	struct miscdevice	miscdev;
	u8			channel;	/* EXYNOS channel id */
	u8			ch_seq;		/* per-channel link-header seq */
	u32			tx_max;		/* max app payload per frame */
	struct sk_buff_head	rxq;		/* one skb per received frame */
	wait_queue_head_t	read_wq;
};

struct s5300_modem {
	struct device		*dev;
	struct device		*rc_dev;
	struct pci_dev		*pdev;
	struct gpio_desc	*cp2ap_wakeup;
	struct gpio_desc	*cp_active;	/* CP2AP_PHONE_ACTIVE (crash detect) */
	struct gpio_desc	*cp_partial_rst; /* AP2CP_PARTIAL_RST_N (drive high) */

	phys_addr_t		ipc_phys;
	resource_size_t		ipc_size;
	void __iomem		*ipc;
	phys_addr_t		msi_phys;
	void __iomem		*msi;
	phys_addr_t		pktproc_phys;
	resource_size_t		pktproc_size;
	void __iomem		*pktproc;

	u32			db_bus_addr;
	void __iomem		*doorbell;

	const struct firmware	*pbl;
	size_t			boot_off;	/* BOOT section offset within pbl */
	size_t			boot_size;	/* BOOT section length */
	u16			frame_seq;	/* SIT frame counter (download) */
	u8			ch_seq;		/* SIT per-channel counter */
	u32			irq_count;	/* MSI interrupts seen (debug) */
	u32			last_cp2ap;	/* last CP2AP_MSG value (debug) */
	struct work_struct	boot_work;
	struct completion	init_done;
	spinlock_t		lock;	/* orders ap2cp_msg word + doorbell */
	bool			online;
	bool			adopt;	/* live MAIN found at probe: no reset/boot */
	bool			cold_cycled;	/* CP was rail-cycled this probe */

	/*
	 * CP-driven runtime PCIe power management (post-ONLINE): the CP parks its
	 * link when idle and drives CP2AP_WAKEUP to ask for it back.  The relink
	 * sleeps, so it runs on an ordered workqueue off the wakeup IRQ.
	 */
	int			cp2ap_irq;
	int			cp_active_irq;	/* -1 when unavailable */
	struct workqueue_struct	*pm_wq;
	struct work_struct	pm_work;
	struct mutex		pcie_onoff_lock; /* serialises relink up/down */
	bool			pm_armed;	/* cp2ap_irq enabled (boot-seq guard) */
	bool			link_up;	/* RC link powered on (sm->lock) */
	bool			db_reserved;	/* doorbell deferred to wake (sm->lock) */
	bool			cp_wants_up;	/* CP2AP_WAKEUP level latched at edge */
	bool			main_armed;	/* MAIN took a link-up ISR (services rings) */

	/* Runtime control channel: umts_router AT commands over the RAW ring. */
	struct wwan_port	*at_port;
	spinlock_t		tx_lock;	/* serialises RAW txq producers */
	u8			at_ch_seq;	/* per-channel header sequence */

	/*
	 * SIT control plane on the FMT ring (umts_ipc0, ch 0xF5).  The CP gates
	 * its secondary channels on this being serviced + acked; exposed as its
	 * own port so a userspace RIL can drive the modem init handshake.
	 */
	struct wwan_port	*ctrl_port;
	u16			fmt_frame_seq;	/* FMT SIT frame counter */
	u8			fmt_ch_seq;	/* FMT per-channel sequence */

	/*
	 * OEM/GEMS channel (ch 0x82) on the FMT ring, exposed as /dev/umts_oem1.
	 * Both post-ONLINE FMT writers (SIT + oem) share tx_lock and fmt_frame_seq
	 * via s5300_fmt_ring_tx(); fmt_tx_wq wakes a blocked oem writer when the CP
	 * drains the txq and frees ring space.
	 */
	struct s5300_chardev	oem;
	wait_queue_head_t	fmt_tx_wq;

	/* RFS file channel (umts_rfs0, ch 0x29) on the NORM_RAW ring. */
	struct wwan_port	*rfs_port;
	u8			rfs_ch_seq;	/* RFS per-channel sequence */

	/* In-kernel RFS server: carrierconfig/NV files the CP pulls post-attach. */
	struct s5300_rfs_file	rfs_files[S5300_RFS_MAXFID];
	struct work_struct	rfs_work;
	struct list_head	rfs_rxq;	/* queued CP requests (rfs_lock) */
	spinlock_t		rfs_lock;
	struct workqueue_struct	*rfs_wq;
	u8			*rfs_txbuf;	/* one READ frame (rfs_work only) */

	/*
	 * PKTPROC data path: the raw-IP data netdev (rmnet0).  The DL sktbuf ring
	 * is drained from the MSI handler into skbs; dl_fore is the AP's re-arm
	 * (buffers-available) pointer and dl_done its private consumer cursor, both
	 * per DL queue.  The UL NORM ring is fed by ndo_start_xmit; ul_done is the
	 * producer index, touched only under tx_lock.
	 */
	struct net_device	*ndev;
	u32			dl_fore[S5300_PKTPROC_DL_NUM_QUEUE];
	u32			dl_done[S5300_PKTPROC_DL_NUM_QUEUE];
	u32			ul_done;
	u16			ul_cp_quota;
	u8			ul_end_bit_owner;
	bool			ul_active;
};

/*
 * Program the doorbell BAR and read it back, like downstream does (it
 * computes the doorbell offset from what actually landed, and its restore
 * path runs a "BAR0 value correction" rewrite).  The ROM-phase BAR is 1M
 * so the hardware aligns the programmed address down; any base that still
 * decodes the doorbell bus address is fine.
 */
static int s5300_program_doorbell_bar(struct s5300_modem *sm)
{
	u32 val = 0, base;
	int try;

	for (try = 0; try < 10; try++) {
		pci_write_config_dword(sm->pdev, PCI_BASE_ADDRESS_0,
				       sm->db_bus_addr);
		pci_write_config_dword(sm->pdev, PCI_BASE_ADDRESS_1, 0);
		pci_read_config_dword(sm->pdev, PCI_BASE_ADDRESS_0, &val);
		base = val & PCI_BASE_ADDRESS_MEM_MASK;
		if (base && base <= sm->db_bus_addr &&
		    sm->db_bus_addr - base < SZ_1M) {
			if (try)
				dev_warn(sm->dev,
					 "doorbell BAR stuck after %d retries (%#x)\n",
					 try, val);
			return 0;
		}
		udelay(100);
	}

	dev_err(sm->dev, "doorbell BAR won't hold %#x (reads %#x)\n",
		sm->db_bus_addr, val);
	return -EIO;
}

/*
 * Memory TLPs are address-routed: the root port only forwards them
 * downstream inside its type-1 memory window, and because the doorbell BAR
 * is programmed behind the PCI core's back (no child resource was ever
 * assigned) the core leaves that window closed -- every doorbell access
 * dies at the root port with the EP config-reachable but memory-dead.
 * Downstream opens it implicitly by routing its 4K BAR through
 * pci_assign_resource(); open it explicitly over the doorbell's 1M-aligned
 * range instead.  The root port is the RC's own DBI, not the flaky ROM, so
 * a single verified write suffices.
 */
static int s5300_open_bridge_window(struct s5300_modem *sm)
{
	struct pci_dev *bridge = pci_upstream_bridge(sm->pdev);
	u32 base = sm->db_bus_addr & ~(SZ_1M - 1);
	u32 limit = base + SZ_1M - 1;
	u32 want = (((limit >> 16) & 0xfff0) << 16) | ((base >> 16) & 0xfff0);
	u32 val;
	u16 cmd;

	if (!bridge)
		return -ENODEV;

	pci_read_config_dword(bridge, PCI_MEMORY_BASE, &val);
	if (val != want) {
		dev_info(sm->dev,
			 "opening root-port memory window %#x-%#x (was %#010x)\n",
			 base, limit, val);
		pci_write_config_dword(bridge, PCI_MEMORY_BASE, want);
		pci_read_config_dword(bridge, PCI_MEMORY_BASE, &val);
		if (val != want) {
			dev_err(sm->dev,
				"root-port window won't hold (%#010x)\n", val);
			return -EIO;
		}
	}

	/*
	 * The core re-assigns the root port's own (dummy, dw_pcie_setup_rc
	 * zeroes it) BAR0 into the bottom of the translation window at
	 * enumeration; TLPs matching an RP BAR are consumed by the port,
	 * not forwarded, so evict it from the doorbell range.
	 */
	pci_read_config_dword(bridge, PCI_BASE_ADDRESS_0, &val);
	if ((val & PCI_BASE_ADDRESS_MEM_MASK) >= base &&
	    (val & PCI_BASE_ADDRESS_MEM_MASK) <= limit) {
		dev_info(sm->dev,
			 "evicting root-port BAR0 (%#010x) from the doorbell window\n",
			 val);
		pci_write_config_dword(bridge, PCI_BASE_ADDRESS_0, 0);
		pci_write_config_dword(bridge, PCI_BASE_ADDRESS_1, 0);
	}

	pci_read_config_word(bridge, PCI_COMMAND, &cmd);
	if ((cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
	    (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER))
		pci_write_config_word(bridge, PCI_COMMAND, cmd |
				      PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

	return 0;
}

/*
 * Ring the doorbell with the downstream retry shape: right after a (re)train
 * the EP's memory decode may not be settled yet, so an all-ones read-back
 * gets the command register and doorbell BAR repaired and the write retried
 * (downstream s51xx_pcie_send_doorbell_int() retries at 1 ms up to 100x;
 * keep it short here because the IPC path rings from hard-IRQ context).
 * Returns true if the write took.  If config space itself reads 0xffff the link
 * is *physically* down (the CP parked it): reprogramming BARs cannot revive that
 * and hammering it can wedge the CP, so bail and let the caller escalate to a
 * relink.
 */
static bool s5300_send_doorbell(struct s5300_modem *sm, u32 val)
{
	int try;
	u16 cmd;

	for (try = 0; try < 10; try++) {
		writel(val, sm->doorbell);
		if (readl(sm->doorbell) != 0xffffffff)
			return true;

		pci_read_config_word(sm->pdev, PCI_COMMAND, &cmd);
		if (cmd == 0xffff) {
			dev_dbg(sm->dev, "doorbell %#x: link down, need relink\n",
				val);
			return false;
		}
		if ((cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
		    (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER))
			pci_write_config_word(sm->pdev, PCI_COMMAND, cmd |
					      PCI_COMMAND_MEMORY |
					      PCI_COMMAND_MASTER);
		s5300_program_doorbell_bar(sm);
		s5300_open_bridge_window(sm);
		udelay(100);
	}

	dev_err(sm->dev, "doorbell %#x kept reading back all-ones\n", val);
	return false;
}

/*
 * The ROM derives its boot-status DMA target from the MSI message address
 * registers; downstream re-drives them whenever they read back zero
 * (print_msi_register(): "MSI Message Reg == 0x0 - set MSI again!!!").
 */
static void s5300_verify_msi_target(struct s5300_modem *sm)
{
	u32 lo = 0;
	int try;

	for (try = 0; try < 5; try++) {
		pci_read_config_dword(sm->pdev,
				      sm->pdev->msi_cap + PCI_MSI_ADDRESS_LO,
				      &lo);
		if (lo == lower_32_bits(sm->msi_phys))
			return;
		dev_warn(sm->dev, "MSI address reads %#x, re-driving\n", lo);
		pci_restore_msi_state(sm->pdev);
	}

	dev_err(sm->dev, "MSI address won't hold %pap\n", &sm->msi_phys);
}

/*
 * Downstream pcie_send_ap2cp_irq(): stage the interrupt word, then ring the
 * doorbell -- but only while the link is up.  The word lives in AP DRAM the CP
 * DMAs, so it is always safe to stage; when the CP has parked the link (link_up
 * false, post-ONLINE runtime PM) defer the doorbell and nudge AP2CP_WAKEUP.  The
 * CP answers on CP2AP_WAKEUP, s5300_pm_work() relinks and flushes the reserved
 * doorbell.  During boot link_up stays true, so the INIT/PHONE handshake rings
 * immediately as before.
 */
static void s5300_send_ipc_irq(struct s5300_modem *sm, u32 val)
{
	bool wake = false;
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);
	writel(val, sm->ipc + S5300_IPC_AP2CP_MSG);
	if (sm->link_up && s5300_send_doorbell(sm, S5300_DB_MSG)) {
		sm->db_reserved = false;
	} else {
		/*
		 * The CP has parked (link_up already false) or the link is
		 * physically down despite link_up (doorbell read back all-ones,
		 * e.g. the CP dropped the link mid-transfer without the GPIO park
		 * handshake).  Either way clear link_up so s5300_pm_work() relinks,
		 * keep the doorbell reserved, and nudge AP2CP_WAKEUP so the CP
		 * answers on CP2AP_WAKEUP; the relink then flushes it.
		 */
		sm->link_up = false;
		sm->db_reserved = true;
		wake = true;
	}
	spin_unlock_irqrestore(&sm->lock, flags);

	if (wake) {
		dev_info(sm->dev,
			 "ctrl tx while parked: staged msg %#x, nudging AP2CP_WAKEUP (cp_active %d)\n",
			 val, sm->cp_active ?
			 gpiod_get_value(sm->cp_active) : -1);
		/*
		 * AP-initiated wake, mirroring downstream pcie_send_ap2cp_irq() ->
		 * s5100_try_gpio_cp_wakeup(): drive AP2CP_WAKEUP high and RETURN.  Do
		 * NOT relink here -- the parked CP has to bring its own PHY back up
		 * first, which it signals by raising CP2AP_WAKEUP.  Retraining before
		 * that ack parks the LTSSM in Polling (0x3).  The CP2AP_WAKEUP rising
		 * edge drives s5300_pm_work(), which relinks and flushes the reserved
		 * doorbell -- exactly the CP-initiated wake path.
		 */
		zumapro_pcie_modem_wake(sm->rc_dev);
	}
}

/*
 * Restore the endpoint config the boot bounce established (D0, forced doorbell
 * BAR0, open root-port memory window, MSI target) after a runtime relink -- the
 * same restore s5300_boot_work() runs after the boot-phase bounce, since
 * modem_link_up()'s host_init re-runs dw_pcie_setup_rc and resets the RC MSI
 * address.  MAIN rebuilds its own inbound doorbell decode on its link-up ISR;
 * this repairs the AP-side outbound routing the PERST cycle reset.
 */
static void s5300_relink_restore(struct s5300_modem *sm)
{
	pci_set_power_state(sm->pdev, PCI_D0);
	pci_restore_state(sm->pdev);
	pci_set_master(sm->pdev);
	zumapro_pcie_set_msi_target(sm->rc_dev, sm->msi_phys);
	s5300_program_doorbell_bar(sm);
	s5300_open_bridge_window(sm);
	s5300_verify_msi_target(sm);
}

#define S5300_PARK_SETTLE_MS	30

/*
 * Debug: when set, never park the runtime PCIe link -- keep it in L0 after
 * ONLINE even when the CP requests deep sleep.  Lets us test the FMT/RAW
 * control path (AT/SIT/RFS) with the link permanently up, isolating a
 * ring-servicing bug from the parked-CP GPIO wake handshake.
 */
static bool s5300_no_park;
module_param_named(no_park, s5300_no_park, bool, 0644);
MODULE_PARM_DESC(no_park, "keep the runtime PCIe link up (skip CP deep-sleep park)");

/*
 * Debug: publish ds_det=1 so the CP runs deep-sleep link PM (default).  Set 0 to
 * leave ds_det clear so the CP never parks the runtime link -- isolates MAIN's
 * runtime-IPC arming from the (separately broken) parked-CP wake handshake.
 */
static bool s5300_ds_det = true;
module_param_named(ds_det, s5300_ds_det, bool, 0644);
MODULE_PARM_DESC(ds_det, "publish ds_det=1 for CP deep-sleep link PM (0 = link stays up)");

/*
 * AP capability part 0 advertised at INIT_START.  Vendor value is 0x3
 * (PKTPROC_UL|CH_EXTENSION, cpif_full.txt).  Bit0 PKTPROC_UL makes the CP DMA
 * uplink from the UL pktproc rings, so it is only safe once we publish a valid
 * UL info region (we do).  Knob so we can prove bit0 is the arming gate.
 */
/*
 * Vendor komodo value: 0x3 (PKTPROC_UL|CH_EXTENSION, cpif_full.txt).  Proven
 * on-device together with pktproc=1: fresh boot to ONLINE, both legacy rings
 * serviced, AT round-trips, runtime park/wake cycles.  (The earlier deaths
 * this was suspected for were stale per-device NV/replay partition content.)
 */
static uint s5300_ap_cap = 0x3;
module_param_named(ap_cap, s5300_ap_cap, uint, 0644);
MODULE_PARM_DESC(ap_cap, "AP capability[0] published at INIT_START (vendor 0x3)");

/*
 * Master pktproc kill-switch.  pktproc=0 skips the DL desc fill and the DL/UL
 * info publishes regardless of ap_cap (pure legacy IPC, the tegu fallback
 * config).  Default ON: the vendor config is proven on-device with fresh
 * NV/replay partition content.
 */
static bool s5300_pktproc = true;
module_param_named(pktproc, s5300_pktproc, bool, 0644);
MODULE_PARM_DESC(pktproc, "publish pktproc regions (0 = pure legacy IPC)");

/*
 * adopt=0 forces a warm reset + fresh boot even when a live MAIN is found at
 * probe -- the way to get a clean IPC state (freshly initialised rings) on
 * demand from an otherwise-golden CP.
 */
static bool s5300_allow_adopt = true;
module_param_named(adopt, s5300_allow_adopt, bool, 0644);
MODULE_PARM_DESC(adopt, "adopt a live MAIN at probe instead of resetting (default 1)");

/*
 * cold_cycle DEFAULT 1: power-cycle the CP (s5910 + rails) back to its boot
 * ROM and download a fresh image.  This is the verified-working path (AT
 * returns OK after ONLINE) and the one autoload must take: on a real boot the
 * CP is in its boot ROM, not a live Gen3 endpoint, so there is nothing to
 * adopt.  The old belief that a reset+download always self-resets ~123 ms
 * after ONLINE was wrong -- the cause was AP2CP_PARTIAL_RST_N (gpp21-6) never
 * being driven high, which MAIN samples as "AP requests partial reset."  That
 * is now deasserted in s5300_boot_work() (s5300_drive_partial_rst), so a fresh
 * download survives.
 *
 * cold_cycle=0 skips the reset when a live MAIN already exists (Gen3 endpoint)
 * and ADOPTS it instead -- faster for developer warm reloads, but the ring
 * resync can surface stale buffered data on the first read, so it is not the
 * default.  Autoload never hits the adopt branch (boot-ROM CP is not Gen3).
 */
static bool s5300_cold_cycle = true;
module_param_named(cold_cycle, s5300_cold_cycle, bool, 0644);
MODULE_PARM_DESC(cold_cycle, "full CP power cycle + fresh download (default 1); 0 = adopt a live MAIN");

/*
 * True while the CP still owes us: any legacy txq (FMT or RAW) with head != tail
 * is an AP->CP frame the CP has not yet consumed.  Mirrors downstream
 * check_mem_link_tx_pending() / check_legacy_tx_pending().  readl() only, so it
 * is safe to call under sm->lock.
 */
static bool s5300_tx_pending(struct s5300_modem *sm)
{
	return readl(sm->ipc + S5300_FMT_TXQ_HEAD) !=
			readl(sm->ipc + S5300_FMT_TXQ_TAIL) ||
	       readl(sm->ipc + S5300_RAW_TXQ_HEAD) !=
			readl(sm->ipc + S5300_RAW_TXQ_TAIL);
}

/*
 * Reconcile the RC link power with what the CP asks for on CP2AP_WAKEUP,
 * mirroring downstream s5100_poweron_pcie()/s5100_poweroff_pcie() driven from
 * ap_wakeup_handler().  Runs on an ordered workqueue because the relink sleeps
 * (PHY power cycle + PERST retrain).  CP2AP_WAKEUP high = the CP wants the link
 * up (it woke, or answered our AP2CP_WAKEUP nudge); low = the CP has parked and
 * the link may drop to save power.  Uses our existing (boot-bounce) link_up/down
 * with the same D3hot save/restore the boot re-link uses.
 */
static void s5300_pm_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem, pm_work);
	bool want_up = READ_ONCE(sm->cp_wants_up);
	unsigned long flags;

	/*
	 * Relink ONLY when the CP is actually asking for the link (CP2AP_WAKEUP
	 * high), mirroring downstream s5100_poweron_pcie()'s guard "skip pci power
	 * on: condition not met" when CP2AP_WAKEUP == 0 (modem_ctrl_s5100.c).  An
	 * AP-initiated tx while parked nudges AP2CP_WAKEUP and waits for the CP to
	 * answer with a CP2AP_WAKEUP rising edge, which re-enters this work with
	 * want_up=true; relinking before that answer trains into a dead endpoint
	 * (LTSSM parks in Polling, 0x3).  Re-check the live GPIO under the onoff
	 * lock in case the edge that queued us has already fallen again.
	 */
	mutex_lock(&sm->pcie_onoff_lock);

	if (want_up && !sm->link_up &&
	    gpiod_get_value_cansleep(sm->cp2ap_wakeup)) {
		if (zumapro_pcie_modem_link_up(sm->rc_dev) == 0) {
			s5300_relink_restore(sm);
			spin_lock_irqsave(&sm->lock, flags);
			sm->link_up = true;
			/*
			 * This is MAIN's link-up ISR: it (re)arms its runtime IPC
			 * here, so from now on the vendor tx-pending park debounce
			 * is meaningful (MAIN will actually drain in-flight tx).
			 */
			sm->main_armed = true;
			spin_unlock_irqrestore(&sm->lock, flags);
			dev_info(sm->dev, "CP wakeup: link up (MAIN armed)\n");
		} else {
			dev_err(sm->dev, "CP wakeup: relink failed\n");
		}
	} else if (!want_up && sm->link_up && s5300_no_park) {
		/*
		 * Debug knob: the CP asked to park (CP2AP_WAKEUP low) but we keep the
		 * link in L0 so MAIN stays serviceable.  Isolates "does the runtime
		 * FMT/RAW ring path work with the link up?" from the parked-CP wake
		 * handshake -- with no_park set, an AT write should get an OK without
		 * any relink at all.
		 */
		dev_info(sm->dev, "no_park: CP requested park, keeping link up\n");
	} else if (!want_up && sm->link_up) {
		bool park;

		/*
		 * Don't park while the CP still owes us -- port of downstream
		 * s5100_poweroff_pcie(force_off=false): settle, then abort the
		 * teardown if the CP re-asserted CP2AP_WAKEUP, an AP->CP frame is
		 * still un-drained (txq head != tail), or a doorbell is reserved.
		 * Tearing down (full PERST + PHY off) on the CP2AP_WAKEUP falling
		 * edge with a request in flight drops it -- the source of the
		 * control-channel flakiness.  A later falling edge retries the park;
		 * under ds_det=1 the CP cycles CP2AP_WAKEUP continuously, so the
		 * link is never stranded up (worst case power-only, never a hang).
		 *
		 * Read the (possibly sleeping) GPIO outside sm->lock; commit
		 * link_up=false only under the lock and only when actually parking,
		 * so a concurrent send either rings while the link is genuinely up
		 * or reserves once we've decided to tear down.
		 */
		msleep(S5300_PARK_SETTLE_MS);

		if (gpiod_get_value_cansleep(sm->cp2ap_wakeup)) {
			dev_info(sm->dev, "CP sleep: park deferred (CP re-asserted)\n");
		} else {
			spin_lock_irqsave(&sm->lock, flags);
			/*
			 * Until MAIN has taken its first link-up ISR (main_armed),
			 * it is NOT servicing the runtime rings -- so any AP->CP
			 * frames (e.g. ModemManager's port probes) will never drain
			 * and s5300_tx_pending() is permanently true.  Honouring the
			 * vendor tx-pending defer here would keep the link up forever
			 * and MAIN would never get the park->wake cycle that arms it.
			 * So before MAIN is armed, honour the CP's self-park request
			 * unconditionally; once armed, fall back to the vendor debounce
			 * (keep the link up for genuinely in-flight tx).
			 */
			park = !sm->main_armed ||
			       (!s5300_tx_pending(sm) && !sm->db_reserved);
			if (park)
				sm->link_up = false;
			spin_unlock_irqrestore(&sm->lock, flags);

			if (park) {
				zumapro_pcie_modem_link_down(sm->rc_dev);
				dev_info(sm->dev, "CP sleep: link down (parked)\n");
			}
		}
	}

	mutex_unlock(&sm->pcie_onoff_lock);

	spin_lock_irqsave(&sm->lock, flags);
	if (sm->db_reserved && sm->link_up &&
	    s5300_send_doorbell(sm, S5300_DB_MSG)) {
		/* Link is up -- delivered the doorbell a tx had reserved. */
		sm->db_reserved = false;
		spin_unlock_irqrestore(&sm->lock, flags);
	} else if (sm->db_reserved) {
		/*
		 * Either parked (link_up false) or the relink came back but the
		 * doorbell still read all-ones (link_up stale-true, physically
		 * down).  Clear link_up so the next wake actually relinks instead
		 * of retrying a dead BAR, and re-drive AP2CP_WAKEUP so the CP
		 * answers and the next pm_work flushes the still-reserved doorbell.
		 */
		sm->link_up = false;
		spin_unlock_irqrestore(&sm->lock, flags);
		zumapro_pcie_modem_wake(sm->rc_dev);
	} else {
		spin_unlock_irqrestore(&sm->lock, flags);
	}
}

static irqreturn_t s5300_cp2ap_wakeup_irq(int irq, void *data)
{
	struct s5300_modem *sm = data;

	/*
	 * Latch the level at edge time: the CP pulses CP2AP_WAKEUP faster than the
	 * workqueue can read it, so reading the GPIO in pm_work misses a brief park.
	 * cp2ap_wakeup is a memory-mapped SoC GPIO, so gpiod_get_value never sleeps.
	 */
	int up = gpiod_get_value(sm->cp2ap_wakeup);

	WRITE_ONCE(sm->cp_wants_up, up);
	dev_info(sm->dev, "CP2AP_WAKEUP irq: level %d\n", up);
	queue_work(sm->pm_wq, &sm->pm_work);
	return IRQ_HANDLED;
}

/*
 * CP2AP_PHONE_ACTIVE (gpa5-2): level 1 while the CP runs; the falling edge is
 * the CP crash indication (downstream cp_active_handler -> STATE_CRASH_EXIT).
 * Diagnosis only for now -- distinguishes "CP parked and deaf to the
 * AP2CP_WAKEUP nudge" (active still 1) from "CP died after ONLINE" (active 0).
 */
static irqreturn_t s5300_cp_active_irq(int irq, void *data)
{
	struct s5300_modem *sm = data;
	/* gpa5 alive-block GPIO: memory-mapped, never sleeps (see wakeup irq). */
	int up = gpiod_get_value(sm->cp_active);

	if (up) {
		dev_info(sm->dev, "CP2AP_PHONE_ACTIVE irq: level 1 (CP alive)\n");
	} else {
		/*
		 * Post-mortem: the CP dies too hard to send CMD_CRASH_EXIT, but
		 * its boot/err MSI words and the last cp2ap ctrl msg survive in
		 * AP DRAM -- the corpse's note.
		 */
		dev_err(sm->dev,
			"CP2AP_PHONE_ACTIVE irq: level 0 -- CP CRASHED (err %#x boot_stage %#x sub %#x cafe %#x cp2ap %#x irq #%u)\n",
			readl(sm->msi + S5300_MSI_ERR_REPORT),
			readl(sm->msi + S5300_MSI_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_SUB_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_FLAG_CAFE),
			readl(sm->ipc + S5300_IPC_CP2AP_MSG),
			sm->irq_count);
	}
	return IRQ_HANDLED;
}

/*
 * Downstream init_control_messages(): publish the srinfo and capability
 * offsets and zero the status/capability words before the CP boots.  The
 * AP capability words stay zero (tegu DT: ap_capability_0/1 = 0).
 */
static void s5300_pktproc_fill_desc(struct s5300_modem *sm);

/*
 * Parse the decimal value of "key" within the JSON object bytes [start,end).
 * Returns 0 + *out, or -ENOENT when the key is absent.  Deliberately minimal --
 * hardware_config.json is a small machine-generated file, not arbitrary JSON, so
 * a targeted scan beats dragging a parser into the kernel.
 */
static int s5300_json_uint(const char *start, const char *end,
			   const char *key, u32 *out)
{
	char pat[16];
	const char *p;
	u32 v = 0;

	scnprintf(pat, sizeof(pat), "\"%s\"", key);
	p = strnstr(start, pat, end - start);
	if (!p)
		return -ENOENT;
	for (p += strlen(pat); p < end && *p != ':'; p++)
		;
	for (p++; p < end && (*p == ' ' || *p == '\t'); p++)
		;
	if (p >= end || *p < '0' || *p > '9')
		return -ENOENT;
	for (; p < end && *p >= '0' && *p <= '9'; p++)
		v = v * 10 + (*p - '0');
	*out = v;
	return 0;
}

/*
 * The CDT (Configuration Data Table) fields the bootloader packs into
 * androidboot.cdt_hwid.  cbd parses the hex string by fixed nibble widths and
 * matches a hardware_config.json row on platform/product +
 * stage(=board)/major/minor/rf_sku/rf_sub/modem_hw; that row yields rfid (->
 * rf_config) and hwinfo (-> revision) and the RF_CFG file.
 */
struct s5300_cdt {
	u32 platform, product, board, major, minor, variant, rf_sku, modem_hw, rf_sub;
};

/*
 * cdt_hwid: the bootloader-provided androidboot.cdt_hwid, forwarded in as a
 * module param -- via bootconfig (a kernel.s5xxx_modem.cdt_hwid entry, which the
 * kernel renders onto the cmdline) or by userspace out of /proc/bootconfig.  A
 * module cannot read bootconfig itself (xbc_* is __init-only) and reading
 * /proc from a driver is an upstream no-no, so the value comes in as a param and
 * the parse + hardware_config.json lookup (cbd's scan_cdt_property_data) stay
 * in-kernel.  Empty -> the driver falls back to a staged cp_handover.bin.
 */
static char *s5300_cdt_hwid;
module_param_named(cdt_hwid, s5300_cdt_hwid, charp, 0444);
MODULE_PARM_DESC(cdt_hwid, "androidboot.cdt_hwid for the auto-built handover block");

/* Parse cdt_hwid the way cbd's scan_cdt_property_data() does (fixed nibbles). */
static int s5300_parse_cdt_hwid(struct s5300_cdt *cdt)
{
	if (!s5300_cdt_hwid || !*s5300_cdt_hwid)
		return -ENODATA;
	if (sscanf(s5300_cdt_hwid, "0x%4x%2x%2x%4x%2x%2x%2x%2x%4x",
		   &cdt->platform, &cdt->product, &cdt->board, &cdt->major,
		   &cdt->minor, &cdt->variant, &cdt->rf_sku, &cdt->modem_hw,
		   &cdt->rf_sub) != 9)
		return -EINVAL;
	return 0;
}

/*
 * Scan one configurations[] entry's config_table rows (bounded by [entry,lim))
 * for the match; on the matching row fill rfid (-> rf_config), hwinfo (->
 * revision) and the RF_CFG basename (config_file is an absolute vendor path).
 * Returns 0 on match, -ENOENT otherwise.
 */
static int s5300_hwcfg_row(const char *entry, const char *lim,
			   const struct s5300_cdt *cdt, u32 *rfid, u32 *hwinfo,
			   char *rf_cfg, size_t rf_cfg_sz)
{
	const char *r = entry;

	while ((r = strnstr(r, "\"config_file\"", lim - r))) {
		const char *objend = strnstr(r, "}", lim - r);
		const char *v, *name, *base, *s;
		u32 stage, major, minor, rf_sku, rf_sub, modem_hw;

		if (!objend)
			break;
		if (s5300_json_uint(r, objend, "stage", &stage) ||
		    s5300_json_uint(r, objend, "major", &major) ||
		    s5300_json_uint(r, objend, "minor", &minor) ||
		    s5300_json_uint(r, objend, "rf_sku", &rf_sku) ||
		    s5300_json_uint(r, objend, "rf_sub", &rf_sub) ||
		    s5300_json_uint(r, objend, "modem_hw", &modem_hw) ||
		    stage != cdt->board || major != cdt->major ||
		    minor != cdt->minor || rf_sku != cdt->rf_sku ||
		    rf_sub != cdt->rf_sub || modem_hw != cdt->modem_hw ||
		    s5300_json_uint(r, objend, "rfid", rfid) ||
		    s5300_json_uint(r, objend, "hwinfo", hwinfo)) {
			r = objend + 1;
			continue;
		}
		v = strchr(r, ':');
		if (v && v < objend)
			v = strchr(v, '"');
		if (!v || v >= objend)
			break;
		for (name = ++v; v < objend && *v != '"'; v++)
			;
		for (base = name, s = name; s < v; s++)
			if (*s == '/')
				base = s + 1;
		scnprintf(rf_cfg, rf_cfg_sz, "%.*s", (int)(v - base), base);
		return 0;
	}
	return -ENOENT;
}

/*
 * Match this device's hardware_config.json row from the parsed CDT and hand back
 * rfid (-> rf_config), hwinfo (-> revision) and the RF_CFG basename.  Mirrors
 * cbd: pick the configurations[] entry on (platform, product), then the
 * config_table row on (stage=board, major, minor, rf_sku, rf_sub, modem_hw).
 */
static int s5300_hwcfg_lookup(struct s5300_modem *sm, const struct s5300_cdt *cdt,
			      u32 *rfid, u32 *hwinfo, char *rf_cfg, size_t rf_cfg_sz)
{
	const struct firmware *json;
	char *buf, *entry, *bufend;
	int ret = -ENOENT;

	if (request_firmware_direct(&json, S5300_HWCFG_FW, sm->dev))
		return -ENOENT;
	buf = kvmalloc(json->size + 1, GFP_KERNEL);
	if (!buf) {
		release_firmware(json);
		return -ENOMEM;
	}
	memcpy(buf, json->data, json->size);
	buf[json->size] = '\0';
	release_firmware(json);
	bufend = buf + strlen(buf);

	entry = buf;
	while ((entry = strstr(entry, "\"platform\""))) {
		char *entry_end = strstr(entry + 10, "\"platform\"");
		char *lim = entry_end ? entry_end : bufend;
		u32 plat, prod;

		if (!s5300_json_uint(entry, lim, "platform", &plat) &&
		    !s5300_json_uint(entry, lim, "product", &prod) &&
		    plat == cdt->platform && prod == cdt->product &&
		    !s5300_hwcfg_row(entry, lim, cdt, rfid, hwinfo, rf_cfg,
				     rf_cfg_sz)) {
			ret = 0;
			break;
		}
		if (!entry_end)
			break;
		entry = entry_end;
	}
	kvfree(buf);
	return ret;
}

/* Parse the CDT hwid param, then resolve rfid/hwinfo/RF_CFG from the json. */
static int s5300_resolve_hwcfg(struct s5300_modem *sm, struct s5300_cdt *cdt,
			       u32 *rfid, u32 *hwinfo, char *rf_cfg, size_t sz)
{
	int ret = s5300_parse_cdt_hwid(cdt);

	if (ret)
		return ret;
	return s5300_hwcfg_lookup(sm, cdt, rfid, hwinfo, rf_cfg, sz);
}

/*
 * Assemble the 161-byte t_handover_block_info MAIN reads (the vendor's
 * IOCTL_HANDOVER_BLOCK_INFO) entirely from device sources -- no staged blob,
 * works on any caimito s5400.  The HW/RF words come from androidboot.cdt_hwid +
 * a hardware_config.json row (exactly like cbd's update_handover_block_info),
 * the two IMEIs from /chosen/config, the CP signature from cpsha.  Returns 0 with
 * buf filled, or -errno if a source is missing (caller tries cp_handover.bin).
 */
static int s5300_build_handover(struct s5300_modem *sm, u8 *buf)
{
	const struct firmware *sha;
	struct device_node *cnode;
	struct s5300_cdt cdt;
	u32 rfid, hwinfo;
	char rf_cfg[80];
	const void *imei;
	int ret, len;

	ret = s5300_resolve_hwcfg(sm, &cdt, &rfid, &hwinfo, rf_cfg, sizeof(rf_cfg));
	if (ret)
		return ret;

	memset(buf, 0, S5300_HANDOVER_SIZE);
	put_unaligned_le32(1,            buf + 0);	/* version */
	put_unaligned_le32(cdt.platform, buf + 4);	/* project_id */
	put_unaligned_le32(hwinfo,       buf + 8);	/* revision (json hwinfo) */
	put_unaligned_le32(cdt.major,    buf + 12);
	put_unaligned_le32(cdt.minor,    buf + 16);
	put_unaligned_le32(cdt.rf_sku,   buf + 20);	/* modem_sku */
	put_unaligned_le32(cdt.modem_hw, buf + 24);
	/* cpinfo0..2 (28/32/36) are runtime flags; cbd left them 0 on this device */
	put_unaligned_le32(cdt.rf_sub,   buf + 40);
	put_unaligned_le32(rfid,         buf + 44);	/* rf_config (json rfid) */
	put_unaligned_le32(cdt.product,  buf + 48);	/* reserved[0] */
	put_unaligned_le32(cdt.board,    buf + 52);	/* reserved[1] (stage) */
	put_unaligned_le32(cdt.variant,  buf + 56);	/* reserved[2] */
	/* reserved[3] (60) = 0 */

	cnode = of_find_node_by_path("/chosen/config");
	if (!cnode)
		return -ENOENT;
	imei = of_get_property(cnode, "imei1", &len);
	if (imei && len > 0)
		memcpy(buf + S5300_HANDOVER_IMEI_OFS, imei, min(len, 16));
	imei = of_get_property(cnode, "imei2", &len);
	if (imei && len > 0)
		memcpy(buf + S5300_HANDOVER_IMEI_OFS + 16, imei, min(len, 16));
	of_node_put(cnode);
	if (!buf[S5300_HANDOVER_IMEI_OFS])	/* imei1 is mandatory */
		return -ENODATA;

	/* cpsha is the 64-char hex CP signature; the trailing NUL is already set. */
	ret = request_firmware_direct(&sha, S5300_CPSHA_FW, sm->dev);
	if (ret)
		return ret;
	if (sha->size < 64) {
		release_firmware(sha);
		return -EINVAL;
	}
	memcpy(buf + S5300_HANDOVER_CPSIG_OFS, sha->data, 64);
	release_firmware(sha);

	dev_info(sm->dev,
		 "built handover: platform %u product %u revision %u rf_config %u\n",
		 cdt.platform, cdt.product, hwinfo, rfid);
	return 0;
}

static void s5300_init_control_messages(struct s5300_modem *sm)
{
	const struct firmware *fw;
	u8 handover[S5300_HANDOVER_SIZE];
	int i;

	writel(S5300_IPC_SRINFO_OFFSET, sm->ipc + S5300_IPC_SRINFO_OFS_PTR);
	writel(S5300_IPC_CAP_BASE, sm->ipc + S5300_IPC_CAP_OFS_PTR);
	/*
	 * Message words included (downstream init_ctrl_msg() in power_on_cp):
	 * stale VALID bits from a previous boot must not be readable once the
	 * MSI handler is live.
	 */
	writel(0, sm->ipc + S5300_IPC_AP2CP_MSG);
	writel(0, sm->ipc + S5300_IPC_CP2AP_MSG);
	/*
	 * ds_det=1 (0x4000) tells the CP to run its deep-sleep link PM (park the
	 * link when idle, cycle CP2AP_WAKEUP).  Debug knob s5300_ds_det=0 leaves it
	 * clear so the CP keeps the runtime link permanently up -- lets us exercise
	 * the FMT/RAW control path with no park/wake handshake at all (the wake is
	 * separately broken), isolating whether MAIN arms its runtime IPC.
	 */
	writel(s5300_ds_det ? S5300_IPC_DS_DET : 0,
	       sm->ipc + S5300_IPC_AP2CP_STATUS);
	writel(0, sm->ipc + S5300_IPC_CP2AP_STATUS);
	for (i = 0; i < S5300_IPC_CAP_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_CAP_BASE + 4 * i);

	/*
	 * Stage the handover block (HW/RF config + IMEIs + signature) MAIN reads
	 * to configure itself -- the vendor's IOCTL_HANDOVER_BLOCK_INFO, which the
	 * bare boot handshake otherwise omits, leaving MAIN unable to arm.  Prefer
	 * building it from device sources (DT + cpsha); fall back to a pre-staged
	 * cp_handover.bin.  Absent both, warn and continue (like cbd) -- the modem
	 * still reaches ONLINE but may not service the rings.
	 */
	if (s5300_build_handover(sm, handover) == 0) {
		memcpy_toio(sm->ipc + S5300_IPC_HANDOVER_OFS, handover,
			    S5300_HANDOVER_SIZE);
		dev_info(sm->dev, "built handover block from device sources\n");
	} else if (request_firmware_direct(&fw, S5300_HANDOVER_FW, sm->dev) == 0) {
		if (fw->size == S5300_HANDOVER_SIZE) {
			memcpy_toio(sm->ipc + S5300_IPC_HANDOVER_OFS,
				    fw->data, fw->size);
			dev_info(sm->dev, "staged handover block (%zu bytes)\n",
				 fw->size);
		} else {
			dev_warn(sm->dev,
				 "%s wrong size %zu (want %d); skipping\n",
				 S5300_HANDOVER_FW, fw->size, S5300_HANDOVER_SIZE);
		}
		release_firmware(fw);
	} else {
		dev_warn(sm->dev,
			 "no handover block (DT/cpsha or %s); MAIN may not arm\n",
			 S5300_HANDOVER_FW);
	}

	/*
	 * Fill the pktproc DL descriptor rings here (process context, before BL1)
	 * -- 13.8k iomem writes, far too much for the INIT_START hard IRQ.  The
	 * small info-region headers are published later, at their vendor phases:
	 * DL at INIT_START, UL at PHONE_START.  Skipped entirely in ap_cap=0
	 * (pure-legacy Steffen-mode) boots.
	 */
	if (s5300_pktproc && s5300_ap_cap)
		s5300_pktproc_fill_desc(sm);
}

/*
 * pktproc bring-up matched to the vendor sequence.  The vendor sets up the DL
 * rings at INIT_START (pktproc_init) and the UL rings at PHONE_START, inside the
 * capability handshake -- so we mirror that split.  Geometry is copied verbatim
 * from the vendor cpif boot log (4 DL queues sktbuf, 2 UL queues) so the CP DMAs
 * into exactly the layout it expects.  We do not move pktproc data (control
 * rides the legacy rings, only rmnet uses pktproc) -- the regions just have to
 * be valid so MAIN arms and the CP's DL/UL DMA lands in the reserved carveout.
 *
 * s5300_pktproc_fill_desc() does the heavy sktbuf-descriptor fill (13.8k iomem
 * writes) once at boot in process context; the two publish helpers write only
 * the small info-region headers + q_info, so they are safe to call from the
 * INIT_START / PHONE_START hard-IRQ handlers.
 */
static void s5300_pktproc_fill_desc(struct s5300_modem *sm)
{
	void __iomem *desc;
	u32 cp_buff;
	int q, j;

	if (!sm->pktproc)
		return;

	/* zero both info regions so a later publish writes onto clean state */
	memset_io(sm->pktproc + S5300_PKTPROC_DL_INFO_OFS, 0, SZ_4K);
	memset_io(sm->pktproc + S5300_PKTPROC_UL_INFO_OFS, 0, SZ_4K);

	/*
	 * DL sktbuf descriptors: cp_data_paddr is the low 36 bits of the first
	 * u64 (32-bit addr fits the low word).  Point each at its own
	 * true_packet_size slot so any CP downlink DMA lands in-carveout.
	 */
	for (q = 0; q < S5300_PKTPROC_DL_NUM_QUEUE; q++) {
		cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_BUFF_OFS +
			  q * S5300_PKTPROC_DL_Q_BUFF_SZ;
		desc = sm->pktproc + S5300_PKTPROC_DL_DESC_OFS +
		       q * S5300_PKTPROC_DL_Q_DESC_SZ;
		memset_io(desc, 0, S5300_PKTPROC_DL_Q_DESC_SZ);
		for (j = 0; j < S5300_PKTPROC_DL_NUM_DESC; j++) {
			void __iomem *d = desc + j * S5300_PKTPROC_DESC_SKTBUF_SZ;
			u32 ctrl = 0;

			/*
			 * Address in bytes 0-3; the control byte (byte 5) carries
			 * the ring-boundary markers, so a HEAD/RINGEND-aware CP arms
			 * the ring.  writel(ctrl << 8) lands ctrl in byte 5 and zeros
			 * paddr[35:32] (byte 4) and status (byte 6).
			 */
			if (j == 0)
				ctrl |= S5300_PKTPROC_CTRL_HEAD;
			if (j == S5300_PKTPROC_DL_NUM_DESC - 1)
				ctrl |= S5300_PKTPROC_CTRL_RINGEND;
			writel(cp_buff + j * S5300_PKTPROC_DL_TRUE_PKT, d);
			writel(ctrl << 8, d + 4);
		}
	}
}

/* DL info region (struct pktproc_info_v2), published at INIT_START. */
static void s5300_pktproc_publish_dl(struct s5300_modem *sm)
{
	void __iomem *info, *qinfo;
	u32 cp_desc, cp_buff, hdr;
	int q;

	if (!sm->pktproc) {
		dev_warn(sm->dev,
			 "no pktproc carveout; MAIN may reach ONLINE but not arm\n");
		return;
	}

	info = sm->pktproc + S5300_PKTPROC_DL_INFO_OFS;
	for (q = 0; q < S5300_PKTPROC_DL_NUM_QUEUE; q++) {
		cp_desc = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_DESC_OFS +
			  q * S5300_PKTPROC_DL_Q_DESC_SZ;
		cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_BUFF_OFS +
			  q * S5300_PKTPROC_DL_Q_BUFF_SZ;
		/* q_info[q] at info + 4 (after the 4-byte header), 20 bytes each */
		qinfo = info + 4 + q * 20;
		writel(cp_desc, qinfo + 0);
		writel(S5300_PKTPROC_DL_NUM_DESC, qinfo + 4);
		writel(cp_buff, qinfo + 8);
		/*
		 * fore_ptr is the AP "buffers-available" pointer.  fill_desc armed
		 * all num_desc slots, so offer num_desc-1 (circ leaves one slot as
		 * the empty gap); the drain re-arms from here as the CP consumes.
		 */
		writel(S5300_PKTPROC_DL_NUM_DESC - 1, qinfo + 12);
		writel(0, qinfo + 16);		/* rear_ptr (CP producer) */
		sm->dl_fore[q] = S5300_PKTPROC_DL_NUM_DESC - 1;
		sm->dl_done[q] = 0;
	}
	/* header last: num_queues:4 | desc_mode:2 | irq_mode:2 | max_packet_size:16 */
	hdr = (S5300_PKTPROC_DL_NUM_QUEUE & 0xf) |
	      ((S5300_PKTPROC_DL_DESC_MODE & 0x3) << 4) |
	      ((0u & 0x3) << 6) |
	      ((S5300_PKTPROC_DL_MAX_PKT & 0xffff) << 8);
	writel(hdr, info);
	dev_info(sm->dev, "published DL pktproc: %d queues\n",
		 S5300_PKTPROC_DL_NUM_QUEUE);
}

/* UL info region (struct pktproc_info_ul), published at PHONE_START. */
static void s5300_pktproc_publish_ul(struct s5300_modem *sm)
{
	void __iomem *info, *qinfo;
	u32 cp_desc, cp_buff;
	int q, ndesc;

	if (!sm->pktproc)
		return;

	info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFS;
	for (q = 0; q < S5300_PKTPROC_UL_NUM_QUEUE; q++) {
		/* vendor UL queue geometry (asymmetric), 32-byte descriptors */
		if (q == 0) {
			ndesc = S5300_PKTPROC_UL_Q0_NUM_DESC;
			cp_desc = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_DESC_OFS;
			cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_BUFF_OFS;
		} else {
			ndesc = S5300_PKTPROC_UL_Q1_NUM_DESC;
			cp_desc = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_DESC_OFS +
				  S5300_PKTPROC_UL_Q0_NUM_DESC *
				  S5300_PKTPROC_DESC_UL_SZ;
			cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_UL_BUFF_OFS +
				  S5300_PKTPROC_UL_Q0_NUM_DESC *
				  S5300_PKTPROC_UL_Q0_MAX_PKT;
		}
		/*
		 * struct pktproc_info_ul: header u32 + cp_quota u32, then
		 * q_info_ul[] -> q_info[q] at info + 8 + q*20.
		 */
		qinfo = info + 8 + q * 20;
		writel(cp_desc, qinfo + 0);
		writel(ndesc, qinfo + 4);
		writel(cp_buff, qinfo + 8);
		writel(0, qinfo + 12);		/* fore_ptr (AP producer) */
		writel(0, qinfo + 16);		/* rear_ptr (CP consumer) */
	}
	/*
	 * Header: only num_queues is AP-owned (vendor create_ul writes just that).
	 * end_bit_owner (word0 bit24) and cp_quota (info + 4) are written by the CP
	 * once it sees the PKTPROC_UL capability and are read back at PHONE_START --
	 * do not write them here or the CP's values get clobbered.
	 */
	writel(S5300_PKTPROC_UL_NUM_QUEUE & 0xf, info);
	dev_info(sm->dev, "published UL pktproc: %d queues\n",
		 S5300_PKTPROC_UL_NUM_QUEUE);
}

/*
 * Downstream set_ap_capabilities(): publish the AP capability words at
 * INIT_START, before answering PIF_INIT_DONE, so the CP reads them as it comes
 * up.  Load-bearing: the CP only begins servicing the runtime IPC (the FMT/SIT
 * control channel and its unsolicited identity burst) once it sees a non-zero
 * AP capability; with the words left at zero the CP stays silent on FMT and the
 * SIT control port never round-trips.
 */
static void s5300_set_ap_capabilities(struct s5300_modem *sm)
{
	writel(s5300_ap_cap, sm->ipc + S5300_IPC_CAP_AP0);
	writel(0, sm->ipc + S5300_IPC_CAP_AP1);
	dev_info(sm->dev, "AP capability part0 %#x, CP part0 %#x\n", s5300_ap_cap,
		 readl(sm->ipc + S5300_IPC_CAP_BASE + 0x4));
}

/*
 * Downstream init_legacy_link(), run from the PHONE_START handler: clear the
 * queue pointers, then advertise the magic and access-enable words.
 */
static void s5300_init_ipc_queues(struct s5300_modem *sm)
{
	u32 magic, access;
	int i;

	sm->fmt_frame_seq = 0;
	sm->fmt_ch_seq = 0;
	sm->at_ch_seq = 0;
	sm->rfs_ch_seq = 0;

	writel(0, sm->ipc + S5300_IPC_MAGIC);
	writel(0, sm->ipc + S5300_IPC_ACCESS);
	for (i = 0; i < S5300_IPC_Q_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);
	writel(S5300_IPC_MAGIC_VALUE, sm->ipc + S5300_IPC_MAGIC);
	writel(1, sm->ipc + S5300_IPC_ACCESS);

	magic = readl(sm->ipc + S5300_IPC_MAGIC);
	access = readl(sm->ipc + S5300_IPC_ACCESS);
	if (magic != S5300_IPC_MAGIC_VALUE || access != 1)
		dev_err(sm->dev, "IPC init readback failed: magic %#x access %u\n",
			magic, access);
}

/* --- runtime control channel: umts_router AT commands over the RAW ring --- */

static u32 s5300_circ_space(u32 size, u32 in, u32 out)
{
	return (in >= out) ? size - (in - out) - 1 : out - in - 1;
}

static u32 s5300_circ_usage(u32 size, u32 in, u32 out)
{
	return (in >= out) ? in - out : size - (out - in);
}

/* Copy @len bytes into the ring buffer at byte offset @off, wrapping at @size. */
static void s5300_circ_write(void __iomem *buff, u32 size, u32 off,
			     const void *src, u32 len)
{
	u32 first = min(len, size - off);

	memcpy_toio(buff + off, src, first);
	if (first < len)
		memcpy_toio(buff, src + first, len - first);
}

/* Copy @len bytes out of the ring buffer at byte offset @off, wrapping at @size. */
static void s5300_circ_read(void *dst, void __iomem *buff, u32 size, u32 off,
			    u32 len)
{
	u32 first = min(len, size - off);

	memcpy_fromio(dst, buff + off, first);
	if (first < len)
		memcpy_fromio(dst + first, buff, len - first);
}

/*
 * Frame a control message with the 12-byte exynos link header for channel @ch,
 * push it into the NORM_RAW TX ring and signal the CP (ap2cp_msg SEND_RAW +
 * doorbell).  Single-frame only -- control messages are well under 2 KB.
 */
static int s5300_ipc_tx(struct s5300_modem *sm, u8 ch, const u8 *data, u32 len)
{
	u32 flen = ALIGN(S5300_SIT_HDR + len, 8);
	void __iomem *buff = sm->ipc + S5300_RAW_TXQ_BUFF;
	unsigned long flags;
	u8 hdr[S5300_SIT_HDR];
	u32 in, out;

	if (S5300_SIT_HDR + len > 2048)
		return -EMSGSIZE;

	spin_lock_irqsave(&sm->tx_lock, flags);

	in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_TXQ_TAIL);
	if (s5300_circ_space(S5300_RAW_TXQ_SIZE, in, out) < flen) {
		spin_unlock_irqrestore(&sm->tx_lock, flags);
		return -EAGAIN;
	}

	put_unaligned_le16(S5300_SIT_SYNC, hdr + 0);
	put_unaligned_le16(sm->frame_seq++, hdr + 2);
	put_unaligned_le16(S5300_SIT_CFG_SINGLE, hdr + 4);
	put_unaligned_le16(S5300_SIT_HDR + len, hdr + 6);
	hdr[8] = ch;
	hdr[9] = ++sm->at_ch_seq;
	hdr[10] = 0;
	hdr[11] = 0;

	s5300_circ_write(buff, S5300_RAW_TXQ_SIZE, in, hdr, S5300_SIT_HDR);
	s5300_circ_write(buff, S5300_RAW_TXQ_SIZE,
			 (in + S5300_SIT_HDR) % S5300_RAW_TXQ_SIZE, data, len);
	/* the CP polls the head; order the payload ahead of the advance */
	dma_wmb();
	writel((in + flen) % S5300_RAW_TXQ_SIZE, sm->ipc + S5300_RAW_TXQ_HEAD);

	spin_unlock_irqrestore(&sm->tx_lock, flags);

	dev_info(sm->dev, "raw tx: ch %u len %u, txq head %#x->%#x tail %#x\n",
		 ch, len, in, (in + flen) % S5300_RAW_TXQ_SIZE,
		 readl(sm->ipc + S5300_RAW_TXQ_TAIL));
	s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_SEND_RAW);
	return 0;
}

static int s5300_wwan_start(struct wwan_port *port)
{
	return 0;
}

static void s5300_wwan_stop(struct wwan_port *port)
{
}

static int s5300_wwan_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5300_modem *sm = wwan_port_get_drvdata(port);
	int ret;

	if (!sm->online)
		return -ENODEV;
	ret = s5300_ipc_tx(sm, S5300_CH_AT, skb->data, skb->len);
	if (ret) {
		dev_info(sm->dev, "AT tx failed: %d\n", ret);
		return ret;
	}
	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5300_wwan_ops = {
	.start	= s5300_wwan_start,
	.stop	= s5300_wwan_stop,
	.tx	= s5300_wwan_tx,
};

/* Free space (bytes) in the FMT txq; 0 if the CP left the pointers corrupt. */
static u32 s5300_fmt_txq_space(struct s5300_modem *sm)
{
	u32 in = readl(sm->ipc + S5300_FMT_TXQ_HEAD);
	u32 out = readl(sm->ipc + S5300_FMT_TXQ_TAIL);

	if (in >= S5300_FMT_TXQ_SIZE || out >= S5300_FMT_TXQ_SIZE)
		return 0;
	return s5300_circ_space(S5300_FMT_TXQ_SIZE, in, out);
}

/*
 * Frame one app message onto the FMT ring (SIT control plane at ch 0xF5 or the
 * oem/GEMS chardev at ch 0x82) and signal the CP (ap2cp_msg SEND_FMT + doorbell).
 * Both post-ONLINE FMT writers funnel through here, so the txq head RMW and the
 * shared fmt_frame_seq are serialised by tx_lock (shared with the RAW producers);
 * each caller owns its @ch_seq.  Returns -EBUSY on a full ring so the oem writer
 * can wait for space and retry; the SIT port just propagates it.
 */
static int s5300_fmt_ring_tx(struct s5300_modem *sm, u8 ch, u8 *ch_seq,
			     u32 max, const u8 *data, u32 len)
{
	u32 flen = ALIGN(S5300_SIT_HDR + len, 8);
	void __iomem *buff = sm->ipc + S5300_FMT_TXQ_BUFF;
	unsigned long flags;
	u8 hdr[S5300_SIT_HDR];
	u32 in, out;

	if (!READ_ONCE(sm->online))
		return -ENODEV;
	if (len == 0 || len > max)
		return -EMSGSIZE;

	spin_lock_irqsave(&sm->tx_lock, flags);

	in = readl(sm->ipc + S5300_FMT_TXQ_HEAD);
	out = readl(sm->ipc + S5300_FMT_TXQ_TAIL);
	if (in >= S5300_FMT_TXQ_SIZE || out >= S5300_FMT_TXQ_SIZE) {
		spin_unlock_irqrestore(&sm->tx_lock, flags);
		dev_err_ratelimited(sm->dev, "fmt txq ptr oob (in %#x out %#x)\n",
				    in, out);
		return -EIO;
	}
	if (s5300_circ_space(S5300_FMT_TXQ_SIZE, in, out) < flen) {
		spin_unlock_irqrestore(&sm->tx_lock, flags);
		return -EBUSY;
	}

	put_unaligned_le16(S5300_SIT_SYNC, hdr + 0);
	put_unaligned_le16(sm->fmt_frame_seq++, hdr + 2);
	put_unaligned_le16(S5300_SIT_CFG_SINGLE, hdr + 4);
	put_unaligned_le16(S5300_SIT_HDR + len, hdr + 6);
	hdr[8] = ch;
	hdr[9] = ++*ch_seq;
	hdr[10] = 0;
	hdr[11] = 0;

	s5300_circ_write(buff, S5300_FMT_TXQ_SIZE, in, hdr, S5300_SIT_HDR);
	s5300_circ_write(buff, S5300_FMT_TXQ_SIZE,
			 (in + S5300_SIT_HDR) % S5300_FMT_TXQ_SIZE, data, len);
	dma_wmb();
	writel((in + flen) % S5300_FMT_TXQ_SIZE, sm->ipc + S5300_FMT_TXQ_HEAD);

	spin_unlock_irqrestore(&sm->tx_lock, flags);

	s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_SEND_FMT);
	return 0;
}

static int s5300_ctrl_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5300_modem *sm = wwan_port_get_drvdata(port);
	int ret;

	ret = s5300_fmt_ring_tx(sm, S5300_CH_FMT, &sm->fmt_ch_seq, S5300_FMT_MAX,
				skb->data, skb->len);
	if (ret)
		return ret;
	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5300_ctrl_ops = {
	.start	= s5300_wwan_start,
	.stop	= s5300_wwan_stop,
	.tx	= s5300_ctrl_tx,
};

/* --- generic channel chardev (oem_ipc1 today) ---------------------------- */

static int s5300_chardev_open(struct inode *inode, struct file *file)
{
	struct s5300_chardev *cd = container_of(file->private_data,
						struct s5300_chardev, miscdev);

	file->private_data = cd;
	return 0;
}

/* One read() returns exactly one received frame's payload (datagram-like). */
static ssize_t s5300_chardev_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct s5300_chardev *cd = file->private_data;
	struct sk_buff *skb;
	size_t n;

	skb = skb_dequeue(&cd->rxq);
	if (!skb) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(cd->read_wq,
					     (skb = skb_dequeue(&cd->rxq))))
			return -ERESTARTSYS;
	}

	n = min(count, (size_t)skb->len);
	if (copy_to_user(buf, skb->data, n)) {
		/* Keep the message for a retry rather than losing it. */
		skb_queue_head(&cd->rxq, skb);
		return -EFAULT;
	}
	kfree_skb(skb);
	return n;
}

/*
 * One write() is one app message.  It fills at most one FMT ring slot, so on a
 * full ring (-EBUSY) block until the CP drains it (fmt_tx_wq, woken from the IRQ
 * handler) rather than dropping the frame; O_NONBLOCK maps that to -EAGAIN.
 */
static ssize_t s5300_chardev_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct s5300_chardev *cd = file->private_data;
	struct s5300_modem *sm = cd->sm;
	u32 needed;
	u8 *kbuf;
	int ret;

	if (count == 0)
		return 0;
	if (count > cd->tx_max)
		return -EMSGSIZE;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;
	if (copy_from_user(kbuf, buf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	needed = round_up(S5300_SIT_HDR + count, 8);
	for (;;) {
		ret = s5300_fmt_ring_tx(sm, cd->channel, &cd->ch_seq, cd->tx_max,
					kbuf, count);
		if (ret != -EBUSY)
			break;
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		ret = wait_event_interruptible_timeout(sm->fmt_tx_wq,
				s5300_fmt_txq_space(sm) >= needed, HZ);
		if (ret == 0)
			ret = -ETIMEDOUT;
		if (ret < 0)
			break;
		/* Space appeared; retry the send. */
	}

	kfree(kbuf);
	return ret < 0 ? ret : (ssize_t)count;
}

static __poll_t s5300_chardev_poll(struct file *file, poll_table *wait)
{
	struct s5300_chardev *cd = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;	/* TX backpressure lives in write() */

	poll_wait(file, &cd->read_wq, wait);
	if (!skb_queue_empty(&cd->rxq))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations s5300_chardev_fops = {
	.owner		= THIS_MODULE,
	.open		= s5300_chardev_open,
	.read		= s5300_chardev_read,
	.write		= s5300_chardev_write,
	.poll		= s5300_chardev_poll,
};

/*
 * Frame an RFS file message onto the NORM_RAW ring (ch 0x29) and ring the RAW
 * data doorbell (SEND_RAW).  Mirrors s5300_ipc_tx() but with the RFS channel id
 * and its own per-channel sequence; the boot std_dl writer is quiescent
 * post-ONLINE so the ring is ours.
 */
static int s5300_rfs_tx(struct s5300_modem *sm, const u8 *data, u32 len)
{
	u32 flen = ALIGN(S5300_SIT_HDR + len, 8);
	void __iomem *buff = sm->ipc + S5300_RAW_TXQ_BUFF;
	unsigned long flags;
	u8 hdr[S5300_SIT_HDR];
	u32 in, out;

	if (len == 0 || len > S5300_RFS_MAX)
		return -EMSGSIZE;

	spin_lock_irqsave(&sm->tx_lock, flags);

	in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_TXQ_TAIL);
	if (in >= S5300_RAW_TXQ_SIZE || out >= S5300_RAW_TXQ_SIZE ||
	    s5300_circ_space(S5300_RAW_TXQ_SIZE, in, out) < flen) {
		spin_unlock_irqrestore(&sm->tx_lock, flags);
		return -EAGAIN;
	}

	put_unaligned_le16(S5300_SIT_SYNC, hdr + 0);
	put_unaligned_le16(sm->frame_seq++, hdr + 2);
	put_unaligned_le16(S5300_SIT_CFG_SINGLE, hdr + 4);
	put_unaligned_le16(S5300_SIT_HDR + len, hdr + 6);
	hdr[8] = S5300_CH_RFS;
	hdr[9] = ++sm->rfs_ch_seq;
	hdr[10] = 0;
	hdr[11] = 0;

	s5300_circ_write(buff, S5300_RAW_TXQ_SIZE, in, hdr, S5300_SIT_HDR);
	s5300_circ_write(buff, S5300_RAW_TXQ_SIZE,
			 (in + S5300_SIT_HDR) % S5300_RAW_TXQ_SIZE, data, len);
	dma_wmb();
	writel((in + flen) % S5300_RAW_TXQ_SIZE, sm->ipc + S5300_RAW_TXQ_HEAD);

	spin_unlock_irqrestore(&sm->tx_lock, flags);

	s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_SEND_RAW);
	return 0;
}

static int s5300_rfs_port_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5300_modem *sm = wwan_port_get_drvdata(port);
	int ret;

	if (!sm->online)
		return -ENODEV;
	ret = s5300_rfs_tx(sm, skb->data, skb->len);
	if (ret)
		return ret;
	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5300_rfs_ops = {
	.start	= s5300_wwan_start,
	.stop	= s5300_wwan_stop,
	.tx	= s5300_rfs_port_tx,
};

/*
 * Answer the CP's RFS file reads (carrier config + NV) in the kernel instead of
 * forwarding /dev/wwan0rfs0 to a userspace daemon.  Default on; rfs_server=0
 * hands the port back to userspace.
 */
static bool s5300_rfs_server = true;
module_param_named(rfs_server, s5300_rfs_server, bool, 0644);
MODULE_PARM_DESC(rfs_server, "answer the CP's RFS reads in-kernel (0 = /dev/wwan0rfs0)");

/* One queued CP request, handed from the hard-IRQ demux to s5300_rfs_work. */
struct s5300_rfs_frame {
	struct list_head	node;
	u32			len;
	u8			data[];
};

/* Send an RFS app frame, retrying while the RAW tx ring is momentarily full. */
static void s5300_rfs_send(struct s5300_modem *sm, const u8 *frame, u32 len)
{
	int i;

	for (i = 0; i < 200; i++) {
		if (s5300_rfs_tx(sm, frame, len) != -EAGAIN)
			return;
		usleep_range(1000, 2000);
	}
	dev_warn_ratelimited(sm->dev, "RFS tx stalled (ring full)\n");
}

/* OP_STATUS(3): status=success, fid, and (on OPEN) the file size. */
static void s5300_rfs_op_status(struct s5300_modem *sm, u16 token, u32 fid,
				u32 extra)
{
	u8 r[20];

	put_unaligned_le16(S5300_RFS_C_STATUS, r + 0);
	put_unaligned_le16(token, r + 2);
	put_unaligned_le32(12, r + 4);
	put_unaligned_le32(0, r + 8);		/* status 0 = success */
	put_unaligned_le32(fid, r + 0xc);
	put_unaligned_le32(extra, r + 0x10);
	s5300_rfs_send(sm, r, sizeof(r));
}

static void s5300_rfs_close_file(struct s5300_rfs_file *e)
{
	if (e->fw)
		release_firmware(e->fw);
	memset(e, 0, sizeof(*e));
}

/* Serve the next READ chunk for fid, or OP_STATUS when the read is done. */
static void s5300_rfs_read_step(struct s5300_modem *sm, u32 fid)
{
	struct s5300_rfs_file *e = &sm->rfs_files[fid];
	u32 clen = min_t(size_t, e->rremain, S5300_RFS_CHUNK);
	u8 *tx = sm->rfs_txbuf;

	if (!clen || !e->fw || e->roff + clen > e->fw->size) {
		s5300_rfs_op_status(sm, e->token, fid, 0);
		return;
	}
	put_unaligned_le16(S5300_RFS_C_READ, tx + 0);
	put_unaligned_le16(e->token, tx + 2);
	put_unaligned_le32(12 + clen, tx + 4);
	put_unaligned_le32(fid, tx + 8);
	put_unaligned_le32(e->roff, tx + 0xc);
	put_unaligned_le32(clen, tx + 0x10);
	memcpy(tx + 0x14, e->fw->data + e->roff, clen);
	s5300_rfs_send(sm, tx, 0x14 + clen);
	e->roff += clen;
	e->rremain -= clen;
}

/* Ask the CP for the next write chunk for fid (the data is sunk + discarded). */
static void s5300_rfs_write_req(struct s5300_modem *sm, u32 fid)
{
	struct s5300_rfs_file *e = &sm->rfs_files[fid];
	u32 clen = min_t(size_t, e->wremain, S5300_RFS_CHUNK);
	u8 r[20];

	put_unaligned_le16(S5300_RFS_C_WRITE, r + 0);
	put_unaligned_le16(e->token, r + 2);
	put_unaligned_le32(12, r + 4);
	put_unaligned_le32(fid, r + 8);
	put_unaligned_le32(e->woff, r + 0xc);
	put_unaligned_le32(clen, r + 0x10);
	s5300_rfs_send(sm, r, sizeof(r));
}

/* Map + open a whitelisted OPEN(path) request read-only from firmware. */
static int s5300_rfs_open_path(struct s5300_modem *sm, struct s5300_rfs_file *e,
			       const u8 *path, u32 plen)
{
	char name[256], fw[288];
	const char *rel;
	int ret;

	if (plen >= sizeof(name))
		return -ENAMETOOLONG;
	memcpy(name, path, plen);
	name[plen] = '\0';

	/* Whitelist: carrier config only, no path traversal (like rfsd). */
	if (strstr(name, "..") ||
	    (!strstr(name, "carrierconfig") && !strstr(name, "confpack")))
		return -EPERM;
	rel = strstr(name, "carrierconfig/");
	if (!rel)
		return -EINVAL;
	rel += strlen("carrierconfig/");
	scnprintf(fw, sizeof(fw), "%s/%s", S5300_CC_DIR, rel);
	ret = request_firmware_direct(&e->fw, fw, sm->dev);
	dev_dbg(sm->dev, "RFS serve %s: %d\n", fw, ret);
	return ret;
}

/* NV files served read-only from the staged efs partition. */
static const char *s5300_rfs_nv_name(u32 fid)
{
	switch (fid) {
	case 1:	return "google/s5400/efs/nv_normal.bin";
	case 2:	return "google/s5400/efs/nv_protected.bin";
	case 3:	return "google/s5400/efs/nv_user.bin";
	default: return NULL;
	}
}

static void s5300_rfs_open_nv(struct s5300_modem *sm, struct s5300_rfs_file *e,
			      u32 fid)
{
	const char *name = s5300_rfs_nv_name(fid);

	e->used = true;
	if (name && !request_firmware_direct(&e->fw, name, sm->dev))
		e->size = e->fw->size;
}

/* Handle one CP RFS request (process context, single-threaded via rfs_work). */
static void s5300_rfs_handle(struct s5300_modem *sm, const u8 *f, u32 n)
{
	u16 cmd = get_unaligned_le16(f), token = get_unaligned_le16(f + 2);
	u32 fid, off, iolen, iotype, clen, plen;
	struct s5300_rfs_file *e;

	switch (cmd) {
	case S5300_RFS_C_OPEN:			/* 4: open a path */
		if (n < 0x10)
			return;
		fid = get_unaligned_le32(f + 8);
		plen = get_unaligned_le32(f + 0xc);
		if (fid >= S5300_RFS_MAXFID)
			return;
		if (0x10 + plen > n)
			plen = n - 0x10;
		e = &sm->rfs_files[fid];
		s5300_rfs_close_file(e);
		e->used = true;
		if (!s5300_rfs_open_path(sm, e, f + 0x10, plen) && e->fw)
			e->size = e->fw->size;
		s5300_rfs_op_status(sm, token, fid, e->size);
		return;
	case S5300_RFS_C_OPEN_FID:		/* 7: open an NV file by id */
		fid = get_unaligned_le32(f + 8);
		if (fid >= S5300_RFS_MAXFID)
			return;
		e = &sm->rfs_files[fid];
		s5300_rfs_close_file(e);
		s5300_rfs_open_nv(sm, e, fid);
		s5300_rfs_op_status(sm, token, fid, e->size);
		return;
	case S5300_RFS_C_IO:			/* 6: start a read or write */
		if (n < 0x18)
			return;
		fid = get_unaligned_le32(f + 8);
		off = get_unaligned_le32(f + 0xc);
		iolen = get_unaligned_le32(f + 0x10);
		iotype = get_unaligned_le32(f + 0x14);
		if (fid >= S5300_RFS_MAXFID)
			return;
		e = &sm->rfs_files[fid];
		/* NV files may be IO_REQUEST'd without a preceding OPEN_FID. */
		if (!e->used && s5300_rfs_nv_name(fid))
			s5300_rfs_open_nv(sm, e, fid);
		if (!e->used) {			/* never leave a request unanswered */
			s5300_rfs_op_status(sm, token, fid, 0);
			return;
		}
		e->token = token;
		if (iotype == S5300_RFS_IO_READ) {
			e->roff = off;
			e->rremain = iolen;
			s5300_rfs_read_step(sm, fid);
		} else if (iotype == S5300_RFS_IO_WRITE) {
			e->woff = off;
			e->wremain = iolen;
			s5300_rfs_write_req(sm, fid);
		} else {
			s5300_rfs_op_status(sm, token, fid, 0);
		}
		return;
	case S5300_RFS_C_READ:			/* 1: CP ack -> next chunk */
		if (n < 0x10)
			return;
		fid = get_unaligned_le32(f + 0xc);
		if (fid >= S5300_RFS_MAXFID || !sm->rfs_files[fid].used)
			return;
		e = &sm->rfs_files[fid];
		if (e->rremain)
			s5300_rfs_read_step(sm, fid);
		else
			s5300_rfs_op_status(sm, e->token, fid, 0);
		return;
	case S5300_RFS_C_WRITE:			/* 2: CP data -> discard */
		if (n < 0x14)
			return;
		fid = get_unaligned_le32(f + 0xc);
		if (fid >= S5300_RFS_MAXFID || !sm->rfs_files[fid].used)
			return;
		clen = get_unaligned_le32(f + 0x10);
		e = &sm->rfs_files[fid];
		e->woff += clen;
		e->wremain = e->wremain > clen ? e->wremain - clen : 0;
		if (e->wremain)
			s5300_rfs_write_req(sm, fid);
		else
			s5300_rfs_op_status(sm, e->token, fid, 0);
		return;
	case S5300_RFS_C_CLOSE:			/* 5: close + ack */
		fid = get_unaligned_le32(f + 8);
		if (fid >= S5300_RFS_MAXFID)
			return;
		s5300_rfs_close_file(&sm->rfs_files[fid]);
		s5300_rfs_op_status(sm, token, fid, 0);
		return;
	case S5300_RFS_C_STATUS:		/* 3: CP ack -- consume */
		return;
	default:
		dev_dbg(sm->dev, "RFS unknown cmd %#x\n", cmd);
		return;
	}
}

static void s5300_rfs_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem, rfs_work);
	struct s5300_rfs_frame *fr;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&sm->rfs_lock, flags);
		fr = list_first_entry_or_null(&sm->rfs_rxq,
					      struct s5300_rfs_frame, node);
		if (fr)
			list_del(&fr->node);
		spin_unlock_irqrestore(&sm->rfs_lock, flags);
		if (!fr)
			break;
		s5300_rfs_handle(sm, fr->data, fr->len);
		kfree(fr);
	}
}

/* Copy a CP RFS request off the ring (hard IRQ) and queue it for rfs_work. */
static void s5300_rfs_enqueue(struct s5300_modem *sm, void __iomem *buff,
			      u32 off, u32 plen)
{
	struct s5300_rfs_frame *fr;
	unsigned long flags;

	if (plen < 8)
		return;
	fr = kmalloc(sizeof(*fr) + plen, GFP_ATOMIC);
	if (!fr)
		return;
	fr->len = plen;
	s5300_circ_read(fr->data, buff, S5300_RAW_RXQ_SIZE, off, plen);
	spin_lock_irqsave(&sm->rfs_lock, flags);
	list_add_tail(&fr->node, &sm->rfs_rxq);
	spin_unlock_irqrestore(&sm->rfs_lock, flags);
	queue_work(sm->rfs_wq, &sm->rfs_work);
}

/* Drop any queued RFS requests and release cached files (teardown). */
static void s5300_rfs_cleanup(struct s5300_modem *sm)
{
	struct s5300_rfs_frame *fr, *tmp;
	int i;

	list_for_each_entry_safe(fr, tmp, &sm->rfs_rxq, node) {
		list_del(&fr->node);
		kfree(fr);
	}
	for (i = 0; i < S5300_RFS_MAXFID; i++)
		s5300_rfs_close_file(&sm->rfs_files[i]);
}

/*
 * Drain the NORM_RAW rx ring and demux each 12-byte-framed message on its header
 * channel id: the RFS file channel (ch 0x29) goes to the RFS port, umts_router
 * AT (ch 0x15) to the AT port; other channels are dropped.  If the CP asked for
 * a receive ack on ANY runtime raw frame (REQ_ACK_RAW) -- RFS or AT -- answer
 * with RES_ACK_RAW like the FMT path; the AT channel needs its receive ack as
 * much as RFS or the CP's AT response flow stalls.  Runs from the hard IRQ
 * (@intval is the cp2ap_msg sampled there; skbs GFP_ATOMIC), and bounds the
 * alloc against a corrupt CP length.
 */
static void s5300_drain_raw_rxq(struct s5300_modem *sm, u32 intval)
{
	void __iomem *buff = sm->ipc + S5300_RAW_RXQ_BUFF;
	bool had_raw = false;
	u32 in, out;

	in = readl(sm->ipc + S5300_RAW_RXQ_HEAD);
	out = readl(sm->ipc + S5300_RAW_RXQ_TAIL);
	if (in >= S5300_RAW_RXQ_SIZE || out >= S5300_RAW_RXQ_SIZE) {
		dev_err_ratelimited(sm->dev, "raw rxq ptr oob (in %#x out %#x)\n",
				    in, out);
		return;
	}

	while (in != out) {
		u32 usage = s5300_circ_usage(S5300_RAW_RXQ_SIZE, in, out);
		u8 hdr[S5300_SIT_HDR];
		struct wwan_port *port;
		u32 flen, total, plen, max;
		u8 ch;

		if (usage < S5300_SIT_HDR)
			break;
		s5300_circ_read(hdr, buff, S5300_RAW_RXQ_SIZE, out, S5300_SIT_HDR);
		if (get_unaligned_le16(hdr) != S5300_SIT_SYNC) {
			dev_err_ratelimited(sm->dev, "raw rx bad sync %#x; flushing\n",
					    get_unaligned_le16(hdr));
			out = in;
			break;
		}
		flen = get_unaligned_le16(hdr + 6);	/* header + payload */
		total = ALIGN(flen, 8);
		if (flen < S5300_SIT_HDR || total > usage)
			break;			/* partial frame; wait for more */
		ch = hdr[8];
		plen = flen - S5300_SIT_HDR;

		if (ch == S5300_CH_RFS) {
			had_raw = true;
			if (s5300_rfs_server && plen <= S5300_RFS_MAX) {
				/* Serve the request in-kernel, not via the port. */
				s5300_rfs_enqueue(sm, buff,
						  (out + S5300_SIT_HDR) %
							  S5300_RAW_RXQ_SIZE,
						  plen);
				port = NULL;
				max = 0;
			} else {
				port = sm->rfs_port;
				max = S5300_RFS_MAX;
			}
		} else if (ch == S5300_CH_AT) {
			port = sm->at_port;
			max = SZ_2K;
			had_raw = true;
		} else {
			port = NULL;
			max = 0;
		}

		if (port && plen && plen <= max) {
			struct sk_buff *skb = alloc_skb(plen, GFP_ATOMIC);

			if (skb) {
				s5300_circ_read(skb_put(skb, plen), buff,
						S5300_RAW_RXQ_SIZE,
						(out + S5300_SIT_HDR) %
							S5300_RAW_RXQ_SIZE, plen);
				wwan_port_rx(port, skb);
			} else {
				dev_warn_ratelimited(sm->dev,
						     "raw rx drop ch %#x payload %u\n",
						     ch, plen);
			}
		}
		out = (out + total) % S5300_RAW_RXQ_SIZE;
	}

	/* order the payload reads ahead of the tail advance the CP watches */
	dma_wmb();
	writel(out, sm->ipc + S5300_RAW_RXQ_TAIL);

	if (had_raw && (intval & S5300_INT_REQ_ACK_RAW))
		s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_RES_ACK_RAW);
}

/*
 * Copy one channel frame's payload out of the FMT ring into an skb and queue it
 * for the chardev's reader (runs in the drain, hard IRQ: GFP_ATOMIC).  @out is
 * the byte offset of the frame's link header; the payload starts one header in.
 * The backlog is bounded so a daemon that never opens the node cannot exhaust
 * memory -- excess frames are dropped (a draining daemon never hits this).
 */
static void s5300_chardev_rx(struct s5300_chardev *cd, void __iomem *buff,
			     u32 ringsize, u32 out, u32 payload)
{
	struct sk_buff *skb;

	if (skb_queue_len(&cd->rxq) >= S5300_OEM_RXQ_MAX) {
		dev_warn_ratelimited(cd->sm->dev, "%s rxq full, dropping %u\n",
				     cd->miscdev.name, payload);
		return;
	}
	skb = alloc_skb(payload, GFP_ATOMIC);
	if (!skb) {
		dev_warn_ratelimited(cd->sm->dev,
				     "%s alloc_skb(%u) failed, dropping\n",
				     cd->miscdev.name, payload);
		return;
	}
	s5300_circ_read(skb_put(skb, payload), buff, ringsize,
			(out + S5300_SIT_HDR) % ringsize, payload);
	skb_queue_tail(&cd->rxq, skb);
	wake_up_interruptible(&cd->read_wq);
}

/*
 * Drain formatted IPC frames the CP left on the FMT rxq.  The CP routes several
 * IPC_FMT channels through this ring; ch 0xF5 is the SIT control port
 * (WWAN_PORT_SIT) and ch 0x82 is oem_ipc1 (/dev/umts_oem1).  Consume every frame
 * so the ring cannot stall, delivering 0xF5 to the SIT port and 0x82 to the oem
 * chardev and dropping the rest.  If the CP asked for a receive ack (REQ_ACK_FMT
 * in @intval, sampled in the hard IRQ), confirm with RES_ACK_FMT after advancing
 * the tail; the CP gates its secondary channels on that round-trip.  skbs are
 * GFP_ATOMIC.
 */
static void s5300_drain_fmt_rxq(struct s5300_modem *sm, u32 intval)
{
	void __iomem *buff = sm->ipc + S5300_FMT_RXQ_BUFF;
	bool had_data;
	u32 in, out;

	in = readl(sm->ipc + S5300_FMT_RXQ_HEAD);
	out = readl(sm->ipc + S5300_FMT_RXQ_TAIL);
	if (in >= S5300_FMT_RXQ_SIZE || out >= S5300_FMT_RXQ_SIZE) {
		dev_err_ratelimited(sm->dev, "fmt rxq ptr oob (in %#x out %#x)\n",
				    in, out);
		return;
	}
	had_data = (in != out);

	while (in != out) {
		u32 usage = s5300_circ_usage(S5300_FMT_RXQ_SIZE, in, out);
		u8 hdr[S5300_SIT_HDR];
		u32 flen, total, plen;

		if (usage < S5300_SIT_HDR)
			break;
		s5300_circ_read(hdr, buff, S5300_FMT_RXQ_SIZE, out, S5300_SIT_HDR);
		if (get_unaligned_le16(hdr) != S5300_SIT_SYNC) {
			dev_err_ratelimited(sm->dev, "fmt rx bad sync %#x; flushing\n",
					    get_unaligned_le16(hdr));
			out = in;
			break;
		}
		flen = get_unaligned_le16(hdr + 6);
		total = ALIGN(flen, 8);
		if (flen < S5300_SIT_HDR || total > usage)
			break;			/* partial frame; wait for more */
		plen = flen - S5300_SIT_HDR;

		if (hdr[8] == S5300_CH_OEM) {
			if (plen)
				s5300_chardev_rx(&sm->oem, buff,
						 S5300_FMT_RXQ_SIZE, out, plen);
		} else if (hdr[8] != S5300_CH_FMT) {
			dev_info_ratelimited(sm->dev,
					     "fmt rxq drop unhandled ch %#x payload %u\n",
					     hdr[8], plen);
		} else if (sm->ctrl_port && plen) {
			struct sk_buff *skb = alloc_skb(plen, GFP_ATOMIC);

			if (skb) {
				s5300_circ_read(skb_put(skb, plen), buff,
						S5300_FMT_RXQ_SIZE,
						(out + S5300_SIT_HDR) %
							S5300_FMT_RXQ_SIZE, plen);
				wwan_port_rx(sm->ctrl_port, skb);
			}
		}
		out = (out + total) % S5300_FMT_RXQ_SIZE;
	}

	dma_wmb();
	writel(out, sm->ipc + S5300_FMT_RXQ_TAIL);

	if (had_data && (intval & S5300_INT_REQ_ACK_FMT))
		s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_RES_ACK_FMT);
}

/*
 * PKTPROC data path.  The CP DMAs downlink PS packets into the DL sktbuf ring
 * and consumes uplink packets from the NORM UL ring; control traffic keeps
 * riding the legacy FMT/RAW rings.  DL is polled from the shared MSI-0 handler
 * (the CP's per-queue DL MSIs are the exclusive irq_mode vectors we do not
 * wire up); UL is driven by the rmnet0 netdev's ndo_start_xmit.
 */

/*
 * Drain filled DL sktbuf descriptors [done..rear) into skbs and re-arm the
 * freed slots.  No CP IOMMU on this SoC: the packet sits at
 * buff_vbase + true_packet_size*done, length from the descriptor; sniff the IP
 * version nibble and hand it to the raw-IP netdev.  Bounded by num_desc so a
 * garbled rear pointer cannot spin.  Runs in the MSI hard-IRQ once ONLINE.
 */
static void s5300_pktproc_dl_drain(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_DL_INFO_OFS;
	u32 n = S5300_PKTPROC_DL_NUM_DESC;
	int q;

	if (!sm->pktproc || !sm->ndev)
		return;

	for (q = 0; q < S5300_PKTPROC_DL_NUM_QUEUE; q++) {
		void __iomem *qinfo = info + 4 + q * 20;
		void __iomem *descs = sm->pktproc + S5300_PKTPROC_DL_DESC_OFS +
				      q * S5300_PKTPROC_DL_Q_DESC_SZ;
		void __iomem *buff = sm->pktproc + S5300_PKTPROC_DL_BUFF_OFS +
				     q * S5300_PKTPROC_DL_Q_BUFF_SZ;
		u32 cp_buff = S5300_PKTPROC_CP_BASE + S5300_PKTPROC_DL_BUFF_OFS +
			      q * S5300_PKTPROC_DL_Q_BUFF_SZ;
		u32 rear = readl(qinfo + 16) % n;
		u32 done = sm->dl_done[q];
		u32 fore = sm->dl_fore[q];
		u32 space, guard, i;

		if (done == rear)
			continue;
		/* order the descriptor/buffer reads after the rear-ptr sample */
		dma_rmb();

		for (guard = 0; done != rear && guard < n; guard++) {
			void __iomem *d = descs +
					  done * S5300_PKTPROC_DESC_SKTBUF_SZ;
			u32 len = readl(d + 8) & 0xffff;	/* length @ byte 8 */
			struct sk_buff *skb;

			if (len == 0 || len > S5300_PKTPROC_DL_MAX_PKT) {
				sm->ndev->stats.rx_length_errors++;
				goto next;
			}
			skb = netdev_alloc_skb(sm->ndev, len);
			if (!skb) {
				sm->ndev->stats.rx_dropped++;
				goto next;
			}
			memcpy_fromio(skb_put(skb, len),
				      buff + done * S5300_PKTPROC_DL_TRUE_PKT,
				      len);
			skb->protocol = htons((skb->data[0] >> 4) == 6 ?
					      ETH_P_IPV6 : ETH_P_IP);
			skb->dev = sm->ndev;
			skb_reset_mac_header(skb);
			skb_reset_network_header(skb);
			sm->ndev->stats.rx_packets++;
			sm->ndev->stats.rx_bytes += len;
			netif_rx(skb);
next:
			done = (done + 1 == n) ? 0 : done + 1;
		}
		sm->dl_done[q] = done;

		/* Re-arm the freed slots: circ_space(n, fore, done). */
		space = s5300_circ_space(n, fore, done);
		for (i = 0; i < space; i++) {
			void __iomem *d = descs +
					  fore * S5300_PKTPROC_DESC_SKTBUF_SZ;
			u32 ctrl = 0;

			if (fore == 0)
				ctrl |= S5300_PKTPROC_CTRL_HEAD;
			if (fore == n - 1)
				ctrl |= S5300_PKTPROC_CTRL_RINGEND;
			writel(cp_buff + fore * S5300_PKTPROC_DL_TRUE_PKT, d);
			writel(ctrl << 8, d + 4);
			fore = (fore + 1 == n) ? 0 : fore + 1;
		}
		if (space) {
			sm->dl_fore[q] = fore;
			/* re-armed descriptors visible before the fore advance */
			dma_wmb();
			writel(fore, qinfo + 12);
		}
	}
}

/*
 * At PHONE_START: read the CP-written end_bit_owner (UL info word0 bit24) and
 * cp_quota (word1), reset the TX cursor and enable UL transmit -- but only once
 * PKTPROC_UL is advertised (ap_cap bit0); otherwise the CP does not consume the
 * ring and an up'd rmnet0 would ring spurious doorbells.
 */
static void s5300_pktproc_ul_activate(struct s5300_modem *sm)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFS;
	void __iomem *qinfo = info + 8 + S5300_PKTPROC_UL_TXQ * 20;

	if (!sm->pktproc)
		return;

	sm->ul_end_bit_owner = (readl(info) >> 24) & 1;
	sm->ul_cp_quota = readl(info + 4) & 0xffff;
	sm->ul_done = 0;
	writel(0, qinfo + 12);		/* fore_ptr (AP producer) */
	writel(0, qinfo + 16);		/* rear_ptr (CP consumer) */
	sm->ul_active = s5300_pktproc && (s5300_ap_cap & 0x1);

	dev_info(sm->dev, "pktproc UL %s: end_bit_owner=%u cp_quota=%u\n",
		 sm->ul_active ? "active" : "provisioned (UL cap withheld)",
		 sm->ul_end_bit_owner, sm->ul_cp_quota);
}

/*
 * Transmit one skb on the NORM UL queue (q1): copy into the ring buffer, write
 * the 32-byte descriptor, publish fore_ptr and ring the CP with SEND_DATA
 * (== SEND_RAW; the CP scans both rings on this bit).  Serialised on tx_lock, so
 * ul_done needs no extra protection.  Returns false when the ring is full or the
 * frame is oversized -- the caller drops.
 */
static bool s5300_pktproc_ul_xmit(struct s5300_modem *sm, struct sk_buff *skb)
{
	void __iomem *info = sm->pktproc + S5300_PKTPROC_UL_INFO_OFS;
	void __iomem *qinfo = info + 8 + S5300_PKTPROC_UL_TXQ * 20;
	u32 n = S5300_PKTPROC_UL_Q1_NUM_DESC;
	u32 dsize = skb->len + S5300_PKTPROC_UL_CP_PADDING;
	u32 desc_base = S5300_PKTPROC_UL_DESC_OFS +
			S5300_PKTPROC_UL_Q0_NUM_DESC * S5300_PKTPROC_DESC_UL_SZ;
	u32 buff_base = S5300_PKTPROC_UL_BUFF_OFS +
			S5300_PKTPROC_UL_Q0_NUM_DESC * S5300_PKTPROC_UL_Q0_MAX_PKT;
	void __iomem *desc, *buf;
	u32 cp_buf, rear, slot;
	unsigned long flags;
	u8 last;

	if (dsize > S5300_PKTPROC_UL_MAX_PKT)
		return false;

	spin_lock_irqsave(&sm->tx_lock, flags);
	slot = sm->ul_done;
	rear = readl(qinfo + 16) % n;	/* rear_ptr (CP consumer) */
	/* circ_space(n, fore=slot, rear): need at least one free descriptor. */
	if (s5300_circ_space(n, slot, rear) < 1) {
		spin_unlock_irqrestore(&sm->tx_lock, flags);
		return false;
	}

	desc = sm->pktproc + desc_base + slot * S5300_PKTPROC_DESC_UL_SZ;
	buf = sm->pktproc + buff_base + slot * S5300_PKTPROC_UL_MAX_PKT;
	cp_buf = S5300_PKTPROC_CP_BASE + buff_base +
		 slot * S5300_PKTPROC_UL_MAX_PKT;
	last = sm->ul_end_bit_owner == S5300_PKTPROC_END_BIT_AP ? 1 : 0;

	memcpy_toio(buf, skb->data, skb->len);
	writel(dsize, desc + 0x0);		/* data_size (+CP_PADDING) */
	writel(dsize, desc + 0x4);		/* total_pkt_size */
	writel(cp_buf, desc + 0x8);		/* sktbuf_point[31:0] */
	writel(0, desc + 0xc);			/* sktbuf_point[35:32] + pbp */
	writel(last, desc + 0x10);		/* last_desc (bit0) */
	writel(S5300_CH_PDP_FIRST << 8, desc + 0x14);	/* lcid @ byte 21 */
	writel(0, desc + 0x18);
	writel(0, desc + 0x1c);

	slot = (slot + 1 == n) ? 0 : slot + 1;
	sm->ul_done = slot;
	wmb();				/* descriptor + payload land before fore */
	writel(slot, qinfo + 12);	/* fore_ptr (AP producer) */
	spin_unlock_irqrestore(&sm->tx_lock, flags);

	s5300_send_ipc_irq(sm, S5300_INT_VALID | S5300_INT_SEND_RAW);
	return true;
}

static int s5300_ndo_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int s5300_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t s5300_ndo_start_xmit(struct sk_buff *skb,
					struct net_device *ndev)
{
	struct s5300_modem *sm = *(struct s5300_modem **)netdev_priv(ndev);
	unsigned int len;

	/*
	 * ul_xmit copies only the linear head; no scatter-gather feature is
	 * advertised, so this is a no-op today, but guard the invariant.
	 */
	if (skb_linearize(skb)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	len = skb->len;

	if (sm->ul_active && s5300_pktproc_ul_xmit(sm, skb)) {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += len;
	} else {
		ndev->stats.tx_dropped++;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops s5300_netdev_ops = {
	.ndo_open	= s5300_ndo_open,
	.ndo_stop	= s5300_ndo_stop,
	.ndo_start_xmit	= s5300_ndo_start_xmit,
};

static void s5300_netdev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &s5300_netdev_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = 68;
	ndev->max_mtu = ETH_DATA_LEN;
	ndev->tx_queue_len = 1000;
	ndev->needs_free_netdev = true;
}

/*
 * Register the raw-IP data netdev (rmnet0).  Called from s5300_online: RX is
 * guarded by sm->ndev in the drain and TX by sm->ul_active, so the interface
 * appears exactly when the modem is ONLINE.  pktproc-off (legacy IPC) builds
 * skip it -- there is no UL/DL ring to back it.
 */
static void s5300_register_netdev(struct s5300_modem *sm)
{
	struct net_device *ndev;
	int ret;

	if (!sm->pktproc || !s5300_pktproc || !s5300_ap_cap || sm->ndev)
		return;

	ndev = alloc_netdev(sizeof(struct s5300_modem *), "rmnet%d",
			    NET_NAME_ENUM, s5300_netdev_setup);
	if (!ndev) {
		dev_err(sm->dev, "failed to allocate rmnet netdev\n");
		return;
	}
	*(struct s5300_modem **)netdev_priv(ndev) = sm;
	SET_NETDEV_DEV(ndev, sm->dev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(sm->dev, "failed to register rmnet netdev: %d\n", ret);
		free_netdev(ndev);
		return;
	}
	sm->ndev = ndev;
	dev_info(sm->dev, "raw-IP data netdev %s up (ch %#x)\n",
		 ndev->name, S5300_CH_PDP_FIRST);
}

/* Expose the runtime control channel once the CP is ONLINE (process context). */
static void s5300_online(struct s5300_modem *sm)
{
	if (sm->at_port)
		return;

	/*
	 * The link is already up and MAIN is running -- do NOT bounce it here.
	 * The CP has not parked the link and is not in the wakeup handshake, so
	 * a PERST-driven retrain would never complete (the CP only re-trains
	 * once it has itself parked the link to L2 and raised CP2AP_WAKEUP).
	 * Instead hand the link to CP-driven runtime PM: with ds_det=1 published
	 * (s5300_init_control_messages) the CP runs its deep-sleep handshake --
	 * parking the link when idle and cycling CP2AP_WAKEUP -- and each wake
	 * relink (s5300_pm_work) re-arms MAIN's runtime IPC.  Just arm the wakeup
	 * IRQ; the link stays up until the CP first parks it.
	 */
	mutex_lock(&sm->pcie_onoff_lock);
	if (!sm->pm_armed) {
		enable_irq(sm->cp2ap_irq);
		sm->pm_armed = true;
	}
	mutex_unlock(&sm->pcie_onoff_lock);

	if (sm->cp_active)
		dev_info(sm->dev, "CP2AP_PHONE_ACTIVE at ONLINE: %d\n",
			 gpiod_get_value_cansleep(sm->cp_active));
	/*
	 * boot_stage snapshot while alive: if it still reads "done" here but 0
	 * in the crash post-mortem, the CP re-entered its boot ROM (self-reset/
	 * watchdog); if it is already 0 here, MAIN zeroes it benignly at init.
	 */
	dev_info(sm->dev, "at ONLINE: boot_stage %#x sub %#x cafe %#x irq #%u\n",
		 readl(sm->msi + S5300_MSI_BOOT_STAGE),
		 readl(sm->msi + S5300_MSI_SUB_BOOT_STAGE),
		 readl(sm->msi + S5300_MSI_FLAG_CAFE), sm->irq_count);
	/*
	 * Runtime shared-mem control words vs the stock capture
	 * (~/Documents/claude-cpif-capture.rst): stock healthy = ap2cp_msg 0x82,
	 * cp2ap_msg 0x83, ap2cp_united 0x4000 (ds_det), cp2ap_united 0, cap ap
	 * 0x3 / cp 0x7.  Any mismatch here is a static-shmem lead for the +123ms
	 * self-reset; a full match means it's the dynamic bring-up we're missing.
	 */
	dev_info(sm->dev,
		 "at ONLINE shmem: ap2cp_msg %#x cp2ap_msg %#x ap2cp_united %#x cp2ap_united %#x | cap ap0 %#x cp0 %#x magic %#x access %#x\n",
		 readl(sm->ipc + S5300_IPC_AP2CP_MSG),
		 readl(sm->ipc + S5300_IPC_CP2AP_MSG),
		 readl(sm->ipc + S5300_IPC_AP2CP_STATUS),
		 readl(sm->ipc + 0x80c),
		 readl(sm->ipc + S5300_IPC_CAP_AP0),
		 readl(sm->ipc + S5300_IPC_CAP_BASE + 0x4),
		 readl(sm->ipc + S5300_IPC_MAGIC),
		 readl(sm->ipc + S5300_IPC_ACCESS));

	sm->at_port = wwan_create_port(sm->dev, WWAN_PORT_AT, &s5300_wwan_ops,
				       NULL, sm);
	if (IS_ERR(sm->at_port)) {
		dev_err(sm->dev, "failed to create AT port: %ld\n",
			PTR_ERR(sm->at_port));
		sm->at_port = NULL;
		return;
	}
	dev_info(sm->dev, "AT control port up (umts_router, ch %u)\n",
		 S5300_CH_AT);

	/*
	 * The SIT control plane (umts_ipc0, FMT ring, ch 0xF5).  The CP services
	 * the secondary channels (umts_router AT) only once this control plane is
	 * driven and its REQ_ACK round-trips answered, so expose it as its own
	 * port for a userspace RIL to run the modem init handshake.
	 */
	sm->ctrl_port = wwan_create_port(sm->dev, WWAN_PORT_SIT, &s5300_ctrl_ops,
					 NULL, sm);
	if (IS_ERR(sm->ctrl_port)) {
		dev_err(sm->dev, "failed to create SIT port: %ld\n",
			PTR_ERR(sm->ctrl_port));
		sm->ctrl_port = NULL;
	} else {
		dev_info(sm->dev, "SIT control port up (umts_ipc0, ch %u)\n",
			 S5300_CH_FMT);
	}

	/*
	 * The RFS file port (umts_rfs0, ch 0x29 on the NORM_RAW ring): once ONLINE
	 * the modem pulls its carrier config and persists NV over it, and rejects
	 * SETUP_DATA_CALL until an AP-side RFS server answers.  Expose it so a
	 * userspace server can serve those file requests.
	 */
	sm->rfs_port = wwan_create_port(sm->dev, WWAN_PORT_RFS, &s5300_rfs_ops,
					NULL, sm);
	if (IS_ERR(sm->rfs_port)) {
		dev_err(sm->dev, "failed to create RFS port: %ld\n",
			PTR_ERR(sm->rfs_port));
		sm->rfs_port = NULL;
	} else {
		dev_info(sm->dev, "RFS file port up (umts_rfs0, ch %#x)\n",
			 S5300_CH_RFS);
	}

	/*
	 * The raw-IP data path (rmnet0).  The CP DMAs downlink PS packets into the
	 * DL pktproc ring (drained from the MSI handler) and consumes uplink from
	 * the NORM UL ring; carrying real traffic still needs an active data call
	 * (SETUP_DATA_CALL over the SIT control plane) to assign an IP.
	 */
	s5300_register_netdev(sm);
}

static irqreturn_t s5300_irq_handler(int irq, void *data)
{
	struct s5300_modem *sm = data;
	u32 val, cmd;

	sm->irq_count++;
	val = readl(sm->ipc + S5300_IPC_CP2AP_MSG);
	/* Surface interrupt-value changes so post-bounce MSI liveness is visible. */
	if (val != sm->last_cp2ap) {
		dev_info(sm->dev, "IPC irq #%u: cp2ap %#x\n", sm->irq_count, val);
		sm->last_cp2ap = val;
	}
	/*
	 * Poll the DL pktproc ring on every MSI-0.  DL notifications are the
	 * exclusive irq_mode per-queue vectors we do not wire up and arrive as bare
	 * MSIs that need not set INT_VALID, so drain before the INT_VALID gate --
	 * otherwise a DL-only interrupt returns here having done nothing.
	 */
	if (sm->online)
		s5300_pktproc_dl_drain(sm);

	if (!(val & S5300_INT_VALID))
		return IRQ_HANDLED;

	if (!(val & S5300_CMD_VALID)) {
		/* Data notification (SEND_FMT/SEND_RAW), no command. */
		if (sm->online) {
			/*
			 * Both control planes and their REQ_ACK round-trips must be
			 * answered promptly, so drain + ack them here in the hard
			 * IRQ: the NORM_RAW ring (RFS files + umts_router AT) and
			 * the FMT ring (SIT control).
			 */
			s5300_drain_raw_rxq(sm, val);
			s5300_drain_fmt_rxq(sm, val);
			/* CP advancing the FMT txq tail frees space for oem writers. */
			wake_up_interruptible(&sm->fmt_tx_wq);
		} else
			/* Boot: std_dl acks; consume so the CP keeps draining. */
			writel(readl(sm->ipc + S5300_RAW_RXQ_HEAD),
			       sm->ipc + S5300_RAW_RXQ_TAIL);
		return IRQ_HANDLED;
	}

	cmd = val & S5300_CMD_MASK;
	switch (cmd) {
	case S5300_CMD_INIT_START:
		dev_info(sm->dev, "CP INIT_START\n");
		/*
		 * ap_cap=0 is Steffen-mode pure legacy IPC (proven on tegu:
		 * AT/SIT/RFS all ride the legacy rings, SMS works, no pktproc
		 * at all).  The capability check tolerates a lesser AP
		 * ((ap ^ cp) & ap == 0), so the CP falls back to legacy for
		 * everything.  Isolates whether MAIN's pktproc engine reading
		 * our DL/UL regions is what kills the CP ~125 ms after ONLINE.
		 */
		if (s5300_pktproc && s5300_ap_cap) {
			s5300_pktproc_publish_dl(sm);
			/*
			 * Provision the UL geometry here too, before the AP
			 * advertises PKTPROC_UL below: the CP consumes the UL rings
			 * as soon as it reads the capability, so q_info must be valid
			 * first.  The CP does not touch UL memory until the first TX
			 * doorbell, and it writes end_bit_owner/cp_quota afterwards
			 * (read back at PHONE_START by ul_activate).
			 */
			if (s5300_ap_cap & 0x1)
				s5300_pktproc_publish_ul(sm);
		}
		s5300_set_ap_capabilities(sm);
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_PIF_INIT_DONE));
		break;
	case S5300_CMD_PHONE_START:
		dev_info(sm->dev, "CP PHONE_START (AP cap %#x, CP cap %#x)\n",
			 readl(sm->ipc + S5300_IPC_CAP_AP0),
			 readl(sm->ipc + S5300_IPC_CAP_BASE + 0x4));
		if (!sm->online) {
			/*
			 * UL geometry was published at INIT_START; here just read
			 * back the CP-written end_bit_owner/cp_quota and enable TX.
			 */
			if (s5300_pktproc && (s5300_ap_cap & 0x1))
				s5300_pktproc_ul_activate(sm);
			s5300_init_ipc_queues(sm);
			sm->online = true;
		}
		/* Re-entrant PHONE_START just gets the INIT_END again. */
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_INIT_END));
		complete_all(&sm->init_done);
		break;
	case S5300_CMD_CRASH_RESET:
	case S5300_CMD_CRASH_EXIT:
		dev_err(sm->dev, "CP crash notification %#x (err_report %#x)\n",
			cmd, readl(sm->msi + S5300_MSI_ERR_REPORT));
		break;
	default:
		dev_warn(sm->dev, "unknown CP command %#x\n", cmd);
		break;
	}

	return IRQ_HANDLED;
}

/*
 * Force BAR0 to the doorbell page the downstream stack uses.  The write is
 * deliberately not 1M-aligned: whatever BAR0 size the current CP boot stage
 * exposes, the hardware aligns the value down to its own size, and the
 * doorbell register always decodes at bus address 0x14e60000 (downstream
 * pci_db_addr; its driver likewise programs this value into BAR0 and rings
 * that bus address through both boot phases).  Written behind the PCI core's
 * back (the core saw unassignable ROM BARs anyway); the modem never runs
 * with core-managed BARs downstream either.
 */
static int s5300_setup_doorbell(struct s5300_modem *sm)
{
	struct pci_bus_region region;
	/*
	 * pcibios_bus_to_resource() matches host-bridge windows by the
	 * resource type of the passed-in res, so it must be pre-typed MEM --
	 * zero flags match no window and the bus address comes back
	 * untranslated (seen on hardware as "cpu [??? 0x14e60000...]").
	 */
	struct resource res = { .flags = IORESOURCE_MEM };
	int i, ret;

	/*
	 * The PCI core could not place the mask ROM's six 1M BARs in the
	 * small CH0 window; drop them from resource management entirely so
	 * pci_enable_device() has nothing unclaimed to trip over, then
	 * program BAR0 directly (downstream s51xx_pcie_probe() does the
	 * same).
	 */
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		sm->pdev->resource[i].start = 0;
		sm->pdev->resource[i].end = 0;
		sm->pdev->resource[i].flags = 0;
	}
	ret = s5300_program_doorbell_bar(sm);
	if (ret)
		return ret;

	ret = s5300_open_bridge_window(sm);
	if (ret)
		return ret;

	region.start = sm->db_bus_addr;
	region.end = sm->db_bus_addr + SZ_4K - 1;
	pcibios_bus_to_resource(sm->pdev->bus, &res, &region);
	if (!res.start)
		return dev_err_probe(sm->dev, -EINVAL,
				     "doorbell bus address %#x maps to no CPU window\n",
				     sm->db_bus_addr);

	sm->doorbell = devm_ioremap(sm->dev, res.start, SZ_4K);
	if (!sm->doorbell)
		return -ENOMEM;

	dev_info(sm->dev, "doorbell at bus %#x, cpu %pR\n", sm->db_bus_addr,
		 &res);

	return 0;
}

static int s5300_poll_boot_stage(struct s5300_modem *sm, u32 target)
{
	u32 val;
	int i;

	for (i = 0; i < S5300_POLL_COUNT; i++) {
		val = readl(sm->msi + S5300_MSI_BOOT_STAGE);
		if (val == target)
			return 0;
		msleep(S5300_POLL_INTERVAL_MS);
	}

	dev_err(sm->dev,
		"boot_stage stuck at %#x (want %#x, err_report %#x)\n", val,
		target, readl(sm->msi + S5300_MSI_ERR_REPORT));
	return -ETIMEDOUT;
}

/*
 * Stage one BOOT sub-image into the boot slot, publish its address/size
 * through the MSI block and ring the message doorbell.  The vendor
 * set_cp_rom_boot_img() does exactly this for each of the two boot stages.
 */
static void s5300_send_boot_image(struct s5300_modem *sm, size_t off,
				  size_t size)
{
	memcpy_toio(sm->ipc + S5300_BOOT_IMG_OFFSET,
		    sm->pbl->data + off, size);
	writel(lower_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_LO);
	writel(upper_32_bits(sm->ipc_phys + S5300_BOOT_IMG_OFFSET),
	       sm->msi + S5300_MSI_IMG_ADDR_HI);
	writel(size, sm->msi + S5300_MSI_IMG_SIZE);

	/* The ROM reads the MSI target and image descriptor on the doorbell. */
	s5300_verify_msi_target(sm);
	s5300_send_doorbell(sm, S5300_DB_MSG);
}

static int s5300_poll_cp_wakeup(struct s5300_modem *sm)
{
	int i;

	for (i = 0; i < S5300_POLL_COUNT; i++) {
		if (gpiod_get_value_cansleep(sm->cp2ap_wakeup))
			return 0;
		msleep(S5300_POLL_INTERVAL_MS);
	}

	dev_err(sm->dev, "CP2AP_WAKEUP never asserted after the re-link\n");
	return -ETIMEDOUT;
}

/*
 * The CP firmware ships as a factory image (modem.bin) led by a table of
 * contents: 32-byte records { name[12], offset, load_addr, size, crc, misc },
 * the first of which ("TOC") spans the table. Locate the first-stage
 * bootloader ("BOOT") within it, so firmware-name can point at the unmodified
 * vendor modem.bin. A file that is not a TOC (e.g. an already-extracted BOOT
 * blob) is staged whole.
 */
struct s5300_toc_entry {
	char	name[12];
	__le32	offset;
	__le32	load_addr;
	__le32	size;
	__le32	crc;
	__le32	misc;
};

static void s5300_find_boot(struct s5300_modem *sm)
{
	const struct firmware *fw = sm->pbl;
	const struct s5300_toc_entry *toc = (const void *)fw->data;
	size_t i, n;

	/* Default: the file is already the raw BOOT image. */
	sm->boot_off = 0;
	sm->boot_size = fw->size;

	/* A factory image begins with a "TOC" record; pull BOOT out of it. */
	if (fw->size < sizeof(*toc) || strncmp(toc[0].name, "TOC", 4))
		return;

	n = min_t(size_t, le32_to_cpu(toc[0].size) / sizeof(*toc), 64);
	for (i = 0; i < n; i++) {
		size_t off, size;

		if (strncmp(toc[i].name, "BOOT", 5))
			continue;
		off = le32_to_cpu(toc[i].offset);
		size = le32_to_cpu(toc[i].size);
		if (off <= fw->size && size <= fw->size - off) {
			sm->boot_off = off;
			sm->boot_size = size;
			dev_info(sm->dev,
				 "modem.bin TOC: BOOT at +%#zx, %#zx bytes\n",
				 off, size);
		}
		return;
	}
	dev_warn(sm->dev, "no BOOT entry in TOC; staging the whole image\n");
}

/* Locate a TOC section by name; report its file offset, size and (opt) CRC. */
static bool s5300_find_section(struct s5300_modem *sm, const char *name,
			       size_t *off, size_t *size, u32 *crc)
{
	const struct firmware *fw = sm->pbl;
	const struct s5300_toc_entry *toc = (const void *)fw->data;
	size_t i, n, o, s;

	if (fw->size < sizeof(*toc) || strncmp(toc[0].name, "TOC", 4))
		return false;
	n = min_t(size_t, le32_to_cpu(toc[0].size) / sizeof(*toc), 64);
	for (i = 0; i < n; i++) {
		if (strncmp(toc[i].name, name, sizeof(toc[i].name)))
			continue;
		o = le32_to_cpu(toc[i].offset);
		s = le32_to_cpu(toc[i].size);
		if (!o || !s || o > fw->size || s > fw->size - o)
			return false;
		*off = o;
		*size = s;
		if (crc)
			*crc = le32_to_cpu(toc[i].crc);
		return true;
	}
	return false;
}

/*
 * Push one frame into the NORM_RAW TX ring.  Like the vendor write() this does
 * NOT wait for the CP to consume the frame -- the CP drains continuously by
 * polling txq.head (there is no doorbell).  The one thing it MUST do is keep the
 * CP's ack ring drained: the CP posts a std_dl ack per frame on the rxq and
 * stops consuming the txq once the rxq fills, so consuming acks (advancing
 * rxq.tail) is what keeps it moving.  We do that here while waiting for TX room
 * (and again in the IRQ handler / boot wait), which the old "wait on txq.tail
 * but drain the rxq only afterwards" pacing deadlocked against.
 */
static int s5300_ring_push(struct s5300_modem *sm, const void *buf, u32 len)
{
	u32 in, out, space, first, rxq0;
	int spins;

	for (spins = 100000; ; ) {
		/* Consume the CP's acks so it keeps draining the TX ring. */
		writel(readl(sm->ipc + S5300_RAW_RXQ_HEAD),
		       sm->ipc + S5300_RAW_RXQ_TAIL);

		in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
		out = readl(sm->ipc + S5300_RAW_TXQ_TAIL);
		space = (in >= out) ? S5300_RAW_TXQ_SIZE - (in - out) - 1
				    : out - in - 1;
		if (space >= len)
			break;
		/*
		 * A warm CP already has MAIN resident: instead of draining the
		 * boot ring it jumps straight to the IPC handshake and fires
		 * INIT_START..PHONE_START, completing init_done.  Stop pushing.
		 */
		if (completion_done(&sm->init_done))
			return -EALREADY;
		if (!spins--)
			return -ETIMEDOUT;
		usleep_range(100, 200);
	}

	rxq0 = readl(sm->ipc + S5300_RAW_RXQ_HEAD);

	first = min_t(u32, len, S5300_RAW_TXQ_SIZE - in);
	memcpy_toio(sm->ipc + S5300_RAW_TXQ_BUFF + in, buf, first);
	if (first < len)
		memcpy_toio(sm->ipc + S5300_RAW_TXQ_BUFF, buf + first,
			    len - first);
	/* order the payload ahead of the head update the CP polls */
	dma_wmb();
	in = (in + len) % S5300_RAW_TXQ_SIZE;
	writel(in, sm->ipc + S5300_RAW_TXQ_HEAD);

	/*
	 * Pace STRICTLY on the CP's per-frame ack, exactly like cbd (write one
	 * std_dl frame, then read its ack before writing the next).  Without
	 * this we run up to a whole ring ahead of the CP, overrun its frame
	 * pipeline and it faults partway through MAIN (err_report 0x20).  The CP
	 * posts one ack on the rxq per frame; wait for rxq.head to advance, then
	 * consume it.
	 */
	for (spins = 100000; spins--; ) {
		u32 rxq = readl(sm->ipc + S5300_RAW_RXQ_HEAD);

		if (rxq != rxq0) {
			writel(rxq, sm->ipc + S5300_RAW_RXQ_TAIL);
			return 0;
		}
		if (completion_done(&sm->init_done))
			return -EALREADY;
		usleep_range(50, 150);
	}
	return -ETIMEDOUT;
}

/* Fill the 12-byte SIT link header at the head of a frame. */
static void s5300_sit_header(struct s5300_modem *sm, u8 *f, u32 frame_len)
{
	put_unaligned_le16(S5300_SIT_SYNC, f + 0);
	put_unaligned_le16(sm->frame_seq++, f + 2);
	put_unaligned_le16(S5300_SIT_CFG_SINGLE, f + 4);
	put_unaligned_le16(frame_len, f + 6);
	f[8] = S5300_SIT_CH_BOOT;
	f[9] = sm->ch_seq++;
	f[10] = 0;
	f[11] = 0;
}

/* SIT-framed 4-byte std_dl control word (start/end of a section). */
static int s5300_send_ctrl(struct s5300_modem *sm, u8 *scratch, u32 word)
{
	u32 flen = S5300_SIT_HDR + 4;

	s5300_sit_header(sm, scratch, flen);
	put_unaligned_le32(word, scratch + S5300_SIT_HDR);
	memset(scratch + flen, 0, ALIGN(flen, 8) - flen);
	return s5300_ring_push(sm, scratch, ALIGN(flen, 8));
}

/* One std_dl data frame: SIT header + std_dl header + payload chunk, padded. */
static int s5300_send_chunk(struct s5300_modem *sm, u8 *scratch, u8 tag,
			    u32 total, u32 off, const u8 *payload, u32 plen)
{
	u32 flen = S5300_SIT_HDR + S5300_DL_HDR + plen;
	u8 *dl = scratch + S5300_SIT_HDR;

	s5300_sit_header(sm, scratch, flen);
	dl[0] = 0x0b | (tag << 4);
	dl[1] = 0xa1;
	put_unaligned_le16(plen + 8, dl + 2);
	put_unaligned_le32(total, dl + 4);
	put_unaligned_le32(off, dl + 8);
	memcpy(dl + S5300_DL_HDR, payload, plen);
	memset(scratch + flen, 0, ALIGN(flen, 8) - flen);
	return s5300_ring_push(sm, scratch, ALIGN(flen, 8));
}

/*
 * SIT-framed std_dl CRC-verify frame (8-byte payload: op, 0, CRC32).  cbd sends
 * this after MAIN's data and before its END word; without it the PBL never
 * validates MAIN and refuses to launch it (the CP stays stuck in BOOTING).
 */
static int s5300_send_verify(struct s5300_modem *sm, u8 *scratch, u8 tag,
			     u32 crc)
{
	u32 flen = S5300_SIT_HDR + 8;
	u8 *dl = scratch + S5300_SIT_HDR;

	s5300_sit_header(sm, scratch, flen);
	put_unaligned_le16(S5300_DL_SEC_VERIFY(tag), dl + 0);
	put_unaligned_le16(0, dl + 2);
	put_unaligned_le32(crc, dl + 4);
	memset(scratch + flen, 0, ALIGN(flen, 8) - flen);
	return s5300_ring_push(sm, scratch, ALIGN(flen, 8));
}

/*
 * Stream one firmware section (vendor std_dl framing): a start control word,
 * the chunked data frames, an optional CRC-verify frame (MAIN only), then an
 * end control word.  cbd brackets every section this way; the tag is the
 * sequential 1..N stream position, which the CP maps back to the section's TOC
 * entry (and thus its CP-DRAM load address).
 */
static int s5300_dl_section(struct s5300_modem *sm, u8 *scratch, u8 tag,
			    const u8 *data, u32 total, u32 verify_crc)
{
	u32 off = 0;
	int ret;

	ret = s5300_send_ctrl(sm, scratch, S5300_DL_SEC_START(tag));
	if (ret)
		return ret;

	while (off < total) {
		u32 plen = min_t(u32, total - off, S5300_DL_CHUNK);

		ret = s5300_send_chunk(sm, scratch, tag, total, off,
				       data + off, plen);
		if (ret)
			return ret;
		off += plen;
	}

	if (verify_crc) {
		ret = s5300_send_verify(sm, scratch, tag, verify_crc);
		if (ret)
			return ret;
	}

	return s5300_send_ctrl(sm, scratch, S5300_DL_SEC_END(tag));
}

static void s5300_dl_report(struct s5300_modem *sm, const char *what, int ret)
{
	dev_err(sm->dev,
		"%s stall %d: ring head %#x tail %#x rxq_head %#x | CP boot_stage %#x sub %#x err %#x cafe %#x\n",
		what, ret,
		readl(sm->ipc + S5300_RAW_TXQ_HEAD),
		readl(sm->ipc + S5300_RAW_TXQ_TAIL),
		readl(sm->ipc + S5300_RAW_RXQ_HEAD),
		readl(sm->msi + S5300_MSI_BOOT_STAGE),
		readl(sm->msi + S5300_MSI_SUB_BOOT_STAGE),
		readl(sm->msi + S5300_MSI_ERR_REPORT),
		readl(sm->msi + S5300_MSI_FLAG_CAFE));
}

/*
 * A cold CP's download server wants every std_dl stage its TOC lists, and the
 * stage number is the TOC `misc` field -- NOT a sequential 1..N.  On this image
 * the sections happen to be in misc order, so streaming them in this order with
 * a sequential tag lands each on its correct stage (hw-confirmed by cbd.trace):
 *   1 TOC   2 PSP   3 MAIN(+CRC)   4 APM   5 VSS   6 DBGCORE
 *   7 RF_CFG   8 NV_NORM   9 NV_PROT   10 REPLAY
 * Sending MAIN on any stage but 3 makes the CP load it to the wrong section's
 * DRAM address (e.g. PSP's small window) and fault mid-stream (err_report 0x20).
 * Only MAIN carries a CRC-verify frame (the PBL validates it before launch).
 */
static const struct s5300_dl_sec {
	const char	*toc_name;	/* NULL = the raw TOC table */
	const char	*fw_name;	/* non-NULL = request_firmware file */
	bool		verify;		/* send the CRC-verify frame (MAIN) */
	bool		replay;		/* build a FRESH replay tar (see below) */
	bool		rf_cfg;		/* auto-select from hardware_config.json */
} s5300_dl_secs[] = {
	{ NULL,      NULL,                                            false },
	{ "PSP",     NULL,                                            false },
	{ "MAIN",    NULL,                                            true  },
	{ "APM",     NULL,                                            false },
	{ "VSS",     NULL,                                            false },
	{ "DBGCORE", NULL,                                            false },
	{ "RF_CFG",  NULL,                                            false, false, true },
	{ "NV_NORM", "google/s5400/efs/nv_normal.bin",               false },
	{ "NV_PROT", "google/s5400/efs/nv_protected.bin",            false },
	/* fw_name is only the fallback if the fresh tar build fails. */
	{ "REPLAY",  "google/s5400/modem_userdata/replay_region.bin", false,
	  true },
};

/*
 * REPLAY must be a FRESH tar of the live modem_userdata/replay files on every
 * boot -- MAIN rejects a stale archive and then self-disables a few seconds
 * after ONLINE without ever servicing the IPC rings (found the hard way; also
 * documented in cbd-lite).  Downstream cbd assembles it at each boot; mirror
 * that in-kernel from the live dds.bin so no boot-script step is needed.
 *
 * Archive layout, byte-verified against the stock cbd std_dl stream: one
 * GNU-tar member "replay/dds.bin" (mode 0666, uid/gid radio/system 1001/1000,
 * magic "ustar  \0", chksum "%06o\0 "), content at +512, zero-padded to the
 * 512 KiB TOC section size.
 */
#define S5300_REPLAY_DDS_FW	"google/s5400/modem_userdata/replay/dds.bin"
#define S5300_REPLAY_SIZE	0x80000

static void *s5300_build_replay(struct s5300_modem *sm)
{
	const struct firmware *fw;
	unsigned int sum = 0;
	u8 *buf;
	int i, ret;

	ret = request_firmware(&fw, S5300_REPLAY_DDS_FW, sm->dev);
	if (ret) {
		dev_warn(sm->dev, "no %s (%d); using stale replay_region.bin\n",
			 S5300_REPLAY_DDS_FW, ret);
		return NULL;
	}
	if (fw->size > S5300_REPLAY_SIZE - 3 * 512) {
		dev_warn(sm->dev, "dds.bin too large (%#zx)\n", fw->size);
		release_firmware(fw);
		return NULL;
	}

	buf = kvzalloc(S5300_REPLAY_SIZE, GFP_KERNEL);
	if (!buf) {
		release_firmware(fw);
		return NULL;
	}

	memcpy(buf, "replay/dds.bin", 15);		/* name[100] */
	memcpy(buf + 100, "0000666", 8);		/* mode */
	memcpy(buf + 108, "0001751", 8);		/* uid: radio (1001) */
	memcpy(buf + 116, "0001750", 8);		/* gid: system (1000) */
	snprintf(buf + 124, 12, "%011zo", fw->size);	/* size */
	snprintf(buf + 136, 12, "%011llo",		/* mtime: fresh */
		 (unsigned long long)ktime_get_real_seconds());
	memset(buf + 148, ' ', 8);			/* chksum: spaces */
	buf[156] = '0';					/* typeflag: file */
	memcpy(buf + 257, "ustar  ", 8);		/* magic+version */
	memcpy(buf + 265, "radio", 6);			/* uname */
	memcpy(buf + 297, "system", 7);			/* gname */

	for (i = 0; i < 512; i++)
		sum += buf[i];
	snprintf(buf + 148, 8, "%06o", sum);		/* NUL lands at [154] */
	buf[155] = ' ';

	memcpy(buf + 512, fw->data, fw->size);
	dev_info(sm->dev, "built fresh replay tar (dds.bin %#zx bytes)\n",
		 fw->size);
	release_firmware(fw);
	return buf;
}

/*
 * One-shot gunzip of an in-memory blob into a fresh kvmalloc buffer (caller
 * kvfree).  The device modem partition stores RF_CFG_* gzipped and the kernel
 * firmware loader only auto-decompresses xz/zstd, so handle gzip here.  Mirrors
 * lib/decompress_inflate.c: strip the gzip header, then raw-inflate (the kernel
 * zlib_inflateInit2 rejects the +16 gzip window).  *out_size gets the length.
 */
static void *s5300_gunzip(struct s5300_modem *sm, const u8 *in, size_t in_size,
			  size_t *out_size)
{
	struct z_stream_s strm = {};
	const u8 *p = in, *end = in + in_size;
	u32 osize;
	u8 flg;
	void *out;
	int rc;

	if (in_size < 18 || in[0] != 0x1f || in[1] != 0x8b || in[2] != 0x08)
		return NULL;			/* not a gzip/deflate stream */
	flg = in[3];
	p += 10;				/* fixed gzip header */
	if (flg & 0x04) {			/* FEXTRA */
		if (p + 2 > end)
			return NULL;
		p += 2 + (p[0] | (p[1] << 8));
	}
	if (flg & 0x08)				/* FNAME (asciz) */
		while (p < end && *p++)
			;
	if (flg & 0x10)				/* FCOMMENT (asciz) */
		while (p < end && *p++)
			;
	if (flg & 0x02)				/* FHCRC */
		p += 2;
	if (p + 8 >= end)			/* deflate stream + 8-byte trailer */
		return NULL;

	osize = get_unaligned_le32(in + in_size - 4);	/* gzip ISIZE */
	if (!osize || osize > 32 * 1024 * 1024)
		return NULL;

	strm.workspace = kvmalloc(zlib_inflate_workspacesize(), GFP_KERNEL);
	if (!strm.workspace)
		return NULL;
	out = kvmalloc(osize, GFP_KERNEL);
	if (!out) {
		kvfree(strm.workspace);
		return NULL;
	}

	strm.next_in = (u8 *)p;
	strm.avail_in = end - 8 - p;		/* raw deflate, minus trailer */
	strm.next_out = out;
	strm.avail_out = osize;

	rc = zlib_inflateInit2(&strm, -MAX_WBITS);	/* raw deflate */
	if (rc == Z_OK)
		rc = zlib_inflate(&strm, Z_FINISH);
	zlib_inflateEnd(&strm);
	kvfree(strm.workspace);

	if (rc != Z_STREAM_END || strm.total_out != osize) {
		dev_warn(sm->dev, "RF_CFG gunzip failed (rc %d, %lu/%u)\n",
			 rc, strm.total_out, osize);
		kvfree(out);
		return NULL;
	}
	*out_size = osize;
	return out;
}

/*
 * Load an RF_CFG image named fw_path into a fresh buffer (caller kvfree).  Tries
 * the name as-is first (raw, or xz/zst which the loader decompresses); if absent,
 * tries fw_path.gz and gunzips it -- the device modem partition ships RF_CFG_*
 * gzipped.
 */
static int s5300_load_rf_cfg(struct s5300_modem *sm, const char *fw_path,
			     void **out, size_t *outsz)
{
	const struct firmware *fw;
	char gz[176];
	void *buf;

	if (request_firmware_direct(&fw, fw_path, sm->dev) == 0) {
		buf = kvmalloc(fw->size, GFP_KERNEL);
		if (buf) {
			memcpy(buf, fw->data, fw->size);
			*outsz = fw->size;
			*out = buf;
		}
		release_firmware(fw);
		return buf ? 0 : -ENOMEM;
	}

	scnprintf(gz, sizeof(gz), "%s.gz", fw_path);
	if (request_firmware_direct(&fw, gz, sm->dev))
		return -ENOENT;
	buf = s5300_gunzip(sm, fw->data, fw->size, outsz);
	release_firmware(fw);
	if (!buf)
		return -EIO;
	*out = buf;
	return 0;
}

/*
 * Pick this device's RF_CFG image, decompress it, and hand back a fresh buffer
 * in out/outsz (caller kvfree).  Uses the same cdt_hwid -> hardware_config.json
 * resolution as the handover build; the matched row's config_file basename is
 * loaded from S5300_RF_CFG_DIR.  Falls back to S5300_RF_CFG_FALLBACK on any miss.
 */
static int s5300_select_rf_cfg(struct s5300_modem *sm, void **out, size_t *outsz)
{
	struct s5300_cdt cdt;
	u32 rfid, hwinfo;
	char rf_cfg[80], path[160];

	if (s5300_resolve_hwcfg(sm, &cdt, &rfid, &hwinfo, rf_cfg,
				sizeof(rf_cfg)) == 0 && rf_cfg[0]) {
		scnprintf(path, sizeof(path), "%s/%s", S5300_RF_CFG_DIR, rf_cfg);
		dev_info(sm->dev, "RF_CFG auto-selected %s (rfid=%u hwinfo=%u)\n",
			 rf_cfg, rfid, hwinfo);
		return s5300_load_rf_cfg(sm, path, out, outsz);
	}
	dev_warn(sm->dev, "RF_CFG auto-select failed; using %s\n",
		 S5300_RF_CFG_FALLBACK);
	return s5300_load_rf_cfg(sm, S5300_RF_CFG_FALLBACK, out, outsz);
}

static int s5300_download_main(struct s5300_modem *sm)
{
	const struct s5300_toc_entry *toc = (const void *)sm->pbl->data;
	u8 *scratch;
	int i, ret = 0;

	scratch = kmalloc(S5300_SIT_HDR + S5300_DL_HDR + S5300_DL_CHUNK + 8,
			  GFP_KERNEL);
	if (!scratch)
		return -ENOMEM;

	/* The ring was armed in boot mode at boot_work() start; reset framing. */
	sm->frame_seq = 0;
	sm->ch_seq = 0;

	for (i = 0; i < ARRAY_SIZE(s5300_dl_secs); i++) {
		const struct s5300_dl_sec *d = &s5300_dl_secs[i];
		const struct firmware *fw = NULL;
		const char *name = d->toc_name ? d->toc_name : "TOC";
		const u8 *data;
		size_t off = 0, size;
		u32 crc = 0;
		u8 tag = i + 1;		/* cbd's sequential stage nibble 1..7 */

		void *replay_buf = NULL;
		void *rf_cfg_buf = NULL;

		/*
		 * Warm CP: it went ONLINE without draining the ring.  On a
		 * cold-cycled boot this can only be a stale resident MAIN
		 * (handshaked against the previous session's rings); keep
		 * streaming so it restarts against ours.
		 */
		if (!sm->cold_cycled && completion_done(&sm->init_done)) {
			dev_info(sm->dev, "CP online mid-boot; skipping download\n");
			goto done;
		}

		if (d->replay)
			replay_buf = s5300_build_replay(sm);

		if (!d->toc_name) {
			/* Stage 1: the raw TOC table (first record spans it). */
			data = sm->pbl->data;
			size = le32_to_cpu(toc[0].size);
		} else if (replay_buf) {
			/* REPLAY: freshly built tar (MAIN rejects a stale one). */
			data = replay_buf;
			size = S5300_REPLAY_SIZE;
		} else if (d->rf_cfg) {
			/*
			 * RF_CFG: auto-selected via hardware_config.json, and
			 * gunzipped -- the caller owns rf_cfg_buf (kvfree below).
			 */
			ret = s5300_select_rf_cfg(sm, &rf_cfg_buf, &size);
			if (ret) {
				dev_err(sm->dev,
					"RF_CFG select failed (tag %u): %d\n",
					tag, ret);
				goto done;
			}
			data = rf_cfg_buf;
		} else if (d->fw_name) {
			/* NV (or replay fallback): a vendor partition file. */
			ret = request_firmware(&fw, d->fw_name, sm->dev);
			if (ret) {
				dev_err(sm->dev, "cold boot needs %s (tag %u): %d\n",
					d->fw_name, tag, ret);
				goto done;
			}
			data = fw->data;
			size = fw->size;
		} else if (s5300_find_section(sm, d->toc_name, &off, &size, &crc)) {
			/* MAIN/VSS/APM carried in modem.bin. */
			data = sm->pbl->data + off;
		} else {
			ret = dev_err_probe(sm->dev, -ENOENT,
					    "%s not in TOC\n", d->toc_name);
			goto done;
		}

		dev_info(sm->dev, "streaming %s (tag %u, %#zx bytes%s)\n",
			 name, tag, size, d->verify ? ", CRC-verified" : "");
		ret = s5300_dl_section(sm, scratch, tag, data, size,
				       d->verify ? crc : 0);
		kvfree(replay_buf);
		kvfree(rf_cfg_buf);
		if (fw)
			release_firmware(fw);
		if (ret == -EALREADY)
			goto online;
		if (ret) {
			s5300_dl_report(sm, name, ret);
			goto done;
		}
		/* Drain probe: after the first section, has the CP consumed it? */
		if (i == 0)
			dev_info(sm->dev,
				 "TOC pushed: ring head %#x tail %#x rxq_head %#x\n",
				 readl(sm->ipc + S5300_RAW_TXQ_HEAD),
				 readl(sm->ipc + S5300_RAW_TXQ_TAIL),
				 readl(sm->ipc + S5300_RAW_RXQ_HEAD));
	}

	/*
	 * All ten sections delivered: push the standalone std_dl finalise word
	 * (cbd's 0xa400, CP acks 0xc400 -- "image complete, boot MAIN"), then
	 * ring the msg doorbell.  The CP drains sections by polling and stops
	 * once it has them all, so unlike a section frame the finalise needs a
	 * nudge to wake the CP back up to consume it.  Non-fatal: whether or not
	 * it drains, fall through to wait for the INIT_START handshake.
	 */
	if (completion_done(&sm->init_done))
		goto online;
	dev_info(sm->dev, "download complete; sending boot command\n");
	{
		u32 in, flen = ALIGN(S5300_SIT_HDR + 4, 8);

		s5300_sit_header(sm, scratch, S5300_SIT_HDR + 4);
		put_unaligned_le32(S5300_DL_FINISH, scratch + S5300_SIT_HDR);
		memset(scratch + S5300_SIT_HDR + 4, 0, flen - (S5300_SIT_HDR + 4));

		in = readl(sm->ipc + S5300_RAW_TXQ_HEAD);
		memcpy_toio(sm->ipc + S5300_RAW_TXQ_BUFF + in, scratch, flen);
		dma_wmb();
		writel((in + flen) % S5300_RAW_TXQ_SIZE,
		       sm->ipc + S5300_RAW_TXQ_HEAD);

		/* Nudge the CP to process the completion / boot MAIN. */
		s5300_send_doorbell(sm, S5300_DB_MSG);

		for (i = 0; i < 500; i++) {
			if (readl(sm->ipc + S5300_RAW_TXQ_TAIL) ==
			    readl(sm->ipc + S5300_RAW_TXQ_HEAD))
				break;
			if (completion_done(&sm->init_done))
				goto online;
			usleep_range(1000, 2000);
		}
		dev_info(sm->dev,
			 "boot cmd: txq %#x/%#x rxq %#x/%#x cp2ap %#x (drained %s)\n",
			 readl(sm->ipc + S5300_RAW_TXQ_HEAD),
			 readl(sm->ipc + S5300_RAW_TXQ_TAIL),
			 readl(sm->ipc + S5300_RAW_RXQ_HEAD),
			 readl(sm->ipc + S5300_RAW_RXQ_TAIL),
			 readl(sm->ipc + S5300_IPC_CP2AP_MSG),
			 i < 500 ? "yes" : "no");
	}
	ret = 0;
	goto done;

online:
	dev_info(sm->dev, "CP online mid-stream; download done\n");
	ret = 0;
done:
	kfree(scratch);
	return ret;
}

/*
 * Drive AP2CP_PARTIAL_RST_N (cp-partial-rst-gpios, gpp21-6) HIGH = deasserted,
 * mirroring the vendor power_on_cp().  This active-low line is a partial-reset
 * request the CP samples; if we leave it floating/low MAIN takes it as "AP wants
 * a partial reset" and resets itself ~200 ms after ONLINE (a purely CP-internal
 * reset, seen with a bare online).  The descriptor is requested output-high at
 * probe; re-assert here so a warm reload also lands it high.
 */
static void s5300_drive_partial_rst(struct s5300_modem *sm)
{
	if (!sm->cp_partial_rst)
		return;
	gpiod_set_value_cansleep(sm->cp_partial_rst, 1);
}

static void s5300_boot_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem,
					      boot_work);
	struct pci_dev *pdev = sm->pdev;
	size_t bl1_size, btl_off, btl_size;
	unsigned long flags;
	int ret, i, attempt;

	s5300_drive_partial_rst(sm);

	if (sm->adopt) {
		/*
		 * Warm-reload / self-recovery adopt: MAIN is alive at Gen3 but
		 * the AP and CP ring views are DESYNCED -- our head/tail are
		 * stale (left at the previous session's download-era offsets),
		 * MAIN has its own, so a fresh AT frame lands where MAIN is not
		 * reading and the txq tail never advances.  Re-run the vendor
		 * init_legacy_link (magic + zero every FMT/RAW head/tail) so
		 * both sides restart the rings from 0, and clear the ctrl-msg
		 * words, before exposing the ports.
		 */
		writel(0, sm->ipc + S5300_IPC_AP2CP_MSG);
		writel(0, sm->ipc + S5300_IPC_CP2AP_MSG);
		s5300_init_ipc_queues(sm);
		spin_lock_irqsave(&sm->lock, flags);
		sm->online = true;
		sm->link_up = true;
		spin_unlock_irqrestore(&sm->lock, flags);
		complete_all(&sm->init_done);
		dev_info(sm->dev, "adopted live MAIN (rings resynced)\n");
		s5300_online(sm);
		return;
	}

	for (attempt = 0; ; attempt++) {
		/*
		 * Clear ALL of the vendor clear_boot_stage() MSI status fields,
		 * not just boot_stage: after a CP crash the bootloader leaves
		 * flag_cafe=0xcafe and sub_boot_stage=0x1fff behind, and the ROM
		 * reads them on the next boot -- a stale flag_cafe wedges the CP
		 * at INIT_START (it never advances to PHONE_START).  Only a true
		 * power-off cleared it before; clearing them here does the same.
		 */
		writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
		writel(0, sm->msi + S5300_MSI_SUB_BOOT_STAGE);
		writel(0, sm->msi + S5300_MSI_FLAG_CAFE);
		writel(0, sm->msi + S5300_MSI_OTP_VERSION);
		writel(0, sm->msi + S5300_MSI_DB_LOOP_CNT);
		writel(0, sm->msi + S5300_MSI_DB_RECEIVED);
		writel(0, sm->msi + S5300_MSI_BOOT_SIZE);
		s5300_init_control_messages(sm);

		/*
		 * Arm the shared-memory IPC region up front (vendor
		 * init_legacy_link in link_start_normal_boot, before BL1): the
		 * CP polls the SIT boot magic at IPC+0 to recognise the region,
		 * and MEM_ACCESS gates AP visibility.  Both must be live before
		 * the bootloader hands over to MAIN and starts the post-bounce
		 * INIT_START handshake.
		 */
		writel(S5300_SHM_BOOT_MAGIC, sm->ipc + S5300_IPC_MAGIC);
		writel(1, sm->ipc + S5300_IPC_MEM_ACCESS);
		/*
		 * Zero every FMT+RAW queue head/tail word (vendor
		 * init_legacy_link clears them all before boot).  We had only
		 * cleared the RAW download ring, leaving the FMT queue pointers
		 * at their power-on 0xffffffff; the CP's IPC validation rejects
		 * that and never leaves the bootloader.
		 */
		for (i = 0; i < S5300_IPC_Q_WORDS; i++)
			writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);

		/* Config (forced BAR0, MSI cap) must survive the re-link. */
		pci_save_state(pdev);

		/*
		 * Two-stage boot (vendor start_normal_boot_bl1() then
		 * start_normal_boot_bootloader()): the BOOT section is split,
		 * each half fed through the same boot slot with its own message
		 * doorbell.  Stage 1 (BL1, first S5300_BL1_IMG_SIZE bytes)
		 * drives boot_stage to BL1_DONE; stage 2 (the remainder, the
		 * "bootloader") drives it to DONE.
		 */
		bl1_size = min_t(size_t, S5300_BL1_IMG_SIZE, sm->boot_size);
		btl_off  = sm->boot_off + bl1_size;
		btl_size = sm->boot_size - bl1_size;

		dev_info(sm->dev, "BL1 download (%#zx bytes at %pap+%#x)\n",
			 bl1_size, &sm->ipc_phys, S5300_BOOT_IMG_OFFSET);
		s5300_send_boot_image(sm, sm->boot_off, bl1_size);
		ret = s5300_poll_boot_stage(sm, S5300_BOOT_STAGE_BL1_DONE);
		if (!ret) {
			dev_info(sm->dev,
				 "BL1 up; bootloader download (%#zx bytes)\n",
				 btl_size);
			s5300_send_boot_image(sm, btl_off, btl_size);
			ret = s5300_poll_boot_stage(sm, S5300_BOOT_STAGE_DONE);
		}
		if (!ret)
			break;
		if (attempt >= 1)
			goto out_fw;

		/*
		 * Boot wedged (boot_stage stuck, sometimes err_report 0xc) --
		 * escalate to a genuine CP power cycle (down to boot ROM) which
		 * also relinks, then restore the endpoint config the reset wiped
		 * (doorbell BAR0, MSI capability -- the ROM derives its
		 * boot-status DMA target from the MSI address registers) and
		 * retry the download once.
		 */
		dev_warn(sm->dev, "boot wedged; escalating to a CP power cycle\n");
		ret = zumapro_pcie_modem_power_cycle(sm->rc_dev);
		if (ret) {
			dev_err(sm->dev, "post-power-cycle relink failed: %d\n",
				ret);
			goto out_fw;
		}
		pci_restore_state(pdev);
		pci_set_master(pdev);
		s5300_open_bridge_window(sm);
		zumapro_pcie_set_msi_target(sm->rc_dev, sm->msi_phys);
		s5300_verify_msi_target(sm);
	}

	/* Clear boot_stage before the re-link (vendor clears it here too). */
	writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
	dev_info(sm->dev,
		 "bootloader up (boot_stage done); re-establishing the link\n");

	/*
	 * Vendor start_normal_boot_bootloader(): after the bootloader the CP
	 * drops the PCIe link and re-requests it via CP2AP_WAKEUP.  Only a FULL
	 * teardown + re-establishment (pcie_poweroff -> pcie_poweron, NOT an
	 * in-place retrain) re-arms the CP's inbound DMA -- it will not read the
	 * MAIN image ring or raise INIT_START until the AP tears the link down,
	 * brings it back, restores the endpoint config and rings the link-ack.
	 * Mirror the vendor PM save/restore around the drop.
	 */
	if (pci_set_power_state(pdev, PCI_D3hot))
		dev_warn(sm->dev, "could not D3hot the CP before the re-link\n");
	ret = zumapro_pcie_modem_link_down(sm->rc_dev);
	if (ret)
		goto out_fw;
	ret = s5300_poll_cp_wakeup(sm);
	if (ret)
		goto out_fw;
	ret = zumapro_pcie_modem_link_up(sm->rc_dev);
	if (ret) {
		dev_err(sm->dev, "link re-establishment failed: %d\n", ret);
		goto out_fw;
	}
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_set_master(pdev);

	/*
	 * link_up()'s host_init re-ran dw_pcie_setup_rc, which reset the RC's MSI
	 * receive address back to its default page.  Re-drive it to the modem MSI
	 * carveout (vendor pcie_set_msi_ctrl_addr(), called right after
	 * pcie_poweron) so the CP's INIT_START MSI lands where we can see it --
	 * without this the CP drains the ring but its INIT_START notification is
	 * delivered to the wrong address and the handshake never completes.  Then
	 * re-open the bridge window and re-arm the endpoint-side MSI target.
	 */
	zumapro_pcie_set_msi_target(sm->rc_dev, sm->msi_phys);
	s5300_open_bridge_window(sm);
	s5300_verify_msi_target(sm);

	/*
	 * Ring the PCIe link-ack doorbell the CP bootloader waits for (vendor
	 * s5100_poweron_pcie: intval_ap2cp_pcie_link_ack = 0x1000e, DT index 0xe
	 * -- a DIFFERENT doorbell from the boot-image msg doorbell) so it leaves
	 * the bootloader to read the MAIN ring and raise INIT_START.
	 */
	s5300_send_doorbell(sm, S5300_DB_LINK_ACK);

	/*
	 * Warm CP: the warm reset preserved the CP's self-powered DRAM, so MAIN
	 * is resident.  Like the vendor, the bootloader just re-runs it and
	 * raises INIT_START on its own -- no download at all.  Wait for that.
	 *
	 * But never after a cold cycle.  The CP's DDR is self-powered, so a
	 * stale MAIN survives even a rail cycle and the bootloader happily
	 * re-runs it -- an instance that handshaked against the PREVIOUS
	 * session's rings.  It drains our txq and answers nothing (an AT write
	 * is consumed, no reply).  A cold cycle must always re-stream MAIN so
	 * it re-runs INIT_START/PHONE_START against the rings we just cleared.
	 */
	if (!sm->cold_cycled) {
		dev_info(sm->dev, "link re-established; waiting for resident MAIN\n");
		if (wait_for_completion_timeout(&sm->init_done, 5 * HZ)) {
			release_firmware(sm->pbl);
			sm->pbl = NULL;
			goto online;
		}
	} else {
		/* Drop any handshake a stale resident MAIN raised mid-boot. */
		reinit_completion(&sm->init_done);
		dev_info(sm->dev,
			 "cold cycle: forcing a fresh MAIN download\n");
	}

	/*
	 * Cold CP (MAIN not resident -- e.g. first boot after a full CP power
	 * loss): fall back to streaming the firmware sections into the NORM_RAW
	 * ring (cbd's post-bounce std_dl download).  Then wait again.
	 */
	dev_info(sm->dev, "no resident MAIN; streaming firmware to the CP\n");
	ret = s5300_download_main(sm);
	release_firmware(sm->pbl);
	sm->pbl = NULL;
	if (ret) {
		dev_err(sm->dev, "firmware download failed: %d\n", ret);
		return;
	}

	dev_info(sm->dev, "firmware streamed; waiting for CP INIT_START\n");
	/*
	 * The CP acks every boot frame into the RX ring; cbd consumes each ack
	 * (bootdump_read) as it goes, so by the end rxq.tail == rxq.head.  We
	 * blasted the frames without reading the acks, so drain the RX ring now
	 * -- the CP appears to gate "download accepted -> boot MAIN" on the AP
	 * having consumed them.  Keep draining while we wait, and log the CP's
	 * progress so a stuck handshake is visible.
	 */
	{
		int s;

		u32 fmt_dumped = 0;

		for (s = 0; s < 150; s++) {
			u32 fh = readl(sm->ipc + S5300_FMT_RXQ_HEAD);

			/*
			 * MSI-loss fallback: a deep (S5910 DCXO) cold cycle
			 * resets the CP's MSI capability, so its INIT_START /
			 * PHONE_START land in cp2ap_msg but no interrupt fires
			 * and we wedge.  Poll the command handler here so the
			 * handshake still completes; harmless when the MSI does
			 * fire (the handler no-ops on an already-processed cmd).
			 */
			s5300_irq_handler(0, sm);

			writel(readl(sm->ipc + S5300_RAW_RXQ_HEAD),
			       sm->ipc + S5300_RAW_RXQ_TAIL);
			/* If the CP posted an FMT control message, dump it once. */
			if (fh && fh != fmt_dumped) {
				u32 n = min_t(u32, fh, 64);

				print_hex_dump(KERN_INFO, "s5xxx fmt: ",
					       DUMP_PREFIX_OFFSET, 16, 1,
					       sm->ipc + S5300_FMT_RXQ_BUFF, n, false);
				fmt_dumped = fh;
			}
			if (completion_done(&sm->init_done))
				break;
			if (s % 10 == 0)
				dev_info(sm->dev,
					 "wait %ds: irq %u cp2ap %#x sub %#x txq %#x/%#x rxq %#x/%#x fmt %#x/%#x\n",
					 s / 10, sm->irq_count,
					 readl(sm->ipc + S5300_IPC_CP2AP_MSG),
					 readl(sm->msi + S5300_MSI_SUB_BOOT_STAGE),
					 readl(sm->ipc + S5300_RAW_TXQ_HEAD),
					 readl(sm->ipc + S5300_RAW_TXQ_TAIL),
					 readl(sm->ipc + S5300_RAW_RXQ_HEAD),
					 readl(sm->ipc + S5300_RAW_RXQ_TAIL),
					 fh, readl(sm->ipc + S5300_FMT_RXQ_TAIL));
			msleep(100);
		}
	}

	if (!completion_done(&sm->init_done)) {
		dev_err(sm->dev,
			"CP handshake timed out (cp2ap %#x boot_stage %#x sub %#x err %#x cafe %#x)\n",
			readl(sm->ipc + S5300_IPC_CP2AP_MSG),
			readl(sm->msi + S5300_MSI_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_SUB_BOOT_STAGE),
			readl(sm->msi + S5300_MSI_ERR_REPORT),
			readl(sm->msi + S5300_MSI_FLAG_CAFE));
		return;
	}

online:
	dev_info(sm->dev, "CP is ONLINE\n");
	s5300_online(sm);
	return;

out_fw:
	release_firmware(sm->pbl);
	sm->pbl = NULL;
}

static int s5300_map_region(struct s5300_modem *sm, const char *name,
			    phys_addr_t *phys, resource_size_t *size,
			    void __iomem **map)
{
	struct device_node *np;
	struct reserved_mem *rmem;
	int idx;

	idx = of_property_match_string(sm->dev->of_node, "memory-region-names",
				       name);
	if (idx < 0)
		return idx;
	np = of_parse_phandle(sm->dev->of_node, "memory-region", idx);
	if (!np)
		return -ENOENT;
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return -ENOENT;

	*phys = rmem->base;
	if (size)
		*size = rmem->size;
	/*
	 * Non-cached: the CP reads the staged PBL and writes boot/IPC state
	 * by PCIe DMA, and the HSI1 IO-coherency plumbing is not set up yet.
	 */
	*map = devm_ioremap_wc(sm->dev, rmem->base, rmem->size);
	if (!*map)
		return -ENOMEM;

	return 0;
}

static int s5300_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *rc_pdev;
	struct device_node *rc_node;
	struct s5300_modem *sm;
	const char *fw_name;
	u16 cmd, lnksta;
	int ret;

	sm = devm_kzalloc(dev, sizeof(*sm), GFP_KERNEL);
	if (!sm)
		return -ENOMEM;

	sm->dev = dev;
	spin_lock_init(&sm->lock);
	spin_lock_init(&sm->tx_lock);
	mutex_init(&sm->pcie_onoff_lock);
	init_completion(&sm->init_done);
	INIT_WORK(&sm->boot_work, s5300_boot_work);
	INIT_WORK(&sm->pm_work, s5300_pm_work);
	INIT_WORK(&sm->rfs_work, s5300_rfs_work);
	INIT_LIST_HEAD(&sm->rfs_rxq);
	spin_lock_init(&sm->rfs_lock);
	init_waitqueue_head(&sm->fmt_tx_wq);
	sm->oem.sm = sm;
	sm->oem.channel = S5300_CH_OEM;
	sm->oem.tx_max = S5300_FMT_MAX;
	skb_queue_head_init(&sm->oem.rxq);
	init_waitqueue_head(&sm->oem.read_wq);
	sm->link_up = true;	/* boot handshake rings directly; PM arms at ONLINE */
	platform_set_drvdata(pdev, sm);

	rc_node = of_parse_phandle(dev->of_node, "google,pcie", 0);
	if (!rc_node)
		return dev_err_probe(dev, -EINVAL, "missing google,pcie\n");
	rc_pdev = of_find_device_by_node(rc_node);
	of_node_put(rc_node);
	if (!rc_pdev)
		return -EPROBE_DEFER;
	sm->rc_dev = &rc_pdev->dev;
	if (!sm->rc_dev->driver) {
		ret = -EPROBE_DEFER;
		goto err_rc;
	}

	ret = of_property_read_u32(dev->of_node, "samsung,doorbell-addr",
				   &sm->db_bus_addr);
	if (ret) {
		dev_err_probe(dev, ret, "missing samsung,doorbell-addr\n");
		goto err_rc;
	}

	sm->cp2ap_wakeup = devm_gpiod_get(dev, "cp2ap-wakeup", GPIOD_IN);
	if (IS_ERR(sm->cp2ap_wakeup)) {
		ret = dev_err_probe(dev, PTR_ERR(sm->cp2ap_wakeup),
				    "failed to get CP2AP_WAKEUP\n");
		goto err_rc;
	}

	/* CP2AP_PHONE_ACTIVE crash monitor; optional (diagnosis only). */
	sm->cp_active = devm_gpiod_get_optional(dev, "cp2ap-active", GPIOD_IN);
	if (IS_ERR(sm->cp_active)) {
		ret = dev_err_probe(dev, PTR_ERR(sm->cp_active),
				    "failed to get CP2AP_PHONE_ACTIVE\n");
		goto err_rc;
	}

	/*
	 * AP2CP_PARTIAL_RST_N: driven high (deasserted) so MAIN does not read it
	 * as a partial-reset request and self-reset after ONLINE.  Optional -- CPs
	 * that do not sample it (e.g. tegu s5300) simply omit the property.
	 */
	sm->cp_partial_rst = devm_gpiod_get_optional(dev, "cp-partial-rst",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sm->cp_partial_rst)) {
		ret = dev_err_probe(dev, PTR_ERR(sm->cp_partial_rst),
				    "failed to get AP2CP_PARTIAL_RST_N\n");
		goto err_rc;
	}

	ret = s5300_map_region(sm, "ipc", &sm->ipc_phys, &sm->ipc_size,
			       &sm->ipc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map IPC carveout\n");
		goto err_rc;
	}
	ret = s5300_map_region(sm, "msi", &sm->msi_phys, NULL, &sm->msi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to map MSI carveout\n");
		goto err_rc;
	}
	/*
	 * Optional: the pktproc carveout the CP DMAs downlink packets into.  We
	 * only publish a minimal DL info region here so MAIN arms its runtime IPC;
	 * a DT without it still boots (older overlays), MAIN just won't arm.
	 */
	if (s5300_map_region(sm, "pktproc", &sm->pktproc_phys, &sm->pktproc_size,
			     &sm->pktproc))
		dev_warn(dev, "no pktproc carveout mapped\n");

	if (of_property_read_string(dev->of_node, "firmware-name", &fw_name))
		fw_name = "tegu/cp_pbl.bin";
	ret = request_firmware(&sm->pbl, fw_name, dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to load %s\n", fw_name);
		goto err_rc;
	}
	s5300_find_boot(sm);
	if (sm->boot_size > sm->ipc_size - S5300_BOOT_IMG_OFFSET) {
		ret = dev_err_probe(dev, -EFBIG, "PBL too large\n");
		goto err_fw;
	}

	/*
	 * The zumapro root port carries the same 144d:a5a5 ID as the modem
	 * endpoint, and it registers first -- a bare first-match lookup
	 * returns the root port.  Match the endpoint by port type.
	 */
	sm->pdev = NULL;
	while ((sm->pdev = pci_get_device(S5300_PCI_VENDOR_ID,
					  S5300_PCI_DEVICE_ID, sm->pdev))) {
		if (pci_pcie_type(sm->pdev) == PCI_EXP_TYPE_ENDPOINT)
			break;
	}
	if (!sm->pdev) {
		ret = dev_err_probe(dev, -ENODEV,
				    "CP endpoint not enumerated\n");
		goto err_fw;
	}
	dev_info(dev, "CP endpoint %s\n", pci_name(sm->pdev));

	/*
	 * A live MAIN (Gen3 endpoint) is ADOPTED, not reset: the light warm
	 * reset does not take against a running MAIN (its link re-trains at
	 * Gen3 x2 and BL1 lands on deaf ears, boot_stage stuck at 0), the SPMI
	 * patches collide with the CP's own bus traffic (arbitration loses vs
	 * the live master), and killing a working modem on a module reload is
	 * the wrong trade anyway.  Everything else -- parked link, dead CP,
	 * boot ROM at Gen1 -- goes through the parked-aware warm reset, so a
	 * reload is a cold boot exactly when it has to be.
	 */
	if (s5300_cold_cycle) {
		/*
		 * FULL CP power cycle (vendor gpio_power_offon_cp): genuinely
		 * powers the CP down and up so a crashed / self-reset MAIN is
		 * back in its boot ROM.  A warm reset (CP_WRST pulse) cannot
		 * stop a running MAIN -- the fresh download then lands on it and
		 * wedges at INIT_START -- which is why cold_cycle uses this, not
		 * modem_reset.  The power cycle includes the relink, so there is
		 * no separate warm reset here.
		 */
		ret = zumapro_pcie_modem_power_cycle(sm->rc_dev);
		if (ret)
			dev_warn(dev, "CP power cycle: no link (%d); continuing\n",
				 ret);
		sm->cold_cycled = true;
	} else {
		/*
		 * A live MAIN (Gen3 endpoint after a bare reload) is adopted;
		 * anything else (parked, Gen1 boot ROM) gets the warm reset.
		 * Config-space read, not the pci-exynos helper, so the module
		 * needs no symbol the flashed kernel might lack; a parked link
		 * reads all-ones and falls through to the reset.
		 */
		pcie_capability_read_word(sm->pdev, PCI_EXP_LNKSTA, &lnksta);
		if (s5300_allow_adopt && lnksta != 0xffff &&
		    (lnksta & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_8_0GB) {
			dev_info(dev, "live MAIN (Gen3 link); adopting\n");
			sm->adopt = true;
		} else {
			dev_info(dev, "CP LNKSTA %#x; warm-resetting\n", lnksta);
			ret = zumapro_pcie_modem_reset(sm->rc_dev);
			if (ret)
				dev_warn(dev, "CP warm reset: no link (%d)\n",
					 ret);
		}
	}

	/*
	 * Downstream keeps every form of link PM off for the whole CP boot
	 * (L1SS only comes on from complete_normal_boot(), and its config
	 * accessors pin the link at L0 for each access).  The core enabled
	 * ASPM L1 at enumeration; take it back out before poking config
	 * space -- the mask ROM's config emulation has been seen returning
	 * garbled completions to rapid access bursts.
	 */
	pci_disable_link_state(sm->pdev, PCIE_LINK_STATE_L0S |
			       PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_CLKPM);

	/* Before MSI allocation: the EP capability must carry the carveout. */
	ret = zumapro_pcie_set_msi_target(sm->rc_dev, sm->msi_phys);
	if (ret)
		goto err_pci;

	ret = s5300_setup_doorbell(sm);
	if (ret)
		goto err_pci;

	ret = pci_enable_device(sm->pdev);
	if (ret) {
		dev_err(dev, "pci_enable_device: %d\n", ret);
		goto err_pci;
	}
	pci_set_master(sm->pdev);
	/* MSE by hand: BAR0 is programmed behind the PCI core's back. */
	pci_read_config_word(sm->pdev, PCI_COMMAND, &cmd);
	pci_write_config_word(sm->pdev, PCI_COMMAND, cmd | PCI_COMMAND_MEMORY);

	/*
	 * The core caches the MSI capability offset at enumeration time; if
	 * the mask ROM exposed its capability list late, re-look it up so
	 * vector allocation does not fail on a stale zero.
	 */
	if (!sm->pdev->msi_cap) {
		sm->pdev->msi_cap = pci_find_capability(sm->pdev,
							PCI_CAP_ID_MSI);
		dev_warn(dev, "MSI capability re-lookup: %#x\n",
			 sm->pdev->msi_cap);
	}

	/*
	 * Dormant belt-and-braces: if the walk still comes up empty, probe
	 * the DW-default offset downstream hardcodes (print_msi_register())
	 * and install it directly, logging what the raw reads see.
	 */
	if (!sm->pdev->msi_cap) {
		u16 w40, w50, w52;
		u32 hdr;

		pci_read_config_word(sm->pdev, 0x40, &w40);
		pci_read_config_word(sm->pdev, S5300_ROM_MSI_CAP, &w50);
		pci_read_config_word(sm->pdev, S5300_ROM_MSI_CAP + 2, &w52);
		pci_read_config_dword(sm->pdev, S5300_ROM_MSI_CAP, &hdr);
		dev_warn(dev,
			 "MSI cap probe: w@0x40 %#06x w@0x50 %#06x w@0x52 %#06x dw@0x50 %#010x\n",
			 w40, w50, w52, hdr);
		if ((hdr & PCI_CAP_ID_MASK) == PCI_CAP_ID_MSI)
			sm->pdev->msi_cap = S5300_ROM_MSI_CAP;
	}

	/*
	 * Reserve the RC's own MSI vectors 0-3 so the modem's four vectors land
	 * at data base 4 (MME=2): MAIN raises INIT_START (and every MAIN-phase
	 * interrupt) on MSI message 4, which then lands on the endpoint's vector
	 * 0 where s5300_irq_handler() sees it.  Without this the CP drains the
	 * whole download but its INIT_START MSI is unrepresentable and the
	 * handshake never completes.
	 */
	ret = zumapro_pcie_reserve_msi_base(sm->rc_dev, S5300_MSI_VECTORS);
	if (ret) {
		dev_err(dev, "reserving MSI base failed: %d\n", ret);
		goto err_disable;
	}

	/*
	 * Request up to four vectors, but this mask ROM only advertises MMC=0
	 * (one vector capable), so we get exactly one -- and with vectors 0-3
	 * reserved above, that single vector lands at MSI message-data base 4,
	 * which is where MAIN raises INIT_START (and the ROM its boot acks).
	 * That is all bring-up needs; the TX/pktproc vectors come with the data
	 * path.  request_irq() below is on the first (only) allocated vector.
	 */
	ret = pci_alloc_irq_vectors(sm->pdev, 1, S5300_MSI_VECTORS,
				    PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(dev, "MSI alloc: %d (power state %d, msi_cap %#x)\n",
			ret, sm->pdev->current_state, sm->pdev->msi_cap);
		goto err_disable;
	}
	dev_info(dev, "%d MSI vector(s) (reserved base 4)\n", ret);

	/*
	 * Wipe the stale control words BEFORE the MSI handler goes live.  The
	 * IPC carveout is AP DRAM that survives a CP power cycle / module
	 * reload, so cp2ap_msg keeps the previous session's value (e.g. 0xc8 =
	 * PHONE_START).  The boot ROM's first MSI would then make the handler
	 * process that stale PHONE_START, prematurely set sm->online=true and
	 * run init_ipc_queues -- so the REAL post-download PHONE_START is
	 * skipped, the runtime IPC magic (0xAA) is never written over the boot
	 * magic (0xBDBD), and MAIN self-resets ~123 ms after ONLINE seeing
	 * boot-magic at runtime.  Clearing cp2ap_msg here is the fix.
	 */
	writel(0, sm->ipc + S5300_IPC_AP2CP_MSG);
	writel(0, sm->ipc + S5300_IPC_CP2AP_MSG);
	sm->online = false;

	ret = request_irq(pci_irq_vector(sm->pdev, 0), s5300_irq_handler, 0,
			  "s5300-ipc", sm);
	if (ret)
		goto err_vectors;

	/*
	 * CP-driven runtime PCIe PM: the CP toggles CP2AP_WAKEUP to ask for the
	 * link back (or announce it is parking).  Requested disabled (IRQF_NO_AUTOEN)
	 * -- the boot path still polls this GPIO for the mid-boot re-link -- and
	 * enabled once the CP reaches ONLINE (PHONE_START).  Ordered wq: relinks
	 * must not race.
	 */
	sm->pm_wq = alloc_ordered_workqueue("s5300-pm", 0);
	if (!sm->pm_wq) {
		ret = -ENOMEM;
		goto err_irq;
	}
	/* Ordered wq for the in-kernel RFS server; stateful ack ping-pong. */
	sm->rfs_wq = alloc_ordered_workqueue("s5300-rfs", 0);
	if (!sm->rfs_wq) {
		ret = -ENOMEM;
		goto err_wq;
	}
	sm->rfs_txbuf = devm_kmalloc(dev, 0x14 + S5300_RFS_CHUNK, GFP_KERNEL);
	if (!sm->rfs_txbuf) {
		ret = -ENOMEM;
		goto err_rfs;
	}
	sm->cp2ap_irq = gpiod_to_irq(sm->cp2ap_wakeup);
	if (sm->cp2ap_irq < 0) {
		ret = dev_err_probe(dev, sm->cp2ap_irq, "CP2AP_WAKEUP to irq\n");
		goto err_rfs;
	}
	ret = request_irq(sm->cp2ap_irq, s5300_cp2ap_wakeup_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_NO_AUTOEN, "s5300-cp2ap-wakeup", sm);
	if (ret) {
		dev_err(dev, "CP2AP_WAKEUP request_irq: %d\n", ret);
		goto err_rfs;
	}

	/*
	 * Armed from the start (unlike the wakeup IRQ): logs the CP coming
	 * alive during boot and any crash after ONLINE.  Non-fatal if missing.
	 */
	sm->cp_active_irq = -1;
	if (sm->cp_active) {
		int irq = gpiod_to_irq(sm->cp_active);

		if (irq >= 0 && !request_irq(irq, s5300_cp_active_irq,
					     IRQF_TRIGGER_RISING |
					     IRQF_TRIGGER_FALLING,
					     "s5300-cp-active", sm))
			sm->cp_active_irq = irq;
		else
			dev_warn(dev, "CP2AP_PHONE_ACTIVE irq unavailable\n");
	}

	/*
	 * The oem/GEMS channel (ch 0x82 on the FMT ring), exposed as /dev/umts_oem1
	 * for the userspace daemon that answers the CP's UE-capability-config file
	 * requests.  Registered up front; it only carries traffic once ONLINE.
	 */
	sm->oem.miscdev.minor = MISC_DYNAMIC_MINOR;
	sm->oem.miscdev.name = "umts_oem1";
	sm->oem.miscdev.fops = &s5300_chardev_fops;
	sm->oem.miscdev.parent = dev;
	ret = misc_register(&sm->oem.miscdev);
	if (ret) {
		dev_err(dev, "misc_register(oem): %d\n", ret);
		goto err_cp_irq;
	}

	schedule_work(&sm->boot_work);

	return 0;

err_cp_irq:
	if (sm->cp_active_irq >= 0)
		free_irq(sm->cp_active_irq, sm);
	free_irq(sm->cp2ap_irq, sm);
err_rfs:
	destroy_workqueue(sm->rfs_wq);
err_wq:
	destroy_workqueue(sm->pm_wq);
err_irq:
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
err_vectors:
	pci_free_irq_vectors(sm->pdev);
err_disable:
	pci_disable_device(sm->pdev);
err_pci:
	pci_dev_put(sm->pdev);
err_fw:
	release_firmware(sm->pbl);
err_rc:
	put_device(sm->rc_dev);
	return ret;
}

static void s5300_remove(struct platform_device *pdev)
{
	struct s5300_modem *sm = platform_get_drvdata(pdev);

	/*
	 * Quiesce runtime PM first: free_irq() stops new wakeup IRQs and waits for
	 * the in-flight handler, so cancel_work_sync() then flushes the last relink
	 * before the endpoint state it touches goes away.
	 */
	free_irq(sm->cp2ap_irq, sm);
	if (sm->cp_active_irq >= 0)
		free_irq(sm->cp_active_irq, sm);
	cancel_work_sync(&sm->pm_work);
	destroy_workqueue(sm->pm_wq);

	cancel_work_sync(&sm->boot_work);
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
	/* Main IRQ gone -> no more DL drains or RFS enqueues. */
	if (sm->ndev)
		unregister_netdev(sm->ndev);	/* needs_free_netdev frees it */
	cancel_work_sync(&sm->rfs_work);
	destroy_workqueue(sm->rfs_wq);
	s5300_rfs_cleanup(sm);
	misc_deregister(&sm->oem.miscdev);
	skb_queue_purge(&sm->oem.rxq);
	if (sm->rfs_port)
		wwan_remove_port(sm->rfs_port);
	if (sm->ctrl_port)
		wwan_remove_port(sm->ctrl_port);
	if (sm->at_port)
		wwan_remove_port(sm->at_port);
	release_firmware(sm->pbl);
	pci_free_irq_vectors(sm->pdev);
	pci_disable_device(sm->pdev);
	pci_dev_put(sm->pdev);
	put_device(sm->rc_dev);
}

static const struct of_device_id s5300_of_match[] = {
	{ .compatible = "samsung,s5300-modem" },
	{ },
};
MODULE_DEVICE_TABLE(of, s5300_of_match);

/*
 * Quiesce on system shutdown/reboot: stop every worker and IRQ path that
 * could touch the CP link.  Rebooting with an active instance once panicked
 * in the shutdown path (an access to a dead CP link throws an SError on this
 * SoC), so with autoload enabled this hook is load-bearing.
 */
static void s5300_shutdown(struct platform_device *pdev)
{
	struct s5300_modem *sm = platform_get_drvdata(pdev);

	if (!sm)
		return;

	/* Stop rmnet TX from ringing the doorbell on a link about to die. */
	sm->ul_active = false;
	if (sm->ndev)
		netif_tx_disable(sm->ndev);

	free_irq(sm->cp2ap_irq, sm);
	if (sm->cp_active_irq >= 0)
		free_irq(sm->cp_active_irq, sm);
	cancel_work_sync(&sm->pm_work);
	cancel_work_sync(&sm->boot_work);
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
	cancel_work_sync(&sm->rfs_work);
	dev_info(sm->dev, "quiesced for shutdown\n");
}

static struct platform_driver s5300_driver = {
	.probe	= s5300_probe,
	.remove	= s5300_remove,
	.shutdown = s5300_shutdown,
	.driver	= {
		.name		= "s5300-modem",
		.of_match_table	= s5300_of_match,
	},
};
module_platform_driver(s5300_driver);

/*
 * Fixed-path firmware the driver loads.  The per-device handover block and the
 * RF_CFG_<sha1> image (chosen at runtime from hardware_config.json) have no
 * fixed name and are not declared here.
 */
MODULE_FIRMWARE(S5300_HANDOVER_FW);
MODULE_FIRMWARE(S5300_CPSHA_FW);
MODULE_FIRMWARE(S5300_HWCFG_FW);
MODULE_FIRMWARE(S5300_RF_CFG_FALLBACK);
MODULE_FIRMWARE("google/s5400/efs/nv_normal.bin");
MODULE_FIRMWARE("google/s5400/efs/nv_protected.bin");
MODULE_FIRMWARE("google/s5400/modem_userdata/replay_region.bin");
MODULE_FIRMWARE(S5300_REPLAY_DDS_FW);

MODULE_DESCRIPTION("Samsung Exynos Modem 5300 PCIe boot driver");
MODULE_LICENSE("GPL");
