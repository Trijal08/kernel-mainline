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
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-zumapro.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>

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
/* s5400 diagnostic fields (vendor struct msi_reg_type). */
#define S5300_MSI_FLAG_CAFE		0x30	/* CP-alive marker */
#define S5300_MSI_SUB_BOOT_STAGE	0x34

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
#define S5300_IPC_AP2CP_MSG		0x800
#define S5300_IPC_CP2AP_MSG		0x804
#define S5300_IPC_AP2CP_STATUS		0x808
#define S5300_IPC_CP2AP_STATUS		0x80c
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

/*
 * Exynos link-header channel IDs (downstream exynos-cpif.h).  Runtime control
 * channels are multiplexed over the shared rings and demuxed by the ch_id byte
 * in the 12-byte header: FMT (245) = umts_ipc0 RIL control on the FMT ring;
 * AT (21, EXYNOS_CH_ID_BT_DUN) = umts_router AT commands on the NORM_RAW ring.
 */
#define S5300_CH_FMT			245
#define S5300_CH_AT			21

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
/* legacy FMT queue (DT legacy_fmt_*): the CP posts control messages here. */
#define S5300_FMT_RXQ_HEAD		0x10		/* CP advances */
#define S5300_FMT_RXQ_TAIL		0x14		/* AP advances */
#define S5300_FMT_RXQ_BUFF		0x1000
#define S5300_FMT_RXQ_SIZE		0x1000

/* Exynos SIT link header (12B) + Samsung std_dl download header (12B). */
#define S5300_SIT_HDR			12
#define S5300_SIT_SYNC			0xabcd
#define S5300_SIT_CFG_SINGLE		0xc000		/* EXYNOS_SINGLE_MASK<<8 */
#define S5300_SIT_CH_BOOT		0xf1		/* EXYNOS_CH_ID_BOOT */
#define S5300_DL_HDR			12
#define S5300_DL_CHUNK			0xc000		/* 49152-byte payload cap */
/* std_dl control words: per-section start/end carry a tag; the standalone
 * finalise word (cbd emits 0xa400, CP acks 0xc400) tells the CP the image is
 * complete and to boot MAIN. */
#define S5300_DL_SEC_START(tag)		(0xa100 | ((tag) << 4))
#define S5300_DL_SEC_END(tag)		(0xa10d | ((tag) << 4))
#define S5300_DL_SEC_VERIFY(tag)	(0xa301 | ((tag) << 4))	/* CRC-verify */
#define S5300_DL_FINISH			0xa400

struct s5300_modem {
	struct device		*dev;
	struct device		*rc_dev;
	struct pci_dev		*pdev;
	struct gpio_desc	*cp2ap_wakeup;

	phys_addr_t		ipc_phys;
	resource_size_t		ipc_size;
	void __iomem		*ipc;
	phys_addr_t		msi_phys;
	void __iomem		*msi;

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

	/*
	 * CP-driven runtime PCIe power management (post-ONLINE): the CP parks its
	 * link when idle and drives CP2AP_WAKEUP to ask for it back.  The relink
	 * sleeps, so it runs on an ordered workqueue off the wakeup IRQ.
	 */
	int			cp2ap_irq;
	struct workqueue_struct	*pm_wq;
	struct work_struct	pm_work;
	struct mutex		pcie_onoff_lock; /* serialises relink up/down */
	bool			pm_armed;	/* cp2ap_irq enabled (boot-seq guard) */
	bool			link_up;	/* RC link powered on (sm->lock) */
	bool			db_reserved;	/* doorbell deferred to wake (sm->lock) */
	bool			cp_wants_up;	/* CP2AP_WAKEUP level latched at edge */

	/* Runtime control channel: umts_router AT commands over the RAW ring. */
	struct wwan_port	*at_port;
	struct work_struct	rx_work;	/* drain the RX rings (process ctx) */
	spinlock_t		tx_lock;	/* serialises RAW txq producers */
	u8			at_ch_seq;	/* per-channel header sequence */
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
 */
static void s5300_send_doorbell(struct s5300_modem *sm, u32 val)
{
	int try;
	u16 cmd;

	for (try = 0; try < 10; try++) {
		writel(val, sm->doorbell);
		if (readl(sm->doorbell) != 0xffffffff)
			return;

		pci_read_config_word(sm->pdev, PCI_COMMAND, &cmd);
		if (cmd != 0xffff &&
		    (cmd & (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)) !=
		    (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER))
			pci_write_config_word(sm->pdev, PCI_COMMAND, cmd |
					      PCI_COMMAND_MEMORY |
					      PCI_COMMAND_MASTER);
		s5300_program_doorbell_bar(sm);
		s5300_open_bridge_window(sm);
		udelay(100);
	}

	dev_err(sm->dev, "doorbell %#x kept reading back all-ones\n", val);
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
	if (sm->link_up) {
		sm->db_reserved = false;
		s5300_send_doorbell(sm, S5300_DB_MSG);
	} else {
		sm->db_reserved = true;
		wake = true;
	}
	spin_unlock_irqrestore(&sm->lock, flags);

	if (wake)
		zumapro_pcie_modem_wake(sm->rc_dev);
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

	mutex_lock(&sm->pcie_onoff_lock);

	if (want_up && !sm->link_up) {
		if (zumapro_pcie_modem_link_up(sm->rc_dev) == 0) {
			s5300_relink_restore(sm);
			spin_lock_irqsave(&sm->lock, flags);
			sm->link_up = true;
			spin_unlock_irqrestore(&sm->lock, flags);
			dev_info(sm->dev, "CP wakeup: link up\n");
		} else {
			dev_err(sm->dev, "CP wakeup: relink failed\n");
		}
	} else if (!want_up && sm->link_up) {
		/*
		 * Clear link_up *before* the teardown so a concurrent
		 * s5300_send_ipc_irq() reserves its doorbell instead of ringing it
		 * into an endpoint that is about to be held in PERST.
		 */
		spin_lock_irqsave(&sm->lock, flags);
		sm->link_up = false;
		spin_unlock_irqrestore(&sm->lock, flags);
		pci_set_power_state(sm->pdev, PCI_D3hot);
		zumapro_pcie_modem_link_down(sm->rc_dev);
		dev_info(sm->dev, "CP sleep: link down\n");
	}

	mutex_unlock(&sm->pcie_onoff_lock);

	/* Flush a doorbell a tx deferred while the link was parked. */
	spin_lock_irqsave(&sm->lock, flags);
	if (sm->link_up && sm->db_reserved) {
		sm->db_reserved = false;
		s5300_send_doorbell(sm, S5300_DB_MSG);
	}
	spin_unlock_irqrestore(&sm->lock, flags);
}

static irqreturn_t s5300_cp2ap_wakeup_irq(int irq, void *data)
{
	struct s5300_modem *sm = data;

	/*
	 * Latch the level at edge time: the CP pulses CP2AP_WAKEUP faster than the
	 * workqueue can read it, so reading the GPIO in pm_work misses a brief park.
	 * cp2ap_wakeup is a memory-mapped SoC GPIO, so gpiod_get_value never sleeps.
	 */
	WRITE_ONCE(sm->cp_wants_up, gpiod_get_value(sm->cp2ap_wakeup));
	queue_work(sm->pm_wq, &sm->pm_work);
	return IRQ_HANDLED;
}

/*
 * Downstream init_control_messages(): publish the srinfo and capability
 * offsets and zero the status/capability words before the CP boots.  The
 * AP capability words stay zero (tegu DT: ap_capability_0/1 = 0).
 */
static void s5300_init_control_messages(struct s5300_modem *sm)
{
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
	 * Publish ds_det=1 (0x4000): load-bearing for runtime IPC.  It makes the
	 * CP run its deep-sleep link handshake -- park the link when idle and
	 * cycle CP2AP_WAKEUP -- and each wake relink (s5300_pm_work) re-arms MAIN's
	 * runtime IPC so the FMT/RAW control queues get drained.  With ds_det=0 the
	 * CP never takes a link-up ISR and the queues stay stuck.
	 */
	writel(S5300_IPC_DS_DET, sm->ipc + S5300_IPC_AP2CP_STATUS);
	writel(0, sm->ipc + S5300_IPC_CP2AP_STATUS);
	for (i = 0; i < S5300_IPC_CAP_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_CAP_BASE + 4 * i);
}

/*
 * Downstream init_legacy_link(), run from the PHONE_START handler: clear the
 * queue pointers, then advertise the magic and access-enable words.
 */
static void s5300_init_ipc_queues(struct s5300_modem *sm)
{
	u32 magic, access;
	int i;

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
	if (ret)
		return ret;
	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5300_wwan_ops = {
	.start	= s5300_wwan_start,
	.stop	= s5300_wwan_stop,
	.tx	= s5300_wwan_tx,
};

/*
 * Drain one RX ring: parse each 12-byte-framed message, hand AT-channel (ch 21)
 * payloads to the WWAN port, and advance the tail past everything consumed.
 * Frames for channels we don't handle yet are skipped (dropped).
 */
static void s5300_rx_ring(struct s5300_modem *sm, u32 head_off, u32 tail_off,
			  u32 buff_off, u32 size)
{
	void __iomem *buff = sm->ipc + buff_off;
	u32 in, out;

	in = readl(sm->ipc + head_off);
	out = readl(sm->ipc + tail_off);

	while (in != out) {
		u32 usage = s5300_circ_usage(size, in, out);
		u8 hdr[S5300_SIT_HDR];
		u32 flen, total, plen;
		u8 ch;

		if (usage < S5300_SIT_HDR)
			break;
		s5300_circ_read(hdr, buff, size, out, S5300_SIT_HDR);
		if (get_unaligned_le16(hdr) != S5300_SIT_SYNC) {
			dev_err_ratelimited(sm->dev, "rx bad sync %#x; flushing\n",
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

		if (ch == S5300_CH_AT && sm->at_port && plen) {
			struct sk_buff *skb = alloc_skb(plen, GFP_KERNEL);

			if (skb) {
				s5300_circ_read(skb_put(skb, plen), buff, size,
						(out + S5300_SIT_HDR) % size, plen);
				wwan_port_rx(sm->at_port, skb);
			}
		}
		out = (out + total) % size;
	}

	/* order the payload reads ahead of the tail advance the CP watches */
	dma_wmb();
	writel(out, sm->ipc + tail_off);
}

static void s5300_rx_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem, rx_work);

	if (!sm->at_port)
		return;			/* port not up yet; retry on next IRQ */
	s5300_rx_ring(sm, S5300_RAW_RXQ_HEAD, S5300_RAW_RXQ_TAIL,
		      S5300_RAW_RXQ_BUFF, S5300_RAW_RXQ_SIZE);
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
	/* Drain anything the CP queued while the port was coming up. */
	schedule_work(&sm->rx_work);
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
	if (!(val & S5300_INT_VALID))
		return IRQ_HANDLED;

	if (!(val & S5300_CMD_VALID)) {
		/* Data notification (SEND_FMT/SEND_RAW), no command. */
		if (sm->online)
			/* Runtime: parse the RX rings in process context. */
			schedule_work(&sm->rx_work);
		else
			/* Boot: std_dl acks; consume so the CP keeps draining. */
			writel(readl(sm->ipc + S5300_RAW_RXQ_HEAD),
			       sm->ipc + S5300_RAW_RXQ_TAIL);
		return IRQ_HANDLED;
	}

	cmd = val & S5300_CMD_MASK;
	switch (cmd) {
	case S5300_CMD_INIT_START:
		dev_info(sm->dev, "CP INIT_START\n");
		s5300_send_ipc_irq(sm, S5300_CMD(S5300_CMD_PIF_INIT_DONE));
		break;
	case S5300_CMD_PHONE_START:
		dev_info(sm->dev, "CP PHONE_START\n");
		if (!sm->online) {
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
} s5300_dl_secs[] = {
	{ NULL,      NULL,                                            false },
	{ "PSP",     NULL,                                            false },
	{ "MAIN",    NULL,                                            true  },
	{ "APM",     NULL,                                            false },
	{ "VSS",     NULL,                                            false },
	{ "DBGCORE", NULL,                                            false },
	{ "RF_CFG",  "google/s5400/rf_cfg.bin",                       false },
	{ "NV_NORM", "google/s5400/efs/nv_normal.bin",               false },
	{ "NV_PROT", "google/s5400/efs/nv_protected.bin",            false },
	{ "REPLAY",  "google/s5400/modem_userdata/replay_region.bin", false },
};

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

		/* Warm CP: it went ONLINE without draining the ring. */
		if (completion_done(&sm->init_done)) {
			dev_info(sm->dev, "CP online mid-boot; skipping download\n");
			goto done;
		}

		if (!d->toc_name) {
			/* Stage 1: the raw TOC table (first record spans it). */
			data = sm->pbl->data;
			size = le32_to_cpu(toc[0].size);
		} else if (d->fw_name) {
			/* NV/REPLAY: section data is a vendor partition file. */
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

static void s5300_boot_work(struct work_struct *work)
{
	struct s5300_modem *sm = container_of(work, struct s5300_modem,
					      boot_work);
	struct pci_dev *pdev = sm->pdev;
	size_t bl1_size, btl_off, btl_size;
	int ret, i;

	/* Reset boot_stage and initialise the control-message region. */
	writel(0, sm->msi + S5300_MSI_BOOT_STAGE);
	s5300_init_control_messages(sm);

	/*
	 * Arm the shared-memory IPC region up front (vendor init_legacy_link in
	 * link_start_normal_boot, before BL1): the CP polls the SIT boot magic
	 * at IPC+0 to recognise the region, and MEM_ACCESS gates AP visibility.
	 * Both must be live before the bootloader hands over to MAIN and starts
	 * the post-bounce INIT_START handshake.
	 */
	writel(S5300_SHM_BOOT_MAGIC, sm->ipc + S5300_IPC_MAGIC);
	writel(1, sm->ipc + S5300_IPC_MEM_ACCESS);
	/*
	 * Zero every FMT+RAW queue head/tail word (vendor init_legacy_link
	 * clears them all before boot).  We had only cleared the RAW download
	 * ring, leaving the FMT queue pointers at their power-on 0xffffffff;
	 * the CP's IPC validation rejects that and never leaves the bootloader.
	 */
	for (i = 0; i < S5300_IPC_Q_WORDS; i++)
		writel(0, sm->ipc + S5300_IPC_Q_HEAD_TAIL + 4 * i);

	/* Config state (forced BAR0, MSI capability) must survive the re-link. */
	pci_save_state(pdev);

	/*
	 * Two-stage boot (vendor start_normal_boot_bl1() then
	 * start_normal_boot_bootloader()): the BOOT section is split, and each
	 * half is fed through the same boot slot with its own message doorbell.
	 * Stage 1 (BL1, first S5300_BL1_IMG_SIZE bytes) drives boot_stage to
	 * BL1_DONE; stage 2 (the remainder, the "bootloader") drives it to DONE.
	 */
	bl1_size = min_t(size_t, S5300_BL1_IMG_SIZE, sm->boot_size);
	btl_off  = sm->boot_off + bl1_size;
	btl_size = sm->boot_size - bl1_size;

	dev_info(sm->dev, "BL1 download (%#zx bytes at %pap+%#x)\n",
		 bl1_size, &sm->ipc_phys, S5300_BOOT_IMG_OFFSET);
	s5300_send_boot_image(sm, sm->boot_off, bl1_size);
	ret = s5300_poll_boot_stage(sm, S5300_BOOT_STAGE_BL1_DONE);
	if (ret)
		goto out_fw;

	dev_info(sm->dev, "BL1 up; bootloader download (%#zx bytes)\n", btl_size);
	s5300_send_boot_image(sm, btl_off, btl_size);
	ret = s5300_poll_boot_stage(sm, S5300_BOOT_STAGE_DONE);
	if (ret)
		goto out_fw;

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
	 * Warm CP (the common case here): the warm reset preserved the CP's
	 * self-powered DRAM, so MAIN is resident.  Like the vendor, the
	 * bootloader just re-runs it and raises INIT_START on its own -- no
	 * download at all.  Wait for that first.
	 */
	dev_info(sm->dev, "link re-established; waiting for resident MAIN\n");
	if (wait_for_completion_timeout(&sm->init_done, 5 * HZ)) {
		release_firmware(sm->pbl);
		sm->pbl = NULL;
		goto online;
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
	u16 cmd;
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
	INIT_WORK(&sm->rx_work, s5300_rx_work);
	INIT_WORK(&sm->pm_work, s5300_pm_work);
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
	sm->cp2ap_irq = gpiod_to_irq(sm->cp2ap_wakeup);
	if (sm->cp2ap_irq < 0) {
		ret = dev_err_probe(dev, sm->cp2ap_irq, "CP2AP_WAKEUP to irq\n");
		goto err_wq;
	}
	ret = request_irq(sm->cp2ap_irq, s5300_cp2ap_wakeup_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_NO_AUTOEN, "s5300-cp2ap-wakeup", sm);
	if (ret) {
		dev_err(dev, "CP2AP_WAKEUP request_irq: %d\n", ret);
		goto err_wq;
	}

	schedule_work(&sm->boot_work);

	return 0;

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
	cancel_work_sync(&sm->pm_work);
	destroy_workqueue(sm->pm_wq);

	cancel_work_sync(&sm->boot_work);
	free_irq(pci_irq_vector(sm->pdev, 0), sm);
	cancel_work_sync(&sm->rx_work);
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

static struct platform_driver s5300_driver = {
	.probe	= s5300_probe,
	.remove	= s5300_remove,
	.driver	= {
		.name		= "s5300-modem",
		.of_match_table	= s5300_of_match,
	},
};
module_platform_driver(s5300_driver);

MODULE_DESCRIPTION("Samsung Exynos Modem 5300 PCIe boot driver");
MODULE_LICENSE("GPL");
