// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google AOC (Always-On Compute) coprocessor - firmware load.
 *
 * The AOC is an always-on coprocessor on Tensor SoCs that owns Bluetooth,
 * audio and the sensor hub.  Its DRAM carveout is fenced from the non-secure
 * AP, and its firmware must be authenticated and unlocked by the GSA security
 * core before the AOC can run.
 *
 * This driver stages the AOC image into the carveout and asks the GSA to
 * authenticate it: the signed 4K header goes in coherent memory the GSA reads
 * by DMA, the body goes into the carveout, and gsa_load_aoc_fw_image() hands
 * the GSA both.  Bringing the AOC out of reset and the runtime IPC are separate,
 * later steps.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <linux/soc/google/aoc.h>
#include <linux/soc/samsung/exynos-gsa.h>
#include <linux/math64.h>
#include <linux/timekeeping.h>
#include <linux/trusty/trusty_ipc.h>
#include <asm/arch_timer.h>
#include <clocksource/arm_arch_timer.h>

/*
 * The AOC's secure setup (its SysMMU's secure context and the reset release)
 * is done by a Trusty app, the GSA hardware manager for the AOC, reached over
 * tipc.  The GSA authenticates the image over its mailbox (gsa_load_aoc_fw_image)
 * but only this app can start the core.
 */
#define AOC_HWMGR_PORT		"com.android.trusty.gsa.hwmgr.aoc"

/* Sub-commands of the hwmgr STATE command (enum gsa_aoc_cmd). */
#define AOC_TZ_GET_STATE	0
#define AOC_TZ_START		1

/* GSA hwmgr tipc protocol (from the vendor hwmgr-ipc.h). */
#define HWMGR_CMD_RESP		(1U << 31)
#define HWMGR_CMD_STATE_CMD	1

struct hwmgr_state_req {
	u32 hdr_cmd;			/* HWMGR_CMD_STATE_CMD */
	u32 state_cmd;			/* AOC_TZ_START / AOC_TZ_GET_STATE */
};

struct hwmgr_state_rsp {
	u32 hdr_cmd;			/* HWMGR_CMD_STATE_CMD | HWMGR_CMD_RESP */
	u32 hdr_err;
	u32 state;
};

#define AOC_TZ_CONNECT_TIMEOUT_MS	5000
#define AOC_TZ_MSG_TIMEOUT_MS		10000
#define AOC_TZ_BUF_TIMEOUT_MS		10000

#define AOC_FIRMWARE_NAME	"google/aoc.bin"

/* A signed AOC image begins with a fixed-size authentication header (the cert
 * the GSA verifies); the body that the certificate signs follows it.
 */
#define AOC_AUTH_HEADER_SIZE	4096

/* The running AOC firmware stamps this into its DRAM control block. */
#define AOC_MAGIC		0xa0c00a0cU

/*
 * A signed image's body starts with a superbin header; its u32 at +0x0c
 * ("image_size") is the offset of the IPC control block within the carveout.
 */
#define AOC_SUPERBIN_IMAGE_SIZE	0x0c

/*
 * Before the core starts, the AP writes a parameter block (magic + key/value
 * entries) at ipc_offset; the AOC reads it at boot to learn where its carveout
 * is and which board it is on, then overwrites it with its control block.
 */
#define AOC_PARAMETER_MAGIC		0x0a0cda7a
#define kAOCBoardID			0x1001
#define kAOCBoardRevision		0x1002
#define kAOCSRAMRepaired		0x1003
#define kAOCCarveoutAddress		0x1005
#define kAOCCarveoutSize		0x1006
#define kAOCSensorDirectHeapAddress	0x1007
#define kAOCSensorDirectHeapSize	0x1008
#define kAOCForceVNOM			0x1009
#define kAOCDisableMM			0x100a
#define kAOCEnableUART			0x100b
#define kAOCPlaybackHeapAddress		0x100c
#define kAOCPlaybackHeapSize		0x100d
#define kAOCCaptureHeapAddress		0x100e
#define kAOCCaptureHeapSize		0x100f
#define kAOCForceSpeakerUltrasonic	0x1010
#define kAOCRandSeed			0x1011
#define kAOCChipRevision		0x1012
#define kAOCChipType			0x1013
#define kAOCGnssType			0x1014
#define kAOCVolteReleaseMif		0x1015
#define kAOCChipProductId		0x1016
#define kAOCWifiChip			0x1017

/* The AOC's own view of the carveout base (its DRAM window). */
#define AOC_DRAM_BASE			0x98000000
#define AOC_SENSOR_HEAP_SIZE		SZ_4M
#define AOC_PLAYBACK_HEAP_SIZE		SZ_16K
#define AOC_CAPTURE_HEAP_SIZE		SZ_64K

/*
 * The AOC power-on request register (the "aoc_req" block).  The AP writes 1 to
 * ask the AOC power controller to keep the core powered, and the controller
 * acknowledges at +0x40.  This must be asserted before the GSA starts the core,
 * or the core has no power and the start silently fails.
 */
#define AOC_REQ_PHYS			0x154b0000
#define AOC_REQ_SIZE			0x1000
#define AOC_REQ_ACK_OFFSET		0x40

/*
 * The AOC's SSMT (stream security management table).  It stamps the partition
 * IDs the AOC's DMA carries; the vendor sets every stream to "bypass" so the
 * transactions are accepted downstream.  Registers NS_READ_PID/NS_WRITE_PID.
 */
#define AOC_SSMT_PHYS			0x19060000
#define AOC_SSMT_SIZE			0x5000
#define SSMT_BYPASS_VALUE		0x80000000
#define SSMT_NS_READ_PID(n)		(0x4000 + 4 * (n))
#define SSMT_NS_WRITE_PID(n)		(0x4200 + 4 * (n))

/*
 * The AOC's SysMMU non-secure VM0 context, for reading back fault state after a
 * start attempt (base 0x19090000; see exynos-iommu v9 register map).
 */
#define AOC_SYSMMU_PHYS			0x19090000
#define AOC_SYSMMU_SIZE			0x10000
#define REG_V9_CTRL_VM			0x8000
#define REG_V9_FLPT_BASE_VM		0x8404
#define REG_V9_FAULT_VA_VM		0x8070
#define REG_V9_FAULT_INFO0_VM		0x8074

/* SoC chip-id (OTP) block - zumapro uses the zuma layout (product_ver 1). */
#define AOC_CHIPID_PHYS			0x10000000
#define CHIPID_PRODUCT_ID_MASK		0xfffff000
#define CHIPID_TYPE_MASK		0x000000ff
#define CHIPID_MAIN_REV_MASK		0x0000000f
#define CHIPID_SUB_REV_REG		0x10
#define CHIPID_SUB_REV_SHIFT		16

/*
 * The AOC's SysMMU mapping table lives in the signed 4K header, inside the
 * image_config (struct aoc_auth_header in google-modules/aoc/aoc_firmware.c).
 * signature[512] + key[512] precede the header body; the v2 body reaches
 * image_config at 1408 (v1 at 1376).  Within image_config the iommu table
 * offset/size are u16 at +20/+22, and each entry is a u64 of packed page
 * numbers (VA[19:0], PA[43:20], SIZE[63:44], all in 4K units).
 */
#define AOC_HDR_GENERATION_OFF		1028
#define AOC_IMG_CONFIG_V1_OFF		1376
#define AOC_IMG_CONFIG_V2_OFF		1408
#define AOC_IMG_CFG_IOMMU_OFF		20
#define AOC_IMG_CFG_IOMMU_SIZE		22
#define AOC_IOMMU_VA(v)			((((u64)(v) >> 0)  & 0xfffff)  << 12)
#define AOC_IOMMU_PA(v)			((((u64)(v) >> 20) & 0xffffff) << 12)
#define AOC_IOMMU_SZ(v)			((((u64)(v) >> 44) & 0xfffff)  << 12)

struct aoc_data {
	struct device *dev;
	struct device *gsa;		/* the GSA that authenticates our image */
	phys_addr_t carveout_base;
	size_t carveout_size;
	void *carveout;			/* mapped for staging the body */
	u32 ipc_offset;			/* control block offset within the carveout */
	u32 board_id;
	u32 board_rev;
	u32 chip_product_id;
	u32 chip_type;
	u32 chip_rev;
	struct iommu_domain *domain;	/* the AOC's SysMMU translation */

	/* Runtime IPC, valid once the AOC has published its control block. */
	void *ipc;			/* control block = ipc base = carveout + ipc_offset */
	u32 n_services;
	u32 service_size;
	u32 services_offset;
	struct aoc_service *svc_tbl;	/* one wrapper per published service */
	void __iomem *mbox[3];		/* the three AOC2AP mailbox blocks */
	struct aoc_mbox_irq {
		struct aoc_data *aoc;
		int block;
	} mbox_irq[3];
	atomic_t mbox_irqs;		/* AOC->AP interrupts seen (diagnostic) */

	/* The core's own console, which is the only view of what it is doing. */
	struct aoc_service *log_svc;
	struct delayed_work log_work;
	struct dentry *dbgfs;		/* service-table dump */

	/* Trusty channel to the GSA AOC hardware manager. */
	struct tipc_chan *tz_chan;
	struct completion tz_reply;
	struct mutex tz_req_lock;	/* serialises requests */
	struct mutex tz_rsp_lock;	/* protects the response buffer */
	void *tz_rsp;
	size_t tz_rsp_size;
	int tz_rsp_res;
};

/* Read the SoC chip-id the (chip-specific) AOC firmware validates at boot. */
static void aoc_read_chipid(struct aoc_data *aoc)
{
	void __iomem *cid = ioremap(AOC_CHIPID_PHYS, SZ_256);
	u32 raw;

	if (!cid)
		return;
	raw = readl(cid);
	aoc->chip_product_id = raw & CHIPID_PRODUCT_ID_MASK;
	aoc->chip_type = raw & CHIPID_TYPE_MASK;
	aoc->chip_rev = ((raw & CHIPID_MAIN_REV_MASK) << 4) |
			((readl(cid + CHIPID_SUB_REV_REG) >> CHIPID_SUB_REV_SHIFT) & 0xf);
	iounmap(cid);
	dev_info(aoc->dev, "chip id: product %#x type %#x rev %#x\n",
		 aoc->chip_product_id, aoc->chip_type, aoc->chip_rev);
}

/* Stamp every AOC DMA stream as "bypass" in the SSMT (mirrors the vendor). */
static void aoc_configure_ssmt(struct aoc_data *aoc)
{
	void __iomem *ssmt;
	int sid;

	ssmt = devm_ioremap(aoc->dev, AOC_SSMT_PHYS, AOC_SSMT_SIZE);
	if (!ssmt) {
		dev_warn(aoc->dev, "cannot map ssmt_aoc\n");
		return;
	}
	for (sid = 0; sid <= 32; sid++) {
		if (sid == 31)			/* the vendor skips stream 31 */
			continue;
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt + SSMT_NS_READ_PID(sid));
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt + SSMT_NS_WRITE_PID(sid));
	}
}

/* Ask the AOC power controller to keep the core powered, and wait for its ack. */
static int aoc_request_on(struct aoc_data *aoc)
{
	void __iomem *req;
	unsigned int i;
	u32 ack = 0;

	req = devm_ioremap(aoc->dev, AOC_REQ_PHYS, AOC_REQ_SIZE);
	if (!req)
		return -ENOMEM;

	writel(1, req);
	for (i = 0; i < 20; i++) {	/* up to ~2s, like the vendor */
		ack = readl(req + AOC_REQ_ACK_OFFSET);
		if (ack)
			break;
		msleep(100);
	}
	dev_info(aoc->dev, "aoc_req asserted, ack=%u\n", ack);
	return 0;
}

/* Write the boot parameter block the AOC reads at ipc_offset. */
static void aoc_write_params(struct aoc_data *aoc)
{
	volatile u32 *p = (volatile u32 *)((u8 *)aoc->carveout + aoc->ipc_offset);
	/* Heaps are carved from the end of the carveout, in the AOC's view. */
	u32 sensor_heap = AOC_DRAM_BASE + aoc->carveout_size - AOC_SENSOR_HEAP_SIZE;
	u32 playback_heap = sensor_heap - AOC_PLAYBACK_HEAP_SIZE;
	u32 capture_heap = playback_heap - AOC_CAPTURE_HEAP_SIZE;
	const struct { u32 key, val; } tbl[] = {
		{ kAOCBoardID,			aoc->board_id },
		{ kAOCBoardRevision,		aoc->board_rev },
		{ kAOCSRAMRepaired,		0 },
		{ kAOCCarveoutAddress,		lower_32_bits(aoc->carveout_base) },
		{ kAOCCarveoutSize,		aoc->carveout_size },
		{ kAOCSensorDirectHeapAddress,	sensor_heap },
		{ kAOCSensorDirectHeapSize,	AOC_SENSOR_HEAP_SIZE },
		{ kAOCPlaybackHeapAddress,	playback_heap },
		{ kAOCPlaybackHeapSize,		AOC_PLAYBACK_HEAP_SIZE },
		{ kAOCCaptureHeapAddress,	capture_heap },
		{ kAOCCaptureHeapSize,		AOC_CAPTURE_HEAP_SIZE },
		{ kAOCForceVNOM,		0 },
		{ kAOCDisableMM,		0 },
		{ kAOCEnableUART,		0 },
		{ kAOCForceSpeakerUltrasonic,	0 },
		{ kAOCRandSeed,			get_random_u32() },
		{ kAOCChipRevision,		aoc->chip_rev },
		{ kAOCChipType,			aoc->chip_type },
		{ kAOCGnssType,			0 },
		{ kAOCVolteReleaseMif,		0 },
		{ kAOCChipProductId,		aoc->chip_product_id },
		{ kAOCWifiChip,			0 },
	};
	unsigned int i, n = ARRAY_SIZE(tbl);

	*p++ = AOC_PARAMETER_MAGIC;
	*p++ = n;
	*p++ = 12 + n * 12;			/* header + n * (key,len,val) */
	for (i = 0; i < n; i++) {
		*p++ = tbl[i].key;
		*p++ = sizeof(u32);
		*p++ = tbl[i].val;
	}
	wmb();					/* land before the core starts */
}

/*
 * Program the AOC's SysMMU with the firmware's mapping table.  The AOC runs at
 * the fixed virtual addresses baked into its image (its code/IPC view is at
 * 0x98000000, backed by the 0x94000000 carveout), so without this the core
 * faults on its first fetch and the GSA reports it stuck.  The secure GSA path
 * would set this up on a stock device; here the AP reaches the SysMMU directly.
 */
static int aoc_setup_iommu(struct aoc_data *aoc, const struct firmware *fw)
{
	struct device *dev = aoc->dev;
	struct iommu_domain *domain;
	const __le64 *ent;
	unsigned int i, n;
	size_t cfg;
	u16 ioff, isize;
	u32 gen;
	int ret;

	if (!device_iommu_mapped(dev))
		return dev_err_probe(dev, -ENODEV,
				     "no SysMMU bound (check DT iommus=)\n");

	gen = le32_to_cpu(*(const __le32 *)(fw->data + AOC_HDR_GENERATION_OFF));
	cfg = (gen == 2) ? AOC_IMG_CONFIG_V2_OFF : AOC_IMG_CONFIG_V1_OFF;
	ioff = le16_to_cpu(*(const __le16 *)(fw->data + cfg + AOC_IMG_CFG_IOMMU_OFF));
	isize = le16_to_cpu(*(const __le16 *)(fw->data + cfg + AOC_IMG_CFG_IOMMU_SIZE));
	if (!isize || isize % sizeof(u64) ||
	    cfg + ioff + isize > AOC_AUTH_HEADER_SIZE) {
		dev_err(dev, "bad iommu table (off %#x size %#x)\n", ioff, isize);
		return -EINVAL;
	}
	ent = (const __le64 *)(fw->data + cfg + ioff);
	n = isize / sizeof(u64);

	domain = iommu_paging_domain_alloc(dev);
	if (IS_ERR(domain))
		return dev_err_probe(dev, PTR_ERR(domain),
				     "cannot allocate a paging domain\n");

	ret = iommu_attach_device(domain, dev);
	if (ret) {
		dev_err(dev, "cannot attach the SysMMU: %d\n", ret);
		goto err_free;
	}

	for (i = 0; i < n; i++) {
		u64 v = le64_to_cpu(ent[i]);
		u64 va = AOC_IOMMU_VA(v), pa = AOC_IOMMU_PA(v), sz = AOC_IOMMU_SZ(v);

		ret = iommu_map(domain, va, pa, sz, IOMMU_READ | IOMMU_WRITE,
				GFP_KERNEL);
		if (ret) {
			dev_err(dev, "iommu_map %#llx->%#llx (%#llx) failed: %d\n",
				va, pa, sz, ret);
			goto err_detach;
		}
		dev_dbg(dev, "iommu: VA %#llx -> PA %#llx size %#llx\n",
			va, pa, sz);
	}

	aoc->domain = domain;
	return 0;

err_detach:
	iommu_detach_device(domain, dev);
err_free:
	iommu_domain_free(domain);
	return ret;
}

/* Stage the image and have the GSA authenticate it. */
static int aoc_load_image(struct aoc_data *aoc, const struct firmware *fw)
{
	struct device *dev = aoc->dev;
	size_t body_size;
	dma_addr_t hdr_da;
	void *hdr;
	int ret;

	if (fw->size <= AOC_AUTH_HEADER_SIZE) {
		dev_err(dev, "firmware too small (%zu bytes)\n", fw->size);
		return -EINVAL;
	}
	body_size = fw->size - AOC_AUTH_HEADER_SIZE;
	if (body_size > aoc->carveout_size) {
		dev_err(dev, "body (%zu) exceeds carveout (%zu)\n",
			body_size, aoc->carveout_size);
		return -EFBIG;
	}

	aoc->ipc_offset = le32_to_cpu(*(const __le32 *)(fw->data +
			  AOC_AUTH_HEADER_SIZE + AOC_SUPERBIN_IMAGE_SIZE));

	/*
	 * The 4K signed header, in coherent memory the GSA reads by DMA.  It is
	 * allocated on the GSA device, not the AOC: the AOC now sits behind its
	 * SysMMU, so a coherent buffer on it would carry a translated IOVA the
	 * GSA (a separate bus master) cannot follow.
	 */
	hdr = dma_alloc_coherent(aoc->gsa, AOC_AUTH_HEADER_SIZE, &hdr_da, GFP_KERNEL);
	if (!hdr)
		return -ENOMEM;
	memcpy(hdr, fw->data, AOC_AUTH_HEADER_SIZE);

	/* The body, staged into the carveout for the GSA to verify + unlock. */
	memcpy(aoc->carveout, fw->data + AOC_AUTH_HEADER_SIZE, body_size);
	/* flush the write-combined body to DRAM before the GSA reads it */
	wmb();

	ret = gsa_load_aoc_fw_image(aoc->gsa, hdr_da, aoc->carveout_base);
	if (ret)
		dev_err(dev, "GSA rejected the AOC image: %d\n", ret);
	else
		dev_info(dev, "AOC firmware authenticated and loaded by the GSA\n");

	dma_free_coherent(aoc->gsa, AOC_AUTH_HEADER_SIZE, hdr, hdr_da);
	return ret;
}

/* struct aoc_control_block field offsets. */
#define AOC_CB_STATUS		0x0c
#define AOC_CB_SERVICES		0x18
#define AOC_CB_SERVICE_SIZE	0x1c
#define AOC_CB_SERVICES_OFFSET	0x20
#define AOC_CB_FW_VERSION_NAME	0x24
#define AOC_CB_HW_VERSION_NAME	0x54
#define AOC_VERSION_LEN		48
#define AOC_CB_SYSTEM_CLOCK_OFFSET 0x84
/* struct aoc_ipc_service_header: name[32], then flags and two ring regions. */
#define AOC_SERVICE_NAME_LEN	32

static u32 cb_rd(const void *cb, size_t off)
{
	return le32_to_cpu(*(const __le32 *)((const u8 *)cb + off));
}

/* Dump the running AOC's IPC control block and its service table. */
static void aoc_dump_control_block(struct aoc_data *aoc, const void *cb)
{
	u32 status   = cb_rd(cb, AOC_CB_STATUS);
	u32 services = cb_rd(cb, AOC_CB_SERVICES);
	u32 sz       = cb_rd(cb, AOC_CB_SERVICE_SIZE);
	u32 soff     = cb_rd(cb, AOC_CB_SERVICES_OFFSET);
	char fw[AOC_VERSION_LEN + 1], hw[AOC_VERSION_LEN + 1];
	unsigned int i;

	memcpy(fw, (const u8 *)cb + AOC_CB_FW_VERSION_NAME, AOC_VERSION_LEN);
	fw[AOC_VERSION_LEN] = '\0';
	memcpy(hw, (const u8 *)cb + AOC_CB_HW_VERSION_NAME, AOC_VERSION_LEN);
	hw[AOC_VERSION_LEN] = '\0';

	dev_info(aoc->dev, "fw '%s' hw '%s' status %#x, %u services (size %u, off %#x)\n",
		 fw, hw, status, services, sz, soff);

	if (services > 256) {
		dev_warn(aoc->dev, "implausible service count; skipping table\n");
		return;
	}
	for (i = 0; i < services; i++) {
		const void *svc = (const u8 *)cb + soff + (size_t)i * sz;
		char name[AOC_SERVICE_NAME_LEN + 1];

		memcpy(name, svc, AOC_SERVICE_NAME_LEN);
		name[AOC_SERVICE_NAME_LEN] = '\0';
		dev_dbg(aoc->dev, "  service[%u] = '%s'\n", i, name);
	}
}

/* Trusty channel: a reply arrived - copy it out and wake the waiter. */
static struct tipc_msg_buf *aoc_tz_handle_msg(void *data,
					      struct tipc_msg_buf *rxbuf)
{
	struct aoc_data *aoc = data;

	mutex_lock(&aoc->tz_rsp_lock);
	if (aoc->tz_rsp) {
		size_t len = mb_avail_data(rxbuf);

		if (len <= aoc->tz_rsp_size) {
			memcpy(aoc->tz_rsp, mb_get_data(rxbuf, len), len);
			aoc->tz_rsp_res = len;
		} else {
			aoc->tz_rsp_res = -EMSGSIZE;
		}
	}
	mutex_unlock(&aoc->tz_rsp_lock);
	complete(&aoc->tz_reply);

	return rxbuf;
}

/* Trusty channel: connect/disconnect events also wake the waiter. */
static void aoc_tz_handle_event(void *data, int event)
{
	struct aoc_data *aoc = data;

	complete(&aoc->tz_reply);
}

static const struct tipc_chan_ops aoc_tz_ops = {
	.handle_msg = aoc_tz_handle_msg,
	.handle_event = aoc_tz_handle_event,
};

/* Open the tipc channel to the GSA AOC hardware manager and wait for it. */
static int aoc_tz_connect(struct aoc_data *aoc)
{
	struct tipc_chan *chan;
	long left;
	int ret;

	chan = tipc_create_channel(NULL, &aoc_tz_ops, aoc);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	reinit_completion(&aoc->tz_reply);
	ret = tipc_chan_connect(chan, AOC_HWMGR_PORT);
	if (ret) {
		tipc_chan_destroy(chan);
		return ret;
	}

	left = wait_for_completion_timeout(&aoc->tz_reply,
					   msecs_to_jiffies(AOC_TZ_CONNECT_TIMEOUT_MS));
	if (left <= 0) {
		tipc_chan_shutdown(chan);
		tipc_chan_destroy(chan);
		return -ETIMEDOUT;
	}

	aoc->tz_chan = chan;
	return 0;
}

/*
 * Send an AOC hardware-manager state command over Trusty and return the AOC
 * state the GSA reports.  This is the path that programs the AOC's secure
 * SysMMU context and releases the core from reset - the mailbox cannot.
 */
static int aoc_tz_state_cmd(struct aoc_data *aoc, u32 cmd)
{
	struct hwmgr_state_req req = {
		.hdr_cmd = HWMGR_CMD_STATE_CMD,
		.state_cmd = cmd,
	};
	struct hwmgr_state_rsp rsp = { };
	struct tipc_msg_buf *txbuf;
	long left;
	int ret;

	mutex_lock(&aoc->tz_req_lock);

	if (!aoc->tz_chan) {
		ret = aoc_tz_connect(aoc);
		if (ret) {
			dev_err(aoc->dev, "cannot reach %s: %d\n",
				AOC_HWMGR_PORT, ret);
			goto out;
		}
		dev_info(aoc->dev, "connected to %s\n", AOC_HWMGR_PORT);
	}

	txbuf = tipc_chan_get_txbuf_timeout(aoc->tz_chan, AOC_TZ_BUF_TIMEOUT_MS);
	if (IS_ERR(txbuf)) {
		ret = PTR_ERR(txbuf);
		goto out;
	}
	memcpy(mb_put_data(txbuf, sizeof(req)), &req, sizeof(req));

	mutex_lock(&aoc->tz_rsp_lock);
	aoc->tz_rsp = &rsp;
	aoc->tz_rsp_size = sizeof(rsp);
	aoc->tz_rsp_res = -ENOMSG;
	mutex_unlock(&aoc->tz_rsp_lock);

	reinit_completion(&aoc->tz_reply);
	ret = tipc_chan_queue_msg(aoc->tz_chan, txbuf);
	if (ret) {
		tipc_chan_put_txbuf(aoc->tz_chan, txbuf);
		goto out_detach;
	}

	left = wait_for_completion_timeout(&aoc->tz_reply,
					   msecs_to_jiffies(AOC_TZ_MSG_TIMEOUT_MS));
	ret = (left <= 0) ? -ETIMEDOUT : aoc->tz_rsp_res;

out_detach:
	mutex_lock(&aoc->tz_rsp_lock);
	aoc->tz_rsp = NULL;
	aoc->tz_rsp_size = 0;
	mutex_unlock(&aoc->tz_rsp_lock);
out:
	mutex_unlock(&aoc->tz_req_lock);

	if (ret < 0)
		return ret;
	if (ret < (int)sizeof(rsp) ||
	    rsp.hdr_cmd != (HWMGR_CMD_STATE_CMD | HWMGR_CMD_RESP) || rsp.hdr_err)
		return -EIO;

	return rsp.state;
}

/* Poll the AOC state + control block over a few seconds; log the trajectory. */
static bool aoc_check_alive(struct aoc_data *aoc)
{
	const void *cb = (const u8 *)aoc->carveout + aoc->ipc_offset;
	static const unsigned int delays_ms[] = { 100, 400, 500, 1000, 1000 };
	unsigned int elapsed = 0;
	unsigned int i;
	u32 magic = 0;
	int state;

	if (aoc->ipc_offset + 0x100 > aoc->carveout_size) {
		dev_warn(aoc->dev, "ipc_offset %#x out of range\n", aoc->ipc_offset);
		return false;
	}

	for (i = 0; i < ARRAY_SIZE(delays_ms); i++) {
		msleep(delays_ms[i]);
		elapsed += delays_ms[i];

		state = aoc_tz_state_cmd(aoc, AOC_TZ_GET_STATE);
		magic = cb_rd(cb, 0);
		dev_dbg(aoc->dev, "+%ums: AOC state %d, control-block magic %#x\n",
			elapsed, state, magic);
		if (magic == AOC_MAGIC)
			break;
	}

	if (magic != AOC_MAGIC) {
		/* Not up.  Dump the first control-block words: the AOC may have
		 * left a crash signature even though it never published.
		 */
		const __le32 *w = cb;
		void __iomem *mmu;

		dev_info(aoc->dev,
			 "AOC did not publish (magic %#x); cb dump: %#x %#x %#x %#x %#x %#x %#x %#x\n",
			 magic, le32_to_cpu(w[0]), le32_to_cpu(w[1]),
			 le32_to_cpu(w[2]), le32_to_cpu(w[3]), le32_to_cpu(w[4]),
			 le32_to_cpu(w[5]), le32_to_cpu(w[6]), le32_to_cpu(w[7]));

		/*
		 * Read the SysMMU's VM0 state.  A non-zero fault VA means the AOC
		 * executed and faulted (in the non-secure context) at that
		 * address - proof the core ran.  CTRL/FLPT confirm translation
		 * was still live for the attempt.
		 */
		mmu = ioremap(AOC_SYSMMU_PHYS, AOC_SYSMMU_SIZE);
		if (mmu) {
			dev_info(aoc->dev,
				 "sysmmu VM0: ctrl %#x flpt %#x fault_va %#x fault_info %#x\n",
				 readl(mmu + REG_V9_CTRL_VM),
				 readl(mmu + REG_V9_FLPT_BASE_VM),
				 readl(mmu + REG_V9_FAULT_VA_VM),
				 readl(mmu + REG_V9_FAULT_INFO0_VM));
			iounmap(mmu);
		}
		return false;
	}

	dev_info(aoc->dev, "AOC alive: control block at carveout+%#x\n",
		 aoc->ipc_offset);
	aoc_dump_control_block(aoc, cb);
	return true;
}

/*
 * ---------------------------------------------------------------------------
 * Runtime IPC.
 *
 * Once the core is online it publishes a control block at ipc_offset holding a
 * table of named services.  Each service is a pair of memory regions (one per
 * direction) carved from the carveout: AOC_UP (index 0) is the AOC->AP stream
 * the AP reads, AOC_DOWN (index 1) is the AP->AOC stream the AP writes.  A
 * region is either a byte ring or a slotted message queue, addressed by the
 * shared tx/rx/wp/rp counters in its descriptor.  The AP notifies the AOC of a
 * write, and is notified of one, through a doorbell mailbox; each service names
 * the doorbell channel to use in its flags.
 *
 * The carveout is mapped write-combining and the AOC is not cache-coherent with
 * the AP, so the descriptor counters are accessed with READ_ONCE/WRITE_ONCE and
 * the payload with explicit barriers before a doorbell / after an interrupt.
 * ---------------------------------------------------------------------------
 */

#define AOC_UP			0	/* FW -> AP (AP reads) */
#define AOC_DOWN		1	/* AP -> FW (AP writes) */

/* struct aoc_ipc_service_header: name[32], flags, then two regions. */
#define SVC_FLAGS		0x20
#define SVC_REGIONS		0x24
#define REG_STRIDE		28	/* sizeof(struct aoc_ipc_memory_region) */
/* region field offsets */
#define REG_OFFSET		0x00	/* ring base, relative to the ipc base */
#define REG_SIZE		0x04	/* ring size, or queue slot size, in bytes */
#define REG_SLOTS		0x08	/* number of queue slots */
#define REG_TX			0x0c	/* bytes/slots written (by the region's writer) */
#define REG_RX			0x10	/* bytes/slots read (by the region's reader) */
#define REG_WP			0x14	/* write offset into the ring */
#define REG_RP			0x18	/* read offset into the ring */

#define SVC_TYPE_MASK		0x00070000
#define SVC_TYPE_SHIFT		16
#define SVC_TYPE_QUEUE		0
#define SVC_TYPE_RING		1
#define SVC_IRQ_MASK		0x00f80000
#define SVC_IRQ_SHIFT		19

#define AOC_MSG_HDR_SIZE	8	/* struct aoc_ipc_message_header (queues) */

/*
 * The AOC2AP doorbell mailboxes: three whitechapel blocks (for the A32, F1 and
 * P6 cores), 16 channels each, so a service's 0..47 channel index selects both
 * the block and the bit.  These match the vendor's mbox@1520/1522/1524_0000.
 */
#define AOC_MBOX_BLOCKS		3
#define AOC_MBOX_CH_PER_BLOCK	16
#define AOC_MBOX_SIZE		0x1000
#define MB_INTGR0		0x0020	/* AP -> AOC: generate interrupt */
#define MB_INTCR1		0x0044	/* AOC -> AP: clear interrupt */
#define MB_INTMR1		0x0048	/* AOC -> AP: interrupt mask */
#define MB_INTSR1		0x004c	/* AOC -> AP: interrupt status */
static const phys_addr_t aoc_mbox_phys[AOC_MBOX_BLOCKS] = {
	0x15200000, 0x15220000, 0x15240000,
};

/*
 * The control service's version command (from the vendor aoc-interface): a
 * write/read round-trip that returns the core's build hash and link time.  Used
 * as an end-to-end self-test of the IPC path.
 */
#define AOC_DATA_TYPE_CMD	0
#define CMD_SYS_VERSION_GET_ID	6
#define AOC_VERSION_STR_MAX	64
struct aoc_container_hdr {
	u8 type;
	u8 cntr;
	__le16 len;
} __packed;
struct aoc_cmd_hdr {
	struct aoc_container_hdr parent;
	__le16 id;
	__le16 reply;
} __packed;
struct cmd_sys_version_get {
	struct aoc_cmd_hdr hdr;
	__le32 core;
	char version[AOC_VERSION_STR_MAX];
	char link_time[AOC_VERSION_STR_MAX];
} __packed;

/* The carveout is write-combining; keep these accesses uncached and ordered. */
static u16 ipc_r16(const void *p)
{
	return le16_to_cpu(READ_ONCE(*(const __le16 *)p));
}

static void ipc_w16(void *p, u16 v)
{
	WRITE_ONCE(*(__le16 *)p, cpu_to_le16(v));
}

static u32 ipc_r32(const void *p)
{
	return le32_to_cpu(READ_ONCE(*(const __le32 *)p));
}

static void ipc_w32(void *p, u32 v)
{
	WRITE_ONCE(*(__le32 *)p, cpu_to_le32(v));
}

static void *svc_region(void *svc, int dir)
{
	return (u8 *)svc + SVC_REGIONS + dir * REG_STRIDE;
}

static u32 svc_flags(void *svc)
{
	return ipc_r32((u8 *)svc + SVC_FLAGS);
}

static int svc_type(void *svc)
{
	return (svc_flags(svc) & SVC_TYPE_MASK) >> SVC_TYPE_SHIFT;
}

static u32 svc_mbox(void *svc)
{
	return (svc_flags(svc) & SVC_IRQ_MASK) >> SVC_IRQ_SHIFT;
}

/*
 * A handle to one service, handed to consumers (see <linux/soc/google/aoc.h>).
 * It caches the header pointer and doorbell channel, and carries the optional
 * per-service interrupt handler.  The table is built once the AOC is online.
 */
struct aoc_service {
	void *hdr;			/* service header in the carveout */
	struct aoc_data *aoc;
	u32 mbox;			/* doorbell channel (0..47) */
	void (*handler)(struct aoc_service *svc, void *priv);
	void *priv;
};

/* Look a service up by name in the published table. */
static struct aoc_service *aoc_service_lookup(struct aoc_data *aoc,
					      const char *name)
{
	unsigned int i;

	if (!aoc->svc_tbl)
		return NULL;
	for (i = 0; i < aoc->n_services; i++) {
		char nm[AOC_SERVICE_NAME_LEN + 1];

		memcpy(nm, aoc->svc_tbl[i].hdr, AOC_SERVICE_NAME_LEN);
		nm[AOC_SERVICE_NAME_LEN] = '\0';
		if (!strcmp(nm, name))
			return &aoc->svc_tbl[i];
	}
	return NULL;
}

/* Ring the AOC's doorbell for the given 0..47 channel. */
static void aoc_signal(struct aoc_data *aoc, u32 channel)
{
	u32 blk = channel / AOC_MBOX_CH_PER_BLOCK;
	u32 bit = channel % AOC_MBOX_CH_PER_BLOCK;

	if (blk >= AOC_MBOX_BLOCKS || !aoc->mbox[blk])
		return;
	wmb();				/* IPC writes must land before the doorbell */
	writel(1U << bit, aoc->mbox[blk] + MB_INTGR0);
}

/* Is there a message for the AP to read on this service? */
static bool aoc_service_readable(void *svc)
{
	void *r = svc_region(svc, AOC_UP);

	return ipc_r32((u8 *)r + REG_TX) != ipc_r32((u8 *)r + REG_RX);
}

/* Write one queued message (AP -> AOC).  Returns 0 or a negative errno. */
static int aoc_queue_write(struct aoc_data *aoc, void *svc,
			   const void *buf, size_t len)
{
	void *r = svc_region(svc, AOC_DOWN);
	u32 size = ipc_r32((u8 *)r + REG_SIZE);
	u32 slots = ipc_r32((u8 *)r + REG_SLOTS);
	u32 tx = ipc_r32((u8 *)r + REG_TX);
	u32 rx = ipc_r32((u8 *)r + REG_RX);
	u32 wp = ipc_r32((u8 *)r + REG_WP);
	u8 *dst;
	u32 off;

	if (!slots || len + AOC_MSG_HDR_SIZE > size)
		return -EMSGSIZE;
	if ((tx - rx) >= slots)			/* queue full */
		return -EAGAIN;

	off = ipc_r32((u8 *)r + REG_OFFSET) + size * (tx % slots);
	dst = (u8 *)aoc->ipc + off;
	ipc_w16(dst, len);			/* message header: length + reserved */
	ipc_w16(dst + 2, 0);
	ipc_w16(dst + 4, 0);
	ipc_w16(dst + 6, 0);
	memcpy(dst + AOC_MSG_HDR_SIZE, buf, len);
	wmb();					/* data before the advanced index */
	ipc_w32((u8 *)r + REG_TX, tx + 1);
	ipc_w32((u8 *)r + REG_WP, (wp + 1) % size);
	return 0;
}

/* Read one queued message (AOC -> AP).  Returns the length or a negative errno. */
static int aoc_queue_read(struct aoc_data *aoc, void *svc, void *buf, size_t max)
{
	void *r = svc_region(svc, AOC_UP);
	u32 size = ipc_r32((u8 *)r + REG_SIZE);
	u32 slots = ipc_r32((u8 *)r + REG_SLOTS);
	u32 tx = ipc_r32((u8 *)r + REG_TX);
	u32 rx = ipc_r32((u8 *)r + REG_RX);
	u32 rp = ipc_r32((u8 *)r + REG_RP);
	const u8 *src;
	u32 off;
	u16 len;

	if (!slots || tx == rx)
		return -ENODATA;

	off = ipc_r32((u8 *)r + REG_OFFSET) + size * (rx % slots);
	src = (const u8 *)aoc->ipc + off;
	len = ipc_r16(src);
	if (len > max || len + AOC_MSG_HDR_SIZE > size)
		return -EMSGSIZE;

	rmb();					/* index seen before the data */
	memcpy(buf, src + AOC_MSG_HDR_SIZE, len);
	ipc_w32((u8 *)r + REG_RX, rx + 1);
	ipc_w32((u8 *)r + REG_RP, (rp + 1) % size);
	return len;
}

/* Drain bytes from a ring (AOC -> AP).  Returns the count or a negative errno. */
static int aoc_ring_read(struct aoc_data *aoc, void *svc, void *buf, size_t max)
{
	void *r = svc_region(svc, AOC_UP);
	u32 size = ipc_r32((u8 *)r + REG_SIZE);
	u32 tx = ipc_r32((u8 *)r + REG_TX);
	u32 rx = ipc_r32((u8 *)r + REG_RX);
	u32 wp = ipc_r32((u8 *)r + REG_WP);
	u32 rp = ipc_r32((u8 *)r + REG_RP);
	const u8 *ring = (const u8 *)aoc->ipc + ipc_r32((u8 *)r + REG_OFFSET);
	u32 avail, to_read;

	if (!size)
		return -ENODATA;

	/* If the writer lapped us, drop the stale bytes and keep the last ring. */
	if ((tx - rx) > size) {
		u32 adv = (tx - rx) - size;

		rx += adv;
		rp = (rp + adv) % size;
		ipc_w32((u8 *)r + REG_RX, rx);
		ipc_w32((u8 *)r + REG_RP, rp);
	}

	if (wp > rp)
		avail = wp - rp;
	else if (wp < rp)
		avail = wp + size - rp;
	else
		avail = (tx == rx) ? 0 : size;
	if (!avail)
		return -ENODATA;

	to_read = min(avail, (u32)max);
	rmb();
	if (rp + to_read <= size) {
		memcpy(buf, ring + rp, to_read);
	} else {
		u32 part = size - rp;

		memcpy(buf, ring + rp, part);
		memcpy((u8 *)buf + part, ring, to_read - part);
	}
	ipc_w32((u8 *)r + REG_RX, rx + to_read);
	ipc_w32((u8 *)r + REG_RP, (rp + to_read) % size);
	return to_read;
}

/* Fill a ring (AP -> AOC).  Returns the count written or a negative errno. */
static int aoc_ring_write(struct aoc_data *aoc, void *svc,
			  const void *buf, size_t len)
{
	void *r = svc_region(svc, AOC_DOWN);
	u32 size = ipc_r32((u8 *)r + REG_SIZE);
	u32 tx = ipc_r32((u8 *)r + REG_TX);
	u32 rx = ipc_r32((u8 *)r + REG_RX);
	u32 wp = ipc_r32((u8 *)r + REG_WP);
	u8 *ring = (u8 *)aoc->ipc + ipc_r32((u8 *)r + REG_OFFSET);
	const u8 *src = buf;
	u32 to_write;

	if (!size)
		return -ENODATA;
	/*
	 * Only what the AOC has already read may be written over.  tx and rx
	 * are free-running byte counters, so their difference is what is still
	 * in flight, and it stays right across their wraparound.
	 */
	to_write = min((u32)len, size - (tx - rx));
	if (!to_write)
		return 0;			/* full: the reader is behind */
	if (wp + to_write <= size) {
		memcpy(ring + wp, src, to_write);
	} else {
		u32 part = size - wp;

		memcpy(ring + wp, src, part);
		memcpy(ring, src + part, to_write - part);
	}
	wmb();					/* data before the advanced index */
	ipc_w32((u8 *)r + REG_TX, tx + to_write);
	ipc_w32((u8 *)r + REG_WP, (wp + to_write) % size);
	return to_write;
}

/*
 * The in-kernel service API (declared in <linux/soc/google/aoc.h>).  Consumers
 * hold an opaque struct aoc_service *; here we unwrap it to the header pointer
 * and the owning device, and dispatch on the service type.
 */

/*
 * The AOC stamps its events in its own clock domain: nanoseconds off the
 * architected counter, biased by the offset its firmware publishes in the
 * control block.  CLOCK_BOOTTIME advances with that same counter, so
 * expressing "now" in the AOC's domain and differencing the two converts
 * exactly, with no drift to correct for.
 */
#define AOC_TS_SANITY_NS	(10ULL * NSEC_PER_SEC)

u64 aoc_ts_to_boottime_ns(struct device *dev, u64 aoc_ts)
{
	struct aoc_data *aoc = platform_get_drvdata(to_platform_device(dev));
	u64 now_cnt, now_boot, aoc_now_ns, delta_ns, offset;
	u32 rate;

	if (!aoc || !aoc->ipc)
		return 0;

	/*
	 * CNTFRQ_EL0 rather than arch_timer_get_rate(): the latter is not
	 * exported to modules, and the two agree here because nothing
	 * overrides the rate -- the device tree's timer node carries no
	 * clock-frequency, so the arch timer takes CNTFRQ_EL0 as authoritative
	 * too.
	 */
	rate = arch_timer_get_cntfrq();
	if (!rate)
		return 0;

	offset = (u64)cb_rd(aoc->ipc, AOC_CB_SYSTEM_CLOCK_OFFSET) |
		 ((u64)cb_rd(aoc->ipc, AOC_CB_SYSTEM_CLOCK_OFFSET + 4) << 32);

	now_cnt = arch_timer_read_counter();
	now_boot = ktime_get_boottime_ns();
	aoc_now_ns = mul_u64_u64_div_u64(now_cnt - offset, NSEC_PER_SEC, rate);

	if (aoc_ts <= aoc_now_ns) {
		delta_ns = aoc_now_ns - aoc_ts;
		/* Too far back to believe, or before boot: let the caller stamp. */
		if (delta_ns > AOC_TS_SANITY_NS || delta_ns > now_boot)
			return 0;
		return now_boot - delta_ns;
	}

	/* Stamped just ahead of our read; the two clock reads race. */
	delta_ns = aoc_ts - aoc_now_ns;
	if (delta_ns > AOC_TS_SANITY_NS)
		return 0;

	return now_boot + delta_ns;
}
EXPORT_SYMBOL_GPL(aoc_ts_to_boottime_ns);

struct aoc_service *aoc_service_find(struct device *dev, const char *name)
{
	struct aoc_data *aoc = platform_get_drvdata(to_platform_device(dev));

	if (!aoc)
		return NULL;
	return aoc_service_lookup(aoc, name);
}
EXPORT_SYMBOL_GPL(aoc_service_find);

int aoc_service_read(struct aoc_service *svc, void *buf, size_t len)
{
	if (svc_type(svc->hdr) == SVC_TYPE_RING)
		return aoc_ring_read(svc->aoc, svc->hdr, buf, len);
	return aoc_queue_read(svc->aoc, svc->hdr, buf, len);
}
EXPORT_SYMBOL_GPL(aoc_service_read);

int aoc_service_write(struct aoc_service *svc, const void *buf, size_t len)
{
	int ret;

	if (svc_type(svc->hdr) == SVC_TYPE_RING) {
		ret = aoc_ring_write(svc->aoc, svc->hdr, buf, len);
	} else {
		ret = aoc_queue_write(svc->aoc, svc->hdr, buf, len);
		if (ret == 0)
			ret = len;		/* report the payload as accepted */
	}
	if (ret >= 0)
		aoc_signal(svc->aoc, svc->mbox);
	return ret;
}
EXPORT_SYMBOL_GPL(aoc_service_write);

bool aoc_service_can_read(struct aoc_service *svc)
{
	return aoc_service_readable(svc->hdr);
}
EXPORT_SYMBOL_GPL(aoc_service_can_read);

bool aoc_service_can_write(struct aoc_service *svc)
{
	void *r = svc_region(svc->hdr, AOC_DOWN);

	if (svc_type(svc->hdr) == SVC_TYPE_RING)
		return true;			/* ring writes always make progress */
	return ipc_r32((u8 *)r + REG_TX) - ipc_r32((u8 *)r + REG_RX) <
	       ipc_r32((u8 *)r + REG_SLOTS);
}
EXPORT_SYMBOL_GPL(aoc_service_can_write);

/*
 * The AOC's running position in this service's byte stream, for use as a PCM
 * hardware pointer: the bytes it has consumed on a playback (AP->AOC) service,
 * or produced on a capture (AOC->AP) one.
 */
u32 aoc_service_progress(struct aoc_service *svc, bool playback)
{
	void *r = svc_region(svc->hdr, playback ? AOC_DOWN : AOC_UP);

	return ipc_r32((u8 *)r + (playback ? REG_RX : REG_TX));
}
EXPORT_SYMBOL_GPL(aoc_service_progress);

void aoc_service_set_handler(struct aoc_service *svc,
			     void (*handler)(struct aoc_service *svc, void *priv),
			     void *priv)
{
	svc->priv = priv;
	smp_wmb();				/* priv visible before the handler */
	WRITE_ONCE(svc->handler, handler);
}
EXPORT_SYMBOL_GPL(aoc_service_set_handler);

/*
 * AOC -> AP doorbell: the core has produced or consumed something.  Demux each
 * asserted channel to the services that named it and call their handlers.
 */
static irqreturn_t aoc_mbox_isr(int irq, void *dev_id)
{
	struct aoc_mbox_irq *mi = dev_id;
	struct aoc_data *aoc = mi->aoc;
	void __iomem *base = aoc->mbox[mi->block];
	u32 status = readl(base + MB_INTSR1);

	if (!status)
		return IRQ_NONE;

	writel(status, base + MB_INTCR1);	/* ack every asserted channel */
	atomic_inc(&aoc->mbox_irqs);
	rmb();					/* see the AOC's writes after the ack */

	while (status) {
		unsigned int bit = __ffs(status);
		u32 channel = mi->block * AOC_MBOX_CH_PER_BLOCK + bit;
		unsigned int i;

		status &= ~(1U << bit);
		for (i = 0; i < aoc->n_services; i++) {
			struct aoc_service *svc = &aoc->svc_tbl[i];
			void (*h)(struct aoc_service *, void *);

			if (svc->mbox != channel)
				continue;
			h = READ_ONCE(svc->handler);
			if (h)
				h(svc, svc->priv);
		}
	}
	return IRQ_HANDLED;
}

/* Build the service table and map the doorbell mailboxes + their interrupts. */
static int aoc_ipc_init(struct aoc_data *aoc, struct platform_device *pdev)
{
	const void *cb = (const u8 *)aoc->carveout + aoc->ipc_offset;
	unsigned int i;
	int b, irq, ret;

	aoc->ipc = (u8 *)aoc->carveout + aoc->ipc_offset;
	aoc->n_services = cb_rd(cb, AOC_CB_SERVICES);
	aoc->service_size = cb_rd(cb, AOC_CB_SERVICE_SIZE);
	aoc->services_offset = cb_rd(cb, AOC_CB_SERVICES_OFFSET);
	atomic_set(&aoc->mbox_irqs, 0);

	if (!aoc->n_services || aoc->n_services > 256)
		return -EINVAL;

	/* One handle per service; the interrupt path indexes this table. */
	aoc->svc_tbl = devm_kcalloc(aoc->dev, aoc->n_services,
				    sizeof(*aoc->svc_tbl), GFP_KERNEL);
	if (!aoc->svc_tbl)
		return -ENOMEM;
	for (i = 0; i < aoc->n_services; i++) {
		void *hdr = (u8 *)aoc->ipc + aoc->services_offset +
			    (size_t)i * aoc->service_size;

		aoc->svc_tbl[i].hdr = hdr;
		aoc->svc_tbl[i].aoc = aoc;
		aoc->svc_tbl[i].mbox = svc_mbox(hdr);
	}

	for (b = 0; b < AOC_MBOX_BLOCKS; b++) {
		aoc->mbox[b] = devm_ioremap(aoc->dev, aoc_mbox_phys[b],
					    AOC_MBOX_SIZE);
		if (!aoc->mbox[b])
			return -ENOMEM;
		writel(0xffff, aoc->mbox[b] + MB_INTCR1);	/* clear stale */
		writel(0, aoc->mbox[b] + MB_INTMR1);		/* unmask all */

		aoc->mbox_irq[b].aoc = aoc;
		aoc->mbox_irq[b].block = b;
		irq = platform_get_irq(pdev, b);
		if (irq < 0)
			return irq;
		ret = devm_request_irq(aoc->dev, irq, aoc_mbox_isr, 0,
				       "aoc-mbox", &aoc->mbox_irq[b]);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * End-to-end self-test: ask the control service for the A32 core's build info.
 * A successful round-trip proves the doorbell, the queue write and the queue
 * read all work against the live firmware.  We poll for the reply rather than
 * wait on the interrupt so the data path is validated even if delivery is not.
 */
static void aoc_log_work(struct work_struct *w)
{
	struct aoc_data *aoc = container_of(to_delayed_work(w), struct aoc_data,
					    log_work);
	char buf[257];
	int ret, n;

	for (n = 0; n < 64; n++) {
		ret = aoc_service_read(aoc->log_svc, buf, sizeof(buf) - 1);
		if (ret <= 0)
			break;
		buf[ret] = '\0';
		dev_dbg(aoc->dev, "AOC: %s\n", buf);
	}
	schedule_delayed_work(&aoc->log_work, msecs_to_jiffies(200));
}

static void aoc_ipc_selftest(struct aoc_data *aoc)
{
	struct cmd_sys_version_get cmd = { };
	struct aoc_service *ctrl, *log;
	char scratch[256];
	int ret, i;

	ctrl = aoc_service_lookup(aoc, "control");
	if (!ctrl) {
		dev_warn(aoc->dev, "no control service in the table\n");
		return;
	}
	if (svc_type(ctrl->hdr) != SVC_TYPE_QUEUE) {
		dev_warn(aoc->dev, "control service is not a queue (type %d)\n",
			 svc_type(ctrl->hdr));
		return;
	}

	/* Discard anything stale left in the reply direction (bounded). */
	for (i = 0; i < 64 && aoc_service_can_read(ctrl); i++)
		aoc_service_read(ctrl, scratch, sizeof(scratch));

	cmd.hdr.parent.type = AOC_DATA_TYPE_CMD;
	cmd.hdr.parent.len = cpu_to_le16(sizeof(cmd));
	cmd.hdr.id = cpu_to_le16(CMD_SYS_VERSION_GET_ID);
	cmd.core = cpu_to_le32(1);			/* the A32 core */

	ret = aoc_service_write(ctrl, &cmd, sizeof(cmd));	/* writes + signals */
	if (ret < 0) {
		dev_err(aoc->dev, "control write failed: %d\n", ret);
		return;
	}

	for (i = 0; i < 100; i++) {			/* up to ~1s */
		if (aoc_service_can_read(ctrl))
			break;
		msleep(10);
	}

	ret = aoc_service_read(ctrl, &cmd, sizeof(cmd));
	if (ret < 0) {
		dev_err(aoc->dev, "control read failed: %d (mbox irqs=%d)\n",
			ret, atomic_read(&aoc->mbox_irqs));
		return;
	}

	dev_info(aoc->dev,
		 "IPC round-trip OK: A32 build '%.64s' linked '%.64s' (mbox irqs=%d)\n",
		 cmd.version, cmd.link_time, atomic_read(&aoc->mbox_irqs));

	/* Bonus RX proof: surface whatever the AOC has logged so far. */
	log = aoc_service_lookup(aoc, "logging");
	if (log) {
		ret = aoc_service_read(log, scratch, sizeof(scratch) - 1);
		if (ret > 0) {
			scratch[ret] = '\0';
			dev_info(aoc->dev, "AOC log: %s\n", scratch);
		}
		/* Keep draining it: what the core says as it runs is the only
		 * account we get of what it thinks it is doing.
		 */
		aoc->log_svc = log;
		schedule_delayed_work(&aoc->log_work, msecs_to_jiffies(200));
	}
}

/*
 * The published service table, dumped live on read.  The AOC creates services
 * as its subsystems come up, so this is not a boot-time constant: a consumer
 * driver (Bluetooth, say) needs to know whether its service exists yet, and
 * whether powering the peripheral makes one appear.  One line per service:
 *
 *	index name type mbox
 *
 * This is debugfs rather than sysfs on purpose -- a sysfs show() is capped at
 * one page, which silently truncated the tail of a 94-entry table (and the
 * interesting entries are exactly the ones a firmware appends last).  seq_file
 * paginates properly.
 */
static int aoc_services_show(struct seq_file *s, void *unused)
{
	static const char * const types[] = { "queue", "ring", "buffer" };
	struct aoc_data *aoc = s->private;
	u32 live, size, soff;
	unsigned int i;

	if (!aoc || !aoc->ipc) {
		seq_puts(s, "AOC offline\n");
		return 0;
	}

	/*
	 * Re-read the control block rather than walking the table cached at
	 * probe: the question this file exists to answer is whether the AOC
	 * publishes a service later (when a peripheral is powered), and a
	 * cached count can never show that.  Entries past the cached
	 * n_services have no handle in svc_tbl yet, so they are flagged: they
	 * would need a re-probe (or a table rebuild) before a consumer driver
	 * could bind to them.
	 */
	live = cb_rd(aoc->ipc, AOC_CB_SERVICES);
	size = cb_rd(aoc->ipc, AOC_CB_SERVICE_SIZE);
	soff = cb_rd(aoc->ipc, AOC_CB_SERVICES_OFFSET);

	seq_printf(s, "# %u services live (%u at probe), size %u, off %#x\n",
		   live, aoc->n_services, size, soff);

	if (!size || live > 256) {
		seq_puts(s, "# implausible table; not walking it\n");
		return 0;
	}

	for (i = 0; i < live; i++) {
		void *hdr = (u8 *)aoc->ipc + soff + (size_t)i * size;
		char nm[AOC_SERVICE_NAME_LEN + 1];
		int t = svc_type(hdr);

		memcpy(nm, hdr, AOC_SERVICE_NAME_LEN);
		nm[AOC_SERVICE_NAME_LEN] = '\0';
		seq_printf(s, "%3u %-32s %-6s %2u%s\n", i, nm,
			   t < (int)ARRAY_SIZE(types) ? types[t] : "?",
			   svc_mbox(hdr),
			   i >= aoc->n_services ? "  [appeared after probe]" : "");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aoc_services);

static int aoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *gsa_pdev;
	const struct firmware *fw;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct aoc_data *aoc;
	int ret;

	aoc = devm_kzalloc(dev, sizeof(*aoc), GFP_KERNEL);
	if (!aoc)
		return -ENOMEM;
	aoc->dev = dev;
	mutex_init(&aoc->tz_req_lock);
	INIT_DELAYED_WORK(&aoc->log_work, aoc_log_work);
	mutex_init(&aoc->tz_rsp_lock);
	init_completion(&aoc->tz_reply);
	platform_set_drvdata(pdev, aoc);

	/* The GSA that will authenticate our image. */
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no gsa-device phandle\n");
	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!gsa_pdev)
		return -EPROBE_DEFER;
	aoc->gsa = &gsa_pdev->dev;
	if (!aoc->gsa->driver) {
		put_device(aoc->gsa);
		return -EPROBE_DEFER;
	}

	/* The AOC DRAM carveout the firmware body is staged into. */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		ret = dev_err_probe(dev, -EINVAL, "no memory-region\n");
		goto err_put_gsa;
	}
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem) {
		ret = dev_err_probe(dev, -EINVAL, "bad memory-region\n");
		goto err_put_gsa;
	}
	aoc->carveout_base = rmem->base;
	aoc->carveout_size = rmem->size;

	/*
	 * Write-combining, not cached: the GSA reads the staged body straight
	 * from DRAM, so our writes must not linger in the CPU cache.
	 */
	aoc->carveout = devm_memremap(dev, aoc->carveout_base,
				      aoc->carveout_size, MEMREMAP_WC);
	if (IS_ERR(aoc->carveout)) {
		ret = PTR_ERR(aoc->carveout);
		goto err_put_gsa;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		goto err_put_gsa;

	of_property_read_u32(dev->of_node, "aoc-board-id", &aoc->board_id);
	of_property_read_u32(dev->of_node, "aoc-board-rev", &aoc->board_rev);
	aoc_read_chipid(aoc);

	dev_info(dev, "carveout %pa (%zu MiB), gsa %s\n", &aoc->carveout_base,
		 aoc->carveout_size >> 20, dev_name(aoc->gsa));

	/*
	 * Before touching anything: sample the AOC state and the carveout fence.
	 * A non-INACTIVE state or a non-0xff carveout word here means the AOC was
	 * already booted (by the bootloader) and re-loading it is the wrong move.
	 */
	dev_dbg(dev, "pre-load: GSA state %d, carveout[0]=%#x\n",
		gsa_aoc_get_state(aoc->gsa),
		le32_to_cpu(*(const __le32 *)aoc->carveout));

	/* Loaded as a module after the rootfs is up, so a plain synchronous
	 * request is fine and reports the authentication result inline.
	 */
	ret = request_firmware(&fw, AOC_FIRMWARE_NAME, dev);
	if (ret) {
		dev_err(dev, "cannot load %s: %d\n", AOC_FIRMWARE_NAME, ret);
		goto err_put_gsa;
	}
	dev_info(dev, "loaded %s (%zu bytes)\n", AOC_FIRMWARE_NAME, fw->size);

	/* Ask the power controller to keep the AOC on before we start it. */
	aoc_request_on(aoc);
	aoc_configure_ssmt(aoc);

	aoc->dbgfs = debugfs_create_dir("aoc", NULL);
	debugfs_create_file("services", 0444, aoc->dbgfs, aoc,
			    &aoc_services_fops);

	if (aoc_load_image(aoc, fw) == 0) {
		dev_dbg(dev, "post-load pre-start: GSA state %d\n",
			gsa_aoc_get_state(aoc->gsa));
		/*
		 * Program the SysMMU from the image's mapping table so the core
		 * can reach its DRAM/MMIO, then hand it its boot parameters.
		 */
		ret = aoc_setup_iommu(aoc, fw);
		if (ret) {
			release_firmware(fw);
			goto err_put_gsa;
		}
		aoc_write_params(aoc);
		/*
		 * The core's reset and its SysMMU secure context are in the
		 * secure domain - the AP cannot touch them and the GSA mailbox
		 * will not - so ask the GSA AOC hardware manager, over Trusty,
		 * to set the AOC up and take it out of reset.
		 */
		ret = aoc_tz_state_cmd(aoc, AOC_TZ_START);
		if (ret < 0) {
			dev_err(dev, "Trusty failed to start the AOC: %d\n", ret);
		} else {
			dev_info(dev, "AOC started via Trusty (state now %d)\n", ret);
			if (aoc_check_alive(aoc)) {
				ret = aoc_ipc_init(aoc, pdev);
				if (ret)
					dev_warn(dev, "AOC IPC init failed: %d\n",
						 ret);
				else
					aoc_ipc_selftest(aoc);
			}
		}
	}
	release_firmware(fw);

	return 0;

err_put_gsa:
	put_device(aoc->gsa);
	return ret;
}

static void aoc_remove(struct platform_device *pdev)
{
	struct aoc_data *aoc = platform_get_drvdata(pdev);

	debugfs_remove_recursive(aoc->dbgfs);
	cancel_delayed_work_sync(&aoc->log_work);
	if (aoc->tz_chan) {
		tipc_chan_shutdown(aoc->tz_chan);
		tipc_chan_destroy(aoc->tz_chan);
	}
	if (aoc->domain) {
		iommu_detach_device(aoc->domain, aoc->dev);
		iommu_domain_free(aoc->domain);
	}
	put_device(aoc->gsa);
}


static const struct of_device_id aoc_of_match[] = {
	{ .compatible = "google,aoc" },
	{}
};
MODULE_DEVICE_TABLE(of, aoc_of_match);

static struct platform_driver aoc_driver = {
	.probe = aoc_probe,
	.remove = aoc_remove,
	.driver = {
		.name = "google-aoc",
		.of_match_table = aoc_of_match,
	},
};
module_platform_driver(aoc_driver);

MODULE_DESCRIPTION("Google AOC coprocessor firmware loader");
MODULE_LICENSE("GPL");
