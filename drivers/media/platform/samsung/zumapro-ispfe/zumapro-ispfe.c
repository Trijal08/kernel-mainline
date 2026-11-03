// SPDX-License-Identifier: GPL-2.0
/*
 * Google Tensor G4 (zumapro) camera front end: MIPI CSI-2 DC-PHY, CSIS link
 * and the ISPFE "logical channel" Bayer WDMA, exposed as a raw V4L2 capture
 * node fed straight from the sensor.
 *
 * One sensor per port; port N is CSIS link N, captured through logical
 * channel N. The register-level bring-up is the vendor HAL's, read off the
 * stock device (see the Pixel 9 Pro port notes).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

/* CSIS link, one per 0x10000 */
#define LINK(n)			((n) * 0x10000)
#define LINK_CMN_CTRL		0x04
#define LINK_CMN_CTRL_SW_RESET	BIT(1)
#define LINK_UPD_SDW		0x0c
#define LINK_INT0_MSK		0x10
#define LINK_INT0_SRC		0x14
#define LINK_INT1_MSK		0x18
#define LINK_INT1_SRC		0x1c
#define LINK_FS_INT_MSK		0x20
#define LINK_FE_INT_MSK		0x28
#define LINK_ISP_CONFIG_CH(c)	(0x40 + (c) * 0x10)
#define LINK_ISP_RESOL_CH(c)	(0x44 + (c) * 0x10)

/* DC-PHY, inside the CSIS block */
#define PHY_BASE		0xc0000
#define PHY_BIAS_CON(i)		(PHY_BASE + 0x1000 + (i) * 4)
#define PHY_LANE_GNR_CON0	0x00
#define PHY_LANE_GNR_CON1	0x04
#define PHY_LANE_ANA_CON(i)	(0x08 + (i) * 4)
#define PHY_LANE_TIME_CON0	0x30
#define PHY_LANE_TIME_CON1	0x34
#define PHY_LANE_DESKEW_CON0	0x40
#define PHY_LANE_DESKEW_CON4	0x50
#define PHY_LANE_CRC_DLY_CON0	0x5c
#define PHY_LANE_CRC_CON1	0x64
#define PHY_LANE_CRC_CON2	0x68
#define PHY_LANE_CRC_DLY_CON3	0x9c

#define PMU_MIPI_PHY_CON	0x3ebc	/* bit 0: bypass isolation, all slices */

/* ISPFE: FE reset block and the CSIS core */
#define FE_HW_MODE_CFG0		0x20008
#define FE_HW_MODE_CFG2		0x20010
#define CORE_LOCH_INT_ISR	0x2008c
#define CORE_LOCH_INT_IMR	0x20094
#define CORE_LOCH_EN_SET	0x200a0
#define CORE_LOCH_EN_CLR	0x200a4
#define CORE_FRAME_ID_CLR	0x200b0
#define LOCH(n)			(0x20400 + (n) * 0x400)
#define LOCH_WDMA_CTX_EN	0x18
#define LOCH_WDMA_CFG_EN	0x1c
#define LOCH_PROC_ISR		0x24
#define LOCH_PROC_IER		0x2c
#define LOCH_PROC_IMR		0x30
#define LOCH_PROC_SOF		BIT(0)
#define LOCH_PROC_EOF		BIT(1)
#define LOCH_PROC_SHADOW	BIT(3)
#define LOCH_PROC_DMA_DONE	BIT(4)
#define LOCH_ERR_ISR		0x38
#define LOCH_BATCH_MSK_SOF	0x60
#define LOCH_BATCH_MSK_EOF	0x64
#define LOCH_BATCH_MSK_DMA	0x6c
#define LOCH_BATCH_MSK_FRAME	0x70
#define LOCH_ISP_RESOL		0x74
#define LOCH_BATCH_CTL		0x88
#define LOCH_CTL		0x8c
#define LOCH_CTL_LINK(l)	((l) << 24)
#define LOCH_CTL_FMT_RAW10	(2 << 8)
#define LOCH_CFG_MISS		0xac
#define LOCH_WDMA_EN		0x140
#define LOCH_WDMA_CROP_EN	0x144
#define LOCH_WDMA_MEM_FORMAT	0x150
#define LOCH_WDMA_STRIDE	0x154
#define LOCH_WDMA_BASE_LSB	0x1c0
#define LOCH_WDMA_BASE_MSB	0x1c4

#define MEM_FORMAT_RAW10_16	2	/* 16-bit, LSB aligned: V4L2 SRGGB10 */
#define MEM_FORMAT_RAW10_PACKED	4	/* CSI-2 packed: V4L2 SRGGB10P */

#define ISPFE_RATE_STREAMING	711000000
#define ISPFE_RATE_IDLE		111000000

/* Which PHY slice a link's lanes sit on, and the link control word. */
struct ispfe_link_cfg {
	u8 link;
	u16 slice;		/* offset of the slice in the PHY block, >> 8 */
	u8 phy_reset;		/* bit in SYSREG_ISPFE 0x500 */
	u32 cmn_ctrl;
};

static const struct ispfe_link_cfg ispfe_links[] = {
	{ .link = 6, .slice = 0x31, .phy_reset = 5, .cmn_ctrl = 0x205e01 },	/* front IMX858, 3-trio C-PHY */
};

static const struct ispfe_fmt {
	u32 fourcc;
	u32 code;
	u8 mem_format;
	u8 bpp_num, bpp_den;	/* bytes per pixel as a fraction */
} ispfe_fmts[] = {
	{ V4L2_PIX_FMT_SRGGB10P, MEDIA_BUS_FMT_SRGGB10_1X10, MEM_FORMAT_RAW10_PACKED, 5, 4 },
	{ V4L2_PIX_FMT_SBGGR10P, MEDIA_BUS_FMT_SBGGR10_1X10, MEM_FORMAT_RAW10_PACKED, 5, 4 },
	{ V4L2_PIX_FMT_SGBRG10P, MEDIA_BUS_FMT_SGBRG10_1X10, MEM_FORMAT_RAW10_PACKED, 5, 4 },
	{ V4L2_PIX_FMT_SGRBG10P, MEDIA_BUS_FMT_SGRBG10_1X10, MEM_FORMAT_RAW10_PACKED, 5, 4 },
	{ V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, MEM_FORMAT_RAW10_16, 2, 1 },
	{ V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, MEM_FORMAT_RAW10_16, 2, 1 },
	{ V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, MEM_FORMAT_RAW10_16, 2, 1 },
	{ V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, MEM_FORMAT_RAW10_16, 2, 1 },
};

struct ispfe_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct ispfe {
	struct device *dev;
	void __iomem *csis;
	void __iomem *core;
	void __iomem *sysreg;
	struct regmap *pmu;
	struct clk_bulk_data *clks;
	int nclks;

	const struct ispfe_link_cfg *cfg;
	struct v4l2_subdev *sensor;
	struct fwnode_handle *sensor_ep;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock;
	struct v4l2_pix_format pix;
	const struct ispfe_fmt *fmt;
	u32 sensor_w, sensor_h;

	spinlock_t qlock;
	struct list_head queued;
	struct ispfe_buf *active;
	unsigned int sequence;
};

static inline void link_wr(struct ispfe *fe, u32 off, u32 v)
{
	writel(v, fe->csis + LINK(fe->cfg->link) + off);
}

static inline void lane_wr(struct ispfe *fe, int lane, u32 off, u32 v)
{
	writel(v, fe->csis + PHY_BASE + (fe->cfg->slice << 8) + (lane + 1) * 0x100 + off);
}

static inline void loch_wr(struct ispfe *fe, u32 off, u32 v)
{
	writel(v, fe->core + LOCH(fe->cfg->link) + off);
}

static inline u32 loch_rd(struct ispfe *fe, u32 off)
{
	return readl(fe->core + LOCH(fe->cfg->link) + off);
}

static const struct ispfe_fmt *ispfe_find_fmt(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ispfe_fmts); i++)
		if (ispfe_fmts[i].fourcc == fourcc)
			return &ispfe_fmts[i];
	return NULL;
}

/* -------------------------------------------------------------------------- */
/* Hardware */


static void ispfe_phy_up(struct ispfe *fe)
{
	static const u32 bias[] = { 0x10, 0x110, 0x3223, 0, 0x240, 0 };
	int i, lane;

	regmap_update_bits(fe->pmu, PMU_MIPI_PHY_CON, BIT(0), BIT(0));
	writel(BIT(fe->cfg->phy_reset), fe->sysreg);	/* SYSREG_ISPFE 0x500: per-PHY reset release */
	for (i = 0; i < ARRAY_SIZE(bias); i++)
		writel(bias[i], fe->csis + PHY_BIAS_CON(i));
	/* the slice's clock-lane slot, left as the probe sequence had it */
	writel(0xe08, fe->csis + PHY_BASE + (fe->cfg->slice << 8) + PHY_LANE_GNR_CON0);
	for (lane = 0; lane < 3; lane++) {
		lane_wr(fe, lane, PHY_LANE_GNR_CON0, 0);
		lane_wr(fe, lane, PHY_LANE_GNR_CON1, 0x1334);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(0), 0x1);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(1), 0xf23b);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(2), 0x5);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(3), 0x8600);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(4), 0x4040);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(5), 0x200);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(6), 0xe08);
		lane_wr(fe, lane, PHY_LANE_ANA_CON(7), 0x40);
		lane_wr(fe, lane, PHY_LANE_TIME_CON0, 0);
		lane_wr(fe, lane, PHY_LANE_TIME_CON1, 0x34);
		lane_wr(fe, lane, PHY_LANE_DESKEW_CON0, 0);
		lane_wr(fe, lane, PHY_LANE_DESKEW_CON4, 0);
		lane_wr(fe, lane, PHY_LANE_CRC_DLY_CON0, 0x1000);
		lane_wr(fe, lane, PHY_LANE_CRC_CON1, 0x1501);
		lane_wr(fe, lane, PHY_LANE_CRC_CON2, 0x3);
		lane_wr(fe, lane, PHY_LANE_CRC_DLY_CON3, 0x80);
	}
}

static void ispfe_phy_down(struct ispfe *fe)
{
	int lane;

	for (lane = 0; lane < 3; lane++)
		lane_wr(fe, lane, PHY_LANE_GNR_CON0, 0);
	writel(0, fe->csis + PHY_BASE + (fe->cfg->slice << 8) + PHY_LANE_GNR_CON0);
	regmap_update_bits(fe->pmu, PMU_MIPI_PHY_CON, BIT(0), 0);
}

static void ispfe_link_up(struct ispfe *fe, u32 width, u32 height)
{
	void __iomem *l = fe->csis + LINK(fe->cfg->link);
	int lane;

	writel(readl(l + LINK_CMN_CTRL) | LINK_CMN_CTRL_SW_RESET, l + LINK_CMN_CTRL);
	link_wr(fe, LINK_INT0_MSK, 0xffffffff);
	link_wr(fe, LINK_INT1_MSK, 0xffffffff);
	/* ch0: VC0 DT 0x2b (RAW10); ch1: VC1, the sensor's embedded/PDAF lines */
	link_wr(fe, LINK_ISP_CONFIG_CH(0), 0x30ac);
	link_wr(fe, LINK_ISP_RESOL_CH(0), height << 16 | width);
	link_wr(fe, LINK_ISP_CONFIG_CH(1), 0x130ac);
	link_wr(fe, LINK_ISP_RESOL_CH(1), 684 << 16 | width);
	link_wr(fe, LINK_UPD_SDW, 0xf);
	link_wr(fe, LINK_CMN_CTRL, 0x5201);
	link_wr(fe, LINK_FS_INT_MSK, 0);
	link_wr(fe, LINK_FE_INT_MSK, 0);
	link_wr(fe, LINK_CMN_CTRL, fe->cfg->cmn_ctrl);
	for (lane = 0; lane < 3; lane++)
		lane_wr(fe, lane, PHY_LANE_GNR_CON0, 3);
}

static void ispfe_arm(struct ispfe *fe, struct ispfe_buf *buf)
{
	dma_addr_t dma = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);

	loch_wr(fe, LOCH_WDMA_BASE_LSB, lower_32_bits(dma));
	loch_wr(fe, LOCH_WDMA_BASE_MSB, upper_32_bits(dma));
	loch_wr(fe, LOCH_WDMA_EN, 1);
	loch_wr(fe, LOCH_WDMA_CFG_EN, 1);
	fe->active = buf;
}

/* qlock held */
static void ispfe_arm_next(struct ispfe *fe)
{
	struct ispfe_buf *buf;

	if (fe->active || list_empty(&fe->queued))
		return;
	buf = list_first_entry(&fe->queued, struct ispfe_buf, list);
	list_del(&buf->list);
	ispfe_arm(fe, buf);
}

static void ispfe_loch_up(struct ispfe *fe)
{
	writel(BIT(fe->cfg->link), fe->core + CORE_FRAME_ID_CLR);
	loch_wr(fe, LOCH_ISP_RESOL, fe->sensor_h << 16 | fe->sensor_w);
	loch_wr(fe, LOCH_BATCH_CTL, 0);
	loch_wr(fe, LOCH_BATCH_MSK_SOF, 1);
	loch_wr(fe, LOCH_BATCH_MSK_EOF, 1);
	loch_wr(fe, LOCH_BATCH_MSK_DMA, 1);
	loch_wr(fe, LOCH_BATCH_MSK_FRAME, 1);
	loch_wr(fe, LOCH_PROC_ISR, 0xffffffff);
	loch_wr(fe, LOCH_PROC_IER, LOCH_PROC_SOF | LOCH_PROC_EOF | LOCH_PROC_SHADOW | LOCH_PROC_DMA_DONE);
	loch_wr(fe, LOCH_PROC_IMR, LOCH_PROC_DMA_DONE);
	loch_wr(fe, LOCH_WDMA_STRIDE, fe->pix.bytesperline);
	loch_wr(fe, LOCH_WDMA_MEM_FORMAT, fe->fmt->mem_format);
	loch_wr(fe, LOCH_WDMA_CROP_EN, 0);
	loch_wr(fe, LOCH_CTL, LOCH_CTL_LINK(fe->cfg->link) | LOCH_CTL_FMT_RAW10);
	loch_wr(fe, LOCH_WDMA_CTX_EN, 1);
	writel(readl(fe->core + CORE_LOCH_INT_IMR) | BIT(fe->cfg->link), fe->core + CORE_LOCH_INT_IMR);
	writel(BIT(fe->cfg->link), fe->core + CORE_LOCH_EN_SET);
}

static void ispfe_loch_down(struct ispfe *fe)
{
	writel(BIT(fe->cfg->link), fe->core + CORE_LOCH_EN_CLR);
	loch_wr(fe, LOCH_PROC_IMR, 0);
	loch_wr(fe, LOCH_WDMA_CTX_EN, 0);
	loch_wr(fe, LOCH_WDMA_CFG_EN, 0);
	loch_wr(fe, LOCH_WDMA_EN, 0);
}

static irqreturn_t ispfe_irq(int irq, void *data)
{
	struct ispfe *fe = data;
	u32 isr = loch_rd(fe, LOCH_PROC_ISR);
	struct ispfe_buf *buf;

	if (!isr)
		return IRQ_NONE;
	loch_wr(fe, LOCH_PROC_ISR, isr);
	if (!(isr & LOCH_PROC_DMA_DONE))
		return IRQ_HANDLED;

	spin_lock(&fe->qlock);
	buf = fe->active;
	fe->active = NULL;
	if (buf) {
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		buf->vb.sequence = fe->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, fe->pix.sizeimage);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	ispfe_arm_next(fe);
	spin_unlock(&fe->qlock);
	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------- */
/* vb2 */

static int ispfe_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct ispfe *fe = vb2_get_drv_priv(vq);

	if (*nplanes)
		return sizes[0] < fe->pix.sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = fe->pix.sizeimage;
	return 0;
}

static int ispfe_buf_prepare(struct vb2_buffer *vb)
{
	struct ispfe *fe = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < fe->pix.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, fe->pix.sizeimage);
	return 0;
}

static void ispfe_buf_queue(struct vb2_buffer *vb)
{
	struct ispfe *fe = vb2_get_drv_priv(vb->vb2_queue);
	struct ispfe_buf *buf = container_of(to_vb2_v4l2_buffer(vb), struct ispfe_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&fe->qlock, flags);
	list_add_tail(&buf->list, &fe->queued);
	if (vb2_is_streaming(vb->vb2_queue))
		ispfe_arm_next(fe);
	spin_unlock_irqrestore(&fe->qlock, flags);
}

static void ispfe_return_buffers(struct ispfe *fe, enum vb2_buffer_state state)
{
	struct ispfe_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&fe->qlock, flags);
	if (fe->active) {
		vb2_buffer_done(&fe->active->vb.vb2_buf, state);
		fe->active = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &fe->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&fe->qlock, flags);
}

static int ispfe_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ispfe *fe = vb2_get_drv_priv(vq);
	unsigned long flags;
	int ret;

	ret = video_device_pipeline_alloc_start(&fe->vdev);
	if (ret)
		goto err_buffers;
	ret = pm_runtime_resume_and_get(fe->dev);
	if (ret < 0)
		goto err_pipeline;
	ret = clk_bulk_prepare_enable(fe->nclks, fe->clks);
	if (ret)
		goto err_pm;
	/* the link clock and the PHY's digital clock follow this; idle is too slow */
	for (int i = 0; i < fe->nclks; i++)
		clk_set_rate(fe->clks[i].clk, ISPFE_RATE_STREAMING);

	ispfe_phy_up(fe);
	writel(0xf, fe->core + FE_HW_MODE_CFG0);
	writel(1, fe->core + FE_HW_MODE_CFG2);
	usleep_range(1000, 2000);
	ispfe_link_up(fe, fe->sensor_w, fe->sensor_h);
	fe->sequence = 0;
	ispfe_loch_up(fe);
	spin_lock_irqsave(&fe->qlock, flags);
	ispfe_arm_next(fe);
	spin_unlock_irqrestore(&fe->qlock, flags);

	ret = v4l2_subdev_call(fe->sensor, video, s_stream, 1);
	if (ret)
		goto err_hw;
	return 0;

err_hw:
	ispfe_loch_down(fe);
	ispfe_phy_down(fe);
	clk_bulk_disable_unprepare(fe->nclks, fe->clks);
err_pm:
	pm_runtime_put(fe->dev);
err_pipeline:
	video_device_pipeline_stop(&fe->vdev);
err_buffers:
	ispfe_return_buffers(fe, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void ispfe_stop_streaming(struct vb2_queue *vq)
{
	struct ispfe *fe = vb2_get_drv_priv(vq);

	v4l2_subdev_call(fe->sensor, video, s_stream, 0);
	ispfe_loch_down(fe);
	ispfe_phy_down(fe);
	for (int i = 0; i < fe->nclks; i++)
		clk_set_rate(fe->clks[i].clk, ISPFE_RATE_IDLE);
	clk_bulk_disable_unprepare(fe->nclks, fe->clks);
	pm_runtime_put(fe->dev);
	video_device_pipeline_stop(&fe->vdev);
	ispfe_return_buffers(fe, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ispfe_vb2_ops = {
	.queue_setup = ispfe_queue_setup,
	.buf_prepare = ispfe_buf_prepare,
	.buf_queue = ispfe_buf_queue,
	.start_streaming = ispfe_start_streaming,
	.stop_streaming = ispfe_stop_streaming,
};

/* -------------------------------------------------------------------------- */
/* V4L2 ioctls */

static int ispfe_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "zumapro-ispfe", sizeof(cap->driver));
	strscpy(cap->card, "zumapro-ispfe", sizeof(cap->card));
	return 0;
}

static int ispfe_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(ispfe_fmts))
		return -EINVAL;
	if (f->mbus_code && f->mbus_code != ispfe_fmts[f->index].code)
		return -EINVAL;
	f->pixelformat = ispfe_fmts[f->index].fourcc;
	return 0;
}

static int ispfe_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct ispfe *fe = video_drvdata(file);

	f->fmt.pix = fe->pix;
	return 0;
}

static const struct ispfe_fmt *ispfe_fill_pix(struct ispfe *fe, struct v4l2_pix_format *pix)
{
	const struct ispfe_fmt *fmt = ispfe_find_fmt(pix->pixelformat) ?: &ispfe_fmts[0];

	/* the sensor's mode is the only size there is */
	if (fe->sensor) {
		struct v4l2_subdev_state *state = v4l2_subdev_lock_and_get_active_state(fe->sensor);
		struct v4l2_mbus_framefmt *mf = v4l2_subdev_state_get_format(state, 0);

		fe->sensor_w = mf->width;
		fe->sensor_h = mf->height;
		v4l2_subdev_unlock_state(state);
	}
	pix->width = fe->sensor_w;
	pix->height = fe->sensor_h;
	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->bytesperline = ALIGN(pix->width * fmt->bpp_num / fmt->bpp_den, 32);
	pix->sizeimage = pix->bytesperline * pix->height;
	return fmt;
}

static int ispfe_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	ispfe_fill_pix(video_drvdata(file), &f->fmt.pix);
	return 0;
}

static int ispfe_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct ispfe *fe = video_drvdata(file);

	if (vb2_is_busy(&fe->queue))
		return -EBUSY;
	fe->fmt = ispfe_fill_pix(fe, &f->fmt.pix);
	fe->pix = f->fmt.pix;
	return 0;
}

static int ispfe_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
	struct ispfe *fe = video_drvdata(file);

	if (fsize->index || !ispfe_find_fmt(fsize->pixel_format))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fe->pix.width;
	fsize->discrete.height = fe->pix.height;
	return 0;
}

static const struct v4l2_ioctl_ops ispfe_ioctl_ops = {
	.vidioc_querycap = ispfe_querycap,
	.vidioc_enum_fmt_vid_cap = ispfe_enum_fmt,
	.vidioc_g_fmt_vid_cap = ispfe_g_fmt,
	.vidioc_try_fmt_vid_cap = ispfe_try_fmt,
	.vidioc_s_fmt_vid_cap = ispfe_s_fmt,
	.vidioc_enum_framesizes = ispfe_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations ispfe_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/* -------------------------------------------------------------------------- */
/* Sensor binding */

static int ispfe_notify_bound(struct v4l2_async_notifier *nf, struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct ispfe *fe = container_of(nf, struct ispfe, notifier);
	int ret;

	fe->sensor = sd;
	ret = v4l2_create_fwnode_links_to_pad(sd, &fe->pad, MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		return ret;
	fe->fmt = ispfe_fill_pix(fe, &fe->pix);
	return 0;
}

static int ispfe_notify_complete(struct v4l2_async_notifier *nf)
{
	struct ispfe *fe = container_of(nf, struct ispfe, notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&fe->v4l2_dev);
	if (ret)
		return ret;
	dev_info(fe->dev, "%s: %s on link %u\n", video_device_node_name(&fe->vdev), fe->sensor->name, fe->cfg->link);
	return 0;
}

static const struct v4l2_async_notifier_operations ispfe_notify_ops = {
	.bound = ispfe_notify_bound,
	.complete = ispfe_notify_complete,
};

static int ispfe_parse_dt(struct ispfe *fe)
{
	struct fwnode_handle *ep;
	struct fwnode_endpoint fwep;
	int i, ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(fe->dev), NULL);
	if (!ep)
		return dev_err_probe(fe->dev, -ENODEV, "no sensor endpoint\n");
	ret = fwnode_graph_parse_endpoint(ep, &fwep);
	if (ret)
		goto out;
	ret = -ENODEV;
	for (i = 0; i < ARRAY_SIZE(ispfe_links); i++)
		if (ispfe_links[i].link == fwep.port)
			fe->cfg = &ispfe_links[i], ret = 0;
	if (ret)
		dev_err(fe->dev, "no configuration for link %u\n", fwep.port);
	fe->sensor_ep = ep;
out:
	if (ret)
		fwnode_handle_put(ep);
	return ret;
}

static int ispfe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ispfe *fe;
	struct resource *res;
	struct v4l2_async_connection *asc;
	int irq, ret;

	fe = devm_kzalloc(dev, sizeof(*fe), GFP_KERNEL);
	if (!fe)
		return -ENOMEM;
	fe->dev = dev;
	platform_set_drvdata(pdev, fe);
	mutex_init(&fe->lock);
	spin_lock_init(&fe->qlock);
	INIT_LIST_HEAD(&fe->queued);

	fe->csis = devm_platform_ioremap_resource_byname(pdev, "csis");
	if (IS_ERR(fe->csis))
		return PTR_ERR(fe->csis);
	fe->core = devm_platform_ioremap_resource_byname(pdev, "ispfe");
	if (IS_ERR(fe->core))
		return PTR_ERR(fe->core);
	/* one word inside the power domain's SYSREG window: map, don't claim */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	if (!res)
		return -EINVAL;
	fe->sysreg = devm_ioremap(dev, res->start, resource_size(res));
	if (!fe->sysreg)
		return -ENOMEM;
	fe->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(fe->pmu))
		return dev_err_probe(dev, PTR_ERR(fe->pmu), "pmu syscon\n");
	fe->nclks = devm_clk_bulk_get_all(dev, &fe->clks);
	if (fe->nclks < 0)
		return dev_err_probe(dev, fe->nclks, "clocks\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, ispfe_irq, IRQF_SHARED, dev_name(dev), fe);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;
	/* frames are 13 MB contiguous; optional pool for boards without an IOMMU path */
	of_reserved_mem_device_init(dev);
	ret = ispfe_parse_dt(fe);
	if (ret)
		return ret;
	pm_runtime_enable(dev);

	strscpy(fe->mdev.model, "zumapro-ispfe", sizeof(fe->mdev.model));
	fe->mdev.dev = dev;
	media_device_init(&fe->mdev);
	fe->v4l2_dev.mdev = &fe->mdev;
	ret = v4l2_device_register(dev, &fe->v4l2_dev);
	if (ret)
		goto err_mdev;

	fe->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&fe->vdev.entity, 1, &fe->pad);
	if (ret)
		goto err_v4l2;
	fe->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fe->queue.io_modes = VB2_MMAP | VB2_DMABUF;
	fe->queue.drv_priv = fe;
	fe->queue.buf_struct_size = sizeof(struct ispfe_buf);
	fe->queue.ops = &ispfe_vb2_ops;
	fe->queue.mem_ops = &vb2_dma_contig_memops;
	fe->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	fe->queue.lock = &fe->lock;
	fe->queue.min_queued_buffers = 1;
	fe->queue.dev = dev;
	ret = vb2_queue_init(&fe->queue);
	if (ret)
		goto err_v4l2;
	strscpy(fe->vdev.name, "zumapro-ispfe", sizeof(fe->vdev.name));
	fe->vdev.fops = &ispfe_fops;
	fe->vdev.ioctl_ops = &ispfe_ioctl_ops;
	fe->vdev.release = video_device_release_empty;
	fe->vdev.v4l2_dev = &fe->v4l2_dev;
	fe->vdev.vfl_dir = VFL_DIR_RX;
	fe->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
	fe->vdev.lock = &fe->lock;
	fe->vdev.queue = &fe->queue;
	video_set_drvdata(&fe->vdev, fe);
	fe->sensor_w = 3840;
	fe->sensor_h = 2736;
	fe->pix.pixelformat = ispfe_fmts[0].fourcc;
	fe->fmt = ispfe_fill_pix(fe, &fe->pix);

	ret = media_device_register(&fe->mdev);
	if (ret)
		goto err_v4l2;
	/* registered now so the sensor's link has a media device to land in */
	ret = video_register_device(&fe->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_mdev_reg;
	v4l2_async_nf_init(&fe->notifier, &fe->v4l2_dev);
	asc = v4l2_async_nf_add_fwnode_remote(&fe->notifier, fe->sensor_ep, struct v4l2_async_connection);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto err_nf;
	}
	fe->notifier.ops = &ispfe_notify_ops;
	ret = v4l2_async_nf_register(&fe->notifier);
	if (ret)
		goto err_nf;
	return 0;

err_nf:
	v4l2_async_nf_cleanup(&fe->notifier);
	video_unregister_device(&fe->vdev);
err_mdev_reg:
	media_device_unregister(&fe->mdev);
err_v4l2:
	v4l2_device_unregister(&fe->v4l2_dev);
err_mdev:
	media_device_cleanup(&fe->mdev);
	pm_runtime_disable(dev);
	fwnode_handle_put(fe->sensor_ep);
	of_reserved_mem_device_release(dev);
	return ret;
}

static void ispfe_remove(struct platform_device *pdev)
{
	struct ispfe *fe = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&fe->notifier);
	v4l2_async_nf_cleanup(&fe->notifier);
	video_unregister_device(&fe->vdev);
	media_device_unregister(&fe->mdev);
	v4l2_device_unregister(&fe->v4l2_dev);
	media_device_cleanup(&fe->mdev);
	pm_runtime_disable(&pdev->dev);
	fwnode_handle_put(fe->sensor_ep);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct of_device_id ispfe_of_match[] = {
	{ .compatible = "google,zumapro-ispfe" },
	{ }
};
MODULE_DEVICE_TABLE(of, ispfe_of_match);

static struct platform_driver ispfe_driver = {
	.probe = ispfe_probe,
	.remove = ispfe_remove,
	.driver = {
		.name = "zumapro-ispfe",
		.of_match_table = ispfe_of_match,
	},
};
module_platform_driver(ispfe_driver);

MODULE_DESCRIPTION("Google zumapro camera front end (CSIS + ISPFE WDMA) capture driver");
MODULE_LICENSE("GPL");
