// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor G4 Zumapro DPU bring-up scaffold.
 *
 * DECON0 exposes an opt-in, trace-shaped CRTC path for hardware-on
 * validation: framebuffers scan out through DPP0's linear RGB fetch, with
 * a color-map window as the planeless fallback.
 */

#include <linux/bitops.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modes.h>
#include <drm/drm_vblank.h>

#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "regs-zumapro-dpu.h"

struct zumapro_dpp {
	struct device *dev;
	void __iomem *dma_regs;
	void __iomem *dpp_regs;
	void __iomem *sramc_regs;
	void __iomem *hdr_comm_regs;
	u32 id;
	u32 attributes;
	u32 axi_port;
	u32 scale_down;
	u32 scale_up;
	bool video_formats;
	bool initialized;
	const u32 *pixel_formats;
	unsigned int num_pixel_formats;
	const struct zumapro_dpp_restrictions *restrictions;
};

struct zumapro_decon {
	struct device *dev;
	struct drm_device *drm_dev;
	struct exynos_drm_crtc *crtc;
	struct exynos_drm_plane plane;
	struct exynos_drm_plane_config plane_config;
	void __iomem *main_regs;
	void __iomem *win_regs;
	void __iomem *sub_regs;
	void __iomem *wincon_regs;
	void __iomem *dqe_regs;
	u32 id;
	u32 cgc_dma_id;
	u32 max_windows;
	const struct zumapro_panel_pipeline *pipeline;
	struct zumapro_dpp *dpp;
	int dpp_count;
	void *dma_priv;
	/* serializes INT_EN/INT_PEND between commits, vblank ops and the IRQ */
	spinlock_t slock;
	bool enabled;
	bool start_pending;
	bool win_dirty;
	/* gates frame_start vblank delivery; the HW interrupt stays enabled */
	bool vblank_enabled;
	/* completes the vblank when a triggered frame never starts */
	struct timer_list frame_start_timer;
};

struct zumapro_decon_desc {
	u32 id;
	const char * const *reg_names;
	unsigned int num_reg_names;
	const char * const *irq_names;
	unsigned int num_irq_names;
	bool has_cgc_dma;
};

static const char * const zumapro_dpp_reg_names[] = {
	"dma",
	"dpp",
	"scl_coef",
	"sramc",
	"hdr_comm",
	"hdr",
};

static const char * const zumapro_dpp_irq_names[] = {
	"dma",
	"dpp",
};

static const char * const zumapro_decon_reg_names[] = {
	"main",
	"win",
	"sub",
	"wincon",
	"dqe",
	"dqe-cgc",
	"cgc-dma",
};

static const char * const zumapro_decon0_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
	"dimming_start",
	"dimming_end",
	"cgc-dma",
};

static const char * const zumapro_decon1_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
	"cgc-dma",
};

static const char * const zumapro_decon2_reg_names[] = {
	"main",
	"win",
	"sub",
	"wincon",
};

static const char * const zumapro_decon2_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
};

static const struct zumapro_decon_desc zumapro_decon_descs[] = {
	{
		.id = 0,
		.reg_names = zumapro_decon_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon_reg_names),
		.irq_names = zumapro_decon0_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon0_irq_names),
		.has_cgc_dma = true,
	}, {
		.id = 1,
		.reg_names = zumapro_decon_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon_reg_names),
		.irq_names = zumapro_decon1_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon1_irq_names),
		.has_cgc_dma = true,
	}, {
		.id = 2,
		.reg_names = zumapro_decon2_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon2_reg_names),
		.irq_names = zumapro_decon2_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon2_irq_names),
	},
};

struct zumapro_dpp_format {
	u32 drm_format;
	enum zumapro_dpu_dma_format dma_format;
	enum zumapro_dpu_dpp_format dpp_format;
};

struct zumapro_dpp_size_range {
	u32 min;
	u32 max;
	u32 align;
};

struct zumapro_dpp_restrictions {
	struct zumapro_dpp_size_range src_f_w;
	struct zumapro_dpp_size_range src_f_h;
	struct zumapro_dpp_size_range src_w;
	struct zumapro_dpp_size_range src_h;
	u32 src_x_align;
	u32 src_y_align;

	struct zumapro_dpp_size_range dst_f_w;
	struct zumapro_dpp_size_range dst_f_h;
	struct zumapro_dpp_size_range dst_w;
	struct zumapro_dpp_size_range dst_h;
	u32 dst_x_align;
	u32 dst_y_align;

	struct zumapro_dpp_size_range blk_w;
	struct zumapro_dpp_size_range blk_h;
	u32 blk_x_align;
	u32 blk_y_align;

	u32 src_h_rot_max;
};

struct zumapro_panel_mode {
	const char *name;
	u32 clock_khz;
	u16 hdisplay;
	u16 hsync_start;
	u16 hsync_end;
	u16 htotal;
	u16 vdisplay;
	u16 vsync_start;
	u16 vsync_end;
	u16 vtotal;
	u16 width_mm;
	u16 height_mm;
	u16 vblank_usec;
	u16 te_usec;
	u8 refresh_hz;
	bool preferred;
	bool lp_mode;
};

struct zumapro_panel_pipeline {
	const struct zumapro_panel_mode *modes;
	unsigned int num_modes;
	const struct drm_dsc_config *dsc;
	u32 data_path;
	u32 out_type;
	enum zumapro_decon_fifo dsimif_fifo;
	u8 dsimif;
	u8 dsc_count;
	u8 data_lanes;
	u16 default_hs_clk_mbps;
	u16 alternate_hs_clk_mbps;
	u16 esc_clk_mhz;
	u32 pmsk[4];
	bool non_continuous_clock;
};

static const u32 zumapro_dpp_graphics_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
};

static const u32 zumapro_dpp_video_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV61,
	DRM_FORMAT_P010,
	DRM_FORMAT_YUV420_8BIT,
	DRM_FORMAT_YUV420_10BIT,
};

/*
 * The hardware names the 8888 and 565 formats by component order in
 * memory while DRM fourccs describe a little-endian packed word, so
 * those map to the byte-reversed name.  Panel-confirmed both ways: a
 * DRM XRGB8888 buffer scanned out as ZUMAPRO_DMA_FORMAT_XRGB8888 shows
 * (R,G,B) <- bytes (1,2,3) instead of (2,1,0), and both 565 fourccs
 * render the test pattern correctly through the reversed names.
 *
 * The 2101010 pair does not follow that rule and its two hardware
 * labels are additionally crossed: a per-fourcc-encoded test pattern
 * scanned out as ZUMAPRO_DMA_FORMAT_ARGB2101010 or _ABGR2101010 comes
 * out with red and blue exactly swapped (smooth ramp, no bit garble),
 * so value 18 decodes DRM ARGB2101010 and value 19 DRM ABGR2101010.
 *
 * The A-at-LSB variants (RGBA/BGRA1010102, values 16 and 17) do not
 * decode as any A-at-LSB layout on this hardware - both probe as the
 * same wrapped-sawtooth garbage rather than a channel swap - and the
 * FP16 values decode the component order correctly but wrap mid-tones
 * into sawtooths.  Neither is exposed until their decode is understood.
 */
static const struct zumapro_dpp_format zumapro_dpp_formats[] = {
	{ DRM_FORMAT_ARGB8888, ZUMAPRO_DMA_FORMAT_BGRA8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_ABGR8888, ZUMAPRO_DMA_FORMAT_RGBA8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGBA8888, ZUMAPRO_DMA_FORMAT_ABGR8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGRA8888, ZUMAPRO_DMA_FORMAT_ARGB8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_XRGB8888, ZUMAPRO_DMA_FORMAT_BGRX8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_XBGR8888, ZUMAPRO_DMA_FORMAT_RGBX8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGBX8888, ZUMAPRO_DMA_FORMAT_XBGR8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGRX8888, ZUMAPRO_DMA_FORMAT_XRGB8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGB565, ZUMAPRO_DMA_FORMAT_BGR565,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGR565, ZUMAPRO_DMA_FORMAT_RGB565,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_ARGB2101010, ZUMAPRO_DMA_FORMAT_ABGR2101010,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_ABGR2101010, ZUMAPRO_DMA_FORMAT_ARGB2101010,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_NV12, ZUMAPRO_DMA_FORMAT_NV12,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_NV21, ZUMAPRO_DMA_FORMAT_NV21,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_NV16, ZUMAPRO_DMA_FORMAT_NV16,
	  ZUMAPRO_DPP_FORMAT_YUV422_8P },
	{ DRM_FORMAT_NV61, ZUMAPRO_DMA_FORMAT_NV61,
	  ZUMAPRO_DPP_FORMAT_YUV422_8P },
	{ DRM_FORMAT_P010, ZUMAPRO_DMA_FORMAT_YUV420_P010,
	  ZUMAPRO_DPP_FORMAT_YUV420_P010 },
	{ DRM_FORMAT_YUV420_8BIT, ZUMAPRO_DMA_FORMAT_NV12,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_YUV420_10BIT, ZUMAPRO_DMA_FORMAT_YUV420_P010,
	  ZUMAPRO_DPP_FORMAT_YUV420_P010 },
};

static const struct zumapro_dpp_restrictions zumapro_dpp_restrictions = {
	.src_f_w = { 16, 65534, 1 },
	.src_f_h = { 16, 8190, 1 },
	.src_w = { 16, 4096, 1 },
	.src_h = { 16, 4096, 1 },
	.src_x_align = 1,
	.src_y_align = 1,

	.dst_f_w = { 16, 8190, 1 },
	.dst_f_h = { 16, 8190, 1 },
	.dst_w = { 16, 4096, 1 },
	.dst_h = { 16, 4096, 1 },
	.dst_x_align = 1,
	.dst_y_align = 1,

	.blk_w = { 4, 4096, 1 },
	.blk_h = { 4, 4096, 1 },
	.blk_x_align = 1,
	.blk_y_align = 1,

	.src_h_rot_max = 2160,
};

static const struct zumapro_dpp_format *
zumapro_dpp_find_format(u32 drm_format)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_dpp_formats); i++)
		if (zumapro_dpp_formats[i].drm_format == drm_format)
			return &zumapro_dpp_formats[i];

	return NULL;
}

static int zumapro_dpp_validate_formats(struct device *dev,
					const u32 *formats,
					unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (!zumapro_dpp_find_format(formats[i]))
			return dev_err_probe(dev, -EINVAL,
					     "missing hardware mapping for DRM format %#x\n",
					     formats[i]);
	}

	return 0;
}

static int zumapro_dpp_select_formats(struct zumapro_dpp *dpp)
{
	if (dpp->video_formats) {
		dpp->pixel_formats = zumapro_dpp_video_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(zumapro_dpp_video_formats);
	} else {
		dpp->pixel_formats = zumapro_dpp_graphics_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(zumapro_dpp_graphics_formats);
	}

	return zumapro_dpp_validate_formats(dpp->dev, dpp->pixel_formats,
					    dpp->num_pixel_formats);
}

static void zumapro_dpu_update_bits(void __iomem *regs, u32 offset, u32 mask,
				    u32 val)
{
	u32 tmp;

	tmp = readl(regs + offset);
	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, regs + offset);
}

/* Downstream in-flight-count and outstanding-MO budgets for every IDMA. */
#define ZUMAPRO_DPP_IC_MAX			0x40
#define ZUMAPRO_DPP_ASSIGNED_MO			0x40

/*
 * Downstream derives this recovery budget from the DPU clock; 0x412f8 is the
 * value it computed for the traced first frame, and the deadlock timeout is
 * defined as 51 of those units (one frame plus DVFS margin).
 */
#define ZUMAPRO_DPP_RCV_NUM			0x412f8

static int zumapro_dpp_reset(struct zumapro_dpp *dpp)
{
	u32 val;
	int ret;

	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_ENABLE,
				ZUMAPRO_RDMA_ENABLE_SRESET,
				ZUMAPRO_RDMA_ENABLE_SRESET);
	ret = readl_poll_timeout(dpp->dma_regs + ZUMAPRO_RDMA_ENABLE, val,
				 !(val & ZUMAPRO_RDMA_ENABLE_SRESET), 10, 2000);
	if (ret)
		dev_warn(dpp->dev, "DPP%u IDMA reset did not complete: %d\n",
			 dpp->id, ret);

	return ret;
}

static void zumapro_dpp_init(struct zumapro_dpp *dpp)
{
	/* Downstream only resets the IDMA when it stopped on a deadlock. */
	if (readl(dpp->dma_regs + ZUMAPRO_RDMA_IRQ) & ZUMAPRO_RDMA_IRQ_DEADLOCK)
		zumapro_dpp_reset(dpp);

	writel(0x44444444, dpp->dma_regs + ZUMAPRO_RDMA_QOS_LUT_LOW);
	writel(0x44444444, dpp->dma_regs + ZUMAPRO_RDMA_QOS_LUT_HIGH);
	writel(0, dpp->dma_regs + ZUMAPRO_RDMA_DYNAMIC_GATING_EN);
	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_IN_CTRL_0,
				ZUMAPRO_RDMA_ALPHA_MASK |
				ZUMAPRO_RDMA_IC_MAX_MASK,
				ZUMAPRO_RDMA_ALPHA(0xff) |
				ZUMAPRO_RDMA_IC_MAX(ZUMAPRO_DPP_IC_MAX));
	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_ENABLE,
				ZUMAPRO_RDMA_ENABLE_ASSIGNED_MO_MASK,
				ZUMAPRO_RDMA_ENABLE_ASSIGNED_MO(
					ZUMAPRO_DPP_ASSIGNED_MO));
}

static void zumapro_dpp_update(struct zumapro_dpp *dpp,
			       const struct exynos_drm_plane_state *state)
{
	const struct drm_framebuffer *fb = state->base.fb;
	const struct zumapro_dpp_format *format =
		zumapro_dpp_find_format(fb->format->format);
	u32 src_f_w = fb->pitches[0] / fb->format->cpp[0];
	u32 io_con;

	/* SRAM resources: linear RGB, no scaling/rotation/compression. */
	writel(ZUMAPRO_SRAMC_DST_BOTTOM(state->crtc.y + state->crtc.h - 1) |
	       ZUMAPRO_SRAMC_DST_TOP(state->crtc.y),
	       dpp->sramc_regs + ZUMAPRO_SRAMC_COM_DST_POSITION);
	writel(ZUMAPRO_SRAMC_FORMAT(ZUMAPRO_SRAMC_FMT_RGB32BIT),
	       dpp->sramc_regs + ZUMAPRO_SRAMC_COM_MODE);

	zumapro_dpu_update_bits(dpp->dpp_regs, ZUMAPRO_DPP_COM_SCL_CTRL,
				ZUMAPRO_DPP_SCL_ENABLE, 0);

	/*
	 * IDMA source geometry.  There is no stride register for linear
	 * formats; SRC_WIDTH carries the full buffer width in pixels.
	 */
	writel(ZUMAPRO_RDMA_SRC_OFFSET_Y(state->src.y) |
	       ZUMAPRO_RDMA_SRC_OFFSET_X(state->src.x),
	       dpp->dma_regs + ZUMAPRO_RDMA_SRC_OFFSET);
	writel(src_f_w, dpp->dma_regs + ZUMAPRO_RDMA_SRC_WIDTH);
	writel(fb->height, dpp->dma_regs + ZUMAPRO_RDMA_SRC_HEIGHT);
	writel(ZUMAPRO_RDMA_SIZE_HEIGHT(state->src.h) |
	       ZUMAPRO_RDMA_SIZE_WIDTH(state->src.w),
	       dpp->dma_regs + ZUMAPRO_RDMA_IMG_SIZE);

	writel(ZUMAPRO_DPP_IMG_SIZE_HEIGHT(state->src.h) |
	       ZUMAPRO_DPP_IMG_SIZE_WIDTH(state->src.w),
	       dpp->dpp_regs + ZUMAPRO_DPP_COM_IMG_SIZE);

	/* Scaler in 1:1 bypass: zero initial phase, output size = input. */
	writel(0, dpp->dpp_regs + ZUMAPRO_DPP_COM_SCL_HPOSITION);
	writel(0, dpp->dpp_regs + ZUMAPRO_DPP_COM_SCL_VPOSITION);
	writel(ZUMAPRO_DPP_IMG_SIZE_HEIGHT(state->crtc.h) |
	       ZUMAPRO_DPP_IMG_SIZE_WIDTH(state->crtc.w),
	       dpp->dpp_regs + ZUMAPRO_DPP_COM_SCL_SCALED_IMG_SIZE);

	writel(ZUMAPRO_COMM_SIZE_VSIZE(state->crtc.h) |
	       ZUMAPRO_COMM_SIZE_HSIZE(state->crtc.w),
	       dpp->hdr_comm_regs + ZUMAPRO_LSI_COMM_SIZE);

	writel(lower_32_bits(exynos_drm_fb_dma_addr(state->base.fb, 0)),
	       dpp->dma_regs + ZUMAPRO_RDMA_BASEADDR_P0);
	writel(0, dpp->dma_regs + ZUMAPRO_RDMA_BASEADDR_P1);

	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_IN_CTRL_0,
				ZUMAPRO_RDMA_IMG_FORMAT_MASK,
				ZUMAPRO_RDMA_IMG_FORMAT(format->dma_format));

	io_con = ZUMAPRO_DPP_IMG_FORMAT(format->dpp_format);
	if (fb->format->has_alpha)
		io_con |= ZUMAPRO_DPP_ALPHA_SEL_PER_PIXEL;
	zumapro_dpu_update_bits(dpp->dpp_regs, ZUMAPRO_DPP_COM_IO_CON,
				ZUMAPRO_DPP_ALPHA_SEL_PER_PIXEL |
				ZUMAPRO_DPP_BPC_MODE_10BIT |
				ZUMAPRO_DPP_IMG_FORMAT_MASK, io_con);
	zumapro_dpu_update_bits(dpp->hdr_comm_regs, ZUMAPRO_LSI_COMM_IO_CON,
				ZUMAPRO_COMM_BPC_MODE_10BIT |
				ZUMAPRO_COMM_IMG_FORMAT_MASK,
				ZUMAPRO_COMM_IMG_FORMAT(format->dpp_format));

	/*
	 * No rotation, no block crop, no compression.  These must be written
	 * explicitly: the fetch inherits whatever the bootloader left in
	 * IN_CTRL_0, and leftover rot/block/compression bits spatially remap
	 * the fetched image.
	 */
	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_IN_CTRL_0,
				ZUMAPRO_RDMA_ROT_MASK |
				ZUMAPRO_RDMA_AFBC_EN | ZUMAPRO_RDMA_SBWC_EN |
				ZUMAPRO_RDMA_SAJC_EN | ZUMAPRO_RDMA_BLOCK_EN,
				0);
	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_RECOVERY_CTRL,
				ZUMAPRO_RDMA_RECOVERY_EN, 0);
	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_RECOVERY_CTRL,
				ZUMAPRO_RDMA_RECOVERY_NUM_MASK,
				ZUMAPRO_RDMA_RECOVERY_NUM(ZUMAPRO_DPP_RCV_NUM));

	zumapro_dpu_update_bits(dpp->dma_regs, ZUMAPRO_RDMA_DEADLOCK_CTRL,
				ZUMAPRO_RDMA_DEADLOCK_NUM_MASK |
				ZUMAPRO_RDMA_DEADLOCK_NUM_EN,
				ZUMAPRO_RDMA_DEADLOCK_NUM(
					ZUMAPRO_DPP_RCV_NUM * 51) |
				ZUMAPRO_RDMA_DEADLOCK_NUM_EN);
}

#define ZUMAPRO_DSC_6BIT_SIGNED(_v)	((_v) & 0x3f)

static const struct drm_dsc_config zumapro_tg4c_dsc = {
	.dsc_version_major = 1,
	.dsc_version_minor = 2,
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_count = 2,
	.slice_width = 540,
	.slice_height = 24,
	.simple_422 = false,
	.pic_width = 1080,
	.pic_height = 2424,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126,
	},
	.rc_range_params = {
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(2) },
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 5, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 6, ZUMAPRO_DSC_6BIT_SIGNED(-2) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-4) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-6) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 8, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 9, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 10, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 10, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 11, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 5, 11, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 9, 12, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 12, 13, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 7,
	.scale_increment_interval = 588,
	.nfl_bpg_offset = 1069,
	.slice_bpg_offset = 1085,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 540,
	.native_422 = false,
	.native_420 = false,
	.second_line_bpg_offset = 0,
	.nsl_bpg_offset = 0,
	.second_line_offset_adj = 0,
};

static const struct zumapro_panel_mode zumapro_tg4c_modes[] = {
	{
		.name = "1080x2424@60:60",
		.clock_khz = 167922,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.te_usec = 8450,
		.refresh_hz = 60,
		.preferred = true,
	}, {
		.name = "1080x2424@120:120",
		.clock_khz = 335844,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.te_usec = 276,
		.refresh_hz = 120,
	}, {
		.name = "1080x2424@30:30",
		.clock_khz = 83961,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.refresh_hz = 30,
		.lp_mode = true,
	},
};

static const struct zumapro_panel_pipeline zumapro_decon0_tg4c_pipeline = {
	.modes = zumapro_tg4c_modes,
	.num_modes = ARRAY_SIZE(zumapro_tg4c_modes),
	.dsc = &zumapro_tg4c_dsc,
	.data_path = ZUMAPRO_DECON_ENHANCE_DQE_ON |
		     ZUMAPRO_DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF0,
	.out_type = ZUMAPRO_DECON_OUT_DSI0,
	.dsimif_fifo = ZUMAPRO_DECON0_OFIFO0,
	.dsimif = 0,
	.dsc_count = 2,
	.data_lanes = 4,
	.default_hs_clk_mbps = 1102,
	.alternate_hs_clk_mbps = 1000,
	.esc_clk_mhz = 20,
	.pmsk = { 0x02, 0xb3, 0x02, 0x5cab },
	.non_continuous_clock = true,
};

/*
 * Komodo (Pixel 9 Pro XL, google,gs-km4): 1344x2992 dual-DSC command-mode
 * panel.  Timings and DSC config mirror panel-google-komodo.c, which is the
 * authority -- mode_valid() below matches on them exactly, so they must stay
 * in step with the panel's mode table.
 */
static const struct drm_dsc_config zumapro_km4_dsc = {
	.dsc_version_major = 1,
	.dsc_version_minor = 2,
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_count = 2,
	.slice_width = 672,
	.slice_height = 34,
	.simple_422 = false,
	.pic_width = 1344,
	.pic_height = 2992,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.initial_dec_delay = 592,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126,
	},
	.rc_range_params = {
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(2) },
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 5, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 6, ZUMAPRO_DSC_6BIT_SIGNED(-2) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-4) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-6) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 8, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 9, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 10, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 11, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 12, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 5, 13, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 7, 13, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 13, 15, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 9,
	.scale_increment_interval = 932,
	.nfl_bpg_offset = 745,
	.slice_bpg_offset = 616,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 672,
	.native_422 = false,
	.native_420 = false,
	.second_line_bpg_offset = 0,
	.nsl_bpg_offset = 0,
	.second_line_offset_adj = 0,
};

static const struct zumapro_panel_mode zumapro_km4_modes[] = {
	{
		/* htotal 1490 = 1344 + 80 + 24 + 42, vtotal 3030 */
		.name = "1344x2992@120:120",
		.clock_khz = 541764,
		.hdisplay = 1344,
		.hsync_start = 1424,
		.hsync_end = 1448,
		.htotal = 1490,
		.vdisplay = 2992,
		.vsync_start = 3004,
		.vsync_end = 3008,
		.vtotal = 3030,
		.width_mm = 70,
		.height_mm = 156,
		.vblank_usec = 120,
		.te_usec = 276,
		.refresh_hz = 120,
		.preferred = true,
	}, {
		.name = "1344x2992@60:60",
		.clock_khz = 270882,
		.hdisplay = 1344,
		.hsync_start = 1424,
		.hsync_end = 1448,
		.htotal = 1490,
		.vdisplay = 2992,
		.vsync_start = 3004,
		.vsync_end = 3008,
		.vtotal = 3030,
		.width_mm = 70,
		.height_mm = 156,
		.vblank_usec = 120,
		.te_usec = 8450,
		.refresh_hz = 60,
	}, {
		/* htotal 1150 = 1008 + 80 + 24 + 38, vtotal 2280 */
		.name = "1008x2244@120:120",
		.clock_khz = 314640,
		.hdisplay = 1008,
		.hsync_start = 1088,
		.hsync_end = 1112,
		.htotal = 1150,
		.vdisplay = 2244,
		.vsync_start = 2256,
		.vsync_end = 2260,
		.vtotal = 2280,
		.width_mm = 70,
		.height_mm = 156,
		.vblank_usec = 120,
		.te_usec = 276,
		.refresh_hz = 120,
	}, {
		.name = "1008x2244@60:60",
		.clock_khz = 157320,
		.hdisplay = 1008,
		.hsync_start = 1088,
		.hsync_end = 1112,
		.htotal = 1150,
		.vdisplay = 2244,
		.vsync_start = 2256,
		.vsync_end = 2260,
		.vtotal = 2280,
		.width_mm = 70,
		.height_mm = 156,
		.vblank_usec = 120,
		.te_usec = 8450,
		.refresh_hz = 60,
	},
};

static const struct zumapro_panel_pipeline zumapro_decon0_km4_pipeline = {
	.modes = zumapro_km4_modes,
	.num_modes = ARRAY_SIZE(zumapro_km4_modes),
	.dsc = &zumapro_km4_dsc,
	/*
	 * Route the blender straight to the DSC encoders and out, with the
	 * enhancement path left off.  Turning DQE on here (as tg4c does) wedges
	 * the DECON: it goes to RUN with IDLE stuck low, the shadow update is
	 * never consumed and frame start never fires, so the panel is fed a
	 * stalled stream.  The previous komodo driver drove this panel with
	 * DATA_PATH = 0xb1, i.e. this value without ENHANCE_DQE_ON.
	 */
	.data_path = ZUMAPRO_DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF0,
	.out_type = ZUMAPRO_DECON_OUT_DSI0,
	.dsimif_fifo = ZUMAPRO_DECON0_OFIFO0,
	.dsimif = 0,
	.dsc_count = 2,
	.data_lanes = 4,
	.default_hs_clk_mbps = 1368,
	.alternate_hs_clk_mbps = 1288,
	.esc_clk_mhz = 20,
	.pmsk = { 0x02, 0xde, 0x02, 0xa800 },
	.non_continuous_clock = true,
};

static const struct zumapro_decon_desc *zumapro_decon_desc_by_id(u32 id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_decon_descs); i++)
		if (zumapro_decon_descs[i].id == id)
			return &zumapro_decon_descs[i];

	return NULL;
}

struct zumapro_decon_pps_word {
	u32 offset;
	u32 value;
};

/*
 * Final PPS register values from the downstream TG4C wake trace.  Downstream
 * reaches some of these by RMW, but the milestone-1 comparison is final state.
 */
static const struct zumapro_decon_pps_word zumapro_tg4c_dsc_pps[] = {
	{ 0x40, 0x12000089 },
	{ 0x44, 0x30800978 },
	{ 0x48, 0x04380018 },
	{ 0x4c, 0x021c021c },
	{ 0x50, 0x0200020e },
	{ 0x54, 0x0020024c },
	{ 0x58, 0x0007000c },
	{ 0x5c, 0x042d043d },
	{ 0x60, 0x180010f0 },
	{ 0x64, 0x030c2000 },
	{ 0x68, 0x060b0b33 },
	{ 0x6c, 0x0e1c2a38 },
	{ 0x70, 0x46546269 },
	{ 0x78, 0x7d7e0102 },
	{ 0x7c, 0x01000940 },
	{ 0x80, 0x09be19fc },
	{ 0x84, 0x19fa19f8 },
	{ 0x88, 0x1a381a78 },
	{ 0x8c, 0x1ab62ab6 },
	{ 0x90, 0x2af42af4 },
	{ 0x94, 0x4b346374 },
};

static void zumapro_decon_write_dsc_pps(struct zumapro_decon *decon, u8 dsc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_tg4c_dsc_pps); i++)
		writel(zumapro_tg4c_dsc_pps[i].value,
		       decon->sub_regs + ZUMAPRO_DSC_OFFSET(dsc) +
		       zumapro_tg4c_dsc_pps[i].offset);
}

static u32 zumapro_dqe_size(u32 width, u32 height)
{
	return ZUMAPRO_DQE_IMG_VSIZE(height) |
	       ZUMAPRO_DQE_IMG_HSIZE(width);
}

static u32 zumapro_dqe_atc_ibsi(u32 width, u32 height, u32 div)
{
	u32 hori_grid = DIV_ROUND_UP(width, 8);
	u32 vert_grid = DIV_ROUND_UP(height, 16);
	u32 ibsi_x = (1 << 16) / (hori_grid * div);
	u32 ibsi_y = (1 << 16) / (vert_grid * div);

	return ZUMAPRO_DQE_ATC_IBSI_Y(ibsi_y) |
	       ZUMAPRO_DQE_ATC_IBSI_X(ibsi_x);
}

static u32 zumapro_dqe_atc_cdf(u32 width, u32 height)
{
	u32 pixels = width * height;
	u32 tmp;
	u32 shift;
	u32 denom;
	u32 div;

	if (!pixels)
		return 0;

	tmp = (481 * pixels) / (255 * (1 << 14));
	if (!tmp)
		return 0;

	if (tmp & (tmp - 1))
		shift = fls(tmp);
	else
		shift = fls(tmp) - 1;

	denom = pixels >> shift;
	if (!denom)
		return 0;

	div = ((1 << 14) / denom) * 255;

	return ZUMAPRO_DQE_ATC_CDF_SHIFT(shift) |
	       ZUMAPRO_DQE_ATC_CDF_DIV_VAL(div);
}

static void zumapro_decon_program_dqe(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	u32 width = mode->hdisplay;
	u32 height = mode->vdisplay;
	u32 size = zumapro_dqe_size(width, height);

	writel(size, decon->dqe_regs + ZUMAPRO_DQE_TOP_IMG_SIZE);
	writel(size, decon->dqe_regs + ZUMAPRO_DQE_TOP_FRM_SIZE);
	writel(ZUMAPRO_DQE_FULL_PXL_NUM(width * height),
	       decon->dqe_regs + ZUMAPRO_DQE_TOP_FRM_PXL_NUM);
	writel(zumapro_dqe_atc_ibsi(width, height, 4),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_PARTIAL_IBSI_P1);
	writel(zumapro_dqe_atc_ibsi(width, height, 2),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_PARTIAL_IBSI_P2);
	writel(zumapro_dqe_atc_cdf(width, height),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_CDF_DIV);
	writel(0, decon->dqe_regs + ZUMAPRO_DQE_ATC_CONTROL);
	writel(0, decon->dqe_regs + ZUMAPRO_DQE_DISP_DITHER_V4);
}

static void zumapro_decon_program_outfifo(struct zumapro_decon *decon)
{
	/* Downstream DECON0 primary OUTFIFO owns SRAM banks 0..10. */
	writel(0x11111111, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(0));
	writel(0x00000111, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(1));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(2));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(3));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(0));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(1));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(2));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(3));

	writel(0, decon->main_regs + ZUMAPRO_DECON_OF_PIXEL_ORDER);
	writel(0x1, decon->main_regs + ZUMAPRO_DECON_OF_URGENT_EN);
	writel(0x08000400, decon->main_regs + ZUMAPRO_DECON_OF_RD_URGENT_0);
	writel(0x10, decon->main_regs + ZUMAPRO_DECON_OF_RD_URGENT_1);
	writel(0, decon->main_regs + ZUMAPRO_DECON_OF_WR_URGENT_0);
	writel(0x1, decon->main_regs + ZUMAPRO_DECON_OF_DTA_CONTROL);
	writel(0x32000600, decon->main_regs + ZUMAPRO_DECON_OF_DTA_THRESHOLD);
}

static void zumapro_decon_program_dsc(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;
	const struct drm_dsc_config *dsc = pipeline->dsc;
	u32 outfifo_width;
	u8 i;

	for (i = 0; i < pipeline->dsc_count; i++) {
		writel(0x222, decon->sub_regs + ZUMAPRO_DSC_CONTROL1(i));
		writel(0x30b4, decon->sub_regs + ZUMAPRO_DSC_CONTROL3(i));
		zumapro_decon_write_dsc_pps(decon, i);
	}

	outfifo_width = DIV_ROUND_UP(dsc->slice_width, 3);
	writel(ZUMAPRO_DECON_OF_HEIGHT(mode->vdisplay) |
	       ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_0);
	writel(ZUMAPRO_DECON_OF_TH_1H,
	       decon->main_regs + ZUMAPRO_DECON_OF_TH_TYPE);
	writel(ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_1);
	writel(ZUMAPRO_DECON_OF_HEIGHT(dsc->slice_height) |
	       ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_2);
}

static void zumapro_decon_program_lcd(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;

	writel(ZUMAPRO_DECON_OF_HEIGHT(mode->vdisplay) |
	       ZUMAPRO_DECON_OF_WIDTH(mode->hdisplay),
	       decon->main_regs + ZUMAPRO_DECON_BLD_BG_IMG_SIZE_PRI);

	zumapro_decon_program_outfifo(decon);

	writel(ZUMAPRO_DSIMIF_SEL_DSIM(pipeline->dsimif_fifo),
	       decon->sub_regs + ZUMAPRO_DSIMIF_SEL(pipeline->dsimif));
	writel(pipeline->data_path,
	       decon->main_regs + ZUMAPRO_DECON_DATA_PATH_CON_0);

	zumapro_decon_program_dqe(decon, mode);
	zumapro_decon_program_dsc(decon, mode);

	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_CMD_MODE,
				  ZUMAPRO_DECON_GLOBAL_CON_CMD_MODE);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F, 0);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_SEL_MASK |
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_SEL_DDI0 |
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK);
}

static void zumapro_decon_program_colormap_window(struct zumapro_decon *decon,
						 const struct drm_display_mode *mode)
{
	u32 blend_func;
	u32 blend_coeff;
	u32 end_pos;

	blend_func = ZUMAPRO_DECON_WIN_FUNC(ZUMAPRO_DECON_WIN_FUNC_USER_DEFINED) |
		     ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_SEL(
				ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_AF);
	blend_coeff =
		ZUMAPRO_DECON_WIN_FG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO) |
		ZUMAPRO_DECON_WIN_FG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO);
	end_pos = ZUMAPRO_DECON_WIN_POS_Y(mode->vdisplay - 1) |
		  ZUMAPRO_DECON_WIN_POS_X(mode->hdisplay - 1);

	writel(blend_func, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_0(0));
	writel(blend_coeff, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_1(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_START_POSITION(0));
	writel(end_pos, decon->win_regs + ZUMAPRO_DECON_WIN_END_POSITION(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_START_TIME_CON(0));
	writel(ZUMAPRO_DECON_WIN_MAPCOLOR_EN,
	       decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_COLORMAP_0(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_COLORMAP_1(0));
	writel(ZUMAPRO_DECON_WIN_MAPCOLOR_EN | ZUMAPRO_DECON_WIN_EN,
	       decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(0));
}

/*
 * Downstream _decon_reinit_locked(): adopt a DECON handed over live from
 * the bootloader instead of stopping and resetting it.  Quiesce the
 * interrupt state, drop all window enables (shadow-protected, so they
 * latch together with the first commit's programming) and mask the
 * trigger so no frame kicks reach the adopted pipeline until that commit.
 */
static void zumapro_decon_handover(struct zumapro_decon *decon)
{
	unsigned long flags;
	u32 win_count;
	u32 win;

	spin_lock_irqsave(&decon->slock, flags);
	writel(0, decon->main_regs + ZUMAPRO_DECON_INT_EN);
	writel(ZUMAPRO_DECON_INT_FRAME_START | ZUMAPRO_DECON_INT_FRAME_DONE |
	       ZUMAPRO_DECON_INT_EXTRA,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	writel(ZUMAPRO_DECON_INT_RESOURCE_CONFLICT | ZUMAPRO_DECON_INT_TIMEOUT,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);
	spin_unlock_irqrestore(&decon->slock, flags);

	win_count = min_t(u32, decon->max_windows, ZUMAPRO_DPU_MAX_WINDOWS);
	for (win = 0; win < win_count; win++)
		writel(0, decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(win));

	spin_lock_irqsave(&decon->slock, flags);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				ZUMAPRO_DECON_HW_TRIG_EN |
				ZUMAPRO_DECON_HW_TRIG_MASK,
				ZUMAPRO_DECON_HW_TRIG_MASK);
	spin_unlock_irqrestore(&decon->slock, flags);
}

static int zumapro_decon_wait_run(struct zumapro_decon *decon)
{
	u32 val;

	return readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				  val, val & ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS,
				  10, 2000);
}

static void zumapro_decon_start(struct zumapro_decon *decon)
{
	unsigned long flags;
	int ret;

	/*
	 * Enable the interrupt master, frame_start, frame_done and the extra
	 * source (resource conflict/timeout diagnostics).  frame_start must be
	 * unmasked here, before the trigger is unmasked below: the very first
	 * frame after enable is driven by a panel TE the moment the trigger
	 * goes live, and if its frame_start interrupt is not yet enabled the
	 * vblank is lost (then the commit's flip event never gets delivered).
	 * vblank delivery is gated in software (decon->vblank_enabled) so the
	 * HW interrupt can stay enabled for the whole enabled-DECON lifetime
	 * without enable_vblank()/disable_vblank() racing the trigger or the
	 * deferred vblank-disable against an incoming frame.  Stale pending
	 * bits would fire as soon as the master bit is set, so clear them first.
	 */
	spin_lock_irqsave(&decon->slock, flags);
	writel(ZUMAPRO_DECON_INT_FRAME_START | ZUMAPRO_DECON_INT_FRAME_DONE |
	       ZUMAPRO_DECON_INT_EXTRA,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	writel(ZUMAPRO_DECON_INT_RESOURCE_CONFLICT | ZUMAPRO_DECON_INT_TIMEOUT,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);
	writel(ZUMAPRO_DECON_INT_RESOURCE_CONFLICT | ZUMAPRO_DECON_INT_TIMEOUT,
	       decon->main_regs + ZUMAPRO_DECON_INT_EN_EXTRA);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_INT_EN,
				ZUMAPRO_DECON_INT_MASTER |
				ZUMAPRO_DECON_INT_FRAME_START |
				ZUMAPRO_DECON_INT_FRAME_DONE |
				ZUMAPRO_DECON_INT_EXTRA,
				ZUMAPRO_DECON_INT_MASTER |
				ZUMAPRO_DECON_INT_FRAME_START |
				ZUMAPRO_DECON_INT_FRAME_DONE |
				ZUMAPRO_DECON_INT_EXTRA);
	spin_unlock_irqrestore(&decon->slock, flags);

	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_DQE,
				  ZUMAPRO_DECON_SHD_DQE);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_ALL_WINDOWS,
				  ZUMAPRO_DECON_SHD_ALL_WINDOWS);

	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN |
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F,
				  ZUMAPRO_DECON_GLOBAL_CON_EN |
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP);

	ret = zumapro_decon_wait_run(decon);
	if (ret)
		dev_warn(decon->dev, "DECON%u did not enter run state: %d\n",
			 decon->id, ret);

	/*
	 * Unmask the trigger for the first frame; the next commit's
	 * atomic_begin re-masks it before reprogramming.
	 */
	spin_lock_irqsave(&decon->slock, flags);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_EN);
	spin_unlock_irqrestore(&decon->slock, flags);
}

static void zumapro_decon_stop(struct zumapro_decon *decon)
{
	unsigned long flags;
	u32 val;
	u32 win_count;
	u32 win;
	int ret;

	spin_lock_irqsave(&decon->slock, flags);
	writel(0, decon->main_regs + ZUMAPRO_DECON_INT_EN);
	writel(ZUMAPRO_DECON_INT_FRAME_START | ZUMAPRO_DECON_INT_FRAME_DONE |
	       ZUMAPRO_DECON_INT_EXTRA,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	writel(ZUMAPRO_DECON_INT_RESOURCE_CONFLICT | ZUMAPRO_DECON_INT_TIMEOUT,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);
	spin_unlock_irqrestore(&decon->slock, flags);

	win_count = min_t(u32, decon->max_windows, ZUMAPRO_DPU_MAX_WINDOWS);
	for (win = 0; win < win_count; win++)
		writel(0, decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(win));

	spin_lock_irqsave(&decon->slock, flags);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_MASK);
	spin_unlock_irqrestore(&decon->slock, flags);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F, 0);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP);

	/* Let a frame in flight drain before the reset, like downstream. */
	ret = readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				 val,
				 !(val & ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS),
				 10, 50000);
	if (ret) {
		/*
		 * The per-frame stop latches on a trigger; with the trigger
		 * masked (always the case when stopping a bootloader handoff,
		 * where it has been masked since early boot) it never does.
		 * Fall back to downstream's instant stop, which clears the
		 * non-shadowed enable directly.
		 */
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_GLOBAL_CON,
					ZUMAPRO_DECON_GLOBAL_CON_EN |
					ZUMAPRO_DECON_GLOBAL_CON_EN_F, 0);
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_SHD_REG_UP_REQ,
					ZUMAPRO_DECON_SHD_GLOBAL |
					ZUMAPRO_DECON_SHD_CMP,
					ZUMAPRO_DECON_SHD_GLOBAL |
					ZUMAPRO_DECON_SHD_CMP);

		ret = readl_poll_timeout(decon->main_regs +
					 ZUMAPRO_DECON_GLOBAL_CON,
					 val,
					 !(val & ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS),
					 10, 50000);
		/*
		 * Expected when the link is already down (the bridge chain
		 * disables before the CRTC, so a frame caught in flight has
		 * nowhere to drain); the soft reset below recovers and warns
		 * if it does not complete.
		 */
		if (ret)
			dev_dbg(decon->dev,
				"DECON%u still running after stop, relying on reset\n",
				decon->id);
	}

	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_SRESET,
				  ZUMAPRO_DECON_GLOBAL_CON_SRESET);

	ret = readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				 val, !(val & ZUMAPRO_DECON_GLOBAL_CON_SRESET),
				 10, 2000);
	if (ret)
		dev_warn(decon->dev, "DECON%u reset did not complete: %d\n",
			 decon->id, ret);
}

static enum drm_mode_status
zumapro_decon_mode_valid(struct exynos_drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	struct zumapro_decon *decon = crtc->ctx;
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;
	unsigned int i;

	for (i = 0; i < pipeline->num_modes; i++) {
		const struct zumapro_panel_mode *panel_mode = &pipeline->modes[i];

		if (mode->clock == panel_mode->clock_khz &&
		    mode->hdisplay == panel_mode->hdisplay &&
		    mode->hsync_start == panel_mode->hsync_start &&
		    mode->hsync_end == panel_mode->hsync_end &&
		    mode->htotal == panel_mode->htotal &&
		    mode->vdisplay == panel_mode->vdisplay &&
		    mode->vsync_start == panel_mode->vsync_start &&
		    mode->vsync_end == panel_mode->vsync_end &&
		    mode->vtotal == panel_mode->vtotal)
			return MODE_OK;
	}

	return MODE_BAD;
}

static void zumapro_decon_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	const struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;
	int ret;

	if (decon->enabled)
		return;

	if (!mode->hdisplay || !mode->vdisplay)
		return;

	ret = pm_runtime_resume_and_get(decon->dev);
	if (ret < 0) {
		dev_err(decon->dev, "failed to resume DECON%u: %d\n",
			decon->id, ret);
		return;
	}

	/*
	 * Downstream decon_reg_init() disables the DECON-internal automatic
	 * clock gating and the dynamic QACTIVE Q-channel handshake before
	 * touching anything else ("clock gating is disabled during initial
	 * bringup").  Left in the handed-off dynamic mode, the hardware can
	 * signal idle to the QCH mid-reconfiguration and the next register
	 * access into the gated domain stalls the interconnect.
	 */
	zumapro_dpu_update_bits(decon->main_regs,
				ZUMAPRO_DECON_CLOCK_CON(decon->id),
				ZUMAPRO_DECON_CLOCK_CON_AUTO_CG_MASK |
				ZUMAPRO_DECON_CLOCK_CON_QACTIVE |
				ZUMAPRO_DECON_CLOCK_CON_QACTIVE_PLL, 0);

	/*
	 * Downstream never resets a pipeline handed over live from the
	 * bootloader (DECON_STATE_HANDOVER): it drops the window enables
	 * and reprograms the running block.  Stop and soft-reset only a
	 * block that is not running (warm boot without display init, or
	 * re-enable after our own disable).
	 */
	if (readl(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON) &
	    ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS)
		zumapro_decon_handover(decon);
	else
		zumapro_decon_stop(decon);
	if (decon->dpp)
		decon->dpp->initialized = false;

	zumapro_decon_program_lcd(decon, mode);
	zumapro_decon_program_colormap_window(decon, mode);

	/*
	 * Scanout must not start while the DSIM frame geometry is still
	 * unprogrammed and the panel uninitialized; both happen in the
	 * bridge-enable phase, after this hook.  Defer the start/unmask to
	 * atomic_flush, which runs after the bridge chain is enabled.
	 */
	decon->start_pending = true;
	decon->enabled = true;
}

static void zumapro_decon_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;

	if (!decon->enabled)
		return;

	zumapro_decon_stop(decon);
	decon->enabled = false;
	decon->start_pending = false;
	timer_delete_sync(&decon->frame_start_timer);
	pm_runtime_put_sync(decon->dev);
}

static void zumapro_decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	unsigned long flags;
	u32 val;
	int ret;

	if (!decon->enabled)
		return;

	/*
	 * Downstream's decon_reg_wait_update_done_and_mask(): wait until the
	 * previous commit's shadow-update requests have been consumed (they
	 * self-clear when a trigger latches them), then mask the trigger so
	 * plane, window and DPP registers can be programmed without a TE
	 * latching half-written state.
	 */
	ret = readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_SHD_REG_UP_REQ,
				 val, !val, 10, 300000);
	if (ret)
		dev_warn(decon->dev,
			 "DECON%u shadow update not consumed: %#x\n",
			 decon->id, val);

	spin_lock_irqsave(&decon->slock, flags);
	zumapro_dpu_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				ZUMAPRO_DECON_HW_TRIG_MASK,
				ZUMAPRO_DECON_HW_TRIG_MASK);
	spin_unlock_irqrestore(&decon->slock, flags);
}

/*
 * A command-mode frame is triggered by the panel's TE, so a frame that never
 * starts strands the commit that armed it: the vblank counter never advances
 * and the flip event queued by exynos_crtc_handle_event() is never delivered.
 * drm_atomic_helper_wait_for_vblanks() then warns after 100 ms, and the *next*
 * commit blocks 10 s per object in drm_atomic_helper_wait_for_dependencies(),
 * so one lost frame_start freezes the display for ~30 s.
 *
 * Downstream treats a missing frame_start as an expected condition rather than
 * an impossibility, and recovers: decon_wait_for_flip_done() gives the frame a
 * bounded time to start and otherwise calls decon_force_vblank_event().  Do the
 * same here with a timer, since this driver uses the generic commit tail.
 *
 * The timeout is deliberately shorter than the 100 ms
 * drm_atomic_helper_wait_for_vblanks() allows, so recovery lands before the
 * helper gives up and the stall never reaches the commit chain at all.  At
 * 60 Hz it is still more than three TE periods, and as downstream does, a
 * frame_start that is merely *pending* (the threaded handler has not run yet)
 * is not counted as a miss, so scheduler latency cannot trigger this.
 *
 * This is damage control, not a cure: the frame genuinely did not scan out, and
 * completing the vblank only keeps the pipeline moving.  The underlying cause
 * of a lost TE belongs in the panel/DSIM path.
 */
#define ZUMAPRO_DECON_FRAME_START_TIMEOUT_MS	60

static void zumapro_decon_frame_start_timeout(struct timer_list *t)
{
	struct zumapro_decon *decon = timer_container_of(decon, t,
							frame_start_timer);
	unsigned long flags;
	bool armed;
	u32 pend;

	spin_lock_irqsave(&decon->slock, flags);
	armed = decon->enabled;
	pend = armed ? readl(decon->main_regs + ZUMAPRO_DECON_INT_PEND) &
		       ZUMAPRO_DECON_INT_FRAME_START : 0;
	spin_unlock_irqrestore(&decon->slock, flags);

	if (!armed || pend)
		return;

	dev_warn_ratelimited(decon->dev,
			     "DECON%u frame start timed out, completing vblank\n",
			     decon->id);

	if (decon->crtc)
		drm_crtc_handle_vblank(&decon->crtc->base);
}

static void zumapro_decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	bool frame_expected = false;
	unsigned long flags;

	if (!decon->enabled)
		return;

	if (decon->start_pending) {
		zumapro_decon_start(decon);
		decon->start_pending = false;
		decon->win_dirty = false;
		frame_expected = true;
	} else if (decon->win_dirty) {
		int ret;

		/*
		 * Downstream's per-commit decon_reg_start(): request the
		 * window shadow updates, re-arm the per-frame enable and the
		 * global update (in command mode EN_F is consumed by each
		 * frame), drop a stale frame_start so the next one reflects
		 * this update, wait for run status, and only then unmask the
		 * trigger so the prepared frame transfers on the next TE.
		 */
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_SHD_REG_UP_REQ,
					ZUMAPRO_DECON_SHD_ALL_WINDOWS,
					ZUMAPRO_DECON_SHD_ALL_WINDOWS);
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_GLOBAL_CON,
					ZUMAPRO_DECON_GLOBAL_CON_EN |
					ZUMAPRO_DECON_GLOBAL_CON_EN_F,
					ZUMAPRO_DECON_GLOBAL_CON_EN |
					ZUMAPRO_DECON_GLOBAL_CON_EN_F);
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_SHD_REG_UP_REQ,
					ZUMAPRO_DECON_SHD_GLOBAL |
					ZUMAPRO_DECON_SHD_CMP,
					ZUMAPRO_DECON_SHD_GLOBAL |
					ZUMAPRO_DECON_SHD_CMP);

		spin_lock_irqsave(&decon->slock, flags);
		writel(ZUMAPRO_DECON_INT_FRAME_START,
		       decon->main_regs + ZUMAPRO_DECON_INT_PEND);
		spin_unlock_irqrestore(&decon->slock, flags);

		ret = zumapro_decon_wait_run(decon);
		if (ret)
			dev_warn(decon->dev,
				 "DECON%u did not enter run state: %d\n",
				 decon->id, ret);

		spin_lock_irqsave(&decon->slock, flags);
		zumapro_dpu_update_bits(decon->main_regs,
					ZUMAPRO_DECON_TRIG_CON,
					ZUMAPRO_DECON_HW_TRIG_MASK, 0);
		spin_unlock_irqrestore(&decon->slock, flags);
		decon->win_dirty = false;
		frame_expected = true;
	}

	/*
	 * A frame is now expected: either decon_start() or the win_dirty branch
	 * unmasked the trigger.  Arm the recovery timer before handing the event
	 * over, so a frame that never starts cannot strand this commit.
	 */
	if (frame_expected)
		mod_timer(&decon->frame_start_timer,
			  jiffies +
			  msecs_to_jiffies(ZUMAPRO_DECON_FRAME_START_TIMEOUT_MS));

	/* Arm the commit's flip event for delivery on the next frame_start. */
	exynos_crtc_handle_event(crtc);
}

static int zumapro_decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	unsigned long flags;

	spin_lock_irqsave(&decon->slock, flags);
	decon->vblank_enabled = true;
	spin_unlock_irqrestore(&decon->slock, flags);

	return 0;
}

static void zumapro_decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	unsigned long flags;

	spin_lock_irqsave(&decon->slock, flags);
	decon->vblank_enabled = false;
	spin_unlock_irqrestore(&decon->slock, flags);
}

/*
 * Command-mode frame transfers begin on a panel TE with the trigger
 * unmasked, so frame_start fires once per transferred frame and serves as
 * the vblank source.
 */
static irqreturn_t zumapro_decon_frame_start_irq(int irq, void *dev_id)
{
	struct zumapro_decon *decon = dev_id;
	bool deliver;
	u32 pend;

	spin_lock(&decon->slock);
	pend = readl(decon->main_regs + ZUMAPRO_DECON_INT_PEND) &
	       ZUMAPRO_DECON_INT_FRAME_START;
	if (pend)
		writel(pend, decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	deliver = decon->vblank_enabled;
	spin_unlock(&decon->slock);

	if (!pend)
		return IRQ_NONE;

	/* The frame started, so the recovery timer is not needed. */
	timer_delete(&decon->frame_start_timer);

	if (deliver && decon->crtc)
		drm_crtc_handle_vblank(&decon->crtc->base);

	return IRQ_HANDLED;
}

static irqreturn_t zumapro_decon_frame_done_irq(int irq, void *dev_id)
{
	struct zumapro_decon *decon = dev_id;
	u32 pend;

	spin_lock(&decon->slock);
	pend = readl(decon->main_regs + ZUMAPRO_DECON_INT_PEND) &
	       ZUMAPRO_DECON_INT_FRAME_DONE;
	if (pend)
		writel(pend, decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	spin_unlock(&decon->slock);

	return pend ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t zumapro_decon_extra_irq(int irq, void *dev_id)
{
	struct zumapro_decon *decon = dev_id;
	u32 pend;
	u32 extra = 0;

	spin_lock(&decon->slock);
	pend = readl(decon->main_regs + ZUMAPRO_DECON_INT_PEND) &
	       ZUMAPRO_DECON_INT_EXTRA;
	if (pend) {
		writel(pend, decon->main_regs + ZUMAPRO_DECON_INT_PEND);
		extra = readl(decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);
		writel(extra, decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);
	}
	spin_unlock(&decon->slock);

	if (!pend)
		return IRQ_NONE;

	dev_err_ratelimited(decon->dev,
			    "DECON%u %s%s(extra interrupt %#x)\n", decon->id,
			    extra & ZUMAPRO_DECON_INT_RESOURCE_CONFLICT ?
				"resource conflict " : "",
			    extra & ZUMAPRO_DECON_INT_TIMEOUT ?
				"timeout " : "",
			    extra);

	return IRQ_HANDLED;
}

static void zumapro_decon_update_plane(struct exynos_drm_crtc *crtc,
				       struct exynos_drm_plane *plane)
{
	struct zumapro_decon *decon = crtc->ctx;
	struct exynos_drm_plane_state *state =
		to_exynos_plane_state(plane->base.state);
	struct zumapro_dpp *dpp = decon->dpp;
	unsigned int win = plane->index;
	u32 blend_func;
	u32 blend_coeff;
	u32 ch;

	if (!decon->enabled || !dpp || !state->base.fb)
		return;

	if (!zumapro_dpp_find_format(state->base.fb->format->format))
		return;

	if (!dpp->initialized) {
		zumapro_dpp_init(dpp);
		dpp->initialized = true;
	}

	zumapro_dpp_update(dpp, state);

	blend_func = ZUMAPRO_DECON_WIN_FUNC(ZUMAPRO_DECON_WIN_FUNC_USER_DEFINED) |
		     ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_SEL(
				ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_AF);
	blend_coeff =
		ZUMAPRO_DECON_WIN_FG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO) |
		ZUMAPRO_DECON_WIN_FG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO);

	writel(blend_func, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_0(win));
	writel(blend_coeff, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_1(win));
	writel(ZUMAPRO_DECON_WIN_POS_Y(state->crtc.y) |
	       ZUMAPRO_DECON_WIN_POS_X(state->crtc.x),
	       decon->win_regs + ZUMAPRO_DECON_WIN_START_POSITION(win));
	writel(ZUMAPRO_DECON_WIN_POS_Y(state->crtc.y + state->crtc.h - 1) |
	       ZUMAPRO_DECON_WIN_POS_X(state->crtc.x + state->crtc.w - 1),
	       decon->win_regs + ZUMAPRO_DECON_WIN_END_POSITION(win));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_START_TIME_CON(win));

	/* The window channel map skips the L7 layer, absent on Zumapro. */
	ch = dpp->id;
	if (ch >= 7)
		ch++;
	writel(ZUMAPRO_DECON_WIN_CHMAP(ch) | ZUMAPRO_DECON_WIN_EN,
	       decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(win));

	decon->win_dirty = true;
}

static void zumapro_decon_disable_plane(struct exynos_drm_crtc *crtc,
					struct exynos_drm_plane *plane)
{
	struct zumapro_decon *decon = crtc->ctx;

	if (!decon->enabled)
		return;

	writel(0, decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(plane->index));
	decon->win_dirty = true;
}

static const struct exynos_drm_crtc_ops zumapro_decon_crtc_ops = {
	.atomic_enable = zumapro_decon_atomic_enable,
	.atomic_disable = zumapro_decon_atomic_disable,
	.atomic_begin = zumapro_decon_atomic_begin,
	.atomic_flush = zumapro_decon_atomic_flush,
	.enable_vblank = zumapro_decon_enable_vblank,
	.disable_vblank = zumapro_decon_disable_vblank,
	.mode_valid = zumapro_decon_mode_valid,
	.update_plane = zumapro_decon_update_plane,
	.disable_plane = zumapro_decon_disable_plane,
};

static int zumapro_decon_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct zumapro_decon *decon = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	if (!decon->pipeline)
		return dev_err_probe(dev, -ENODEV,
				     "DECON%u has no validated panel pipeline\n",
				     decon->id);

	decon->drm_dev = drm_dev;
	decon->plane_config.pixel_formats = zumapro_dpp_graphics_formats;
	decon->plane_config.num_pixel_formats =
		ARRAY_SIZE(zumapro_dpp_graphics_formats);
	decon->plane_config.zpos = 0;
	decon->plane_config.type = DRM_PLANE_TYPE_PRIMARY;

	ret = exynos_plane_init(drm_dev, &decon->plane, 0,
				&decon->plane_config);
	if (ret)
		return ret;

	decon->crtc = exynos_drm_crtc_create(drm_dev, &decon->plane.base,
					     EXYNOS_DISPLAY_TYPE_LCD,
					     &zumapro_decon_crtc_ops,
					     decon);
	if (IS_ERR(decon->crtc))
		return PTR_ERR(decon->crtc);

	return exynos_drm_register_dma(drm_dev, dev, &decon->dma_priv);
}

static void zumapro_decon_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct zumapro_decon *decon = dev_get_drvdata(dev);

	if (decon->crtc)
		zumapro_decon_atomic_disable(decon->crtc);

	exynos_drm_unregister_dma(decon->drm_dev, dev, &decon->dma_priv);
}

static const struct component_ops zumapro_decon_component_ops = {
	.bind = zumapro_decon_bind,
	.unbind = zumapro_decon_unbind,
};

static int zumapro_read_u32_compat(struct device *dev, const char *name,
				   const char *legacy_name, u32 *value)
{
	int ret;

	if (of_property_present(dev->of_node, name)) {
		ret = of_property_count_u32_elems(dev->of_node, name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     name);
		ret = of_property_read_u32(dev->of_node, name, value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "malformed DT property %s\n", name);
		return 0;
	}

	if (legacy_name) {
		if (!of_property_present(dev->of_node, legacy_name))
			return dev_err_probe(dev, -EINVAL,
					     "missing DT property %s\n", name);

		ret = of_property_count_u32_elems(dev->of_node, legacy_name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     legacy_name);
		ret = of_property_read_u32(dev->of_node, legacy_name, value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "malformed DT property %s\n",
					     legacy_name);

		dev_warn(dev, "using legacy DT property %s; prefer %s\n",
			 legacy_name, name);
		return 0;
	}

	return dev_err_probe(dev, -EINVAL, "missing DT property %s\n", name);
}

static int zumapro_read_u32_optional_compat(struct device *dev,
					    const char *name,
					    const char *legacy_name,
					    u32 *value)
{
	int ret;

	if (of_property_present(dev->of_node, name)) {
		ret = of_property_count_u32_elems(dev->of_node, name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     name);
		of_property_read_u32(dev->of_node, name, value);
		return 0;
	}

	if (!legacy_name || !of_property_present(dev->of_node, legacy_name))
		return 0;

	ret = of_property_count_u32_elems(dev->of_node, legacy_name);
	if (ret != 1)
		return dev_err_probe(dev, -EINVAL,
				     "DT property %s must contain one u32\n",
				     legacy_name);

	of_property_read_u32(dev->of_node, legacy_name, value);

	if (legacy_name)
		dev_warn(dev, "using legacy DT property %s; prefer %s\n",
			 legacy_name, name);

	return 0;
}

static int zumapro_check_reg_names(struct platform_device *pdev,
				   const char * const *names, unsigned int count)
{
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (!platform_get_resource_byname(pdev, IORESOURCE_MEM,
						  names[i]))
			return dev_err_probe(dev, -EINVAL,
					     "missing %s MMIO resource\n",
					     names[i]);
	}

	return 0;
}

static int zumapro_check_irq_names(struct platform_device *pdev,
				   const char * const *names, unsigned int count)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int irq;

	for (i = 0; i < count; i++) {
		irq = platform_get_irq_byname_optional(pdev, names[i]);
		if (irq < 0)
			return dev_err_probe(dev, irq, "missing %s IRQ\n",
					     names[i]);
	}

	return 0;
}

static int zumapro_dpp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zumapro_dpp *dpp;
	int ret;

	dpp = devm_kzalloc(dev, sizeof(*dpp), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

	dpp->dev = dev;

	ret = zumapro_read_u32_compat(dev, "google,dpp-id", "dpp,id",
				      &dpp->id);
	if (ret)
		return ret;

	if (dpp->id >= ZUMAPRO_DPU_FETCH_DPP_COUNT)
		return dev_err_probe(dev, -EINVAL,
				     "DPP%u is not a normal fetch DPP\n",
				     dpp->id);

	ret = zumapro_read_u32_optional_compat(dev, "google,dpp-attributes",
					       "attr", &dpp->attributes);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,axi-port",
					       "port", &dpp->axi_port);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,scale-down",
					       "scale_down", &dpp->scale_down);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,scale-up",
					       "scale_up", &dpp->scale_up);
	if (ret)
		return ret;

	dpp->video_formats = of_property_read_bool(dev->of_node,
						   "google,video-formats") ||
			     of_property_read_bool(dev->of_node, "dpp,video");

	ret = zumapro_dpp_select_formats(dpp);
	if (ret)
		return ret;
	dpp->restrictions = &zumapro_dpp_restrictions;

	ret = zumapro_check_reg_names(pdev, zumapro_dpp_reg_names,
				      ARRAY_SIZE(zumapro_dpp_reg_names));
	if (ret)
		return ret;

	ret = zumapro_check_irq_names(pdev, zumapro_dpp_irq_names,
				      ARRAY_SIZE(zumapro_dpp_irq_names));
	if (ret)
		return ret;

	dpp->dma_regs = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(dpp->dma_regs))
		return PTR_ERR(dpp->dma_regs);

	dpp->dpp_regs = devm_platform_ioremap_resource_byname(pdev, "dpp");
	if (IS_ERR(dpp->dpp_regs))
		return PTR_ERR(dpp->dpp_regs);

	dpp->sramc_regs = devm_platform_ioremap_resource_byname(pdev, "sramc");
	if (IS_ERR(dpp->sramc_regs))
		return PTR_ERR(dpp->sramc_regs);

	dpp->hdr_comm_regs =
		devm_platform_ioremap_resource_byname(pdev, "hdr_comm");
	if (IS_ERR(dpp->hdr_comm_regs))
		return PTR_ERR(dpp->hdr_comm_regs);

	platform_set_drvdata(pdev, dpp);
	dev_dbg(dev, "registered DPP%u topology\n", dpp->id);

	return 0;
}

static const struct of_device_id zumapro_dpp_of_match[] = {
	{ .compatible = "google,zumapro-dpp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zumapro_dpp_of_match);

struct platform_driver zumapro_dpp_driver = {
	.probe = zumapro_dpp_probe,
	.driver = {
		.name = "exynos-zumapro-dpp",
		.of_match_table = zumapro_dpp_of_match,
	},
};

static int zumapro_decon_request_irq(struct platform_device *pdev,
				     struct zumapro_decon *decon,
				     const char *name, irq_handler_t handler)
{
	struct device *dev = &pdev->dev;
	int irq;
	int ret;

	irq = platform_get_irq_byname(pdev, name);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, handler, 0, dev_name(dev), decon);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request %s IRQ\n",
				     name);

	return 0;
}

static int zumapro_decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct zumapro_decon_desc *desc;
	struct zumapro_decon *decon;
	int ret;

	decon = devm_kzalloc(dev, sizeof(*decon), GFP_KERNEL);
	if (!decon)
		return -ENOMEM;

	decon->dev = dev;

	ret = zumapro_read_u32_compat(dev, "google,decon-id", "decon,id",
				      &decon->id);
	if (ret)
		return ret;

	desc = zumapro_decon_desc_by_id(decon->id);
	if (!desc)
		return dev_err_probe(dev, -EINVAL, "unsupported DECON%u\n",
				     decon->id);

	if (decon->id == 0) {
		const char *panel = NULL;

		/*
		 * The pipeline carries the panel's mode list, DSC config and
		 * link parameters, and mode_valid() rejects anything not in it,
		 * so it has to match the panel actually wired to this DECON.
		 * Boards name theirs; tg4c stays the default for tegu, which
		 * predates the property.
		 */
		of_property_read_string(dev->of_node, "google,panel-pipeline",
					&panel);
		if (panel && !strcmp(panel, "km4"))
			decon->pipeline = &zumapro_decon0_km4_pipeline;
		else
			decon->pipeline = &zumapro_decon0_tg4c_pipeline;
	}

	if (desc->has_cgc_dma) {
		ret = zumapro_read_u32_optional_compat(dev,
						       "google,cgc-dma-id",
						       "cgc-dma,id",
						       &decon->cgc_dma_id);
		if (ret)
			return ret;
	}

	ret = zumapro_read_u32_optional_compat(dev, "google,max-windows",
					       "max_win", &decon->max_windows);
	if (ret)
		return ret;

	ret = zumapro_check_reg_names(pdev, desc->reg_names,
				      desc->num_reg_names);
	if (ret)
		return ret;

	ret = zumapro_check_irq_names(pdev, desc->irq_names,
				      desc->num_irq_names);
	if (ret)
		return ret;

	decon->main_regs = devm_platform_ioremap_resource_byname(pdev, "main");
	if (IS_ERR(decon->main_regs))
		return PTR_ERR(decon->main_regs);

	decon->win_regs = devm_platform_ioremap_resource_byname(pdev, "win");
	if (IS_ERR(decon->win_regs))
		return PTR_ERR(decon->win_regs);

	decon->sub_regs = devm_platform_ioremap_resource_byname(pdev, "sub");
	if (IS_ERR(decon->sub_regs))
		return PTR_ERR(decon->sub_regs);

	decon->wincon_regs =
		devm_platform_ioremap_resource_byname(pdev, "wincon");
	if (IS_ERR(decon->wincon_regs))
		return PTR_ERR(decon->wincon_regs);

	if (decon->pipeline) {
		decon->dqe_regs =
			devm_platform_ioremap_resource_byname(pdev, "dqe");
		if (IS_ERR(decon->dqe_regs))
			return PTR_ERR(decon->dqe_regs);
	}

	decon->dpp_count = of_count_phandle_with_args(dev->of_node, "dpps",
						      NULL);
	if (decon->dpp_count == -ENOENT)
		decon->dpp_count = 0;
	else if (decon->dpp_count < 0)
		return dev_err_probe(dev, decon->dpp_count,
				     "failed to parse dpps\n");

	if (decon->dpp_count > 0) {
		struct platform_device *dpp_pdev;
		struct device_node *np;

		np = of_parse_phandle(dev->of_node, "dpps", 0);
		dpp_pdev = np ? of_find_device_by_node(np) : NULL;
		of_node_put(np);
		if (!dpp_pdev)
			return -EPROBE_DEFER;

		decon->dpp = platform_get_drvdata(dpp_pdev);
		if (!device_link_add(dev, &dpp_pdev->dev,
				     DL_FLAG_AUTOREMOVE_CONSUMER)) {
			put_device(&dpp_pdev->dev);
			return dev_err_probe(dev, -EINVAL,
					     "failed to link DPP device\n");
		}
		put_device(&dpp_pdev->dev);

		if (!decon->dpp)
			return -EPROBE_DEFER;
	}

	spin_lock_init(&decon->slock);
	timer_setup(&decon->frame_start_timer,
		    zumapro_decon_frame_start_timeout, 0);

	/*
	 * The DPU power domains are always on, so the registers are reachable
	 * here.  The bootloader hands off a live DECON that may have interrupt
	 * sources enabled and latched; quiesce them before the line has a
	 * handler.
	 */
	writel(0, decon->main_regs + ZUMAPRO_DECON_INT_EN);
	writel(0, decon->main_regs + ZUMAPRO_DECON_INT_EN_EXTRA);
	writel(ZUMAPRO_DECON_INT_FRAME_START | ZUMAPRO_DECON_INT_FRAME_DONE |
	       ZUMAPRO_DECON_INT_EXTRA,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND);
	writel(ZUMAPRO_DECON_INT_RESOURCE_CONFLICT | ZUMAPRO_DECON_INT_TIMEOUT,
	       decon->main_regs + ZUMAPRO_DECON_INT_PEND_EXTRA);

	ret = zumapro_decon_request_irq(pdev, decon, "frame_start",
					zumapro_decon_frame_start_irq);
	if (ret)
		return ret;

	ret = zumapro_decon_request_irq(pdev, decon, "frame_done",
					zumapro_decon_frame_done_irq);
	if (ret)
		return ret;

	ret = zumapro_decon_request_irq(pdev, decon, "extra",
					zumapro_decon_extra_irq);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, decon);
	pm_runtime_enable(dev);

	ret = component_add(dev, &zumapro_decon_component_ops);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	dev_info(dev, "registered DECON%u CRTC with %d DPPs\n",
		 decon->id, decon->dpp_count);

	return 0;
}

static void zumapro_decon_remove(struct platform_device *pdev)
{
	struct zumapro_decon *decon = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	component_del(dev, &zumapro_decon_component_ops);
	timer_delete_sync(&decon->frame_start_timer);
	pm_runtime_disable(dev);
}

static const struct of_device_id zumapro_decon_of_match[] = {
	{ .compatible = "google,zumapro-decon" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zumapro_decon_of_match);

struct platform_driver zumapro_decon_driver = {
	.probe = zumapro_decon_probe,
	.remove = zumapro_decon_remove,
	.driver = {
		.name = "exynos-zumapro-decon",
		.of_match_table = zumapro_decon_of_match,
	},
};

MODULE_DESCRIPTION("Google Tensor G4 Zumapro DPU bring-up scaffold");
MODULE_LICENSE("GPL");
