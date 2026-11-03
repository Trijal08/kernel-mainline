// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX858 image sensor, as fitted in the Pixel 9 Pro (front camera).
 *
 * One mode, 3840x2736 RAW10 over 3-trio C-PHY, replayed from the vendor
 * register tables. Only the streaming controls libcamera needs.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX858_REG_ID		CCI_REG16(0x0016)
#define IMX858_ID		0x0858
#define IMX858_REG_MODE_SELECT	CCI_REG8(0x0100)
#define IMX858_REG_EXPOSURE	CCI_REG16(0x0202)
#define IMX858_REG_AGAIN	CCI_REG16(0x0204)
#define IMX858_REG_FLL		CCI_REG16(0x0340)

#define IMX858_WIDTH		3840
#define IMX858_HEIGHT		2736
#define IMX858_LLP		7500
#define IMX858_FLL		7848
#define IMX858_FLL_MAX		0xffff
#define IMX858_EXPOSURE_MARGIN	48
#define IMX858_AGAIN_MAX	960	/* gain = 1024 / (1024 - code) */
/* the vendor mode runs 15 fps: measured 14.9 from the WDMA frame-done IRQ */
#define IMX858_PIXEL_RATE	(IMX858_LLP * IMX858_FLL * 15ULL)

/* Vendor power-up order; delays after each rail. */
static const char *const imx858_supplies[] = { "pp3200_bb", "pp1350", "vdig", "vddio", "af" };
static const unsigned int imx858_supply_delay_us[] = { 0, 0, 0, 1000, 0 };

static const struct cci_reg_sequence imx858_mode_3840x2736[] = {
	{ CCI_REG8(0x0136), 0x18 }, { CCI_REG8(0x0137), 0x93 }, { CCI_REG8(0x3304), 0x00 }, { CCI_REG8(0x33f0), 0x04 },
	{ CCI_REG8(0x33f1), 0x05 }, { CCI_REG8(0x0111), 0x03 }, { CCI_REG8(0x1200), 0x02 }, { CCI_REG8(0x1201), 0x02 },
	{ CCI_REG8(0x130b), 0x00 }, { CCI_REG8(0x1340), 0x00 }, { CCI_REG8(0x3bc0), 0xbf }, { CCI_REG8(0x3bc4), 0xbf },
	{ CCI_REG8(0x3bc8), 0xbf }, { CCI_REG8(0x3bcc), 0xbf }, { CCI_REG8(0x558f), 0x00 }, { CCI_REG8(0x5e2e), 0x00 },
	{ CCI_REG8(0x5e2f), 0x32 }, { CCI_REG8(0x5e32), 0x08 }, { CCI_REG8(0x5e33), 0xcd }, { CCI_REG8(0x5e64), 0x00 },
	{ CCI_REG8(0x5e65), 0x32 }, { CCI_REG8(0x5e68), 0x0b }, { CCI_REG8(0x5e69), 0x97 }, { CCI_REG8(0x61e8), 0x50 },
	{ CCI_REG8(0x61e9), 0x00 }, { CCI_REG8(0x61ea), 0x50 }, { CCI_REG8(0x61eb), 0x00 }, { CCI_REG8(0x7220), 0xff },
	{ CCI_REG8(0x7221), 0xff }, { CCI_REG8(0x7222), 0xff }, { CCI_REG8(0x7223), 0xff }, { CCI_REG8(0x7755), 0x09 },
	{ CCI_REG8(0x775b), 0x01 }, { CCI_REG8(0x7a28), 0x2d }, { CCI_REG8(0x7a29), 0x30 }, { CCI_REG8(0x7a2a), 0x30 },
	{ CCI_REG8(0x7a2b), 0x0e }, { CCI_REG8(0x7a2c), 0x10 }, { CCI_REG8(0x7a2d), 0x10 }, { CCI_REG8(0x7a2e), 0x0e },
	{ CCI_REG8(0x7a2f), 0x0f }, { CCI_REG8(0x7a30), 0x0f }, { CCI_REG8(0x7a31), 0x10 }, { CCI_REG8(0x7a32), 0x10 },
	{ CCI_REG8(0x7a33), 0x10 }, { CCI_REG8(0x7a34), 0x0e }, { CCI_REG8(0x7a35), 0x12 }, { CCI_REG8(0x7a36), 0x15 },
	{ CCI_REG8(0x7a3a), 0x2d }, { CCI_REG8(0x7a3b), 0x30 }, { CCI_REG8(0x7a3c), 0x31 }, { CCI_REG8(0x7a3d), 0x2b },
	{ CCI_REG8(0x7a3e), 0x2d }, { CCI_REG8(0x7a3f), 0x2e }, { CCI_REG8(0x7a40), 0x2e }, { CCI_REG8(0x7a41), 0x2f },
	{ CCI_REG8(0x7a42), 0x2f }, { CCI_REG8(0x7a43), 0x2e }, { CCI_REG8(0x7a44), 0x2f }, { CCI_REG8(0x7a45), 0x2e },
	{ CCI_REG8(0x7a46), 0x2f }, { CCI_REG8(0x7a47), 0x31 }, { CCI_REG8(0x7a48), 0x34 }, { CCI_REG8(0x7a4c), 0x2f },
	{ CCI_REG8(0x7a4d), 0x31 }, { CCI_REG8(0x7a4e), 0x31 }, { CCI_REG8(0x7a4f), 0x2d }, { CCI_REG8(0x7a50), 0x2f },
	{ CCI_REG8(0x7a51), 0x31 }, { CCI_REG8(0x7a52), 0x2f }, { CCI_REG8(0x7a53), 0x31 }, { CCI_REG8(0x7a54), 0x31 },
	{ CCI_REG8(0x7a55), 0x2f }, { CCI_REG8(0x7a56), 0x30 }, { CCI_REG8(0x7a57), 0x30 }, { CCI_REG8(0x7a58), 0x30 },
	{ CCI_REG8(0x7a59), 0x31 }, { CCI_REG8(0x7a5a), 0x36 }, { CCI_REG8(0x7a5b), 0x31 }, { CCI_REG8(0x7a5c), 0x33 },
	{ CCI_REG8(0x7a5e), 0x2f }, { CCI_REG8(0x7a5f), 0x33 }, { CCI_REG8(0x7a60), 0x32 }, { CCI_REG8(0x7a61), 0x2d },
	{ CCI_REG8(0x7a62), 0x30 }, { CCI_REG8(0x7a63), 0x31 }, { CCI_REG8(0x7a64), 0x30 }, { CCI_REG8(0x7a65), 0x30 },
	{ CCI_REG8(0x7a66), 0x30 }, { CCI_REG8(0x7a67), 0x30 }, { CCI_REG8(0x7a68), 0x31 }, { CCI_REG8(0x7a69), 0x31 },
	{ CCI_REG8(0x7a6a), 0x30 }, { CCI_REG8(0x7a6b), 0x30 }, { CCI_REG8(0x7a6c), 0x37 }, { CCI_REG8(0x7a6d), 0x32 },
	{ CCI_REG8(0x7a6e), 0x33 }, { CCI_REG8(0x7a70), 0x2f }, { CCI_REG8(0x7a71), 0x30 }, { CCI_REG8(0x7a72), 0x31 },
	{ CCI_REG8(0x7a73), 0x31 }, { CCI_REG8(0x7a74), 0x32 }, { CCI_REG8(0x7a75), 0x32 }, { CCI_REG8(0x7a76), 0x31 },
	{ CCI_REG8(0x7a77), 0x31 }, { CCI_REG8(0x7a78), 0x32 }, { CCI_REG8(0x7a79), 0x32 }, { CCI_REG8(0x7a7a), 0x31 },
	{ CCI_REG8(0x7a7b), 0x33 }, { CCI_REG8(0x7a7c), 0x33 }, { CCI_REG8(0x7a7d), 0x34 }, { CCI_REG8(0x7a7f), 0x2f },
	{ CCI_REG8(0x7a80), 0x31 }, { CCI_REG8(0x7a81), 0x32 }, { CCI_REG8(0x7a82), 0x31 }, { CCI_REG8(0x7a83), 0x31 },
	{ CCI_REG8(0x7a84), 0x31 }, { CCI_REG8(0x7a85), 0x31 }, { CCI_REG8(0x7a86), 0x32 }, { CCI_REG8(0x7a87), 0x31 },
	{ CCI_REG8(0x7a88), 0x31 }, { CCI_REG8(0x7a89), 0x32 }, { CCI_REG8(0x7a8a), 0x34 }, { CCI_REG8(0x7a8b), 0x33 },
	{ CCI_REG8(0x7a8c), 0x35 }, { CCI_REG8(0x7a90), 0x02 }, { CCI_REG8(0x7a92), 0x01 }, { CCI_REG8(0x7a95), 0x01 },
	{ CCI_REG8(0x7a98), 0x03 }, { CCI_REG8(0x7aa2), 0x02 }, { CCI_REG8(0x7aa5), 0x05 }, { CCI_REG8(0x7aaa), 0x08 },
	{ CCI_REG8(0x7aab), 0x02 }, { CCI_REG8(0x7ab4), 0x18 }, { CCI_REG8(0x7ab7), 0x06 }, { CCI_REG8(0x7abc), 0x03 },
	{ CCI_REG8(0x7abd), 0x02 }, { CCI_REG8(0x7ace), 0x06 }, { CCI_REG8(0x7acf), 0x07 }, { CCI_REG8(0x7aec), 0x01 },
	{ CCI_REG8(0x7b27), 0x09 }, { CCI_REG8(0x7b28), 0x08 }, { CCI_REG8(0x7b39), 0x06 }, { CCI_REG8(0x7b3a), 0x07 },
	{ CCI_REG8(0x7b48), 0x07 }, { CCI_REG8(0x7b49), 0x09 }, { CCI_REG8(0x7b57), 0x05 }, { CCI_REG8(0x7b58), 0x06 },
	{ CCI_REG8(0x7c18), 0x2d }, { CCI_REG8(0x7c1e), 0x2d }, { CCI_REG8(0x7c22), 0x23 }, { CCI_REG8(0x7c23), 0x1e },
	{ CCI_REG8(0x7d5d), 0x19 }, { CCI_REG8(0x7d5e), 0x19 }, { CCI_REG8(0x7d5f), 0x19 }, { CCI_REG8(0x7d60), 0x19 },
	{ CCI_REG8(0x7d61), 0x19 }, { CCI_REG8(0x7d62), 0x19 }, { CCI_REG8(0x7d64), 0x19 }, { CCI_REG8(0x7d65), 0x19 },
	{ CCI_REG8(0x7d66), 0x19 }, { CCI_REG8(0x7d67), 0x19 }, { CCI_REG8(0x7d68), 0x19 }, { CCI_REG8(0x7d69), 0x19 },
	{ CCI_REG8(0x7d6b), 0x19 }, { CCI_REG8(0x7d6c), 0x19 }, { CCI_REG8(0x7d6d), 0x19 }, { CCI_REG8(0x7d6e), 0x19 },
	{ CCI_REG8(0x7d6f), 0x19 }, { CCI_REG8(0x7d70), 0x19 }, { CCI_REG8(0x7d72), 0x19 }, { CCI_REG8(0x7d73), 0x19 },
	{ CCI_REG8(0x7d74), 0x19 }, { CCI_REG8(0x7d75), 0x19 }, { CCI_REG8(0x7d76), 0x19 }, { CCI_REG8(0x7d77), 0x19 },
	{ CCI_REG8(0x7d79), 0x19 }, { CCI_REG8(0x7d7a), 0x19 }, { CCI_REG8(0x7d7b), 0x19 }, { CCI_REG8(0x7d7c), 0x19 },
	{ CCI_REG8(0x7d7d), 0x19 }, { CCI_REG8(0x7d7f), 0x19 }, { CCI_REG8(0x7d80), 0x19 }, { CCI_REG8(0x7d81), 0x19 },
	{ CCI_REG8(0x7d82), 0x19 }, { CCI_REG8(0x7d83), 0x19 }, { CCI_REG8(0x90b4), 0x0b }, { CCI_REG8(0x90b5), 0x2c },
	{ CCI_REG8(0x90b8), 0x0c }, { CCI_REG8(0x90b9), 0x3c }, { CCI_REG8(0x90e7), 0x01 }, { CCI_REG8(0x920c), 0x90 },
	{ CCI_REG8(0x920e), 0x53 }, { CCI_REG8(0x920f), 0x0c }, { CCI_REG8(0x9210), 0xa0 }, { CCI_REG8(0x9212), 0xdd },
	{ CCI_REG8(0x9213), 0xda }, { CCI_REG8(0x9214), 0xa0 }, { CCI_REG8(0x9216), 0xeb }, { CCI_REG8(0x9217), 0x96 },
	{ CCI_REG8(0x9218), 0xa0 }, { CCI_REG8(0x921a), 0xdd }, { CCI_REG8(0x921b), 0xd7 }, { CCI_REG8(0x9674), 0x21 },
	{ CCI_REG8(0x9675), 0x5c }, { CCI_REG8(0x96af), 0x01 }, { CCI_REG8(0x9739), 0x00 }, { CCI_REG8(0x973a), 0x13 },
	{ CCI_REG8(0x973b), 0x04 }, { CCI_REG8(0x973d), 0x00 }, { CCI_REG8(0x973e), 0x1c }, { CCI_REG8(0x973f), 0xf4 },
	{ CCI_REG8(0x9741), 0x00 }, { CCI_REG8(0x9742), 0x32 }, { CCI_REG8(0x9743), 0x48 }, { CCI_REG8(0xa2c3), 0x18 },
	{ CCI_REG8(0xa2f5), 0x04 }, { CCI_REG8(0xa722), 0x00 }, { CCI_REG8(0xad01), 0x0a }, { CCI_REG8(0xad02), 0x0a },
	{ CCI_REG8(0xad0e), 0x02 }, { CCI_REG8(0xdda9), 0x4e }, { CCI_REG8(0x4102), 0xfe }, { CCI_REG8(0x4113), 0x00 },
	{ CCI_REG8(0x504b), 0x0d }, { CCI_REG8(0x520c), 0x01 }, { CCI_REG8(0x520d), 0x07 }, { CCI_REG8(0x9cd5), 0x00 },
	{ CCI_REG8(0x1240), 0x00 }, { CCI_REG8(0x1241), 0x02 }, { CCI_REG8(0x1242), 0x03 }, { CCI_REG8(0x1243), 0x04 },
	{ CCI_REG8(0x1244), 0x04 }, { CCI_REG8(0x9cd2), 0x00 }, { CCI_REG8(0x9cd3), 0x82 }, { CCI_REG8(0x1248), 0x00 },
	{ CCI_REG8(0x0112), 0x0a }, { CCI_REG8(0x0113), 0x0a }, { CCI_REG8(0x0114), 0x02 }, { CCI_REG8(0x3239), 0x00 },
	{ CCI_REG8(0x0342), 0x0f }, { CCI_REG8(0x0343), 0xb8 }, { CCI_REG8(0x3850), 0x01 }, { CCI_REG8(0x3851), 0xc4 },
	{ CCI_REG8(0x0340), 0x0e }, { CCI_REG8(0x0341), 0x28 }, { CCI_REG8(0x0344), 0x00 }, { CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 }, { CCI_REG8(0x0347), 0x30 }, { CCI_REG8(0x0348), 0x1f }, { CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x17 }, { CCI_REG8(0x034b), 0xcf }, { CCI_REG8(0x0900), 0x01 }, { CCI_REG8(0x0901), 0x44 },
	{ CCI_REG8(0x0902), 0x00 }, { CCI_REG8(0x3005), 0x02 }, { CCI_REG8(0x3006), 0x02 }, { CCI_REG8(0x3140), 0x0a },
	{ CCI_REG8(0x3144), 0x00 }, { CCI_REG8(0x3148), 0x04 }, { CCI_REG8(0x31c0), 0x43 }, { CCI_REG8(0x31c1), 0x43 },
	{ CCI_REG8(0x3205), 0x00 }, { CCI_REG8(0x0408), 0x00 }, { CCI_REG8(0x0409), 0x10 }, { CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 }, { CCI_REG8(0x040c), 0x07 }, { CCI_REG8(0x040d), 0xe0 }, { CCI_REG8(0x040e), 0x05 },
	{ CCI_REG8(0x040f), 0xe8 }, { CCI_REG8(0x034c), 0x07 }, { CCI_REG8(0x034d), 0xe0 }, { CCI_REG8(0x034e), 0x05 },
	{ CCI_REG8(0x034f), 0xe8 }, { CCI_REG8(0x0301), 0x05 }, { CCI_REG8(0x0303), 0x02 }, { CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x01 }, { CCI_REG8(0x0307), 0x0c }, { CCI_REG8(0x030b), 0x02 }, { CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x03 }, { CCI_REG8(0x030f), 0xbe }, { CCI_REG8(0x3104), 0x01 }, { CCI_REG8(0x324c), 0x01 },
	{ CCI_REG8(0x3803), 0x01 }, { CCI_REG8(0x3804), 0x01 }, { CCI_REG8(0x3805), 0x01 }, { CCI_REG8(0x3806), 0x01 },
	{ CCI_REG8(0x38a0), 0x01 }, { CCI_REG8(0x38a1), 0x5e }, { CCI_REG8(0x38a2), 0x00 }, { CCI_REG8(0x38a3), 0x00 },
	{ CCI_REG8(0x38a4), 0x00 }, { CCI_REG8(0x38a5), 0x00 }, { CCI_REG8(0x38a8), 0x01 }, { CCI_REG8(0x38a9), 0x5e },
	{ CCI_REG8(0x38aa), 0x00 }, { CCI_REG8(0x38ab), 0x00 }, { CCI_REG8(0x38ac), 0x00 }, { CCI_REG8(0x38ad), 0x00 },
	{ CCI_REG8(0x38d0), 0x00 }, { CCI_REG8(0x38d1), 0xd2 }, { CCI_REG8(0x38d2), 0x00 }, { CCI_REG8(0x38d3), 0xd2 },
	{ CCI_REG8(0x38e0), 0x00 }, { CCI_REG8(0x38e1), 0x00 }, { CCI_REG8(0x38e2), 0x00 }, { CCI_REG8(0x38e3), 0x00 },
	{ CCI_REG8(0x38e4), 0x00 }, { CCI_REG8(0x38e5), 0x00 }, { CCI_REG8(0x38e6), 0x00 }, { CCI_REG8(0x38e7), 0x00 },
	{ CCI_REG8(0x3b00), 0x00 }, { CCI_REG8(0x3b01), 0x00 }, { CCI_REG8(0x3b04), 0x00 }, { CCI_REG8(0x3b05), 0x00 },
	{ CCI_REG8(0x3b06), 0x00 }, { CCI_REG8(0x3b07), 0x00 }, { CCI_REG8(0x3b0a), 0x00 }, { CCI_REG8(0x3b0b), 0x00 },
	{ CCI_REG8(0x0202), 0x03 }, { CCI_REG8(0x0203), 0xe8 }, { CCI_REG8(0x0204), 0x01 }, { CCI_REG8(0x0205), 0x34 },
	{ CCI_REG8(0x020e), 0x01 }, { CCI_REG8(0x020f), 0x00 }, { CCI_REG8(0x3103), 0x00 }, { CCI_REG8(0x3422), 0x01 },
	{ CCI_REG8(0x3423), 0xfc }, { CCI_REG8(0x3190), 0x00 }, { CCI_REG8(0x0224), 0x01 }, { CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x0216), 0x00 }, { CCI_REG8(0x0217), 0x00 }, { CCI_REG8(0x0218), 0x01 }, { CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x30a4), 0x01 }, { CCI_REG8(0x30a6), 0x00 }, { CCI_REG8(0x30c6), 0x01 }, { CCI_REG8(0x30c8), 0x01 },
	{ CCI_REG8(0x30f2), 0x01 }, { CCI_REG8(0x30f3), 0x01 }, { CCI_REG8(0x30a5), 0x2b }, { CCI_REG8(0x30a7), 0x30 },
	{ CCI_REG8(0x30c7), 0x30 }, { CCI_REG8(0x30c9), 0x30 }, { CCI_REG8(0x30a2), 0x00 }, { CCI_REG8(0x30c4), 0x01 },
	{ CCI_REG8(0x30f1), 0x01 }, { CCI_REG8(0x30a3), 0x30 }, { CCI_REG8(0x30c5), 0x30 }, { CCI_REG8(0x3970), 0x00 },
	{ CCI_REG8(0x0008), 0x00 }, { CCI_REG8(0x0009), 0x40 }, { CCI_REG8(0x0138), 0x01 }, { CCI_REG8(0x0200), 0x0b },
	{ CCI_REG8(0x0201), 0x60 }, { CCI_REG8(0x0202), 0x1e }, { CCI_REG8(0x0203), 0x78 }, { CCI_REG8(0x0204), 0x02 },
	{ CCI_REG8(0x0205), 0xb7 }, { CCI_REG8(0x0340), 0x1e }, { CCI_REG8(0x0341), 0xa8 }, { CCI_REG8(0x0342), 0x1d },
	{ CCI_REG8(0x0343), 0x4c }, { CCI_REG8(0x0346), 0x01 }, { CCI_REG8(0x0347), 0x50 }, { CCI_REG8(0x034a), 0x16 },
	{ CCI_REG8(0x034b), 0xaf }, { CCI_REG8(0x034c), 0x0f }, { CCI_REG8(0x034d), 0x00 }, { CCI_REG8(0x034e), 0x0a },
	{ CCI_REG8(0x034f), 0xb0 }, { CCI_REG8(0x0409), 0x80 }, { CCI_REG8(0x040c), 0x0f }, { CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x0a }, { CCI_REG8(0x040f), 0xb0 }, { CCI_REG8(0x0901), 0x22 }, { CCI_REG8(0x31c0), 0x41 },
	{ CCI_REG8(0x31c1), 0x41 }, { CCI_REG8(0x33f0), 0x07 }, { CCI_REG8(0x3850), 0x03 }, { CCI_REG8(0x3851), 0x48 },
	{ CCI_REG8(0x38a0), 0x00 }, { CCI_REG8(0x38a1), 0x51 }, { CCI_REG8(0x38a3), 0x51 }, { CCI_REG8(0x38a5), 0x51 },
	{ CCI_REG8(0x38a9), 0x56 }, { CCI_REG8(0x38aa), 0x01 }, { CCI_REG8(0x38ab), 0x56 }, { CCI_REG8(0x38ac), 0x01 },
	{ CCI_REG8(0x38ad), 0x56 }, { CCI_REG8(0x38d0), 0x09 }, { CCI_REG8(0x38d1), 0x10 }, { CCI_REG8(0x38d2), 0x09 },
	{ CCI_REG8(0x38d3), 0x10 }, { CCI_REG8(0x3b00), 0x06 }, { CCI_REG8(0x3b01), 0x20 }, { CCI_REG8(0x3b04), 0x06 },
	{ CCI_REG8(0x3b05), 0x20 },
};

struct imx858 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct regulator *supplies[ARRAY_SIZE(imx858_supplies)];
	struct gpio_desc *reset;
	struct clk *mclk;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure, *vblank;
};

static inline struct imx858 *to_imx858(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx858, sd);
}

static int imx858_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx858 *imx = container_of(ctrl->handler, struct imx858, ctrls);
	struct device *dev = imx->sd.dev;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		__v4l2_ctrl_modify_range(imx->exposure, 4,
					 IMX858_HEIGHT + ctrl->val - IMX858_EXPOSURE_MARGIN,
					 1, imx->exposure->val);

	if (!pm_runtime_get_if_in_use(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx->regmap, IMX858_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx->regmap, IMX858_REG_AGAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx->regmap, IMX858_REG_FLL, IMX858_HEIGHT + ctrl->val, &ret);
		break;
	}
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static const struct v4l2_ctrl_ops imx858_ctrl_ops = { .s_ctrl = imx858_set_ctrl };

static int imx858_init_controls(struct imx858 *imx)
{
	struct v4l2_ctrl_handler *h = &imx->ctrls;
	struct v4l2_ctrl *c;
	int ret;

	v4l2_ctrl_handler_init(h, 5);
	c = v4l2_ctrl_new_std(h, NULL, V4L2_CID_PIXEL_RATE, IMX858_PIXEL_RATE,
			      IMX858_PIXEL_RATE, 1, IMX858_PIXEL_RATE);
	if (c)
		c->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	c = v4l2_ctrl_new_std(h, NULL, V4L2_CID_HBLANK, IMX858_LLP - IMX858_WIDTH,
			      IMX858_LLP - IMX858_WIDTH, 1, IMX858_LLP - IMX858_WIDTH);
	if (c)
		c->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx->vblank = v4l2_ctrl_new_std(h, &imx858_ctrl_ops, V4L2_CID_VBLANK,
					IMX858_FLL - IMX858_HEIGHT, IMX858_FLL_MAX - IMX858_HEIGHT,
					1, IMX858_FLL - IMX858_HEIGHT);
	imx->exposure = v4l2_ctrl_new_std(h, &imx858_ctrl_ops, V4L2_CID_EXPOSURE, 4,
					  IMX858_FLL - IMX858_EXPOSURE_MARGIN, 1, 0x1e78);
	v4l2_ctrl_new_std(h, &imx858_ctrl_ops, V4L2_CID_ANALOGUE_GAIN, 0,
			  IMX858_AGAIN_MAX, 1, 0x2b7);
	if (h->error) {
		ret = h->error;
		v4l2_ctrl_handler_free(h);
		return ret;
	}
	imx->sd.ctrl_handler = h;
	return 0;
}

static int imx858_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int imx858_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;
	fse->min_width = fse->max_width = IMX858_WIDTH;
	fse->min_height = fse->max_height = IMX858_HEIGHT;
	return 0;
}

static int imx858_set_pad_format(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, 0);

	f->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	f->width = IMX858_WIDTH;
	f->height = IMX858_HEIGHT;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = V4L2_COLORSPACE_RAW;
	f->ycbcr_enc = V4L2_YCBCR_ENC_601;
	f->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	f->xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->format = *f;
	return 0;
}

static int imx858_get_selection(struct v4l2_subdev *sd,
				const struct v4l2_subdev_client_info *ci,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		/* one binned mode; the full array is 2x in each direction */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX858_WIDTH;
		sel->r.height = IMX858_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int imx858_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_TRY };

	return imx858_set_pad_format(sd, NULL, state, &fmt);
}

static int imx858_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct imx858 *imx = to_imx858(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sd->dev);
	if (ret < 0)
		return ret;
	ret = cci_multi_reg_write(imx->regmap, imx858_mode_3840x2736,
				  ARRAY_SIZE(imx858_mode_3840x2736), NULL);
	if (!ret)
		ret = __v4l2_ctrl_handler_setup(&imx->ctrls);
	if (!ret)
		ret = cci_write(imx->regmap, IMX858_REG_MODE_SELECT, 1, NULL);
	if (ret)
		pm_runtime_put_autosuspend(sd->dev);
	return ret;
}

static int imx858_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct imx858 *imx = to_imx858(sd);

	cci_write(imx->regmap, IMX858_REG_MODE_SELECT, 0, NULL);
	pm_runtime_put_autosuspend(sd->dev);
	return 0;
}

static const struct v4l2_subdev_pad_ops imx858_pad_ops = {
	.enum_mbus_code = imx858_enum_mbus_code,
	.enum_frame_size = imx858_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx858_set_pad_format,
	.get_selection = imx858_get_selection,
	.enable_streams = imx858_enable_streams,
	.disable_streams = imx858_disable_streams,
};

static const struct v4l2_subdev_video_ops imx858_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops imx858_subdev_ops = {
	.video = &imx858_video_ops,
	.pad = &imx858_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx858_internal_ops = {
	.init_state = imx858_init_state,
};

static int imx858_power_on(struct device *dev)
{
	struct imx858 *imx = to_imx858(dev_get_drvdata(dev));
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(imx858_supplies); i++) {
		ret = regulator_enable(imx->supplies[i]);
		if (ret) {
			dev_err(dev, "%s: %d\n", imx858_supplies[i], ret);
			while (i-- > 0)
				regulator_disable(imx->supplies[i]);
			return ret;
		}
		if (imx858_supply_delay_us[i])
			fsleep(imx858_supply_delay_us[i]);
	}
	gpiod_set_value_cansleep(imx->reset, 1);
	ret = clk_prepare_enable(imx->mclk);
	if (ret)
		dev_err(dev, "mclk: %d\n", ret);
	fsleep(5000);
	return ret;
}

static int imx858_power_off(struct device *dev)
{
	struct imx858 *imx = to_imx858(dev_get_drvdata(dev));
	int i;

	clk_disable_unprepare(imx->mclk);
	gpiod_set_value_cansleep(imx->reset, 0);
	for (i = ARRAY_SIZE(imx858_supplies); i-- > 0;)
		regulator_disable(imx->supplies[i]);
	return 0;
}

static int imx858_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx858 *imx;
	u64 id;
	int i, ret;

	imx = devm_kzalloc(dev, sizeof(*imx), GFP_KERNEL);
	if (!imx)
		return -ENOMEM;
	v4l2_i2c_subdev_init(&imx->sd, client, &imx858_subdev_ops);
	imx->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx->regmap))
		return PTR_ERR(imx->regmap);
	for (i = 0; i < ARRAY_SIZE(imx858_supplies); i++) {
		imx->supplies[i] = devm_regulator_get(dev, imx858_supplies[i]);
		if (IS_ERR(imx->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(imx->supplies[i]), "%s\n", imx858_supplies[i]);
	}
	imx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);	/* XCLR: high = run */
	if (IS_ERR(imx->reset))
		return dev_err_probe(dev, PTR_ERR(imx->reset), "reset gpio\n");
	imx->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(imx->mclk))
		return dev_err_probe(dev, PTR_ERR(imx->mclk), "mclk\n");

	ret = imx858_power_on(dev);
	if (ret)
		return ret;
	ret = cci_read(imx->regmap, IMX858_REG_ID, &id, NULL);
	if (!ret && id != IMX858_ID)
		ret = -ENODEV;
	if (ret) {
		dev_err(dev, "chip id 0x%llx: %d\n", id, ret);
		imx858_power_off(dev);
		return ret;
	}

	ret = imx858_init_controls(imx);
	if (ret)
		goto err_power;
	imx->sd.internal_ops = &imx858_internal_ops;
	imx->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx->sd.entity, 1, &imx->pad);
	if (ret)
		goto err_ctrls;
	ret = v4l2_subdev_init_finalize(&imx->sd);
	if (ret)
		goto err_entity;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&imx->sd);
	if (ret)
		goto err_pm;
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_get_noresume(dev);
	v4l2_subdev_cleanup(&imx->sd);
err_entity:
	media_entity_cleanup(&imx->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx->ctrls);
err_power:
	imx858_power_off(dev);
	return ret;
}

static void imx858_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx858 *imx = to_imx858(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx->ctrls);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx858_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops imx858_pm_ops = {
	RUNTIME_PM_OPS(imx858_power_off, imx858_power_on, NULL)
};

static const struct of_device_id imx858_of_match[] = {
	{ .compatible = "sony,imx858" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx858_of_match);

static struct i2c_driver imx858_driver = {
	.driver = {
		.name = "imx858",
		.of_match_table = imx858_of_match,
		.pm = pm_ptr(&imx858_pm_ops),
	},
	.probe = imx858_probe,
	.remove = imx858_remove,
};
module_i2c_driver(imx858_driver);

MODULE_DESCRIPTION("Sony IMX858 image sensor driver");
MODULE_LICENSE("GPL");
