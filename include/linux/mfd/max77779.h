/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google Inc.
 * Copyright 2025 Linaro Ltd.
 *
 * Maxim MAX77779 core driver
 */

#ifndef __LINUX_MFD_MAX77779_H
#define __LINUX_MFD_MAX77779_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

/*
 * PMIC (top) block register map. NOTE: the MAX77779 interrupt registers are
 * NOT the MAX77759's. There is no TOPSYS register on this part at all: the
 * per-source mask is INTB_MASK at 0x24 (not 0x23, which is a *status*
 * register), and the top-system faults live in INT_STS/INT_MASK at 0x2a/0x2b
 * behind INTSRC_STS.PMICTOP rather than behind bit 1. Using the MAX77759
 * offsets reads a mask register as status, writes the mask into a status
 * register, and leaves every source permanently unmasked.
 */
#define MAX77779_PMIC_REG_PMIC_ID               0x00
#define MAX77779_PMIC_REG_PMIC_REVISION         0x01
#define MAX77779_PMIC_REG_OTP_REVISION          0x02
/* INTSRC_STS is write-1-to-clear and summarises the sub-blocks below it. */
#define MAX77779_PMIC_REG_INTSRC_STS            0x22
#define   MAX77779_PMIC_REG_INTSRC_STS_TCPC     BIT(0)
#define   MAX77779_PMIC_REG_INTSRC_STS_FG       BIT(1)
#define   MAX77779_PMIC_REG_INTSRC_STS_CHGR     BIT(2)
#define   MAX77779_PMIC_REG_INTSRC_STS_I2CM     BIT(3)
#define   MAX77779_PMIC_REG_INTSRC_STS_BATTVIMON BIT(4)
#define   MAX77779_PMIC_REG_INTSRC_STS_GPIO     BIT(5)
#define   MAX77779_PMIC_REG_INTSRC_STS_VDROOP   BIT(6)
#define   MAX77779_PMIC_REG_INTSRC_STS_PMICTOP  BIT(7)
#define MAX77779_PMIC_REG_VDROOP_INT            0x23
#define   MAX77779_PMIC_REG_VDROOP_INT_OILO2_CNT BIT(0)
#define   MAX77779_PMIC_REG_VDROOP_INT_OILO1_CNT BIT(1)
#define   MAX77779_PMIC_REG_VDROOP_INT_UVLO2_CNT BIT(2)
#define   MAX77779_PMIC_REG_VDROOP_INT_UVLO1_CNT BIT(3)
#define   MAX77779_PMIC_REG_VDROOP_INT_BAT_OILO2 BIT(4)
#define   MAX77779_PMIC_REG_VDROOP_INT_BAT_OILO1 BIT(5)
#define   MAX77779_PMIC_REG_VDROOP_INT_SYS_UVLO2 BIT(6)
#define   MAX77779_PMIC_REG_VDROOP_INT_SYS_UVLO1 BIT(7)
/* INTB_MASK masks INTSRC_STS bit for bit; 1 masks. */
#define MAX77779_PMIC_REG_INTB_MASK             0x24
#define MAX77779_PMIC_REG_SPMI_INT_MASK         0x25
#define MAX77779_PMIC_REG_SPMI_INT_PRIORITY     0x26
#define MAX77779_PMIC_REG_VDROOP_INT_MASK       0x27
#define MAX77779_PMIC_REG_VDROOP_INT_SPMI_MASK  0x28
#define MAX77779_PMIC_REG_VDROOP_INT_SPMI_PRIORITY 0x29
/* INT_STS is the PMICTOP source; write-1-to-clear. Bit 0 (SPR_0) is spare. */
#define MAX77779_PMIC_REG_INT_STS               0x2a
#define   MAX77779_PMIC_REG_INT_STS_SPMIERR     BIT(1)
#define   MAX77779_PMIC_REG_INT_STS_VDDPOR      BIT(2)
#define   MAX77779_PMIC_REG_INT_STS_SYSMSGI     BIT(3)
#define   MAX77779_PMIC_REG_INT_STS_SYSUVLO     BIT(4)
#define   MAX77779_PMIC_REG_INT_STS_SYSOVLO     BIT(5)
#define   MAX77779_PMIC_REG_INT_STS_TSHDN       BIT(6)
#define   MAX77779_PMIC_REG_INT_STS_APCMDRESI   BIT(7)
/*
 * INT_MASK masks INT_STS from bit 1 up. Bit 0 is not a mask: FSHIP_NOT_RD is
 * a latch that reads 1 until the ship-mode exit reason has been read out, so
 * it must be left alone when masking.
 */
#define MAX77779_PMIC_REG_INT_MASK              0x2b
#define   MAX77779_PMIC_REG_INT_MASK_FSHIP_NOT_RD BIT(0)
#define MAX77779_PMIC_REG_I2C_CNFG              0x40
#define MAX77779_PMIC_REG_SWRESET               0x50
#define MAX77779_PMIC_REG_CONTROL_FG            0x51

/*
 * MaxQ (RISC-V) block. Shares the 'top' I2C address -- these are the PMIC
 * block's RISCV_* registers. The MAX77759's UIC_INTx/UIC_STATUSx window at
 * 0x64..0x73 does not exist here, and neither does a 33-byte mailbox: the
 * command window is AP_DATAOUT1..8 plus the opcode register that latches it,
 * and the response window is AP_DATAIN0..10.
 */
#define MAX77779_MAXQ_REG_RISCV_DEVICE_ID       0x60
#define MAX77779_MAXQ_REG_RISCV_DEVICE_REV      0x61
#define MAX77779_MAXQ_REG_RISCV_FW_REV          0x62
#define MAX77779_MAXQ_REG_RISCV_FW_SUB_REV      0x63
#define MAX77779_MAXQ_REG_AP_DATAOUT1           0x81
#define MAX77779_MAXQ_REG_AP_DATAOUT8           0x88
#define MAX77779_MAXQ_REG_AP_DATAOUT_OPCODE     0x89
#define MAX77779_MAXQ_REG_AP_DATAIN0            0xb1
#define MAX77779_MAXQ_REG_AP_DATAIN10           0xbb
#define MAX77779_MAXQ_REG_SYSMSG                0xbc
#define MAX77779_MAXQ_REG_COMMAND_HW            0xbf

/*
 * Charger block register map. NOTE: despite the shared register *names*, the
 * MAX77779 addresses are NOT those of the MAX77759 -- the whole block is
 * shifted (CHG_CNFG_00 is 0xbc, not 0xb9) and there is an extra
 * CHG_DETAILS_04. Using the MAX77759 offsets aims every config write at the
 * wrong register and the mode writes at read-only status registers.
 */
#define MAX77779_CHGR_REG_CHG_INT               0xb0
#define MAX77779_CHGR_REG_CHG_INT2              0xb1
#define MAX77779_CHGR_REG_CHG_INT_MASK          0xb3
#define MAX77779_CHGR_REG_CHG_INT2_MASK         0xb4
#define MAX77779_CHGR_REG_CHG_INT_OK            0xb6
#define   MAX77779_CHGR_REG_CHG_INT_OK_CHG      BIT(4)
#define   MAX77779_CHGR_REG_CHG_INT_OK_CHGIN    BIT(6)
#define MAX77779_CHGR_REG_CHG_DETAILS_00        0xb7
#define   MAX77779_CHGR_REG_CHG_DETAILS_00_CHGIN_DTLS  GENMASK(6, 5)
#define MAX77779_CHGR_REG_CHG_DETAILS_01        0xb8
#define   MAX77779_CHGR_REG_CHG_DETAILS_01_CHG_DTLS    GENMASK(3, 0)
#define   MAX77779_CHGR_REG_CHG_DETAILS_01_BAT_DTLS    GENMASK(6, 4)
#define MAX77779_CHGR_REG_CHG_DETAILS_02        0xb9
#define   MAX77779_CHGR_REG_CHG_DETAILS_02_CHGIN_STS   BIT(5)
#define MAX77779_CHGR_REG_CHG_DETAILS_03        0xba
#define MAX77779_CHGR_REG_CHG_DETAILS_04        0xbb
#define   MAX77779_CHGR_REG_CHG_DETAILS_04_FSHIP_EXIT_DTLS GENMASK(1, 0)
#define MAX77779_CHGR_REG_CHG_CNFG_00           0xbc
#define   MAX77779_CHGR_REG_CHG_CNFG_00_MODE    GENMASK(3, 0)
#define MAX77779_CHGR_REG_CHG_CNFG_01           0xbd
#define MAX77779_CHGR_REG_CHG_CNFG_02           0xbe
#define   MAX77779_CHGR_REG_CHG_CNFG_02_CHGCC   GENMASK(5, 0)
#define MAX77779_CHGR_REG_CHG_CNFG_03           0xbf
#define MAX77779_CHGR_REG_CHG_CNFG_04           0xc0
#define   MAX77779_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM     GENMASK(5, 0)
#define MAX77779_CHGR_REG_CHG_CNFG_05           0xc1
#define MAX77779_CHGR_REG_CHG_CNFG_06           0xc2
#define   MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT GENMASK(3, 2)
#define MAX77779_CHGR_REG_CHG_CNFG_07           0xc3
#define MAX77779_CHGR_REG_CHG_CNFG_08           0xc4
#define MAX77779_CHGR_REG_CHG_CNFG_09           0xc5
#define   MAX77779_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM     GENMASK(6, 0)
#define   MAX77779_CHGR_REG_CHG_CNFG_09_NO_AUTOIBUS    BIT(7)
#define MAX77779_CHGR_REG_CHG_CNFG_10           0xc6
#define MAX77779_CHGR_REG_CHG_CNFG_11           0xc7
#define MAX77779_CHGR_REG_CHG_CNFG_12           0xc8
#define   MAX77779_CHGR_REG_CHG_CNFG_12_CHGINSEL       BIT(5)
#define   MAX77779_CHGR_REG_CHG_CNFG_12_WCINSEL        BIT(6)
#define   MAX77779_CHGR_REG_CHG_CNFG_12_CHG_EN         BIT(7)
#define MAX77779_CHGR_REG_CHG_CNFG_13           0xc9
#define MAX77779_CHGR_REG_CHG_CNFG_14           0xca
#define MAX77779_CHGR_REG_CHG_CNFG_15           0xcb
#define MAX77779_CHGR_REG_CHG_CNFG_16           0xcc
#define MAX77779_CHGR_REG_CHG_CNFG_17           0xcd

/* Charger MODE field (CHG_CNFG_00.MODE) values */
enum max77779_chgr_mode {
	MAX77779_CHGR_MODE_ALL_OFF		= 0x0,
	MAX77779_CHGR_MODE_BUCK_ON		= 0x4,
	MAX77779_CHGR_MODE_CHGR_BUCK_ON		= 0x5,
	MAX77779_CHGR_MODE_BOOST_UNO_ON		= 0x8,
	MAX77779_CHGR_MODE_BOOST_ON		= 0x9,
	MAX77779_CHGR_MODE_OTG_BOOST_ON		= 0xa,
};

/*
 * Scratchpad sub-block. Unlike the others it sits behind its own I2C address
 * and is addressed as 16-bit registers, so one MAX77779_SP_REG_DATA register
 * covers two scratchpad bytes. PAGE_CTRL selects which 256-byte page the data
 * window maps; the battery-backed values the bootloader looks at all live in
 * page 0.
 */
#define MAX77779_SP_REG_PAGE_CTRL               0x70
#define MAX77779_SP_REG_DATA                    0x80
#define MAX77779_SP_REG_MAX                     0xff
#define MAX77779_SP_PAGE_SIZE                   256

/* MaxQ opcodes for max77779_maxq_command() */
#define MAX77779_MAXQ_CMD_MAXLENGTH (MAX77779_MAXQ_REG_AP_DATAOUT_OPCODE - \
				     MAX77779_MAXQ_REG_AP_DATAOUT1 + \
				     1)
#define MAX77779_MAXQ_RSP_MAXLENGTH (MAX77779_MAXQ_REG_AP_DATAIN10 - \
				     MAX77779_MAXQ_REG_AP_DATAIN0 + \
				     1)

#define MAX77779_MAXQ_OPCODE_GPIO_TRIGGER_READ   0x21
#define MAX77779_MAXQ_OPCODE_GPIO_TRIGGER_WRITE  0x22
#define MAX77779_MAXQ_OPCODE_GPIO_CONTROL_READ   0x23
#define MAX77779_MAXQ_OPCODE_GPIO_CONTROL_WRITE  0x24
#define MAX77779_MAXQ_OPCODE_USER_SPACE_READ     0x81
#define MAX77779_MAXQ_OPCODE_USER_SPACE_WRITE    0x82

/**
 * struct max77779 - core max77779 internal data structure
 *
 * @regmap_top: Regmap for accessing TOP registers
 * @maxq_lock: Lock for serializing access to MaxQ
 * @regmap_maxq: Regmap for accessing MaxQ registers
 * @cmd_done: Used to signal completion of a MaxQ command
 * @regmap_charger: Regmap for accessing charger registers
 *
 * The MAX77779 comprises several sub-blocks, namely TOP, MaxQ, Charger,
 * Fuel Gauge, and TCPCI.
 */
struct max77779 {
	struct regmap *regmap_top;

	/* This protects MaxQ commands - only one can be active */
	struct mutex maxq_lock;
	struct regmap *regmap_maxq;
	struct completion cmd_done;

	struct regmap *regmap_charger;
};

/**
 * struct max77779_maxq_command - structure containing the MaxQ command to
 * send
 *
 * @length: The number of bytes to send.
 * @cmd: The data to send.
 */
struct max77779_maxq_command {
	u8 length;
	u8 cmd[] __counted_by(length);
};

/**
 * struct max77779_maxq_response - structure containing the MaxQ response
 *
 * @length: The number of bytes to receive.
 * @rsp: The data received. Must have at least @length bytes space.
 */
struct max77779_maxq_response {
	u8 length;
	u8 rsp[] __counted_by(length);
};

/**
 * max77779_maxq_command() - issue a MaxQ command and wait for the response
 * and associated data
 *
 * @max77779: The core max77779 device handle.
 * @cmd: The command to be sent.
 * @rsp: Any response data associated with the command will be copied here;
 *     can be %NULL if the command has no response (other than ACK).
 *
 * Return: 0 on success, a negative error number otherwise.
 */
int max77779_maxq_command(struct max77779 *max77779,
			  const struct max77779_maxq_command *cmd,
			  struct max77779_maxq_response *rsp);

#endif /* __LINUX_MFD_MAX77779_H */
