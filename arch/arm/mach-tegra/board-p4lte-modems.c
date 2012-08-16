/* linux/arch/arm/mach-xxxx/board-tuna-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_data/modem.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/gpio-sec.h>
#include "board-p4lte.h"

#define S5P_MEMREG(x)	(S5P_VA_SROMC + (x))

#define S5P_SROM_BW	S5P_MEMREG(0x00)
#define S5P_SROM_BC0	S5P_MEMREG(0x04)
#define S5P_SROM_BC1	S5P_MEMREG(0x08)
#define S5P_SROM_BC2	S5P_MEMREG(0x0C)
#define S5P_SROM_BC3	S5P_MEMREG(0x10)

#define DPRAM_START_ADDRESS	0xD0000000
#define DPRAM_SIZE				0x4000
#define CBP_EDPRAM_SIZE		0x4000
#define DPRAM_END_ADDRESS	(DPRAM_START_ADDRESS + DPRAM_SIZE - 1)

#define INT_MASK_REQ_ACK_F	0x0020
#define INT_MASK_REQ_ACK_R	0x0010
#define INT_MASK_RES_ACK_F	0x0008
#define INT_MASK_RES_ACK_R	0x0004
#define INT_MASK_SEND_F	0x0002
#define INT_MASK_SEND_R	0x0001

#define INT_MASK_REQ_ACK_RFS	0x0400 /* Request RES_ACK_RFS		*/
#define INT_MASK_RES_ACK_RFS	0x0200 /* Response of REQ_ACK_RFS	*/
#define INT_MASK_SEND_RFS		0x0100 /* Indicate sending RFS data */
#define SROM_CS0_BASE			0x04000000
#define SROM_WIDTH				0x01000000
#define SROM_NUM_ADDR_BITS		14
#define MAGIC_DMDL				0x4445444C

extern void tegra_init_snor();

/* For "bus width and wait control (BW)" register */
enum sromc_attr {
	SROMC_DATA_16   = 0x1,	/* 16-bit data bus	*/
	SROMC_BYTE_ADDR = 0x2,	/* Byte base address	*/
	SROMC_WAIT_EN   = 0x4,	/* Wait enabled		*/
	SROMC_BYTE_EN   = 0x8,	/* Byte access enabled	*/
	SROMC_MASK      = 0xF
};

/* DPRAM configuration */
struct sromc_cfg {
	enum sromc_attr attr;
	unsigned size;
	unsigned csn;		/* CSn #			*/
	unsigned addr;		/* Start address (physical)	*/
	unsigned end;		/* End address (physical)	*/
};

/* DPRAM access timing configuration */
struct sromc_access_cfg {
	u32 tacs;		/* Address set-up before CSn		*/
	u32 tcos;		/* Chip selection set-up before OEn	*/
	u32 tacc;		/* Access cycle				*/
	u32 tcoh;		/* Chip selection hold on OEn		*/
	u32 tcah;		/* Address holding time after CSn	*/
	u32 tacp;		/* Page mode access cycle at Page mode	*/
	u32 pmc;		/* Page Mode config			*/
};

#define IRQ_DPRAM_INT_N	gpio_to_irq(GPIO_DP_INT_AP)
#define IRQ_PHONE_ACTIVE	gpio_to_irq(GPIO_PHONE_ACTIVE)
#define GPIO_PHONE_ON		GPIO_CP_ON
#define GPIO_DPRAM_INT_N	GPIO_DP_INT_AP
#define GPIO_PHONE_RST_N	GPIO_CP_RST

static struct sromc_cfg cbp_edpram_cfg = {
	.attr = SROMC_DATA_16 | SROMC_BYTE_EN,
	.size = CBP_EDPRAM_SIZE,
};

static struct sromc_access_cfg cbp_edpram_access_cfg[] = {
	[DPRAM_SPEED_LOW] = {
		.tacs = 0x00 << 28,
		.tcos = 0x00 << 24,
		.tacc = 0x0F << 16,
		.tcoh = 0x00 << 12,
		.tcah = 0x00 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
};

#define CBP_DP_FMT_TX_BUFF_SZ	2044
#define CBP_DP_RAW_TX_BUFF_SZ	6128
#define CBP_DP_FMT_RX_BUFF_SZ	2044
#define CBP_DP_RAW_RX_BUFF_SZ	6128

#define MAX_CBP_EDPRAM_IPC_DEV	(IPC_RAW + 1)	/* FMT, RAW */

struct cbp_edpram_ipc_cfg {
	u16 magic;
	u16 access;

	u16 fmt_tx_head;
	u16 fmt_tx_tail;
	u8  fmt_tx_buff[CBP_DP_FMT_TX_BUFF_SZ];

	u16 raw_tx_head;
	u16 raw_tx_tail;
	u8  raw_tx_buff[CBP_DP_RAW_TX_BUFF_SZ];

	u16 fmt_rx_head;
	u16 fmt_rx_tail;
	u8  fmt_rx_buff[CBP_DP_FMT_RX_BUFF_SZ];

	u16 raw_rx_head;
	u16 raw_rx_tail;
	u8  raw_rx_buff[CBP_DP_RAW_RX_BUFF_SZ];

	u8  padding[16];
	u16 mbx_cp2ap;
	u16 mbx_ap2cp;
};

struct cbp_edpram_circ {
	u16 __iomem *head;
	u16 __iomem *tail;
	u8  __iomem *buff;
	u32          size;
};

struct cbp_edpram_ipc_device {
	char name[16];
	int  id;

	struct cbp_edpram_circ txq;
	struct cbp_edpram_circ rxq;

	u16 mask_req_ack;
	u16 mask_res_ack;
	u16 mask_send;
};

struct cbp_edpram_ipc_map {
	u16 __iomem *magic;
	u16 __iomem *access;

	struct cbp_edpram_ipc_device dev[MAX_CBP_EDPRAM_IPC_DEV];

	u16 __iomem *mbx_ap2cp;
	u16 __iomem *mbx_cp2ap;
};

static int cbp_edpram_ota_reset(void);
static struct cbp_edpram_ipc_map cbp_ipc_map;
static void setup_dpram_speed(unsigned csn, struct sromc_access_cfg *acc_cfg);
static void cbp_edpram_reset(void);
static void cbp_edpram_clr_intr(void);
static u16  cbp_edpram_recv_intr(void);
static void cbp_edpram_send_intr(u16 irq_mask);
static u16  cbp_edpram_recv_msg(void);
static void cbp_edpram_send_msg(u16 msg);

static u16  cbp_edpram_get_magic(void);
static void cbp_edpram_set_magic(u16 value);
static u16  cbp_edpram_get_access(void);
static void cbp_edpram_set_access(u16 value);

static u32  cbp_edpram_get_tx_head(int dev_id);
static u32  cbp_edpram_get_tx_tail(int dev_id);
static void cbp_edpram_set_tx_head(int dev_id, u32 head);
static void cbp_edpram_set_tx_tail(int dev_id, u32 tail);
static u8 __iomem *cbp_edpram_get_tx_buff(int dev_id);
static u32  cbp_edpram_get_tx_buff_size(int dev_id);

static u32  cbp_edpram_get_rx_head(int dev_id);
static u32  cbp_edpram_get_rx_tail(int dev_id);
static void cbp_edpram_set_rx_head(int dev_id, u32 head);
static void cbp_edpram_set_rx_tail(int dev_id, u32 tail);
static u8 __iomem *cbp_edpram_get_rx_buff(int dev_id);
static u32  cbp_edpram_get_rx_buff_size(int dev_id);

static u16  cbp_edpram_get_mask_req_ack(int dev_id);
static u16  cbp_edpram_get_mask_res_ack(int dev_id);
static u16  cbp_edpram_get_mask_send(int dev_id);

static struct modemlink_dpram_control cbp_edpram_ctrl = {
	.reset      = cbp_edpram_reset,

	.clear_intr = cbp_edpram_clr_intr,
	.recv_intr  = cbp_edpram_recv_intr,
	.send_intr  = cbp_edpram_send_intr,
	.recv_msg   = cbp_edpram_recv_msg,
	.send_msg   = cbp_edpram_send_msg,

	.get_magic  = cbp_edpram_get_magic,
	.set_magic  = cbp_edpram_set_magic,
	.get_access = cbp_edpram_get_access,
	.set_access = cbp_edpram_set_access,

	.get_tx_head = cbp_edpram_get_tx_head,
	.get_tx_tail = cbp_edpram_get_tx_tail,
	.set_tx_head = cbp_edpram_set_tx_head,
	.set_tx_tail = cbp_edpram_set_tx_tail,
	.get_tx_buff = cbp_edpram_get_tx_buff,
	.get_tx_buff_size = cbp_edpram_get_tx_buff_size,

	.get_rx_head = cbp_edpram_get_rx_head,
	.get_rx_tail = cbp_edpram_get_rx_tail,
	.set_rx_head = cbp_edpram_set_rx_head,
	.set_rx_tail = cbp_edpram_set_rx_tail,
	.get_rx_buff = cbp_edpram_get_rx_buff,
	.get_rx_buff_size = cbp_edpram_get_rx_buff_size,

	.get_mask_req_ack = cbp_edpram_get_mask_req_ack,
	.get_mask_res_ack = cbp_edpram_get_mask_res_ack,
	.get_mask_send    = cbp_edpram_get_mask_send,
	.ota_reset = cbp_edpram_ota_reset,

	.dp_base = NULL,
	.dp_size = 0,
	.dp_type = EXT_DPRAM,

	.dpram_irq        = TEGRA_GPIO_TO_IRQ(GPIO_DP_INT_AP),
	.dpram_irq_flags  = IRQF_DISABLED,
	.dpram_irq_name   = "CBP71_EDPRAM_IRQ",
	.dpram_wlock_name = "CBP71_EDPRAM_WLOCK",

	.max_ipc_dev = MAX_CBP_EDPRAM_IPC_DEV,
};

/*
** CDMA target platform data
*/
static struct modem_io_t cdma_cbp71_io_devices[] = {
	[0] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[1] = {
		.name = "cdma_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[2] = {
		.name = "cdma_rfs0",
		.id = 0x33,		/* 0x13 (ch.id) | 0x20 (mask) */
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[3] = {
		.name = "cdma_boot0",
		.id = 0x1,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[4] = {
		.name = "cdma_rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[5] = {
		.name = "cdma_rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[6] = {
		.name = "cdma_rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[7] = {
		.name = "cdma_rmnet3",
		.id = 0x2D,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[8] = {
		.name = "cdma_rmnet4",
		.id = 0x27,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[9] = {
		.name = "cdma_rmnet5", /* DM Port IO device */
		.id = 0x3A,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[10] = {
		.name = "cdma_rmnet6", /* AT CMD IO device */
		.id = 0x31,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[11] = {
		.name = "cdma_ramdump0",
		.id = 0x1,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[12] = {
		.name = "cdma_cplog", /* cp log io-device */
		.id = 0x3D,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},

};

/* cdma target platform data */
static struct modem_data cdma_cbp71_modem_data = {
	.name = "cbp7.1",

	/* ToDo: always power on vbat 3.3v it is not cennected GPIO */
	.gpio_cp_on = GPIO_PHONE_ON,
	.gpio_cp_off       = GPIO_VIA_PS_HOLD_OFF,
	.gpio_reset_req_n =  0,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_cp_dump_int = 0, /*ToDo:*/
	.gpio_cp_warm_reset = 0,

	.modem_type = VIA_CBP71,
	.link_types = (1 << LINKDEV_DPRAM),
	.modem_net = CDMA_NETWORK,
	.link_name  = "cbp71_edpram",
	.dpram_ctl = &cbp_edpram_ctrl,
	.num_iodevs = ARRAY_SIZE(cdma_cbp71_io_devices),
	.iodevs = cdma_cbp71_io_devices,
	.use_handover = true,
};

static struct resource cdma_cbp71_modem_res[] = {
	[0] = {
		.name = "dpram",
		.start = DPRAM_START_ADDRESS,
		.end = DPRAM_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "dpram_irq",
		.start = TEGRA_GPIO_TO_IRQ(GPIO_DP_INT_AP),
		.end = TEGRA_GPIO_TO_IRQ(GPIO_DP_INT_AP),
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "cp_active_irq",
		.start = TEGRA_GPIO_TO_IRQ(GPIO_PHONE_ACTIVE),
		.end = TEGRA_GPIO_TO_IRQ(GPIO_PHONE_ACTIVE),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device cdma_cbp71_modem = {
	.name = "modem_if",
	.id = 2,
	.num_resources = ARRAY_SIZE(cdma_cbp71_modem_res),
	.resource = cdma_cbp71_modem_res,
	.dev = {
		.platform_data = &cdma_cbp71_modem_data,
	},
};

static int cbp_edpram_ota_reset(void)
{
	unsigned gpio_cp_rst = cdma_cbp71_modem_data.gpio_cp_reset;
	unsigned gpio_cp_on = cdma_cbp71_modem_data.gpio_cp_on;
	unsigned int *magickey_va;
	int i;

	pr_err("[MODEM_IF] %s Modem OTA reset\n", __func__);
	magickey_va = ioremap_nocache(DPRAM_START_ADDRESS, \
				 sizeof(unsigned int));
	if (!magickey_va) {
		pr_err("%s: ioremap fail\n", __func__);
		return -ENOMEM;
	}

	gpio_set_value(gpio_cp_on, 1);
	msleep(100);
	gpio_set_value(gpio_cp_rst, 0);

	for (i = 0; i < 3; i++) {
		*magickey_va = MAGIC_DMDL;
		if (*magickey_va == MAGIC_DMDL) {
			pr_err("magic key is ok!");
			break;
		}
	}

	msleep(500);
	gpio_set_value(gpio_cp_rst, 1);
	for (i = 0; i < 3; i++) {
		*magickey_va = MAGIC_DMDL;
		if (*magickey_va == MAGIC_DMDL) {
			pr_err("magic key is ok!");
			break;
		}
	}

	iounmap(magickey_va);

	return 0;
}

static void cbp_edpram_reset(void)
{
	return;
}

static void cbp_edpram_clr_intr(void)
{
	ioread16(cbp_ipc_map.mbx_cp2ap);
}

static u16 cbp_edpram_recv_intr(void)
{
	return ioread16(cbp_ipc_map.mbx_cp2ap);
}

static void cbp_edpram_send_intr(u16 irq_mask)
{
	iowrite16(irq_mask, cbp_ipc_map.mbx_ap2cp);
}

static u16 cbp_edpram_recv_msg(void)
{
	return ioread16(cbp_ipc_map.mbx_cp2ap);
}

static void cbp_edpram_send_msg(u16 msg)
{
	iowrite16(msg, cbp_ipc_map.mbx_ap2cp);
}

static u16 cbp_edpram_get_magic(void)
{
	return ioread16(cbp_ipc_map.magic);
}

static void cbp_edpram_set_magic(u16 value)
{
	iowrite16(value, cbp_ipc_map.magic);
}

static u16 cbp_edpram_get_access(void)
{
	return ioread16(cbp_ipc_map.access);
}

static void cbp_edpram_set_access(u16 value)
{
	iowrite16(value, cbp_ipc_map.access);
}

static u32 cbp_edpram_get_tx_head(int dev_id)
{
	return ioread16(cbp_ipc_map.dev[dev_id].txq.head);
}

static u32 cbp_edpram_get_tx_tail(int dev_id)
{
	return ioread16(cbp_ipc_map.dev[dev_id].txq.tail);
}

static void cbp_edpram_set_tx_head(int dev_id, u32 head)
{
	iowrite16((u16)head, cbp_ipc_map.dev[dev_id].txq.head);
}

static void cbp_edpram_set_tx_tail(int dev_id, u32 tail)
{
	iowrite16((u16)tail, cbp_ipc_map.dev[dev_id].txq.tail);
}

static u8 __iomem *cbp_edpram_get_tx_buff(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].txq.buff;
}

static u32 cbp_edpram_get_tx_buff_size(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].txq.size;
}

static u32 cbp_edpram_get_rx_head(int dev_id)
{
	return ioread16(cbp_ipc_map.dev[dev_id].rxq.head);
}

static u32 cbp_edpram_get_rx_tail(int dev_id)
{
	return ioread16(cbp_ipc_map.dev[dev_id].rxq.tail);
}

static void cbp_edpram_set_rx_head(int dev_id, u32 head)
{
	return iowrite16((u16)head, cbp_ipc_map.dev[dev_id].rxq.head);
}

static void cbp_edpram_set_rx_tail(int dev_id, u32 tail)
{
	return iowrite16((u16)tail, cbp_ipc_map.dev[dev_id].rxq.tail);
}

static u8 __iomem *cbp_edpram_get_rx_buff(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].rxq.buff;
}

static u32 cbp_edpram_get_rx_buff_size(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].rxq.size;
}

static u16 cbp_edpram_get_mask_req_ack(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].mask_req_ack;
}

static u16 cbp_edpram_get_mask_res_ack(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].mask_res_ack;
}

static u16 cbp_edpram_get_mask_send(int dev_id)
{
	return cbp_ipc_map.dev[dev_id].mask_send;
}

/* Set dynamic environment for a modem */
static void setup_cdma_modem_env(void)
{
	/*
	** Config DPRAM control structure
	*/
	if (system_rev == 1)
		cbp_edpram_cfg.csn = 1;
	else
		cbp_edpram_cfg.csn = 0;

	cbp_edpram_cfg.addr = DPRAM_START_ADDRESS;
	cbp_edpram_cfg.end  = DPRAM_END_ADDRESS;

	if (system_rev == 1)
		cbp_edpram_ctrl.dpram_irq = IRQ_DPRAM_INT_N;
}
static u8 *cbp_edpram_remap_mem_region(struct sromc_cfg *cfg)
{
	int			      dp_addr = 0;
	int			      dp_size = 0;
	u8 __iomem                   *dp_base = NULL;
	struct cbp_edpram_ipc_cfg    *ipc_map = NULL;
	struct cbp_edpram_ipc_device *dev = NULL;

	dp_addr = cfg->addr;
	dp_size = cfg->size;
	dp_base = (u8 *)ioremap_nocache(dp_addr, dp_size);
	if (!dp_base) {
		pr_err("[MDM] <%s> dpram base ioremap fail\n", __func__);
		return NULL;
	}
	pr_info("[MDM] <%s> DPRAM VA=0x%08X\n", __func__, (int)dp_base);

	cbp_edpram_ctrl.dp_base = (u8 __iomem *)dp_base;
	cbp_edpram_ctrl.dp_size = dp_size;

	/* Map for IPC */
	ipc_map = (struct cbp_edpram_ipc_cfg *)dp_base;

	/* Magic code and access enable fields */
	cbp_ipc_map.magic  = (u16 __iomem *)&ipc_map->magic;
	cbp_ipc_map.access = (u16 __iomem *)&ipc_map->access;

	/* FMT */
	dev = &cbp_ipc_map.dev[IPC_FMT];

	strcpy(dev->name, "FMT");
	dev->id = IPC_FMT;

	dev->txq.head = (u16 __iomem *)&ipc_map->fmt_tx_head;
	dev->txq.tail = (u16 __iomem *)&ipc_map->fmt_tx_tail;
	dev->txq.buff = (u8 __iomem *)&ipc_map->fmt_tx_buff[0];
	dev->txq.size = CBP_DP_FMT_TX_BUFF_SZ;

	dev->rxq.head = (u16 __iomem *)&ipc_map->fmt_rx_head;
	dev->rxq.tail = (u16 __iomem *)&ipc_map->fmt_rx_tail;
	dev->rxq.buff = (u8 __iomem *)&ipc_map->fmt_rx_buff[0];
	dev->rxq.size = CBP_DP_FMT_RX_BUFF_SZ;

	dev->mask_req_ack = INT_MASK_REQ_ACK_F;
	dev->mask_res_ack = INT_MASK_RES_ACK_F;
	dev->mask_send    = INT_MASK_SEND_F;

	/* RAW */
	dev = &cbp_ipc_map.dev[IPC_RAW];

	strcpy(dev->name, "RAW");
	dev->id = IPC_RAW;

	dev->txq.head = (u16 __iomem *)&ipc_map->raw_tx_head;
	dev->txq.tail = (u16 __iomem *)&ipc_map->raw_tx_tail;
	dev->txq.buff = (u8 __iomem *)&ipc_map->raw_tx_buff[0];
	dev->txq.size = CBP_DP_RAW_TX_BUFF_SZ;

	dev->rxq.head = (u16 __iomem *)&ipc_map->raw_rx_head;
	dev->rxq.tail = (u16 __iomem *)&ipc_map->raw_rx_tail;
	dev->rxq.buff = (u8 __iomem *)&ipc_map->raw_rx_buff[0];
	dev->rxq.size = CBP_DP_RAW_RX_BUFF_SZ;

	dev->mask_req_ack = INT_MASK_REQ_ACK_R;
	dev->mask_res_ack = INT_MASK_RES_ACK_R;
	dev->mask_send    = INT_MASK_SEND_R;

	/* Mailboxes */
	cbp_ipc_map.mbx_ap2cp = (u16 __iomem *)&ipc_map->mbx_ap2cp;
	cbp_ipc_map.mbx_cp2ap = (u16 __iomem *)&ipc_map->mbx_cp2ap;

	return dp_base;
}

static void init_hw_setting_p4(void)
{
	u32 reg;
    /* initial pin settings - dpram driver control */
	gpio_request(GPIO_DP_INT_AP, "dpram/IRQ_DPRAM_INT_N");
	gpio_direction_input(GPIO_DP_INT_AP);
	tegra_gpio_enable(GPIO_DP_INT_AP);
	irq_set_irq_type(IRQ_DPRAM_INT_N, IRQ_TYPE_EDGE_FALLING);

	gpio_request(GPIO_PHONE_ACTIVE, "dpram/IRQ_PHONE_ACTIVE");
	gpio_direction_input(GPIO_PHONE_ACTIVE);
	tegra_gpio_enable(GPIO_PHONE_ACTIVE);
	irq_set_irq_type(IRQ_PHONE_ACTIVE, IRQ_TYPE_EDGE_BOTH);

	if (system_rev > 0x0A) {
		if (gpio_is_valid(GPIO_PHONE_ON)) {
			if (gpio_request(GPIO_PHONE_ON, "dpram/GPIO_PHONE_ON"))
				printk(KERN_ERR "request fail GPIO_PHONE_ON\n");
			gpio_direction_output(GPIO_PHONE_ON, GPIO_LEVEL_LOW);
		}
		gpio_set_value(GPIO_PHONE_ON, GPIO_LEVEL_LOW);
	} else {
		if (gpio_is_valid(GPIO_CP_ON_REV05)) {
			if (gpio_request(GPIO_CP_ON_REV05,
					"dpram/GPIO_PHONE_ON"))
				printk(KERN_ERR "request fail GPIO_PHONE_ON\n");
			gpio_direction_output(GPIO_CP_ON_REV05, GPIO_LEVEL_LOW);
		}
		gpio_set_value(GPIO_CP_ON_REV05, GPIO_LEVEL_LOW);
	}

	if (gpio_is_valid(GPIO_PHONE_RST_N)) {
		if (gpio_request(GPIO_PHONE_RST_N, "dpram/GPIO_PHONE_RST_N"))
			printk(KERN_ERR "request fail GPIO_PHONE_RST_N\n");
		reg = readl(IO_ADDRESS(0x6000d108));
		writel(reg | (0x01 << 6), IO_ADDRESS(0x6000d108));
		gpio_direction_output(GPIO_PHONE_RST_N, GPIO_LEVEL_LOW);
	}

	if (gpio_is_valid(GPIO_VIA_PS_HOLD_OFF)) {
		if (gpio_request(GPIO_VIA_PS_HOLD_OFF,
					"dpram/GPIO_VIA_PS_HOLD_OFF"))
			printk(KERN_ERR "request fail GPIO_VIA_PS_HOLD_OFF\n");
		reg = readl(IO_ADDRESS(0x6000d184));
		writel(reg | (0x1 << 5), IO_ADDRESS(0x6000d184));
		gpio_direction_output(GPIO_VIA_PS_HOLD_OFF, GPIO_LEVEL_HIGH);
		tegra_gpio_enable(GPIO_VIA_PS_HOLD_OFF);
	}

	tegra_init_snor();
}
static void init_sromc(void)
{
	struct clk *clk = NULL;

	/* SROMC clk enable */
	clk = clk_get(NULL, "sromc");
	if (!clk) {
		pr_err("[MDM/E] <%s> SROMC clock gate fail\n", __func__);
		return;
	}
	clk_enable(clk);
}

static struct sromc_cfg cmc_idpram_cfg = {
	.attr = SROMC_DATA_16,
	.size = 0,
};

static struct sromc_access_cfg cmc_idpram_access_cfg[] = {
	[DPRAM_SPEED_LOW] = {
		.tacs = 0x01 << 28,
		.tcos = 0x01 << 24,
		.tacc = 0x1B << 16,
		.tcoh = 0x01 << 12,
		.tcah = 0x01 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
	[DPRAM_SPEED_HIGH] = {
		.tacs = 0x01 << 28,
		.tcos = 0x01 << 24,
		.tacc = 0x0B << 16,
		.tcoh = 0x01 << 12,
		.tcah = 0x01 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
};

#define DP_FMT_TX_BUFF_SZ	2040
#define DP_RAW_TX_BUFF_SZ	4088
#define DP_RFS_TX_BUFF_SZ	1016
#define DP_FMT_RX_BUFF_SZ	2040
#define DP_RAW_RX_BUFF_SZ	5112
#define DP_RFS_RX_BUFF_SZ	2036

#define MAX_CMC_IDPRAM_IPC_DEV	(IPC_RFS + 1)	/* FMT, RAW, RFS */

struct dpram_ipc_cfg {
	u16 magic;
	u16 access;

	u32 fmt_tx_head;
	u32 fmt_tx_tail;
	u8  fmt_tx_buff[DP_FMT_TX_BUFF_SZ];

	u32 raw_tx_head;
	u32 raw_tx_tail;
	u8  raw_tx_buff[DP_RAW_TX_BUFF_SZ];

	u32 rfs_tx_head;
	u32 rfs_tx_tail;
	u8  rfs_tx_buff[DP_RFS_TX_BUFF_SZ];

	u32 fmt_rx_head;
	u32 fmt_rx_tail;
	u8  fmt_rx_buff[DP_FMT_RX_BUFF_SZ];

	u32 raw_rx_head;
	u32 raw_rx_tail;
	u8  raw_rx_buff[DP_RAW_RX_BUFF_SZ];

	u32 rfs_rx_head;
	u32 rfs_rx_tail;
	u8  rfs_rx_buff[DP_RFS_RX_BUFF_SZ];
};

struct dpram_circ {
	u32 __iomem *head;
	u32 __iomem *tail;
	u8  __iomem *buff;
	u32          size;
};

struct dpram_ipc_device {
	char name[16];
	int  id;

	struct dpram_circ txq;
	struct dpram_circ rxq;

	u16 mask_req_ack;
	u16 mask_res_ack;
	u16 mask_send;
};

struct dpram_ipc_map {
	u16 __iomem *magic;
	u16 __iomem *access;

	struct dpram_ipc_device dev[MAX_CMC_IDPRAM_IPC_DEV];
};

static struct modem_io_t lte_io_devices[] = {
	[0] = {
		.name = "lte_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[1] = {
		.name = "lte_rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[2] = {
		.name = "lte_rfs0",
		.id = 0x0,
		.format = IPC_RFS,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[3] = {
		.name = "lte_boot0",
		.id = 0x0,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[4] = {
		.name = "lte_rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[5] = {
		.name = "lte_rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[6] = {
		.name = "lte_rmnet3",
		.id = 0x2D,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[7] = {
		.name = "lte_multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[8] = {
		.name = "lte_rmnet4", /* DM Port io-device */
		.id = 0x3F,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
	[9] = {
		.name = "lte_ramdump0",
		.id = 0x0,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
};

static struct modemlink_pm_data lte_link_pm_data = {
	.name = "lte_link_pm",

	.gpio_link_enable = 0,
	.gpio_link_active  = GPIO_AP2LTE_STATUS,
	.gpio_link_hostwake = GPIO_LTE2AP_WAKEUP,
	.gpio_link_slavewake = GPIO_AP2LTE_WAKEUP,

	/*
	.port_enable = host_port_enable,
	.freqlock = ATOMIC_INIT(0),
	.cpufreq_lock = exynos_cpu_frequency_lock,
	.cpufreq_unlock = exynos_cpu_frequency_unlock,
	*/
};

static struct modem_data lte_modem_data = {
	.name = "cmc220",

	.gpio_cp_on = GPIO_220_PMIC_PWRON,
	.gpio_reset_req_n = 0,
	.gpio_cp_reset = GPIO_CMC_RST,
	.gpio_pda_active = 0,/*NOT YET CONNECTED*/
	.gpio_phone_active = GPIO_LTE_ACTIVE,
	.gpio_cp_dump_int = GPIO_LTE_ACTIVE,/*TO BE CHECKED*/

	.gpio_cp_warm_reset = 0,
	.gpio_cp_off = GPIO_220_PMIC_PWRHOLD_OFF,
#ifdef CONFIG_LTE_MODEM_CMC220
	.gpio_slave_wakeup = GPIO_AP2LTE_WAKEUP,
	.gpio_host_wakeup = GPIO_LTE2AP_WAKEUP,
	.gpio_host_active = GPIO_AP2LTE_STATUS,
#endif
	.modem_type = SEC_CMC220,
	.link_types = LINKTYPE(LINKDEV_USB),
	.modem_net = LTE_NETWORK,
	.num_iodevs = ARRAY_SIZE(lte_io_devices),
	.iodevs = lte_io_devices,
	.link_pm_data = &lte_link_pm_data,
	.use_handover = true,
};

static void lte_modem_cfg_gpio(void)
{
	unsigned gpio_cp_on = lte_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = lte_modem_data.gpio_cp_reset;
	unsigned gpio_phone_active = lte_modem_data.gpio_phone_active;
	unsigned gpio_cp_off = lte_modem_data.gpio_cp_off;
	unsigned gpio_slave_wakeup = lte_modem_data.gpio_slave_wakeup;
	unsigned gpio_host_wakeup = lte_modem_data.gpio_host_wakeup;
	unsigned gpio_host_active = lte_modem_data.gpio_host_active;

	if (gpio_cp_on) {
		gpio_request(gpio_cp_on, "LTE_ON");
		gpio_direction_output(gpio_cp_on, 0);
	}

	if (gpio_cp_rst) {
		gpio_request(gpio_cp_rst, "LTE_RST");
		gpio_direction_output(gpio_cp_rst, 0);
	}

	if (gpio_phone_active) {
		gpio_request(gpio_phone_active, "LTE_ACTIVE");
		gpio_direction_input(gpio_phone_active);
	}

	if (gpio_cp_off) {
		gpio_request(gpio_cp_off, "LTE_OFF");
		gpio_direction_output(gpio_cp_off, 1);
	}

	if (gpio_slave_wakeup) {
		gpio_request(gpio_slave_wakeup, "LTE_SLAVE_WAKEUP");
		gpio_direction_output(gpio_slave_wakeup, 0);
	}

	if (gpio_host_wakeup) {
		gpio_request(gpio_host_wakeup, "LTE_HOST_WAKEUP");
		gpio_direction_input(gpio_host_wakeup);
	}

	if (gpio_host_active) {
		gpio_request(gpio_host_active, "LTE_HOST_ACTIVE");
		gpio_direction_output(gpio_host_active, 1);
	}
}

void set_host_states(int type)
{
	int spin = 20;

	pr_info("%s(%d)\n", __func__, type);

	if (!type) {
		gpio_direction_output(lte_modem_data.gpio_host_active, type);
		return;
	}

	if (gpio_get_value(lte_modem_data.gpio_host_wakeup)) {
		gpio_direction_output(lte_modem_data.gpio_host_active, type);
		do {
			msleep(10);
			if (!gpio_get_value(lte_modem_data.gpio_host_wakeup))
				break;
		} while (spin--);
	} else
		pr_err("mif: host wakeup is low\n");
}

int get_cp_active_state(void)
{
	return gpio_get_value(lte_modem_data.gpio_phone_active);
}

static struct resource lte_modem_res[] = {
	[0] = {
		.name = "lte_phone_active",
		/* phone active irq */
		.start = TEGRA_GPIO_TO_IRQ(GPIO_LTE_ACTIVE),
		.end = TEGRA_GPIO_TO_IRQ(GPIO_LTE_ACTIVE),
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.name = "lte_host_wakeup",
		/* host wakeup irq */
		.start = TEGRA_GPIO_TO_IRQ(GPIO_LTE2AP_WAKEUP),
		.end = TEGRA_GPIO_TO_IRQ(GPIO_LTE2AP_WAKEUP),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device lte_modem_wake = {
	.name = "modem_lte_wake",
	.id = -1,
};

static struct platform_device lte_modem = {
	.name = "modem_if",
	.id = 1,
	.num_resources = ARRAY_SIZE(lte_modem_res),
	.resource = lte_modem_res,
	.dev = {
		.platform_data = &lte_modem_data,
	},
};

int __init p4lte_modem_init(void)
{
	lte_modem_wake.dev.platform_data = &lte_modem_data;
	platform_device_register(&lte_modem_wake);
	return 0;
}

static struct platform_device cdma_modem_pm = {
	.name = "cdma_modem_pm",
	.id = -1,
	.dev = {
		.platform_data = &cdma_cbp71_modem_data,
	},
};

int __init p4lte_cdma_modem_pm_init(void)
{
	platform_device_register(&cdma_modem_pm);
	return 0;
}

static int __init init_modem(void)
{
#ifdef CONFIG_CDMA_MODEM_CBP71
	setup_cdma_modem_env();
	init_hw_setting_p4();
#endif

#ifdef CONFIG_CDMA_MODEM_CBP71
	if (!cbp_edpram_remap_mem_region(&cbp_edpram_cfg))
		return -1;
	platform_device_register(&cdma_cbp71_modem);
#endif

#ifdef CONFIG_LTE_MODEM_CMC220
	lte_modem_cfg_gpio();
	platform_device_register(&lte_modem);
#endif

	return 0;
}
late_initcall(init_modem);
/*device_initcall(init_modem);*/
