/*
 * arch/arm/mach-tegra/board-p4.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/nct1008.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/mfd/tps6586x.h>
#include <linux/memblock.h>
#include <linux/atmel_mxt1386.h>
#include <linux/sec_jack.h>
#include <linux/bcm4751-rfkill.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/vfs.h>
#include <linux/file.h>
#include <linux/tegra_uart.h>

#include <linux/uaccess.h>
#include <linux/spi/spi.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/spdif.h>
#include <mach/audio.h>
#include <mach/kbc.h>
#include <linux/power/p4_battery.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>

#include "board.h"
#include "clock.h"
#include "board-p4.h"
#include "devices.h"
#include <mach/gpio-sec.h>
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#include <media/s5k5bbgx_p4.h>
#include <media/s5k5ccgx.h>
#include "pm.h"
#include <linux/30pin_con.h>
#ifdef CONFIG_SEC_MODEM
#include <mach/sec_modem.h>
#endif
#ifdef CONFIG_SAMSUNG_LPM_MODE
#include <linux/moduleparam.h>
#endif

#ifdef CONFIG_KERNEL_DEBUG_SEC
#include <linux/kernel_sec_common.h>
#endif

#if defined(CONFIG_SEC_KEYBOARD_DOCK)
#include <linux/sec_keyboard_struct.h>
#endif

#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
#include <mach/tdmb_pdata.h>
#endif

extern int p3_gpio_i2c_init(void);

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct board_revision {
	unsigned int value;
	unsigned int gpio_value;
	char string[20];
};

/* We'll enumerate board revision from 10
 * to avoid a confliction with revision numbers of P3
*/
struct board_revision p4_board_rev[] = {
	{10, 0x16, "Rev00"},
	{11, 0x01, "Rev01"},
	{12, 0x02, "Rev02" },
	{13, 0x03, "Rev03" },
	{14, 0x04, "Rev04" },
};

struct board_usb_data {
	struct mutex ldo_en_lock;
	int usb_regulator_on[3];
};

static struct board_usb_data usb_data;
#ifdef CONFIG_SAMSUNG_LPM_MODE
int charging_mode_from_boot;

/* Get charging_mode status from kernel CMDLINE parameter. */
__module_param_call("", charging_mode,  &param_ops_int,
		&charging_mode_from_boot, 0, 0644);
MODULE_PARM_DESC(charging_mode_from_boot, "Charging mode parameter value.");
#endif

struct bootloader_message {
	char command[32];
	char status[32];
};

static struct mxt_callbacks *charger_callbacks;



/* REBOOT_MODE
 *
 * These defines must be kept in sync with the bootloader.
 */
#define REBOOT_MODE_NONE                0
#define REBOOT_MODE_DOWNLOAD            1
#define REBOOT_MODE_NORMAL              2
#define REBOOT_MODE_UPDATE              3
#define REBOOT_MODE_RECOVERY            4
#define REBOOT_MODE_FOTA                5
#define REBOOT_MODE_FASTBOOT            7
#define REBOOT_MODE_DOWNLOAD_FAILED     8
#define REBOOT_MODE_DOWNLOAD_SUCCESS    9

#define MISC_DEVICE "/dev/block/mmcblk0p6"

int cmc623_current_type = 0;
SYMBOL_EXPORT(cmc623_current_type);

static int write_bootloader_message(char *cmd, int mode)
{
	struct file *filp;
	mm_segment_t oldfs;
	int ret = 0;
	loff_t pos = 2048L;  /* bootloader message offset in MISC.*/

	struct bootloader_message  bootmsg;

	memset(&bootmsg, 0, sizeof(struct bootloader_message));

	if (mode == REBOOT_MODE_RECOVERY) {
		strcpy(bootmsg.command, "boot-recovery");
#ifdef CONFIG_KERNEL_DEBUG_SEC
		reboot_mode = REBOOT_MODE_RECOVERY;
		kernel_sec_set_debug_level(KERNEL_SEC_DEBUG_LEVEL_LOW);
		kernel_sec_clear_upload_magic_number();
#endif
	} else if (mode == REBOOT_MODE_FASTBOOT)
		strcpy(bootmsg.command, "boot-fastboot");
	else if (mode == REBOOT_MODE_NORMAL)
		strcpy(bootmsg.command, "boot-reboot");
	else if (mode == REBOOT_MODE_FOTA)
		strcpy(bootmsg.command, "boot-fota");
	else if (mode == REBOOT_MODE_NONE)
		strcpy(bootmsg.command, "boot-normal");
	else
		strcpy(bootmsg.command, cmd);

	bootmsg.status[0] = (char) mode;


	filp = filp_open(MISC_DEVICE, O_WRONLY, 0);

	if (IS_ERR(filp)) {
		pr_info("failed to open MISC : '%s'.\n", MISC_DEVICE);
		return 0;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_write(filp, (const char *)&bootmsg,
			sizeof(struct bootloader_message), &pos);

	set_fs(oldfs);

	if (ret < 0)
		pr_info("failed to write on MISC\n");
	else
		pr_info("command : %s written on MISC\n", bootmsg.command);

	fput(filp);
	filp_close(filp, NULL);

	return ret;
}

/* Boot Mode Physical Addresses and Magic Token */
#define BOOT_MODE_P_ADDR	(0x20000000 - 0x0C)
#define BOOT_MAGIC_P_ADDR	(0x20000000 - 0x10)
#define BOOT_MAGIC_TOKEN	0x626F6F74

static void write_bootloader_mode(char boot_mode)
{
	void __iomem *to_io;
#if 0
	to_io = ioremap(BOOT_MODE_P_ADDR, 4);
	writel((unsigned long)boot_mode, to_io);
	iounmap(to_io);
#endif
	/* Write a magic value to a 2nd memory location to distinguish between a
	 * cold boot and a reboot.
	 */
	to_io = ioremap(BOOT_MAGIC_P_ADDR, 4);
	writel(BOOT_MAGIC_TOKEN, to_io);
	iounmap(to_io);
}

#ifdef CONFIG_SAMSUNG_LPM_MODE
extern unsigned int usb_charger_Disconnect_times;
#endif

static int p3_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode;
	u32 value;
	value = gpio_get_value(GPIO_TA_nCONNECTED);

	if (code == SYS_RESTART) {
		mode = REBOOT_MODE_NORMAL;
		if (_cmd) {
			if (!strcmp((char *)_cmd, "recovery"))
				mode = REBOOT_MODE_RECOVERY;
			else if (!strcmp((char *)_cmd, "bootloader"))
				mode = REBOOT_MODE_FASTBOOT;
			else if (!strcmp((char *)_cmd, "fota"))
				mode = REBOOT_MODE_FOTA;
			else if (!strcmp((char *)_cmd, "download"))
				mode = REBOOT_MODE_DOWNLOAD;
		}
	} else if (code == SYS_POWER_OFF &&
#ifdef CONFIG_SAMSUNG_LPM_MODE
	/* added extra check to avoid LPM reboot*/
	charging_mode_from_boot == true && !usb_charger_Disconnect_times &&
#endif
	!value)
		mode = REBOOT_MODE_NORMAL;
	else
		mode = REBOOT_MODE_NONE;

	pr_debug("%s, Reboot Mode : %d\n", __func__, mode);

	write_bootloader_mode(mode);

	write_bootloader_message(_cmd, mode);

	return NOTIFY_DONE;
}

static struct notifier_block p3_reboot_notifier = {
	.notifier_call = p3_notifier_call,
};

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 11,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

#ifdef CONFIG_LINK_DEVICE_HSIC
static int notify_hsic_host_ready(void)
{
	set_host_states(&tegra_ehci2_device, TEGRA_HOST_ON);
	return 0;
}
static int notify_hsic_host_off(void)
{
	set_host_states(&tegra_ehci2_device, TEGRA_HOST_OFF);
	return 0;
}

static struct tegra_uhsic_config hsic_phy_config = {
	.enable_gpio = GPIO_HSIC_EN,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
	.usb_phy_ready = notify_hsic_host_ready,
	.post_phy_off = notify_hsic_host_off,
	.device_wake = set_slave_wake,
};
#else
static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};
#endif

#ifdef CONFIG_BT_BCM4330
static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

/* UART Interface for Bluetooth */
extern void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport);

static struct tegra_uart_platform_data bt_uart_pdata = {
	.wake_peer = bcm_bt_lpm_exit_lpm_locked,
};
#endif

#if 0
static __initdata struct tegra_clk_init_table p4_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_32k",	32768,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "pll_c",	"clk_m",	586000000,	true},
	{ "vde",	"pll_m",	240000000,	false},
	{ NULL,		NULL,		0,		0},
};
#endif
static __initdata struct tegra_clk_init_table p3_clk_init_tbl_pclk_68[] = {
	/* name		parent		rate		enabled */
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartc",      "pll_m",        600000000,      false},
	{ "blink",      "clk_32k",      32768,          false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_32k",	32768,		false},
    /*Set PLLC for PLCK 68.9Mhz (586Mhz)*/
	{ "pll_c",	"clk_m",	586000000,	true},
	{ "pll_a",	NULL,		11289600,	true},
	{ "pll_a_out0",	NULL,		11289600,	true},
	{ "clk_dev1",   "pll_a_out0",   0,              true},
	{ "i2s1",	"pll_a_out0",	11289600,	true},
	{ "i2s2",	"pll_a_out0",	11289600,	true},
	{ "audio",	"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",	22579200,	true},
	{ "spdif_out",	"pll_a_out0",	5644800,	false},
	{ "vde",	"pll_m",	240000000,	false},
	{ NULL,		NULL,		0,		0},
};

static __initdata struct tegra_clk_init_table p3_clk_init_tbl_pclk_76[] = {
	/* name		parent		rate		enabled */
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartc",      "pll_m",        600000000,      false},
	{ "blink",      "clk_32k",      32768,          false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_32k",	32768,		false},
    /*Set PLLC for PLCK 76Mhz (570Mhz)*/
	{ "pll_c",	"clk_m",	570000000,	true},
	{ "pll_a",	NULL,		11289600,	true},
	{ "pll_a_out0",	NULL,		11289600,	true},
	{ "clk_dev1",   "pll_a_out0",   0,              true},
	{ "i2s1",	"pll_a_out0",	11289600,	true},
	{ "i2s2",	"pll_a_out0",	11289600,	true},
	{ "audio",	"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",	22579200,	true},
	{ "spdif_out",	"pll_a_out0",	5644800,	false},
	{ "vde",	"pll_m",	240000000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_ulpi_config p4_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data p4_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
	.phy_config = &p4_ehci2_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
	.default_enable = true,
};

static struct tegra_i2c_platform_data p3_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data p3_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 10000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr = 0x00FC,
};

static struct tegra_i2c_platform_data p3_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
};

static struct tegra_i2c_platform_data p3_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_dvc		= true,
};

static void p3_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &p3_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &p3_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &p3_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &p3_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);

}
static struct platform_device *p4_uart_devices[] __initdata = {
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarta_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data p4_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTB is the debug port. */
	pr_info("Selecting UARTB as the debug console\n");
	p4_uart_devices[0] = &debug_uartb_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartb");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
	}
}

static void __init p4_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	p4_uart_pdata.parent_clk_list = uart_parent_clk;
	p4_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &p4_uart_pdata;
	tegra_uartb_device.dev.platform_data = &p4_uart_pdata;
#ifdef CONFIG_BT_BCM4330
	tegra_uartc_device.dev.platform_data = &bt_uart_pdata;
#else
	tegra_uartc_device.dev.platform_data = &p4_uart_pdata;
#endif
	tegra_uartd_device.dev.platform_data = &p4_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(p4_uart_devices,
				ARRAY_SIZE(p4_uart_devices));
}

#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = GPIO_##_gpio,	\
		.active_low = 0,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button p3_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, EXT_WAKEUP, 1), /* EXT_WAKEUP */
};

#define PMC_WAKE_STATUS 0x14

static int p3_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	if (status & TEGRA_WAKE_GPIO_PS4) {
		writel(TEGRA_WAKE_GPIO_PS4,
			IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	}

	return status & TEGRA_WAKE_GPIO_PS4 ? KEY_POWER : KEY_RESERVED;
}

#ifdef CONFIG_SAMSUNG_LPM_MODE
static bool p3_check_lpm(void)
{
	return charging_mode_from_boot ? true : false;
}
#endif

static struct gpio_keys_platform_data p3_keys_platform_data = {
	.buttons	= p3_keys,
	.nbuttons	= ARRAY_SIZE(p3_keys),
	.wakeup_key	= p3_wakeup_key,
#ifdef CONFIG_SAMSUNG_LPM_MODE
	.check_lpm = p3_check_lpm,
#endif
};

static struct platform_device p3_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &p3_keys_platform_data,
	},
};

static struct platform_device p3_audio_device = {
	.name	= "tegra-snd-wm8994",
	.id	= 0,
};

static int p4_jack_init(void)
{
	int ret = 0;
	int ear_micbias = 0;

	if (system_rev < 0x3)
		ear_micbias = TEGRA_GPIO_PH3;
	else
		ear_micbias = GPIO_EAR_MICBIAS_EN;

	ret = gpio_request(GPIO_MICBIAS_EN, "micbias_enable");
	if (ret < 0)
		return ret;

	ret = gpio_request(ear_micbias, "ear_micbias_enable");
	if (ret < 0) {
		gpio_free(ear_micbias);
		return ret;
	}

	ret = gpio_direction_output(GPIO_MICBIAS_EN, 0);
	if (ret < 0)
		goto cleanup;

	ret = gpio_direction_output(ear_micbias, 0);
	if (ret < 0)
		goto cleanup;

	tegra_gpio_enable(GPIO_MICBIAS_EN);
	tegra_gpio_enable(ear_micbias);
	tegra_gpio_enable(GPIO_DET_3_5);
	tegra_gpio_enable(GPIO_EAR_SEND_END);

cleanup:
	gpio_free(GPIO_MICBIAS_EN);
	gpio_free(ear_micbias);

	return ret;
}

static void sec_jack_set_micbias_state(bool on)
{
	printk(KERN_DEBUG
		"Board P4 : Enterring sec_jack_set_micbias_state = %d\n", on);
	if (system_rev < 0x3)
		gpio_set_value(TEGRA_GPIO_PH3, on);
	else
		gpio_set_value(GPIO_EAR_MICBIAS_EN, on);
}

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, default to 3pole if it stays
		 * in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 0,
		.delay_ms = 0, /* delay 20ms in stmpe811 driver */
		.check_count = 2,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 900, unstable zone, default to 3pole if it stays
		 * in this range for a 400ms (20ms delays, 20 samples)
		 */
		.adc_high = 900,
		.delay_ms = 0,
		.check_count = 20,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 900 < adc <= 2000, unstable zone, default to 4pole if it
		 * stays in this range for 400ms (20ms delays, 20 samples)
		 */
		.adc_high = 2000,
		.delay_ms = 0,
		.check_count = 20,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 2000 < adc <= 3800, default to 4 pole if it stays */
		/* in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 3800,
		.delay_ms = 0,
		.check_count = 2,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 3800, unstable zone, default to 3pole if it stays
		 * in this range for a second (20ms delays, 50 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 0,
		.check_count = 50,
		.jack_type = SEC_HEADSET_3POLE,
	},
};


/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=150, stable zone */
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 150,
	},
	{
		/* 151 <= adc <= 330, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low	= 151,
		.adc_high	= 330,
	},
	{
		/* 331 <= adc <= 1000, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 331,
		.adc_high	= 1000,
	},
};

static int sec_jack_get_adc_value(void)
{
	s16 ret;
	if (system_rev < 0x2)
		ret = 2000; /* temporary fix: adc_get_value(0); */
	else
		ret = stmpe811_adc_get_value(4);
	printk(KERN_DEBUG
		"Board P4 : Enterring sec_jack_get_adc_value = %d\n", ret);
	return  ret;
}

struct sec_jack_platform_data sec_jack_pdata = {
	.set_micbias_state = sec_jack_set_micbias_state,
	.get_adc_value = sec_jack_get_adc_value,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.buttons_zones = sec_jack_buttons_zones,
	.num_buttons_zones = ARRAY_SIZE(sec_jack_buttons_zones),
	.det_gpio = GPIO_DET_3_5,
	.send_end_gpio = GPIO_EAR_SEND_END,
};

static struct platform_device sec_device_jack = {
	.name			= "sec_jack",
	.id			= 1, /* will be used also for gpio_event id */
	.dev.platform_data	= &sec_jack_pdata,
};

static void p3_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(p3_keys); i++)
		tegra_gpio_enable(p3_keys[i].gpio);
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

void tegra_acc_power(u8 token, bool active)
{
	int gpio_acc_en;
	int gpio_acc_5v;
	int try_cnt = 0;
	static bool enable = false;
	static u8 acc_en_token = 0;

	gpio_acc_en = GPIO_ACCESSORY_EN;
	gpio_acc_5v = GPIO_V_ACCESSORY_5V;

	/*	token info
		0 : force power off,
		1 : usb otg
		2 : ear jack
		3 : keyboard
	*/

	if (active) {
		acc_en_token |= (1 << token);
		enable = true;
		gpio_direction_output(gpio_acc_en, 1);
		msleep(1);
		while(!gpio_get_value(gpio_acc_5v)) {
			gpio_direction_output(gpio_acc_en, 0);
			msleep(10);
			gpio_direction_output(gpio_acc_en, 1);
			if (try_cnt > 30) {
				pr_err("[acc] failed to enable the accessory_en");
				break;
			} else
				try_cnt++;
		}
	} else {
		if (0 == token) {
			gpio_direction_output(gpio_acc_en, 0);
			enable = false;
		} else {
			acc_en_token &= ~(1 << token);
			if (0 == acc_en_token) {
				gpio_direction_output(gpio_acc_en, 0);
				enable = false;
			}
		}
	}
	pr_info("Board P4 : %s token : (%d,%d) %s\n", __func__,
		token, active, enable ? "on" : "off");
}

static int p3_kbc_keycode[] = {
	[0] = KEY_VOLUMEDOWN,
	[1] = KEY_VOLUMEUP,
};

static struct tegra_kbc_platform_data p3_kbc_platform = {
	.debounce_cnt = 10,
	.repeat_cnt = 1024,
	.scan_timeout_cnt = 3000 * 32,
	.pin_cfg = {
		[0] = {
			.num = 0,
			.pin_type = kbc_pin_row,
		},
		[1] = {
			.num = 1,
			.pin_type = kbc_pin_row,
		},
		[17] = {
			.num = 1,
			.pin_type = kbc_pin_col,
		},
	},
	.plain_keycode = p3_kbc_keycode,
};

static struct resource p3_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device p3_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &p3_kbc_platform,
	},
	.resource = p3_kbc_resources,
	.num_resources = ARRAY_SIZE(p3_kbc_resources),
};

static void tegra_otg_en(int active)
{
	int gpio_otg_en;

	gpio_otg_en = GPIO_OTG_EN;

	active = !!active;
	gpio_direction_output(gpio_otg_en, active);
	pr_info("Board P4 : %s = %d\n", __func__, active);
}

static void tegra_usb_ldo_en(int active, int instance)
{
	struct regulator *reg = regulator_get(NULL, "vdd_ldo6");
	int ret = 0;
	int try_times = 5;

	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get vdd_ldo6 regulator\n", __func__);
		return;
	}

	pr_info("Board P4 : %s=%d instance=%d present regulator=%d\n",
		 __func__, active, instance, usb_data.usb_regulator_on[instance]);
	mutex_lock(&usb_data.ldo_en_lock);

	if (active) {
		if (!usb_data.usb_regulator_on[instance]) {
			do {
				ret = regulator_enable(reg);
				if (ret == 0)
					break;
				msleep(3);
			} while(try_times--);
			if (ret == 0)
				usb_data.usb_regulator_on[instance] = 1;
			else
				pr_err("%s: failed to turn on \
					vdd_ldo6 regulator\n", __func__);
		}
	} else {
		regulator_disable(reg);
		usb_data.usb_regulator_on[instance] = 0;
	}
	regulator_put(reg);

	mutex_unlock(&usb_data.ldo_en_lock);
}

static bool check_samsung_charger(void)
{
	bool result;
	int sum = 0;
	int count;
	int vol_1;
	usb_path_type old_usb_sel_status;
	struct regulator *reg = regulator_get(NULL, "vdd_ldo6");

	/* when device wakes from suspend due to charger being plugged
	 * in, this code runs before nct1008.c resume enables ldo6
	 * so we need to enable it here.
	 */
	regulator_enable(reg);
	udelay(10);

	old_usb_sel_status = usb_sel_status;
	p3_set_usb_path(USB_SEL_ADC);

	mdelay(100);

	if (system_rev < 0x02)
		/*
		for (count = 0; count < 2; count++)
			sum += adc_get_value(1);

		vol_1 = sum / 2;
		pr_debug("%s: samsung_charger_adc = %d !!\n", __func__, vol_1);

		if ((vol_1 > 900) && (vol_1 < 1300))
			result = true;
		else*/
		result = false;
	else {
		for (count = 0; count < 2; count++)
			sum += stmpe811_adc_get_value(6);

		vol_1 = sum / 2;
		pr_debug("%s: samsung_charger_adc = %d !!\n", __func__, vol_1);

		if ((vol_1 > 800) && (vol_1 < 1800))
			result = true;
		else
			result = false;
	}

	mdelay(50);

	p3_set_usb_path(old_usb_sel_status);

	regulator_disable(reg);
	regulator_put(reg);

	pr_debug("%s: returning %d\n", __func__, result);
	return result;
}

void p3_bat_gpio_init(void)
{
	gpio_request(GPIO_TA_EN, "GPIO_TA_EN");
	gpio_direction_output(GPIO_TA_EN, 0);
	tegra_gpio_enable(GPIO_TA_EN);

	gpio_request(GPIO_TA_nCONNECTED, "GPIO_TA_nCONNECTED");
	gpio_direction_input(GPIO_TA_nCONNECTED);
	tegra_gpio_enable(GPIO_TA_nCONNECTED);

	gpio_request(GPIO_TA_nCHG, "GPIO_TA_nCHG");
	gpio_direction_input(GPIO_TA_nCHG);
	tegra_gpio_enable(GPIO_TA_nCHG);

	gpio_request(GPIO_CURR_ADJ, "GPIO_CURR_ADJ");
	if (check_samsung_charger() == 1)
		gpio_direction_output(GPIO_CURR_ADJ, 1);
	else
		gpio_direction_output(GPIO_CURR_ADJ, 0);
	tegra_gpio_enable(GPIO_CURR_ADJ);

	gpio_request(GPIO_FUEL_ALRT, "GPIO_FUEL_ALRT");
	gpio_direction_input(GPIO_FUEL_ALRT);
	tegra_gpio_enable(GPIO_FUEL_ALRT);

	pr_info("Battery GPIO initialized.\n");

}

static void  p3_inform_charger_connection(int mode)
{
	if (charger_callbacks && charger_callbacks->inform_charger)
		charger_callbacks->inform_charger(charger_callbacks, mode);
};

static struct p3_battery_platform_data p3_battery_platform = {
	.charger = {
		.enable_line = GPIO_TA_EN,
		.connect_line = GPIO_TA_nCONNECTED,
		.fullcharge_line = GPIO_TA_nCHG,
		.currentset_line = GPIO_CURR_ADJ,
	},
	.check_dedicated_charger = check_samsung_charger,
	.init_charger_gpio = p3_bat_gpio_init,
	.inform_charger_connection = p3_inform_charger_connection,

#if defined(CONFIG_TARGET_LOCALE_KOR)
	.temp_high_threshold = 60000,
	/* 546 + 32 (dT) ->600 by kim2054's request*/
	.temp_high_recovery = 42600,	/* 426 */
	.temp_low_recovery = -1000,		/* -10 */
	.temp_low_threshold = -4000,		/* -40 */
	.charge_duration = 10*60*60,	/* 10 hour */
	.recharge_duration = 2*60*60,	/* 2 hour */
	.recharge_voltage = 4150,	/*4.15V */
#elif defined(CONFIG_MACH_SAMSUNG_P4TMO)
	.temp_high_threshold = 50000,	/* 50c */
	.temp_high_recovery = 45000,	/* 45c */
	.temp_low_recovery = 2000,		/* 2c */
	.temp_low_threshold = 0,		/* 0c */
	.charge_duration = 10*60*60,	/* 10 hour */
	.recharge_duration = 1.5*60*60,	/* 1.5 hour */
	.recharge_voltage = 4150,	/*4.15V */
#else
	.temp_high_threshold = 50000,	/* 50c */
	.temp_high_recovery = 42000,	/* 42c */
	.temp_low_recovery = 2000,		/* 2c */
	.temp_low_threshold = 0,		/* 0c */
	.charge_duration = 10*60*60,	/* 10 hour */
	.recharge_duration = 1.5*60*60,	/* 1.5 hour */
	.recharge_voltage = 4150,	/*4.15V */
#endif
};

static struct platform_device p3_battery_device = {
	.name = "p3-battery",
	.id = -1,
	.dev = {
		.platform_data = &p3_battery_platform,
	},
};
#ifdef CONFIG_SEC_KEYBOARD_DOCK
#if 0
struct uart_platform_data {
        void(*send_to_keyboard)(unsigned int val);
};

struct kbd_callbacks {
        void (*get_data)(struct kbd_callbacks *, unsigned int val);
#if 0
        int (*check_keyboard_dock)(struct kbd_callbacks *, int val);
#endif
};

static struct kbd_callbacks sec_kdb_cb;

static void uart_to_keyboard(unsigned int val)
{
        if (sec_kdb_cb && sec_kdb_cb->get_data)
                sec_kdb_cb->get_data(sec_kdb_cb, val);
}

static int check_keyboard(struct kbd_callbacks *, int val)
{
        if (sec_kdb_cb && sec_kdb_cb->check_keyboard_dock)
                return sec_kdb_cb->check_keyboard_dock(sec_kdb_cb, val);
        return 0;
}

static void sec_keyboard_register_callbacks(struct kbd_callbacks *cb)
{
        sec_kdb_cb = cb;
}

static struct dock_keyboard_platform_data kbd_pdata {
        .enable= ,
        .disable= ,
        .register_cb =sec_keyboard_register_callbacks,
};

static struct platform_device sec_keyboard = {
        .name   = "sec_keyboard",
        .id     = -1,
        .dev = {
                .platform_data = &kbd_pdata,
        }
};

static struct uart_platform_data uart_pdata {
        .send_to_keyboard =uart_to_keyboard,
};

#else
static int dock_wakeup(void)
{
        unsigned long status =
               readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

       if (status & TEGRA_WAKE_GPIO_PI5) {
                writel(TEGRA_WAKE_GPIO_PI5,
                       IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
       }

       return status & TEGRA_WAKE_GPIO_PI5 ? KEY_WAKEUP : KEY_RESERVED;
}

static struct dock_keyboard_platform_data kbd_pdata = {
        .acc_power = tegra_acc_power,
        .wakeup_key = dock_wakeup,
        .accessory_irq_gpio = GPIO_ACCESSORY_INT,
};

static struct platform_device sec_keyboard = {
        .name   = "sec_keyboard",
        .id     = -1,
        .dev = {
                .platform_data = &kbd_pdata,
        }
};
#endif
#endif

#ifdef CONFIG_30PIN_CONN
struct acc_con_platform_data acc_con_pdata = {
	.otg_en = tegra_otg_en,
	.acc_power = tegra_acc_power,
	.usb_ldo_en = tegra_usb_ldo_en,
	.accessory_irq_gpio = GPIO_ACCESSORY_INT,
	.dock_irq_gpio = GPIO_DOCK_INT,
	.mhl_irq_gpio = GPIO_MHL_INT,
	.hdmi_hpd_gpio = GPIO_HDMI_HPD,
};
struct platform_device sec_device_connector = {
	.name = "acc_con",
	.id = -1,
	.dev.platform_data = &acc_con_pdata,
};
#endif

static struct platform_device *p3_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_KEYBOARD_GPIO
	&p3_keys_device,
	&p3_kbc_device,
#endif
	&tegra_wdt_device,
	&p3_battery_device,
	&tegra_avp_device,
	&tegra_camera,
	&sec_device_jack,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
#ifdef CONFIG_30PIN_CONN
	&sec_device_connector,
#ifdef CONFIG_SEC_KEYBOARD_DOCK
	&sec_keyboard,
#endif
#endif
	&tegra_spdif_device,
	&tegra_das_device,
/*	&spdif_dit_device,*/
	&bluetooth_dit_device,
#ifdef CONFIG_BT_BCM4330
	&bcm4330_bluetooth_device,
#endif
	&tegra_pcm_device,
	&p3_audio_device,
#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
	&tegra_spi_device3,
#endif
};

static void sec_s5k5ccgx_init(void)
{
	printk(KERN_INFO "%s,,\n", __func__);

	tegra_gpio_enable(GPIO_CAM_PMIC_EN1);	/*3M_CORE_1.2V*/
	tegra_gpio_enable(GPIO_CAM_PMIC_EN2);	/*CAM_AVDD2.8V*/
	tegra_gpio_enable(GPIO_CAM_PMIC_EN3);	/*2M_DVDD_1.8V*/
	tegra_gpio_enable(GPIO_CAM_PMIC_EN4);	/*CAM_IO_1.8V*/
	tegra_gpio_enable(GPIO_CAM_R_nRST);		/*3M nRST*/
	tegra_gpio_enable(GPIO_CAM_R_nSTBY);		/*3M STBY*/
/*temp	tegra_gpio_enable(GPIO_CAM_MOVIE_EN);	//flash ??*/
	tegra_gpio_enable(GPIO_CAM_FLASH_SET);	/*flash ??*/

	gpio_request(GPIO_CAM_PMIC_EN1, "CAMERA_PMIC_EN1");
	gpio_request(GPIO_CAM_PMIC_EN2, "CAMERA_PMIC_EN2");
	gpio_request(GPIO_CAM_PMIC_EN3, "CAMERA_PMIC_EN3");
	gpio_request(GPIO_CAM_PMIC_EN4, "CAMERA_PMIC_EN4");
	gpio_request(GPIO_CAM_R_nRST, "CAMERA_CAM_Left_nRST");
	gpio_request(GPIO_CAM_R_nSTBY, "CAMERA_CAM_nSTBY");
	gpio_request(GPIO_CAM_FLASH_EN, "CAM_FLASH_EN");
	gpio_request(GPIO_CAM_FLASH_SET, "CAM_FLASH_SET");

	gpio_direction_output(GPIO_CAM_PMIC_EN1, 0);
	gpio_direction_output(GPIO_CAM_PMIC_EN2, 0);
	gpio_direction_output(GPIO_CAM_PMIC_EN3, 0);
	gpio_direction_output(GPIO_CAM_PMIC_EN4, 0);
	gpio_direction_output(GPIO_CAM_R_nRST, 0);
	gpio_direction_output(GPIO_CAM_R_nSTBY, 0);
	gpio_direction_output(GPIO_CAM_FLASH_EN, 0);
	gpio_direction_output(GPIO_CAM_FLASH_SET, 0);

#if 0
	printk(KERN_INFO "<=PCAM=> LOW 1: %d\n",  \
	gpio_get_value(GPIO_CAM_PMIC_EN1));
	printk(KERN_INFO "<=PCAM=> LOW 2: %d\n",  \
		gpio_get_value(GPIO_CAM_PMIC_EN2));
	printk(KERN_INFO "<=PCAM=> LOW 4: %d\n",  \
		gpio_get_value(GPIO_CAM_PMIC_EN4));

	gpio_direction_output(GPIO_CAM_PMIC_EN1, 1);
	gpio_direction_output(GPIO_CAM_PMIC_EN2, 1);
	gpio_direction_output(GPIO_CAM_PMIC_EN4, 1);
	mdelay(10);

	printk(KERN_INFO "<=PCAM=> HIGH 1: %d\n", \
		gpio_get_value(GPIO_CAM_PMIC_EN1));
	printk(KERN_INFO "<=PCAM=> HIGH 2: %d\n",	\
		gpio_get_value(GPIO_CAM_PMIC_EN2));
	printk(KERN_INFO "<=PCAM=> HIGH 4: %d\n",	\
		gpio_get_value(GPIO_CAM_PMIC_EN4));

	gpio_direction_output(GPIO_CAM_PMIC_EN1, 0);
	gpio_direction_output(GPIO_CAM_PMIC_EN2, 0);
	gpio_direction_output(GPIO_CAM_PMIC_EN4, 0);
	mdelay(10);

	printk(KERN_INFO "<=PCAM=> LOW 1: %d\n",	\
		gpio_get_value(GPIO_CAM_PMIC_EN1));
	printk(KERN_INFO "<=PCAM=> LOW 2: %d\n",	\
		gpio_get_value(GPIO_CAM_PMIC_EN2));
	printk(KERN_INFO "<=PCAM=> LOW 4: %d\n",	\
		gpio_get_value(GPIO_CAM_PMIC_EN4));
#endif
}

struct tegra_pingroup_config mclk = {
	TEGRA_PINGROUP_CSUS,
	TEGRA_MUX_VI_SENSOR_CLK,
	TEGRA_PUPD_PULL_DOWN,
	TEGRA_TRI_TRISTATE
};

/*static void P3_s5k5ccgx_flash_on()
{
	//gpio_set_value(GPIO_CAM_FLASH_SET, 1);
	gpio_set_value(GPIO_CAM_FLASH_SET, 1);

}

static void P3_s5k5ccgx_flash_off()
{
	//gpio_set_value(GPIO_CAM_FLASH_SET, 0);
	gpio_set_value(GPIO_CAM_FLASH_SET, 0);
}*/

static void p3_s5k5ccgx_power_on(void)
{
	printk(KERN_INFO "%s,,\n", __func__);
	gpio_set_value(GPIO_CAM_R_nSTBY, 0);/*3M STBY low*/
	gpio_set_value(GPIO_CAM_R_nRST, 0); /*3M nRST low*/
	gpio_set_value(GPIO_CAM_F_nSTBY, 0); /* 2M STBY low*/
	gpio_set_value(GPIO_CAM_F_nRST, 0); /* 2M nRST low*/
	gpio_set_value(GPIO_CAM_PMIC_EN1, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN4, 0);
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN1, 1); /* 3M_CORE_1.2V, 3M_AF_2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 1); /* CAM_AVDD2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 1); /* 2M_DVDD_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN4, 1); /* CAM_IO_1.8V*/
	udelay(100);

	tegra_pinmux_set_func(&mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	udelay(100);

	gpio_set_value(GPIO_CAM_R_nSTBY, 1); /*3M STBY high*/
	udelay(200);

	gpio_set_value(GPIO_CAM_R_nRST, 1); /*3M nRST high*/
	msleep(10);
}

static void p3_s5k5ccgx_power_off(void)
{
	printk(KERN_INFO "%s,,\n", __func__);
	msleep(3);
	gpio_set_value(GPIO_CAM_R_nRST, 0); /*3M nRST Low*/
	udelay(100);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	udelay(100);

	gpio_set_value(GPIO_CAM_R_nSTBY, 0); /*3M STBY Low*/
	udelay(100);

	gpio_set_value(GPIO_CAM_PMIC_EN4, 0);/* CAM_IO_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 0); /* 2M_DVDD_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);/* CAM_AVDD2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN1, 0);/* 3M_CORE_1.2V, 3M_AF_2.8V*/
	if (system_rev < 14)
		msleep(800);
	else {
		msleep(200);
		printk(KERN_INFO "p3_s5k5ccgx_power_off msleep--------200ms  system_rev = %d\n", system_rev);
	}
}

#define FLASH_MOVIE_MODE_CURRENT_100_PERCENT	1
#define FLASH_MOVIE_MODE_CURRENT_79_PERCENT	3
#define FLASH_MOVIE_MODE_CURRENT_50_PERCENT	7
#define FLASH_MOVIE_MODE_CURRENT_32_PERCENT	11
#define FLASH_MOVIE_MODE_CURRENT_28_PERCENT	12

#define FLASH_TIME_LATCH_US			500
#define FLASH_TIME_EN_SET_US			1

/* The AAT1274 uses a single wire interface to write data to its
 * control registers. An incoming value is written by sending a number
 * of rising edges to EN_SET. Data is 4 bits, or 1-16 pulses, and
 * addresses are 17 pulses or more. Data written without an address
 * controls the current to the LED via the default address 17. */
static void aat1274_write(int value)
{
	while (value--) {
		gpio_set_value(GPIO_CAM_FLASH_SET, 0);
		udelay(FLASH_TIME_EN_SET_US);
		gpio_set_value(GPIO_CAM_FLASH_SET, 1);
		udelay(FLASH_TIME_EN_SET_US);
	}
	udelay(FLASH_TIME_LATCH_US);
	/* At this point, the LED will be on */
}

static int P3_s5k5ccgx_flash(int enable)
{
	/* Turn main flash on or off by asserting a value on the EN line. */
	printk(KERN_INFO "========== flash enable = %d\n", enable);
	gpio_set_value(GPIO_CAM_FLASH_EN, enable);

	return 0;
}

static int P3_s5k5ccgx_af_assist(int enable)
{
	/* Turn assist light on or off by asserting a value on the EN_SET
	 * line. The default illumination level of 1/7.3 at 100% is used */
	printk(KERN_INFO "========== flash af_assist = %d\n", enable);
#if 0
	gpio_set_value(GPIO_CAM_FLASH_EN, enable);
	if (enable) {
		udelay(100);
		aat1274_write(FLASH_MOVIE_MODE_CURRENT_28_PERCENT);
	} else{
		gpio_set_value(GPIO_CAM_FLASH_EN, 0);
		gpio_set_value(GPIO_CAM_FLASH_SET, 0);
	}
#endif
	gpio_set_value(GPIO_CAM_FLASH_EN, 0);
	if (enable)
		aat1274_write(FLASH_MOVIE_MODE_CURRENT_100_PERCENT);
	else
		gpio_set_value(GPIO_CAM_FLASH_SET, 0);

	return 0;
}

static int P3_s5k5ccgx_torch(int enable)
{
	/* Turn torch mode on or off by writing to the EN_SET line. A level
	 * of 1/7.3 and 50% is used (half AF assist brightness). */
	gpio_set_value(GPIO_CAM_FLASH_EN, 0);
	if (enable)
		aat1274_write(FLASH_MOVIE_MODE_CURRENT_79_PERCENT);
	else
		gpio_set_value(GPIO_CAM_FLASH_SET, 0);

	return 0;
}

struct s5k5ccgx_platform_data p3_s5k5ccgx_data = {
	.power_on = p3_s5k5ccgx_power_on,
	.power_off = p3_s5k5ccgx_power_off,
	.flash_onoff = P3_s5k5ccgx_flash,
	.af_assist_onoff = P3_s5k5ccgx_af_assist,
	.torch_onoff = P3_s5k5ccgx_torch,
	/*.isp_int_read = p3_s5k5ccgx_isp_int_read*/
};

static const struct i2c_board_info sec_s5k5ccgx_camera[] = {
	{
		/*I2C_BOARD_INFO("imx073", 0x3E>>1),*/
		I2C_BOARD_INFO("s5k5ccgx", 0x78>>1), /* 0xAC*/
		.platform_data = &p3_s5k5ccgx_data,
	},
};

struct tegra_pingroup_config s5k5bbgx_mclk = {
	TEGRA_PINGROUP_CSUS, TEGRA_MUX_VI_SENSOR_CLK,
	TEGRA_PUPD_PULL_DOWN, TEGRA_TRI_TRISTATE
};

void p3_s5k5bbgx_power_on(void)
{
	printk(KERN_INFO "%s,,\n", __func__);
	gpio_set_value(GPIO_CAM_R_nSTBY, 0); /*3M STBY low*/
	gpio_set_value(GPIO_CAM_R_nRST, 0); /*3M nRST low*/
	gpio_set_value(GPIO_CAM_F_nSTBY, 0); /* 2M STBY low*/
	gpio_set_value(GPIO_CAM_F_nRST, 0); /* 2M nRST low*/
	gpio_set_value(GPIO_CAM_PMIC_EN1, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 0);
	gpio_set_value(GPIO_CAM_PMIC_EN4, 0);
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN1, 1); /* 3M_CORE_1.2V, 3M_AF_2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 1); /* CAM_AVDD2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 1); /* 2M_DVDD_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN4, 1); /* CAM_IO_1.8V*/
	udelay(100);

	tegra_pinmux_set_func(&mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	udelay(100);

	gpio_set_value(GPIO_CAM_F_nSTBY, 1); /* 2M STBY High*/
	udelay(100);

	gpio_set_value(GPIO_CAM_F_nRST, 1); /* 2M nRST High*/
	msleep(10); /*udelay(200);*/
}

void p3_s5k5bbgx_power_off(void)
{
	msleep(3);
	gpio_set_value(GPIO_CAM_F_nRST, 0); /* 2M nRST Low*/
	udelay(100);

	gpio_set_value(GPIO_CAM_F_nSTBY, 0); /* 2M STBY Low*/
	udelay(100);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	udelay(100);

	gpio_set_value(GPIO_CAM_PMIC_EN4, 0); /* CAM_IO_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN3, 0); /* 2M_DVDD_1.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN2, 0); /* CAM_AVDD2.8V*/
	udelay(100);
	gpio_set_value(GPIO_CAM_PMIC_EN1, 0); /* 3M_CORE_1.2V, 3M_AF_2.8V*/
	if (system_rev < 14)
		msleep(800);
	else
		msleep(200);
}

struct s5k5bbgx_platform_data p3_s5k5bbgx_data = {
	.power_on = p3_s5k5bbgx_power_on,
	.power_off = p3_s5k5bbgx_power_off
};

static const struct i2c_board_info sec_s5k5bbgx_camera[] = {
	{
		I2C_BOARD_INFO("s5k5bbgx", 0x5a>>1),
		.platform_data = &p3_s5k5bbgx_data,
	},
};

static int __init p3_camera_init(void)
{
	int status;
	sec_s5k5ccgx_init();
	status = i2c_register_board_info(3, sec_s5k5ccgx_camera,
				ARRAY_SIZE(sec_s5k5ccgx_camera));
	status = i2c_register_board_info(3, sec_s5k5bbgx_camera,
				ARRAY_SIZE(sec_s5k5bbgx_camera));

	return 0;
}


static void p3_touch_exit_hw(void)
{
	pr_info("p3_touch_exit_hw\n");
	gpio_free(GPIO_TOUCH_INT);
	gpio_free(GPIO_TOUCH_RST);
	gpio_free(GPIO_TOUCH_EN);

	tegra_gpio_disable(GPIO_TOUCH_INT);
	tegra_gpio_disable(GPIO_TOUCH_RST);
	tegra_gpio_disable(GPIO_TOUCH_EN);
}


static void p3_touch_suspend_hw(void)
{
	gpio_direction_output(GPIO_TOUCH_RST, 0);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TOUCH_EN, 0);
}

static void p3_touch_resume_hw(void)
{
	gpio_direction_output(GPIO_TOUCH_RST, 1);
	gpio_direction_output(GPIO_TOUCH_EN, 1);
	gpio_direction_input(GPIO_TOUCH_INT);
	msleep(120);
}

static void p3_register_touch_callbacks(struct mxt_callbacks *cb)
{
	charger_callbacks = cb;
}

/*p3 touch : atmel_mxt1386*/
static void p3_touch_init_hw(void)
{
	pr_info("p3_touch_init_hw\n");
	gpio_request(GPIO_TOUCH_EN, "TOUCH_EN");
	gpio_request(GPIO_TOUCH_RST, "TOUCH_RST");
	gpio_request(GPIO_TOUCH_INT, "TOUCH_INT");

	gpio_direction_output(GPIO_TOUCH_EN, 1);
	gpio_direction_output(GPIO_TOUCH_RST, 1);
	gpio_direction_input(GPIO_TOUCH_INT);

	tegra_gpio_enable(GPIO_TOUCH_EN);
	tegra_gpio_enable(GPIO_TOUCH_RST);
	tegra_gpio_enable(GPIO_TOUCH_INT);
}

static struct mxt_platform_data p3_touch_platform_data = {
	.numtouch = 10,
	.max_x  = 1279,
	.max_y  = 799,
	.init_platform_hw  = p3_touch_init_hw,
	.exit_platform_hw  = p3_touch_exit_hw,
	.suspend_platform_hw = p3_touch_suspend_hw,
	.resume_platform_hw = p3_touch_resume_hw,
	.register_cb = p3_register_touch_callbacks,
	/*mxt_power_config*/
	/* Set Idle Acquisition Interval to 32 ms. */
	.power_config.idleacqint = 32,
	.power_config.actvacqint = 255,
	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	.power_config.actv2idleto = 50,
	/*acquisition_config*/
	/* Atmel: 8 -> 10*/
	.acquisition_config.chrgtime = 10,
	.acquisition_config.reserved = 0,
	.acquisition_config.tchdrift = 5,
	/* Atmel: 0 -> 10*/
	.acquisition_config.driftst = 10,
	/* infinite*/
	.acquisition_config.tchautocal = 0,
	/* disabled*/
	.acquisition_config.sync = 0,
#ifdef MXT_CALIBRATE_WORKAROUND
	/*autocal config at wakeup status*/
	.acquisition_config.atchcalst = 9,
	.acquisition_config.atchcalsthr = 48,
	/* Atmel: 50 => 10 : avoid wakeup lockup : 2 or 3 finger*/
	.acquisition_config.atchcalfrcthr = 10,
	.acquisition_config.atchcalfrcratio = 215,
#else
	/* Atmel: 5 -> 0 -> 9  (to avoid ghost touch problem)*/
	.acquisition_config.atchcalst = 9,
	/* Atmel: 50 -> 55 -> 48 ->10 (to avoid ghost touch problem)*/
	.acquisition_config.atchcalsthr = 10,
	/* 50-> 20 (To avoid  wakeup touch lockup)  */
	.acquisition_config.atchcalfrcthr = 20,
	/* 25-> 0  (To avoid  wakeup touch lockup */
	.acquisition_config.atchcalfrcratio = 0,
#endif
	/*multitouch_config*/
	/* enable + message-enable*/
	.touchscreen_config.ctrl = 0x8b,
	.touchscreen_config.xorigin = 0,
	.touchscreen_config.yorigin = 0,
	.touchscreen_config.xsize = 27,
	.touchscreen_config.ysize = 42,
	.touchscreen_config.akscfg = 0,
	/* Atmel: 0x11 -> 0x21 -> 0x11*/
	.touchscreen_config.blen = 0x11,
	/* Atmel: 50 -> 55 -> 48,*/
	.touchscreen_config.tchthr = 48,
	.touchscreen_config.tchdi = 2,
	/* orient : Horizontal flip */
	.touchscreen_config.orient = 1,
	.touchscreen_config.mrgtimeout = 0,
	.touchscreen_config.movhysti = 10,
	.touchscreen_config.movhystn = 1,
	 /* Atmel  0x20 ->0x21 -> 0x2e(-2)*/
	.touchscreen_config.movfilter = 0x50,
	.touchscreen_config.numtouch = MXT_MAX_NUM_TOUCHES,
	.touchscreen_config.mrghyst = 5, /*Atmel 10 -> 5*/
	 /* Atmel 20 -> 5 -> 50 (To avoid One finger Pinch Zoom) */
	.touchscreen_config.mrgthr = 50,
	.touchscreen_config.amphyst = 10,
	.touchscreen_config.xrange = 799,
	.touchscreen_config.yrange = 1279,
	.touchscreen_config.xloclip = 0,
	.touchscreen_config.xhiclip = 0,
	.touchscreen_config.yloclip = 0,
	.touchscreen_config.yhiclip = 0,
	.touchscreen_config.xedgectrl = 0,
	.touchscreen_config.xedgedist = 0,
	.touchscreen_config.yedgectrl = 0,
	.touchscreen_config.yedgedist = 0,
	.touchscreen_config.jumplimit = 18,
	.touchscreen_config.tchhyst = 10,
	.touchscreen_config.xpitch = 1,
	.touchscreen_config.ypitch = 3,
	/*noise_suppression_config*/
	.noise_suppression_config.ctrl = 0x87,
	.noise_suppression_config.reserved = 0,
	.noise_suppression_config.reserved1 = 0,
	.noise_suppression_config.reserved2 = 0,
	.noise_suppression_config.reserved3 = 0,
	.noise_suppression_config.reserved4 = 0,
	.noise_suppression_config.reserved5 = 0,
	.noise_suppression_config.reserved6 = 0,
	.noise_suppression_config.noisethr = 30,
	.noise_suppression_config.reserved7 = 0,/*1;*/
	.noise_suppression_config.freqhopscale = 0,
	.noise_suppression_config.freq[0] = 10,
	.noise_suppression_config.freq[1] = 18,
	.noise_suppression_config.freq[2] = 23,
	.noise_suppression_config.freq[3] = 30,
	.noise_suppression_config.freq[4] = 36,
	.noise_suppression_config.reserved8 = 0, /* 3 -> 0*/
	/*cte_config*/
	.cte_config.ctrl = 0,
	.cte_config.cmd = 0,
	.cte_config.mode = 0,
	/*16 -> 4 -> 8*/
	.cte_config.idlegcafdepth = 8,
	/*63 -> 16 -> 54(16ms sampling)*/
	.cte_config.actvgcafdepth = 54,
	.cte_config.voltage = 0x3c,
	/* (enable + non-locking mode)*/
	.gripsupression_config.ctrl = 0,
	.gripsupression_config.xlogrip = 0, /*10 -> 0*/
	.gripsupression_config.xhigrip = 0, /*10 -> 0*/
	.gripsupression_config.ylogrip = 0, /*10 -> 15*/
	.gripsupression_config.yhigrip = 0,/*10 -> 15*/
	.palmsupression_config.ctrl = 1,
	.palmsupression_config.reserved1 = 0,
	.palmsupression_config.reserved2 = 0,
	/* 40 -> 20(For PalmSuppression detect) */
	.palmsupression_config.largeobjthr = 10,
	/* 5 -> 50(For PalmSuppression detect) */
	.palmsupression_config.distancethr = 50,
	.palmsupression_config.supextto = 5,
	/*config change for ta connected*/
	.idleacqint_for_ta_connect = 255,
	.tchthr_for_ta_connect = 80,
	.noisethr_for_ta_connect = 50,
	.idlegcafdepth_ta_connect = 32,
	.fherr_cnt = 0,
	.fherr_chg_cnt = 10,
	.tch_blen_for_fherr = 0x11,
	.tchthr_for_fherr = 85,
	.noisethr_for_fherr = 50,
	.movefilter_for_fherr = 0x57,
	.jumplimit_for_fherr = 30,
	.freqhopscale_for_fherr = 1,
	.freq_for_fherr1[0] = 10,
	.freq_for_fherr1[1] = 12,
	.freq_for_fherr1[2] = 18,
	.freq_for_fherr1[3] = 40,
	.freq_for_fherr1[4] = 72,
	.freq_for_fherr2[0] = 45,
	.freq_for_fherr2[1] = 49,
	.freq_for_fherr2[2] = 55,
	.freq_for_fherr2[3] = 59,
	.freq_for_fherr2[4] = 63,
	.freq_for_fherr3[0] = 7,
	.freq_for_fherr3[1] = 33,
	.freq_for_fherr3[2] = 39,
	.freq_for_fherr3[3] = 52,
	.freq_for_fherr3[4] = 64,
	.fherr_cnt_no_ta = 0,
	.fherr_chg_cnt_no_ta = 1,
	.tch_blen_for_fherr_no_ta = 0,
	.tchthr_for_fherr_no_ta = 45,
	.movfilter_fherr_no_ta = 0,
	.noisethr_for_fherr_no_ta = 40,
#ifdef MXT_CALIBRATE_WORKAROUND
	/*autocal config at idle status*/
	.atchcalst_idle = 9,
	.atchcalsthr_idle = 10,
	.atchcalfrcthr_idle = 50,
	/* Atmel: 25 => 55 : avoid idle palm on lockup*/
	.atchcalfrcratio_idle = 55,
#endif
};

static const struct i2c_board_info sec_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO("sec_touch", 0x4c),
		.irq		= TEGRA_GPIO_TO_IRQ(GPIO_TOUCH_INT),
		.platform_data = &p3_touch_platform_data,

	},
};

static int __init p3_touch_init(void)
{
	p3_touch_init_hw();
	i2c_register_board_info(1, sec_i2c_touch_info,
					ARRAY_SIZE(sec_i2c_touch_info));

	return 0;
}

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
#if 0
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
#endif
			.vbus_gpio = -1,
			.usb_ldo_en = tegra_usb_ldo_en,
			.vbus_en = tegra_acc_power,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			.usb_ldo_en = tegra_usb_ldo_en,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_OTG,
			.power_down_on_bus_suspend = 0,
			.default_enable = true,
	},
#ifdef CONFIG_LINK_DEVICE_HSIC
	[1] = {
			.phy_config = &hsic_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
			.default_enable = true,
	},
#else
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
			.default_enable = true,
	},
#endif
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.hotplug = 1,
			.default_enable = true,
	},
};

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on = true,  /* use dma by default */
	.i2s_clk_rate = 5644800,
	.mode = SPDIF_BIT_MODE_MODE16BIT,
	.fifo_fmt = 0,
};


static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
	.otg_en = tegra_otg_en,
	.currentlimit_irq = TEGRA_GPIO_TO_IRQ(GPIO_V_ACCESSORY_5V),
};

static struct bcm4751_rfkill_platform_data p3_gps_rfkill_pdata = {
	.gpio_nrst = GPIO_GPS_N_RST,
	.gpio_pwr_en	= GPIO_GPS_PWR_EN,
};

static struct platform_device p3_gps_rfkill_device = {
	.name = "bcm4751_rfkill",
	.id	= -1,
	.dev	= {
		.platform_data = &p3_gps_rfkill_pdata,
	},
};

static int __init p3_gps_init(void)
{
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32,clk32->parent->rate);
		clk_enable(clk32);
	}

	tegra_gpio_enable(GPIO_GPS_N_RST);
	tegra_gpio_enable(GPIO_GPS_PWR_EN);

	return 0;
}

int	is_JIG_ON_high()
{
	return !gpio_get_value(GPIO_IFCONSENSE);
}
EXPORT_SYMBOL(is_JIG_ON_high);

static void p3_usb_init(void)
{
	int gpio_otg_en;
	int ret;

	mutex_init(&usb_data.ldo_en_lock);
	usb_data.usb_regulator_on[0] = 0;
	usb_data.usb_regulator_on[1] = 0;
	usb_data.usb_regulator_on[2] = 0;

	tegra_gpio_enable(GPIO_V_ACCESSORY_5V);
	ret = gpio_request(GPIO_V_ACCESSORY_5V, "GPIO_V_ACCESSORY_5V");
	if (ret) {
		pr_err("%s: gpio_request() for V_ACCESSORY_5V failed\n",
			__func__);
		return;
	}
	gpio_direction_input(GPIO_V_ACCESSORY_5V);

	gpio_otg_en = GPIO_OTG_EN;

	tegra_gpio_enable(gpio_otg_en);
	ret = gpio_request(gpio_otg_en, "GPIO_OTG_EN");
	if (ret) {
		pr_err("%s: gpio_request() for OTG_EN failed\n",
			__func__);
		return;
	}
	gpio_direction_output(gpio_otg_en, 0);

	tegra_gpio_enable(GPIO_ACCESSORY_EN);
	gpio_request(GPIO_ACCESSORY_EN, "GPIO_ACCESSORY_EN");
	if (ret) {
		pr_err("%s: gpio_request() for ACCESSORY_EN failed\n",
			__func__);
		return;
	}
	gpio_direction_output(GPIO_ACCESSORY_EN, 0);



	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	platform_device_register(&tegra_udc_device);
#if !defined CONFIG_LINK_DEVICE_HSIC
	platform_device_register(&tegra_ehci2_device);
#endif
#if 0
	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
#endif
}

#if defined CONFIG_LINK_DEVICE_HSIC
static int __init tegra_ehci2_hsic_init(void)
{
#ifdef CONFIG_SAMSUNG_LPM_MODE
	if (charging_mode_from_boot)
		return 0;
#endif
	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	return platform_device_register(&tegra_ehci2_device);
};
late_initcall(tegra_ehci2_hsic_init);
#endif

static void p4_check_hwrev(void)
{
	unsigned int value, rev_no, i;
	struct board_revision *board_rev;

	board_rev = p4_board_rev;
	rev_no = ARRAY_SIZE(p4_board_rev);

	gpio_request(GPIO_HW_REV0, "GPIO_HW_REV0");
	gpio_request(GPIO_HW_REV1, "GPIO_HW_REV1");
	gpio_request(GPIO_HW_REV2, "GPIO_HW_REV2");
	gpio_request(GPIO_HW_REV3, "GPIO_HW_REV3");
	gpio_request(GPIO_HW_REV4, "GPIO_HW_REV4");

	tegra_gpio_enable(GPIO_HW_REV0);
	tegra_gpio_enable(GPIO_HW_REV1);
	tegra_gpio_enable(GPIO_HW_REV2);
	tegra_gpio_enable(GPIO_HW_REV3);
	tegra_gpio_enable(GPIO_HW_REV4);

	gpio_direction_input(GPIO_HW_REV0);
	gpio_direction_input(GPIO_HW_REV1);
	gpio_direction_input(GPIO_HW_REV2);
	gpio_direction_input(GPIO_HW_REV3);
	gpio_direction_input(GPIO_HW_REV4);

	value = gpio_get_value(GPIO_HW_REV0) |
			(gpio_get_value(GPIO_HW_REV1)<<1) |
			(gpio_get_value(GPIO_HW_REV2)<<2) |
			(gpio_get_value(GPIO_HW_REV3)<<3) |
			(gpio_get_value(GPIO_HW_REV4)<<4);

	for (i = 0; i < rev_no; i++) {
		if (board_rev[i].gpio_value == value)
			break;
	}

	system_rev = (i == rev_no) ? board_rev[rev_no-1].value : board_rev[i].value;

	if (i == rev_no)
		pr_warn("%s: Valid revision NOT found! Latest one will be assigned!\n", __func__);

	pr_info("%s: system_rev = %d (gpio value = 0x%02x)\n", __func__, system_rev, value);
}

static void p3_power_off(void)
{
	int ret;
	u32 value;

	ret = tps6586x_power_off();
	if (ret)
		pr_err("p3: failed to power off\n");

	while (1)
		;
}

static void __init p3_power_off_init(void)
{
	pm_power_off = p3_power_off;
}

#ifdef CONFIG_KERNEL_DEBUG_SEC
/* Debug level control */
static ssize_t show_sec_debug_level(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int sec_debug_level = kernel_sec_get_debug_level();
	char buffer[5];
	memcpy(buffer, &sec_debug_level, 4);
	buffer[4] = '\0';
	return sprintf(buf, "%s\n", buffer);
}

static ssize_t store_sec_debug_level(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int sec_debug_level = 0;
	memcpy(&sec_debug_level, buf, 4);

	printk("%s %x\n", __func__, sec_debug_level);

	if (!(sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_LOW
			|| sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_MID
			|| sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_HIGH))
		return -EINVAL;

	kernel_sec_set_debug_level(sec_debug_level);

	return count;
}

static DEVICE_ATTR(sec_debug_level, 0644, show_sec_debug_level, store_sec_debug_level);

/* -- Debug level control */
#endif
#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
static void tdmb_gpio_on(void)
{
	printk(KERN_DEBUG "TDMB tdmb_gpio_on\n");

	nvhost_t20_boost_2d(true);

	tegra_gpio_disable(GPIO_TDMB_SPI_CS);
	tegra_gpio_disable(GPIO_TDMB_SPI_CLK);
	tegra_gpio_disable(GPIO_TDMB_SPI_MOSI);
	tegra_gpio_disable(GPIO_TDMB_SPI_MISO);

	gpio_set_value(GPIO_TDMB_EN, 1);
	usleep_range(10000, 10000);
	gpio_set_value(GPIO_TDMB_RST, 0);
	usleep_range(2000, 2000);
	gpio_set_value(GPIO_TDMB_RST, 1);
	usleep_range(10000, 10000);
}

static void tdmb_gpio_off(void)
{
	printk(KERN_DEBUG "TDMB tdmb_gpio_off\n");

	gpio_set_value(GPIO_TDMB_RST, 0);
	usleep_range(1000, 1000);
	gpio_set_value(GPIO_TDMB_EN, 0);

	tegra_gpio_enable(GPIO_TDMB_SPI_CS);
	tegra_gpio_enable(GPIO_TDMB_SPI_CLK);
	tegra_gpio_enable(GPIO_TDMB_SPI_MOSI);
	tegra_gpio_enable(GPIO_TDMB_SPI_MISO);
	gpio_set_value(GPIO_TDMB_SPI_CS, 0);
	gpio_set_value(GPIO_TDMB_SPI_CLK, 0);
	gpio_set_value(GPIO_TDMB_SPI_MOSI, 0);
	gpio_set_value(GPIO_TDMB_SPI_MISO, 0);

	nvhost_t20_boost_2d(false);
}

static struct tdmb_platform_data tdmb_pdata = {
	.gpio_on = tdmb_gpio_on,
	.gpio_off = tdmb_gpio_off,
	.irq = TEGRA_GPIO_TO_IRQ(GPIO_TDMB_INT),
};

static struct platform_device tdmb_device = {
	.name			= "tdmb",
	.id				= -1,
	.dev			= {
		.platform_data = &tdmb_pdata,
	},
};

static struct spi_board_info tegra_spi_tdmb_devices[] __initdata = {
	{
		.modalias = "tdmbspi",
		.platform_data = NULL,
		.max_speed_hz = 5000000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
};

static int __init tdmb_dev_init(void)
{
	printk(KERN_DEBUG "TDMB tdmb_dev_init\n");

	tegra_gpio_enable(GPIO_TDMB_RST);
	tegra_gpio_enable(GPIO_TDMB_EN);
	tegra_gpio_enable(GPIO_TDMB_INT);

	gpio_request(GPIO_TDMB_EN, "TDMB_EN");
	gpio_direction_output(GPIO_TDMB_EN, 0);
	gpio_request(GPIO_TDMB_RST, "TDMB_RST");
	gpio_direction_output(GPIO_TDMB_RST, 0);
	gpio_request(GPIO_TDMB_INT, "TDMB_INT");
	gpio_direction_input(GPIO_TDMB_INT);

	platform_device_register(&tdmb_device);

	if (spi_register_board_info(tegra_spi_tdmb_devices,
		ARRAY_SIZE(tegra_spi_tdmb_devices)) != 0) {
		pr_err("%s: spi_register_board_info error\n", __func__);
	}
	return 0;
}
#endif
static void __init tegra_p3_init(void)
{
	struct board_info BoardInfo;
	int ret = 0;

	if(cmc623_current_type) {
		tegra_clk_init_from_table(p3_clk_init_tbl_pclk_76);
	}
	else {
		tegra_clk_init_from_table(p3_clk_init_tbl_pclk_68);
	}
	p3_pinmux_init();
	p3_i2c_init();
	p4_uart_init();
#if !defined CONFIG_LINK_DEVICE_HSIC
	tegra_ehci2_device.dev.platform_data
		= &p4_ehci2_ulpi_platform_data;
#endif
	tegra_spdif_device.dev.platform_data = &tegra_spdif_pdata;
	platform_add_devices(p3_devices, ARRAY_SIZE(p3_devices));
	tegra_ram_console_debug_init();
	p3_sdhci_init();
	p3_gpio_i2c_init();
	p3_camera_init();
	p3_regulator_init();

	tegra_get_board_info(&BoardInfo);

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create sec class!\n");

	p4_check_hwrev();
	p4_jack_init();
#ifdef CONFIG_SAMSUNG_LPM_MODE
	if (!charging_mode_from_boot) {
		p3_touch_init();
	} else {
		p3_touch_init_hw();
		p3_touch_suspend_hw();
	}
#else
	p3_touch_init();
#endif
	p3_keys_init();
	p3_usb_init();
	p3_gps_init();
	p3_panel_init();
	p3_sensors_init();
	p3_power_off_init();
	p3_emc_init();
	tegra_release_bootloader_fb();

	register_reboot_notifier(&p3_reboot_notifier);

#ifdef CONFIG_KERNEL_DEBUG_SEC
	/* Add debug level node */
	struct device *platform = p3_devices[0]->dev.parent;
	ret = device_create_file(platform, &dev_attr_sec_debug_level);
	if (ret)
		printk("Fail to create sec_debug_level file\n");
#endif
#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
	tdmb_dev_init();
#endif

}

int __init tegra_p4_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_p4_protected_aperture_init);

void __init tegra_p3_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M + SZ_1M, SZ_16M);
	tegra_ram_console_debug_reserve(SZ_1M);
}

#ifdef CONFIG_TARGET_LOCALE_KOR
MACHINE_START(SAMSUNG_P3, MODELNAME)
	.boot_params    = 0x00000100,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_p3_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_p3_reserve,
	.timer          = &tegra_timer,
MACHINE_END
#else
MACHINE_START(SAMSUNG_P3, "p3")
	.boot_params    = 0x00000100,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.init_machine	= tegra_p3_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_p3_reserve,
	.timer          = &tegra_timer,
MACHINE_END
#endif

static int __init setup_cmc623_type(char *str)
{
    printk("~~~~~~~~~~~~~setup cmc623 type ~~~~~~~~~~~~~~\n");
    cmc623_current_type = 1;
	return 0;
}
__setup("CMC623F",setup_cmc623_type);
