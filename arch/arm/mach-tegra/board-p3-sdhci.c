/*
 * arch/arm/mach-tegra/board-p3-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/skbuff.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include <mach/gpio-p3.h>
#include "gpio-names.h"
#include "board.h"

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define WLAN_STATIC_SCAN_BUF0			5
#define WLAN_STATIC_SCAN_BUF1			6
#define PREALLOC_WLAN_SEC_NUM			4
#define PREALLOC_WLAN_BUF_NUM			160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;
	printk(KERN_INFO"%s: WIFI MEM Allocated\n", __func__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define GPIO_WLAN_EN	TEGRA_GPIO_PQ2
struct platform_device *tegra_wlan_pdevice;
EXPORT_SYMBOL(tegra_wlan_pdevice);

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int p3_wifi_status_register(void (*callback)(int , void *), void *);
static struct clk *wifi_32k_clk;

static int p3_wifi_reset(int on);
static int p3_wifi_power(int on);
static int p3_wifi_set_carddetect(int val);

static struct wifi_platform_data p3_wifi_control = {
	.set_power	= p3_wifi_power,
	.set_reset	= p3_wifi_reset,
	.set_carddetect = p3_wifi_set_carddetect,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc = brcm_wlan_mem_prealloc,
#endif
};

static struct resource wifi_resource[] = {
	[0] = {
		.name  = "bcmdhd_wlan_irq",
		.start = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0),
		.end   = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device p3_wifi_device = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources  = 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &p3_wifi_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start	= INT_SDMMC1,
		.end	= INT_SDMMC1,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start	= INT_SDMMC3,
		.end	= INT_SDMMC3,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start	= INT_SDMMC4,
		.end	= INT_SDMMC4,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
	.cis  = {
		.vendor 	= 0x02d0,
		.device 	= 0x4329,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= p3_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data0,
		.built_in = 0,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = TEGRA_GPIO_PI5,
	.wp_gpio = TEGRA_GPIO_PH1,
	.power_gpio = TEGRA_GPIO_PT3,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.is_8bit = 1,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.mmc_data = {
		.built_in = 1,
	}
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int p3_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	printk(KERN_INFO "%s: start\n", __func__);
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int p3_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: start\n", __func__);
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int p3_wifi_power(int on)
{
	printk(KERN_INFO "%s: start\n", __func__);
	pr_debug("%s: %d\n", __func__, on);

	gpio_set_value(GPIO_WLAN_EN, on);
	mdelay(100);

	if (on)
		clk_enable(wifi_32k_clk);
	else
		clk_disable(wifi_32k_clk);

	return 0;
}

static int p3_wifi_reset(int on)
{
	printk(KERN_INFO "%s: start\n", __func__);
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init p3_wifi_init(void)
{
	printk(KERN_INFO "%s: start\n", __func__);
	wifi_32k_clk = clk_get_sys(NULL, "blink");
	if (IS_ERR(wifi_32k_clk)) {
		pr_err("%s: unable to get blink clock\n", __func__);
		return PTR_ERR(wifi_32k_clk);
	}

	gpio_request(GPIO_WLAN_EN, "wlan_power");
	tegra_gpio_enable(GPIO_WLAN_EN);
	gpio_direction_output(GPIO_WLAN_EN, 0);

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif

	platform_device_register(&p3_wifi_device);

	device_init_wakeup(&p3_wifi_device.dev, 1);
	device_set_wakeup_enable(&p3_wifi_device.dev, 0);

	return 0;
}
int __init p3_sdhci_init(void)
{
	printk(KERN_INFO "%s: start\n", __func__);
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device0);

	tegra_wlan_pdevice = &tegra_sdhci_device0;

	p3_wifi_init();
	return 0;
}
