/*
 * arch/arm/mach-tegra/board-p4-panel.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/gpio-sec.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"

#define FB_X_RES 1280
#define FB_Y_RES 800
#define FB_BITS_PER_PIXEL 32

#define p4_pnl_pwr_enb	TEGRA_GPIO_PC6
#define p4_bl_enb		TEGRA_GPIO_PD4
#define p4_lvds_shutdown	TEGRA_GPIO_PB2
#define p4_hdmi_hpd	TEGRA_GPIO_PN7
#define p4_hdmi_enb	TEGRA_GPIO_PV5

/*panel power on sequence timing*/
#define p4_pnl_to_lvds_ms	0
#define p4_lvds_to_bl_ms	200

#ifdef CONFIG_TEGRA_DC
static struct regulator *p3_hdmi_reg = NULL;
static struct regulator *p3_hdmi_pll = NULL;
#endif

static struct platform_device p3_backlight_device = {
	.name	= "cmc623_pwm_bl",
	.id	= -1,
};

static int p4_disp1_check_fb(struct device *dev, struct fb_info *info);

#ifdef CONFIG_TEGRA_DC
static int p4_panel_enable(void)
{
	struct regulator *reg = regulator_get(NULL, "vdd_ldo4");

	if (!reg) {
		regulator_enable(reg);
		regulator_put(reg);
	}

	gpio_set_value(p4_pnl_pwr_enb, 1);
	mdelay(p4_pnl_to_lvds_ms);
	gpio_set_value(p4_lvds_shutdown, 1);
	mdelay(p4_lvds_to_bl_ms);
	return 0;
}

static int p4_panel_disable(void)
{
	gpio_set_value(p4_lvds_shutdown, 0);
	gpio_set_value(p4_pnl_pwr_enb, 0);
	return 0;
}

static int p3_hdmi_enable(void)
{
#if 0
	if (!p3_hdmi_reg) {
		p3_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD07 */
		if (IS_ERR_OR_NULL(p3_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			p3_hdmi_reg = NULL;
			return PTR_ERR(p3_hdmi_reg);
		}
	}
	regulator_enable(p3_hdmi_reg);

	if (!p3_hdmi_pll) {
		p3_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD08 */
		if (IS_ERR_OR_NULL(p3_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			p3_hdmi_pll = NULL;
			regulator_disable(p3_hdmi_reg);
			p3_hdmi_reg = NULL;
			return PTR_ERR(p3_hdmi_pll);
		}
	}
	regulator_enable(p3_hdmi_pll);
#endif
	return 0;
}

static int p3_hdmi_disable(void)
{
#if 0
	regulator_disable(p3_hdmi_reg);
	regulator_disable(p3_hdmi_pll);
#endif
	return 0;
}

static struct resource p3_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource p3_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

#if defined(CONFIG_TARGET_LOCALE_KOR)

static struct tegra_dc_mode p3_panel_modes[] = {
	{          /* SAMSULG PLS LCD panel */
		.pclk = 72000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 48,
		.v_sync_width = 3,
		.h_back_porch = 88,
		.v_back_porch = 34-1,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 16,
		.v_front_porch = 1+1,
		.flags = TEGRA_DC_MODE_FLAG_NEG_V_SYNC
			| TEGRA_DC_MODE_FLAG_NEG_H_SYNC,
	},
};

#else // kyNam_110614_
static struct tegra_dc_mode p3_panel_modes_pclk_68[] = {
	{	/* SAMSULG PLS LCD panel */
		.pclk = 68941176, /* for lcd pclk 68.94Mhz */
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 48,
		.v_sync_width = 3,
		.h_back_porch = 64,
		.v_back_porch = 11,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 16,
		.v_front_porch = 2,
		.flags = TEGRA_DC_MODE_FLAG_NEG_V_SYNC
			| TEGRA_DC_MODE_FLAG_NEG_H_SYNC,
	},
};
#endif

static struct tegra_dc_mode p3_panel_modes_pclk_76[] = {
	{	/* SAMSULG PLS LCD panel */
		.pclk = 76000000, /* for lcd pclk 76Mhz */
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 48,
		.v_sync_width = 18,
		.h_back_porch = 64,
		.v_back_porch = 76,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 16,
		.v_front_porch = 6,
		.flags = TEGRA_DC_MODE_FLAG_NEG_V_SYNC
			| TEGRA_DC_MODE_FLAG_NEG_H_SYNC,
	},
};



static struct tegra_fb_data p3_fb_data = {
	.win		= 0,
	.xres		= FB_X_RES,
	.yres		= FB_Y_RES,
	.bits_per_pixel	= FB_BITS_PER_PIXEL,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data p3_hdmi_fb_data = {
	.win		= 0,
	.xres		= FB_X_RES,
	.yres		= FB_Y_RES,
	.bits_per_pixel	= FB_BITS_PER_PIXEL,
};

#ifdef CONFIG_KERNEL_DEBUG_SEC
struct struct_frame_buf_mark {
	u32 special_mark_1;
	u32 special_mark_2;
	u32 special_mark_3;
	u32 special_mark_4;
	void *p_fb;/*it must be physical address*/
	u32 resX;
	u32 resY;
	u32 bpp; /*color depth : 16 or 24*/
	u32 frames; /*frame buffer count : 2*/
};
static struct struct_frame_buf_mark frame_buf_mark = {
	.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0)),
	.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0)),
	.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0)),
	.special_mark_4 = (('f' << 24) | ('b' << 16) | ('u' << 8) | ('f' << 0)),
	.p_fb = 0,
	.resX = 1280, /* it has dependency on h/w */
	.resY = 800, /* it has dependency on h/w */
	.bpp = 32, /* it has dependency on h/w */
	.frames = 2
};
#endif

#if defined(CONFIG_TARGET_LOCALE_KOR)
static struct tegra_dc_out p3_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 24,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= p3_panel_modes,
	.n_modes	= ARRAY_SIZE(p3_panel_modes),

};
#else
static struct tegra_dc_out p3_disp1_out_pclk_68 = {
	.type		= TEGRA_DC_OUT_RGB,
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 24,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= p3_panel_modes_pclk_68,
	.n_modes	= ARRAY_SIZE(p3_panel_modes_pclk_68),

	/* P4 */
	.width		= 217,	/* in mm */
	.height		= 135,	/* in mm */
};
#endif

static struct tegra_dc_out p3_disp1_out_pclk_76 = {
	.type		= TEGRA_DC_OUT_RGB,
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 24,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes	 	= p3_panel_modes_pclk_76,
	.n_modes 	= ARRAY_SIZE(p3_panel_modes_pclk_76),

	/* P4 */
	.width		= 217,	/* in mm */
	.height		= 135,	/* in mm */
};



static struct tegra_dc_out p3_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 13,
	.hotplug_gpio	= p4_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(74250),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= p3_hdmi_enable,
	.disable	= p3_hdmi_disable,
};

#if defined(CONFIG_TARGET_LOCALE_KOR)

static struct tegra_dc_platform_data p3_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &p3_disp1_out,
	.fb		= &p3_fb_data,
};

#else

/* default pdata for PCLK 68.9Mhz For CMC623 */
static struct tegra_dc_platform_data p3_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &p3_disp1_out_pclk_68,
	.fb		= &p3_fb_data,
};

#endif

/* default pdata for PCLK 76Mhz For CMC623F */
static struct tegra_dc_platform_data p3_disp1_pdata_pclk_76 = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &p3_disp1_out_pclk_76,
	.fb		= &p3_fb_data,
};


static struct tegra_dc_platform_data p3_disp2_pdata = {
	.flags		= 0,
	.default_out	= &p3_disp2_out,
	.fb		= &p3_hdmi_fb_data,
};

static struct nvhost_device p3_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= p3_disp1_resources,
	.num_resources	= ARRAY_SIZE(p3_disp1_resources),
	.dev = {
		.platform_data = &p3_disp1_pdata,
	},
};

static int p4_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &p3_disp1_device.dev;
}

static struct nvhost_device p3_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= p3_disp2_resources,
	.num_resources	= ARRAY_SIZE(p3_disp2_resources),
	.dev = {
		.platform_data = &p3_disp2_pdata,
	},
};
#else
static int p4_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout p3_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data p3_nvmap_data = {
	.carveouts	= p3_carveouts,
	.nr_carveouts	= ARRAY_SIZE(p3_carveouts),
};

static struct platform_device p3_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &p3_nvmap_data,
	},
};
#endif

static struct platform_device p3_device_cmc623 = {
		.name			= "sec_cmc623",
		.id			= -1,
};

static struct platform_device *p3_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&p3_nvmap_device,
#endif
	&tegra_pwfm2_device,
	&p3_backlight_device,
	&p3_device_cmc623,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend p3_panel_early_suspender;

static void p3_panel_early_suspend(struct early_suspend *h)
{
	unsigned i;

	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
}

static void p3_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

extern int cmc623_current_type;
int __init p3_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

	tegra_gpio_enable(GPIO_HDMI_HPD);
	gpio_request(GPIO_HDMI_HPD, "hdmi_hpd");
	gpio_direction_input(GPIO_HDMI_HPD);

#ifdef CONFIG_HAS_EARLYSUSPEND
	p3_panel_early_suspender.suspend = p3_panel_early_suspend;
	p3_panel_early_suspender.resume = p3_panel_late_resume;
	p3_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&p3_panel_early_suspender);
#endif

#if defined(CONFIG_TEGRA_NVMAP)
	p3_carveouts[1].base = tegra_carveout_start;
	p3_carveouts[1].size = tegra_carveout_size;
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = nvhost_device_register(&tegra_grhost_device);
	if (err)
		return err;
#endif

	err = platform_add_devices(p3_gfx_devices,
				   ARRAY_SIZE(p3_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&p3_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&p3_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif

	if(cmc623_current_type){
		printk("CMC623 INFO : Integrated CMC623F From FUJITSU\n");
		p3_disp1_device.dev.platform_data = &p3_disp1_pdata_pclk_76;
		//printk("CMC623 INFO : Set PLCK to %d\n",p3_disp1_device.dev.platform_data.default_out.modes.pclk);
	}
	else {
		printk("CMC623 INFO : Integrated CMC623 From S.LSI\n");
		//printk("CMC623 INFO : Set PLCK to %d\n",p3_disp1_device.dev.platform_data.default_out.modes.pclk);
	}


	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));


#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&p3_disp1_device);

	/* Rev 01 devices don't have working hdmi?  or the params
	 * aren't right for Rev 01.  Disable to allow Rev 01 to boot
	 */
	if (system_rev >= 0x2) {
		/* HDMI FB */
		if (!err)
			err = nvhost_device_register(&p3_disp2_device);
	}
#endif

#ifdef CONFIG_KERNEL_DEBUG_SEC
	frame_buf_mark.bpp = p3_fb_data.bits_per_pixel;
	/*it has dependency on h/w*/
	frame_buf_mark.p_fb = (void *)p3_disp1_resources[2].start;
	/*it has dependency on project*/
#endif

	return err;
}

