/* /linux/drivers/misc/modem_if/modem_modemctl_device_cmc220.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#define DEBUG

#include <linux/init.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_link_device_usb.h"
#define HOST_WUP_LEVEL 1


static void mc_state_fsm(struct modem_ctl *mc)
{
	int cp_reset  = gpio_get_value(mc->gpio_cp_reset);
	int cp_active = gpio_get_value(mc->gpio_phone_active);
	int old_state = mc->phone_state;
	int new_state = mc->phone_state;

	mif_err("old_state = %d, cp_reset = %d, cp_active = %d\n",
			old_state, cp_reset, cp_active);
	if (cp_active) {
		if (old_state == STATE_CRASH_EXIT) {
			mif_info("LTE DUMP END!!!\n");
			mif_err("LTE DUMP END!!!\n");
			mif_err("new_state = OFFLINE\n");
		} else if (old_state == STATE_BOOTING) {
			new_state = STATE_ONLINE;
			mif_err("new_state = ONLINE\n");
		} else {
			mif_err("Don't care!!!\n");
		}
	} else {
		if (old_state == STATE_ONLINE) {
			new_state = STATE_CRASH_EXIT;
			mif_info("LTE CRASHED!!!\n");
			mif_err("LTE CRASHED!!!\n");
			mif_err("new_state = CRASH_EXIT\n");
		} else {
			mif_err("Don't care!!!\n");
		}
	}

	if (old_state != new_state)
		mc->iod->modem_state_changed(mc->iod, new_state);
}

static void mc_work(struct work_struct *work_arg)
{
	struct modem_ctl *mc = NULL;

	mc = container_of(work_arg, struct modem_ctl, dwork.work);

	mc_state_fsm(mc);
}

static irqreturn_t modemctl_resume_irq(int irq, void *dev_id)
{
	struct modem_ctl *mc = (struct modem_ctl *)dev_id;
	int val = gpio_get_value(mc->gpio_host_wakeup);

	if (val != HOST_WUP_LEVEL) {

		if (gpio_get_value(mc->gpio_host_active) != 0) {
			gpio_set_value(mc->gpio_slave_wakeup, 1);
			printk(KERN_INFO "> S- WUP %d\n",
				gpio_get_value(mc->gpio_slave_wakeup));
		}

		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static irqreturn_t phone_active_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	int cp_reset = gpio_get_value(mc->gpio_cp_reset);
	printk(KERN_ERR "CMC220 phone_active_handler: %d\n", cp_reset);
	if (cp_reset)
		schedule_delayed_work(&mc->dwork, 1);

	return IRQ_HANDLED;
}

static int cmc220_on(struct modem_ctl *mc)
{
	mif_info("on\n");
	mif_err("device = %s\n", mc->bootd->name);
	if (!mc->gpio_cp_off || !mc->gpio_cp_on || !mc->gpio_cp_reset)
		return 0;

	gpio_set_value(mc->gpio_cp_on, 1);
	msleep(300);

	gpio_set_value(mc->gpio_cp_reset, 1);
	msleep(300);


	mc->phone_state = STATE_OFFLINE;

	return 0;
}

static int cmc220_off(struct modem_ctl *mc)
{
	mif_info("off\n");
	mif_err("<%s>\n", mc->bootd->name);

	if (!mc->gpio_cp_off || !mc->gpio_cp_on || !mc->gpio_cp_reset)
		return 0;

	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	mdelay(300);

	gpio_set_value(mc->gpio_cp_off, 0);
	mdelay(300);


	mc->phone_state = STATE_OFFLINE;

	return 0;
}

static int cmc220_force_crash_exit(struct modem_ctl *mc)
{

	mif_info("crash_exit\n");
	mif_err("<%s>\n", mc->bootd->name);

	mc->iod->modem_state_changed(mc->iod, STATE_CRASH_EXIT);

	return 0;
}

static int cmc220_dump_reset(struct modem_ctl *mc)
{
	mif_info("dump_reset\n");
	mif_err("<%s>\n", mc->bootd->name);

	gpio_set_value(mc->gpio_host_active, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);

	udelay(200);

	gpio_set_value(mc->gpio_cp_reset, 1);

	msleep(300);

	mc->phone_state = STATE_BOOTING;
	return 0;
}

static int cmc220_reset(struct modem_ctl *mc)
{
	mif_info("reset\n");
/*
	if (cmc220_off(mc))
		return -ENXIO;

	msleep(100);

	if (cmc220_on(mc))
		return -ENXIO;
*/
	gpio_set_value(mc->gpio_host_active, 1);
	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(100);
	gpio_set_value(mc->gpio_cp_reset, 1);
	msleep(100);
	mc->phone_state = STATE_BOOTING;
	return 0;
}

static int cmc220_boot_on(struct modem_ctl *mc)
{
	mif_debug("\n");
	mif_err("<%s>\n", mc->bootd->name);

	mif_err("phone_state = STATE_BOOTING\n");
	mc->iod->modem_state_changed(mc->iod, STATE_BOOTING);

	return 0;
}

static int cmc220_boot_off(struct modem_ctl *mc)
{
	mif_debug("\n");
	mif_err("\n");
	return 0;
}

static void cmc220_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on = cmc220_on;
	mc->ops.modem_off = cmc220_off;
	mc->ops.modem_reset = cmc220_reset;
	mc->ops.modem_boot_on = cmc220_boot_on;
	mc->ops.modem_boot_off = cmc220_boot_off;
	mc->ops.modem_force_crash_exit = cmc220_force_crash_exit;
	mc->ops.modem_dump_reset = cmc220_dump_reset;
}

int cmc220_init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	int ret = 0;
	int irq = 0;
	unsigned long flag = 0;
	struct platform_device *pdev = NULL;

	mc->gpio_cp_on        = pdata->gpio_cp_on;
	mc->gpio_cp_off       = pdata->gpio_cp_off;
	mc->gpio_cp_reset     = pdata->gpio_cp_reset;
	mc->gpio_phone_active = pdata->gpio_phone_active;
	/*TODO: check the GPIO map*/
	mc->gpio_pda_active   = pdata->gpio_pda_active;
	mc->gpio_cp_dump_int  = pdata->gpio_cp_dump_int;
	mc->gpio_flm_uart_sel = pdata->gpio_flm_uart_sel;
	mc->gpio_slave_wakeup = pdata->gpio_slave_wakeup;
	mc->gpio_host_active  = pdata->gpio_host_active;
	mc->gpio_host_wakeup  = pdata->gpio_host_wakeup;


	if (!mc->gpio_cp_on || !mc->gpio_cp_reset || !mc->gpio_phone_active) {
		mif_err("no GPIO data\n");
		return -ENXIO;
	}

	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_cp_on, 0);
	if (mc->gpio_cp_off)
		gpio_set_value(mc->gpio_cp_off, 1);

	cmc220_get_ops(mc);
	dev_set_drvdata(mc->dev, mc);
	INIT_DELAYED_WORK(&mc->dwork, mc_work);

	pdev = to_platform_device(mc->dev);
	mc->irq_phone_active = platform_get_irq_byname(pdev, "lte_phone_active");
	if (!mc->irq_phone_active) {
		mif_err("get irq fail\n");
		return -1;
	}

	irq = mc->irq_phone_active;
	mif_err("PHONE_ACTIVE IRQ# = %d\n", irq);

	flag = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND;
	/* flag |= IRQF_NO_SUSPEND; */
	ret = request_irq(irq, phone_active_handler, flag, "cmc_active", mc);
	if (ret) {
		mif_err("request_irq fail (%d)\n", ret);
		return ret;
	}


/*	disable_irq(irq); */
	ret = enable_irq_wake(irq);
	if (ret)
		mif_err("enable_irq_wake fail (%d)\n", ret);

	mif_err("IRQ#%d handler is registered.\n", irq);

	return 0;
}
