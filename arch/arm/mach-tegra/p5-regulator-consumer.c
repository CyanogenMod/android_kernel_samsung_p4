/* p5-regulator-consumer.c
 *
 * Copyright (C) 2011 Samsung Electronics
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
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>

static int p5_enable_regulator_consumer(bool enable)
{
	struct regulator *vdd_ldo3_regulator;
	int ret = 0;

	vdd_ldo3_regulator = regulator_get(NULL, "vdd_ldo3");
	if (IS_ERR(vdd_ldo3_regulator)) {
		pr_err("%s: failed to get %s\n", __func__, "vdd_ldo3");
		ret = -ENODEV;
		goto out1;
	}

	if (system_rev >= 9) {
		if (enable) {
			/* Power On Sequence
			 *
			 */
			pr_info("%s: enable LDOs\n", __func__);
			if (!regulator_is_enabled(vdd_ldo3_regulator))
				regulator_enable(vdd_ldo3_regulator);
		} else {
			/* Power Off Sequence
			 *
			 */
			pr_info("%s: disable LDOs\n", __func__);
			regulator_force_disable(vdd_ldo3_regulator);
		}
	}

	regulator_put(vdd_ldo3_regulator);
out1:
	return ret;
}


static int regulator_consumer_probe(struct platform_device *pdev)
{

	pr_info("%s: loading p5-regulator-consumer\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
static int regulator_consumer_suspend(struct device *dev)
{
	/*p5_enable_regulator_consumer(false);*/
	/* disable for charger icon problem maybe sleep current 80uA increase*/
	return 0;
}

static int regulator_consumer_resume(struct device *dev)
{
	/*p5_enable_regulator_consumer(true);*/
	/* disable for charger icon problem maybe sleep current 80uA increase*/

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops regulator_consumer_pm_ops = {
#ifdef CONFIG_PM
	.suspend = regulator_consumer_suspend,
	.resume = regulator_consumer_resume,
#endif /* CONFIG_PM */
};

static struct platform_driver regulator_consumer_driver = {
	.probe = regulator_consumer_probe,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "p5-regulator-consumer",
		   .pm = &regulator_consumer_pm_ops,
	},
};

static int __init regulator_consumer_init(void)
{
	return platform_driver_register(&regulator_consumer_driver);
}
module_init(regulator_consumer_init);

MODULE_AUTHOR("ms925.kim@samsung.com");
MODULE_AUTHOR("sungsup0.lim@samsung.com");
MODULE_DESCRIPTION("P5 regulator consumer driver");
MODULE_LICENSE("GPL");
