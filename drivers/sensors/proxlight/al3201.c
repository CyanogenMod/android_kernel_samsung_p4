/*
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define LSC_DBG

#define SENSOR_AL3201_ADDR	0x1c
#define SENSOR_BH1721FVC_ADDR	0x23

#define al3201_DRV_NAME		"AL3201"
#define DRIVER_VERSION		"1.0"

#define LUX_MIN_VALUE		0

#if defined(CONFIG_MACH_SAMSUNG_P5)
#define LUX_MAX_VALUE		70000
#define LIMIT_RESET_COUNT	5
#else
#define LUX_MAX_VALUE		65528
#endif

#if defined(CONFIG_MACH_SAMSUNG_P4TMO)
#define LIGHT_UPWORD_BUFFER		5
#define LIGHT_DOWNWORD_BUFFER	20
#endif

#define AL3201_RAN_COMMAND	0x01
#define AL3201_RAN_MASK		0x01
#define AL3201_RAN_SHIFT	(0)
#define AL3201_RST_MASK		0x03
#define AL3201_RST_SHIFT	(2)

#define AL3201_RT_COMMAND	0x02
#define AL3201_RT_MASK		0x03
#define AL3201_RT_SHIFT	(0)

#define AL3201_POW_COMMAND	0x00
#define AL3201_POW_MASK		0x03
#define AL3201_POW_UP		0x03
#define AL3201_POW_DOWN		0x00
#define AL3201_POW_SHIFT	(0)

#define	AL3201_ADC_LSB		0x0c
#define	AL3201_ADC_MSB		0x0d

#define AL3201_NUM_CACHABLE_REGS	5

static const u8 al3201_reg[AL3201_NUM_CACHABLE_REGS] = {
	0x00,
	0x01,
	0x02,
	0x0c,
	0x0d
};

#if defined(CONFIG_MACH_SAMSUNG_P4TMO)
static const int adc_table[5] = {
	20,
	180,
	1750,
	17000,
	65000,
};
#endif

enum SENSOR_STATE {
	OFF = 0,
	ON = 1,
};

struct al3201_data {
	struct i2c_client *client;
	struct mutex lock;
	struct input_dev *input;
	struct work_struct work_light;
	struct workqueue_struct *wq;
	struct hrtimer timer;
	struct class *factory_class;
	struct device *factory_dev;

	ktime_t light_poll_delay;

	u8 reg_cache[AL3201_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;

	int state;

#if defined(CONFIG_MACH_SAMSUNG_P5)
	int reset_cnt;
	int zero_cnt;
#endif

#if defined(CONFIG_MACH_SAMSUNG_P4TMO)
	int light_count;
	int light_buffer;
	int light_buffer_cnt;
	int light_level_state;
#endif

};

/*
 * register access helpers
 */
static int al3201_read_reg(struct i2c_client *client,
			   u32 reg, u8 mask, u8 shift)
{
	struct al3201_data *data = i2c_get_clientdata(client);

	return (data->reg_cache[reg] & mask) >> shift;
}

static int al3201_write_reg(struct i2c_client *client,
			    u32 reg, u8 mask, u8 shift, u8 val)
{
	u8 tmp;
	int ret = 0;
	struct al3201_data *data = i2c_get_clientdata(client);

	if (reg >= AL3201_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;
	else
		pr_err("%s: I2C read failed!\n", __func__);

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3201_sw_reset(struct i2c_client *client)
{
	al3201_write_reg(client, AL3201_RAN_COMMAND,
			 AL3201_RST_MASK, AL3201_RST_SHIFT, 0x02);
	msleep(20);
	al3201_write_reg(client, AL3201_RAN_COMMAND,
			 AL3201_RST_MASK, AL3201_RST_SHIFT, 0x00);
	return 0;
}

static int al3201_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = al3201_read_reg(client, AL3201_RAN_COMMAND,
			      AL3201_RAN_MASK, AL3201_RAN_SHIFT);
	return tmp;
}

static int al3201_set_range(struct i2c_client *client, int range)
{
	return al3201_write_reg(client, AL3201_RAN_COMMAND,
				AL3201_RAN_MASK, AL3201_RAN_SHIFT, range);
}

/* Response time */
static int al3201_set_response_time(struct i2c_client *client, int time)
{
	return al3201_write_reg(client, AL3201_RT_COMMAND,
				AL3201_RT_MASK, AL3201_RT_SHIFT, time);
}

/* power_state */
static int al3201_set_power_state(struct i2c_client *client, int state)
{
	return al3201_write_reg(client, AL3201_POW_COMMAND,
				AL3201_POW_MASK, AL3201_POW_SHIFT,
				state ? AL3201_POW_UP : AL3201_POW_DOWN);
}

static int al3201_get_power_state(struct i2c_client *client)
{
	u8 cmdreg;
	struct al3201_data *data = i2c_get_clientdata(client);

	cmdreg = data->reg_cache[AL3201_POW_COMMAND];
	return (cmdreg & AL3201_POW_MASK) >> AL3201_POW_SHIFT;
}

/* power & timer enable */
static int al3201_enable(struct al3201_data *data)
{
	int err;

#if defined(CONFIG_MACH_SAMSUNG_P4TMO)
	data->light_buffer = 0;
	data->light_count = 0;
	data->light_level_state = 0;
	data->light_buffer_cnt = LIGHT_UPWORD_BUFFER;
#endif

#if defined(CONFIG_MACH_SAMSUNG_P5)
	data->reset_cnt = 0;
	data->zero_cnt = 0;
#endif

	err = al3201_set_power_state(data->client, ON);
	if (err) {
		pr_err("%s: Failed to write byte (POWER_UP)\n", __func__);
		return err;
	}

	hrtimer_start(&data->timer, data->light_poll_delay, HRTIMER_MODE_REL);

	return err;
}

/* power & timer disable */
static int al3201_disable(struct al3201_data *data)
{
	int err;

	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work_light);

	err = al3201_set_power_state(data->client, OFF);
	if (unlikely(err != 0))
		pr_err("%s: Failed to write byte (POWER_DOWN)\n", __func__);

	return err;
}

static int al3201_get_adc_value(struct i2c_client *client)
{
	int lsb, msb, range;
	u32 val;
	struct al3201_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);

	lsb = i2c_smbus_read_byte_data(client, AL3201_ADC_LSB);
	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3201_ADC_MSB);

	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = al3201_get_range(client);
	val = (u32) (msb << 8 | lsb);

	return val;
}

static void al3201_work_func_light(struct work_struct *work)
{
	int i;
	u32 result = 0;

	struct al3201_data *data =
	    container_of(work, struct al3201_data, work_light);

	result = al3201_get_adc_value(data->client);

#if defined(CONFIG_MACH_SAMSUNG_P5)
	if (result > 69000)
		result = 69000;
#else
	if (result > 60000)
		result = 60000;
#endif

#if defined(CONFIG_MACH_SAMSUNG_P4TMO)
	for (i = 0; ARRAY_SIZE(adc_table); i++)
		if (result <= adc_table[i])
			break;

	if (data->light_buffer == i) {
		if (data->light_count++ >= data->light_buffer_cnt) {
			input_report_rel(data->input, REL_MISC, result + 1);
			input_sync(data->input);
			data->light_level_state = i;
			data->light_count = 0;
		}
	} else {
		data->light_buffer = i;
		data->light_count = 0;

		if (data->light_level_state >= i)
			data->light_buffer_cnt = LIGHT_DOWNWORD_BUFFER;
		else
			data->light_buffer_cnt = LIGHT_UPWORD_BUFFER;
	}
#else
	input_report_rel(data->input, REL_MISC, result + 1);
	input_sync(data->input);
#endif

#if defined(CONFIG_MACH_SAMSUNG_P5)
	if (result == 0) {
		if (data->zero_cnt++ > 25) {
			data->zero_cnt = 0;
			if (data->reset_cnt++ <= LIMIT_RESET_COUNT)
				al3201_sw_reset(data);
			else
				data->reset_cnt == LIMIT_RESET_COUNT + 1;
		}
	} else {
		data->reset_cnt = 0;
		data->zero_cnt = 0;
	}
#endif

}

static enum hrtimer_restart al3201_timer_func(struct hrtimer *timer)
{
	struct al3201_data *data =
	    container_of(timer, struct al3201_data, timer);

	queue_work(data->wq, &data->work_light);
	hrtimer_forward_now(&data->timer, data->light_poll_delay);

	return HRTIMER_RESTART;
}

/*
 * sysfs layer
 */

/* factory test*/
static ssize_t lightsensor_file_cmd_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "\n");
}

static ssize_t lightsensor_file_cmd_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(lightsensor_file_cmd, S_IRUGO | S_IWUSR | S_IWGRP,
		   lightsensor_file_cmd_show, lightsensor_file_cmd_store);

static ssize_t factory_file_illuminance_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	int result;
	struct input_dev *input = to_input_dev(dev);
	struct al3201_data *data = input_get_drvdata(input);

	/* No LUX data if not operational */
	if (data->state == OFF) {
		al3201_set_power_state(data->client, ON);
		msleep(180);
	}

	result = al3201_get_adc_value(data->client);

	if (data->state == OFF)
		al3201_set_power_state(data->client, OFF);

	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(lightsensor_file_illuminance, S_IRUGO,
		   factory_file_illuminance_show, NULL);

static ssize_t sensor_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_AL3201_ADDR);
}

static DEVICE_ATTR(sensor_info, S_IRUGO, sensor_info_show, NULL);

#ifdef LSC_DBG
/* engineer mode */
static ssize_t al3201_em_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	u8 tmp;
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct al3201_data *data = input_get_drvdata(input);

	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		mutex_lock(&data->lock);
		tmp = i2c_smbus_read_byte_data(data->client, al3201_reg[i]);
		mutex_unlock(&data->lock);

		printk(KERN_INFO "Reg[0x%x] Val[0x%x]\n", al3201_reg[i], tmp);
	}
	return 0;
}

static ssize_t al3201_em_write(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret = 0;
	u32 addr, val;
	struct input_dev *input = to_input_dev(dev);
	struct al3201_data *data = input_get_drvdata(input);

	sscanf(buf, "%x%x", &addr, &val);
	printk(KERN_INFO "Write [%x] to Reg[%x]...\n", val, addr);

	mutex_lock(&data->lock);

	ret = i2c_smbus_write_byte_data(data->client, addr, val);
	if (!ret)
		data->reg_cache[addr] = val;

	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(em, S_IWUSR | S_IRUGO, al3201_em_read, al3201_em_write);
#endif

static ssize_t al3201_poll_delay_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct al3201_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", ktime_to_ns(data->light_poll_delay));
}

static ssize_t al3201_poll_delay_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	int err;
	int64_t new_delay;
	struct al3201_data *data = dev_get_drvdata(dev);

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	printk(KERN_INFO "new delay = %lldns, old delay = %lldns\n",
	       new_delay, ktime_to_ns(data->light_poll_delay));

	if (new_delay != ktime_to_ns(data->light_poll_delay)) {
		data->light_poll_delay = ns_to_ktime(new_delay);
		if (data->state) {
			al3201_disable(data);
			al3201_enable(data);
		}
	}

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   al3201_poll_delay_show, al3201_poll_delay_store);

static ssize_t al3201_light_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct al3201_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->state);
}

static ssize_t al3201_light_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	bool new_value = false;
	int err = 0;
	struct al3201_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1")) {
		new_value = true;
	} else if (sysfs_streq(buf, "0")) {
		new_value = false;
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	printk(KERN_INFO "new_value = %d, old state = %d\n",
			new_value, data->state);

	if (new_value && (!data->state)) {
		err = al3201_enable(data);
		if (!err) {
			data->state = ON;
		} else {
			pr_err("%s: couldn't enable", __func__);
			data->state = OFF;
		}
	} else if (!new_value && (data->state)) {
		err = al3201_disable(data);
		if (!err)
			data->state = OFF;
		else
			pr_err("%s: couldn't disable", __func__);
	}

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   al3201_light_enable_show, al3201_light_enable_store);

static struct attribute *al3201_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,

#ifdef LSC_DBG
	&dev_attr_em.attr,
#endif
	NULL
};

static const struct attribute_group al3201_attribute_group = {
	.attrs = al3201_attributes,
};

static int al3201_init_client(struct i2c_client *client)
{
	int i;
	struct al3201_data *data = i2c_get_clientdata(client);

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, al3201_reg[i]);
		if (v < 0)
			return -ENODEV;
		data->reg_cache[i] = v;
	}

	/*
	* 0 : Low resolution range, 0 to 32768 lux
	* 1 : High resolution range, 0 to 8192 lux
	*/
	al3201_set_range(client, 1);
	/* 0x02 : Response time 200ms low pass fillter */
	al3201_set_response_time(client, 0x02);
	/* chip power off */
	al3201_set_power_state(client, OFF);

	return 0;
}

static int __devinit al3201_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int err = 0;
	struct al3201_data *data;
	struct input_dev *input_dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk(KERN_INFO "%s: start!\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("%s, failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	/* initialize the AL3201 chip */
	err = al3201_init_client(client);
	if (err) {
		pr_err("%s: No search al3201 lightsensor!\n", __func__);
		goto err_initializ_chip;
	}

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	data->timer.function = al3201_timer_func;
	data->state = OFF;

	data->wq = alloc_workqueue("al3201_wq", WQ_UNBOUND | WQ_RESCUER, 1);
	if (!data->wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	INIT_WORK(&data->work_light, al3201_work_func_light);

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;
	}

	input_set_drvdata(input_dev, data);
	input_dev->name = "light_sensor";
	input_set_capability(input_dev, EV_REL, REL_MISC);

	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}

	data->input = input_dev;

	err = sysfs_create_group(&input_dev->dev.kobj, &al3201_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	data->factory_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(data->factory_class)) {
		pr_err("Failed to create class(lightsensor)!\n");
		err = PTR_ERR(data->factory_class);
		goto err_factory_sysfs_create;
	}

	data->factory_dev = device_create(data->factory_class, NULL, 0,
					  data, "switch_cmd");
	if (IS_ERR(data->factory_dev)) {
		pr_err("Failed to create device(switch_cmd_dev)!\n");
		err = PTR_ERR(data->factory_dev);
		goto err_factory_device_create;
	}

	err = device_create_file(data->factory_dev,
				 &dev_attr_lightsensor_file_cmd);
	if (err < 0) {
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_lightsensor_file_cmd.attr.name);
		goto err_file_cmd_attr_create;
	}

	err = device_create_file(data->factory_dev,
				 &dev_attr_lightsensor_file_illuminance);
	if (err < 0) {
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_lightsensor_file_illuminance.attr.name);
		goto err_illuminance_attr_create;
	}

	err = device_create_file(data->factory_dev, &dev_attr_sensor_info);
	if (err < 0) {
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_sensor_info.attr.name);
		goto err_sensor_info_attr_create;
	}

	printk(KERN_INFO "%s: success!\n", __func__);
	goto done;

 err_sensor_info_attr_create:
	device_remove_file(data->factory_dev,
			   &dev_attr_lightsensor_file_illuminance);
 err_illuminance_attr_create:
	device_remove_file(data->factory_dev, &dev_attr_lightsensor_file_cmd);
 err_file_cmd_attr_create:
	device_destroy(data->factory_class, 0);
 err_factory_device_create:
	class_destroy(data->factory_class);
 err_factory_sysfs_create:
	sysfs_remove_group(&data->input->dev.kobj, &al3201_attribute_group);
 err_sysfs_create_group_light:
	input_unregister_device(data->input);
 err_input_register_device_light:
 err_input_allocate_device_light:
	destroy_workqueue(data->wq);
 err_create_workqueue:
 err_initializ_chip:
	mutex_destroy(&data->lock);
	kfree(data);
 done:
	printk(KERN_INFO "%s: fail!\n", __func__);
	return err;
}

static int al3201_remove(struct i2c_client *client)
{
	struct al3201_data *data = i2c_get_clientdata(client);

	device_remove_file(data->factory_dev, &dev_attr_sensor_info);
	device_remove_file(data->factory_dev, &dev_attr_lightsensor_file_cmd);
	device_remove_file(data->factory_dev,
			   &dev_attr_lightsensor_file_illuminance);
	device_destroy(data->factory_class, 0);
	class_destroy(data->factory_class);
	sysfs_remove_group(&data->input->dev.kobj, &al3201_attribute_group);
	input_unregister_device(data->input);

	al3201_set_power_state(client, OFF);

	if (data->state)
		al3201_disable(data);

	destroy_workqueue(data->wq);
	mutex_destroy(&data->lock);
	kfree(data);

	return 0;
}

static int al3201_suspend(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct al3201_data *data = i2c_get_clientdata(client);

	if (data->state) {
		err = al3201_disable(data);
		if (err)
			pr_err("%s: could not disable\n", __func__);
	}

	return err;
}

static int al3201_resume(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct al3201_data *data = i2c_get_clientdata(client);

	if (data->state) {
		err = al3201_enable(data);
		if (err)
			pr_err("%s: could not enable\n", __func__);
	}
	return err;
}

static const struct i2c_device_id al3201_id[] = {
	{al3201_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, al3201_id);

static const struct dev_pm_ops al3201_pm_ops = {
	.suspend = al3201_suspend,
	.resume = al3201_resume,
};

static struct i2c_driver al3201_driver = {
	.driver = {
		   .name = al3201_DRV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &al3201_pm_ops,
		   },
	.probe = al3201_probe,
	.remove = al3201_remove,
	.id_table = al3201_id,
};

static int __init al3201_init(void)
{
	return i2c_add_driver(&al3201_driver);
}

static void __exit al3201_exit(void)
{
	i2c_del_driver(&al3201_driver);
}

MODULE_AUTHOR("Kyusung Kim <gs0816.kim@samsung.com>");
MODULE_DESCRIPTION("AL3201 Ambient light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3201_init);
module_exit(al3201_exit);
