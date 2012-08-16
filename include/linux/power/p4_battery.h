/*
 *  p4_battery.h
 *  charger systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_P3_BATTERY_H
#define _LINUX_P3_BATTERY_H


extern int p3_low_batt_compensation(int fg_soc, int fg_vcell, int fg_current);
extern void reset_low_batt_comp_cnt(void);
extern int check_jig_on(void);
extern int get_fuelgauge_value(int data);
extern struct max17042_chip *max17042_chip_data;
extern int charging_mode_from_boot;
extern int check_usb_status;

struct max8903_charger_data {
	int enable_line;
	int connect_line;
	int fullcharge_line;
	int currentset_line;
};

struct p3_battery_platform_data {
	struct max8903_charger_data charger;
	bool	(*check_dedicated_charger) (void);
	void	(*init_charger_gpio) (void);
	void (*inform_charger_connection) (int);
	int temp_high_threshold;
	int temp_high_recovery;
	int temp_low_recovery;
	int temp_low_threshold;
#ifdef CONFIG_MACH_SAMSUNG_P4LTE
	int temp_high_threshold_lpm;
	int temp_high_recovery_lpm;
	int temp_low_recovery_lpm;
	int temp_low_threshold_lpm;
#endif
	int charge_duration;
	int recharge_duration;
	int recharge_voltage;
};

// for test driver
#define __TEST_DEVICE_DRIVER__

#endif

