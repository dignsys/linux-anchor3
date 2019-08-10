// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxe2000 regulator driver
 * Copyright (c) 2018 Jongshin, Park <pjsin865@nexell.co.kr>
 */

#ifndef __LINUX_REGULATOR_NXE2000_H
#define __LINUX_REGULATOR_NXE2000_H

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#define nxe2000_rails(_name) "NXE2000_" #_name

/* RICHOH Regulator IDs */
enum regulator_id {
	NXE2000_ID_DC1,
	NXE2000_ID_DC2,
	NXE2000_ID_DC3,
	NXE2000_ID_DC4,
	NXE2000_ID_DC5,
	NXE2000_ID_LDO1,
	NXE2000_ID_LDO2,
	NXE2000_ID_LDO3,
	NXE2000_ID_LDO4,
	NXE2000_ID_LDO5,
	NXE2000_ID_LDO6,
	NXE2000_ID_LDO7,
	NXE2000_ID_LDO8,
	NXE2000_ID_LDO9,
	NXE2000_ID_LDO10,
	NXE2000_ID_LDORTC1,
	NXE2000_ID_LDORTC2,
};

struct nxe2000_regulator_platform_data {
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
	struct regulator_consumer_supply consumer;
	const char *supply_name;
	int id;
	int init_uV;
	unsigned init_enable : 1;
	unsigned init_apply : 1;
	int set_init_uV;
	int sleep_uV;
	int sleep_slots;
	unsigned long ext_pwr_req;
	unsigned long flags;
};

extern int nxe2000_regulator_enable_eco_mode(struct regulator_dev *rdev);
extern int nxe2000_regulator_disable_eco_mode(struct regulator_dev *rdev);
extern int nxe2000_regulator_enable_eco_slp_mode(struct regulator_dev *rdev);
extern int nxe2000_regulator_disable_eco_slp_mode(struct regulator_dev *rdev);

#endif
