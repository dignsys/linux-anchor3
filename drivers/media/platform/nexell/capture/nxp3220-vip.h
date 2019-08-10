// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#ifndef _NX_VIP_H
#define _NX_VIP_H

#include <linux/atomic.h>
#include <linux/spinlock.h>

enum {
	VIP_CLIPPER	= (1 << 0),
	VIP_DECIMATOR	= (1 << 1),
};

enum {
	VIP_VS_INT	= 0,
	VIP_HS_INT	= 1,
	VIP_OD_INT	= 2,
};

struct nx_bus_fmt_map {
	u32 media_bus_fmt;
	u32 nx_bus_fmt;
};

struct nx_mem_fmt_map {
	u32 pixel_fmt;
	u32 media_bus_fmt;
	u32 nx_mem_fmt;
};

bool nx_vip_is_valid(u32 module);
int nx_vip_reset(u32 module);
int nx_vip_clock_enable(u32 module, bool enable);
int nx_vip_padout_clock_enable(u32 module, bool enable);
int nx_vip_register_irq_entry(u32 module, struct nx_v4l2_irq_entry *e);
int nx_vip_unregister_irq_entry(u32 module, struct nx_v4l2_irq_entry *e);
int nx_vip_is_running(u32 module, u32 child);
int nx_vip_run(u32 module, u32 child);
int nx_vip_stop(u32 module, u32 child);
int nx_vip_find_nx_bus_format(u32 media_bus_fmt, u32 *found);
int nx_vip_find_mbus_format(u32 nx_bus_fmt, u32 *found);
int nx_vip_find_nx_mem_format(u32 media_bus_fmt, u32 *found);
int nx_vip_find_mbus_mem_format(u32 nx_mem_fmt, u32 *found);

#endif
