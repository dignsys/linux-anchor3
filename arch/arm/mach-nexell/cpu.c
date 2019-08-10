// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018  Nexell Co., Ltd.
// Youngbok, Park <ybpark@nexell.co.kr>

#include <asm/mach/arch.h>

static const char * const nexell_dt_compat[] = {
	"nexell,nxp3220",
	NULL
};

/*------------------------------------------------------------------------------
 * Maintainer: Nexell Co., Ltd.
 */
DT_MACHINE_START(NEXELL_DT, "NEXELL (Device Tree Support)")
	.dt_compat	= nexell_dt_compat,
MACHINE_END
