/* SPDX-License-Identifier: (GPL-2.0+ or MIT) */
/*
 * Nexell NXP3220 SoC clock reset IDs
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#ifndef _DT_BINDINGS_RESET_NEXELL_NXP3220_H
#define _DT_BINDINGS_RESET_NEXELL_NXP3220_H

/*
 * Reset for SRC/SYS
 */

/* To be added more resets */
#define CLK_RESET_SPI0_CORE		0
#define CLK_RESET_SPI1_CORE		1
#define CLK_RESET_SPI2_CORE		2

#define CLK_RESET_CMU_NR		3

/*
 * Reset for MM
 */
#define CLK_RESET_MM_AXI		0
#define CLK_RESET_MM_ROTATOR_AXI	1
#define CLK_RESET_MM_G2D_AXI		2
#define CLK_RESET_MM_DEINTERLACE_AXI	3
#define CLK_RESET_MM_VIP_AXI		4
#define CLK_RESET_MM_DPC_AXI		5
#define CLK_RESET_MM_CODA960_AXI	6
#define CLK_RESET_MM_APB		7
#define CLK_RESET_MM_SYSREG_APB		8
#define CLK_RESET_MM_DEINTERLACE_APB	9
#define CLK_RESET_MM_CODA960_APB	12
#define CLK_RESET_MM_DPC_X2		15
#define CLK_RESET_MM_LVDS_VCLK		17
#define CLK_RESET_MM_CODA960_CORE	18
#define CLK_RESET_MM_LVDS_PHY		19

#define CLK_RESET_MM_NR			20

#endif /* _DT_BINDINGS_RESET_NEXELL_NXP3220_H */
