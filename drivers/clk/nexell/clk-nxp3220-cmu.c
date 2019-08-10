// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 SoC Clock CMU(SRC + SYS) driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/nxp3220-clk.h>
#include <dt-bindings/reset/nexell,nxp3220-reset.h>
#include "clk-nexell.h"
#include "clk-nxp3220.h"

#define SYS0_AXI			0x0200
#define SYS0_HSIF_AXI			0x0400
#define CPU_BACKUP0			0x0600
#define CSSYS0_HCLK			0x0800
#define BLK_CMU0_APB			0x0C00
#define VIP_PADOUT0			0x1000
#define VIP_PADOUT1			0x1200
#define VIP_PADOUT2			0x1400
#define VIP_PADOUT3			0x1600
#define VIP_PADOUT4			0x1800
#define HPM_SYS0			0x1A00
#define UART0_CORE			0x1C00
#define UART0_APB			0x1E00
#define I2S0_CORE			0x2200
#define I2S1_CORE			0x2400
#define I2S2_CORE			0x2600
#define I2S3_CORE			0x2800
#define I2C0_APB			0x2A00
#define SDMMC0_CORE			0x3000
#define SDMMC1_CORE			0x3200
#define SDMMC2_CORE			0x3400
#define SPI0_APB			0x3600
#define SPI0_CORE			0x3800
#define PDM0_AXI			0x3A00
#define PDM0_CORE			0x3C00
#define PWM0_APB			0x3E00
#define PWM0_TCLK0			0x4000
#define PWM0_TCLK1			0x4200
#define PWM0_TCLK2			0x4400
#define PWM0_TCLK3			0x4600
#define CAN0_CORE			0x4800
#define CAN1_CORE			0x4A00
#define TIMER0_APB			0x4C00
#define TIMER0_TCLK0			0x4E00
#define TIMER0_TCLK1			0x5000
#define TIMER0_TCLK2			0x5200
#define TIMER0_TCLK3			0x5400
#define SECURE_TIMER0_APB		0x5600
#define SECURE_TIMER0_TCLK0		0x5800
#define SECURE_TIMER0_TCLK1		0x5A00
#define SECURE_TIMER0_TCLK2		0x5C00
#define SECURE_TIMER0_TCLK3		0x5E00
#define SMC0_AXI			0x6000
#define SPDIFTX0_CORE			0x6200
#define GMAC_RGMII0_TX			0x6400
#define GMAC_RGMII0_PTP_REF		0x6600
#define GMAC_RMII0_PTP_REF		0x6A00
#define NANDC0_AXI			0x6E00
#define MM0_AXI				0x7000
#define VIP0_PADOUT0			0x7200
#define VIP0_PADOUT1			0x7400
#define DPC0_X2				0x7600
#define LVDS0_VCLK			0x7800
#define CODA960_0_CORE			0x7A00
#define USB0_AHB			0x7C00

#define MUX_PLL0_CLK_NUM		0
#define MUX_PLL1_CLK_NUM		1
#define MUX_PLL_CPU_DIV_NUM		2
#define MUX_PLL_DDR0_DIV_NUM		3
#define MUX_PLL_DDR1_DIV_NUM		4
#define MUX_EXT_SRC_CLK0_NUM		5
#define MUX_OSCCLK_IN_NUM		6

/*
 * MUX name must be one of clock, registerd by DT
 */
#ifndef CONFIG_CPU_FREQ
PNAME(src_mux_p) = { "pll0", "pll1_div", "pll_cpu_div",
	"pll_ddr0_div", "pll_ddr1_div", "ext_clk", "oscclk"};
#else
PNAME(src_mux_p) = { "pll0", "pll1_div",
	"pll_ddr0_div", "pll_ddr1_div", "ext_clk", "oscclk"};

#endif
PNAME(snd_mux_p) = { "pll1_div", "pll_ddr1_div", "ext_clk" };

static u32 src_mux_table[] = {
	MUX_PLL0_CLK_NUM,
	MUX_PLL1_CLK_NUM,
#ifndef CONFIG_CPU_FREQ
	MUX_PLL_CPU_DIV_NUM,
#endif
	MUX_PLL_DDR0_DIV_NUM,
	MUX_PLL_DDR1_DIV_NUM,
	MUX_EXT_SRC_CLK0_NUM,
	MUX_OSCCLK_IN_NUM,
};

static u32 snd_mux_table[] = {
	MUX_PLL1_CLK_NUM,
	MUX_PLL_DDR1_DIV_NUM,
	MUX_EXT_SRC_CLK0_NUM,
};

#define COMP_BASE_SRC(_id, cname) \
	COMP_BASE(_id, NULL, cname, src_mux_p, \
		  CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED)

#define COMP_BASE_SRC_CRIT(_id, cname) \
	COMP_BASE(_id, NULL, cname, src_mux_p, \
		  CLK_SET_RATE_NO_REPARENT | CLK_IS_CRITICAL)

#define COMP_BASE_SRC_F(_id, cname, f) \
	COMP_BASE(_id, NULL, cname, src_mux_p, f)

#define COMP_BASE_SND(_id, cname, f) \
	COMP_BASE(_id, NULL, cname, snd_mux_p, f)

#define COMP_MUX_SRC(o)		COMP_MUX_T(o, 0, 4, src_mux_table, 0)
#define COMP_MUX_SND(o)		COMP_MUX_T(o, 0, 4, snd_mux_table, 0)

#define COMP_DIV_SRC(o) 	COMP_DIV(o + 0x60, 0, 8, 0)
#define COMP_DIV_SRC_F(o, f) 	COMP_DIV(o + 0x60, 0, 8, f)

#define COMP_GATE_SRC(o) 	COMP_GATE(o + 0x10, o + 0x20, 0, 0)

static const struct nexell_composite_clock src_clks[] __initconst = {
	{
		COMP_BASE_SRC_CRIT(CLK_SRC_SYS0_AXI, "src_sys0_axi")
		COMP_MUX_SRC(SYS0_AXI)
		COMP_DIV_SRC(SYS0_AXI)
		COMP_GATE_SRC(SYS0_AXI)
	}, {
		COMP_BASE_SRC(CLK_SRC_SYS0_HSIF_AXI, "src_sys0_hsif_axi")
		COMP_MUX_SRC(SYS0_HSIF_AXI)
		COMP_DIV_SRC(SYS0_HSIF_AXI)
		COMP_GATE_SRC(SYS0_HSIF_AXI)
	}, {
		COMP_BASE_SRC(CLK_SRC_CPU_BACKUP0, "src_cpu_backup0")
		COMP_MUX_SRC(CPU_BACKUP0)
		COMP_DIV_SRC(CPU_BACKUP0)
		COMP_GATE_SRC(CPU_BACKUP0)
	}, {
		COMP_BASE_SRC(CLK_SRC_CSSYS0_HCLK, "src_cssys0_hclk")
		COMP_MUX_SRC(CSSYS0_HCLK)
		COMP_DIV_SRC(CSSYS0_HCLK)
		COMP_GATE_SRC(CSSYS0_HCLK)
	}, {
		COMP_BASE_SRC(CLK_SRC_BLK_CMU0_APB, "src_blk_cmu0_apb")
		COMP_MUX_SRC(BLK_CMU0_APB)
		COMP_DIV_SRC(BLK_CMU0_APB)
		COMP_GATE_SRC(BLK_CMU0_APB)
	}, {
		COMP_BASE_SRC(CLK_SRC_VIP_PADOUT0, "src_vip_padout0")
		COMP_MUX_SRC(VIP_PADOUT0)
		COMP_DIV_SRC(VIP_PADOUT0)
		COMP_GATE_SRC(VIP_PADOUT0)
	}, {
		COMP_BASE_SRC(CLK_SRC_VIP_PADOUT1, "src_vip_padout1")
		COMP_MUX_SRC(VIP_PADOUT1)
		COMP_DIV_SRC(VIP_PADOUT1)
		COMP_GATE_SRC(VIP_PADOUT1)
	}, {
		COMP_BASE_SRC(CLK_SRC_VIP_PADOUT2, "src_vip_padout2")
		COMP_MUX_SRC(VIP_PADOUT2)
		COMP_DIV_SRC(VIP_PADOUT2)
		COMP_GATE_SRC(VIP_PADOUT2)
	}, {
		COMP_BASE_SRC(CLK_SRC_VIP_PADOUT3, "src_vip_padout3")
		COMP_MUX_SRC(VIP_PADOUT3)
		COMP_DIV_SRC(VIP_PADOUT3)
		COMP_GATE_SRC(VIP_PADOUT3)
	}, {
		COMP_BASE_SRC(CLK_SRC_VIP_PADOUT4, "src_vip_padout4")
		COMP_MUX_SRC(VIP_PADOUT4)
		COMP_DIV_SRC(VIP_PADOUT4)
		COMP_GATE_SRC(VIP_PADOUT4)
	}, {
		COMP_BASE_SRC(CLK_SRC_HPM_SYS0, "src_hpm_sys0")
		COMP_MUX_SRC(HPM_SYS0)
		COMP_DIV_SRC(HPM_SYS0)
		COMP_GATE_SRC(HPM_SYS0)
	}, {
		COMP_BASE_SRC(CLK_SRC_UART0_CORE, "src_uart0_core")
		COMP_MUX_SRC(UART0_CORE)
		COMP_DIV_SRC(UART0_CORE)
		COMP_GATE_SRC(UART0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_UART0_APB, "src_uart_apb")
		COMP_MUX_SRC(UART0_APB)
		COMP_DIV_SRC(UART0_APB)
		COMP_GATE_SRC(UART0_APB)
	}, {
		COMP_BASE_SND(CLK_SRC_I2S0_CORE, "src_i2s0_core", CLK_DIVIDER_ROUND_CLOSEST)
		COMP_MUX_SND(I2S0_CORE)
		COMP_DIV_SRC_F(I2S0_CORE, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(I2S0_CORE)
	}, {
		COMP_BASE_SND(CLK_SRC_I2S1_CORE, "src_i2s1_core", CLK_DIVIDER_ROUND_CLOSEST)
		COMP_MUX_SND(I2S1_CORE)
		COMP_DIV_SRC_F(I2S1_CORE, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(I2S1_CORE)
	}, {
		COMP_BASE_SND(CLK_SRC_I2S2_CORE, "src_i2s2_core", CLK_DIVIDER_ROUND_CLOSEST)
		COMP_MUX_SND(I2S2_CORE)
		COMP_DIV_SRC_F(I2S2_CORE, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(I2S2_CORE)
	}, {
		COMP_BASE_SND(CLK_SRC_I2S3_CORE, "src_i2s3_core", CLK_DIVIDER_ROUND_CLOSEST)
		COMP_MUX_SND(I2S3_CORE)
		COMP_DIV_SRC_F(I2S3_CORE, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(I2S3_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_I2C0_APB, "src_i2c0_apb")
		COMP_MUX_SRC(I2C0_APB)
		COMP_DIV_SRC(I2C0_APB)
		COMP_GATE_SRC(I2C0_APB)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_SDMMC0_CORE, "src_sdmmc0_core", 0)
		COMP_MUX_SRC(SDMMC0_CORE)
		COMP_DIV_SRC(SDMMC0_CORE)
		COMP_GATE_SRC(SDMMC0_CORE)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_SDMMC1_CORE, "src_sdmmc1_core", 0)
		COMP_MUX_SRC(SDMMC1_CORE)
		COMP_DIV_SRC(SDMMC1_CORE)
		COMP_GATE_SRC(SDMMC1_CORE)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_SDMMC2_CORE, "src_sdmmc2_core", 0)
		COMP_MUX_SRC(SDMMC2_CORE)
		COMP_DIV_SRC(SDMMC2_CORE)
		COMP_GATE_SRC(SDMMC2_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_SPI0_APB, "src_spi0_apb")
		COMP_MUX_SRC(SPI0_APB)
		COMP_DIV_SRC(SPI0_APB)
		COMP_GATE_SRC(SPI0_APB)
	}, {
		COMP_BASE_SRC(CLK_SRC_SPI0_CORE, "src_spi0_core")
		COMP_MUX_SRC(SPI0_CORE)
		COMP_DIV_SRC(SPI0_CORE)
		COMP_GATE_SRC(SPI0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_PDM0_AXI, "src_pdm0_axi")
		COMP_MUX_SRC(PDM0_AXI)
		COMP_DIV_SRC(PDM0_AXI)
		COMP_GATE_SRC(PDM0_AXI)
	}, {
		COMP_BASE_SRC(CLK_SRC_PDM0_CORE, "src_pdm0_core")
		COMP_MUX_SRC(PDM0_CORE)
		COMP_DIV_SRC(PDM0_CORE)
		COMP_GATE_SRC(PDM0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_PWM0_APB, "src_pwm0_apb")
		COMP_MUX_SRC(PWM0_APB)
		COMP_DIV_SRC(PWM0_APB)
		COMP_GATE_SRC(PWM0_APB)
	}, {
		COMP_BASE_SRC(CLK_SRC_PWM0_TCLK0, "src_pwm0_tclk0")
		COMP_MUX_SRC(PWM0_TCLK0)
		COMP_DIV_SRC(PWM0_TCLK0)
		COMP_GATE_SRC(PWM0_TCLK0)
	}, {
		COMP_BASE_SRC(CLK_SRC_PWM0_TCLK1, "src_pwm0_tclk1")
		COMP_MUX_SRC(PWM0_TCLK1)
		COMP_DIV_SRC(PWM0_TCLK1)
		COMP_GATE_SRC(PWM0_TCLK1)
	}, {
		COMP_BASE_SRC(CLK_SRC_PWM0_TCLK2, "src_pwm0_tclk2")
		COMP_MUX_SRC(PWM0_TCLK2)
		COMP_DIV_SRC(PWM0_TCLK2)
		COMP_GATE_SRC(PWM0_TCLK2)
	}, {
		COMP_BASE_SRC(CLK_SRC_PWM0_TCLK3, "src_pwm0_tclk3")
		COMP_MUX_SRC(PWM0_TCLK3)
		COMP_DIV_SRC(PWM0_TCLK3)
		COMP_GATE_SRC(PWM0_TCLK3)
	}, {
		COMP_BASE_SRC(CLK_SRC_CAN0_CORE, "src_can0_core")
		COMP_MUX_SRC(CAN0_CORE)
		COMP_DIV_SRC(CAN0_CORE)
		COMP_GATE_SRC(CAN0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_CAN1_CORE, "src_can1_core")
		COMP_MUX_SRC(CAN1_CORE)
		COMP_DIV_SRC(CAN1_CORE)
		COMP_GATE_SRC(CAN1_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_TIMER0_APB, "src_timer0_apb")
		COMP_MUX_SRC(TIMER0_APB)
		COMP_DIV_SRC(TIMER0_APB)
		COMP_GATE_SRC(TIMER0_APB)
	}, {
		COMP_BASE_SRC(CLK_SRC_TIMER0_TCLK0, "src_timer0_tclk0")
		COMP_MUX_SRC(TIMER0_TCLK0)
		COMP_DIV_SRC(TIMER0_TCLK0)
		COMP_GATE_SRC(TIMER0_TCLK0)
	}, {
		COMP_BASE_SRC(CLK_SRC_TIMER0_TCLK1, "src_timer0_tclk1")
		COMP_MUX_SRC(TIMER0_TCLK1)
		COMP_DIV_SRC(TIMER0_TCLK1)
		COMP_GATE_SRC(TIMER0_TCLK1)
	}, {
		COMP_BASE_SRC(CLK_SRC_TIMER0_TCLK2, "src_timer0_tclk2")
		COMP_MUX_SRC(TIMER0_TCLK2)
		COMP_DIV_SRC(TIMER0_TCLK2)
		COMP_GATE_SRC(TIMER0_TCLK2)
	}, {
		COMP_BASE_SRC(CLK_SRC_TIMER0_TCLK3, "src_timer0_tclk3")
		COMP_MUX_SRC(TIMER0_TCLK3)
		COMP_DIV_SRC(TIMER0_TCLK3)
		COMP_GATE_SRC(TIMER0_TCLK3)
	}, {
		COMP_BASE_SRC(CLK_SRC_SECURE_TIMER0_APB, "src_secure_timer0_apb")
		COMP_MUX_SRC(SECURE_TIMER0_APB)
		COMP_DIV_SRC(SECURE_TIMER0_APB)
		COMP_GATE_SRC(SECURE_TIMER0_APB)
	}, {
		COMP_BASE_SRC(CLK_SRC_SECURE_TIMER0_TCLK0, "src_secure_timer0_tclk0")
		COMP_MUX_SRC(SECURE_TIMER0_TCLK0)
		COMP_DIV_SRC(SECURE_TIMER0_TCLK0)
		COMP_GATE_SRC(SECURE_TIMER0_TCLK0)
	}, {
		COMP_BASE_SRC(CLK_SRC_SECURE_TIMER0_TCLK1, "src_secure_timer0_tclk1")
		COMP_MUX_SRC(SECURE_TIMER0_TCLK1)
		COMP_DIV_SRC(SECURE_TIMER0_TCLK1)
		COMP_GATE_SRC(SECURE_TIMER0_TCLK1)
	}, {
		COMP_BASE_SRC(CLK_SRC_SECURE_TIMER0_TCLK2, "src_secure_timer0_tclk2")
		COMP_MUX_SRC(SECURE_TIMER0_TCLK2)
		COMP_DIV_SRC(SECURE_TIMER0_TCLK2)
		COMP_GATE_SRC(SECURE_TIMER0_TCLK2)
	}, {
		COMP_BASE_SRC(CLK_SRC_SECURE_TIMER0_TCLK3, "src_secure_timer0_tclk3")
		COMP_MUX_SRC(SECURE_TIMER0_TCLK3)
		COMP_DIV_SRC(SECURE_TIMER0_TCLK3)
		COMP_GATE_SRC(SECURE_TIMER0_TCLK3)
	}, {
		COMP_BASE_SRC(CLK_SRC_SMC0_AXI, "src_smc0_axi")
		COMP_MUX_SRC(SMC0_AXI)
		COMP_DIV_SRC(SMC0_AXI)
		COMP_GATE_SRC(SMC0_AXI)
	}, {
		COMP_BASE_SRC(CLK_SRC_SPDIFTX0_CORE, "src_spdiftx0_core")
		COMP_MUX_SRC(SPDIFTX0_CORE)
		COMP_DIV_SRC(SPDIFTX0_CORE)
		COMP_GATE_SRC(SPDIFTX0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_GMAC_RGMII0_TX, "src_gmac_rgmii0_tx")
		COMP_MUX_SRC(GMAC_RGMII0_TX)
		COMP_DIV_SRC(GMAC_RGMII0_TX)
		COMP_GATE_SRC(GMAC_RGMII0_TX)
	}, {
		COMP_BASE_SRC(CLK_SRC_GMAC_RGMII0_PTP_REF, "src_gmac_rgmii0_ptp_ref")
		COMP_MUX_SRC(GMAC_RGMII0_PTP_REF)
		COMP_DIV_SRC(GMAC_RGMII0_PTP_REF)
		COMP_GATE_SRC(GMAC_RGMII0_PTP_REF)
	}, {
		COMP_BASE_SRC(CLK_SRC_GMAC_RMII0_PTP_REF, "src_gmac_rmii0_ptp_ref")
		COMP_MUX_SRC(GMAC_RMII0_PTP_REF)
		COMP_DIV_SRC(GMAC_RMII0_PTP_REF)
		COMP_GATE_SRC(GMAC_RMII0_PTP_REF)
	}, {
		COMP_BASE_SRC(CLK_SRC_NANDC0_AXI, "src_nandc0_axi")
		COMP_MUX_SRC(NANDC0_AXI)
		COMP_DIV_SRC(NANDC0_AXI)
		COMP_GATE_SRC(NANDC0_AXI)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_MM0_AXI, "src_mm0_axi", 0)
		COMP_MUX_SRC(MM0_AXI)
		COMP_DIV_SRC(MM0_AXI)
		COMP_GATE_SRC(MM0_AXI)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_VIP0_PADOUT0, "src_vip0_padout0", 0)
		COMP_MUX_SRC(VIP0_PADOUT0)
		COMP_DIV_SRC(VIP0_PADOUT0)
		COMP_GATE_SRC(VIP0_PADOUT0)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_VIP0_PADOUT1, "src_vip0_padout1", 0)
		COMP_MUX_SRC(VIP0_PADOUT1)
		COMP_DIV_SRC(VIP0_PADOUT1)
		COMP_GATE_SRC(VIP0_PADOUT1)
	}, {
		COMP_BASE_SRC(CLK_SRC_DPC0_X2, "src_dpc0_x2")
		COMP_MUX_SRC(DPC0_X2)
		COMP_DIV_SRC_F(DPC0_X2, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(DPC0_X2)
	}, {
		COMP_BASE_SRC(CLK_SRC_LVDS0_VCLK, "src_lvds0_vclk")
		COMP_MUX_SRC(LVDS0_VCLK)
		COMP_DIV_SRC_F(LVDS0_VCLK, CLK_DIVIDER_ROUND_CLOSEST)
		COMP_GATE_SRC(LVDS0_VCLK)
	}, {
		COMP_BASE_SRC_F(CLK_SRC_CODA960_0_CORE, "src_coda960_0_core", 0)
		COMP_MUX_SRC(CODA960_0_CORE)
		COMP_DIV_SRC(CODA960_0_CORE)
		COMP_GATE_SRC(CODA960_0_CORE)
	}, {
		COMP_BASE_SRC(CLK_SRC_USB0_AHB, "src_usb0_ahb")
		COMP_MUX_SRC(USB0_AHB)
		COMP_DIV_SRC(USB0_AHB)
		COMP_GATE_SRC(USB0_AHB)
	},
};

#define DIV_SYS(_id, cname, pname, o)	\
	__DIV(_id, NULL, cname, pname, o, 0, 16, 0, 0)

#define DIV_SYS_F(_id, cname, pname, o, f, df)	\
	__DIV(_id, NULL, cname, pname, o, 0, 16, f, df)

static const struct nexell_div_clock sys_div_clks[] __initconst = {
	DIV_SYS(CLK_SYS_DIV_SYS0_AXI, "div_sys_sys0_axi", "src_sys0_axi",
	    SYS0_AXI + 0x60),
	DIV_SYS(CLK_SYS_DIV_SYS0_APB, "div_sys_sys0_apb", "div_sys_sys0_axi",
	    SYS0_AXI + 0x64),
	DIV_SYS(CLK_SYS_DIV_SYS0_HSIF_AXI, "div_sys_sys0_hsif_axi",
	    "src_sys0_hsif_axi", SYS0_HSIF_AXI + 0x60),
	DIV_SYS(CLK_SYS_DIV_SYS0_HSIF_APB, "div_sys_sys0_hsif_apb",
	    "div_sys_sys0_hsif_axi", SYS0_HSIF_AXI + 0x64),
	DIV_SYS(CLK_SYS_DIV_CPU_BACKUP0, "div_sys_cpu_backup0",
		"src_cpu_backup0", CPU_BACKUP0 + 0x64),
	DIV_SYS(CLK_SYS_DIV_CSSYS0_HCLK, "div_sys_cssys0_hclk",
		"src_cssys0_hclk", CSSYS0_HCLK + 0x60),
	DIV_SYS(CLK_SYS_DIV_BLK_CMU0_APB, "div_sys_blk_cmu0_apb",
		"src_blk_cmu0_apb", BLK_CMU0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_VIP_PADOUT0, "div_sys_vip_padout0",
		"src_vip_padout0", VIP_PADOUT0 + 0x60),
	DIV_SYS(CLK_SYS_DIV_VIP_PADOUT1, "div_sys_vip_padout1",
		"src_vip_padout1", VIP_PADOUT1 + 0x60),
	DIV_SYS(CLK_SYS_DIV_VIP_PADOUT2, "div_sys_vip_padout2",
		"src_vip_padout2", VIP_PADOUT2 + 0x60),
	DIV_SYS(CLK_SYS_DIV_VIP_PADOUT3, "div_sys_vip_padout3",
		"src_vip_padout3", VIP_PADOUT3 + 0x60),
	DIV_SYS(CLK_SYS_DIV_VIP_PADOUT4, "div_sys_vip_padout4",
		"src_vip_padout4", VIP_PADOUT4 + 0x60),
	DIV_SYS(CLK_SYS_DIV_HPM_SYS0, "div_sys_hpm_sys0",
		"src_hpm_sys0", HPM_SYS0 + 0x60),
	DIV_SYS(CLK_SYS_DIV_UART0_CORE, "div_sys_uart0_core",
		"src_uart0_core", UART0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_UART0_APB, "div_sys_uart0_apb",
		"src_uart_apb", UART0_APB + 0x60),
	DIV_SYS_F(CLK_SYS_DIV_I2S0_CORE, "div_sys_i2s0_core",
		"src_i2s0_core", I2S0_CORE + 0x60,
		0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_SYS_F(CLK_SYS_DIV_I2S1_CORE, "div_sys_i2s1_core",
		"src_i2s1_core", I2S1_CORE + 0x60,
		0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_SYS_F(CLK_SYS_DIV_I2S2_CORE, "div_sys_i2s2_core",
		"src_i2s2_core", I2S2_CORE + 0x60,
		0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_SYS_F(CLK_SYS_DIV_I2S3_CORE, "div_sys_i2s3_core",
		"src_i2s3_core", I2S3_CORE + 0x60,
		0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_SYS(CLK_SYS_DIV_I2C0_APB, "div_sys_i2c0_apb",
		"src_i2c0_apb", I2C0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_SDMMC0_CORE, "div_sys_sdmmc0_core",
		"src_sdmmc0_core", SDMMC0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_SDMMC1_CORE, "div_sys_sdmmc1_core",
		"src_sdmmc1_core", SDMMC1_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_SDMMC2_CORE, "div_sys_sdmmc2_core",
		"src_sdmmc2_core", SDMMC2_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_SPI0_APB, "div_sys_spi0_apb",
		"src_spi0_apb", SPI0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_SPI0_CORE, "div_sys_spi0_core",
		"src_spi0_core", SPI0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_PDM0_AXI, "div_sys_pdm0_axi",
		"src_pdm0_axi", PDM0_AXI + 0x60),
	DIV_SYS(CLK_SYS_DIV_PDM0_CORE, "div_sys_pdm0_core",
		"src_pdm0_core", PDM0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_PWM0_APB, "div_sys_pwm0_apb",
		"src_pwm0_apb", PWM0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_PWM0_TCLK0, "div_sys_pwm0_tclk0",
		"src_pwm0_tclk0", PWM0_TCLK0 + 0x60),
	DIV_SYS(CLK_SYS_DIV_PWM0_TCLK1, "div_sys_pwm0_tclk1",
		"src_pwm0_tclk1", PWM0_TCLK1 + 0x60),
	DIV_SYS(CLK_SYS_DIV_PWM0_TCLK2, "div_sys_pwm0_tclk2",
		"src_pwm0_tclk2", PWM0_TCLK2 + 0x60),
	DIV_SYS(CLK_SYS_DIV_PWM0_TCLK3, "div_sys_pwm0_tclk3",
		"src_pwm0_tclk3", PWM0_TCLK3 + 0x60),
	DIV_SYS(CLK_SYS_DIV_CAN0_CORE, "div_sys_can0_core",
		"src_can0_core", CAN0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_CAN1_CORE, "div_sys_can1_core",
		"src_can1_core", CAN1_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_TIMER0_APB, "div_sys_timer0_apb",
		"src_timer0_apb", TIMER0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_TIMER0_TCLK0, "div_sys_timer0_tclk0",
		"src_timer0_tclk0", TIMER0_TCLK0 + 0x60),
	DIV_SYS(CLK_SYS_DIV_TIMER0_TCLK1, "div_sys_timer0_tclk1",
		"src_timer0_tclk1", TIMER0_TCLK1 + 0x60),
	DIV_SYS(CLK_SYS_DIV_TIMER0_TCLK2, "div_sys_timer0_tclk2",
		"src_timer0_tclk2", TIMER0_TCLK2 + 0x60),
	DIV_SYS(CLK_SYS_DIV_TIMER0_TCLK3, "div_sys_timer0_tclk3",
		"src_timer0_tclk3", TIMER0_TCLK3 + 0x60),
	DIV_SYS(CLK_SYS_DIV_SECURE_TIMER0_APB, "div_sys_secure_timer0_apb",
		"src_secure_timer0_apb", SECURE_TIMER0_APB + 0x60),
	DIV_SYS(CLK_SYS_DIV_SECURE_TIMER0_TCLK0, "div_sys_secure_timer0_tclk0",
		"src_secure_timer0_tclk0", SECURE_TIMER0_TCLK0 + 0x60),
	DIV_SYS(CLK_SYS_DIV_SECURE_TIMER0_TCLK1, "div_sys_secure_timer0_tclk1",
		"src_secure_timer0_tclk1", SECURE_TIMER0_TCLK1 + 0x60),
	DIV_SYS(CLK_SYS_DIV_SECURE_TIMER0_TCLK2, "div_sys_secure_timer0_tclk2",
		"src_secure_timer0_tclk2", SECURE_TIMER0_TCLK2 + 0x60),
	DIV_SYS(CLK_SYS_DIV_SECURE_TIMER0_TCLK3, "div_sys_secure_timer0_tclk3",
		"src_secure_timer0_tclk3", SECURE_TIMER0_TCLK3 + 0x60),
	DIV_SYS(CLK_SYS_DIV_SMC0_AXI, "div_sys_smc0_axi",
		"src_smc0_axi", SMC0_AXI + 0x60),
	DIV_SYS(CLK_SYS_DIV_SPDIFTX0_CORE, "div_sys_spdiftx0_core",
		"src_spdiftx0_core", SPDIFTX0_CORE + 0x60),
	DIV_SYS(CLK_SYS_DIV_GMAC_RGMII0_TX, "div_sys_gmac_rgmii0_tx",
		"src_gmac_rgmii0_tx", GMAC_RGMII0_TX + 0x60),
	DIV_SYS(CLK_SYS_DIV_GMAC_RGMII0_PTP_REF, "div_sys_gmac_rgmii0_ptp_ref",
		"src_gmac_rgmii0_ptp_ref", GMAC_RGMII0_PTP_REF + 0x60),
	DIV_SYS(CLK_SYS_DIV_GMAC_RMII0_PTP_REF, "div_sys_gmac_rmii0_ptp_ref",
		"src_gmac_rmii0_ptp_ref", GMAC_RMII0_PTP_REF + 0x60),
	DIV_SYS(CLK_SYS_DIV_NANDC0_AXI, "div_sys_nandc0_axi",
		"src_nandc0_axi", NANDC0_AXI + 0x60),
};

/* Set CLK_IGNORE_UNUSED until we find which clocks can be disabled */
#define GATE_SYS(_id, cname, pname, o, b, f, gf)		\
	GATE(_id, cname, pname, o, o + 0x10, b, (f) | CLK_IGNORE_UNUSED, gf)

#define GATE_SYS_NP(_id, cname, pname, o, b, f, gf)		\
	GATE_NP(_id, cname, pname, o, o + 0x10, b, (f) | CLK_IGNORE_UNUSED, gf)

static const struct nexell_gate_clock sys_gate_clks[] __initconst = {
	GATE_SYS_NP(CLK_SYS0_AXI, "sys0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 0, 0, 0),
	GATE_SYS_NP(CLK_SYS_BUS0_AXI, "sys_bus0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 1, 0, 0),
	GATE_SYS_NP(CLK_ETC_BUS0_AXI, "etc_bus0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 2, 0, 0),
	GATE_SYS_NP(CLK_FSYS_BUS0_AXI, "fsys_bus0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 3, 0, 0),
	GATE_SYS_NP(CLK_DMA_BUS0_AXI, "dma_bus0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 4, 0, 0),
	GATE_SYS_NP(CLK_CFG_BUS0_AXI, "cfg_bus0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 5, 0, 0),
	GATE_SYS_NP(CLK_AXISRAM0_AXI, "axisram0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 6, 0, 0),
	GATE_SYS_NP(CLK_MP2TSI0_AXI, "mp2tsi0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 7, 0, 0),
	GATE_SYS_NP(CLK_MP2TSI1_AXI, "mp2tsi1_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 8, 0, 0),
	GATE_SYS_NP(CLK_DMA0_AXI, "dma0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 10, 0, 0),
	GATE_SYS_NP(CLK_SSS0_AXI, "sss0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 11, 0, 0),
	GATE_SYS_NP(CLK_DMA1_AXI, "dma1_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 12, 0, 0),
	GATE_SYS_NP(CLK_SDMA0_AXI, "sdma0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 13, 0, 0),
	GATE_SYS_NP(CLK_SDMA1_AXI, "sdma1_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 14, 0, 0),
	GATE_SYS_NP(CLK_MDMA0_AXI, "mdma0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 15, 0, 0),
	GATE_SYS_NP(CLK_SDMMC0_AXI, "sdmmc0_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 16, 0, 0),
	GATE_SYS_NP(CLK_SDMMC1_AXI, "sdmmc1_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 17, 0, 0),
	GATE_SYS_NP(CLK_SDMMC2_AXI, "sdmmc2_axi", "div_sys_sys0_axi",
		  SYS0_AXI + 0x10, 18, 0, 0),

	GATE_SYS_NP(CLK_SYS0_APB, "sys0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 19, 0, 0),
	GATE_SYS_NP(CLK_DPC_PAD_PLACE0, "dpc_pad_place0", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 20, 0, 0),
	GATE_SYS_NP(CLK_I2S0_APB, "i2s0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 21, 0, 0),
	GATE_SYS_NP(CLK_I2S1_APB, "i2s1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 22, 0, 0),
	GATE_SYS_NP(CLK_I2S2_APB, "i2s2_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 23, 0, 0),
	GATE_SYS_NP(CLK_I2S3_APB, "i2s3_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 24, 0, 0),
	GATE_SYS_NP(CLK_MP2TSI0_APB, "mp2tsi0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 25, 0, 0),
	GATE_SYS_NP(CLK_MP2TSI1_APB, "mp2tsi1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 26, 0, 0),
	GATE_SYS_NP(CLK_WDT0_APB, "wdt0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 27, 0, 0),
	GATE_SYS_NP(CLK_WDT0_POR, "wdt0_por", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 28, 0, 0),
	GATE_SYS_NP(CLK_SECURE_WDT0_APB, "secure_wdt0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 29, 0, 0),
	GATE_SYS_NP(CLK_SECURE_WDT0_POR, "secure_wdt0_por", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 30, 0, 0),
	GATE_SYS_NP(CLK_SYSREG_SYS0_APB, "sysreg_sys0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x10, 31, 0, 0),

	GATE_SYS_NP(CLK_ECID0_APB, "ecid0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 0, 0, 0),
	GATE_SYS_NP(CLK_SYSCTRL_TOP_APB, "sysctrl_top_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 1, 0, 0),
	GATE_SYS_NP(CLK_CAN0_APB, "can0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 2, 0, 0),
	GATE_SYS_NP(CLK_CAN1_APB, "can1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 3, 0, 0),
	GATE_SYS_NP(CLK_TMU0_APB, "tmu0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 4, CLK_IS_CRITICAL, 0),
	GATE_SYS_NP(CLK_DMA0_APB, "dma0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 5, 0, 0),
	GATE_SYS_NP(CLK_SSS0_APB, "sss0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 6, 0, 0),
	GATE_SYS_NP(CLK_DMA1_APB, "dma1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 7, 0, 0),
	GATE_SYS_NP(CLK_SMC0_APB, "smc0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 8, 0, 0),
	GATE_SYS_NP(CLK_GPIO0_APB, "gpio0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 9, 0, 0),
	GATE_SYS_NP(CLK_SDMA0_APB, "sdma0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 10, 0, 0),
	GATE_SYS_NP(CLK_GPIO1_APB, "gpio1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 11, 0, 0),
	GATE_SYS_NP(CLK_SDMA1_APB, "sdma1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 12, 0, 0),
	GATE_SYS_NP(CLK_GPIO2_APB, "gpio2_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 13, 0, 0),
	GATE_SYS_NP(CLK_GPIO3_APB, "gpio3_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 14, 0, 0),
	GATE_SYS_NP(CLK_GPIO4_APB, "gpio4_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 15, 0, 0),
	GATE_SYS_NP(CLK_ADC0_APB, "adc0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 16, 0, 0),
	GATE_SYS_NP(CLK_MDMA0_APB, "mdma0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 17, 0, 0),
	GATE_SYS_NP(CLK_DUMMY0_APB, "dummy0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 18, 0, 0),
	GATE_SYS_NP(CLK_SPDIFTX0_APB, "spdiftx0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 19, 0, 0),
	GATE_SYS_NP(CLK_SPDIFRX0_APB, "spdifrx0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 20, 0, 0),
	GATE_SYS_NP(CLK_SDMMC0_APB, "sdmmc0_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 21, 0, 0),
	GATE_SYS_NP(CLK_SDMMC1_APB, "sdmmc1_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 22, 0, 0),
	GATE_SYS_NP(CLK_SDMMC2_APB, "sdmmc2_apb", "div_sys_sys0_apb",
		  SYS0_AXI + 0x14, 23, 0, 0),

	GATE_SYS_NP(CLK_SYS0_HSIF_AXI, "sys0_hsif_axi", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 0, CLK_IS_CRITICAL, 0),
	GATE_SYS_NP(CLK_BLK_HSIF_DATA_BUS0_AXI, "blk_hsif_data_bus0_axi",
		  "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 1, CLK_IS_CRITICAL, 0),
	GATE_SYS_NP(CLK_SDMMC0_AHB, "sdmmc0_ahb", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 2, 0, 0),
	GATE_SYS_NP(CLK_SDMMC1_AHB, "sdmmc1_ahb", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 3, 0, 0),
	GATE_SYS_NP(CLK_SDMMC2_AHB, "sdmmc2_ahb", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 4, 0, 0),
	GATE_SYS_NP(CLK_GMAC_RGMII0_AXI, "gmac_rgmii0_axi", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 5, 0, 0),
	GATE_SYS_NP(CLK_GMAC_RMII0_AXI, "gmac_rmii0_axi", "div_sys_sys0_hsif_axi",
		  SYS0_HSIF_AXI + 0x10, 6, 0, 0),

	GATE_SYS_NP(CLK_SYS0_HSIF_APB, "sys0_hsif_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 7, CLK_IS_CRITICAL, 0),
	GATE_SYS_NP(CLK_BLK_HSIF_DATA_BUS0_APB, "blk_hsif_data_bus0_apb",
		  "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 8, 0, 0),
	GATE_SYS_NP(CLK_HSIF0_APB, "hsif0_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 9, CLK_IS_CRITICAL, 0),
	GATE_SYS_NP(CLK_SYSREG_HSIF0_APB, "sysreg_hsif0_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 10, 0, 0),
	GATE_SYS_NP(CLK_GMAC_RGMII0_APB, "gmac_rgmii0_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 11, 0, 0),
	GATE_SYS_NP(CLK_GMAC_RMII0_APB, "gmac_rmii0_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 12, 0, 0),
	GATE_SYS_NP(CLK_NANDC0_APB, "nandc0_apb", "div_sys_sys0_hsif_apb",
		  SYS0_HSIF_AXI + 0x10, 13, 0, 0),
	GATE_SYS_NP(CLK_CPU_BACKUP0, "cpu_backup0", "div_sys_cpu_backup0",
		  CPU_BACKUP0 + 0x10, 0, 0, 0),
	GATE_SYS_NP(CLK_CSSYS0_HCLK, "cssys0_hclk",
		 "div_sys_cssys0_hclk", CSSYS0_HCLK + 0x10, 0, 0, 0),
	GATE_SYS_NP(CLK_BLK_CMU0_APB, "blk_cmu0_apb",
		 "div_sys_blk_cmu0_apb", BLK_CMU0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_VIP_PADOUT0, "vip_padout0",
		 "div_sys_vip_padout0", VIP_PADOUT0 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_VIP_PADOUT1, "vip_padout1",
		 "div_sys_vip_padout1", VIP_PADOUT1 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_VIP_PADOUT2, "vip_padout2",
		 "div_sys_vip_padout2", VIP_PADOUT2 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_VIP_PADOUT3, "vip_padout3",
		 "div_sys_vip_padout3", VIP_PADOUT3 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_VIP_PADOUT4, "vip_padout4",
		 "div_sys_vip_padout4", VIP_PADOUT4 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_HPM_SYS0, "hpm_sys0",
		 "div_sys_hpm_sys0", HPM_SYS0 + 0x10, 0, 0, 0),
	GATE_SYS_NP(CLK_UART0_CORE, "uart0_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 0, 0, 0),
	GATE_SYS_NP(CLK_UART1_CORE, "uart1_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 1, 0, 0),
	GATE_SYS_NP(CLK_UART2_CORE, "uart2_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 2, 0, 0),
	GATE_SYS_NP(CLK_UART3_CORE, "uart3_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 3, 0, 0),
	GATE_SYS_NP(CLK_UART4_CORE, "uart4_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 4, 0, 0),
	GATE_SYS_NP(CLK_UART5_CORE, "uart5_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 5, 0, 0),
	GATE_SYS_NP(CLK_UART6_CORE, "uart6_core",
		 "div_sys_uart0_core", UART0_CORE + 0x10, 6, 0, 0),
	GATE_SYS(CLK_UART0_APB, "uart0_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_UART1_APB, "uart1_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 1, 0, 0),
	GATE_SYS(CLK_UART2_APB, "uart2_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 2, 0, 0),
	GATE_SYS(CLK_UART3_APB, "uart3_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 3, 0, 0),
	GATE_SYS(CLK_UART4_APB, "uart4_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 4, 0, 0),
	GATE_SYS(CLK_UART5_APB, "uart5_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 5, 0, 0),
	GATE_SYS(CLK_UART6_APB, "uart6_apb",
		 "div_sys_uart0_apb", UART0_APB + 0x10, 6, 0, 0),
	GATE_SYS(CLK_I2S0_CORE, "i2s0_core",
		 "div_sys_i2s0_core", I2S0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_I2S1_CORE, "i2s1_core",
		 "div_sys_i2s1_core", I2S1_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_I2S2_CORE, "i2s2_core",
		 "div_sys_i2s2_core", I2S2_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_I2S3_CORE, "i2s3_core",
		 "div_sys_i2s3_core", I2S3_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_I2C0_APB, "i2c0_apb",
		 "div_sys_i2c0_apb", I2C0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_I2C1_APB, "i2c1_apb",
		 "div_sys_i2c0_apb", I2C0_APB + 0x10, 1, 0, 0),
	GATE_SYS(CLK_I2C2_APB, "i2c2_apb",
		 "div_sys_i2c0_apb", I2C0_APB + 0x10, 2, 0, 0),
	GATE_SYS(CLK_I2C3_APB, "i2c3_apb",
		 "div_sys_i2c0_apb", I2C0_APB + 0x10, 3, 0, 0),
	GATE_SYS(CLK_I2C4_APB, "i2c4_apb",
		 "div_sys_i2c0_apb", I2C0_APB + 0x10, 4, 0, 0),
	GATE_SYS(CLK_SDMMC0_CORE, "sdmmc0_core",
		 "div_sys_sdmmc0_core", SDMMC0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SDMMC1_CORE, "sdmmc1_core",
		 "div_sys_sdmmc1_core", SDMMC1_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SDMMC2_CORE, "sdmmc2_core",
		 "div_sys_sdmmc2_core", SDMMC2_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SPI0_APB, "spi0_apb",
		 "div_sys_spi0_apb", SPI0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SPI1_APB, "spi1_apb",
		 "div_sys_spi0_apb", SPI0_APB + 0x10, 1, 0, 0),
	GATE_SYS(CLK_SPI2_APB, "spi2_apb",
		 "div_sys_spi0_apb", SPI0_APB + 0x10, 2, 0, 0),
	GATE_SYS(CLK_SPI0_CORE, "spi0_core",
		 "div_sys_spi0_core", SPI0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SPI1_CORE, "spi1_core",
		 "div_sys_spi0_core", SPI0_CORE + 0x10, 1, 0, 0),
	GATE_SYS(CLK_SPI2_CORE, "spi2_core",
		 "div_sys_spi0_core", SPI0_CORE + 0x10, 2, 0, 0),
	GATE_SYS(CLK_PDM0_AXI, "pdm0_axi",
		 "div_sys_pdm0_axi", PDM0_AXI + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PDM0_CORE, "pdm0_core",
		 "div_sys_pdm0_core", PDM0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PWM0_APB, "pwm0_apb",
		 "div_sys_pwm0_apb", PWM0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PWM0_TCLK0, "pwm0_tclk0",
		 "div_sys_pwm0_tclk0", PWM0_TCLK0 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PWM0_TCLK1, "pwm0_tclk1",
		 "div_sys_pwm0_tclk1", PWM0_TCLK1 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PWM0_TCLK2, "pwm0_tclk2",
		 "div_sys_pwm0_tclk2", PWM0_TCLK2 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_PWM0_TCLK3, "pwm0_tclk3",
		 "div_sys_pwm0_tclk3", PWM0_TCLK3 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_CAN0_CORE, "can0_core",
		 "div_sys_can0_core", CAN0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_CAN1_CORE, "can1_core",
		 "div_sys_can1_core", CAN1_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_TIMER0_APB, "timer0_apb",
		 "div_sys_timer0_apb", TIMER0_APB + 0x10, 0, 0, 0),
	GATE_SYS(CLK_TIMER0_TCLK0, "timer0_tclk0",
		 "div_sys_timer0_tclk0", TIMER0_TCLK0 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_TIMER0_TCLK1, "timer0_tclk1",
		 "div_sys_timer0_tclk1", TIMER0_TCLK1 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_TIMER0_TCLK2, "timer0_tclk2",
		 "div_sys_timer0_tclk2", TIMER0_TCLK2 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_TIMER0_TCLK3, "timer0_tclk3",
		 "div_sys_timer0_tclk3", TIMER0_TCLK3 + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SECURE_TIMER0_APB, "secure_timer0_apb",
		 "div_sys_secure_timer0_apb", SECURE_TIMER0_APB + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_SECURE_TIMER0_TCLK0, "secure_timer0_tclk0",
		 "div_sys_secure_timer0_tclk0", SECURE_TIMER0_TCLK0 + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_SECURE_TIMER0_TCLK1, "secure_timer0_tclk1",
		 "div_sys_secure_timer0_tclk1", SECURE_TIMER0_TCLK1 + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_SECURE_TIMER0_TCLK2, "secure_timer0_tclk2",
		 "div_sys_secure_timer0_tclk2", SECURE_TIMER0_TCLK2 + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_SECURE_TIMER0_TCLK3, "secure_timer0_tclk3",
		 "div_sys_secure_timer0_tclk3", SECURE_TIMER0_TCLK3 + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_SMC0_AXI, "smc0_axi",
		 "div_sys_smc0_axi", SMC0_AXI + 0x10, 0, 0, 0),
	GATE_SYS(CLK_SPDIFTX0_CORE, "spdiftx0_core",
		 "div_sys_spdiftx0_core", SPDIFTX0_CORE + 0x10, 0, 0, 0),
	GATE_SYS(CLK_GMAC_RGMII0_TX, "gmac_rgmii0_tx",
		 "div_sys_gmac_rgmii0_tx", GMAC_RGMII0_TX + 0x10, 0, 0, 0),
	GATE_SYS(CLK_GMAC_RGMII0_PTP_REF, "gmac_rgmii0_ptp_ref",
		 "div_sys_gmac_rgmii0_ptp_ref", GMAC_RGMII0_PTP_REF + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_GMAC_RMII0_PTP_REF, "gmac_rmii0_ptp_ref",
		 "div_sys_gmac_rmii0_ptp_ref", GMAC_RMII0_PTP_REF + 0x10,
		 0, 0, 0),
	GATE_SYS(CLK_NANDC0_AXI, "nandc0_axi",
		 "div_sys_nandc0_axi", NANDC0_AXI + 0x10, 0, 0, 0),
};

static const struct nexell_clk_reset sys_resets[] = {
	[CLK_RESET_SPI0_CORE] =
		CLK_RESET(SPI0_CORE + 0x40, SPI0_CORE + 0x30, 0),
	[CLK_RESET_SPI1_CORE] =
		CLK_RESET(SPI0_CORE + 0x40, SPI0_CORE + 0x30, 1),
	[CLK_RESET_SPI2_CORE] =
		CLK_RESET(SPI0_CORE + 0x40, SPI0_CORE + 0x30, 2),
};

#define CLK_SRC_REG(o)						\
	CLK_REG(o, 0xff),					\
	CLK_REG_CLR(o + 0x10, o + 0x20, 0x1),			\
	CLK_REG(o + 0x60, 0xff)					\

static const struct nexell_clk_reg src_regs[] __initconst = {
	CLK_SRC_REG(SYS0_AXI),
	CLK_SRC_REG(SYS0_HSIF_AXI),
	CLK_SRC_REG(CPU_BACKUP0),
	CLK_SRC_REG(CSSYS0_HCLK),
	CLK_SRC_REG(BLK_CMU0_APB),
	CLK_SRC_REG(VIP_PADOUT0),
	CLK_SRC_REG(VIP_PADOUT1),
	CLK_SRC_REG(VIP_PADOUT2),
	CLK_SRC_REG(VIP_PADOUT3),
	CLK_SRC_REG(VIP_PADOUT4),
	CLK_SRC_REG(HPM_SYS0),
	CLK_SRC_REG(UART0_CORE),
	CLK_SRC_REG(UART0_APB),
	CLK_SRC_REG(I2S0_CORE),
	CLK_SRC_REG(I2S1_CORE),
	CLK_SRC_REG(I2S2_CORE),
	CLK_SRC_REG(I2S3_CORE),
	CLK_SRC_REG(I2C0_APB),
	CLK_SRC_REG(SDMMC0_CORE),
	CLK_SRC_REG(SDMMC1_CORE),
	CLK_SRC_REG(SDMMC2_CORE),
	CLK_SRC_REG(SPI0_APB),
	CLK_SRC_REG(SPI0_CORE),
	CLK_SRC_REG(PDM0_AXI),
	CLK_SRC_REG(PDM0_CORE),
	CLK_SRC_REG(PWM0_APB),
	CLK_SRC_REG(PWM0_TCLK0),
	CLK_SRC_REG(PWM0_TCLK1),
	CLK_SRC_REG(PWM0_TCLK2),
	CLK_SRC_REG(PWM0_TCLK3),
	CLK_SRC_REG(CAN0_CORE),
	CLK_SRC_REG(CAN1_CORE),
	CLK_SRC_REG(TIMER0_APB),
	CLK_SRC_REG(TIMER0_TCLK0),
	CLK_SRC_REG(TIMER0_TCLK1),
	CLK_SRC_REG(TIMER0_TCLK2),
	CLK_SRC_REG(TIMER0_TCLK3),
	CLK_SRC_REG(SECURE_TIMER0_APB),
	CLK_SRC_REG(SECURE_TIMER0_TCLK0),
	CLK_SRC_REG(SECURE_TIMER0_TCLK1),
	CLK_SRC_REG(SECURE_TIMER0_TCLK2),
	CLK_SRC_REG(SECURE_TIMER0_TCLK3),
	CLK_SRC_REG(SMC0_AXI),
	CLK_SRC_REG(SPDIFTX0_CORE),
	CLK_SRC_REG(GMAC_RGMII0_TX),
	CLK_SRC_REG(GMAC_RGMII0_PTP_REF),
	CLK_SRC_REG(GMAC_RMII0_PTP_REF),
	CLK_SRC_REG(NANDC0_AXI),
	CLK_SRC_REG(MM0_AXI),
	CLK_SRC_REG(VIP0_PADOUT0),
	CLK_SRC_REG(VIP0_PADOUT1),
	CLK_SRC_REG(DPC0_X2),
	CLK_SRC_REG(LVDS0_VCLK),
	CLK_SRC_REG(CODA960_0_CORE),
	CLK_SRC_REG(USB0_AHB),
};

#define CLK_SYS_REG(o)						\
	CLK_REG_CLR(o + 0x10, o + 0x20, 0xffffffff),		\
	CLK_REG(o + 0x60, 0xff)					\

static const struct nexell_clk_reg sys_regs[] __initconst = {
	CLK_SYS_REG(SYS0_AXI),
	CLK_SYS_REG(SYS0_AXI + 0x4),
	CLK_SRC_REG(SYS0_HSIF_AXI),
	CLK_REG(SYS0_HSIF_AXI + 0x64, 0xff),
	CLK_SYS_REG(CPU_BACKUP0),
	CLK_SYS_REG(CSSYS0_HCLK),
	CLK_SYS_REG(BLK_CMU0_APB),
	CLK_SYS_REG(VIP_PADOUT0),
	CLK_SYS_REG(VIP_PADOUT1),
	CLK_SYS_REG(VIP_PADOUT2),
	CLK_SYS_REG(VIP_PADOUT3),
	CLK_SYS_REG(VIP_PADOUT4),
	CLK_SYS_REG(HPM_SYS0),
	CLK_SYS_REG(UART0_CORE),
	CLK_SYS_REG(UART0_APB),
	CLK_SYS_REG(I2S0_CORE),
	CLK_SYS_REG(I2S1_CORE),
	CLK_SYS_REG(I2S2_CORE),
	CLK_SYS_REG(I2S3_CORE),
	CLK_SYS_REG(I2C0_APB),
	CLK_SYS_REG(SDMMC0_CORE),
	CLK_SYS_REG(SDMMC1_CORE),
	CLK_SYS_REG(SDMMC2_CORE),
	CLK_SYS_REG(SPI0_APB),
	CLK_SYS_REG(SPI0_CORE),
	CLK_SYS_REG(PDM0_AXI),
	CLK_SYS_REG(PDM0_CORE),
	CLK_SYS_REG(PWM0_APB),
	CLK_SYS_REG(PWM0_TCLK0),
	CLK_SYS_REG(PWM0_TCLK1),
	CLK_SYS_REG(PWM0_TCLK2),
	CLK_SYS_REG(PWM0_TCLK3),
	CLK_SYS_REG(CAN0_CORE),
	CLK_SYS_REG(CAN1_CORE),
	CLK_SYS_REG(TIMER0_APB),
	CLK_SYS_REG(TIMER0_TCLK0),
	CLK_SYS_REG(TIMER0_TCLK1),
	CLK_SYS_REG(TIMER0_TCLK2),
	CLK_SYS_REG(TIMER0_TCLK3),
	CLK_SYS_REG(SECURE_TIMER0_APB),
	CLK_SYS_REG(SECURE_TIMER0_TCLK0),
	CLK_SYS_REG(SECURE_TIMER0_TCLK1),
	CLK_SYS_REG(SECURE_TIMER0_TCLK2),
	CLK_SYS_REG(SECURE_TIMER0_TCLK3),
	CLK_SYS_REG(SMC0_AXI),
	CLK_SYS_REG(SPDIFTX0_CORE),
	CLK_SYS_REG(GMAC_RGMII0_TX),
	CLK_SYS_REG(GMAC_RGMII0_PTP_REF),
	CLK_SYS_REG(GMAC_RMII0_PTP_REF),
	CLK_SYS_REG(NANDC0_AXI),
};

static LIST_HEAD(nxp3220_cmu_reg_cache_list);

static int nxp3220_cmu_clk_suspend(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_reg_cache_list, node)
		nexell_clk_save(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
	return 0;
}

static void nxp3220_cmu_clk_resume(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_reg_cache_list, node)
		nexell_clk_restore(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
}

static struct syscore_ops nxp3220_cmu_syscore_ops = {
	.suspend = nxp3220_cmu_clk_suspend,
	.resume = nxp3220_cmu_clk_resume,
};

static void __init nxp3220_cmu_init(struct device_node *np)
{
	void __iomem *reg, *sys_reg;
	struct nexell_clk_data *ctx;

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	sys_reg = of_iomap(np, 1);
	if (!sys_reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	ctx = nexell_clk_init(reg, CLK_CMU_NR);
	if (!ctx) {
		pr_err("%s: Failed to initialize clock data\n", __func__);
		return;
	}

	nxp3220_clk_register_composite(ctx, src_clks, ARRAY_SIZE(src_clks));

	nexell_clk_sleep_init(ctx->reg, &nxp3220_cmu_syscore_ops,
			      &nxp3220_cmu_reg_cache_list,
			      src_regs, ARRAY_SIZE(src_regs));

	/* Register CMU_SYS clocks */
	ctx->reg = sys_reg;

	nexell_clk_register_div(ctx, sys_div_clks, ARRAY_SIZE(sys_div_clks));
	nxp3220_clk_register_gate(ctx, sys_gate_clks,
				  ARRAY_SIZE(sys_gate_clks));

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: failed to add clock provider\n", __func__);

	nexell_clk_sleep_init(ctx->reg, &nxp3220_cmu_syscore_ops,
			      &nxp3220_cmu_reg_cache_list,
			      sys_regs, ARRAY_SIZE(sys_regs));

	/* Register reset controls */
	ctx->reset.ops = &nexell_clk_reset_ops;
	ctx->reset.nr_resets = ARRAY_SIZE(sys_resets);
	ctx->reset.of_node = np;

	ctx->reset_table = sys_resets;
	ctx->max_reset_id = CLK_RESET_CMU_NR;

	if (reset_controller_register(&ctx->reset))
		pr_err("%s: failed to register clock reset controller\n",
		       __func__);
}
CLK_OF_DECLARE(nxp3220_cmu, "nexell,nxp3220-cmu", nxp3220_cmu_init);
