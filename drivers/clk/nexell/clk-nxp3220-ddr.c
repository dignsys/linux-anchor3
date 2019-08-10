// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 SoC Clock CMU-DDR driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/of_address.h>
#include <dt-bindings/clock/nxp3220-clk.h>
#include "clk-nexell.h"
#include "clk-nxp3220.h"

#define DDR_DDR0		0x0200
#define DDR_AXI			0x0400
#define PLL_DDR0_DIV		0x0600
#define PLL_DDR1_DIV		0x0800

#define DIV_DDR(_id, cname, pname, o)	\
	__DIV(_id, NULL, cname, pname, o, 0, 8, 0, 0)

PNAME(ddr_mux_p) = { "pll_ddr0_div", "pll_ddr1_div" };

static const struct nexell_mux_clock ddr_mux_clks[] __initconst = {
	MUX(CLK_MUX_DDR, "mux_ddr", ddr_mux_p, DDR_DDR0, 0, 1),
};

static const struct nexell_div_clock ddr_div_clks[] __initconst = {
	DIV_DDR(CLK_DDR_DIV, "div_ddr", "mux_ddr",
		DDR_DDR0 + 0x60),
	DIV_DDR(CLK_DDR_DIV_AXI, "div_ddr_axi", "mux_ddr",
		DDR_AXI + 0x60),
	DIV_DDR(CLK_DDR_DIV_APB, "div_ddr_apb", "div_ddr_axi",
		DDR_AXI + 0x64),
	DIV_DDR(CLK_DDR_DIV_PLL_DDR0, "div_pll_ddr0", "pll_ddr0_div",
		PLL_DDR0_DIV + 0x60),
	DIV_DDR(CLK_DDR_DIV_PLL_DDR1, "div_pll_ddr1", "pll_ddr1_div",
		PLL_DDR1_DIV + 0x60),
};

#define GATE_DDR(_id, cname, pname, o, b, f, gf)		\
	GATE(_id, cname, pname, o, o + 0x10, b, (f) | CLK_IGNORE_UNUSED, gf)

static const struct nexell_gate_clock ddr_gate_clks[] __initconst = {
	GATE_DDR(CLK_DDR, "ddr", "div_ddr",
	     DDR_DDR0 + 0x10, 0, 0, 0),
	GATE_DDR(CLK_DDR_AXI, "ddr_axi", "div_ddr_axi",
	     DDR_AXI + 0x10, 0, 0, 0),
	GATE_DDR(CLK_DDR_TZASC, "ddr_tzasc", "div_ddr_axi",
	     DDR_AXI + 0x10, 1, 0, 0),
	GATE_DDR(CLK_DDR_APB, "ddr_apb", "div_ddr_apb",
	     DDR_AXI + 0x10, 2, 0, 0),
	GATE_DDR(CLK_DDR_SYSREG_APB, "ddr_sysreg_apb", "div_ddr_apb",
	     DDR_AXI + 0x10, 3, 0, 0),
	GATE_DDR(CLK_PLL_DDR0, "pll_ddr0_gate", "pll_ddr0_div",
	     PLL_DDR0_DIV + 0x10, 0, 0, 0),
	GATE_DDR(CLK_PLL_DDR1, "pll_ddr1_gate", "pll_ddr1_div",
	     PLL_DDR1_DIV + 0x10, 0, 0, 0),
};

static void __init nxp3220_cmu_ddr_init(struct device_node *np)
{
	void __iomem *reg;
	struct nexell_clk_data *ctx;

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	ctx = nexell_clk_init(reg, CLK_DDR_NR);
	if (!ctx) {
		pr_err("%s: Failed to initialize clock data\n", __func__);
		return;
	}

	nexell_clk_register_mux(ctx, ddr_mux_clks, ARRAY_SIZE(ddr_mux_clks));
	nexell_clk_register_div(ctx, ddr_div_clks, ARRAY_SIZE(ddr_div_clks));
	nxp3220_clk_register_gate(ctx, ddr_gate_clks,
				  ARRAY_SIZE(ddr_gate_clks));

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: failed to add clock provider\n", __func__);
}
CLK_OF_DECLARE(nxp3220_cmu_ddr, "nexell,nxp3220-cmu-ddr",
	       nxp3220_cmu_ddr_init);
