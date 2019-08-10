// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 SoC Clock CMU-CPU driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/nxp3220-clk.h>
#include "clk-nexell.h"
#include "clk-nxp3220.h"

#define CPU_CPU0_ARM			0x0200
#define PLL_CPU_DIV0			0x0400
#define HPM_CPU0			0x0600

#define DIV_CPU(_id, cname, pname, o)	\
	__DIV(_id, NULL, cname, pname, o, 0, 8, 0, 0)

/* FIXME: resolve div_sys_cpu_backup0 dependency chain */
PNAME(cpu_mux_p) = { "pll_cpu_div" };

static const struct nexell_mux_clock cpu_mux_clks[] __initconst = {
	MUX(CLK_MUX_CPU_ARM, "mux_cpu_arm", cpu_mux_p, CPU_CPU0_ARM, 0, 4),
	MUX(CLK_MUX_CPU_HPM, "mux_cpu_hpm", cpu_mux_p, HPM_CPU0, 0, 4),
};

static const struct nexell_div_clock cpu_div_clks[] __initconst = {
	DIV_CPU(CLK_CPU_DIV_ARM, "div_cpu_arm", "mux_cpu_arm",
		CPU_CPU0_ARM + 0x60),
	DIV_CPU(CLK_CPU_DIV_AXI, "div_cpu_axi", "div_cpu_arm",
		CPU_CPU0_ARM + 0x64),
	DIV_CPU(CLK_CPU_DIV_ATCLK, "div_cpu_atclk", "div_cpu_arm",
		CPU_CPU0_ARM + 0x68),
	DIV_CPU(CLK_CPU_DIV_CNTCLK, "div_cpu_cntclk", "div_cpu_arm",
		CPU_CPU0_ARM + 0x6c),
	DIV_CPU(CLK_CPU_DIV_TSCLK, "div_cpu_tsclk", "div_cpu_arm",
		CPU_CPU0_ARM + 0x70),
	DIV_CPU(CLK_CPU_DIV_DBGAPB, "div_cpu_dbgapb", "div_cpu_arm",
		CPU_CPU0_ARM + 0x74),
	DIV_CPU(CLK_CPU_DIV_APB, "div_cpu_apb", "div_cpu_arm",
		CPU_CPU0_ARM + 0x78),
	DIV_CPU(CLK_CPU_DIV_HPM, "div_cpu_hpm", "mux_cpu_hpm",
		HPM_CPU0 + 0x60),
};

#define GATE_CPU(_id, cname, pname, o, b, f, gf)		\
	GATE(_id, cname, pname, o, o + 0x10, b, (f) | CLK_IGNORE_UNUSED, gf)

static const struct nexell_gate_clock cpu_gate_clks[] __initconst = {
	GATE_CPU(CLK_CPU_ARM, "cpu_arm", "div_cpu_arm",
	     CPU_CPU0_ARM + 0x10, 0, 0, 0),
	GATE_CPU(CLK_CPU_AXI, "cpu_axi", "div_cpu_axi",
	     CPU_CPU0_ARM + 0x10, 2, 0, 0),
	GATE_CPU(CLK_CPU_AXIM, "cpu_axim", "div_cpu_axi",
	     CPU_CPU0_ARM + 0x10, 3, 0, 0),
	GATE_CPU(CLK_CPU_ATCLK, "cpu_atclk", "div_cpu_atclk",
	     CPU_CPU0_ARM + 0x10, 4, 0, 0),
	GATE_CPU(CLK_CPU_CNTCLK, "cpu_cntclk", "div_cpu_cntclk",
	     CPU_CPU0_ARM + 0x10, 5, 0, 0),
	GATE_CPU(CLK_CPU_TSCLK, "cpu_tsclk", "div_cpu_tsclk",
	     CPU_CPU0_ARM + 0x10, 6, 0, 0),
	GATE_CPU(CLK_CPU_DBGAPB, "cpu_dbgapb", "div_cpu_dbgapb",
	     CPU_CPU0_ARM + 0x10, 7, 0, 0),
	GATE_CPU(CLK_CPU_APB, "cpu_apb", "div_cpu_apb",
	     CPU_CPU0_ARM + 0x10, 8, 0, 0),
	GATE_CPU(CLK_CPU_SYSREG_APB, "cpu_sysreg_apb", "div_cpu_apb",
	     CPU_CPU0_ARM + 0x10, 9, 0, 0),
	GATE_CPU(CLK_CPU_AXIM_APB, "cpu_axim_apb", "div_cpu_apb",
	     CPU_CPU0_ARM + 0x10, 10, 0, 0),

	GATE_CPU(CLK_CPU_PLL_DIV, "cpu_pll_div", "pll_cpu_div",
	     PLL_CPU_DIV0 + 0x10, 0, 0, 0),
	GATE_CPU(CLK_CPU_HPM, "cpu_hpm", "div_cpu_hpm",
	     HPM_CPU0 + 0x10, 0, 0, 0),
};

#define CLK_CPU_REG(o)						\
	CLK_REG_CLR(o + 0x10, o + 0x20, 0xffffffff),		\
	CLK_REG(o + 0x60, 0xff)					\

static const struct nexell_clk_reg cmu_cpu_regs[] __initconst = {
	CLK_REG(CPU_CPU0_ARM, 0xf),
	CLK_CPU_REG(CPU_CPU0_ARM),
	CLK_REG(CPU_CPU0_ARM + 0x64, 0xff),
	CLK_REG(CPU_CPU0_ARM + 0x68, 0xff),
	CLK_REG(CPU_CPU0_ARM + 0x6c, 0xff),
	CLK_REG(CPU_CPU0_ARM + 0x70, 0xff),
	CLK_REG(CPU_CPU0_ARM + 0x74, 0xff),
	CLK_REG(CPU_CPU0_ARM + 0x78, 0xff),
	CLK_CPU_REG(PLL_CPU_DIV0),
	CLK_CPU_REG(HPM_CPU0),
};

static LIST_HEAD(nxp3220_cmu_cpu_reg_cache_list);

static int nxp3220_cmu_cpu_clk_suspend(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_cpu_reg_cache_list, node)
		nexell_clk_save(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
	return 0;
}

static void nxp3220_cmu_cpu_clk_resume(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_cpu_reg_cache_list, node)
		nexell_clk_restore(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
}

static struct syscore_ops nxp3220_cmu_cpu_syscore_ops = {
	.suspend = nxp3220_cmu_cpu_clk_suspend,
	.resume = nxp3220_cmu_cpu_clk_resume,
};

static void __init nxp3220_cmu_cpu_init(struct device_node *np)
{
	void __iomem *reg;
	struct nexell_clk_data *ctx;

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	ctx = nexell_clk_init(reg, CLK_CPU_NR);
	if (!ctx) {
		pr_err("%s: Failed to initialize clock data\n", __func__);
		return;
	}

	nexell_clk_register_mux(ctx, cpu_mux_clks, ARRAY_SIZE(cpu_mux_clks));
	nexell_clk_register_div(ctx, cpu_div_clks, ARRAY_SIZE(cpu_div_clks));
	nxp3220_clk_register_gate(ctx, cpu_gate_clks,
				  ARRAY_SIZE(cpu_gate_clks));

	nexell_clk_sleep_init(ctx->reg, &nxp3220_cmu_cpu_syscore_ops,
			      &nxp3220_cmu_cpu_reg_cache_list,
			      cmu_cpu_regs, ARRAY_SIZE(cmu_cpu_regs));

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: failed to add clock provider\n", __func__);
}
CLK_OF_DECLARE(nxp3220_cmu_cpu, "nexell,nxp3220-cmu-cpu",
	       nxp3220_cmu_cpu_init);
