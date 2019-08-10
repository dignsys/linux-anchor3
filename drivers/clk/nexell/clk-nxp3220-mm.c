// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 SoC Clock CMU-MM driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/nxp3220-clk.h>
#include <dt-bindings/reset/nexell,nxp3220-reset.h>
#include "clk-nexell.h"
#include "clk-nxp3220.h"

#define MM_AXI			0x0200
#define MM_VIP_PADOUT0		0x0400
#define MM_VIP_PADOUT1		0x0600
#define MM_DPC_X2		0x0800
#define MM_LVDS_VCLK		0x0A00
#define MM_CODA960_CORE		0x0C00

#define DIV_MM(_id, cname, pname, o)	\
	__DIV(_id, NULL, cname, pname, o, 0, 8, 0, 0)

#define DIV_MM_F(_id, cname, pname, o, f, df)	\
	__DIV(_id, NULL, cname, pname, o, 0, 8, f, df)

static const struct nexell_div_clock mm_div_clks[] __initconst = {
	DIV_MM(CLK_MM_DIV_AXI, "div_mm_axi", "src_mm0_axi",
	       MM_AXI + 0x60),
	DIV_MM(CLK_MM_DIV_APB, "div_mm_apb", "div_mm_axi",
	       MM_AXI + 0x64),
	DIV_MM(CLK_MM_DIV_VIP_PADOUT0, "div_mm_vip_padout0",
	       "src_vip0_padout0", MM_VIP_PADOUT0 + 0x60),
	DIV_MM(CLK_MM_DIV_VIP_PADOUT1, "div_mm_vip_padout1",
	       "src_vip0_padout1", MM_VIP_PADOUT1 + 0x60),
	DIV_MM_F(CLK_MM_DIV_DPC_X2, "div_mm_dpc_x2", "src_dpc0_x2",
	       MM_DPC_X2 + 0x60, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_MM_F(CLK_MM_DIV_DPC_X1, "div_mm_dpc_x1", "div_mm_dpc_x2",
	       MM_DPC_X2 + 0x64, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_MM_F(CLK_MM_DIV_LVDS_VCLK, "div_mm_lvds_vclk", "src_lvds0_vclk",
	       MM_LVDS_VCLK + 0x60, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_MM(CLK_MM_DIV_CODA960_CORE, "div_mm_coda960_core",
	       "src_coda960_0_core", MM_CODA960_CORE + 0x60),
};

#define GATE_MM(_id, cname, pname, o, b, f, gf)		\
	GATE(_id, cname, pname, o, o + 0x10, b, f, gf)

static const struct nexell_gate_clock mm_gate_clks[] __initconst = {
	GATE_MM(CLK_MM_AXI, "mm_axi", "div_mm_axi",
	     MM_AXI + 0x10, 0, CLK_IGNORE_UNUSED, 0),
	GATE_MM(CLK_MM_ROTATOR_AXI, "mm_rotator_axi", "div_mm_axi",
	     MM_AXI + 0x10, 2, 0, 0),
	GATE_MM(CLK_MM_G2D_AXI, "mm_g2d_axi", "div_mm_axi",
	     MM_AXI + 0x10, 3, 0, 0),
	GATE_MM(CLK_MM_DEINTERLACE_AXI, "mm_deinterlace_axi", "div_mm_axi",
	     MM_AXI + 0x10, 4, 0, 0),
	GATE_MM(CLK_MM_VIP_AXI, "mm_vip_axi", "div_mm_axi",
	     MM_AXI + 0x10, 5, 0, 0),
	GATE_MM(CLK_MM_DPC_AXI, "mm_dpc_axi", "div_mm_axi",
	     MM_AXI + 0x10, 6, 0, 0),
	GATE_MM(CLK_MM_CODA960_AXI, "mm_coda960_axi", "div_mm_axi",
	     MM_AXI + 0x10, 7, 0, 0),
	GATE_MM(CLK_MM_APB, "mm_apb", "div_mm_apb",
	     MM_AXI + 0x10, 8, CLK_IGNORE_UNUSED, 0),
	GATE_MM(CLK_MM_SYSREG_APB, "mm_sysreg_apb", "div_mm_apb",
	     MM_AXI + 0x10, 9, 0, 0),
	GATE_MM(CLK_MM_DEINTERLACE_APB, "mm_deinterlace_apb", "div_mm_apb",
	     MM_AXI + 0x10, 10, 0, 0),
	GATE_MM(CLK_MM_VIP_APB, "mm_vip_apb", "div_mm_apb",
	     MM_AXI + 0x10, 11, 0, 0),
	GATE_MM(CLK_MM_LVDS_PHY, "mm_lvds_phy", "div_mm_apb",
	     MM_AXI + 0x10, 12, 0, 0),
	GATE_MM(CLK_MM_CODA960_APB, "mm_coda960_apb", "div_mm_apb",
	     MM_AXI + 0x10, 13, 0, 0),
	GATE_MM(CLK_MM_VIP_PADOUT0, "mm_vip_padout0", "div_mm_vip_padout0",
	     MM_VIP_PADOUT0 + 0x10, 0, 0, 0),
	GATE_MM(CLK_MM_VIP_PADOUT1, "mm_vip_padout1", "div_mm_vip_padout1",
	     MM_VIP_PADOUT1 + 0x10, 0, 0, 0),
	GATE_MM(CLK_MM_DPC_X2, "mm_dpc_x2", "div_mm_dpc_x2",
	     MM_DPC_X2 + 0x10, 0, CLK_DIVIDER_ROUND_CLOSEST, 0),
	GATE_MM(CLK_MM_DPC_X1, "mm_dpc_x1", "div_mm_dpc_x1",
	     MM_DPC_X2 + 0x10, 1, CLK_DIVIDER_ROUND_CLOSEST, 0),
	GATE_MM(CLK_MM_LVDS_VCLK, "mm_lvds_vclk", "div_mm_lvds_vclk",
	     MM_LVDS_VCLK + 0x10, 0, 0, 0),
	GATE_MM(CLK_MM_CODA960_CORE, "mm_coda960_core", "div_mm_coda960_core",
	     MM_CODA960_CORE + 0x10, 0, 0, 0),
};

static const struct nexell_clk_reset mm_resets[] = {
	[CLK_RESET_MM_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 0),
	[CLK_RESET_MM_ROTATOR_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 2),
	[CLK_RESET_MM_G2D_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 3),
	[CLK_RESET_MM_DEINTERLACE_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 4),
	[CLK_RESET_MM_VIP_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 5),
	[CLK_RESET_MM_DPC_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 6),
	[CLK_RESET_MM_CODA960_AXI] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 7),
	[CLK_RESET_MM_APB] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 8),
	[CLK_RESET_MM_SYSREG_APB] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 9),
	[CLK_RESET_MM_DEINTERLACE_APB] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 10),
	[CLK_RESET_MM_CODA960_APB] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 13),
	[CLK_RESET_MM_DPC_X2] =
		CLK_RESET(MM_DPC_X2 + 0x40, MM_DPC_X2 + 0x30, 0),
	[CLK_RESET_MM_LVDS_VCLK] =
		CLK_RESET(MM_LVDS_VCLK + 0x40, MM_LVDS_VCLK + 0x30, 0),
	[CLK_RESET_MM_CODA960_CORE] =
		CLK_RESET(MM_CODA960_CORE + 0x40, MM_CODA960_CORE + 0x30, 0),
	[CLK_RESET_MM_LVDS_PHY] =
		CLK_RESET(MM_AXI + 0x40, MM_AXI + 0x30, 12),
};

#define CLK_MM_REG(o)						\
	CLK_REG_CLR(o + 0x10, o + 0x20, 0xffffffff),		\
	CLK_REG(o + 0x60, 0xff)					\

static const struct nexell_clk_reg cmu_mm_regs[] __initconst = {
	CLK_MM_REG(MM_AXI),
	CLK_REG(MM_AXI + 0x64, 0xff),
	CLK_MM_REG(MM_VIP_PADOUT0),
	CLK_MM_REG(MM_VIP_PADOUT1),
	CLK_MM_REG(MM_DPC_X2),
	CLK_REG(MM_DPC_X2 + 0x64, 0xff),
	CLK_MM_REG(MM_LVDS_VCLK),
	CLK_MM_REG(MM_CODA960_CORE),
};

static LIST_HEAD(nxp3220_cmu_mm_reg_cache_list);

static int nxp3220_cmu_mm_clk_suspend(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_mm_reg_cache_list, node)
		nexell_clk_save(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
	return 0;
}

static void nxp3220_cmu_mm_clk_resume(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_mm_reg_cache_list, node)
		nexell_clk_restore(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
}

static struct syscore_ops nxp3220_cmu_mm_syscore_ops = {
	.suspend = nxp3220_cmu_mm_clk_suspend,
	.resume = nxp3220_cmu_mm_clk_resume,
};

static void __init nxp3220_cmu_mm_init(struct device_node *np)
{
	void __iomem *reg;
	struct nexell_clk_data *ctx;

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	ctx = nexell_clk_init(reg, CLK_MM_NR);
	if (!ctx) {
		pr_err("%s: Failed to initialize clock data\n", __func__);
		return;
	}

	nexell_clk_register_div(ctx, mm_div_clks, ARRAY_SIZE(mm_div_clks));
	nxp3220_clk_register_gate(ctx, mm_gate_clks, ARRAY_SIZE(mm_gate_clks));

	nexell_clk_sleep_init(ctx->reg, &nxp3220_cmu_mm_syscore_ops,
			      &nxp3220_cmu_mm_reg_cache_list,
			      cmu_mm_regs, ARRAY_SIZE(cmu_mm_regs));

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: failed to add clock provider\n", __func__);

	/* Register reset controls */
	ctx->reset.ops = &nexell_clk_reset_ops;
	ctx->reset.nr_resets = ARRAY_SIZE(mm_resets);
	ctx->reset.of_node = np;

	ctx->reset_table = mm_resets;
	ctx->max_reset_id = CLK_RESET_MM_NR;

	if (reset_controller_register(&ctx->reset))
		pr_err("%s: failed to register clock reset controller\n",
		       __func__);
}
CLK_OF_DECLARE(nxp3220_cmu_mm, "nexell,nxp3220-cmu-mm",
	       nxp3220_cmu_mm_init);
