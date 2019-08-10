// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 SoC Clock CMU-USB driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/clock/nxp3220-clk.h>
#include "clk-nexell.h"
#include "clk-nxp3220.h"

#define USB_AHB			0x0200

#define DIV_USB(_id, cname, pname, o)	\
	__DIV(_id, NULL, cname, pname, o, 0, 8, 0, 0)

static const struct nexell_div_clock usb_div_clks[] __initconst = {
	DIV_USB(CLK_USB_DIV_AHB, "div_usb_ahb", "src_usb0_ahb",
		USB_AHB + 0x60),
};

#define GATE_USB(_id, cname, pname, o, b, f, gf)		\
	GATE(_id, cname, pname, o, o + 0x10, b, f, gf)

static const struct nexell_gate_clock usb_gate_clks[] __initconst = {
	GATE_USB(CLK_USB_AHB, "usb_ahb", "div_usb_ahb",
		 USB_AHB + 0x10, 0, CLK_IS_CRITICAL, 0),
	GATE_USB(CLK_USB_SYSREG_APB, "usb_sysreg_apb", "div_usb_ahb",
		 USB_AHB + 0x10, 2, 0, 0),
	GATE_USB(CLK_USB_USB20HOST, "usb20host", "div_usb_ahb",
		 USB_AHB + 0x10, 3, 0, 0),
	GATE_USB(CLK_USB_USB20OTG, "usb20otg", "div_usb_ahb",
		 USB_AHB + 0x10, 4, 0, 0),
};

#define CLK_USB_REG(o)						\
	CLK_REG_CLR(o + 0x10, o + 0x20, 0xffffffff),		\
	CLK_REG(o + 0x60, 0xff)					\

static const struct nexell_clk_reg cmu_usb_regs[] __initconst = {
	CLK_USB_REG(USB_AHB),
};

static LIST_HEAD(nxp3220_cmu_usb_reg_cache_list);

static int nxp3220_cmu_usb_clk_suspend(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_usb_reg_cache_list, node)
		nexell_clk_save(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
	return 0;
}

static void nxp3220_cmu_usb_clk_resume(void)
{
	struct nexell_clk_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &nxp3220_cmu_usb_reg_cache_list, node)
		nexell_clk_restore(reg_cache->reg_base, reg_cache->regs,
				reg_cache->num_regs);
}

static struct syscore_ops nxp3220_cmu_usb_syscore_ops = {
	.suspend = nxp3220_cmu_usb_clk_suspend,
	.resume = nxp3220_cmu_usb_clk_resume,
};

static void __init nxp3220_cmu_usb_init(struct device_node *np)
{
	void __iomem *reg;
	struct nexell_clk_data *ctx;

	reg = of_iomap(np, 0);
	if (!reg) {
		pr_err("%s: Failed to get base address\n", __func__);
		return;
	}

	ctx = nexell_clk_init(reg, CLK_USB_NR);
	if (!ctx) {
		pr_err("%s: Failed to initialize clock data\n", __func__);
		return;
	}

	nexell_clk_register_div(ctx, usb_div_clks, ARRAY_SIZE(usb_div_clks));
	nxp3220_clk_register_gate(ctx, usb_gate_clks,
				  ARRAY_SIZE(usb_gate_clks));

	nexell_clk_sleep_init(ctx->reg, &nxp3220_cmu_usb_syscore_ops,
			      &nxp3220_cmu_usb_reg_cache_list,
			      cmu_usb_regs, ARRAY_SIZE(cmu_usb_regs));

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: failed to add clock provider\n", __func__);
}
CLK_OF_DECLARE(nxp3220_cmu_usb, "nexell,nxp3220-cmu-usb",
	       nxp3220_cmu_usb_init);
