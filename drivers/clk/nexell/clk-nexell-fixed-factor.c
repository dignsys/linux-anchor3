// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC Clock PLL driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
static struct clk *_of_fixed_factor_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name;
	u32 flags = 0;
	u32 div, mult;
	int ret;

	if (of_property_read_u32(node, "clock-div", &div)) {
		pr_err("%s Fixed factor clock <%s> must have a clock-div property\n",
			__func__, node->name);
		return ERR_PTR(-EIO);
	}

	if (of_property_read_u32(node, "clock-mult", &mult)) {
		pr_err("%s Fixed factor clock <%s> must have a clock-mult property\n",
			__func__, node->name);
		return ERR_PTR(-EIO);
	}

	of_property_read_u32(node, "clock-flags", &flags);

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);

	clk = clk_register_fixed_factor(NULL, clk_name, parent_name, flags,
					mult, div);
	if (IS_ERR(clk)) {
		/*
		 * If parent clock is not registered, registration would fail.
		 * Clear OF_POPULATED flag so that clock registration can be
		 * attempted again from probe function.
		 */
		of_node_clear_flag(node, OF_POPULATED);
		return clk;
	}

	ret = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (ret) {
		clk_unregister(clk);
		return ERR_PTR(ret);
	}

	return clk;
}

/**
 * of_fixed_factor_clk_setup() - Setup function for simple fixed factor clock
 */
void __init nexell_fixed_factor_clk_setup(struct device_node *node)
{
	_of_fixed_factor_clk_setup(node);
}
CLK_OF_DECLARE(fixed_factor_clk, "nexell,fixed-factor-clock",
		nexell_fixed_factor_clk_setup);
#endif
