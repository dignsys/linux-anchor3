// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell nxp3220 gate clock driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>

#include "clk-nexell.h"
#include "clk-nxp3220.h"

static void clk_nxp3220_gate_endisable(struct clk_hw *hw, int enable)
{
	struct clk_nxp3220_gate *gate = to_clk_nxp3220_gate(hw);
	unsigned long uninitialized_var(flags);

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);
	else
		__acquire(gate->lock);

	if (enable)
		clk_writel(BIT(gate->bit_idx), gate->reg);
	else
		clk_writel(BIT(gate->bit_idx), gate->clr_reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
	else
		__release(gate->lock);
}

static int clk_nxp3220_gate_enable(struct clk_hw *hw)
{
	clk_nxp3220_gate_endisable(hw, 1);

	return 0;
}

static void clk_nxp3220_gate_disable(struct clk_hw *hw)
{
	clk_nxp3220_gate_endisable(hw, 0);
}

int clk_nxp3220_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct clk_nxp3220_gate *gate = to_clk_nxp3220_gate(hw);

	reg = clk_readl(gate->reg);

	reg &= BIT(gate->bit_idx);

	return reg ? 1 : 0;
}

const struct clk_ops clk_nxp3220_gate_ops = {
	.enable = clk_nxp3220_gate_enable,
	.disable = clk_nxp3220_gate_disable,
	.is_enabled = clk_nxp3220_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(clk_nxp3220_gate_ops);

/**
 * clk_hw_register_nxp3220_gate - register a gate clock with the clock
 * framework for nxp3220
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @set_reg: register address to enable this clock
 * @clr_reg: register address to clear this clock
 * @bit_idx: which bit in the register controls gating of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk_hw *clk_hw_register_nxp3220_gate(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *set_reg, void __iomem *clr_reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct clk_nxp3220_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_nxp3220_gate_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct clk_gate assignments */
	gate->reg = set_reg;
	gate->clr_reg = clr_reg;
	gate->bit_idx = bit_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}
EXPORT_SYMBOL_GPL(clk_hw_register_nxp3220_gate);

struct clk *clk_register_nxp3220_gate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *set_reg, void __iomem *clr_reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct clk_hw *hw;

	hw = clk_hw_register_nxp3220_gate(dev, name, parent_name, flags,
					  set_reg, clr_reg, bit_idx,
					  clk_gate_flags, lock);
	if (IS_ERR(hw))
		return ERR_CAST(hw);
	return hw->clk;
}
EXPORT_SYMBOL_GPL(clk_register_nxp3220_gate);

void clk_unregister_nxp3220_gate(struct clk *clk)
{
	struct clk_nxp3220_gate *gate;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	gate = to_clk_nxp3220_gate(hw);

	clk_unregister(clk);
	kfree(gate);
}
EXPORT_SYMBOL_GPL(clk_unregister_nxp3220_gate);

void clk_hw_unregister_nxp3220_gate(struct clk_hw *hw)
{
	struct clk_nxp3220_gate *gate;

	gate = to_clk_nxp3220_gate(hw);

	clk_hw_unregister(hw);
	kfree(gate);
}
EXPORT_SYMBOL_GPL(clk_hw_unregister_nxp3220_gate);

void __init
nxp3220_clk_register_gate(struct nexell_clk_data *ctx,
			  const struct nexell_gate_clock *list,
			  unsigned int clk_num)
{
	struct clk *clk;
	const struct nexell_gate_clock *g;
	int i;

	for (i = 0; i < clk_num; i++) {
		g = &list[i];
		clk = clk_register_nxp3220_gate(NULL, g->name, g->parent_name,
						g->flags, ctx->reg + g->offset,
						ctx->reg + g->clr_offset,
						g->bit_idx, g->gate_flags,
						&ctx->lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       g->name);
			continue;
		}

		nexell_clk_add_lookup(&ctx->clk_data, clk, g->id);
	}
}

static struct clk * __init
nxp3220_clk_register_composite_one(struct nexell_clk_data *ctx,
				   const struct nexell_composite_clock *comp)
{
	struct clk_mux *mux = NULL;
	struct clk_divider *div = NULL;
	struct clk_nxp3220_gate *gate = NULL;
	const struct clk_ops *mux_ops = NULL;
	const struct clk_ops *div_ops = NULL;
	const struct clk_ops *gate_ops = NULL;

	if (comp->has_mux) {
		mux = kzalloc(sizeof(struct clk_mux), GFP_KERNEL);
		if (!mux)
			return ERR_PTR(-ENOMEM);

		mux_ops = &clk_mux_ops;
		mux->reg = ctx->reg + comp->mux_offset;
		mux->shift = comp->mux_shift;
		mux->mask = BIT(comp->mux_width) - 1;
		mux->table = comp->mux_table;
		mux->flags = comp->mux_flags;
		mux->lock = &ctx->lock;
	}

	if (comp->has_div) {
		div = kzalloc(sizeof(struct clk_divider), GFP_KERNEL);
		if (!div)
			goto err_free_mux;

		div_ops = &clk_divider_ops;
		div->reg = ctx->reg + comp->div_offset;
		div->width = comp->div_width;
		div->shift = comp->div_shift;
		div->flags = comp->div_flags;
		div->lock = &ctx->lock;
	}

	if (comp->has_gate) {
		gate = kzalloc(sizeof(struct clk_nxp3220_gate), GFP_KERNEL);
		if (!gate)
			goto err_free_div;

		gate_ops = &clk_nxp3220_gate_ops;
		gate->reg = ctx->reg + comp->gate_offset;
		gate->clr_reg = ctx->reg + comp->gate_clr_offset;
		gate->bit_idx = comp->gate_bit_idx;
		gate->flags = comp->gate_flags;
		gate->lock = &ctx->lock;
	}

	return clk_register_composite(NULL, comp->name, comp->parent_names,
				      comp->num_parents, &mux->hw, mux_ops,
				      &div->hw, div_ops, &gate->hw, gate_ops,
				      comp->flags);

err_free_div:
	kfree(div);

err_free_mux:
	kfree(mux);

	return ERR_PTR(-ENOMEM);
}

void __init
nxp3220_clk_register_composite(struct nexell_clk_data *ctx,
			       const struct nexell_composite_clock *list,
			       unsigned int clk_num)
{
	struct clk *clk;
	int i;

	for (i = 0; i < clk_num; i++) {
		clk = nxp3220_clk_register_composite_one(ctx, &list[i]);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list[i].name);
			continue;
		}

		nexell_clk_add_lookup(&ctx->clk_data, clk, list[i].id);
	}
}

