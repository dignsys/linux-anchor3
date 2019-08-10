// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC Clock PLL driver
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/syscore_ops.h>
#include <linux/clk.h>

#define REF_CLK 24000000UL

#define PLLCTRL			0x0
#define PLLCTRL_MUXSEL_SHIFT	3
#define PLLCTRL_MUXSEL_MASK	BIT_MASK(PLLCTRL_MUXSEL_SHIFT)
#define MUXSEL_OSCCLK		0
#define MUXSEL_PLLFOUT		1
#define PLLCTRL_MUXSEL(x)	(x << PLLCTRL_MUXSEL_SHIFT)
#define PLLCTRL_DIRTYFLAG_SHIFT	1
#define PLLCTRL_DIRTYFLAG_MASK	BIT_MASK(PLLCTRL_DIRTYFLAG_SHIFT)
#define PLLCTRL_RUN_CHANGE_SHIFT	0
#define PLLCTRL_RUN_CHANGE_MASK	BIT_MASK(PLLCTRL_RUN_CHANGE_SHIFT)
#define PLLCTRL_RUN_PLL_UPDATE	15
#define PLLCTRL_RUN_PLL_UPDATE_MASK	BIT_MASK(PLLCTRL_RUN_PLL_UPDATE)

#define PLLDBG0			0x4

#define PLLCFG0			0x20
#define PLLCFG0_RESET_SHIFT	0
#define PLLCFG0_RESET_MASK	BIT_MASK(PLLCFG0_RESET_SHIFT)

#define PLLCFG1			0x30
#define PLLCFG1_M_SHIFT		16
#define PLLCFG1_M_MASK		GENMASK(31, PLLCFG1_M_SHIFT)
#define PLLCFG1_M(x)		(x << PLLCFG1_M_SHIFT)
#define PLLCFG1_P_SHIFT		0
#define PLLCFG1_P_MASK		GENMASK(15, PLLCFG1_P_SHIFT)

#define PLLCFG2			0x40
#define PLLCFG2_K_SHIFT		16
#define PLLCFG2_K_MASK		GENMASK(31, PLLCFG2_K_SHIFT)
#define PLLCFG2_K(x)		(x << PLLCFG2_K_SHIFT)
#define PLLCFG2_S_SHIFT		0
#define PLLCFG2_S_MASK		GENMASK(15, PLLCFG2_S_SHIFT)

struct pll_pms {
	unsigned long rate;
	unsigned int p;
	unsigned int m;
	unsigned int s;
	int k;
};

enum pll_type {
	PLL2555,
	PLL2651,
};

struct clk_pll_data {
	enum pll_type type;
	struct pll_pms *pms_table;
	unsigned int pms_count;
	unsigned long lock_cycle;
};

struct clk_pll {
	struct clk_hw hw;
	const char *name;
	struct regmap *regmap;
	struct pll_pms pms;
	struct clk_pll_data *pll_data;
	spinlock_t lock;
	struct list_head node;

	unsigned long refclk_rate;
};

static LIST_HEAD(pll_list);

#define to_clk_pll(hw) container_of(hw, struct clk_pll, hw)

static struct pll_pms pll2555_pms_table[] = {
	{ .rate = 1200000000U, .p = 4, .m = 400, .s = 1, .k = 0 },
	{ .rate = 1100000000U, .p = 3, .m = 275, .s = 1, .k = 0 },
	{ .rate = 1000000000U, .p = 3, .m = 250, .s = 1, .k = 0 },
	{ .rate =  900000000U, .p = 4, .m = 300, .s = 1, .k = 0 },
	{ .rate =  800000000U, .p = 3, .m = 200, .s = 1, .k = 0 },
	{ .rate =  700000000U, .p = 3, .m = 175, .s = 1, .k = 0 },
	{ .rate =  600000000U, .p = 4, .m = 400, .s = 2, .k = 0 },
	{ .rate =  500000000U, .p = 3, .m = 250, .s = 2, .k = 0 },
	{ .rate =  400000000U, .p = 3, .m = 200, .s = 2, .k = 0 },
	{ .rate =  300000000U, .p = 4, .m = 400, .s = 3, .k = 0 },
	{ .rate =  200000000U, .p = 3, .m = 200, .s = 3, .k = 0 },
	{ .rate =  100000000U, .p = 3, .m = 200, .s = 4, .k = 0 },
};

static struct pll_pms pll2651_pms_table[] = {
	{ .rate = 2500000000U, .p = 6, .m = 625, .s = 0, .k = 0 },
	{ .rate = 2200000000U, .p = 3, .m = 275, .s = 0, .k = 0 },
	{ .rate = 2000000000U, .p = 3, .m = 250, .s = 0, .k = 0 },
	{ .rate = 1800000000U, .p = 2, .m = 150, .s = 0, .k = 0 },
	{ .rate = 1600000000U, .p = 3, .m = 200, .s = 0, .k = 0 },
	{ .rate = 1400000000U, .p = 3, .m = 175, .s = 0, .k = 0 },
	{ .rate = 1354752000U, .p = 2, .m = 113, .s = 0, .k = -6816 },
	{ .rate = 1277952000U, .p = 2, .m = 106, .s = 0, .k = 32506 },
	{ .rate = 1264434000U, .p = 3, .m = 158, .s = 0, .k = 3555 },
	{ .rate = 1228800000U, .p = 3, .m = 154, .s = 0, .k = -26214 },
	{ .rate = 1200000000U, .p = 2, .m = 100, .s = 0, .k = 0 },
	{ .rate = 1179648000U, .p = 2, .m =  98, .s = 0, .k = 0 },
	{ .rate = 1000000000U, .p = 3, .m = 250, .s = 1, .k = 0 },
	{ .rate =  800000000U, .p = 3, .m = 200, .s = 1, .k = 0 },
	{ .rate =  600000000U, .p = 2, .m = 100, .s = 1, .k = 0 },
	{ .rate =  300000000U, .p = 2, .m = 100, .s = 2, .k = 0 },
	{ .rate =  200000000U, .p = 3, .m = 200, .s = 3, .k = 0 },
};

static struct clk_pll_data pll_data_table[] = {
	[PLL2555] = {
		.type = PLL2555,
		.pms_table = pll2555_pms_table,
		.pms_count = ARRAY_SIZE(pll2555_pms_table),
		.lock_cycle = 200,
	}, [PLL2651] = {
		.type = PLL2651,
		.pms_table = pll2651_pms_table,
		.pms_count = ARRAY_SIZE(pll2651_pms_table),
		.lock_cycle = 3000,
	},
};

static inline bool is_match_pms(struct pll_pms p1, struct pll_pms p2)
{
	if (p1.p == p2.p && p1.m == p2.m && p1.s == p2.s && p1.k == p2.k)
		return true;

	return false;
}

static void clk_pll_set_oscmux(struct clk_pll *pll, unsigned int muxsel)
{
	struct regmap *regmap = pll->regmap;

	regmap_update_bits(regmap, PLLCTRL, PLLCTRL_MUXSEL_MASK,
			   PLLCTRL_MUXSEL(muxsel));
}

static inline void clk_pll_wait_lock(struct clk_pll *pll, unsigned long cycle)
{
	u32 st_count, cnt;
	struct regmap *regmap = pll->regmap;

	regmap_read(regmap, PLLDBG0, &st_count);

	st_count += cycle;

	do {
		regmap_read(regmap, PLLDBG0, &cnt);
	} while (cnt < st_count);
}

static long pll_round_rate(struct clk_pll *pll, unsigned long rate,
			   unsigned long parent_rate, struct pll_pms *pms)
{
	struct clk_pll_data *pll_data = pll->pll_data;
	int i;

	for (i = 0; i < pll_data->pms_count; i++) {
		if (rate >= pll_data->pms_table[i].rate) {
			if (pms)
				*pms = pll_data->pms_table[i];
			return rate;
		}
	}

	return -ERANGE;
}

static int __pll_get_pms(struct clk_pll *pll, struct pll_pms *pms)
{
	struct regmap *regmap = pll->regmap;
	u32 cfg1, cfg2;
	int ret;

	if (pms == NULL)
		return -EINVAL;

	ret = regmap_read(regmap, PLLCFG1, &cfg1);
	if (ret)
		return ret;

	ret = regmap_read(regmap, PLLCFG2, &cfg2);
	if (ret)
		return ret;

	pms->p = (cfg1 & PLLCFG1_P_MASK) >> PLLCFG1_P_SHIFT;
	pms->m = (cfg1 & PLLCFG1_M_MASK) >> PLLCFG1_M_SHIFT;
	pms->s = (cfg2 & PLLCFG2_S_MASK) >> PLLCFG2_S_SHIFT;
	pms->k = (s16)((cfg2 & PLLCFG2_K_MASK) >> PLLCFG2_K_SHIFT);

	return 0;
}

static unsigned long nexell_clk_pll_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct clk_pll_data *pll_data = pll->pll_data;
	struct pll_pms pms;
	u64 fout = parent_rate;
	int ret;

	if (fout > REF_CLK)
		fout = REF_CLK;

	ret = __pll_get_pms(pll, &pms);
	if (ret)
		return 0;

	if (pll_data->type == PLL2555)
		fout *= pms.m;
	else if (pll_data->type == PLL2651)
		fout *= (pms.m << 16) + pms.k;
	else {
		fout = 0;
		goto out;
	}

	do_div(fout, (pms.p << pms.s));

	if (pll_data->type == PLL2651)
		fout >>= 16;

	pll->pms = pms;

out:
	return fout;
}

static long nexell_clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);

	return pll_round_rate(pll, rate, *parent_rate, NULL);
}

static void __pll_set_rate_locked(struct clk_pll *pll, struct pll_pms *pms)
{
	struct regmap *regmap = pll->regmap;

	clk_pll_set_oscmux(pll, MUXSEL_OSCCLK);

	regmap_update_bits(regmap, PLLCFG1, (PLLCFG1_P_MASK | PLLCFG1_M_MASK),
			   PLLCFG1_M(pms->m) | pms->p);
	regmap_update_bits(regmap, PLLCFG2, PLLCFG2_S_MASK | PLLCFG2_K_MASK,
			   PLLCFG2_K(pms->k) | pms->s);

	regmap_update_bits(regmap, PLLCFG0, PLLCFG0_RESET_MASK, 0);

	regmap_update_bits(regmap, PLLCTRL, PLLCTRL_DIRTYFLAG_MASK,
			   BIT(PLLCTRL_DIRTYFLAG_SHIFT));

	regmap_update_bits(regmap, PLLCTRL, PLLCTRL_RUN_PLL_UPDATE_MASK,
			   BIT(PLLCTRL_RUN_PLL_UPDATE));

	regmap_update_bits(regmap, PLLCFG0, PLLCFG0_RESET_MASK,
			   BIT(PLLCFG0_RESET_SHIFT));

	regmap_update_bits(regmap, PLLCTRL, PLLCTRL_DIRTYFLAG_MASK,
			   BIT(PLLCTRL_DIRTYFLAG_SHIFT));
	regmap_update_bits(regmap, PLLCTRL, PLLCTRL_RUN_PLL_UPDATE_MASK,
			   BIT(PLLCTRL_RUN_PLL_UPDATE));

	/* Wait (p + 1) * lock_cycle */
	clk_pll_wait_lock(pll, (pms->p + 1) * pll->pll_data->lock_cycle);

	clk_pll_set_oscmux(pll, MUXSEL_PLLFOUT);
}

static int nexell_clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct pll_pms pms;
	unsigned long flags;
	int ret;

	pr_debug("%s: set rate %ld, parent %ld\n",
			pll->name, rate, parent_rate);

	ret = pll_round_rate(pll, rate, parent_rate, &pms);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pll->lock, flags);
	__pll_set_rate_locked(pll, &pms);
	pll->pms = pms;
	spin_unlock_irqrestore(&pll->lock, flags);

	return 0;
}

static const struct clk_ops pll_ops = {
	.recalc_rate = nexell_clk_pll_recalc_rate,
	.round_rate = nexell_clk_pll_round_rate,
	.set_rate = nexell_clk_pll_set_rate,
};

static int nexell_clk_pll_suspend(void)
{
	return 0;
}

static void nexell_clk_pll_resume(void)
{
	struct clk_pll *pll;
	int ret;

	list_for_each_entry(pll, &pll_list, node) {
		struct pll_pms pms;

		ret = __pll_get_pms(pll, &pms);
		if (ret)
			continue;

		if (!is_match_pms(pll->pms, pms))
			__pll_set_rate_locked(pll, &pll->pms);
	}
}

static struct syscore_ops nexell_clk_pll_syscore_ops = {
	.suspend = nexell_clk_pll_suspend,
	.resume = nexell_clk_pll_resume,
};

static void __init nexell_clk_register_pll(struct device_node *np,
				    struct clk_pll_data *pll_data)
{
	struct regmap *regmap;
	struct clk_init_data init;
	struct clk_pll *pll;
	const char *parent_name;
	const char *name = np->name;

	parent_name = of_clk_get_parent_name(np, 0);
	of_property_read_string(np, "clock-output-names", &name);

	regmap = syscon_node_to_regmap(np);
	if (IS_ERR(regmap))
		return;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return;

	pll->refclk_rate = clk_get_rate(__clk_lookup(parent_name));
	pll->pll_data = pll_data;
	pll->regmap = regmap;
	pll->name = name;

	init.name = name;
	init.ops = &pll_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_GATE;

	pll->hw.init = &init;

	spin_lock_init(&pll->lock);

	if (clk_hw_register(NULL, &pll->hw))
		goto out_free;

	if (of_clk_add_hw_provider(np, of_clk_hw_simple_get, &pll->hw))
		goto out_unregister;

	if (list_empty(&pll_list))
		register_syscore_ops(&nexell_clk_pll_syscore_ops);

	list_add_tail(&pll->node, &pll_list);

	return;

out_unregister:
	clk_hw_unregister(&pll->hw);

out_free:
	kfree(pll);
}

static void __init nexell_clk_pll2651_setup(struct device_node *np)
{
	nexell_clk_register_pll(np, &pll_data_table[PLL2651]);
}
CLK_OF_DECLARE(nexell_pll2651, "nexell,pll2651", nexell_clk_pll2651_setup);

static void __init nexell_clk_pll2555_setup(struct device_node *np)
{
	nexell_clk_register_pll(np, &pll_data_table[PLL2555]);
}
CLK_OF_DECLARE(nexell_pll2555, "nexell,pll2555", nexell_clk_pll2555_setup);
