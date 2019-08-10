/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Nexell nxp3220 gate clock header
 * Copyright (c) 2018 Chanho Park <chanho61.park@samsung.com>
 */

#ifndef __CLK_NXP3220_H
#define __CLK_NXP3220_H

/**
 * struct clk_nxp3220_gate - gating clock for nxp3220
 *
 * @hw:		handle between common and hardware-specific interfaces
 * @reg:	register to enable clock
 * @clr_reg:	register to clear clock
 * @bit_idx:	single bit controlling gate
 * @flags:	hardware-specific flags
 * @lock:	register lock
 *
 * Clock which can gate its output.  Implements .enable & .disable
 *
 * Flags: TBD
 */

struct clk_nxp3220_gate {
	struct clk_hw hw;
	void __iomem	*reg;
	void __iomem	*clr_reg;
	u8		bit_idx;
	u8		flags;
	spinlock_t	*lock;
};

#define to_clk_nxp3220_gate(_hw) container_of(_hw, struct clk_nxp3220_gate, hw)

extern const struct clk_ops clk_nxp3220_gate_ops;

struct clk *clk_register_nxp3220_gate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *set_reg, void __iomem *clr_reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock);
struct clk_hw *clk_hw_register_nxp3220_gate(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		void __iomem *set_reg, void __iomem *clr_reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock);
void clk_unregister_nxp3220_gate(struct clk *clk);
void clk_hw_unregister_nxp3220_gate(struct clk_hw *hw);
int clk_nxp3220_gate_is_enabled(struct clk_hw *hw);

extern void __init
nxp3220_clk_register_gate(struct nexell_clk_data *ctx,
			  const struct nexell_gate_clock *list,
			  unsigned int clk_num);
extern void __init
nxp3220_clk_register_composite(struct nexell_clk_data *ctx,
			       const struct nexell_composite_clock *list,
			       unsigned int clk_num);

#endif
