// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC secure I/O driver
 *
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/arm-smccc.h>
#include <linux/soc/nexell/sec_io.h>

#define NEXELL_SMC_FN_BASE	0x82000000
#define NEXELL_SMC_FN(n)	(NEXELL_SMC_FN_BASE + (n))
#define NEXELL_SMC_FN_WRITE	NEXELL_SMC_FN(0x0)
#define NEXELL_SMC_FN_READ	NEXELL_SMC_FN(0x1)

static unsigned long __invoke_psci_fn_smc(unsigned long function_id,
		unsigned long arg0, unsigned long arg1,
		unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return res.a0;
}

unsigned long sec_writel(void __iomem *reg, unsigned long val)
{
	return __invoke_psci_fn_smc(NEXELL_SMC_FN_WRITE, (u32)reg, val, 0);
}

unsigned long sec_readl(void __iomem *reg)
{
	return __invoke_psci_fn_smc(NEXELL_SMC_FN_READ, (u32)reg, 0, 0);
}
