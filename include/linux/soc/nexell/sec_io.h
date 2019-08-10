// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#ifndef __NEXELL_SECURE_IO__
#define __NEXELL_SECURE_IO__

#include <linux/io.h>

unsigned long sec_writel(void __iomem *reg, unsigned long val);
unsigned long sec_readl(void __iomem *reg);

#endif

