// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/err.h>

#include "pinctrl-nexell.h"
#include "pinctrl-nxp3220.h"

static struct {
	void __iomem *gpio_regs;
	struct nx_gpio_reg_set gpio_save;
} gpio_modules[NR_GPIO_MODULE];

static void __iomem *alive_regs;
static u32 alive_detect_save;

/*
 * gpio functions
 */

static void nx_gpio_setbit(void __iomem *addr, u32 bit, bool enable)
{
	u32 value = readl(addr);

	value &= ~(1ul << bit);
	value |= (u32)enable << bit;

	writel(value, addr);
}

static bool nx_gpio_getbit(void __iomem *addr, u32 bit)
{
	return (bool)((readl(addr) >> bit) & 1UL);
}

static void nx_gpio_setbit2(void __iomem *addr, u32 bit, u32 bit_value)
{
	u32 value = readl(addr);

	bit_value &= 0x3;

	value = value & ~(3ul << (bit * 2));
	value = value | (bit_value << (bit * 2));

	writel(value, addr);
}

static u32 nx_gpio_getbit2(void __iomem *addr, u32 bit)
{
	return (readl(addr) >> (bit * 2)) & 3UL;
}

static u32 nx_alive_getbit(void __iomem *addr, u32 bit)
{
	return (readl(addr) >> bit) & 1UL;
}

static void nx_alive_setbit(void __iomem *addr, u32 bit, bool enable)
{
	u32 value = readl(addr);

	value &= ~(1ul << bit);
	value |= (u32)enable << bit;

	writel(value, addr);
}

static void nx_alive_setbit2(void __iomem *addr, u32 bit, u32 bit_value)
{
	u32 value = readl(addr);

	value = (u32)(value & ~(3ul << (bit * 2)));
	value = (u32)(value | (bit_value << (bit * 2)));

	writel(value, addr);
}

static u32 nx_alive_getbit2(void __iomem *addr, u32 bit)
{
	return (readl(addr) >> (bit * 2)) & 3UL;
}

static bool nx_gpio_open_module(u32 idx)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	writel(0xFFFFFFFF, base + GPIO_SLEW_DISABLE_DEFAULT);
	writel(0xFFFFFFFF, base + GPIO_DRV1_DISABLE_DEFAULT);
	writel(0xFFFFFFFF, base + GPIO_DRV0_DISABLE_DEFAULT);
	writel(0xFFFFFFFF, base + GPIO_PULLSEL_DISABLE_DEFAULT);
	writel(0xFFFFFFFF, base + GPIO_PULLENB_DISABLE_DEFAULT);
	writel(0xFFFFFFFF, base + GPIO_INPUTENB_DISABLE_DEFAULT);

	return true;
}

static void nx_gpio_set_output_enable(u32 idx, u32 bitnum, bool enable)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	nx_gpio_setbit(base + GPIO_OUT_ENB, bitnum, enable);
}

static bool nx_gpio_get_output_enable(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	return nx_gpio_getbit(base + GPIO_OUT_ENB, bitnum);
}

static void nx_gpio_set_output_value(u32 idx, u32 bitnum, bool value)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	nx_gpio_setbit(base + GPIO_OUT, bitnum, value);
}

static bool nx_gpio_get_output_value(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	return nx_gpio_getbit(base + GPIO_OUT, bitnum);
}

static bool nx_gpio_get_input_value(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	return nx_gpio_getbit(base + GPIO_PAD, bitnum);
}

static void nx_gpio_set_pad_function(u32 idx, u32 bitnum, int fn)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	nx_gpio_setbit2(base + GPIO_ALTFN + (bitnum / 16) * 4,
			bitnum % 16, (u32)fn);
#ifdef CONFIG_ARCH_NXP3220
	fn = (fn >> 2) & 0x1;
	nx_gpio_setbit(base + GPIO_ALTFNEX, bitnum, fn);
#endif
}

static int nx_gpio_get_pad_function(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;
#ifdef CONFIG_ARCH_NXP3220
	int fn;

	fn = nx_gpio_getbit(base + GPIO_ALTFNEX, bitnum) << 2;
	fn |= nx_gpio_getbit2(base + GPIO_ALTFN + (bitnum / 16) * 4,
			bitnum % 16);

	return fn;
#else
	return nx_gpio_getbit2(base + GPIO_ALTFN + (bitnum / 16) * 4,
			bitnum % 16);
#endif
}

static void nx_gpio_set_drive_strength(u32 idx, u32 bitnum, int drv)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	nx_gpio_setbit(base + GPIO_DRV1, bitnum, ((u32)drv >> 0) & 0x1);
	nx_gpio_setbit(base + GPIO_DRV0, bitnum, ((u32)drv >> 1) & 0x1);
}

static int nx_gpio_get_drive_strength(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;
	u32 value;

	value = nx_gpio_getbit(base + GPIO_DRV0, bitnum) << 1;
	value |= nx_gpio_getbit(base + GPIO_DRV1, bitnum) << 0;

	return (int)value;
}

static void nx_gpio_set_pull_enable(u32 idx, u32 bitnum, int pullsel)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;

	if (pullsel == nx_gpio_pull_down || pullsel == nx_gpio_pull_up) {
		nx_gpio_setbit(base + GPIO_PULLSEL, bitnum, (bool)pullsel);
		nx_gpio_setbit(base + GPIO_PULLENB, bitnum, true);
	} else {
		nx_gpio_setbit(base + GPIO_PULLENB, bitnum, false);
	}
}

static int nx_gpio_get_pull_enable(u32 idx, u32 bitnum)
{
	void __iomem *base = gpio_modules[idx].gpio_regs;
	bool enable;

	enable = nx_gpio_getbit(base + GPIO_PULLENB, bitnum);

	if (enable == true)
		return (int)nx_gpio_getbit(base + GPIO_PULLSEL, bitnum);
	else
		return nx_gpio_pull_off;
}

/*
 * alive alternative functions
 */

static void nx_alive_set_pad_function(void __iomem *base, u32 pin, int fn)
{
	if (fn > nx_gpio_padfunc_1) {
		pr_debug("wrong pad function to alive pin %d\n", pin);
		return;
	}
	nx_alive_setbit2(base + ALIVE_ALTFN_SEL_LOW, pin, fn);

	pr_debug("%s base %p pin %u fn %u\n", __func__, base, pin, fn);
}

static int nx_alive_get_pad_function(void __iomem *base, u32 pin)
{
	return nx_alive_getbit2(base + ALIVE_ALTFN_SEL_LOW, pin);
}

/*
 * alive pad configurations
 */

static void nx_alive_set_drive_strength(void __iomem *base, u32 pin, int drv)
{
	u32 bit = 1UL << pin;
	u32 drv0_reg = drv & 0x2 ? ALIVE_DRV0_SET : ALIVE_DRV0_RST;
	u32 drv1_reg = drv & 0x1 ? ALIVE_DRV1_SET : ALIVE_DRV1_RST;

	writel(bit, base + drv0_reg);
	writel(bit, base + drv1_reg);

	pr_debug("%s base %p pin %u drv %d\n", __func__, base, pin, drv);
}

static int nx_alive_get_drive_strength(void __iomem *base, u32 pin)
{
	u32 value;

	value = nx_alive_getbit(base + ALIVE_DRV0_READ, pin) << 1;
	value |= nx_alive_getbit(base + ALIVE_DRV1_READ, pin) << 0;

	return value;
}

static void nx_alive_set_pull_mode(void __iomem *base, u32 pin, u32 mode)
{
	u32 bit = 1UL << pin;

	if (mode == nx_gpio_pull_off) {
		writel(bit, base + ALIVE_PULL_ENB_RST);
		writel(bit, base + ALIVE_PULL_SEL_RST);
	} else {
		writel(bit, base + ALIVE_PULL_ENB_SET);
		if (mode == nx_gpio_pull_down)
			writel(bit, base + ALIVE_PULL_SEL_RST);
		else
			writel(bit, base + ALIVE_PULL_SEL_SET);
	}

	pr_debug("%s base %p pin %u mode %u\n", __func__, base, pin, mode);
}

static int nx_alive_get_pull_mode(void __iomem *base, u32 pin)
{
	u32 enable = nx_alive_getbit(base + ALIVE_PULL_ENB_READ, pin);

	if (enable)
		return nx_alive_getbit(base + ALIVE_PULL_SEL_READ, pin);
	else
		return nx_gpio_pull_off;
}

/*
 * alive output setting functions
 */

static void nx_alive_set_output_enable(void __iomem *base, u32 pin, bool enable)
{
	u32 bit = 1UL << pin;

	if (enable)
		writel(bit, base + ALIVE_OUT_ENB_SET);
	else
		writel(bit, base + ALIVE_OUT_ENB_RST);
}

static int nx_alive_get_output_enable(void __iomem *base, u32 pin)
{
	return nx_alive_getbit(base + ALIVE_OUT_ENB_READ, pin);
}

static void nx_alive_set_output_value(void __iomem *base, u32 pin, bool value)
{
	u32 bit = 1UL << pin;

	if (value)
		writel(bit, base + ALIVE_PAD_OUT_SET);
	else
		writel(bit, base + ALIVE_PAD_OUT_RST);

}

static bool nx_alive_get_input_value(void __iomem *base, u32 pin)
{
	return nx_alive_getbit(base + ALIVE_PAD_IN, pin);
}

static u32 nx_alive_get_wakeup_status(void __iomem *base)
{
	return readl(base + ALIVE_SLEEP_WAKEUP_STATUS);
}

static void nx_alive_clear_wakeup_status(void __iomem *base)
{
	writel(1, base + ALIVE_CLEAR_WAKEUP_STATUS);
}

static void nx_alive_set_write_enable(void __iomem *base, bool enable)
{
	writel(enable, base + ALIVE_PWRGATE);
}

/*
 * GPIO Operation.
 */

const unsigned char (*gpio_fn_no)[GPIO_NUM_PER_BANK] = NULL;

const unsigned char nxp3220_pio_fn_no[][GPIO_NUM_PER_BANK] = {
	ALT_NO_GPIO_A, ALT_NO_GPIO_B, ALT_NO_GPIO_C,
	ALT_NO_GPIO_D, ALT_NO_GPIO_E, ALT_NO_ALIVE,
};

/*----------------------------------------------------------------------------*/
static spinlock_t lock[NR_GPIO_MODULE + 1]; /* A, B, C, D, E, AliveGPIO */
static unsigned long lock_flags[NR_GPIO_MODULE + 1];

static u32 alive_wake_mask = 0xffffffff;

u32 get_wake_mask(void)
{
	return alive_wake_mask;
}

#define IO_LOCK_INIT(x) spin_lock_init(&lock[x])
#define IO_LOCK(x) spin_lock_irqsave(&lock[x], lock_flags[x])
#define IO_UNLOCK(x) spin_unlock_irqrestore(&lock[x], lock_flags[x])

void nx_soc_gpio_set_io_func(unsigned int io, unsigned int func)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		nx_gpio_set_pad_function(grp, bit, func);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		nx_alive_set_pad_function(alive_regs, bit, func);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};
}

int nx_soc_gpio_get_altnum(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	return gpio_fn_no[grp][bit];
}

unsigned int nx_soc_gpio_get_io_func(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	unsigned int fn = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		fn = nx_gpio_get_pad_function(grp, bit);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		fn = nx_alive_get_pad_function(alive_regs, bit);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return fn;
}

void nx_soc_gpio_set_io_dir(unsigned int io, int out)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		nx_gpio_set_output_enable(grp, bit, out ? true : false);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		nx_alive_set_output_enable(alive_regs, bit, out ? true : false);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};
}

int nx_soc_gpio_get_io_dir(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	int dir = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		dir = nx_gpio_get_output_enable(grp, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		dir = nx_alive_get_output_enable(alive_regs, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return dir;
}

void nx_soc_gpio_set_io_pull(unsigned int io, int val)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	pr_debug("%s (%d.%02d) sel:%d\n", __func__, grp, bit, val);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		nx_gpio_set_pull_enable(grp, bit, val);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		nx_alive_set_pull_mode(alive_regs, bit, val);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};
}

int nx_soc_gpio_get_io_pull(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	int up = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		up = nx_gpio_get_pull_enable(grp, bit);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		up = nx_alive_get_pull_mode(alive_regs, bit);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return up;
}

void nx_soc_gpio_set_io_drv(int io, int drv)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	pr_debug("%s (%d.%02d) drv:%d\n", __func__, grp, bit, drv);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		nx_gpio_set_drive_strength(grp, bit, drv);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		nx_alive_set_drive_strength(alive_regs, bit, drv);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};
}

int nx_soc_gpio_get_io_drv(int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	int drv = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		drv = nx_gpio_get_drive_strength(grp, bit);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		drv = nx_alive_get_drive_strength(alive_regs, bit);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return drv;
}

void nx_soc_gpio_set_out_value(unsigned int io, int high)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);

	pr_debug("%s (%d.%02d) %s\n",
		 __func__, grp, bit, high ? "high" : "low");

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		nx_gpio_set_output_value(grp, bit, high ? true : false);
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		nx_alive_set_output_value(alive_regs, bit, high ? true : false);
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};
}

static int nx_soc_gpio_get_in_value(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	int val = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		val = nx_gpio_get_input_value(grp, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		val = nx_alive_get_input_value(alive_regs, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return val;
}

static int nx_soc_gpio_get_out_value(unsigned int io)
{
	unsigned int grp = PAD_GET_GROUP(io);
	unsigned int bit = PAD_GET_BITNO(io);
	int val = -1;

	pr_debug("%s (%d.%02d)\n", __func__, grp, bit);

	switch (io & ~(32 - 1)) {
	case PAD_GPIO_A:
	case PAD_GPIO_B:
	case PAD_GPIO_C:
	case PAD_GPIO_D:
	case PAD_GPIO_E:
		IO_LOCK(grp);
		val = nx_gpio_get_output_value(grp, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	case PAD_GPIO_ALV:
		IO_LOCK(grp);
		val = nx_alive_get_input_value(alive_regs, bit) ? 1 : 0;
		IO_UNLOCK(grp);
		break;
	default:
		pr_err("fail gpio io:%d, group:%d (%s)\n", io, grp, __func__);
		break;
	};

	return val;
}

int nx_soc_gpio_get_value(unsigned int io)
{
	int val, dir;

	dir = nx_soc_gpio_get_io_dir(io);
	if (dir == 1)	/* Output */
		val = nx_soc_gpio_get_out_value(io);
	else if (dir == 0)	/* Input */
		val = nx_soc_gpio_get_in_value(io);
	else
		val = -1;

	return val;
}

int nx_soc_is_gpio_pin(unsigned int io)
{
	return (io < PAD_GPIO_ALV) ? 1 : 0;
}

static int nxp3220_gpio_suspend(int idx)
{
	struct nx_gpio_reg_set *regs;
	struct nx_gpio_reg_set *gpio_save;

	if (idx < 0 || idx >= NR_GPIO_MODULE)
		return -ENXIO;

	regs = gpio_modules[idx].gpio_regs;
	gpio_save = &gpio_modules[idx].gpio_save;

	gpio_save->gpio_out = readl(&regs->gpio_out);
	gpio_save->gpio_outenb = readl(&regs->gpio_outenb);
	gpio_save->gpio_altfn[0] = readl(&regs->gpio_altfn[0]);
	gpio_save->gpio_altfn[1] = readl(&regs->gpio_altfn[1]);
	gpio_save->gpio_detmode[0] = readl(&regs->gpio_detmode[0]);
	gpio_save->gpio_detmode[1] = readl(&regs->gpio_detmode[1]);
	gpio_save->gpio_detmodeex = readl(&regs->gpio_detmodeex);
	gpio_save->gpio_intenb = readl(&regs->gpio_intenb);

	gpio_save->gpio_slew = readl(&regs->gpio_slew);
	gpio_save->gpio_slew_disable_default =
		readl(&regs->gpio_slew_disable_default);
	gpio_save->gpio_drv1 = readl(&regs->gpio_drv1);
	gpio_save->gpio_drv1_disable_default =
		readl(&regs->gpio_drv1_disable_default);
	gpio_save->gpio_drv0 = readl(&regs->gpio_drv0);
	gpio_save->gpio_drv0_disable_default =
		readl(&regs->gpio_drv0_disable_default);
	gpio_save->gpio_pullsel = readl(&regs->gpio_pullsel);
	gpio_save->gpio_pullsel_disable_default =
		readl(&regs->gpio_pullsel_disable_default);
	gpio_save->gpio_pullenb = readl(&regs->gpio_pullenb);
	gpio_save->gpio_pullenb_disable_default =
		readl(&regs->gpio_pullenb_disable_default);
	gpio_save->gpio_inenb = readl(&regs->gpio_inenb);
	gpio_save->gpio_inenb_disable_default =
		readl(&regs->gpio_inenb_disable_default);
#ifdef CONFIG_ARCH_NXP3220
	gpio_save->gpio_altfnex = readl(&regs->gpio_altfnex);
#endif

	return 0;
}

static int nxp3220_gpio_resume(int idx)
{
	struct nx_gpio_reg_set *regs;
	struct nx_gpio_reg_set *gpio_save;

	if (idx < 0 || idx >= NR_GPIO_MODULE)
		return -ENXIO;

	regs = gpio_modules[idx].gpio_regs;
	gpio_save = &gpio_modules[idx].gpio_save;

	writel(gpio_save->gpio_slew, &regs->gpio_slew);
	writel(gpio_save->gpio_slew_disable_default,
		&regs->gpio_slew_disable_default);
	writel(gpio_save->gpio_drv1, &regs->gpio_drv1);
	writel(gpio_save->gpio_drv1_disable_default,
		&regs->gpio_drv1_disable_default);
	writel(gpio_save->gpio_drv0, &regs->gpio_drv0);
	writel(gpio_save->gpio_drv0_disable_default,
		&regs->gpio_drv0_disable_default);
	writel(gpio_save->gpio_pullsel, &regs->gpio_pullsel);
	writel(gpio_save->gpio_pullsel_disable_default,
		&regs->gpio_pullsel_disable_default);
	writel(gpio_save->gpio_pullenb, &regs->gpio_pullenb);
	writel(gpio_save->gpio_pullenb_disable_default,
		&regs->gpio_pullenb_disable_default);
	writel(gpio_save->gpio_inenb, &regs->gpio_inenb);
	writel(gpio_save->gpio_inenb_disable_default,
		&regs->gpio_inenb_disable_default);
#ifdef CONFIG_ARCH_NXP3220
	writel(gpio_save->gpio_altfnex, &regs->gpio_altfnex);
#endif

	writel(gpio_save->gpio_out, &regs->gpio_out);
	writel(gpio_save->gpio_outenb, &regs->gpio_outenb);
	writel(gpio_save->gpio_altfn[0], &regs->gpio_altfn[0]);
	writel(gpio_save->gpio_altfn[1], &regs->gpio_altfn[1]);
	writel(gpio_save->gpio_detmode[0], &regs->gpio_detmode[0]);
	writel(gpio_save->gpio_detmode[1], &regs->gpio_detmode[1]);
	writel(gpio_save->gpio_detmodeex, &regs->gpio_detmodeex);
	writel(gpio_save->gpio_intenb, &regs->gpio_intenb);
	writel(gpio_save->gpio_intenb, &regs->gpio_detenb);/* DETECT ENABLE */
	writel((u32)0xFFFFFFFF, &regs->gpio_det);	/* CLEAR PENDING */

	return 0;
}

static int nxp3220_alive_suspend(void)
{
	void __iomem *base = alive_regs;

	alive_detect_save = readl(base + ALIVE_DET_READ);
	writel(0xffffffff, base + ALIVE_DET_RST);
	writel(~get_wake_mask(), base + ALIVE_DET_SET);

	return 0;
}

static int nxp3220_alive_resume(void)
{
	void __iomem *base = alive_regs;

	writel(0xffffffff, base + ALIVE_DET_RST);
	writel(alive_detect_save, base + ALIVE_DET_SET);

	return 0;
}

static void nxp3220_retention_suspend(struct nexell_pinctrl_drv_data *drvdata)
{
	const struct nexell_pwr_func *pwr_func;
	void __iomem *base = alive_regs;
	unsigned int domain_mask = 0;
	int i, n, ret;

	for (i = 0; i < drvdata->nr_pwr_groups; i++) {
		pwr_func = &drvdata->pwr_functions[i];

		for (n = 0; n < pwr_func->num_groups; n++) {
			struct nexell_pin_pwr *pwr_pin = &pwr_func->groups[n];
			void __iomem *addr = base;
			const unsigned *pin;
			unsigned npin;
			int grp, bit;
			int val, dir, pullsel;

			if (pwr_pin->drive == NX_PIN_PWR_NONE)
				continue;

			ret = pinctrl_get_group_pins(drvdata->pctl_dev,
						pwr_pin->name, &pin, &npin);
			if (ret < 0)
				continue;

			grp = *pin / 32;
			bit = *pin % 32;
			val = pwr_pin->val;
			dir = pwr_pin->drive == NX_PIN_PWR_INPUT ? 0 : 1;
			pullsel = pwr_pin->pullsel;
			domain_mask |= 1 << pwr_func->domain;
			addr += (grp * ALIVE_PWRDN_OFFSET);

			if (pwr_pin->drive == NX_PIN_PWR_PREV)
				val = nx_gpio_get_input_value(grp, bit) ? 1 : 0;

			if (pwr_pin->pullsel != NX_PIN_PWR_PULL_PREV)
				nx_gpio_set_pull_enable(grp, bit, pullsel);

			nx_alive_setbit(addr + ALIVE_GPIO_OUT, bit, val);
			nx_alive_setbit(addr + ALIVE_GPIO_OUTENB, bit, dir);
			nx_alive_setbit(addr + ALIVE_GPIO_PWRDN, bit, 1);
		}
	}

	for (i = 0; i < NX_PIN_PWR_DOMAIN_NUM; i++) {
		if (domain_mask & 1<<i) {
			nx_alive_setbit(base + ALIVE_NPADHOLDENB, i, 0);
			nx_alive_setbit(base + ALIVE_NPADHOLD, i, 0);
		}
	}
}

static void nxp3220_retention_resume(struct nexell_pinctrl_drv_data *drvdata)
{
	const struct nexell_pwr_func *pwr_func;
	void __iomem *base = alive_regs;
	int i, n, ret;

	for (i = 0; i < NX_PIN_PWR_DOMAIN_NUM; i++)
		nx_alive_setbit(base + ALIVE_NPADHOLDENB, i, 1);

	for (i = 0; i < drvdata->nr_pwr_groups; i++) {
		pwr_func = &drvdata->pwr_functions[i];

		for (n = 0; n < pwr_func->num_groups; n++) {
			struct nexell_pin_pwr *pwr_pin = &pwr_func->groups[n];
			void __iomem *addr = base;
			const unsigned *pin;
			unsigned npin;
			int grp, bit;

			if (pwr_pin->drive == NX_PIN_PWR_NONE)
				continue;

			ret = pinctrl_get_group_pins(drvdata->pctl_dev,
						pwr_pin->name, &pin, &npin);
			if (ret < 0)
				continue;

			grp = *pin / 32;
			bit = *pin % 32;

			addr += (grp * ALIVE_PWRDN_OFFSET);

			nx_alive_setbit(addr + ALIVE_GPIO_PWRDN, bit, 0);
		}
	}
}

static int nxp3220_gpio_device_init(struct list_head *banks, int nr_banks)
{
	struct module_init_data *init_data;
	int i;

	for (i = 0; i < NR_GPIO_MODULE + 1; i++)
		IO_LOCK_INIT(i);

	gpio_fn_no = nxp3220_pio_fn_no;

	i = 0;
	list_for_each_entry(init_data, banks, node) {
		if (init_data->bank_type == EINT_TYPE_GPIO) {
			gpio_modules[i].gpio_regs = init_data->bank_base;

			nx_gpio_open_module(i);
			i++;
		} else if (init_data->bank_type == EINT_TYPE_WKUP) { /* alive */
			alive_regs = init_data->bank_base;

			/*
			 * ALIVE Power Gate must enable for RTC register access.
			 * must be clear wfi jump address
			 */
			nx_alive_set_write_enable(alive_regs, true);
		}
	}

	return 0;
}

/*
 * irq_chip functions
 */

static void irq_gpio_ack(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, bank->irq, bank->name,
		 bit);

	writel((1 << bit), base + GPIO_INT_STATUS); /* irq pend clear */
}

static void irq_gpio_mask(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, bank->irq, bank->name,
		 bit);

	/* mask:irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1 << bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1 << bit), base + GPIO_INT_DET);
}

static void irq_gpio_unmask(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, bank->irq, bank->name,
		 bit);

	/* unmask:irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1 << bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1 << bit), base + GPIO_INT_DET);
}

static int irq_gpio_set_type(struct irq_data *irqd, unsigned int type)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;
	u32 val, alt;
	ulong reg;

	int mode = 0;

	pr_debug("%s: gpio irq=%d, %s.%d, type=0x%x\n", __func__, bank->irq,
		 bank->name, bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:
		pr_warn("%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_RISING:
		mode = NX_GPIO_INTMODE_RISINGEDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode = NX_GPIO_INTMODE_FALLINGEDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode = NX_GPIO_INTMODE_BOTHEDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		mode = NX_GPIO_INTMODE_LOWLEVEL;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		mode = NX_GPIO_INTMODE_HIGHLEVEL;
		break;
	default:
		pr_err("%s: No such irq type %d", __func__, type);
		return -1;
	}

	/*
	 * must change mode to gpio to use gpio interrupt
	 */

	/* gpio input : output disable, input enable */
	writel(readl(base + GPIO_OUT_ENB) & ~(1 << bit), base + GPIO_OUT_ENB);
	writel(readl(base + GPIO_IN_ENB) | (1 << bit), base + GPIO_IN_ENB);

	/* gpio mode : interrupt mode */
	reg = (ulong)(base + GPIO_INT_MODE + (bit / 16) * 4);
	val = (readl((void *)reg) & ~(3 << ((bit & 0xf) * 2))) |
	      ((mode & 0x3) << ((bit & 0xf) * 2));
	writel(val, (void *)reg);

	reg = (ulong)(base + GPIO_INT_MODEEX);
	val = (readl((void *)reg) & ~(1 << bit)) | (((mode >> 2) & 0x1) << bit);
	writel(val, (void *)reg);

	/* gpio alt : gpio mode for irq */
	reg = (ulong)(base + GPIO_ALTFN + (bit / 16) * 4);
	val = readl((void *)reg) & ~(3 << ((bit & 0xf) * 2));
	alt = nx_soc_gpio_get_altnum(bank->grange.pin_base + bit);
	val |= alt << ((bit & 0xf) * 2);
	writel(val, (void *)reg);
#ifdef CONFIG_ARCH_NXP3220
	writel(readl(base + GPIO_ALTFNEX) & ~(1 << bit), base + GPIO_ALTFNEX);
#endif
	pr_debug("%s: set func to gpio. alt:%d, base:%d, bit:%d\n", __func__,
		 alt, bank->grange.pin_base, bit);

	return 0;
}

static void irq_gpio_enable(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, bank->irq, bank->name,
		 bit);

	/* unmask:irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1 << bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1 << bit), base + GPIO_INT_DET);
}

static void irq_gpio_disable(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	int bit = (int)(irqd->hwirq);
	void __iomem *base = bank->virt_base;

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, bank->irq, bank->name,
		 bit);

	/* mask:irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1 << bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1 << bit), base + GPIO_INT_DET);
}

/*
 * irq_chip for gpio interrupts.
 */
static struct irq_chip nxp3220_gpio_irq_chip = {
	.name = "GPIO",
	.irq_ack = irq_gpio_ack,
	.irq_mask = irq_gpio_mask,
	.irq_unmask = irq_gpio_unmask,
	.irq_set_type = irq_gpio_set_type,
	.irq_enable = irq_gpio_enable,
	.irq_disable = irq_gpio_disable,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static int nxp3220_gpio_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	struct nexell_pin_bank *b = h->host_data;

	pr_debug("%s domain map: virq %d and hw %d\n", __func__, virq, (int)hw);

	irq_set_chip_data(virq, b);
	irq_set_chip_and_handler(virq, &nxp3220_gpio_irq_chip,
				 handle_level_irq);
	return 0;
}

/*
 * irq domain callbacks for external gpio interrupt controller.
 */
static const struct irq_domain_ops nxp3220_gpio_irqd_ops = {
	.map = nxp3220_gpio_irq_map, .xlate = irq_domain_xlate_twocell,
};

static irqreturn_t nxp3220_gpio_irq_handler(int irq, void *data)
{
	struct nexell_pin_bank *bank = data;
	void __iomem *base = bank->virt_base;
	u32 stat, mask;
	unsigned int virq;
	int bit;

	mask = readl(base + GPIO_INT_ENB);
	stat = readl(base + GPIO_INT_STATUS) & mask;
	bit = ffs(stat) - 1;

	if (bit == -1) {
		pr_err("Unknown gpio irq=%d, status=0x%08x, mask=0x%08x\r\n",
		       irq, stat, mask);
		writel(-1, (base + GPIO_INT_STATUS)); /* clear gpio status all*/
		return IRQ_NONE;
	}

	virq = irq_linear_revmap(bank->irq_domain, bit);
	if (!virq)
		return IRQ_NONE;

	pr_debug("Gpio irq=%d [%d] (hw %u), stat=0x%08x, mask=0x%08x\n", irq,
		 bit, virq, stat, mask);
	generic_handle_irq(virq);

	return IRQ_HANDLED;
}

/*
 * nxp3220_gpio_irq_init() - setup handling of external gpio interrupts.
 * @d: driver data of nexell pinctrl driver.
 */
static int nxp3220_gpio_irq_init(struct nexell_pinctrl_drv_data *d)
{
	struct nexell_pin_bank *bank;
	struct device *dev = d->dev;
	int ret;
	int i;

	bank = d->ctrl->pin_banks;
	for (i = 0; i < d->ctrl->nr_banks; ++i, ++bank) {
		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;

		ret = devm_request_irq(dev, bank->irq,
				       nxp3220_gpio_irq_handler,
				       0, dev_name(dev), bank);
		if (ret) {
			dev_err(dev, "irq request failed\n");
			ret = -ENXIO;
			goto err_domains;
		}

		bank->irq_domain = irq_domain_add_linear(
				bank->of_node, bank->nr_pins,
				&nxp3220_gpio_irqd_ops, bank);
		if (!bank->irq_domain) {
			dev_err(dev, "gpio irq domain add failed\n");
			ret = -ENXIO;
			goto err_domains;
		}
	}

	return 0;

err_domains:
	for (--i, --bank; i >= 0; --i, --bank) {
		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;
		irq_domain_remove(bank->irq_domain);
		devm_free_irq(dev, bank->irq, d);
	}

	return ret;
}

static void irq_alive_ack(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);

	pr_debug("%s: irq=%d, hwirq=%d\n", __func__, irqd->irq, bit);
	/* ack: irq pend clear */
	writel(1 << bit, base + ALIVE_INT_STATUS);
}

static void irq_alive_mask(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);

	pr_debug("%s: irq=%d, hwirq=%d\n", __func__, irqd->irq, bit);
	/* mask: irq reset (disable) */
	writel(1 << bit, base + ALIVE_INT_ENB_RST);
}

static void irq_alive_unmask(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);

	pr_debug("%s: irq=%d, hwirq=%d\n", __func__, irqd->irq, bit);
	/* mask: irq set (enable) */
	writel(1 << bit, base + ALIVE_INT_ENB_SET);
}

static int irq_alive_set_type(struct irq_data *irqd, unsigned int type)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);
	int offs = 0, i = 0;
	int mode = 0;

	pr_debug("%s: irq=%d, hwirq=%d, type=0x%x\n",
			__func__, irqd->irq, bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:
		pr_warn("%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE;
		break;
	case IRQ_TYPE_EDGE_RISING:
		mode = NX_ALIVE_DETECTMODE_SYNC_RISINGEDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE;
		break; /* and Rising Edge */
	case IRQ_TYPE_LEVEL_LOW:
		mode = NX_ALIVE_DETECTMODE_ASYNC_LOWLEVEL;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		mode = NX_ALIVE_DETECTMODE_ASYNC_HIGHLEVEL;
		break;
	default:
		pr_err("%s: No such irq type %d", __func__, type);
		return -1;
	}

	/* setting all alive detect mode set/reset register */
	for (; i < 6; i++, offs += 0x0C) {
		u32 reg = (i == mode ? ALIVE_MOD_SET : ALIVE_MOD_RESET);

		writel(1 << bit, base + reg + offs);
	}

	/*
	 * set risingedge mode for both edge
	 */
	if (type == IRQ_TYPE_EDGE_BOTH)
		writel(1 << bit, base + ALIVE_MOD_EDGE_SET);

	writel(1 << bit, base + ALIVE_DET_SET);
	writel(1 << bit, base + ALIVE_INT_ENB_SET);
	writel(1 << bit, base + ALIVE_PAD_OUT_RST);

	return 0;
}

static int irq_set_alive_wake(struct irq_data *irqd, unsigned int on)
{
	int bit = (int)(irqd->hwirq);

	pr_info("alive wake bit[%d] %s for irq %d\n",
			bit, on ? "enabled" : "disabled", irqd->irq);

	if (!on)
		alive_wake_mask |= (1 << bit);
	else
		alive_wake_mask &= ~(1 << bit);

	return 0;
}

static void irq_alive_enable(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);

	pr_debug("%s: irq=%d, io=%s.%d\n",
			__func__, bank->irq, bank->name, bit);
	/* unmask:irq set (enable) */
	writel(1 << bit, base + ALIVE_INT_ENB_SET);
}

static void irq_alive_disable(struct irq_data *irqd)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	void __iomem *base = bank->virt_base;
	int bit = (int)(irqd->hwirq);

	pr_debug("%s: irq=%d, io=%s.%d\n", __func__,
			bank->irq, bank->name, bit);
	/* mask:irq reset (disable) */
	writel(1 << bit, base + ALIVE_INT_ENB_RST);
}

/*
 * irq_chip for wakeup interrupts
 */
static struct irq_chip nxp3220_alive_irq_chip = {
	.name = "ALIVE",
	.irq_ack = irq_alive_ack,
	.irq_mask = irq_alive_mask,
	.irq_unmask = irq_alive_unmask,
	.irq_set_type = irq_alive_set_type,
	.irq_set_wake = irq_set_alive_wake,
	.irq_enable = irq_alive_enable,
	.irq_disable = irq_alive_disable,
};

static irqreturn_t nxp3220_alive_irq_handler(int irq, void *data)
{
	struct nexell_pin_bank *bank = data;
	void __iomem *base = bank->virt_base;
	u32 stat, mask;
	unsigned int virq;
	int bit;

	mask = readl(base + ALIVE_INT_ENB_READ);
	stat = readl(base + ALIVE_INT_STATUS) & mask;
	bit = ffs(stat) - 1;

	if (bit == -1) {
		pr_err("Unknown alive irq=%d, status=0x%08x, mask=0x%08x\r\n",
				irq, stat, mask);
		writel(-1, base + ALIVE_INT_STATUS);
		return IRQ_NONE;
	}

	virq = irq_linear_revmap(bank->irq_domain, bit);
	pr_debug("alive irq=%d [%d] (hw %u), stat=0x%08x, mask=0x%08x\n",
			irq, bit, virq, stat, mask);
	if (!virq)
		return IRQ_NONE;

	generic_handle_irq(virq);

	return IRQ_HANDLED;
}

static int nxp3220_alive_irq_map(struct irq_domain *h, unsigned int virq,
				 irq_hw_number_t hw)
{
	pr_debug("%s domain map: virq %d and hw %d\n", __func__, virq, (int)hw);

	irq_set_chip_and_handler(virq, &nxp3220_alive_irq_chip,
				 handle_level_irq);
	irq_set_chip_data(virq, h->host_data);
	return 0;
}

static const struct irq_domain_ops nxp3220_alive_irqd_ops = {
	.map = nxp3220_alive_irq_map, .xlate = irq_domain_xlate_twocell,
};

/*
 * nxp3220_alive_irq_init() - setup handling of wakeup interrupts.
 * @d: driver data of nexell pinctrl driver.
 */
static int nxp3220_alive_irq_init(struct nexell_pinctrl_drv_data *d)
{
	struct nexell_pin_bank *bank;
	struct device *dev = d->dev;
	int ret;
	int i;

	bank = d->ctrl->pin_banks;
	for (i = 0; i < d->ctrl->nr_banks; ++i, ++bank) {
		void __iomem *base = bank->virt_base;

		if (bank->eint_type != EINT_TYPE_WKUP)
			continue;

		/* clear pending, disable irq detect */
		writel(-1, base + ALIVE_INT_ENB_RST);
		writel(-1, base + ALIVE_INT_STATUS);

		ret =
		  devm_request_irq(dev, bank->irq, nxp3220_alive_irq_handler,
				   0, dev_name(dev), bank);
		if (ret) {
			dev_err(dev, "irq request failed\n");
			ret = -ENXIO;
			goto err_domains;
		}

		bank->irq_domain =
		    irq_domain_add_linear(bank->of_node, bank->nr_pins,
					  &nxp3220_alive_irqd_ops, bank);
		if (!bank->irq_domain) {
			dev_err(dev, "gpio irq domain add failed\n");
			ret = -ENXIO;
			goto err_domains;
		}
	}

	return 0;

err_domains:
	for (--i, --bank; i >= 0; --i, --bank) {
		if (bank->eint_type != EINT_TYPE_WKUP)
			continue;
		irq_domain_remove(bank->irq_domain);
		devm_free_irq(dev, bank->irq, d);
	}

	return ret;
}


static void nxp3220_suspend(struct nexell_pinctrl_drv_data *drvdata)
{
	struct nexell_pin_ctrl *ctrl = drvdata->ctrl;
	int nr_banks = ctrl->nr_banks;
	int i;

	for (i = 0; i < nr_banks; i++) {
		struct nexell_pin_bank *bank = &ctrl->pin_banks[i];

		if (bank->eint_type == EINT_TYPE_WKUP) {
			nxp3220_alive_suspend();
			continue;
		}

		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;

		if (nxp3220_gpio_suspend(i) < 0)
			dev_err(drvdata->dev, "failed to suspend bank %d\n", i);
	}

	nxp3220_retention_suspend(drvdata);

	nx_alive_clear_wakeup_status(alive_regs);
}

static const char * const wake_event_name[] = {
	[0] = "FAKERTC",
	[1] = "RTC",
	[2] = "ALIVE 0",
	[3] = "ALIVE 1",
	[4] = "ALIVE 2",
	[5] = "ALIVE 3",
	[6] = "ALIVE 4",
	[7] = "ALIVE 5",
	[8] = "ALIVE 6",
	[9] = "ALIVE 7",
	[10] = "ALIVE 8",
	[11] = "ALIVE 9",
};

#define	WAKE_EVENT_NUM	ARRAY_SIZE(wake_event_name)

static void print_wake_event(void)
{
	int i = 0;
	u32 wake_status = nx_alive_get_wakeup_status(alive_regs);

	for (i = 0; i < WAKE_EVENT_NUM; i++) {
		if (wake_status & (1 << i))
			pr_notice("WAKE SOURCE [%s]\n", wake_event_name[i]);
	}
}

static void nxp3220_resume(struct nexell_pinctrl_drv_data *drvdata)
{
	struct nexell_pin_ctrl *ctrl = drvdata->ctrl;
	int nr_banks = ctrl->nr_banks;
	int i;

	for (i = 0; i < nr_banks; i++) {
		struct nexell_pin_bank *bank = &ctrl->pin_banks[i];

		if (bank->eint_type == EINT_TYPE_WKUP) {
			nxp3220_alive_resume();
			continue;
		}

		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;

		if (nxp3220_gpio_resume(i) < 0)
			dev_err(drvdata->dev, "failed to resume bank %d\n", i);
	}

	nxp3220_retention_resume(drvdata);

	print_wake_event();
}

static int nxp3220_base_init(struct nexell_pinctrl_drv_data *drvdata)
{
	struct nexell_pin_ctrl *ctrl = drvdata->ctrl;
	int nr_banks = ctrl->nr_banks;
	int ret;
	int i;
	struct module_init_data *init_data, *n;
	LIST_HEAD(banks);

	for (i = 0; i < nr_banks; i++) {
		struct nexell_pin_bank *bank = &ctrl->pin_banks[i];

		init_data = kmalloc(sizeof(*init_data), GFP_KERNEL);
		if (!init_data) {
			ret = -ENOMEM;
			goto done;
		}

		INIT_LIST_HEAD(&init_data->node);
		init_data->bank_base = bank->virt_base;
		init_data->bank_type = bank->eint_type;

		list_add_tail(&init_data->node, &banks);
	}

	nxp3220_gpio_device_init(&banks, nr_banks);

done:
	/* free */
	list_for_each_entry_safe(init_data, n, &banks, node) {
		list_del(&init_data->node);
		kfree(init_data);
	}

	return 0;
}

/* pin banks of nxp3220 pin-controller */
static struct nexell_pin_bank nxp3220_pin_banks[] = {
	SOC_PIN_BANK_EINTG(32, "gpioa"),
	SOC_PIN_BANK_EINTG(32, "gpiob"),
	SOC_PIN_BANK_EINTG(32, "gpioc"),
	SOC_PIN_BANK_EINTG(32, "gpiod"),
	SOC_PIN_BANK_EINTG(32, "gpioe"),
	SOC_PIN_BANK_EINTW(14, "alive"),
};

/*
 * Nexell pinctrl driver data for SoC.
 */
const struct nexell_pin_ctrl nxp3220_pin_ctrl[] = {
	{
		.pin_banks = nxp3220_pin_banks,
		.nr_banks = ARRAY_SIZE(nxp3220_pin_banks),
		.base_init = nxp3220_base_init,
		.gpio_irq_init = nxp3220_gpio_irq_init,
		.alive_irq_init = nxp3220_alive_irq_init,
		.suspend = nxp3220_suspend,
		.resume = nxp3220_resume,
	},
};
