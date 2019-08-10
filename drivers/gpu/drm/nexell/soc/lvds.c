// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <dt-bindings/display/nexell.h>

#include "display.h"

struct nx_lvds_dev {
	struct nx_display_ctx ctx;
	struct nx_lvds_reg *reg;
	struct clk *vclk;
	struct clk *phy_clk;
	struct reset_control *rst;
	/* properties */
	unsigned int format; /* 0:VESA, 1:JEIDA, 2: Location */
	int voltage_level;
	int voltage_output;
};

struct nx_lvds_reg {
	u32 lvdsctrl0;		/* 0x00 */
	u32 lvdsvblkctrl0;	/* 0x04 */
	u32 lvdsvblkctrl1;	/* 0x08 */
	u32 lvdsctrl1;		/* 0x0c */
	u32 lvdsctrl2;		/* 0x10 */
	u32 lvdsctrl3;		/* 0x14 */
	u32 lvdsctrl4;		/* 0x18 */
	u32 lvdsloc0;		/* 0x1c */
	u32 lvdsloc1;		/* 0x20 */
	u32 lvdsloc2;		/* 0x24 */
	u32 lvdsloc3;		/* 0x28 */
	u32 lvdsloc4;		/* 0x2c */
	u32 lvdsloc5;		/* 0x30 */
	u32 lvdsloc6;		/* 0x34 */
	u32 lvdsloc7;		/* 0x38 */
	u32 lvdsloc8;		/* 0x3c */
	u32 lvdsloc9;		/* 0x40 */
	u32 lvdsmask0;		/* 0x44 */
	u32 lvdsmask1;		/* 0x48 */
	u32 lvdspol0;		/* 0x4c */
	u32 lvdspol1;		/* 0x50 */
	u32 lvdsda0;		/* 0x54 */
	u32 lvdsda1;		/* 0x58 */
	u32 lvdsda2;		/* 0x5c */
};

#define	DEF_VOLTAGE_LEVEL	(0x3f) /* 8bits width */
#define	DEF_VOLTAGE_OFFSET	LVDS_VOL_OFFS_1_1
#define MHZ(v)			(v * 1000000)

static int lvds_is_enabled(struct nx_lvds_dev *lvds)
{
#ifdef CONFIG_DRM_PRE_INIT_DRM
	struct nx_lvds_reg *reg = lvds->reg;

	if (!__clk_is_enabled(lvds->vclk))
		clk_prepare_enable(lvds->vclk);

	return (readl(&reg->lvdsctrl0) & (1 << 28)) ? 1 : 0;
#else
	return 0;
#endif
}

static int lvds_set_mode(struct nx_drm_display *display,
			 struct drm_display_mode *mode, unsigned int flags)
{
	struct nx_lvds_dev *lvds = display->context;

	pr_debug("%s: pixelclock: %ld, %s\n", __func__, display->vm.pixelclock,
		lvds_is_enabled(lvds) ? "enabled" : "disabled");

	lvds->ctx.vm = &display->vm;

	return 0;
}

static int lvds_prepare(struct nx_drm_display *display)
{
	struct nx_lvds_dev *lvds = display->context;
	struct nx_lvds_reg *reg = lvds->reg;
	unsigned int format = lvds->format;
	int pixelclock = display->vm.pixelclock;
	u32 v_level = lvds->voltage_level;
	u32 v_output = lvds->voltage_output;
	u32 val, pms_v, pms_s, pms_m, pms_p;

	pr_debug("%s: format: %d, voltage level:%d output:0x%x\n",
		 __func__, format, v_level, v_output);

	if (!lvds_is_enabled(lvds))
		clk_set_rate(lvds->vclk, display->vm.pixelclock);

	clk_prepare_enable(lvds->vclk);
	clk_prepare_enable(lvds->phy_clk);

	if (lvds_is_enabled(lvds))
		return 0;

	/* lvdsctrl0 */
	val = readl(&reg->lvdsctrl0);
	val &= ~((1<<24) | (1 << 23) | (0 << 22) | (0 << 21) | (0x3 << 19));
	val |= (0 << 23) /* DE_POL */
		| (0 << 22)	/* HSYNC_POL */
		| (0 << 21)	/* VSYNC_POL */
		| (format << 19) /* LVDS_FORMAT */
		;
	writel(val, &reg->lvdsctrl0);

	/* lvdsctrl2 */
	val = readl(&reg->lvdsctrl2);
	val &= ~((1 << 15) | (1 << 14) | (0x3 << 12) | (0x3f << 6) | (0x3f << 0));
	if (pixelclock >= MHZ(90)) {
		pms_v = 1 << 14;
		pms_s = 1 << 12;
		pms_m = 0xe << 6;
		pms_p = 0xe << 0;
	} else {
		pms_v = 0 << 14;
		pms_s = 0 << 12;
		pms_m = 0xa << 6;
		pms_p = 0xa << 0;
	}
	val |= pms_v | pms_s | pms_m | pms_p;
	writel(val, &reg->lvdsctrl2);

	/* lvdsctrl4 : CNT_VOD_H and FC_CODE */
	val = readl(&reg->lvdsctrl4);
	val &= ~((0xff << 14) | (0x7 << 3));
	val |= (((v_level & 0xff) << 14) | ((v_output & 0x7) << 3));
	writel(val, &reg->lvdsctrl4);

	return 0;
}

static int lvds_enable(struct nx_drm_display *display)
{
	struct nx_lvds_dev *lvds = display->context;
	struct nx_lvds_reg *reg = lvds->reg;

	pr_debug("%s\n", __func__);

	if (lvds_is_enabled(lvds))
		return 0;

	/* DPCENB */
	writel(readl(&reg->lvdsctrl0) | 1 << 28, &reg->lvdsctrl0);

	/* LVDS PHY Reset */
	reset_control_deassert(lvds->rst);

	return 0;
}

static int lvds_disable(struct nx_drm_display *display)
{
	struct nx_lvds_dev *lvds = display->context;
	struct nx_lvds_reg *reg = lvds->reg;

	pr_debug("%s\n", __func__);

	/* DPCENB */
	writel(readl(&reg->lvdsctrl0) & ~(1 << 28), &reg->lvdsctrl0);

	if (!__clk_is_enabled(lvds->vclk))
		clk_disable_unprepare(lvds->vclk);

	if (!__clk_is_enabled(lvds->phy_clk))
		clk_disable_unprepare(lvds->phy_clk);

	reset_control_assert(lvds->rst);

	return 0;
}

static struct nx_drm_display_ops nx_lvds_ops = {
	.set_mode = lvds_set_mode,
	.prepare = lvds_prepare,
	.enable = lvds_enable,
	.disable = lvds_disable,
};

void *nx_drm_display_lvds_get(struct device *dev,
			      struct device_node *node,
			      struct nx_drm_display *display)
{
	struct nx_lvds_dev *lvds;
	struct reset_control *rst;
	u32 format, voltage;

	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return NULL;

	lvds = kzalloc(sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return NULL;

	lvds->reg = of_iomap(dev->of_node, 0);
	if (!lvds->reg)
		return NULL;

	lvds->vclk = of_clk_get_by_name(dev->of_node, "vclk");
	if (!lvds->vclk)
		return NULL;

	lvds->phy_clk = of_clk_get_by_name(dev->of_node, "phy");
	if (!lvds->phy_clk)
		return NULL;

	lvds->rst = rst;
	lvds->format = LVDS_FORMAT_VESA;
	lvds->voltage_level = DEF_VOLTAGE_LEVEL;
	lvds->voltage_output = DEF_VOLTAGE_OFFSET;
	lvds->ctx.dpp.out_format = NX_DPC_FORMAT_RGB666;

	if (!of_property_read_u32(node, "format", &format))
		lvds->format = format;

	if (!of_property_read_u32(node, "voltage-level", &voltage))
		lvds->voltage_level = voltage;

	if (!of_property_read_u32(node, "voltage-output", &voltage))
		lvds->voltage_output = voltage;

	display->context = lvds;
	display->ops = &nx_lvds_ops;

	if (!lvds_is_enabled(lvds))
		reset_control_assert(lvds->rst);

	return &lvds->ctx;
}
