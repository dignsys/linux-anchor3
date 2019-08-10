// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "display.h"

struct nx_rgb_dev {
	struct nx_display_ctx ctx;
	struct regmap *syscon;
	bool mpu_lcd; /* i80 */
};

static void rgb_sel_pad(struct nx_rgb_dev *rgb, int muxsel, int clksel)
{
	struct regmap *syscon = rgb->syscon;

	if (IS_ERR(syscon))
		return;

	regmap_update_bits(syscon, 0x100, 0x7<<4, clksel<<4);
	regmap_update_bits(syscon, 0x100, 0x3, muxsel);
}

static int rgb_set_mode(struct nx_drm_display *display,
			struct drm_display_mode *mode, unsigned int flags)
{
	struct nx_rgb_dev *rgb = display->context;
	struct nx_display_par *dpp = &rgb->ctx.dpp;
	enum nx_dpc_padclk clksel = NX_DPC_PADCLK_CLK;
	enum nx_dpc_padmux muxsel = NX_DPC_PADNUX_MLC;

	rgb->ctx.vm = &display->vm;

	if (rgb->mpu_lcd)
		muxsel = NX_DPC_PADNUX_MPU;

	if (dpp->out_format == NX_DPC_FORMAT_SRGB888)
		clksel = NX_DPC_PADCLK_CLK_DIV2_90;

	rgb_sel_pad(rgb, muxsel, clksel);

	return 0;
}

static int rgb_enable(struct nx_drm_display *display)
{
	return 0;
}

static struct nx_drm_display_ops rgb_ops = {
	.set_mode = rgb_set_mode,
	.enable = rgb_enable,
};

void *nx_drm_display_rgb_get(struct device *dev,
			     struct device_node *node,
			     struct nx_drm_display *display)
{
	struct nx_rgb_dev *rgb;
	struct regmap *syscon;
	u32 mpu_lcd = 0;

	syscon = syscon_regmap_lookup_by_phandle(node, "syscon");
	if (IS_ERR(syscon))
		return NULL;

	rgb = kzalloc(sizeof(*rgb), GFP_KERNEL);
	if (!rgb)
		return NULL;

	of_property_read_u32(node, "panel-mpu", &mpu_lcd);

	rgb->mpu_lcd = mpu_lcd ? true : false;
	rgb->syscon = syscon;

	/* set mpu(i80) LCD */
	rgb->ctx.mpu_lcd = rgb->mpu_lcd;

	/* default out format: RGB888:3 */
	rgb->ctx.dpp.out_format = NX_DPC_FORMAT_RGB888;

	display->context = rgb;
	display->ops = &rgb_ops;

	return &rgb->ctx;
}
