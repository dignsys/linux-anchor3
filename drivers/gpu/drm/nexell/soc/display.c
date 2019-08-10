// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM SoC low interface driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/io.h>

#include "display.h"

#define	VID_FILTER_MAX			2048
#define	RGB_RECTANGLE_MAX		2048

/* the PAD output delay. */
#define	DPC_SYNC_DELAY_RGB_PVD		(1<<0)
#define	DPC_SYNC_DELAY_HSYNC_CP1	(1<<1)
#define	DPC_SYNC_DELAY_VSYNC_FRAME	(1<<2)
#define	DPC_SYNC_DELAY_DE_CP		(1<<3)

#define INTERLACE(f) (f & DISPLAY_FLAGS_INTERLACED ? true : false)
#define RGB_MODE(fmt) ( \
		fmt == NX_DPC_FORMAT_CCIR656 || \
		fmt == NX_DPC_FORMAT_CCIR601_8 || \
		fmt == NX_DPC_FORMAT_CCIR601_16A || \
		fmt == NX_DPC_FORMAT_CCIR601_16B ? false : true)
#define SYNC_POL(s, t) (s & t ? NX_DPC_POL_ACTIVELOW : NX_DPC_POL_ACTIVEHIGH)
#define SYNC_VAL(v, d) (v ? v : d)

/* 12345'678'[8] -> 12345 [5], 123456'78'[8] -> 123456[6] */
static inline u16 rgb888_to_rgb565(u32 rgb)
{
	u8 r = (rgb >> 16) & 0xf8;
	u8 g = (rgb >> 8) & 0xfc;
	u8 b = rgb & 0xf8;

	return (r << 8) | (g << 3) | (b >> 3);
}

/* 12345 [5] -> 12345'123'[8], 123456[6] -> 123456'12'[8] */
static inline u32 rgb565_to_rgb888(u16 rgb)
{
	u8 r5 = (rgb >> 11) & 0x1f;
	u8 g6 = (rgb >> 5) & 0x3f;
	u8 b5 = (rgb >> 0) & 0x1f;
	u8 r8 = ((r5 << 3) & 0xf8) | ((r5 >> 2) & 0x7);
	u8 g8 = ((g6 << 2) & 0xfc) | ((g6 >> 4) & 0x3);
	u8 b8 = ((b5 << 3) & 0xf8) | ((b5 >> 2) & 0x7);

	return (r8 << 16) | (g8 << 8) | (b8);
}

/* 123'45678'[8] -> 123[3], 12'345678'[8] -> 12 [2] */
static inline u8 rgb888_to_rgb332(unsigned int rgb)
{
	u8 r = (rgb >> 16) & 0xe0;
	u8 g = (rgb >> 8) & 0xe0;
	u8 b = (rgb >> 0) & 0xc0;

	return (r) | (g >> 3) | (b >> 6);
}

/* 123[3] -> 123'123'12' [8], 12 [2] -> 12'12'12'12'[8] */
static inline unsigned int rgb332_to_rgb888(u8 rgb)
{
	u8 r3 = (rgb >> 5) & 0x7;
	u8 g3 = (rgb >> 2) & 0x7;
	u8 b2 = (rgb >> 0) & 0x3;
	u8 r8 = ((r3 << 5) | (r3 << 2) | (r3 >> 1));
	u8 g8 = ((g3 << 5) | (g3 << 2) | (g3 >> 1));
	u8 b8 = ((b2 << 6) | (b2 << 4) | (b2 << 2) | b2);

	return (r8 << 16) | (g8 << 8) | (b8);
}

static void nx_display_clock_enable(struct nx_display *dp, bool on)
{
	struct nx_mlc_reg *reg = dp->mlc_base;

	if (on) {
		clk_prepare_enable(dp->clk_axi);

		nx_mlc_set_clock_pclk_mode(reg, NX_PCLK_MODE_ALWAYS);
		nx_mlc_set_clock_bclk_mode(reg, NX_BCLK_MODE_ALWAYS);

		clk_prepare_enable(dp->clk_x2);
		clk_prepare_enable(dp->clk_x1);
	} else {
		clk_disable_unprepare(dp->clk_x1);
		clk_disable_unprepare(dp->clk_x2);
		clk_disable_unprepare(dp->clk_axi);
	}
}

static void nx_display_clock_rate(struct nx_display *dp)
{
	struct videomode *vm = &dp->vm;
	enum nx_dpc_outformat format = dp->dpp->out_format;
	unsigned long rate;
	int div = 1;

	clk_set_rate(dp->clk_x2, vm->pixelclock);
	rate = clk_get_rate(dp->clk_x2);

	if (dp->mpu_lcd)
		div = 2;

	if (format == NX_DPC_FORMAT_MRGB565)
		div = 2;
	else if	(format == NX_DPC_FORMAT_SRGB888)
		div = 6;

	clk_set_rate(dp->clk_x1, rate/div);
}

static int nx_display_is_enabled(struct nx_display *dp)
{
#ifdef CONFIG_DRM_PRE_INIT_DRM
	nx_display_clock_enable(dp, true);

	return nx_dpc_get_enable(dp->dpc_base) ? 1 : 0;
#else
	return 0;
#endif
}

static void nx_display_sync_format(struct nx_display *dp)
{
	struct nx_dpc_reg *reg = dp->dpc_base;
	struct videomode *vm = &dp->vm;
	struct nx_display_par *dpp = dp->dpp;
	enum nx_dpc_outformat format = dpp->out_format;
	bool emb_sync = format == NX_DPC_FORMAT_CCIR656 ? true : false;
	enum nx_dpc_dither r_dither, g_dither, b_dither;

	if ((format == NX_DPC_FORMAT_RGB555) ||
	    (format == NX_DPC_FORMAT_MRGB555A) ||
	    (format == NX_DPC_FORMAT_MRGB555B)) {
		r_dither = NX_DPC_DITHER_5BIT;
		g_dither = NX_DPC_DITHER_5BIT;
		b_dither = NX_DPC_DITHER_5BIT;
	} else if ((format == NX_DPC_FORMAT_RGB565) ||
		   (format == NX_DPC_FORMAT_MRGB565)) {
		r_dither = NX_DPC_DITHER_5BIT;
		b_dither = NX_DPC_DITHER_5BIT;
		g_dither = NX_DPC_DITHER_6BIT;
	} else if ((format == NX_DPC_FORMAT_RGB666) ||
		   (format == NX_DPC_FORMAT_MRGB666)) {
		r_dither = NX_DPC_DITHER_6BIT;
		g_dither = NX_DPC_DITHER_6BIT;
		b_dither = NX_DPC_DITHER_6BIT;
	} else {
		r_dither = NX_DPC_DITHER_BYPASS;
		g_dither = NX_DPC_DITHER_BYPASS;
		b_dither = NX_DPC_DITHER_BYPASS;
	}

	nx_dpc_set_mode(reg, format,
			INTERLACE(vm->flags), RGB_MODE(format),
			dpp->swap_rb ? true : false,
			emb_sync, dpp->yc_order);
	nx_dpc_set_dither(reg, r_dither, g_dither, b_dither);

	dev_dbg(dp->dev, "%s: crtc.%d: %s, %s\n",
		__func__, dp->module, RGB_MODE(format) ? "RGB" : "YUV",
		INTERLACE(vm->flags) ? "INTERACE" : "PROGRESSIVE");
}

static void nx_display_sync_delay(struct nx_display *dp)
{
	struct nx_dpc_reg *reg = dp->dpc_base;
	struct nx_display_par *dpp = dp->dpp;
	int rgb_pvd = 0, hs_cp1 = 7;
	int vs_frm = 7, de_cp2 = 7;

	if (dpp->delay_mask & DPC_SYNC_DELAY_RGB_PVD)
		rgb_pvd = dpp->d_rgb_pvd;
	if (dpp->delay_mask & DPC_SYNC_DELAY_HSYNC_CP1)
		hs_cp1 = dpp->d_hsync_cp1;
	if (dpp->delay_mask & DPC_SYNC_DELAY_VSYNC_FRAME)
		vs_frm = dpp->d_vsync_frame;
	if (dpp->delay_mask & DPC_SYNC_DELAY_DE_CP)
		de_cp2 = dpp->d_de_cp2;

	nx_dpc_set_delay(reg, rgb_pvd, hs_cp1, vs_frm, de_cp2);

	dev_dbg(dp->dev, "%s: crtc.%d: delay RGB:%d, HS:%d, VS:%d, DE:%d\n",
		__func__, dp->module, rgb_pvd, hs_cp1, vs_frm, de_cp2);
}

static void nx_display_sync_mode(struct nx_display *dp)
{
	struct nx_dpc_reg *reg = dp->dpc_base;
	struct videomode *vm = &dp->vm;
	enum display_flags flags = vm->flags;
	struct nx_display_par *dpp = dp->dpp;
	/* check with LOW bit */
	enum nx_dpc_polarity field_pol = dpp->invert_field ?
			NX_DPC_POL_ACTIVELOW : NX_DPC_POL_ACTIVEHIGH;
	enum nx_dpc_polarity hs_pol = SYNC_POL(flags, DISPLAY_FLAGS_HSYNC_LOW);
	enum nx_dpc_polarity vs_pol = SYNC_POL(flags, DISPLAY_FLAGS_VSYNC_LOW);
	int div = INTERLACE(flags) ? 2 : 1;
	int efp = SYNC_VAL(dpp->evfront_porch, vm->vfront_porch);
	int ebp = SYNC_VAL(dpp->evback_porch, vm->vback_porch);
	int esw = SYNC_VAL(dpp->evsync_len, vm->vsync_len);
	int eso = SYNC_VAL(dpp->evstart_offs, 1);
	int eeo = SYNC_VAL(dpp->evend_offs, 1);
	int vso = SYNC_VAL(dpp->vstart_offs, 1);
	int veo = SYNC_VAL(dpp->vend_offs, 1);

	nx_dpc_set_sync(reg, INTERLACE(flags),
			vm->hactive, vm->vactive/div,
			vm->hsync_len, vm->hfront_porch, vm->hback_porch,
			vm->vsync_len, vm->vfront_porch, vm->vback_porch,
			vso, veo, field_pol, hs_pol, vs_pol,
			esw, efp, ebp, eso, eeo);

	dev_dbg(dp->dev,
		"%s: crtc.%d: x:%4d, hfp:%3d, hbp:%3d, hsw:%3d, hi:%d\n",
		 __func__, dp->module, vm->hactive, vm->hfront_porch,
		 vm->hback_porch, vm->hsync_len, hs_pol);
	dev_dbg(dp->dev,
		"%s: crtc.%d: y:%4d, vfp:%3d, vbp:%3d, vsw:%3d, vi:%d\n",
		 __func__, dp->module, vm->vactive/div, vm->vfront_porch,
		 vm->vback_porch, vm->vsync_len, vs_pol);
	dev_dbg(dp->dev,
		"%s: crtc.%d: offset vs:%d, ve:%d, es:%d, ee:%d\n",
		__func__, dp->module, vso, veo, eso, eeo);
	dev_dbg(dp->dev,
		"%s: crtc.%d: even   ef:%d, eb:%d, es:%d]\n",
		__func__, dp->module, efp, ebp, esw);
}

int nx_display_set_mode(struct nx_display *dp)
{
	dev_dbg(dp->dev, "%s: crtc.%d\n", __func__, dp->module);

	if (nx_display_is_enabled(dp)) {
		dev_dbg(dp->dev,
			"%s: crtc.%d power prepared\n", __func__,
			dp->module);
		return 0;
	}

	nx_display_clock_rate(dp);
	nx_display_clock_enable(dp, true);
	nx_display_sync_format(dp);
	nx_display_sync_mode(dp);
	nx_display_sync_delay(dp);

	return 0;
}

void nx_display_enable(struct nx_display *dp, bool on)
{
	struct nx_dpc_reg *reg = dp->dpc_base;
	int count = 5;

	if (on && nx_display_is_enabled(dp)) {
		nx_dpc_clear_interrupt_pending_all(reg);
		dev_dbg(dp->dev,
			"%s: crtc.%d power enabled\n", __func__,
			dp->module);
		return;
	}

	nx_dpc_clear_interrupt_pending_all(reg);
	if (on)
		nx_dpc_set_reg_flush(reg);

	nx_dpc_set_enable(reg, on);
	nx_display_clock_enable(dp, on);

	if (!on)
		return;

	/*
	 * wait for video sync
	 * output devices.
	 */
	while (count-- > 0) {
		if (nx_dpc_get_interrupt_pending(reg))
			break;
		msleep(20);
	}
}

void nx_display_irq_on(struct nx_display *dp, bool on)
{
	struct nx_dpc_reg *reg = dp->dpc_base;

	dev_dbg(dp->dev, "%s: crtc.%d, %s\n",
		__func__, dp->module, on ? "on" : "off");

	nx_dpc_clear_interrupt_pending_all(reg);
	nx_dpc_set_interrupt_enable_all(reg, on);
}

void nx_display_irq_clear(struct nx_display *dp)
{
	nx_dpc_clear_interrupt_pending_all(dp->dpc_base);
}

void nx_display_set_format(struct nx_display *dp, int width, int height)
{
	struct nx_mlc_reg *reg = dp->mlc_base;
	unsigned int bgcolor = dp->back_color;
	int prior = dp->video_priority;

	switch (prior) {
	case 0:
		prior = NX_MLC_PRIORITY_VIDEO_1ST;
		break;
	case 1:
		prior = NX_MLC_PRIORITY_VIDEO_2ND;
		break;
	case 2:
		prior = NX_MLC_PRIORITY_VIDEO_3RD;
		break;
	default:
		dev_err(dp->dev,
			"Invalid video priority (0~3),(%d)\n", prior);
		return;
	}

	dp->width = width;
	dp->height = height;

	dev_dbg(dp->dev, "%s: crtc.%d, %d by %d, priority %d, bg 0x%x\n",
		__func__, dp->module, width, height, prior, bgcolor);

	nx_mlc_set_screen_size(reg, width, height);
	nx_mlc_set_video_priority(reg, prior);
	nx_mlc_set_background(reg, bgcolor & 0x00FFFFFF);
	nx_mlc_set_dirty(reg);
}

void nx_display_set_backcolor(struct nx_display *dp)
{
	struct nx_mlc_reg *reg = dp->mlc_base;
	unsigned int bgcolor = dp->back_color;

	dev_dbg(dp->dev, "%s: crtc.%d, bg:0x%x\n",
		__func__, dp->module, bgcolor);

	nx_mlc_set_background(reg, bgcolor & 0x00FFFFFF);
	nx_mlc_set_dirty(reg);
}

void nx_display_ovl_enable(struct nx_display *dp)
{
	struct nx_mlc_reg *reg = dp->mlc_base;
	struct videomode *vm = &dp->vm;
	struct nx_overlay *ovl;
	int lock_size = 16;

	dev_dbg(dp->dev, "%s: crtc.%d, %dx%d\n",
		__func__, dp->module, dp->width, dp->height);

	nx_mlc_set_field_enable(reg, INTERLACE(vm->flags));
	nx_mlc_set_rgb_gamma_power(reg, 0, 0, 0);
	nx_mlc_set_rgb_gamma_enable(reg, false);
	nx_mlc_set_gamma_priority(reg, 0);
	nx_mlc_set_gamma_dither(reg, false);
	nx_mlc_set_power_mode(reg, true);
	nx_mlc_set_enable(reg, 1);

	list_for_each_entry(ovl, &dp->overlay_list, list) {
		nx_mlc_set_layer_lock_size(reg, ovl->id, lock_size);
		if (ovl->enable) {
			nx_mlc_set_layer_enable(reg, ovl->id, true);
			nx_mlc_set_layer_dirty(reg, ovl->id, true);
			dev_dbg(dp->dev, "%s: %s on\n", __func__, ovl->name);
		}
	}

	nx_mlc_set_dirty(reg);
}

void nx_display_ovl_disable(struct nx_display *dp)
{
	struct nx_mlc_reg *reg = dp->mlc_base;
	struct nx_overlay *ovl;

	dev_dbg(dp->dev, "%s: crtc.%d, %dx%d\n",
		__func__, dp->module, dp->width, dp->height);

	list_for_each_entry(ovl, &dp->overlay_list, list) {
		if (ovl->enable) {
			nx_mlc_set_layer_enable(reg, ovl->id, false);
			nx_mlc_set_layer_dirty(reg, ovl->id, true);
		}
	}

	nx_mlc_set_power_mode(reg, false);
	nx_mlc_set_enable(reg, 0);
	nx_mlc_set_dirty(reg);
}

static int nx_overlay_rgb_set_format(struct nx_overlay *ovl,
				     unsigned int format, int pixelbyte,
				     bool sync)
{
	struct nx_mlc_reg *reg = ovl->base;
	int id = ovl->id;
	int lock_size = 16;
	bool alpha = false;

	dev_dbg(ovl->dp->dev, "%s: %s, fmt:0x%x, pixel:%d\n",
		 __func__, ovl->name, format, pixelbyte);

	ovl->format = format;
	ovl->pixelbyte = pixelbyte;

	/* set alphablend */
	if (format == NX_MLC_FMT_RGB_A1R5G5B5 ||
	    format == NX_MLC_FMT_RGB_A1B5G5R5 ||
	    format == NX_MLC_FMT_RGB_A4R4G4B4 ||
	    format == NX_MLC_FMT_RGB_A4B4G4R4 ||
	    format == NX_MLC_FMT_RGB_A8R3G3B2 ||
	    format == NX_MLC_FMT_RGB_A8B3G3R2 ||
	    format == NX_MLC_FMT_RGB_A8R8G8B8 ||
	    format == NX_MLC_FMT_RGB_A8B8G8R8)
		alpha = true;

	if (ovl->bgr_mode) {
		format |= 1<<31;
		dev_dbg(ovl->dp->dev,
			"%s: BGR plane format:0x%x\n", __func__, format);
	}

	if (ovl->color.alphablend < MAX_ALPHA_VALUE)
		alpha = true;

	if (!ovl->alphablend_on)
		alpha = false;

	nx_mlc_set_layer_lock_size(reg, id, lock_size);
	nx_mlc_set_layer_alpha(reg, id, ovl->color.alphablend, alpha);
	nx_mlc_set_rgb_color_inv(reg, id, ovl->color.invertcolor, false);
	nx_mlc_set_rgb_format(reg, id, format);
	nx_mlc_set_rgb_invalid_position(reg, id, 0, 0, 0, 0, 0, false);
	nx_mlc_set_rgb_invalid_position(reg, id, 1, 0, 0, 0, 0, false);
	nx_mlc_set_layer_dirty(reg, id, sync);

	return 0;
}

static int nx_overlay_rgb_set_pos(struct nx_overlay *ovl,
				  int src_x, int src_y, int src_w, int src_h,
				  int dst_x, int dst_y, int dst_w, int dst_h,
				  bool sync)

{
	struct nx_mlc_reg *reg = ovl->base;
	int id = ovl->id;
	int sx, sy, ex, ey;

	/* source */
	ovl->left = src_x;
	ovl->top = src_y;
	ovl->width = src_w;
	ovl->height = src_h;

	sx = dst_x, sy = dst_y;
	ex = dst_x + dst_w;
	ey = dst_y + dst_h;

	/* max rectangle */
	if (ex > RGB_RECTANGLE_MAX)
		ex = RGB_RECTANGLE_MAX;

	if (ey > RGB_RECTANGLE_MAX)
		ey = RGB_RECTANGLE_MAX;

	dev_dbg(ovl->dp->dev,
		"%s: %s, (%d, %d, %d, %d) to (%d, %d, %d, %d)\n",
		 __func__, ovl->name,
		src_x, src_y, src_w, src_h, sx, sy, ex, ey);

	nx_mlc_set_layer_position(reg, id, sx, sy, ex - 1, ey - 1);
	nx_mlc_set_layer_dirty(reg, id, sync);

	return 0;
}

static void nx_overlay_rgb_enb(struct nx_overlay *ovl, bool on)
{
	struct nx_mlc_reg *reg = ovl->base;
	int id = ovl->id;

	dev_dbg(ovl->dp->dev, "%s: %s, %s (%d)\n", __func__,
		ovl->name, on ? "on" : "off", ovl->enable);

	nx_mlc_set_layer_enable(reg, id, on);
	nx_mlc_set_layer_dirty(reg, id, true);

	if (!ovl->enable) {
		ovl->format = 0x0;
		ovl->pixelbyte = 0;
	}

	ovl->enable = on;
}

static int nx_overlay_yuv_set_format(struct nx_overlay *ovl,
				     unsigned int format, bool sync)
{
	struct nx_mlc_reg *reg = ovl->base;
	int lock_size = 16;

	ovl->format = format;
	format &= 0xffffff;

	dev_dbg(ovl->dp->dev,
		"%s: %s, format: 0x%x\n", __func__, ovl->name, format);

	nx_mlc_set_layer_lock_size(reg, NX_PLANE_VIDEO_LAYER, lock_size);
	nx_mlc_set_vid_format(reg, format);
	nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, sync);

	return 0;
}

static int nx_overlay_yuv_set_pos(struct nx_overlay *ovl,
				  int src_x, int src_y, int src_w, int src_h,
				  int dst_x, int dst_y, int dst_w, int dst_h,
				  bool sync)
{
	struct nx_display *dp = ovl->dp;
	struct nx_mlc_reg *reg = ovl->base;
	int sx, sy, ex, ey;
	int hf = 1, vf = 1;

	ovl->left = src_x;
	ovl->top = src_y;
	ovl->width = src_w;
	ovl->height = src_h;

	/*
	 * max scale size
	 * if ove scale size, fix max
	 */
	if (dst_w > VID_FILTER_MAX)
		dst_w = VID_FILTER_MAX;

	if (dst_h > VID_FILTER_MAX)
		dst_h = VID_FILTER_MAX;

	sx = dst_x, sy = dst_y;
	ex = dst_x + dst_w;
	ey = dst_y + dst_h;

	/* max rectangle */
	if (ex > VID_FILTER_MAX)
		ex = VID_FILTER_MAX;

	if (ey > VID_FILTER_MAX)
		ey = VID_FILTER_MAX;

	dev_dbg(ovl->dp->dev,
		"%s: %s, (%d, %d, %d, %d) to (%d, %d, %d, %d, %d, %d)\n",
		 __func__, ovl->name, src_x, src_y, src_w, src_h,
		 sx, sy, ex, ey, dst_w, dst_h);

	if (ex == 0 || ey == 0 ||
		(src_w == dst_w && src_h == dst_h))
		hf = 0, vf = 0;

	if (dp->video_scale_hf_max && src_w >= dp->video_scale_hf_max)
		hf = 0;

	if (dp->video_scale_vf_max && src_h >= dp->video_scale_vf_max)
		vf = 0;

	ovl->h_filter = hf;
	ovl->v_filter = vf;

	/* set scale and position */
	nx_mlc_set_vid_scale(reg, src_w, src_h, dst_w, dst_h, hf, hf, vf, vf);
	nx_mlc_set_layer_position(reg, NX_PLANE_VIDEO_LAYER,
			sx, sy, ex ? ex - 1 : ex, ey ? ey - 1 : ey);
	nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, sync);

	return 0;
}

static void nx_overlay_yuv_enb(struct nx_overlay *ovl, bool on)
{
	struct nx_mlc_reg *reg = ovl->base;
	int hl, hc, vl, vc;

	dev_dbg(ovl->dp->dev,
		"%s: %s, %s\n", __func__, ovl->name, on ? "on" : "off");

	if (on) {
		nx_mlc_set_vid_line_buffer_power(reg, true);
		nx_mlc_set_layer_enable(reg, NX_PLANE_VIDEO_LAYER, true);
		nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, true);
	} else {
		nx_mlc_set_layer_enable(reg, NX_PLANE_VIDEO_LAYER, false);
		nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, true);

		nx_mlc_get_vid_scale_filter(reg, &hl, &hc, &vl, &vc);
		if (hl | hc | vl | vc)
			nx_mlc_set_vid_scale_filter(reg, 0, 0, 0, 0);
		nx_mlc_set_vid_line_buffer_power(reg, false);
		nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, true);
	}

	if (!ovl->enable)
		ovl->format = 0x0;

	ovl->enable = on;
}

void nx_overlay_set_priority(struct nx_overlay *ovl,
			int priority)
{
	struct nx_display *dp = ovl->dp;
	struct nx_mlc_reg *reg = ovl->base;

	switch (priority) {
	case 0:
		priority = NX_MLC_PRIORITY_VIDEO_1ST;
		break;
	case 1:
		priority = NX_MLC_PRIORITY_VIDEO_2ND;
		break;
	case 2:
		priority = NX_MLC_PRIORITY_VIDEO_3RD;
		break;
	case 3:
		priority = NX_MLC_PRIORITY_VIDEO_4TH;
		break;
	default:
		dev_err(dp->dev,
			"Failed, not support video priority(0~3),(%d)\n",
			priority);
		return;
	}
	dev_dbg(dp->dev, "%s: crtc.%d, priority:%d\n",
		__func__, ovl->dp->module, priority);

	dp->video_priority = priority;

	nx_mlc_set_video_priority(reg, priority);
	nx_mlc_set_dirty(reg);
}

int nx_overlay_set_format(struct nx_overlay *ovl,
			  unsigned int format, int pixelbyte, bool sync)
{
	if (is_video_plane(ovl->type))
		return nx_overlay_yuv_set_format(ovl, format, sync);
	else
		return nx_overlay_rgb_set_format(ovl, format, pixelbyte, sync);
}

int nx_overlay_set_position(struct nx_overlay *ovl,
			    int sx, int sy, int sw, int sh,
			    int dx, int dy, int dw, int dh, bool sync)
{
	int ret;

	if (is_video_plane(ovl->type))
		ret = nx_overlay_yuv_set_pos(ovl,
				sx, sy, sw, sh, dx, dy, dw, dh, sync);
	else
		ret = nx_overlay_rgb_set_pos(ovl,
				sx, sy, sw, sh, dx, dy, dw, dh, sync);

	return ret;
}

void nx_overlay_enable(struct nx_overlay *ovl, bool on)
{
	if (is_video_plane(ovl->type))
		nx_overlay_yuv_enb(ovl, on);
	else
		nx_overlay_rgb_enb(ovl, on);
}

void nx_overlay_set_color(struct nx_overlay *ovl,
			  enum nx_overlay_color type, unsigned int color,
			  bool on, bool adjust)
{
	struct nx_mlc_reg *reg = ovl->base;
	int id = ovl->id;

	dev_dbg(ovl->dp->dev, "%s: %s, type:%d color:0x%x, pixel %d, %s\n",
		__func__, ovl->name, type, color, ovl->pixelbyte,
		on ? "on" : "off");

	switch (type) {
	case NX_COLOR_ALPHA:
		if (color <= 0)
			color = 0;
		if (color >= MAX_ALPHA_VALUE)
			color = MAX_ALPHA_VALUE;

		ovl->color.alpha = (on ? color : MAX_ALPHA_VALUE);
		nx_mlc_set_layer_alpha(reg, id, (u32)color, on);
		break;

	case NX_COLOR_TRANS:
		if (ovl->pixelbyte == 1) {
			color = rgb888_to_rgb332((u32)color);
			color = rgb332_to_rgb888((u8)color);
		}

		if (ovl->pixelbyte == 2) {
			color = rgb888_to_rgb565((u32)color);
			color = rgb565_to_rgb888((u16)color);
		}

		ovl->color.transcolor = (on ? color : 0);
		nx_mlc_set_rgb_transparency(reg, id, color, on);
		break;

	case NX_COLOR_INVERT:
		if (ovl->pixelbyte == 1) {
			color = rgb888_to_rgb332((u32)color);
			color = rgb332_to_rgb888((u8)color);
		}

		if (ovl->pixelbyte == 2) {
			color = rgb888_to_rgb565((u32)color);
			color = rgb565_to_rgb888((u16)color);
		}

		ovl->color.invertcolor = (on ? color : 0);
		nx_mlc_set_rgb_color_inv(reg, id, color, on);
		break;
	default:
		return;
	}

	nx_mlc_set_layer_dirty(reg, id, adjust);
}

void nx_overlay_set_addr_rgb(struct nx_overlay *ovl,
			     unsigned int addr, unsigned int pixelbyte,
			     unsigned int stride, int align, bool sync)
{
	struct nx_mlc_reg *reg = ovl->base;
	int id = ovl->id;
	int cl = ovl->left;
	int ct = ovl->top;
	unsigned int phys = addr + (cl * pixelbyte) + (ct * stride);

	if (align)
		phys = ALIGN(phys, align);

	dev_dbg(ovl->dp->dev,
		"%s: %s, pa:0x%x(0x%x), hs:%d, vs:%d, l:%d, t:%d\n",
		__func__, ovl->name, phys, addr, pixelbyte, stride, cl, ct);
	dev_dbg(ovl->dp->dev,
		"%s: %s, pa:0x%x -> 0x%x aligned %d\n",
		__func__, ovl->name, (addr + (cl * pixelbyte) + (ct * stride)),
		phys, align);

	nx_mlc_wait_vblank(reg, id);
	nx_mlc_set_rgb_stride(reg, id, pixelbyte, stride);
	nx_mlc_set_rgb_address(reg, id, phys);
	nx_mlc_set_layer_dirty(reg, id, sync);
}

void nx_overlay_set_addr_yuv(struct nx_overlay *ovl,
			     unsigned int lu_a, unsigned int lu_s,
			     unsigned int cb_a, unsigned int cb_s,
			     unsigned int cr_a, unsigned int cr_s, int planes,
			     bool sync)
{
	struct nx_mlc_reg *reg = ovl->base;
	int cl = ovl->left;
	int ct = ovl->top;
	int xo = cl, yo = ct;
	unsigned int format;

	/* yuyv */
	if (planes == 1) {
		if (cl % 2)
			dev_warn(ovl->dp->dev,
				"Clip X must be aligned with 2 for YUYV\n");

		lu_a += (cl * 2 + 1) + (ct * lu_s);
		nx_mlc_set_vid_address_yuyv(reg, lu_a, lu_s);
		nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, true);

		dev_dbg(ovl->dp->dev, "%s: %s, lu:0x%x,%d : %d,%d\n",
			__func__, ovl->name, lu_a, lu_s, cl, ct);

		return;
	}

	format = ovl->format & 0xffffff;

	switch (format) {
	case NX_MLC_FMT_VID_420:
		xo = cl/2, yo = ct/2;
		break;
	case NX_MLC_FMT_VID_422:
		xo = cl/2, yo = ct;
		break;
	case NX_MLC_FMT_VID_422_CBCR:
		xo = (cl/2) * 2, yo = ct;
		break;
	case NX_MLC_FMT_VID_420_CBCR:
		xo = (cl/2) * 2, yo = ct/2;
		break;
	case NX_MLC_FMT_VID_444:
		xo = cl, yo = ct;
		break;
	}

	lu_a += cl + (ct * lu_s);
	cb_a = cb_a + xo + (yo * cb_s);
	cr_a = cr_a + xo + (yo * cr_s);

	if (ovl->format & FMT_VID_YUV_TO_YVU)
		swap(cb_a, cr_a);

	dev_dbg(ovl->dp->dev,
		"%s: %s, lu:0x%x,%d, cb:0x%x,%d, cr:0x%x,%d : %d,%d\n",
		__func__, ovl->name, lu_a, lu_s, cb_a, cb_s, cr_a, cr_s,
		cl, ct);

	nx_mlc_wait_vblank(reg, NX_PLANE_VIDEO_LAYER);
	nx_mlc_set_vid_stride(reg, lu_s, cb_s, cr_s);
	nx_mlc_set_vid_address(reg, lu_a, cb_a, cr_a);
	nx_mlc_set_layer_dirty(reg, NX_PLANE_VIDEO_LAYER, sync);
}
