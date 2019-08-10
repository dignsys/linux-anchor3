// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _NEXELL_DISPLAY_DEV_H_
#define _NEXELL_DISPLAY_DEV_H_

#include <drm/drmP.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/reset.h>

#include "../nexell_drv.h"
#include "low.h"

#define	NX_PLANE_VIDEO_LAYER	(3) /* Planes = 0,1 (RGB), 3 (VIDEO) */

struct nx_display_par {
	/* sync format */
	unsigned int out_format;
	int invert_field;	/* 0= Normal Field 1: Invert Field */
	int swap_rb;
	unsigned int yc_order;	/* for CCIR output */
	/* sync delay  */
	int delay_mask;		/* if not 0, set defalut delays */
	int d_rgb_pvd;		/* delay value RGB/PVD signal   , 0 ~ 16, 0 */
	int d_hsync_cp1;	/* delay value HSYNC/CP1 signal , 0 ~ 63, 12 */
	int d_vsync_frame;	/* delay value VSYNC/FRAM signal, 0 ~ 63, 12 */
	int d_de_cp2;		/* delay value DE/CP2 signal    , 0 ~ 63, 12 */
	/* interlace sync */
	int evfront_porch;
	int evback_porch;
	int evsync_len;
	int vstart_offs;
	int vend_offs;
	int evstart_offs;
	int evend_offs;
};

/* RGB/LVDS ...'s display context */
struct nx_display_ctx {
	struct nx_display *dp;
	struct videomode *vm;
	struct nx_display_par dpp;
	enum nx_panel_type panel_type;
	bool mpu_lcd;
};

enum nx_overlay_color {
	NX_COLOR_COLORKEY,
	NX_COLOR_ALPHA,
	NX_COLOR_BRIGHT,
	NX_COLOR_HUE,
	NX_COLOR_CONTRAST,
	NX_COLOR_SATURATION,
	NX_COLOR_GAMMA,
	NX_COLOR_TRANS,
	NX_COLOR_INVERT,
};

struct nx_overlay_prop {
	union color {
		struct {
			struct drm_property *transcolor;
			struct drm_property *alphablend;
			struct drm_property *colorkey;
			struct drm_property *transcolor_on;
			struct drm_property *alphablend_on;
			struct drm_property *colorkey_on;
		} rgb;
	} color;
	struct drm_property *priority;
};

#define FMT_VID_YUV_TO_YVU	(1<<31)

struct nx_overlay {
	struct nx_display *dp;
	void __iomem *base;
	int id;
	char name[16];
	struct list_head list;
	unsigned int type;
	unsigned int format;
	bool bgr_mode;
	int left;
	int top;
	int width;
	int height;
	int pixelbyte;
	int stride;
	unsigned int h_filter;
	unsigned int v_filter;

	/* color control */
	bool alphablend_on;
	bool transcolor_on;
	bool colorkey_on;

	union {
		/* RGB */
		struct {
			int alphablend; /* 0: transparency, 15: opacity */
			u32 transcolor;
			u32 invertcolor;
			u32 colorkey;
		};
		/* VIDEO */
		struct {
			int alpha; /* def= 255, 0 <= Range <= 15 */
			int bright; /* def= 0, -128 <= Range <= 128*/
			int contrast; /* def= 0, 0 <= Range <= 8 */
			double hue; /* def= 0, 0 <= Range <= 360 */
			double saturation; /* def = 0, -100 <= Range <= 100 */
			int satura;
			int gamma;
		};
	} color;
	struct nx_overlay_prop property;
	bool enable;
};

struct nx_display {
	struct device *dev;
	struct drm_crtc *crtc;
	void __iomem *mlc_base;
	void __iomem *dpc_base;
	struct clk *clk_axi;
	struct clk *clk_x1, *clk_x2;
	int irq;
	struct list_head overlay_list;
	unsigned int overlay_types[3]; /* RGB 2Ea, Video 1Ea */
	int num_overlays;
	int module;
	int width;
	int height;
	int video_priority;	/* 0: video>RGBn, 1: RGB0>video>RGB1, */
				/* 2: RGB0 > RGB1 > vidoe .. */
	int video_scale_hf_max;
	int video_scale_vf_max;

	unsigned int back_color;
	unsigned int color_key;
	bool color_key_on;
	bool alpla_blend_on;
	bool boot_on;
	struct videomode vm;
	struct nx_display_par *dpp; /* get from display ctx */
	enum nx_panel_type panel_type;
	bool mpu_lcd;
};

/*
 * Nexell drm display specific interfaces.
 */
void *nx_drm_display_lvds_get(struct device *dev,
			struct device_node *node,
			struct nx_drm_display *display);
void *nx_drm_display_rgb_get(struct device *dev,
			struct device_node *node,
			struct nx_drm_display *display);

/*
 * Nexell drm display SoC interfaces.
 */
int  nx_display_set_mode(struct nx_display *dp);
void nx_display_set_format(struct nx_display *dp, int width, int height);
void nx_display_set_backcolor(struct nx_display *dp);
void nx_display_enable(struct nx_display *dp, bool on);
void nx_display_irq_on(struct nx_display *dp, bool on);
void nx_display_irq_clear(struct nx_display *dp);
void nx_display_ovl_enable(struct nx_display *dp);
void nx_display_ovl_disable(struct nx_display *dp);

void nx_overlay_set_color(struct nx_overlay *ovl,
			enum nx_overlay_color type, unsigned int color,
			bool on, bool adjust);
int nx_overlay_set_format(struct nx_overlay *ovl,
			unsigned int format, int pixelbyte, bool sync);
int nx_overlay_set_position(struct nx_overlay *ovl,
			int sx, int sy, int sw, int sh,
			int dx, int dy, int dw, int dh, bool sync);
void nx_overlay_set_priority(struct nx_overlay *ovl, int priority);
void nx_overlay_enable(struct nx_overlay *ovl, bool on);
void nx_overlay_set_addr_rgb(struct nx_overlay *ovl,
			unsigned int paddr, unsigned int pixelbyte,
			unsigned int stride, int align, bool sync);
void nx_overlay_set_addr_yuv(struct nx_overlay *ovl,
			unsigned int lu_a, unsigned int lu_s,
			unsigned int cb_a, unsigned int cb_s,
			unsigned int cr_a, unsigned int cr_s, int planes,
			bool sync);

#endif
