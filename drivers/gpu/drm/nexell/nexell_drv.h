// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _NEXELL_DRM_DRV_H_
#define _NEXELL_DRM_DRV_H_

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <video/display_timing.h>
#include <video/videomode.h>

#define MAX_CRTCS		2
#define MAX_CONNECTOR		2
#define MAX_PLNAES		4

#define MAX_FB_MODE_WIDTH	4096
#define MAX_FB_MODE_HEIGHT	4096

/*
 * Nexell drm driver private structure.
 */
struct nx_drm_of_data {
	int max_crtcs;
	int max_connectors;
};

struct nx_drm_private {
	struct nx_drm_fb_helper *fb_helper;
	bool force_detect;
	int num_crtcs;
	struct nx_drm_of_data *data;
	struct drm_atomic_state *suspend_state;
};

/*
 * Nexell drm common crtc structure.
 */
struct nx_drm_crtc_ops {
	int (*atomic_check)(struct drm_crtc *crtc,
			struct drm_crtc_state *state);
	int (*begin)(struct drm_crtc *crtc);
	void (*enable)(struct drm_crtc *crtc);
	void (*disable)(struct drm_crtc *crtc);
	void (*flush)(struct drm_crtc *crtc);
	void (*destroy)(struct drm_crtc *crtc);
};

struct nx_drm_irq_ops {
	int (*irq_enable)(struct drm_crtc *crtc, int pipe);
	void (*irq_disable)(struct drm_crtc *crtc, int pipe);
	void (*irq_done)(struct drm_crtc *crtc, int pipe);
	u32 (*get_counter)(struct drm_crtc *crtc, int pipe);
};

struct nx_drm_crtc {
	struct drm_crtc crtc;
	void *context;
	struct nx_drm_crtc_ops *ops;
	int num_planes;
	unsigned int planes_type[MAX_PLNAES];
	int pipe;
	int irq;
	struct nx_drm_irq_ops *irq_ops;
	struct drm_pending_vblank_event *event;
};

#define to_nx_crtc(x)	container_of(x, struct nx_drm_crtc, crtc)

/*
 * Nexell drm common plane structure.
 */
#define	NX_PLANE_TYPE_OVERLAY	DRM_PLANE_TYPE_OVERLAY	/* 0 */
#define	NX_PLANE_TYPE_PRIMARY	DRM_PLANE_TYPE_PRIMARY	/* 1 */
#define	NX_PLANE_TYPE_CURSOR	DRM_PLANE_TYPE_CURSOR	/* 2 */
#define	NX_PLANE_TYPE_RGB	(0<<4)
#define	NX_PLANE_TYPE_VIDEO	(1<<4)
#define	NX_PLANE_TYPE_UNKNOWN	(0xFFFFFFF)

#define	is_video_plane(type)	(type & NX_PLANE_TYPE_VIDEO)

struct nx_drm_plane_ops {
	int (*atomic_check)(struct drm_plane *plane,
			struct drm_plane_state *state);
	int (*update)(struct drm_plane *plane,
			struct drm_framebuffer *fb,
			int crtc_x, int crtc_y, int crtc_w, int crtc_h,
			int src_x, int src_y, int src_w, int src_h);
	void (*disable)(struct drm_plane *plane);
	void (*destroy)(struct drm_plane *plane);
	void (*create_proeprties)(struct drm_device *drm,
			struct drm_crtc *crtc, struct drm_plane *plane,
			struct drm_plane_funcs *plane_funcs);
};

struct nx_drm_plane {
	struct drm_plane plane;
	void *context;
	struct nx_drm_plane_ops *ops;
	int index;
	const uint32_t *formats;
	int format_count;
	int align;
};

#define to_nx_plane(x)	container_of(x, struct nx_drm_plane, plane)

/*
 * Nexell drm common encoder structure.
 */
struct nx_drm_encoder_ops {
	int (*dpms_status)(struct drm_encoder *encoder);
	void (*enable)(struct drm_encoder *encoder);
	void (*disable)(struct drm_encoder *encoder);
	void (*destroy)(struct drm_encoder *encoder);
};

struct nx_drm_mode_max {
	u32 width;
	u32 height;
	int refresh;
};

struct nx_drm_encoder {
	struct drm_encoder encoder;
	struct nx_drm_encoder_ops *ops;
	struct nx_drm_connector *connector;
	struct nx_drm_mode_max max_mode;
	bool enabled;
};

#define to_nx_encoder(e) \
		container_of(e, struct nx_drm_encoder, encoder)

/*
 * Nexell drm common connector structure.
 */
enum nx_panel_type {
	NX_PANEL_TYPE_NONE,
	NX_PANEL_TYPE_RGB,
	NX_PANEL_TYPE_LVDS,
	NX_PANEL_TYPE_MIPI,
	NX_PANEL_TYPE_HDMI,
	NX_PANEL_TYPE_TV,
	NX_PANEL_TYPE_CLUSTER_LCD,
};

struct nx_drm_connect_drv_ops {
	bool (*detect)(struct device *dev,
			struct drm_connector *connector);
	bool (*is_connected)(struct device *dev);
	int (*get_modes)(struct device *dev, struct drm_connector *connector);
	int (*mode_valid)(struct device *dev, struct drm_display_mode *mode);
	void (*set_mode)(struct device *dev, struct drm_display_mode *mode);
	void (*set_sync)(struct device *dev, struct drm_display_mode *mode);
	void (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
};

struct nx_drm_connector {
	struct drm_connector connector;
	struct nx_drm_encoder *encoder;	/* hard linked encoder */
	struct device *dev;	/* display device */
	struct nx_drm_connect_drv_ops *ops;
	struct nx_drm_display *display;	/* for display control */
	unsigned int possible_crtcs;
};

#define to_nx_connector(c) \
		container_of(c, struct nx_drm_connector, connector)

/*
 * Nexell drm common display structure.
 */
struct nx_drm_hdmi_ops {
	u32 (*hpd_status)(struct nx_drm_display *display);
	int (*is_connected)(struct nx_drm_display *display);
	int (*is_valid)(struct nx_drm_display *display,
			struct drm_display_mode *mode);
	int (*get_mode)(struct nx_drm_display *display,
			struct drm_display_mode *mode);
	irqreturn_t (*hpd_irq_cb)(int irq, void *data);
	void *cb_data;
};

struct nx_drm_mipi_ops {
	int (*set_format)(struct nx_drm_display *display,
			struct mipi_dsi_device *device);
	int (*transfer)(struct nx_drm_display *display,
			struct mipi_dsi_host *host,
			const struct mipi_dsi_msg *msg);
};

struct nx_drm_display_ops {
	/* display device control */
	int (*open)(struct nx_drm_display *display, int pipe);
	void (*close)(struct nx_drm_display *display, int pipe);
	int (*prepare)(struct nx_drm_display *display);
	int (*unprepare)(struct nx_drm_display *display);
	int (*enable)(struct nx_drm_display *display);
	int (*disable)(struct nx_drm_display *display);
	/* display mode control */
	int (*set_mode)(struct nx_drm_display *display,
			struct drm_display_mode *mode, unsigned int flags);
	/* display device specific operations */
	struct nx_drm_mipi_ops *mipi;
	struct nx_drm_hdmi_ops *hdmi;
};

struct nx_drm_display {
	struct nx_drm_connector *connector;
	struct device_node *panel_node;
	struct drm_panel *panel;
	struct nx_drm_display_ops *ops;
	void *context; /* display specific data  */
	enum nx_panel_type panel_type;
	struct videomode vm;
	bool fixed_sync;
	int vrefresh;
	int width_mm;
	int height_mm;
	bool check_panel;
	int pipe;
	int irq;
	bool is_connected;
};

/* HPD events */
#define	HDMI_EVENT_PLUG		(1<<0)
#define	HDMI_EVENT_UNPLUG	(1<<1)
#define	HDMI_EVENT_HDCP		(1<<2)

enum nx_panel_type nx_panel_get_type(struct nx_drm_display *display);
const char *nx_panel_get_name(enum nx_panel_type panel);

int nx_drm_crtc_create(struct drm_device *drm, int max_crtcs);
int nx_drm_planes_create(struct drm_device *drm,
			struct drm_crtc *crtc, int pipe,
			struct drm_crtc_funcs *crtc_funcs);
bool nx_drm_plane_support_bgr(void);
int nx_drm_connector_attach(struct drm_device *drm,
			struct drm_connector *connector,
			int pipe, unsigned int possible_crtcs,
			enum nx_panel_type panel_type);
void nx_drm_connector_detach(struct drm_connector *connector);
struct drm_encoder *nx_drm_encoder_create(struct drm_device *drm,
			struct drm_connector *connector, int enc_type,
			int pipe, int possible_crtcs);

/*
 * Nexell drm SoC specific functions
 */
int nx_drm_crtc_init(struct drm_device *drm, struct drm_crtc *crtc, int pipe);
int nx_drm_planes_init(struct drm_device *drm, struct drm_crtc *crtc,
			struct drm_plane **planes, int num_planes,
			enum drm_plane_type *drm_types, bool bgr_mode);
int nx_drm_encoder_init(struct drm_encoder *encoder);
struct nx_drm_display *nx_drm_display_get(struct device *dev,
			struct device_node *node,
			struct drm_connector *connector,
			enum nx_panel_type type);
void nx_drm_display_put(struct device *dev, struct nx_drm_display *display);
int  nx_drm_display_setup(struct nx_drm_display *display,
			struct device_node *node, enum nx_panel_type type);

dma_addr_t nx_drm_get_dma_addr(struct drm_plane *plane);

/*
 * Nexell drm sub drivers
 */
struct nx_drm_subdrv {
	struct list_head list;
	struct device *dev;
	struct drm_device *drm;
	void *driver_data;
	int (*probe)(struct drm_device *drm, struct device *dev);
	void (*remove)(struct drm_device *drm, struct device *dev);
	int (*open)(struct drm_device *drm, struct device *dev,
			struct drm_file *file);
	void (*close)(struct drm_device *drm, struct device *dev,
			struct drm_file *file);
};

int nx_drm_subdrv_register(struct nx_drm_subdrv *subdrv, struct device *dev);
int nx_drm_subdrv_unregister(struct nx_drm_subdrv *subdrv);
int nx_drm_subdrv_probe(struct drm_device *drm);
int nx_drm_subdrv_remove(struct drm_device *drm);
int nx_drm_subdrv_open(struct drm_device *drm, struct drm_file *file);
void nx_drm_subdrv_close(struct drm_device *drm, struct drm_file *file);

/*
 * Nexell drm specific platform drivers
 */
extern struct platform_driver panel_lcd_driver;
extern struct platform_driver drm_g2d_driver;

#ifdef CONFIG_DRM_NEXELL_G2D
#include "nxp3220_g2d.h"
#endif

#endif

