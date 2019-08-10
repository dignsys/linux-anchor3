/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: junghyun, kim <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>

#include <linux/backlight.h>
#include <linux/component.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <video/of_display_timing.h>
#include <dt-bindings/gpio/gpio.h>
#include <drm/nexell_drm.h>

#include "nexell_drv.h"
#include "nexell_fb.h"

struct mipi_resource {
	struct mipi_dsi_host mipi_host;
	struct mipi_dsi_device *mipi_dev;
	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

struct lcd_context {
	struct nx_drm_connector connector;
	/* lcd parameters */
	bool display_timing;
	struct mipi_resource mipi;
	struct gpio_descs *enable_gpios;
	enum of_gpio_flags gpios_active[4];
	int gpios_delay[4];
	struct backlight_device *backlight;
	int backlight_delay;
	bool enabled;
	struct work_struct lcd_power_work;
	struct nx_lcd_ops *lcd_ops;
	/* properties */
	int crtc_pipe;
	unsigned int possible_crtcs_mask;
	struct drm_bridge *bridge;
	struct nx_drm_mode_max max_mode;
};

#define ctx_to_display(c)	\
		((struct nx_drm_display *)(c->connector.display))

static bool panel_lcd_ops_detect(struct device *dev,
				 struct drm_connector *connector)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;
	enum nx_panel_type type = nx_panel_get_type(display);
	int ret;

	DRM_DEBUG_KMS("%s panel remote node: %s\n",
		nx_panel_get_name(type),
		display->panel_node ? "ok" : "none");

	if (display->panel_node) {
		struct drm_panel *drm_panel =
				of_drm_find_panel(display->panel_node);

		/*
		 * builded with module (.ko file).
		 */
		if (!drm_panel) {
			DRM_DEBUG_KMS("Not find panel driver for %s ...\n",
				nx_panel_get_name(type));
			return false;
		}

		display->panel = drm_panel;
		drm_panel_attach(drm_panel, connector);

		if (display->check_panel)
			return display->is_connected;

		if (ops && ops->prepare)
			ops->prepare(display);

		ret = drm_panel_prepare(drm_panel);
		if (!ret) {
			drm_panel_unprepare(drm_panel);
			if (ops && ops->unprepare)
				ops->unprepare(display);

			display->is_connected = true;
		} else {
			drm_panel_detach(drm_panel);
			display->is_connected = false;
		}
		display->check_panel = true;

		DRM_INFO("%s: check panel %s\n", nx_panel_get_name(type),
			display->is_connected ? "connected" : "disconnected");

		return display->is_connected;
	}

	if (!display->panel_node && !ctx->bridge && !ctx->display_timing) {
		DRM_DEBUG_DRIVER("not exist %s panel & timing %s !\n",
			nx_panel_get_name(type), dev_name(dev));
		return false;
	}

	/*
	 * support DT's timing node
	 * when not use panel driver
	 */
	display->is_connected = true;

	return true;
}

static bool panel_lcd_ops_is_connected(struct device *dev)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);

	return display->is_connected;
}

#define INTERLACE(f) (f & DRM_MODE_FLAG_INTERLACE ? true : false)

static int panel_lcd_ops_get_modes(struct device *dev,
				   struct drm_connector *connector)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct videomode *vm = &display->vm;
	struct drm_display_mode *mode;
	u32 hto, vto;

	DRM_DEBUG_KMS("get mode: bridge %s, panel %s\n",
		ctx->bridge ? "attached" : "detached",
		display->panel ? "attached" : "detached");

	if (ctx->bridge && !ctx->display_timing)
		return 0;

	if (display->panel)
		return drm_panel_get_modes(display->panel);

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("Failed to create a new display mode !\n");
		return 0;
	}

	drm_display_mode_from_videomode(vm, mode);

	hto = vm->hactive + vm->hfront_porch + vm->hback_porch + vm->hsync_len;
	vto = vm->vactive + vm->vfront_porch + vm->vback_porch + vm->vsync_len;

	mode->width_mm = display->width_mm;
	mode->height_mm = display->height_mm;
	mode->vrefresh = vm->pixelclock / (hto * vto);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	DRM_DEBUG_KMS("mode:%dx%d, flags:0x%x [%s]\n",
		mode->hdisplay, mode->vdisplay, mode->flags,
		INTERLACE(mode->flags) ? "INTERACE" : "PROGRESSIVE");

	return 1;
}

static int panel_lcd_ops_mode_valid(struct device *dev,
				    struct drm_display_mode *mode)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);

	drm_display_mode_to_videomode(mode, &display->vm);

	display->fixed_sync = true;
	display->width_mm = mode->width_mm;
	display->height_mm = mode->height_mm;

	DRM_DEBUG_KMS("valid mode:%dx%d, flags:0x%x [%s]\n",
		mode->hdisplay, mode->vdisplay, mode->flags,
		INTERLACE(mode->flags) ? "INTERACE" : "PROGRESSIVE");

	return MODE_OK;
}

static void panel_lcd_ops_set_mode(struct device *dev,
				   struct drm_display_mode *mode)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	DRM_DEBUG_KMS("enter\n");

	if (ops && ops->set_mode)
		ops->set_mode(display, mode, 0);
}

static void panel_lcd_work(struct work_struct *work)
{
	struct lcd_context *ctx = container_of(work,
			struct lcd_context, lcd_power_work);
	struct gpio_desc **desc;
	int i;

	if (ctx->enable_gpios) {
		desc = ctx->enable_gpios->desc;
		for (i = 0; i < ctx->enable_gpios->ndescs; i++) {
			DRM_DEBUG_KMS("LCD gpio.%d ative %s %dms\n",
				desc_to_gpio(desc[i]),
				ctx->gpios_active[i] == GPIO_ACTIVE_HIGH
				? "high" : "low ", ctx->gpios_delay[i]);

			/*
			 * If the flag is GPIO_ACTIVE_LOW,
			 * it automatically applies the inverse value.
			 */
			gpiod_set_value_cansleep(desc[i], 1);
			if (ctx->gpios_delay[i])
				mdelay(ctx->gpios_delay[i]);
		}
	}

	if (ctx->backlight) {
		if (ctx->backlight->props.power == FB_BLANK_UNBLANK)
			return;

		if (ctx->backlight_delay)
			msleep(ctx->backlight_delay);

		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
}

static void panel_lcd_on(struct lcd_context *ctx)
{
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	DRM_DEBUG_KMS("enter\n");

	if (ops && ops->prepare)
		ops->prepare(display);

	if (display->panel)
		drm_panel_prepare(display->panel);

	if (display->panel)
		drm_panel_enable(display->panel);

	/* last enable display to prevent LCD fliker */
	if (ops && ops->enable)
		ops->enable(display);
}

static void panel_lcd_off(struct lcd_context *ctx)
{
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	DRM_DEBUG_KMS("enter\n");

	if (display->panel)
		drm_panel_unprepare(display->panel);

	if (display->panel)
		drm_panel_disable(display->panel);

	if (ops && ops->unprepare)
		ops->unprepare(display);

	if (ops && ops->disable)
		ops->disable(display);
}

static void panel_lcd_ops_enable(struct device *dev)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	enum nx_panel_type type = nx_panel_get_type(display);

	DRM_DEBUG_KMS("%s\n", nx_panel_get_name(type));
	if (ctx->enabled)
		return;

	panel_lcd_on(ctx);

	schedule_work(&ctx->lcd_power_work);
	ctx->enabled = true;
}

static void panel_lcd_ops_disable(struct device *dev)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	enum nx_panel_type type = nx_panel_get_type(display);
	struct gpio_desc **desc;
	int i;

	DRM_DEBUG_KMS("%s\n", nx_panel_get_name(type));
	if (!ctx->enabled)
		return;

	cancel_work_sync(&ctx->lcd_power_work);

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	panel_lcd_off(ctx);

	if (ctx->enable_gpios) {
		desc = ctx->enable_gpios->desc;
		for (i = 0; i < ctx->enable_gpios->ndescs; i++)
			/*
			 * If the flag is GPIO_ACTIVE_LOW,
			 * it automatically applies the inverse value.
			 */
			gpiod_set_value_cansleep(desc[i], 0);
	}
	ctx->enabled = false;
}

static int panel_mipi_attach(struct mipi_dsi_host *host,
			     struct mipi_dsi_device *device)
{
	struct mipi_resource *mipi =
			container_of(host, struct mipi_resource, mipi_host);
	struct lcd_context *ctx = container_of(mipi, struct lcd_context, mipi);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct drm_connector *connector = &ctx->connector.connector;

	mipi->lanes = device->lanes;
	mipi->format = device->format;
	mipi->flags = device->mode_flags;
	mipi->mipi_dev = device;

	/* set mipi panel node */
	display->panel_node = device->dev.of_node;

	if (display->ops && display->ops->mipi) {
		struct nx_drm_mipi_ops *mipi_ops = display->ops->mipi;

		if (mipi_ops && mipi_ops->set_format)
			mipi_ops->set_format(display, device);
	}

	if (connector->dev)
		drm_helper_hpd_irq_event(connector->dev);

	DRM_INFO("mipi: %s lanes:%d, format:%d, flags:%lx\n",
		dev_name(&device->dev), device->lanes,
		device->format, device->mode_flags);

	return 0;
}

static int panel_mipi_detach(struct mipi_dsi_host *host,
			     struct mipi_dsi_device *device)
{
	struct mipi_resource *mipi =
			container_of(host, struct mipi_resource, mipi_host);
	struct lcd_context *ctx = container_of(mipi, struct lcd_context, mipi);
	struct nx_drm_display *display = ctx_to_display(ctx);

	DRM_DEBUG_KMS("enter\n");

	display->panel_node = NULL;
	display->panel = NULL;

	return 0;
}

static ssize_t panel_mipi_transfer(struct mipi_dsi_host *host,
				   const struct mipi_dsi_msg *msg)
{
	struct mipi_resource *mipi =
			container_of(host, struct mipi_resource, mipi_host);
	struct lcd_context *ctx = container_of(mipi, struct lcd_context, mipi);
	struct nx_drm_display *display = ctx_to_display(ctx);

	if (display->ops && display->ops->mipi) {
		struct nx_drm_mipi_ops *mipi_ops = display->ops->mipi;

		if (mipi_ops && mipi_ops->transfer)
			return mipi_ops->transfer(display, host, msg);
	}

	return -1;
}

static struct mipi_dsi_host_ops panel_mipi_ops = {
	.attach = panel_mipi_attach,
	.detach = panel_mipi_detach,
	.transfer = panel_mipi_transfer,
};

static struct nx_drm_connect_drv_ops lcd_connector_ops = {
	.detect = panel_lcd_ops_detect,
	.is_connected = panel_lcd_ops_is_connected,
	.get_modes = panel_lcd_ops_get_modes,
	.mode_valid = panel_lcd_ops_mode_valid,
	.set_mode = panel_lcd_ops_set_mode,
	.enable = panel_lcd_ops_enable,
	.disable = panel_lcd_ops_disable,
};

static int panel_lcd_bind_bridge(struct drm_device *drm,
				 struct lcd_context *ctx)
{
	struct drm_encoder *encoder;
	struct nx_drm_private *private = drm->dev_private;
	struct drm_connector *connector = &ctx->connector.connector;
	unsigned int possible_crtcs = ctx->possible_crtcs_mask;
	int err;

	if (possible_crtcs == 0)
		possible_crtcs = (1 << private->num_crtcs) - 1;

	encoder = nx_drm_encoder_create(drm, connector,
			DRM_MODE_ENCODER_TMDS, ctx->crtc_pipe, possible_crtcs);
	if (!encoder)
		return -ENOMEM;

	err = drm_bridge_attach(encoder, ctx->bridge, NULL);
	if (err)
		DRM_ERROR("Failed to attach bridge to drm\n");

	memcpy(&to_nx_encoder(encoder)->max_mode,
		&ctx->max_mode, sizeof(struct nx_drm_mode_max));

	return err;
}

static int panel_lcd_bind(struct device *dev,
			  struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_private *private = drm->dev_private;
	struct drm_connector *connector = &ctx->connector.connector;
	enum nx_panel_type type = nx_panel_get_type(ctx_to_display(ctx));
	struct platform_driver *drv;
	int err = 0;

	DRM_INFO("Bind %s panel\n", nx_panel_get_name(type));

	if (display->panel_node || ctx->display_timing) {
		err = nx_drm_connector_attach(drm, connector,
				ctx->crtc_pipe, ctx->possible_crtcs_mask,
				type);
		if (err)
			goto err_bind;

		memcpy(&ctx->connector.encoder->max_mode,
			&ctx->max_mode, sizeof(struct nx_drm_mode_max));
	}

	if (ctx->bridge) {
		if (panel_lcd_bind_bridge(drm, ctx))
			goto err_bind;
	}

	if (IS_ENABLED(CONFIG_DRM_NEXELL_MIPI_DSI) &&
	    type == NX_PANEL_TYPE_MIPI)
		err = mipi_dsi_host_register(&ctx->mipi.mipi_host);

	if (!err) {
		if (panel_lcd_ops_detect(dev, connector))
			private->force_detect = true;
		return 0;
	}

err_bind:
	drv = to_platform_driver(dev->driver);
	if (drv->remove)
		drv->remove(to_platform_device(dev));

	return err;
}

static void panel_lcd_unbind(struct device *dev,
			     struct device *master, void *data)
{
	struct lcd_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct drm_connector *connector = &ctx->connector.connector;
	struct drm_encoder *encoder;

	if (IS_ENABLED(CONFIG_DRM_NEXELL_MIPI_DSI)) {
		enum nx_panel_type type =
			nx_panel_get_type(ctx_to_display(ctx));

		if (type == NX_PANEL_TYPE_MIPI) {
			struct mipi_resource *mipi =  &ctx->mipi;

			mipi_dsi_host_unregister(&mipi->mipi_host);
		}
	}

	if (display->panel)
		drm_panel_detach(display->panel);

	if (connector)
		nx_drm_connector_detach(connector);

	if (ctx->bridge) {
		encoder = ctx->bridge->encoder;
		if (encoder)
			encoder->funcs->destroy(encoder);
	}
}

static const struct component_ops panel_comp_ops = {
	.bind = panel_lcd_bind,
	.unbind = panel_lcd_unbind,
};

static int panel_lcd_parse_gpio(struct device *dev, struct lcd_context *ctx)
{
	struct device_node *node = dev->of_node;
	struct gpio_descs *gpios;
	struct gpio_desc **desc = NULL;
	int i, ngpios = 0;

	/* parse panel gpios */
	gpios = devm_gpiod_get_array(dev, "enable", GPIOD_ASIS);
	if (-EBUSY == (long)ERR_CAST(gpios)) {
		DRM_INFO("Failed, enable-gpios is busy : %s !!!\n",
			node->full_name);
		gpios = NULL;
	}

	if (!IS_ERR(gpios) && gpios) {
		ngpios = gpios->ndescs;
		desc = gpios->desc;
		ctx->enable_gpios = gpios;	/* set enable_gpios */
		of_property_read_u32_array(node,
			"enable-gpios-delay", ctx->gpios_delay,
			(ngpios-1));
	}

	for (i = 0; i < ngpios; i++) {
		enum of_gpio_flags flags;
		int gpio;

		gpio = of_get_named_gpio_flags(node,
					"enable-gpios", i, &flags);
		if (!gpio_is_valid(gpio)) {
			DRM_ERROR("invalid gpio #%d: %d\n", i, gpio);
			return -EINVAL;
		}

		ctx->gpios_active[i] = flags;
#ifndef CONFIG_DRM_PRE_INIT_DRM
		/*
		 * disable at boottime:
		 * If the flag is GPIO_ACTIVE_LOW,
		 * it automatically applies the inverse value.
		 */
		gpiod_direction_output(desc[i], 0);

		DRM_INFO("LCD enable-gpio.%d act %s\n",
			 gpio, flags == GPIO_ACTIVE_HIGH ?
			 "high" : "low ");
#endif
	}

	return 0;
}

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
static void panel_lcd_parse_backlight(struct device *dev,
				      struct lcd_context *ctx)
{
	struct device_node *node = dev->of_node;
	struct device_node *np;

	np = of_parse_phandle(node, "backlight", 0);
	if (!np)
		return;

	ctx->backlight = of_find_backlight_by_node(np);
	of_node_put(np);

	if (ctx->backlight) {
		of_property_read_u32(node,
				"backlight-delay", &ctx->backlight_delay);
#ifndef CONFIG_DRM_PRE_INIT_DRM
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);

		DRM_INFO("LCD backlight %s, delay:%d\n",
			ctx->backlight->props.power == FB_BLANK_UNBLANK ?
			"on" : "off", ctx->backlight_delay);
#endif
	}
}
#endif

static int panel_lcd_parse_dt(struct platform_device *pdev,
			      struct lcd_context *ctx)
{
	struct device *dev = &pdev->dev;
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct device_node *node = dev->of_node;
	struct device_node *np;
	struct display_timing timing;
	struct drm_panel *panel = NULL;
	struct drm_bridge *bridge = NULL;
	int err;

	DRM_DEBUG_KMS("enter\n");

	/* get crtcs params */
	of_property_read_u32(node, "crtc-pipe", &ctx->crtc_pipe);
	of_property_read_u32(node,
			"crtcs-possible-mask", &ctx->possible_crtcs_mask);

	err = drm_of_find_panel_or_bridge(node, 0, 0, &panel, &bridge);
	if (!err) {
		ctx->bridge = bridge;
		if (panel)
			display->panel_node = panel->dev->of_node;
	}

	/* get panel timing from local. */
	if (!panel) {
		err = panel_lcd_parse_gpio(dev, ctx);
		if (err)
			return err;

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
		panel_lcd_parse_backlight(dev, ctx);
#endif
		/* parse panel lcd size */
		of_property_read_u32(node, "width-mm", &display->width_mm);
		of_property_read_u32(node, "height-mm", &display->height_mm);

		/*
		 * parse display timing (sync)
		 * refer to "drivers/video/of_display_timing.c"
		 * -> of_parse_display_timing
		 */
		err = of_get_display_timing(node, "display-timing", &timing);
		if (!err) {
			videomode_from_timing(&timing, &display->vm);
			ctx->display_timing = true;
		}
	}

	np = of_get_child_by_name(node, "max-resolution");
	if (np) {
		of_property_read_u32(np, "width", &ctx->max_mode.width);
		of_property_read_u32(np, "height", &ctx->max_mode.height);
		of_property_read_u32(np, "refresh", &ctx->max_mode.refresh);
		of_node_put(np);
	}

	DRM_INFO("Panel: %s, Bridge: %s\n",
		panel ? dev_name(panel->dev) :
		ctx->display_timing ? "display-timing" : "None",
		bridge ? "Connected" : "Disconnected");

	if (!display->panel_node && !ctx->bridge && !ctx->display_timing) {
		DRM_ERROR("Failed, not find panel/bridge control node (%s) !\n",
			node->full_name);
		return -ENOENT;
	}

	INIT_WORK(&ctx->lcd_power_work, panel_lcd_work);

	/* parse display control config */
	np = of_get_child_by_name(node, "display-config");
	if (np) {
		err = nx_drm_display_setup(display, np, display->panel_type);
		of_node_put(np);

		return err;
	}

	return 0;
}

static const struct of_device_id panel_lcd_of_match[];

static int panel_lcd_get_display(struct platform_device *pdev,
				 struct lcd_context *ctx)
{
	struct device *dev = &pdev->dev;
	struct nx_drm_connector *nx_connector = &ctx->connector;
	struct drm_connector *connector = &nx_connector->connector;
	const struct of_device_id *id;
	enum nx_panel_type type;

	/* get panel type with of id */
	id = of_match_node(panel_lcd_of_match, dev->of_node);
	type = (enum nx_panel_type)id->data;

	DRM_INFO("Load %s panel\n", nx_panel_get_name(type));

	/* get display for RGB/LVDS/MiPi-DSI */
	nx_connector->display = nx_drm_display_get(
					dev, dev->of_node, connector, type);
	if (!nx_connector->display)
		return -ENOENT;

	return 0;
}

static int panel_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcd_context *ctx;
	struct nx_drm_display_ops *ops;
	enum nx_panel_type type;
	int err;

	DRM_DEBUG_KMS("enter (%s)\n", dev_name(dev));

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->connector.dev = dev;
	ctx->connector.ops = &lcd_connector_ops;

	err = panel_lcd_get_display(pdev, ctx);
	if (err < 0)
		goto err_probe;

	ops = ctx_to_display(ctx)->ops;
	if (ops && ops->open) {
		err = ops->open(ctx_to_display(ctx), ctx->crtc_pipe);
		if (err)
			goto err_probe;
	}

	err = panel_lcd_parse_dt(pdev, ctx);
	if (err < 0)
		goto err_probe;

	type = nx_panel_get_type(ctx_to_display(ctx));

	if (IS_ENABLED(CONFIG_DRM_NEXELL_MIPI_DSI)) {
		if (type == NX_PANEL_TYPE_MIPI) {
			ctx->mipi.mipi_host.ops = &panel_mipi_ops;
			ctx->mipi.mipi_host.dev = dev;
		}
	}

	dev_set_drvdata(dev, ctx);
	component_add(dev, &panel_comp_ops);

	DRM_DEBUG_KMS("done\n");
	return err;

err_probe:
	DRM_ERROR("Failed %s probe !!!\n", dev_name(dev));

	if (ctx && ctx->backlight)
		put_device(&ctx->backlight->dev);

	return err;
}

static int panel_lcd_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcd_context *ctx = dev_get_drvdata(&pdev->dev);
	struct nx_drm_display_ops *ops;
	int i = 0;

	DRM_DEBUG_KMS("enter (%s)\n", dev_name(dev));
	if (!ctx)
		return 0;

	component_del(dev, &panel_comp_ops);

	ops = ctx_to_display(ctx)->ops;
	if (ops && ops->close)
		ops->close(ctx_to_display(ctx), ctx->crtc_pipe);

	nx_drm_display_put(dev, ctx_to_display(ctx));

	if (ctx->enable_gpios) {
		struct gpio_desc **desc = ctx->enable_gpios->desc;

		for (i = 0; i < ctx->enable_gpios->ndescs; i++)
			devm_gpiod_put(dev, desc[i]);
	}

	if (ctx->backlight)
		put_device(&ctx->backlight->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int panel_lcd_suspend(struct device *dev)
{
	return 0;
}

static int panel_lcd_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops panel_lcd_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(panel_lcd_suspend, panel_lcd_resume)
};

static const struct of_device_id panel_lcd_of_match[] = {
	{
		.compatible = "nexell,nxp3220-drm-rgb",
		.data = (void *)NX_PANEL_TYPE_RGB
	},
	{
		.compatible = "nexell,nxp3220-drm-lvds",
		.data = (void *)NX_PANEL_TYPE_LVDS
	},
	{}
};
MODULE_DEVICE_TABLE(of, panel_lcd_of_match);

struct platform_driver panel_lcd_driver = {
	.probe = panel_lcd_probe,
	.remove = panel_lcd_remove,
	.driver = {
		.name = "nexell,display_drm_lcd",
		.owner = THIS_MODULE,
		.of_match_table = panel_lcd_of_match,
		.pm = &panel_lcd_pm,
	},
};
