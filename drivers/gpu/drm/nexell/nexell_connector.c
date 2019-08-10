// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "nexell_drv.h"

static const char * const panel_type_names[] = {
	[NX_PANEL_TYPE_NONE] = "unknown",
	[NX_PANEL_TYPE_RGB]  = "RGB",
	[NX_PANEL_TYPE_LVDS] = "LVDS",
	[NX_PANEL_TYPE_MIPI] = "MIPI",
	[NX_PANEL_TYPE_HDMI] = "HDMI",
	[NX_PANEL_TYPE_TV] = "TV",
	[NX_PANEL_TYPE_CLUSTER_LCD] = "CLUSTER-LCD",
};

enum nx_panel_type nx_panel_get_type(struct nx_drm_display *display)
{
	return display->panel_type;
}

const char *nx_panel_get_name(enum nx_panel_type panel)
{
	return panel_type_names[panel];
}

static int nx_drm_connector_get_modes(struct drm_connector *connector)
{
	struct nx_drm_connector *nx_connector = to_nx_connector(connector);
	struct nx_drm_connect_drv_ops *ops = nx_connector->ops;

	DRM_DEBUG_KMS("enter\n");

	if (ops && ops->get_modes)
		return ops->get_modes(nx_connector->dev, connector);

	DRM_ERROR("Failed to create a new display mode.\n");
	return 0;
}

#define INTERLACE(f) (f & DRM_MODE_FLAG_INTERLACE ? true : false)

static int nx_drm_connector_mode_valid(struct drm_connector *connector,
				       struct drm_display_mode *mode)
{
	struct nx_drm_connector *nx_connector = to_nx_connector(connector);
	struct nx_drm_connect_drv_ops *ops = nx_connector->ops;

	DRM_DEBUG_KMS("bpp specified : %s (%d), mode:%dx%d, flags:0x%x [%s]\n",
		connector->cmdline_mode.bpp_specified ? "yes" : "no",
		connector->cmdline_mode.bpp,
		mode->hdisplay, mode->vdisplay, mode->flags,
		INTERLACE(mode->flags) ? "INTERACE" : "PROGRESSIVE");

	if (ops && ops->mode_valid)
		return ops->mode_valid(nx_connector->dev, mode);

	return MODE_BAD;
}

static struct drm_encoder *nx_drm_best_encoder(struct drm_connector *connector)
{
	struct nx_drm_connector *nx_connector = to_nx_connector(connector);
	struct drm_encoder *encoder = &nx_connector->encoder->encoder;

	WARN(!encoder, "Not connected encoder");
	DRM_DEBUG_KMS("encoodr id:%d connector id:%d\n",
		encoder->base.id, connector->base.id);

	return encoder;
}

static struct drm_connector_helper_funcs nx_drm_connector_helper_funcs = {
	.get_modes = nx_drm_connector_get_modes,
	.mode_valid = nx_drm_connector_mode_valid,
	.best_encoder = nx_drm_best_encoder,
};

static enum drm_connector_status nx_drm_connector_detect(
			struct drm_connector *connector, bool force)
{
	struct nx_drm_connector *nx_connector = to_nx_connector(connector);
	struct nx_drm_connect_drv_ops *ops = nx_connector->ops;
	enum drm_connector_status status = connector_status_disconnected;

	if (ops && ops->detect) {
		if (ops->detect(nx_connector->dev, connector))
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
	}

	DRM_DEBUG_KMS("connector id:%d status: %s\n",
		connector->base.id, status == connector_status_connected ?
		"connected" : "disconnected");

	return status;
}

static void nx_drm_connector_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("enter\n");
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

const static struct drm_connector_funcs nx_drm_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = nx_drm_connector_detect,
	.destroy = nx_drm_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

int nx_drm_connector_attach(struct drm_device *drm,
			    struct drm_connector *connector, int pipe,
			    unsigned int possible_crtcs,
			    enum nx_panel_type panel_type)
{
	struct nx_drm_private *private = drm->dev_private;
	struct drm_encoder *encoder;
	struct nx_drm_connector *nx_connector;
	struct nx_drm_encoder_ops *enc_ops;

	/* bitmask of potential CRTC bindings */
	int con_type = 0, enc_type = 0;
	bool interlace_allowed = false;
	uint8_t polled = 0;
	int err;

	/*
	 * if no possible crtcs, you can connect all crtcs.
	 */
	if (possible_crtcs == 0)
		possible_crtcs = (1 << private->num_crtcs) - 1;

	DRM_DEBUG_KMS("connect pipe.%d crtc mask:0x%x, crtcs:%d\n",
		pipe, possible_crtcs, private->num_crtcs);

	if (WARN_ON(!connector))
		return -EINVAL;

	switch (panel_type) {
	case NX_PANEL_TYPE_RGB:
		con_type = DRM_MODE_CONNECTOR_VGA;
		enc_type = DRM_MODE_ENCODER_TMDS;
		interlace_allowed = true;
		break;
	case NX_PANEL_TYPE_LVDS:
		con_type = DRM_MODE_CONNECTOR_LVDS;
		enc_type = DRM_MODE_ENCODER_LVDS;
		break;
	case NX_PANEL_TYPE_MIPI:	/* MiPi DSI */
		con_type = DRM_MODE_CONNECTOR_DSI;
		enc_type = DRM_MODE_ENCODER_DSI;
		break;
	case NX_PANEL_TYPE_HDMI:
		con_type = DRM_MODE_CONNECTOR_HDMIA;
		enc_type = DRM_MODE_ENCODER_TMDS;
		interlace_allowed = true;
		break;
	case NX_PANEL_TYPE_TV:
		con_type = DRM_MODE_CONNECTOR_TV;
		enc_type = DRM_MODE_ENCODER_TVDAC;
		interlace_allowed = true;
		break;
	case NX_PANEL_TYPE_CLUSTER_LCD:
		con_type = DRM_MODE_CONNECTOR_VGA;
		enc_type = DRM_MODE_ENCODER_TMDS;
		break;
	default:
		con_type = DRM_MODE_CONNECTOR_Unknown;
		DRM_ERROR("Failed, unknown drm connector type(%d)\n",
			panel_type);
		return -EINVAL;
	}
	polled = DRM_CONNECTOR_POLL_HPD;	/* for hpd_irq_event */

	connector->polled = polled;
	connector->interlace_allowed = interlace_allowed;
	connector->dpms = DRM_MODE_DPMS_OFF;

	/*
	 * create encoder
	 */
	encoder = nx_drm_encoder_create(drm, connector, enc_type,
					pipe, possible_crtcs);
	if (IS_ERR(encoder))
		return PTR_ERR(encoder);

	/*
	 * create connector and attach
	 */
	drm_connector_init(drm, connector, &nx_drm_connector_funcs, con_type);
	drm_connector_helper_add(connector, &nx_drm_connector_helper_funcs);
	err = drm_connector_register(connector);
	if (err)
		goto err_encoder;

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("Failed to attach a connector to a encoder\n");
		goto err_connector;
	}

	nx_connector = to_nx_connector(connector);
	nx_connector->encoder = to_nx_encoder(encoder);

	/* inititalize dpms status */
	enc_ops = to_nx_encoder(encoder)->ops;
	if (enc_ops && enc_ops->dpms_status)
		connector->dpms = enc_ops->dpms_status(encoder);

	DRM_DEBUG_KMS("connect %s, encoder id:%d, connector id:%d, dpms %s\n",
		connector->name, encoder->base.id, connector->base.id,
		connector->dpms == DRM_MODE_DPMS_ON ? "on" : "off");

	return err;

err_connector:
	drm_connector_unregister(connector);
err_encoder:
	drm_connector_cleanup(connector);
	if (encoder && encoder->funcs->destroy)
		encoder->funcs->destroy(encoder);

	return err;
}

void nx_drm_connector_detach(struct drm_connector *connector)
{
	struct drm_encoder *encoder = connector->encoder;

	if (encoder && encoder->funcs->destroy)
		encoder->funcs->destroy(encoder);

	if (connector->funcs && connector->funcs->destroy)
		connector->funcs->destroy(connector);
}
