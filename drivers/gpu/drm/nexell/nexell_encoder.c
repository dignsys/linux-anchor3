// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <linux/of_address.h>

#include "nexell_drv.h"

static enum drm_mode_status nx_drm_encoder_mode_valid(
				struct drm_encoder *encoder,
				const struct drm_display_mode *mode)
{
	struct nx_drm_encoder *nx_encoder = to_nx_encoder(encoder);
	struct nx_drm_mode_max *max = &nx_encoder->max_mode;

	DRM_DEBUG_KMS("encoder id:%d\n", encoder->base.id);

	if (max->width && mode->hdisplay > max->width)
		return MODE_BAD;

	if (max->height && mode->vdisplay > max->height)
		return MODE_BAD;

	if (max->refresh && mode->vrefresh > max->refresh)
		return MODE_BAD;

	DRM_DEBUG_KMS("mode ok %d:%d:%d [%d:%d:%d]\n",
			mode->hdisplay, mode->vdisplay, mode->vrefresh,
			max->width, max->height, max->refresh);

	return MODE_OK;
}

static bool nx_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("encoder id:%d\n", encoder->base.id);
	return true;
}

static void nx_drm_encoder_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct drm_device *drm = encoder->dev;
	struct nx_drm_connector *nx_connector =
				to_nx_encoder(encoder)->connector;
	struct drm_connector *connector;

	/*
	 * Link Encoder to CRTC
	 */
	DRM_DEBUG_KMS("encoder id:%d crtc.%d:%p\n",
		encoder->base.id, to_nx_crtc(encoder->crtc)->pipe,
		encoder->crtc);

	list_for_each_entry(connector,
		&drm->mode_config.connector_list, head) {
		if (connector->encoder == encoder) {
			if (nx_connector && nx_connector->ops &&
				nx_connector->ops->set_mode)
				nx_connector->ops->set_mode(
					nx_connector->dev, adjusted_mode);
		}
	}
}

static void nx_drm_encoder_enable(struct drm_encoder *encoder)
{
	struct nx_drm_encoder *nx_encoder = to_nx_encoder(encoder);
	struct nx_drm_connector *nx_connector = nx_encoder->connector;
	bool is_connected = false;

	if (nx_connector && nx_connector->ops &&
		nx_connector->ops->is_connected)
		is_connected =
			nx_connector->ops->is_connected(nx_connector->dev);

	DRM_DEBUG_KMS("encoder id:%d %s, power %s\n",
		encoder->base.id, is_connected ? "connected" : "disconnected",
		nx_encoder->enabled ? "on" : "off");

	if (is_connected) {
		/*
		 * enable : encoder -> connector driver
		 */
		if (nx_encoder->ops && nx_encoder->ops->enable)
			nx_encoder->ops->enable(encoder);

		if (nx_connector && nx_connector->ops &&
			nx_connector->ops->enable)
			nx_connector->ops->enable(nx_connector->dev);

		nx_encoder->enabled = true;
	}
}

static void nx_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct nx_drm_encoder *nx_encoder = to_nx_encoder(encoder);
	struct nx_drm_connector *nx_connector = nx_encoder->connector;

	DRM_DEBUG_KMS("encoder id:%d power %s\n",
		encoder->base.id, nx_encoder->enabled ? "on" : "off");

	if (nx_encoder->enabled) {
		/*
		 * disable : connector driver -> encoder
		 */
		if (nx_connector && nx_connector->ops &&
			nx_connector->ops->disable)
			nx_connector->ops->disable(nx_connector->dev);

		if (nx_encoder->ops && nx_encoder->ops->disable)
			nx_encoder->ops->disable(encoder);

		nx_encoder->enabled = false;
	}
}

const static struct drm_encoder_helper_funcs nx_encoder_helper_funcs = {
	.mode_valid = nx_drm_encoder_mode_valid,
	.mode_fixup = nx_drm_encoder_mode_fixup,
	.mode_set = nx_drm_encoder_mode_set,
	.enable = nx_drm_encoder_enable,
	.disable = nx_drm_encoder_disable,
};

static void nx_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct nx_drm_encoder *nx_encoder = to_nx_encoder(encoder);

	DRM_DEBUG_KMS("enter\n");

	if (nx_encoder->ops && nx_encoder->ops->destroy)
		nx_encoder->ops->destroy(encoder);

	drm_encoder_cleanup(encoder);
	kfree(nx_encoder);
}

const static struct drm_encoder_funcs nx_encoder_funcs = {
	.destroy = nx_drm_encoder_destroy,
};

struct drm_encoder *nx_drm_encoder_create(struct drm_device *drm,
					  struct drm_connector *connector,
					  int enc_type,
					  int pipe, int possible_crtcs)
{
	struct nx_drm_encoder *nx_encoder;
	struct drm_encoder *encoder;

	DRM_DEBUG_KMS("pipe.%d crtc mask:0x%x\n", pipe, possible_crtcs);

	if (WARN_ON(possible_crtcs == 0))
		return ERR_PTR(-EINVAL);

	nx_encoder = kzalloc(sizeof(*nx_encoder), GFP_KERNEL);
	if (!nx_encoder)
		return ERR_PTR(-ENOMEM);

	if (connector)
		nx_encoder->connector = to_nx_connector(connector);

	encoder = &nx_encoder->encoder;
	encoder->possible_crtcs = possible_crtcs;

	nx_drm_encoder_init(encoder);
	drm_encoder_init(drm, encoder, &nx_encoder_funcs, enc_type, NULL);
	drm_encoder_helper_add(encoder, &nx_encoder_helper_funcs);

	return encoder;
}
