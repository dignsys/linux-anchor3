// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>

#include "nexell_drv.h"
#include "nexell_gem.h"

static bool plane_format_bgr;
MODULE_PARM_DESC(fb_bgr, "display plane BGR pixel format");
module_param_named(plane_bgr, plane_format_bgr, bool, 0600);

bool nx_drm_plane_support_bgr(void)
{
	return plane_format_bgr;
}

static void nx_drm_plane_destroy(struct drm_plane *plane)
{
	struct nx_drm_plane *nx_plane = to_nx_plane(plane);
	struct nx_drm_plane_ops *ops = nx_plane->ops;

	if (ops && ops->destroy)
		ops->destroy(plane);

	drm_plane_cleanup(plane);
	kfree(nx_plane);
}

static struct drm_plane_funcs nx_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = nx_drm_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int nx_drm_plane_atomic_check(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	struct nx_drm_plane *nx_plane = to_nx_plane(plane);
	struct nx_drm_plane_ops *ops = nx_plane->ops;

	/* no fb link */
	if (!state->crtc || !state->fb)
		return 0;

	DRM_DEBUG_KMS("crtc.%d plane.%d\n",
		to_nx_crtc(state->crtc)->pipe, nx_plane->index);

	if (ops && ops->atomic_check)
		return ops->atomic_check(plane, state);

	return 0;
}

static void nx_drm_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct drm_crtc *crtc = state->crtc;
	struct nx_drm_plane *nx_plane = to_nx_plane(plane);
	struct nx_drm_plane_ops *ops = nx_plane->ops;

	if (!state->crtc || !state->fb)
		return;

	DRM_DEBUG_KMS("crtc.%d plane.%d\n",
		to_nx_crtc(crtc)->pipe, nx_plane->index);
	DRM_DEBUG_KMS("crtc.%d enable:%d, active:%d, changed:%d\n",
		to_nx_crtc(crtc)->pipe, crtc->state->enable,
		crtc->state->active, crtc->state->active_changed);

	if (!crtc->state->enable)
		return;

	if (ops && ops->update)
		ops->update(plane, state->fb, state->crtc_x, state->crtc_y,
			      state->crtc_w, state->crtc_h,
			      state->src_x >> 16, state->src_y >> 16,
			      state->src_w >> 16, state->src_h >> 16);
}

static void nx_drm_plane_atomic_disable(struct drm_plane *plane,
					struct drm_plane_state *old_state)
{
	struct nx_drm_plane *nx_plane = to_nx_plane(plane);
	struct nx_drm_plane_ops *ops = nx_plane->ops;

	if (!old_state || !old_state->crtc)
		return;

	DRM_DEBUG_KMS("crtc.%d plane.%d\n",
		to_nx_crtc(old_state->crtc)->pipe, nx_plane->index);

	if (ops && ops->disable)
		ops->disable(plane);
}

static const struct drm_plane_helper_funcs nx_plane_helper_funcs = {
	.atomic_check = nx_drm_plane_atomic_check,
	.atomic_update = nx_drm_plane_atomic_update,
	.atomic_disable = nx_drm_plane_atomic_disable,
};

int nx_drm_planes_create(struct drm_device *drm,
			struct drm_crtc *crtc, int pipe,
			struct drm_crtc_funcs *crtc_funcs)
{
	struct drm_plane *plane, *planes[MAX_PLNAES] = { NULL, };
	struct nx_drm_plane *nx_plane;
	int num_planes = to_nx_crtc(crtc)->num_planes;
	enum drm_plane_type drm_types[MAX_PLNAES];
	int count = 0, i = 0;
	int err = 0;

	for (i = 0; i < num_planes; i++) {
		nx_plane = kzalloc(sizeof(struct nx_drm_plane), GFP_KERNEL);
		if (!nx_plane)
			goto err_plane;

		planes[i] = &nx_plane->plane;
	}

	/* setup crtc's planes */
	err = nx_drm_planes_init(drm, crtc, planes, num_planes,
					drm_types, plane_format_bgr);
	if (err < 0)
		goto err_plane;

	/* reset planes */
	num_planes = err;

	/* register drm plane */
	for (i = 0; i < num_planes; i++, count++) {
		enum drm_plane_type drm_type = drm_types[i] & 0xF;
		struct nx_drm_plane_ops *ops;

		plane = planes[i];
		nx_plane = to_nx_plane(planes[i]);

		err = drm_universal_plane_init(drm, plane, (1 << pipe),
					&nx_plane_funcs, nx_plane->formats,
					nx_plane->format_count,
					NULL, drm_type, NULL);
		if (err < 0)
			goto err_plane;

		drm_plane_helper_add(plane, &nx_plane_helper_funcs);

		ops = nx_plane->ops;
		if (ops && ops->create_proeprties)
			ops->create_proeprties(drm,
					crtc, plane, &nx_plane_funcs);

		if (drm_type == DRM_PLANE_TYPE_PRIMARY) {
			err = drm_crtc_init_with_planes(drm, crtc,
					plane, NULL, crtc_funcs, NULL);
			if (err < 0)
				goto err_plane;
		}
		nx_plane->index = i;
	}

	return 0;

err_plane:
	for (i = 0; i < num_planes; i++) {
		if (planes[i]) {
			plane = planes[i];
			plane->funcs->destroy(plane);
		}
		kfree(to_nx_plane(planes[i]));
	}

	return err;
}
