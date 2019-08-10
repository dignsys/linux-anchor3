// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _NEXELL_DRM_FB_H_
#define _NEXELL_DRM_FB_H_

#include <drm/drm_fb_helper.h>
#include <linux/fb.h>

struct nx_drm_fb {
	struct drm_framebuffer fb;
	struct nx_gem_object *obj[4];
};

struct nx_drm_fb_helper {
	struct drm_fb_helper fb_helper;
	struct nx_drm_fb *fb;
	int fb_buffers;
};

static inline struct nx_drm_fb_helper *to_nx_drm_fb_helper(
			struct drm_fb_helper *helper)
{
	return container_of(helper, struct nx_drm_fb_helper, fb_helper);
}

static inline struct nx_drm_fb *to_nx_drm_fb(struct drm_framebuffer *fb)
{
	return container_of(fb, struct nx_drm_fb, fb);
}

int  nx_drm_fb_helper_init(struct drm_device *dev);
void nx_drm_fb_helper_exit(struct drm_device *dev);

struct drm_framebuffer *nx_drm_mode_fb_create(struct drm_device *dev,
			struct drm_file *file_priv,
			const struct drm_mode_fb_cmd2 *mode_cmd);

#endif

