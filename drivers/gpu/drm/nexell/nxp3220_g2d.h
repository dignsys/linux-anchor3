// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell G2D driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _DISPLAY_G2D_H_
#define _DISPLAY_G2D_H_

#define NX_G2D_DRIVER_VER_MAJOR		1
#define NX_G2D_DRIVER_VER_MINOR		1

int nx_drm_g2d_get_version(struct drm_device *drm_dev, void *data,
			   struct drm_file *file);
int nx_drm_g2d_exec_ioctl(struct drm_device *drm_dev, void *data,
			  struct drm_file *file);
int nx_drm_g2d_sync_ioctl(struct drm_device *drm_dev, void *data,
			  struct drm_file *file);
#endif

