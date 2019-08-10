// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include "nexell_drv.h"

static LIST_HEAD(nx_drm_subdrv_list);

int nx_drm_subdrv_register(struct nx_drm_subdrv *subdrv, struct device *dev)
{
	if (WARN_ON(!subdrv))
		return -EINVAL;

	subdrv->dev = dev;
	list_add_tail(&subdrv->list, &nx_drm_subdrv_list);

	return 0;
}

int nx_drm_subdrv_unregister(struct nx_drm_subdrv *subdrv)
{
	if (WARN_ON(!subdrv))
		return -EINVAL;

	list_del(&subdrv->list);

	return 0;
}

int nx_drm_subdrv_probe(struct drm_device *drm)
{
	struct nx_drm_subdrv *subdrv, *n;
	int err;

	if (WARN_ON(!drm))
		return -EINVAL;

	list_for_each_entry_safe(subdrv, n, &nx_drm_subdrv_list, list) {
		if (subdrv->probe) {
			subdrv->drm = drm;

			/*
			 * this probe callback would be called by sub driver
			 * after setting of all resources to this sub driver,
			 * such as clock, irq and register map are done.
			 */
			err = subdrv->probe(drm, subdrv->dev);
			if (err) {
				DRM_DEBUG_KMS("nx drm subdrv probe failed.\n");
				list_del(&subdrv->list);
				continue;
			}
		}
	}

	return 0;
}

int nx_drm_subdrv_remove(struct drm_device *drm)
{
	struct nx_drm_subdrv *subdrv;

	if (WARN_ON(!drm))
		return -EINVAL;

	list_for_each_entry(subdrv, &nx_drm_subdrv_list, list) {
		if (subdrv->remove)
			subdrv->remove(drm, subdrv->dev);
	}

	return 0;
}

int nx_drm_subdrv_open(struct drm_device *drm, struct drm_file *file)
{
	struct nx_drm_subdrv *subdrv;
	int ret;

	list_for_each_entry(subdrv, &nx_drm_subdrv_list, list) {
		if (subdrv->driver_data)
			file->driver_priv = subdrv->driver_data;

		if (subdrv->open) {
			ret = subdrv->open(drm, subdrv->dev, file);
			if (ret)
				goto err;
		}
	}

	return 0;

err:
	list_for_each_entry_continue_reverse(subdrv,
		&nx_drm_subdrv_list, list) {
		if (subdrv->close)
			subdrv->close(drm, subdrv->dev, file);
	}
	return ret;
}

void nx_drm_subdrv_close(struct drm_device *drm, struct drm_file *file)
{
	struct nx_drm_subdrv *subdrv;

	list_for_each_entry(subdrv, &nx_drm_subdrv_list, list) {
		if (subdrv->close)
			subdrv->close(drm, subdrv->dev, file);
	}
}
