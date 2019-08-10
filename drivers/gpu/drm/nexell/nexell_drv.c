// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/component.h>
#include <linux/console.h>

#include <drm/nexell_drm.h>

#include "nexell_drv.h"
#include "nexell_fb.h"
#include "nexell_gem.h"

static const struct of_device_id of_nx_drm_match[];

static void nx_drm_output_poll_changed(struct drm_device *drm)
{
	struct nx_drm_private *private = drm->dev_private;
	struct nx_drm_fb_helper *nx_fb_helper = private->fb_helper;

	DRM_DEBUG_KMS("enter : framebuffer dev %s\n",
		nx_fb_helper ? "exist" : "non exist");

	if (nx_fb_helper)
		drm_fb_helper_hotplug_event(&nx_fb_helper->fb_helper);
	else
		nx_drm_fb_helper_init(drm);
}

static int nx_drm_atomic_check(struct drm_device *dev,
			       struct drm_atomic_state *state)
{
	return drm_atomic_helper_check(dev, state);
}

static struct drm_mode_config_helper_funcs nx_drm_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static struct drm_mode_config_funcs nx_mode_config_funcs = {
	.fb_create = nx_drm_mode_fb_create,
	.output_poll_changed = nx_drm_output_poll_changed,
	.atomic_check = nx_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void nx_drm_mode_config_init(struct drm_device *drm)
{
	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;

	/*
	 * set max width and height as default value
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	drm->mode_config.max_width = MAX_FB_MODE_WIDTH;
	drm->mode_config.max_height = MAX_FB_MODE_HEIGHT;

	drm->mode_config.funcs = &nx_mode_config_funcs;
	drm->mode_config.helper_private = &nx_drm_mode_config_helpers;

	drm->mode_config.allow_fb_modifiers = true;

	DRM_DEBUG_KMS("min %d*%d, max %d*%d\n",
		 drm->mode_config.min_width, drm->mode_config.min_height,
		 drm->mode_config.max_width, drm->mode_config.max_height);
}

static void nx_drm_lastclose(struct drm_device *drm)
{
	struct nx_drm_private *private = drm->dev_private;
	struct drm_fb_helper *fb_helper;

	if (!private || !private->fb_helper)
		return;

	fb_helper = &private->fb_helper->fb_helper;
	if (fb_helper)
		drm_fb_helper_restore_fbdev_mode_unlocked(fb_helper);
}

#ifdef CONFIG_DRM_NEXELL_GEM
static struct drm_ioctl_desc nx_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(NX_GEM_CREATE, nx_drm_gem_create_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NX_GEM_SYNC, nx_drm_gem_sync_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NX_GEM_GET, nx_drm_gem_get_ioctl, DRM_UNLOCKED),
#ifdef CONFIG_DRM_NEXELL_G2D
	DRM_IOCTL_DEF_DRV(NX_G2D_GET_VER, nx_drm_g2d_get_version, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(NX_G2D_DMA_EXEC, nx_drm_g2d_exec_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(NX_G2D_DMA_SYNC, nx_drm_g2d_sync_ioctl, DRM_UNLOCKED),
#endif
};

static const struct file_operations nx_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.mmap = nx_drm_gem_fops_mmap,
};
#else
DEFINE_DRM_GEM_CMA_FOPS(nx_drm_fops);

static struct dma_buf *__drm_gem_prime_export(struct drm_device *drm,
					      struct drm_gem_object *obj,
					      int flags)
{
	/* we want to be able to write in mmapped buffer */
	flags |= O_RDWR;

	return drm_gem_prime_export(drm, obj, flags);
}
#endif

static int nx_drm_open(struct drm_device *dev, struct drm_file *file)
{
	return nx_drm_subdrv_open(dev, file);
}

static void nx_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	nx_drm_subdrv_close(dev, file);
}

static struct drm_driver nx_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM |
			DRIVER_PRIME | DRIVER_ATOMIC | DRIVER_RENDER,
	.open = nx_drm_open,
	.postclose = nx_drm_postclose,
	.lastclose = nx_drm_lastclose,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,

#ifdef CONFIG_DRM_NEXELL_GEM
	.dumb_create = nx_drm_gem_dumb_create,
	.gem_free_object_unlocked = nx_drm_gem_free_object,
	.dumb_map_offset = nx_drm_gem_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,

	.gem_prime_import = nx_drm_gem_prime_import,
	.gem_prime_export = nx_drm_gem_prime_export,
	.gem_prime_get_sg_table = nx_drm_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = nx_drm_gem_prime_import_sg_table,
	.gem_prime_vmap = nx_drm_gem_prime_vmap,
	.gem_prime_vunmap = nx_drm_gem_prime_vunmap,
	.gem_prime_mmap	= nx_drm_gem_prime_mmap,
	.ioctls = nx_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(nx_drm_ioctls),
#else
	.dumb_create = drm_gem_cma_dumb_create,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = __drm_gem_prime_export,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
#endif
	.fops = &nx_drm_fops,
	.name = "nexell",
	.desc = "Nexell SoC DRM",
	.date = "20180514",
	.major = 3,
	.minor = 0,
};

static int nx_drm_bind(struct device *dev)
{
	struct drm_device *drm;
	struct nx_drm_private *private;
	const struct of_device_id *id;
	int ret;

	DRM_DEBUG_DRIVER("enter %s\n", dev_name(dev));

	drm = drm_dev_alloc(&nx_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	private = devm_kzalloc(dev, sizeof(*private), GFP_KERNEL);
	if (!private) {
		ret = -ENOMEM;
		goto err_free_drm;
	}

	id = of_match_node(of_nx_drm_match, drm->dev->of_node);
	private->data = (struct nx_drm_of_data *)id->data;

	drm->dev_private = (void *)private;
	dev_set_drvdata(dev, drm);

	drm_mode_config_init(drm);

	nx_drm_mode_config_init(drm);

	ret = nx_drm_crtc_create(drm, private->data->max_crtcs);
	if (ret)
		goto err_mode_config_cleanup;

	/* Try to bind all sub drivers. */
	ret = component_bind_all(drm->dev, drm);
	if (ret)
		goto err_unbind_all;

	ret = drm_vblank_init(drm, private->num_crtcs);
	if (ret)
		goto err_mode_config_cleanup;

	/* Probe non kms sub drivers and virtual display driver. */
	ret = nx_drm_subdrv_probe(drm);
	if (ret)
		goto err_unbind_all;

	drm_mode_config_reset(drm);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(drm);

	/* register the DRM device */
	ret = drm_dev_register(drm, 0);
	if (ret < 0)
		goto err_cleanup_poll;

	/* force connectors detection for LCD */
	if (private->force_detect)
		drm_helper_hpd_irq_event(drm);

	return 0;

err_cleanup_poll:
	drm_kms_helper_poll_fini(drm);
	nx_drm_subdrv_remove(drm);
err_unbind_all:
	component_unbind_all(drm->dev, drm);
err_mode_config_cleanup:
	drm_mode_config_cleanup(drm);
err_free_drm:
	drm_dev_unref(drm);

	return ret;
}

static void nx_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_dev_unregister(drm);

	nx_drm_subdrv_remove(drm);
	nx_drm_fb_helper_exit(drm);

	drm_kms_helper_poll_fini(drm);

	component_unbind_all(drm->dev, drm);

	drm_mode_config_cleanup(drm);

	dev_set_drvdata(dev, NULL);

	drm_dev_unref(drm);
}

static const struct component_master_ops nx_drm_component_ops = {
	.bind = nx_drm_bind,
	.unbind = nx_drm_unbind,
};

static int compare_dev(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

static int compare_component(struct device *dev, void *data)
{
	const char *t = data;
	const char *f = dev->of_node ?
			dev->of_node->full_name : dev_name(dev);

	return strstr(f, t) ? 1 : 0;
}

/*
 * @ drm driver node name (x:name)
 */
static LIST_HEAD(nx_drm_load_list);
#define NX_DRM_DRIVER(m, n, d, cond) \
	{ .match = m, .name = n, .driver = IS_ENABLED(cond) ? &d : NULL, }

static struct nx_drm_display_driver {
	const char *match, *name;
	struct platform_driver *driver;
	struct list_head list;
} nx_drm_platform_drvs[] = {
	NX_DRM_DRIVER("drm_lvds", "lvds",
			panel_lcd_driver, CONFIG_DRM_NEXELL_LVDS),
	NX_DRM_DRIVER("drm_rgb", "rgb",
			panel_lcd_driver, CONFIG_DRM_NEXELL_RGB),
	NX_DRM_DRIVER("drm_g2d", "g2d",
			drm_g2d_driver, CONFIG_DRM_NEXELL_G2D),
};

static int nx_drm_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct device *dev = &pdev->dev;
	int found = 0;
	int i;

	DRM_DEBUG_DRIVER("enter %s\n", dev_name(dev));

	for (i = 0; i < ARRAY_SIZE(nx_drm_platform_drvs); i++) {
		struct nx_drm_display_driver *drv = &nx_drm_platform_drvs[i];
		const char *name = drv->match;
		struct device *p = NULL, *d;

		if (!drv->driver)
			continue;

		while ((d = bus_find_device(&platform_bus_type, p,
					    (void *)name, compare_component))) {
			put_device(p);
			component_match_add(dev, &match, compare_dev, d);
			p = d;
			found++;
		}
		put_device(p);
	}

	if (!found)
		return -EINVAL;

	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	device_enable_async_suspend(dev);

	/* call master bind */
	return component_master_add_with_match(dev,
				&nx_drm_component_ops, match);
}

static int nx_drm_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &nx_drm_component_ops);

	return 0;
}

static struct nx_drm_of_data nx_nxp3220_match = {
	.max_crtcs = 1,
	.max_connectors = 1,
};

static const struct of_device_id of_nx_drm_match[] = {
	{.compatible = "nexell,nxp3220-drm", .data = &nx_nxp3220_match,},
	{}
};
MODULE_DEVICE_TABLE(of, of_nx_drm_match);

#ifdef CONFIG_PM_SLEEP
static void nx_drm_suspend(struct drm_device *drm)
{
#ifdef CONFIG_DRM_FBDEV_EMULATION
	struct nx_drm_private *private = drm->dev_private;

	console_lock();
	drm_fb_helper_set_suspend(&private->fb_helper->fb_helper, 1);
	console_unlock();
#endif
}

static void nx_drm_resume(struct drm_device *drm)
{
#ifdef CONFIG_DRM_FBDEV_EMULATION
	struct nx_drm_private *private = drm->dev_private;

	console_lock();
	drm_fb_helper_set_suspend(&private->fb_helper->fb_helper, 0);
	console_unlock();
#endif
}

static int nx_drm_pm_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct nx_drm_private *private = drm->dev_private;

	drm_kms_helper_poll_disable(drm);
	nx_drm_suspend(drm);

	private->suspend_state = drm_atomic_helper_suspend(drm);

	if (IS_ERR(private->suspend_state)) {
		nx_drm_resume(drm);
		drm_kms_helper_poll_enable(drm);
		return PTR_ERR(private->suspend_state);
	}

	return 0;
}

static int nx_drm_pm_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct nx_drm_private *private = drm->dev_private;

	drm_atomic_helper_resume(drm, private->suspend_state);
	nx_drm_resume(drm);
	drm_kms_helper_poll_enable(drm);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(nx_drm_pm_ops, nx_drm_pm_suspend, nx_drm_pm_resume);

static struct platform_driver nx_drm_drviver = {
	.probe = nx_drm_probe,
	.remove = nx_drm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "nexell,display_drm",
		.of_match_table = of_match_ptr(of_nx_drm_match),
		.pm = &nx_drm_pm_ops,
	},
};

static int __init nx_drm_init(void)
{
	struct nx_drm_display_driver *e, *drv;
	bool load;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(nx_drm_platform_drvs); i++) {
		drv = &nx_drm_platform_drvs[i];
		load = false;

		if (!drv->driver)
			continue;

		list_for_each_entry(e, &nx_drm_load_list, list) {
			if (e->driver == drv->driver)
				load = true;
		}

		if (load)
			continue;

		DRM_DEBUG_DRIVER("Load [%d] %s\n", i, drv->name);

		ret = platform_driver_register(drv->driver);
		if (ret) {
			DRM_ERROR("Failed to register %s\n", drv->name);
			return ret;
		}

		list_add_tail(&drv->list, &nx_drm_load_list);
	}

	return platform_driver_register(&nx_drm_drviver);
}

static void __exit nx_drm_exit(void)
{
	struct nx_drm_display_driver *e, *drv;
	bool load;
	int i;

	platform_driver_unregister(&nx_drm_drviver);

	for (i = 0; i < ARRAY_SIZE(nx_drm_platform_drvs); i++) {
		drv = &nx_drm_platform_drvs[i];
		load = false;

		if (!drv->driver)
			continue;

		list_for_each_entry(e, &nx_drm_load_list, list) {
			if (e->driver == drv->driver)
				load = true;
		}

		if (!load)
			continue;

		DRM_DEBUG_DRIVER("UnLoad[%d] %s\n", i, drv->name);

		platform_driver_unregister(drv->driver);
		list_del(&drv->list);
	}
}

#ifdef CONFIG_DEFERRED_UP_DRM
early_device_initcall(nx_drm_init);
#else
module_init(nx_drm_init);
#endif
module_exit(nx_drm_exit);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell DRM Driver");
MODULE_LICENSE("GPL");
