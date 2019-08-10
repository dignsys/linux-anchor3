// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <linux/dma-buf.h>

#include "nexell_drv.h"
#include "nexell_fb.h"
#include "nexell_gem.h"

#define PREFERRED_BPP		32

static bool fb_format_bgr;
static bool fb_format_argb;

MODULE_PARM_DESC(fb_bgr, "frame buffer BGR pixel format");
module_param_named(fb_bgr, fb_format_bgr, bool, 0600);
MODULE_PARM_DESC(fb_argb, "frame buffer ARGB pixel format");
module_param_named(fb_argb, fb_format_argb, bool, 0600);

#ifdef CONFIG_DRM_PRE_INIT_DRM
#include <linux/memblock.h>
#include <linux/of_address.h>

static int nx_drm_fb_splash_image_copy(struct device *dev,
				       phys_addr_t logo_phys, size_t logo_size,
				       void *screen_virt, size_t screen_size)
{
	int npages, i;
	struct page **pages;
	void *logo_virt;

	npages = PAGE_ALIGN(logo_size) / PAGE_SIZE;
	pages = kcalloc(npages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < npages; i++)
		pages[i] = pfn_to_page((logo_phys / PAGE_SIZE) + i);

	logo_virt = vmap(pages, npages, VM_MAP, PAGE_KERNEL);
	if (!logo_virt) {
		dev_err(dev, "Failed to map for splash image:0x%lx:%u\n",
			(unsigned long)logo_phys, (unsigned int)logo_size);
		kfree(pages);
		return -ENOMEM;
	}

	DRM_INFO("Copy splash image from 0x%lx(0x%p) to 0x%p size %u\n",
		(unsigned long)logo_phys, logo_virt, screen_virt,
		(unsigned int)screen_size);

	memcpy(screen_virt, logo_virt, screen_size);
	kfree(pages);
	vunmap(logo_virt);

	return 0;
}

/*
 * get previous FB base address from hw register
 */
static dma_addr_t nx_drm_fb_get_splash_base(struct drm_fb_helper *fb_helper)
{
	struct drm_crtc *crtc;
	dma_addr_t dma_addr;
	int i;

	for (i = 0; i < fb_helper->crtc_count; i++) {
		crtc = fb_helper->crtc_info[i].mode_set.crtc;
		if (crtc->primary) {
			dma_addr = nx_drm_get_dma_addr(crtc->primary);
			if (dma_addr)
				return dma_addr;
		}
	}
	return 0;
}

static int nx_drm_fb_splash_boot_logo(struct drm_fb_helper *fb_helper)
{
	struct device_node *node;
	struct drm_device *drm = fb_helper->dev;
	struct fb_info *fbi = fb_helper->fbdev;
	void *screen_base = fbi->screen_base;
	int size = fbi->var.xres * fbi->var.yres *
			(fbi->var.bits_per_pixel / 8);
	dma_addr_t logo_phys = 0;
	int reserved = 0;
	struct resource r;
	int ret;

	node = of_find_node_by_name(NULL, "display_reserved");
	if (node) {
		ret = of_address_to_resource(node, 0, &r);
		if (!ret) {
			of_node_put(node);

			reserved = memblock_is_region_reserved(
					r.start, resource_size(&r));
			if (!reserved)
				DRM_INFO("can't reserved splash:0x%pa:%d\n",
					&r.start, (int)resource_size(&r));

			if (reserved) {
				if (size > resource_size(&r)) {
					DRM_INFO(
					"WARN: reserved splash size %d less %d",
					(int)resource_size(&r), size);
				}
			}
			logo_phys = r.start;
			DRM_INFO("splash image: reserved mem: 0x%x size %u\n",
				 logo_phys, resource_size(&r));
		}
	}

	if (!logo_phys) {
		DRM_INFO("splash image: no reserved memory-region\n");
		DRM_INFO("splash image: memroy from HW\n");
		logo_phys = nx_drm_fb_get_splash_base(fb_helper);
		if (!logo_phys) {
			DRM_ERROR("splash image: not setted memory !!!\n");
			return -EFAULT;
		}
	}

	nx_drm_fb_splash_image_copy(drm->dev, logo_phys, size,
				    screen_base, size);

	if (reserved) {
		memblock_free(r.start, resource_size(&r));
		memblock_remove(r.start, resource_size(&r));
		free_reserved_area(__va(r.start),
				__va(r.start + resource_size(&r)), -1, NULL);
	}

	return 0;
}

static void nx_drm_fb_splash_hotplug_event(struct drm_fb_helper *fb_helper)
{
	/* for FBDEV_EMULATION connection */
	drm_fb_helper_hotplug_event(fb_helper);
}
#endif

static void nx_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct nx_drm_fb *nx_fb = to_nx_drm_fb(fb);
	int i;

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < ARRAY_SIZE(nx_fb->obj); i++) {
		struct drm_gem_object *obj;

		if (!nx_fb->obj[i])
			continue;

		obj = &nx_fb->obj[i]->base;
		drm_gem_object_put_unlocked(obj);
	}

	kfree(nx_fb);
}

static int nx_drm_fb_create_handle(struct drm_framebuffer *fb,
				   struct drm_file *file_priv,
				   unsigned int *handle)
{
	struct nx_drm_fb *nx_fb = to_nx_drm_fb(fb);

	return drm_gem_handle_create(file_priv, &nx_fb->obj[0]->base, handle);
}

static struct drm_framebuffer_funcs nx_drm_framebuffer_funcs = {
	.destroy = nx_drm_fb_destroy,
	.create_handle = nx_drm_fb_create_handle,
};

static struct fb_ops nx_fb_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,
	.fb_pan_display	= drm_fb_helper_pan_display,
};

static struct nx_drm_fb *nx_drm_fb_alloc(
				struct drm_device *drm,
				const struct drm_mode_fb_cmd2 *mode_cmd,
				struct nx_gem_object **nx_obj,
				unsigned int num_planes)
{
	struct nx_drm_fb *nx_fb;
	int i, ret;

	nx_fb = kzalloc(sizeof(*nx_fb), GFP_KERNEL);
	if (!nx_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(drm, &nx_fb->fb, mode_cmd);

	for (i = 0; i < num_planes; i++)
		nx_fb->obj[i] = nx_obj[i];

	ret = drm_framebuffer_init(drm, &nx_fb->fb, &nx_drm_framebuffer_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize framebuffer:%d\n", ret);
		kfree(nx_fb);
		return ERR_PTR(ret);
	}

	return nx_fb;
}

static uint32_t nx_drm_fb_mode_format(uint32_t bpp, uint32_t depth,
				      bool bgr, bool argb)
{
	struct drm_format_name_buf format_name;
	uint32_t fmt;

	switch (bpp) {
	case 8:
		fmt = DRM_FORMAT_C8;
		break;
	case 16:
		if (depth == 15)
			fmt = bgr ? DRM_FORMAT_XBGR1555 : DRM_FORMAT_XRGB1555;
		else
			fmt = bgr ? DRM_FORMAT_BGR565 : DRM_FORMAT_RGB565;
		break;
	case 24:
		fmt = bgr ? DRM_FORMAT_BGR888 : DRM_FORMAT_RGB888;
		break;
	case 32:
		if ((!argb) && (depth == 24))
			fmt = bgr ? DRM_FORMAT_XBGR8888 : DRM_FORMAT_XRGB8888;
		else
			fmt = bgr ? DRM_FORMAT_ABGR8888 : DRM_FORMAT_ARGB8888;
		break;
	default:
		DRM_ERROR("bad bpp, assuming x8r8g8b8 pixel format\n");
		fmt = DRM_FORMAT_XRGB8888;
		break;
	}

	DRM_INFO("FB format:%s\n", drm_get_format_name(fmt, &format_name));

	return fmt;
}

static int nx_drm_fb_helper_probe(struct drm_fb_helper *fb_helper,
				  struct drm_fb_helper_surface_size *sizes)
{
	struct nx_drm_fb_helper *nx_fb_helper = to_nx_drm_fb_helper(fb_helper);
	struct drm_device *drm = fb_helper->dev;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct nx_gem_object *nx_obj;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	unsigned int flags = 0;
	int ret;

	DRM_INFO("framebuffer width(%d), height(%d) and bpp(%d)\n",
		sizes->surface_width, sizes->surface_height,
		sizes->surface_bpp);

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode_cmd.pixel_format = nx_drm_fb_mode_format(sizes->surface_bpp,
		sizes->surface_depth, fb_format_bgr, fb_format_argb);

	/* for double buffer */
	size = mode_cmd.pitches[0] * mode_cmd.height;

	nx_obj = nx_drm_gem_create(drm, size, flags);
	if (IS_ERR(nx_obj))
		return -ENOMEM;

	nx_fb_helper->fb = nx_drm_fb_alloc(drm, &mode_cmd, &nx_obj, 1);
	if (IS_ERR(fb_helper->fb)) {
		DRM_ERROR("Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(fb_helper->fb);
		goto err_framebuffer_release;
	}

	fb_helper->fb = &nx_fb_helper->fb->fb;
	fb = fb_helper->fb;

	fbi = drm_fb_helper_alloc_fbi(fb_helper);
	if (IS_ERR(fbi)) {
		DRM_ERROR("Failed to allocate framebuffer info.\n");
		ret = PTR_ERR(fbi);
		goto err_drm_fb_destroy;
	}
	fbi->par = fb_helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &nx_fb_ops;

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->format->depth);
	drm_fb_helper_fill_var(fbi, fb_helper,
			sizes->fb_width, sizes->fb_height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	drm->mode_config.fb_base = (resource_size_t)nx_obj->dma_addr;
	fbi->screen_base = (void __iomem *)nx_obj->cpu_addr + offset;
	fbi->fix.smem_start = (unsigned long)(nx_obj->dma_addr + offset);
	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	if (fb_helper->crtc_info &&
		fb_helper->crtc_info->desired_mode) {
		struct videomode vm;
		struct drm_display_mode *mode =
				fb_helper->crtc_info->desired_mode;

		drm_display_mode_to_videomode(mode, &vm);
		fbi->var.left_margin = vm.hsync_len + vm.hback_porch;
		fbi->var.right_margin = vm.hfront_porch;
		fbi->var.upper_margin = vm.vsync_len + vm.vback_porch;
		fbi->var.lower_margin = vm.vfront_porch;
		/* pico second */
		fbi->var.pixclock = KHZ2PICOS(vm.pixelclock/1000);
	}

	DRM_INFO("FB counts:%d\n", fbi->var.yres_virtual/fbi->var.yres);

#ifdef CONFIG_DRM_PRE_INIT_DRM
	nx_drm_fb_splash_boot_logo(fb_helper);
#endif
	return 0;

err_drm_fb_destroy:
	nx_drm_fb_destroy(fb);

err_framebuffer_release:
	nx_drm_gem_destroy(nx_obj);

	return ret;
}

static const struct drm_fb_helper_funcs nx_drm_fb_helper_funcs = {
	.fb_probe = nx_drm_fb_helper_probe,
};

static void nx_drm_fb_restore_mode_config(struct drm_device *drm,
					  int crts, int conns)
{
	drm->mode_config.num_connector = conns;
}

int nx_drm_fb_helper_init(struct drm_device *drm)
{
	struct drm_fb_helper *fb_helper;
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;
	struct nx_drm_fb_helper *nx_fb_helper;
	struct nx_drm_private *private = drm->dev_private;
	int max_conns = private->data->max_connectors;
	int num_crtc = drm->mode_config.num_crtc;
	int num_conns = drm->mode_config.num_connector;
	int ret;

	if (!drm->mode_config.num_crtc ||
		!drm->mode_config.num_connector)
		return 0;

	nx_fb_helper = kzalloc(sizeof(*nx_fb_helper), GFP_KERNEL);
	if (!nx_fb_helper)
		return -ENOMEM;

	private->fb_helper = nx_fb_helper;
	fb_helper = &nx_fb_helper->fb_helper;

	drm_fb_helper_prepare(drm, fb_helper, &nx_drm_fb_helper_funcs);

	ret = drm_fb_helper_init(drm, fb_helper, max_conns);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize drm fb fb_helper.\n");
		goto err_free;
	}

	/*
	 * replace 'drm_fb_helper_single_add_all_connectors'.
	 * To set for one framebuffer device
	 */
	drm_connector_list_iter_begin(drm, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		DRM_DEBUG_KMS("FB connector:%s\n", connector->name);

		ret = drm_fb_helper_add_one_connector(fb_helper, connector);
		if (ret) {
			DRM_ERROR("Failed to add connectors.\n");
			drm_connector_list_iter_end(&conn_iter);
			goto err_drm_fb_helper_fini;
		}
	}
	drm_connector_list_iter_end(&conn_iter);

	ret = drm_fb_helper_initial_config(fb_helper, PREFERRED_BPP);
	if (ret < 0) {
		DRM_ERROR("Failed to set initial hw configuration.\n");
		goto err_drm_fb_helper_fini;
	}

	nx_drm_fb_restore_mode_config(drm, num_crtc, num_conns);

	DRM_DEBUG_KMS("crtc num:%d, connector num:%d\n", num_crtc, num_conns);

#ifdef CONFIG_DRM_PRE_INIT_DRM
	nx_drm_fb_splash_hotplug_event(fb_helper);
#endif

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(fb_helper);

err_free:
	kfree(nx_fb_helper);
	private->fb_helper = NULL;

	return ret;
}

void nx_drm_fb_helper_exit(struct drm_device *drm)
{
	struct nx_drm_private *private = drm->dev_private;
	struct drm_fb_helper *fb_helper;
	struct drm_framebuffer *fb;

	if (!private->fb_helper)
		return;

	fb_helper = &private->fb_helper->fb_helper;

	/* release drm framebuffer and real buffer */
	if (fb_helper->fb && fb_helper->fb->funcs) {
		fb = fb_helper->fb;
		if (fb) {
			drm_framebuffer_unregister_private(fb);
			drm_framebuffer_remove(fb);
		}
	}

	drm_fb_helper_unregister_fbi(fb_helper);
	drm_fb_helper_fini(fb_helper);

	kfree(private->fb_helper);
	private->fb_helper = NULL;
}

struct drm_framebuffer *nx_drm_mode_fb_create(
				struct drm_device *drm,
				struct drm_file *file_priv,
				const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct nx_drm_fb *nx_fb;
	struct nx_gem_object *nx_objs[4];
	struct drm_gem_object *obj;
	unsigned int hsub;
	unsigned int vsub;
	int ret;
	int i;

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	for (i = 0; i < drm_format_num_planes(mode_cmd->pixel_format); i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = drm_gem_object_lookup(file_priv, mode_cmd->handles[i]);
		if (!obj) {
			DRM_ERROR("Failed to lookup GEM object\n");
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		min_size = (height - 1) * mode_cmd->pitches[i]
			+ width
			* drm_format_plane_cpp(mode_cmd->pixel_format, i)
			+ mode_cmd->offsets[i];

		if (obj->size < min_size) {
			drm_gem_object_put_unlocked(obj);
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}
		nx_objs[i] = to_nx_gem_obj(obj);
	}

	nx_fb = nx_drm_fb_alloc(drm, mode_cmd, nx_objs, i);
	if (IS_ERR(nx_fb)) {
		ret = PTR_ERR(nx_fb);
		goto err_gem_object_unreference;
	}

	return &nx_fb->fb;

err_gem_object_unreference:
	for (i--; i >= 0; i--)
		drm_gem_object_put_unlocked(&nx_objs[i]->base);

	return ERR_PTR(ret);
}

/*
 * fb with gem
 */
struct nx_gem_object *nx_drm_fb_gem(struct drm_framebuffer *fb,
				    unsigned int plane)
{
	struct nx_drm_fb *nx_fb = to_nx_drm_fb(fb);

	if (plane >= 4)
		return NULL;

	return nx_fb->obj[plane];
}
EXPORT_SYMBOL_GPL(nx_drm_fb_gem);

