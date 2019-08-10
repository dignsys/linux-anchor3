// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM SoC interface driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <uapi/drm/drm_fourcc.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/dma-buf.h>

#include "../nexell_fb.h"
#include "../nexell_gem.h"
#include "display.h"

static const uint32_t support_formats_rgb[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_XBGR4444,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ABGR4444,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
};

static const uint32_t support_formats_vid[] = {
	/* 1 plane */
	DRM_FORMAT_YUYV, /* [31:0] Cr0:Y1:Cb0:Y0 8:8:8:8 little endian */
	/*2 plane YCbCr */
	DRM_FORMAT_NV16, /* 2x1 subsampled Cb:Cr plane : 422 */
	DRM_FORMAT_NV12, /* 2x2 subsampled Cb:Cr plane : 420 */
	/* 3 plane YCbCr */
	DRM_FORMAT_YUV420, /* 2x2 subsampled Cb (1) and Cr (2) planes */
	DRM_FORMAT_YVU420, /* 2x2 subsampled Cr (1) and Cb (2) planes */
	DRM_FORMAT_YUV422, /* 2x1 subsampled Cb (1) and Cr (2) planes */
	DRM_FORMAT_YVU422, /* 2x1 subsampled Cr (1) and Cb (2) planes */
	DRM_FORMAT_YUV444, /* non-subsampled Cb (1) and Cr (2) planes */
	DRM_FORMAT_YVU444, /* non-subsampled Cr (1) and Cb (2) planes */
};

static struct nx_plane_format {
	const void *formats;
	int format_count;
} nx_plane_formats[] = {
	{
		.formats = support_formats_rgb,
		.format_count = ARRAY_SIZE(support_formats_rgb),
	}, {
		.formats = support_formats_vid,
		.format_count = ARRAY_SIZE(support_formats_vid),
	},
};

struct nx_plane_rect {
	int left, right;
	int top, bottom;
};

#define	display_to_ctx(d)	((struct nx_display_ctx *)d->context)

static int plane_rgb_format(uint32_t pixel_format,
			    uint32_t bpp, uint32_t depth, uint32_t *format)
{
	struct drm_format_name_buf format_name;
	uint32_t fmt;

	switch (pixel_format) {
	case DRM_FORMAT_RGB565:
		fmt = NX_MLC_FMT_RGB_R5G6B5;
		break;
	case DRM_FORMAT_BGR565:
		fmt = NX_MLC_FMT_RGB_B5G6R5;
		break;
	case DRM_FORMAT_XRGB1555:
		fmt = NX_MLC_FMT_RGB_X1R5G5B5;
		break;
	case DRM_FORMAT_XBGR1555:
		fmt = NX_MLC_FMT_RGB_X1B5G5R5;
		break;
	case DRM_FORMAT_XRGB4444:
		fmt = NX_MLC_FMT_RGB_X4R4G4B4;
		break;
	case DRM_FORMAT_XBGR4444:
		fmt = NX_MLC_FMT_RGB_X4B4G4R4;
		break;
	case DRM_FORMAT_ARGB1555:
		fmt = NX_MLC_FMT_RGB_A1R5G5B5;
		break;
	case DRM_FORMAT_ABGR1555:
		fmt = NX_MLC_FMT_RGB_A1B5G5R5;
		break;
	case DRM_FORMAT_ARGB4444:
		fmt = NX_MLC_FMT_RGB_A4R4G4B4;
		break;
	case DRM_FORMAT_ABGR4444:
		fmt = NX_MLC_FMT_RGB_A4B4G4R4;
		break;
	case DRM_FORMAT_RGB888:
		fmt = NX_MLC_FMT_RGB_R8G8B8;
		break;
	case DRM_FORMAT_BGR888:
		fmt = NX_MLC_FMT_RGB_B8G8R8;
		break;
	case DRM_FORMAT_XRGB8888:
		fmt = NX_MLC_FMT_RGB_X8R8G8B8;
		break;
	case DRM_FORMAT_XBGR8888:
		fmt = NX_MLC_FMT_RGB_X8B8G8R8;
		break;
	case DRM_FORMAT_ARGB8888:
		fmt = NX_MLC_FMT_RGB_A8R8G8B8;
		break;
	case DRM_FORMAT_ABGR8888:
		fmt = NX_MLC_FMT_RGB_A8B8G8R8;
		break;
	default:
		DRM_ERROR("Failed, not support %s pixel format\n",
			drm_get_format_name(pixel_format, &format_name));
		return -EINVAL;
	}

	*format = fmt;
	return 0;
}

static uint32_t plane_yuv_format(uint32_t fourcc, uint32_t *format)
{
	struct drm_format_name_buf format_name;
	uint32_t fmt;

	switch (fourcc) {
	case DRM_FORMAT_YVU420:
		fmt = NX_MLC_FMT_VID_420 | FMT_VID_YUV_TO_YVU;
		break;
	case DRM_FORMAT_YUV420:
		fmt = NX_MLC_FMT_VID_420;
		break;
	case DRM_FORMAT_YVU422:
		fmt = NX_MLC_FMT_VID_422 | FMT_VID_YUV_TO_YVU;
		break;
	case DRM_FORMAT_YUV422:
		fmt = NX_MLC_FMT_VID_422;
		break;
	case DRM_FORMAT_YVU444:
		fmt = NX_MLC_FMT_VID_444 | FMT_VID_YUV_TO_YVU;
		break;
	case DRM_FORMAT_YUV444:
		fmt = NX_MLC_FMT_VID_444;
		break;
	case DRM_FORMAT_YUYV:
		fmt = NX_MLC_FMT_VID_YUYV;
		break;
	case DRM_FORMAT_NV16:
		fmt = NX_MLC_FMT_VID_422_CBCR;
		break;
	case DRM_FORMAT_NV12:
		fmt = NX_MLC_FMT_VID_420_CBCR;
		break;
	default:
		DRM_ERROR("Failed, not support fourcc %s\n",
		       drm_get_format_name(fourcc, &format_name));
		return -EINVAL;
	}

	*format = fmt;
	return 0;
}

static inline void plane_rgb_rect(struct nx_plane_rect *rect,
				  int clip_x, int clip_y,
				  int clip_w, int clip_h,
				  int act_w, int act_h)
{
	struct nx_plane_rect r = {
		.left = clip_x, .right = clip_x + clip_w,
		.top = clip_y, .bottom = clip_y + clip_h };

	clip_w = r.right - r.left;
	clip_h = r.bottom - r.top;

	if (clip_w < 1 || clip_h < 1)
		return;

	if (r.left > (act_w - 1))
		return;

	if (r.top > (act_h - 1) || r.bottom < 1)
		return;

	if (r.left < 0)
		r.left = 0;

	if (r.right > act_w)
		r.right = act_w;

	rect->left = r.left;
	rect->right = r.right;
	rect->top = r.top;
	rect->bottom = r.bottom;
}

static inline void plane_yuv_rect(struct nx_plane_rect *rect,
				  int clip_x, int clip_y,
				  int clip_w, int clip_h,
				  int act_w, int act_h)
{
	if (clip_x > (act_w - 1) || (clip_x + clip_w) < 1)
		return;

	if (clip_y > (act_h - 1) || (clip_y + clip_h) < 1)
		return;

	rect->left = clip_x;
	rect->top = clip_y;
	rect->bottom = clip_y + clip_h;
	rect->right = clip_x + clip_w;
}

static int nx_display_crtc_begin(struct drm_crtc *crtc)
{
	struct nx_display *dp = to_nx_crtc(crtc)->context;
	struct nx_overlay *ovl = to_nx_plane(crtc->primary)->context;
	int crtc_w, crtc_h;
	bool colorkey_enb = false;

	crtc_w = crtc->state->mode.hdisplay;
	crtc_h = crtc->state->mode.vdisplay;

	DRM_DEBUG_KMS("crtc.%d: [%d,%d]\n", dp->module, crtc_w, crtc_h);

	nx_display_set_backcolor(dp);
	nx_display_set_format(dp, crtc_w, crtc_h);

	if (dp->color_key_on | ovl->colorkey_on)
		colorkey_enb = true;

	/* color key */
	nx_overlay_set_color(ovl, NX_COLOR_TRANS,
			dp->color_key, colorkey_enb, false);

	return 0;
}

static void nx_display_crtc_enable(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct nx_display *dp = to_nx_crtc(crtc)->context;
	struct nx_drm_connector *nx_connector = NULL;
	struct nx_drm_display *display;
	struct nx_display_ctx *ctx;

	DRM_DEBUG_KMS("crtc.%d\n", dp->module);

	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		nx_connector = to_nx_encoder(encoder)->connector;
		if (nx_connector)
			break;
	}

	if (!nx_connector)
		return;

	/* set output device */
	display = nx_connector->display;
	ctx = display_to_ctx(display);
	ctx->dp = dp;

	if (!display->fixed_sync) {
		drm_display_mode_to_videomode(&crtc->state->mode, &dp->vm);
		display->vm = dp->vm;
	} else {
		dp->vm = display->vm;
	}

	dp->dpp = &ctx->dpp;
	dp->panel_type = ctx->panel_type;
	dp->mpu_lcd = ctx->mpu_lcd;

	nx_display_set_mode(dp);
	nx_display_ovl_enable(dp);
	nx_display_enable(dp, true);
}

static void nx_display_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct nx_display *dp = to_nx_crtc(crtc)->context;
	struct nx_drm_connector *nx_connector = NULL;

	DRM_DEBUG_KMS("crtc.%d\n", dp->module);

	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		nx_connector = to_nx_encoder(encoder)->connector;
		if (nx_connector)
			break;
	}

	if (!nx_connector)
		return;

	nx_display_ovl_disable(dp);
	nx_display_enable(dp, false);
}

static void nx_display_crtc_destory(struct drm_crtc *crtc)
{
	struct nx_display *dp = to_nx_crtc(crtc)->context;

	iounmap(dp->mlc_base);
	iounmap(dp->dpc_base);
	kfree(dp);
}

static struct nx_drm_crtc_ops nx_crtc_ops = {
	.destroy = nx_display_crtc_destory,
	.begin = nx_display_crtc_begin,
	.enable = nx_display_crtc_enable,
	.disable = nx_display_crtc_disable,
};

static int nx_display_crtc_irq_enable(struct drm_crtc *crtc, int pipe)
{
	nx_display_irq_on(to_nx_crtc(crtc)->context, true);
	return 0;
}

static void nx_display_crtc_irq_disable(struct drm_crtc *crtc, int pipe)
{
	nx_display_irq_on(to_nx_crtc(crtc)->context, false);
}

static void nx_display_crtc_irq_done(struct drm_crtc *crtc, int pipe)
{
	nx_display_irq_clear(to_nx_crtc(crtc)->context);
}

static struct nx_drm_irq_ops nx_irq_ops = {
	.irq_enable = nx_display_crtc_irq_enable,
	.irq_disable = nx_display_crtc_irq_disable,
	.irq_done = nx_display_crtc_irq_done,
};

static int nx_display_crtc_parse_planes(struct device *dev,
					struct nx_display *dp, int pipe)
{
	struct device_node *np = dev->of_node;
	const char *strings[10];
	int i, size = 0;

	/* port node */
	size = of_property_read_string_array(np, "plane-names", strings, 10);
	if (!size) {
		DRM_ERROR("Failed, not found 'plane-names' property !!!\n");
		return -ENOENT;
	}

	for (i = 0; i < size; i++) {
		if (!strcmp("primary", strings[i])) {
			dp->overlay_types[i] =
				DRM_PLANE_TYPE_PRIMARY | NX_PLANE_TYPE_RGB;
		} else if (!strcmp("video", strings[i])) {
			dp->overlay_types[i] =
				DRM_PLANE_TYPE_OVERLAY | NX_PLANE_TYPE_VIDEO;
			dp->video_priority = i;	/* priority */
		} else {
			dp->overlay_types[i] = NX_PLANE_TYPE_UNKNOWN;
			DRM_ERROR("Failed, unknown plane [%d] %s\n",
				i, strings[i]);
		}
		DRM_DEBUG_KMS("crtc.%d planes[%d]: %s, bg:0x%08x, key:0x%08x\n",
			pipe, i, strings[i], dp->back_color,
			dp->color_key);
	}

	dp->num_overlays = size;
	dp->color_key_on = true;
	dp->alpla_blend_on = true;

	of_property_read_u32(np, "back-color", &dp->back_color);
	of_property_read_u32(np, "color-key", &dp->color_key);
	of_property_read_u32(np, "video-priority", &dp->video_priority);

	if (of_property_read_bool(np, "color-key-disable"))
		dp->color_key_on = false;

	if (of_property_read_bool(np, "alphablend-disable"))
		dp->alpla_blend_on = false;

	of_property_read_u32(np,
		"video-scale-hfilter-max", &dp->video_scale_hf_max);
	of_property_read_u32(np,
		"video-scale-vfilter-max", &dp->video_scale_vf_max);

	return 0;
}

static int nx_display_crtc_parse_clock(struct device *dev,
				       struct nx_display *dp, int pipe)
{
	struct device_node *node = dev->of_node;

	dp->clk_axi = of_clk_get_by_name(node, "axi");
	if (IS_ERR(dp->clk_axi)) {
		dev_err(dev, "Not found clock 'axi'\n");
		return PTR_ERR(dp->clk_axi);
	}

	dp->clk_x1 = of_clk_get_by_name(node, "clk_x1");
	if (IS_ERR(dp->clk_x1)) {
		dev_err(dev, "Not found clock 'clk_x1'\n");
		return PTR_ERR(dp->clk_x1);
	}

	dp->clk_x2 = of_clk_get_by_name(node, "clk_x2");
	if (IS_ERR(dp->clk_x2)) {
		dev_err(dev, "Not found clock 'clk_x2'\n");
		return PTR_ERR(dp->clk_x1);
	}

	return 0;
}

static int nx_display_crtc_parse_dt(struct drm_device *drm,
				    struct nx_display *dp, int pipe)
{
	struct device *dev = drm->dev;
	struct device_node *node = dev->of_node;
	char name[10];
	int id, err;

	DRM_DEBUG_KMS("crtc.%d for %s\n", pipe, dev_name(dev));

	snprintf(name, sizeof(name), "mlc.%d", pipe);
	id = of_property_match_string(node, "reg-names", name);
	dp->mlc_base = of_iomap(node, id);
	if (!dp->mlc_base)
		return -ENOMEM;

	snprintf(name, sizeof(name), "dpc.%d", pipe);
	id = of_property_match_string(node, "reg-names", name);
	dp->dpc_base = of_iomap(node, id);
	if (!dp->dpc_base)
		return -ENOMEM;

	err = of_irq_get(node, pipe);
	if (err < 0)
		return -EINVAL;

	dp->irq = err;
	err = nx_display_crtc_parse_clock(dev, dp, pipe);
	if (err < 0)
		return 0;

	return nx_display_crtc_parse_planes(dev, dp, pipe);
}

int nx_drm_crtc_init(struct drm_device *drm, struct drm_crtc *crtc, int pipe)
{
	struct nx_drm_crtc *nx_crtc;
	struct nx_display *dp;
	int err;

	dp = kzalloc(sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dev = drm->dev;
	dp->module = pipe;
	dp->crtc = crtc;

	INIT_LIST_HEAD(&dp->overlay_list);

	err = nx_display_crtc_parse_dt(drm, dp, pipe);
	if (err < 0) {
		kfree(dp);
		return err;
	}

	nx_crtc = to_nx_crtc(crtc);
	nx_crtc->context = dp;
	nx_crtc->irq = dp->irq;
	nx_crtc->num_planes = dp->num_overlays;
	nx_crtc->ops = &nx_crtc_ops;
	nx_crtc->irq_ops = &nx_irq_ops;

	return 0;
}

static int nx_display_plane_wait_fence(struct drm_framebuffer *fb,
				       struct nx_overlay *ovl)
{
	struct drm_gem_object *obj = to_gem_obj(nx_drm_fb_gem(fb, 0));
	int ret = nx_drm_gem_wait_fence(obj);

	DRM_DEBUG_KMS("crtc.%d plane.%d (%s) : ret:%d\n",
			ovl->dp->module, ovl->id, ovl->name, ret);

	return ret;
}

static int nx_display_plane_update_rgb(struct drm_framebuffer *fb,
				       struct nx_overlay *ovl,
				       int crtc_x, int crtc_y,
				       int crtc_w, int crtc_h,
				       int src_x, int src_y, int src_w,
				       int src_h, int align)
{
	struct nx_gem_object *nx_obj = nx_drm_fb_gem(fb, 0);
	struct nx_display *dp = ovl->dp;
	dma_addr_t dma_addr = nx_obj->dma_addr;
	int pixel = fb->format->cpp[0];
	int pitch = fb->pitches[0];
	unsigned int format;
	struct nx_plane_rect rect = { 0, };
	int ret;

	ret = nx_display_plane_wait_fence(fb, ovl);
	if (ret)
		return ret;

	ret = plane_rgb_format(fb->format->format,
			pixel * 8, fb->format->depth, &format);
	if (ret < 0)
		return ret;

	plane_rgb_rect(&rect, crtc_x, crtc_y, crtc_w, crtc_h,
			dp->width, dp->height);

	nx_overlay_set_format(ovl, format, pixel, false);
	nx_overlay_set_position(ovl,
			src_x, src_y, src_w, src_h, rect.left, rect.top,
			rect.right - rect.left, rect.bottom - rect.top,
			false);
	nx_overlay_set_addr_rgb(ovl, dma_addr, pixel, pitch, align, true);
	nx_overlay_enable(ovl, true);

	return 0;
}

static int nx_display_plane_addr_yuv(struct nx_overlay *ovl,
				     dma_addr_t dma_addrs[4],
				     unsigned int pitches[4],
				     unsigned int offsets[4], int planes,
				     bool sync)
{
	dma_addr_t lua, cba, cra;
	int lus, cbs, crs;
	int ret = 0;

	switch (planes) {
	case 1:
		lua = dma_addrs[0], lus = pitches[0];
		nx_overlay_set_addr_yuv(ovl, lua, lus,
					0, 0, 0, 0, planes, sync);
		break;
	case 2:
	case 3:
		lua = dma_addrs[0];
		cba = offsets[1] ? lua + offsets[1] : dma_addrs[1];
		cra = offsets[2] ? lua + offsets[2] : dma_addrs[2];
		lus = pitches[0], cbs = pitches[1], crs = pitches[2];
		nx_overlay_set_addr_yuv(ovl, lua, lus,
				cba, cbs, cra, crs, planes, sync);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int nx_display_plane_update_yuv(struct drm_framebuffer *fb,
				       struct nx_overlay *ovl,
				       int crtc_x, int crtc_y,
				       int crtc_w, int crtc_h,
				       int src_x, int src_y,
				       int src_w, int src_h)
{
	struct nx_display *dp = ovl->dp;
	struct nx_gem_object *nx_obj[4];
	dma_addr_t dma_addrs[4];
	unsigned int pitches[4], offsets[4];
	unsigned int format;
	int num_overlays;
	struct nx_plane_rect rect = { 0, };
	int i, ret;

	num_overlays = drm_format_num_planes(fb->format->format);
	for (i = 0; i < num_overlays; i++) {
		nx_obj[i] = nx_drm_fb_gem(fb, i);
		dma_addrs[i] = nx_obj[i]->dma_addr;
		offsets[i] = fb->offsets[i];
		pitches[i] = fb->pitches[i];
	}

	ret = plane_yuv_format(fb->format->format, &format);
	if (ret < 0)
		return ret;

	plane_yuv_rect(&rect, crtc_x, crtc_y, crtc_w, crtc_h,
			dp->width, dp->height);

	nx_overlay_set_format(ovl, format, 0, false);
	nx_overlay_set_position(ovl, src_x, src_y, src_w, src_h,
			rect.left, rect.top, rect.right - rect.left,
			rect.bottom - rect.top,
			false);
	ret = nx_display_plane_addr_yuv(ovl,
			dma_addrs, pitches, offsets, num_overlays, true);
	if (ret < 0)
		return ret;

	nx_overlay_enable(ovl, true);

	return 0;
}

static int nx_display_plane_update(struct drm_plane *plane,
				   struct drm_framebuffer *fb,
				   int crtc_x, int crtc_y,
				   int crtc_w, int crtc_h,
				   int src_x, int src_y, int src_w, int src_h)
{
	struct nx_drm_plane *nx_plane = to_nx_plane(plane);
	struct nx_overlay *ovl = nx_plane->context;
	struct drm_format_name_buf format_name;
	int ret;

	DRM_DEBUG_KMS("crtc.%d (%s)\n", ovl->dp->module, ovl->name);
	DRM_DEBUG_KMS("%s crtc:%d,%d,%d,%d, src:%d,%d,%d,%d\n",
		drm_get_format_name(fb->format->format, &format_name),
		crtc_x, crtc_y, crtc_w, crtc_h, src_x, src_y, src_w, src_h);

	if (!is_video_plane(ovl->type))
		ret = nx_display_plane_update_rgb(fb, ovl,
				crtc_x, crtc_y, crtc_w, crtc_h,
				src_x, src_y, src_w, src_h, nx_plane->align);
	else
		ret = nx_display_plane_update_yuv(fb, ovl,
				crtc_x, crtc_y, crtc_w, crtc_h,
				src_x, src_y, src_w, src_h);

	return ret;
}

static void nx_display_plane_disable(struct drm_plane *plane)
{
	nx_overlay_enable(to_nx_plane(plane)->context, false);
}

static void nx_display_plane_set_color(struct drm_plane *plane,
				       enum nx_overlay_color type,
				       unsigned int color,
				       bool enable)
{
	struct nx_overlay *ovl = to_nx_plane(plane)->context;

	if (type == NX_COLOR_COLORKEY)
		type = NX_COLOR_TRANS;

	nx_overlay_set_color(ovl, type, color, enable, true);
}

static void nx_display_plane_set_priority(struct drm_plane *plane, int priority)
{
	nx_overlay_set_priority(to_nx_plane(plane)->context, priority);
}

static int nx_display_plane_set_property(struct drm_plane *plane,
					 struct drm_plane_state *state,
					 struct drm_property *property,
					 uint64_t val)
{
	struct nx_overlay *ovl = to_nx_plane(plane)->context;
	struct nx_overlay_prop *prop = &ovl->property;
	union color *color = &prop->color;

	DRM_DEBUG_KMS("%s : %s 0x%llx\n", ovl->name, property->name, val);

	if (property == color->rgb.colorkey) {
		ovl->color.colorkey = val;
		nx_display_plane_set_color(plane, NX_COLOR_COLORKEY,
				ovl->color.colorkey, ovl->colorkey_on);
	}

	if (property == color->rgb.transcolor) {
		ovl->color.transcolor = val;
		nx_display_plane_set_color(plane, NX_COLOR_TRANS,
				ovl->color.transcolor, ovl->transcolor_on);
	}

	if (property == color->rgb.alphablend) {
		ovl->color.alphablend = val;
		nx_display_plane_set_color(plane, NX_COLOR_ALPHA,
				ovl->color.alphablend, ovl->alphablend_on);
	}

	if (property == color->rgb.colorkey_on) {
		ovl->colorkey_on = val ? true : false;
		nx_display_plane_set_color(plane, NX_COLOR_COLORKEY,
				ovl->color.colorkey,
				ovl->colorkey_on);
	}

	if (property == color->rgb.transcolor_on) {
		ovl->transcolor_on = val ? true : false;
		nx_display_plane_set_color(plane, NX_COLOR_TRANS,
				ovl->color.transcolor,
				ovl->transcolor_on);
	}

	if (property == color->rgb.alphablend_on) {
		ovl->alphablend_on = val ? true : false;
		nx_display_plane_set_color(plane, NX_COLOR_ALPHA,
				ovl->color.alphablend,
				ovl->alphablend_on);
	}

	if (property == prop->priority) {
		ovl->dp->video_priority = val;
		nx_display_plane_set_priority(plane, val);
	}

	return 0;
}

static int nx_display_plane_get_property(struct drm_plane *plane,
					 const struct drm_plane_state *state,
					 struct drm_property *property,
					 uint64_t *val)
{
	struct nx_overlay *ovl = to_nx_plane(plane)->context;
	struct nx_overlay_prop *prop = &ovl->property;
	union color *color = &prop->color;

	DRM_DEBUG_KMS("%s : %s\n", ovl->name, property->name);

	if (property == color->rgb.colorkey)
		*val = ovl->color.colorkey;

	if (property == color->rgb.transcolor)
		*val = ovl->color.transcolor;

	if (property == color->rgb.alphablend)
		*val = ovl->color.alphablend;

	if (property == color->rgb.colorkey_on)
		*val = ovl->colorkey_on ? 1 : 0;

	if (property == color->rgb.transcolor_on)
		*val = ovl->transcolor_on ? 1 : 0;

	if (property == color->rgb.alphablend_on)
		*val = ovl->alphablend_on ? 1 : 0;

	if (property == prop->priority)
		*val = ovl->dp->video_priority;

	return 0;
}

static void nx_display_plane_create_props(struct drm_device *drm,
					  struct drm_crtc *crtc,
					  struct drm_plane *plane,
					  struct drm_plane_funcs *plane_funcs)
{
	struct nx_overlay *ovl = to_nx_plane(plane)->context;
	struct nx_display *dp = ovl->dp;
	struct nx_overlay_prop *prop = &ovl->property;
	union color *color = &prop->color;

	DRM_DEBUG_KMS("crtc.%d plane.%d (%s)\n",
		ovl->dp->module, ovl->id, ovl->name);

	/* RGB */
	if (!is_video_plane(ovl->type)) {
		ovl->color.colorkey = dp->color_key;
		color->rgb.colorkey =
		drm_property_create_range(drm, 0, "colorkey", 0, 0xffffffff);
		drm_object_attach_property(&plane->base,
			color->rgb.colorkey, ovl->color.colorkey);
		color->rgb.transcolor =
		drm_property_create_range(drm, 0, "transcolor", 0, 0xffffffff);
		drm_object_attach_property(&plane->base,
			color->rgb.transcolor, ovl->color.transcolor);
		color->rgb.alphablend =
		drm_property_create_range(drm, 0, "alphablend",
			0, MAX_ALPHA_VALUE);
		drm_object_attach_property(&plane->base,
			color->rgb.alphablend, ovl->color.alphablend);
#ifdef CONFIG_DRM_NEXELL_PROPERTY_COLOR_ENABLE
		color->rgb.colorkey_on =
		drm_property_create_range(drm, 0, "colorkey-enable", 0, 1);
		drm_object_attach_property(&plane->base,
			color->rgb.colorkey_on, ovl->color.colorkey_on);
		color->rgb.transcolor_on =
		drm_property_create_range(drm, 0, "transcolor-enable", 0, 1);
		drm_object_attach_property(&plane->base,
			color->rgb.transcolor_on, ovl->color.transcolor_on);
		color->rgb.alphablend_on =
		drm_property_create_range(drm, 0, "alphablend-enable", 0, 1);
		drm_object_attach_property(&plane->base,
			color->rgb.alphablend_on, ovl->color.alphablend_on);
#endif
	}

	prop->priority =
	drm_property_create_range(drm, 0, "video-priority", 0, 2);
	drm_object_attach_property(&plane->base,
			prop->priority, dp->video_priority);

	/* plane property functions */
	plane_funcs->atomic_set_property = nx_display_plane_set_property;
	plane_funcs->atomic_get_property = nx_display_plane_get_property;
}

static void nx_display_plane_destory(struct drm_plane *plane)
{
	kfree(to_nx_plane(plane)->context);
}

static struct nx_drm_plane_ops nx_plane_ops = {
	.update = nx_display_plane_update,
	.disable = nx_display_plane_disable,
	.create_proeprties = nx_display_plane_create_props,
	.destroy = nx_display_plane_destory,
};

static struct nx_overlay *nx_display_plane_create(
			struct drm_device *drm, struct drm_crtc *crtc,
			struct nx_drm_plane *nx_plane, int id, bool yuv,
			bool bgr_mode)
{
	struct nx_overlay *ovl;
	struct nx_display *dp = to_nx_crtc(crtc)->context;

	ovl = kzalloc(sizeof(*ovl), GFP_KERNEL);
	if (!ovl)
		return NULL;

	ovl->dp = dp;
	ovl->base = dp->mlc_base;
	ovl->id = id;
	ovl->bgr_mode = bgr_mode;

	/* default alpha opacity */
	if (yuv) {
		ovl->type |= NX_PLANE_TYPE_VIDEO;
		ovl->color.alpha = MAX_ALPHA_VALUE;
	} else {
		ovl->color.alphablend = MAX_ALPHA_VALUE;
	}

	ovl->colorkey_on = dp->color_key_on;
	ovl->alphablend_on = dp->alpla_blend_on;
	ovl->transcolor_on = true;

	snprintf(ovl->name, sizeof(ovl->name),
		"%d-%s.%d", dp->module, yuv ? "vid" : "rgb", id);

	return ovl;
}

int nx_drm_planes_init(struct drm_device *drm, struct drm_crtc *crtc,
		       struct drm_plane **planes, int num_overlays,
		       enum drm_plane_type *drm_types, bool bgr_mode)
{
	struct nx_overlay *ovl;
	struct nx_plane_format *fmt;
	struct nx_display *dp = to_nx_crtc(crtc)->context;
	int id, n, i;

	for (n = 0, i = 0; i < num_overlays; i++) {
		struct nx_drm_plane *nx_plane = to_nx_plane(planes[i]);
		bool yuv;

		drm_types[i] = dp->overlay_types[i];

		if ((drm_types[i] & 0xfffffff) == NX_PLANE_TYPE_UNKNOWN)
			continue;

		yuv = is_video_plane(drm_types[i]);
		id = yuv ? NX_PLANE_VIDEO_LAYER : n++;

		ovl = nx_display_plane_create(drm, crtc, to_nx_plane(planes[i]),
					id, yuv, bgr_mode);
		if (!ovl)
			return -ENOMEM;

		list_add_tail(&ovl->list, &dp->overlay_list);

		fmt = &nx_plane_formats[yuv ? 1 : 0];

		nx_plane->context = ovl;
		nx_plane->formats = fmt->formats;
		nx_plane->format_count = fmt->format_count;
		nx_plane->ops = &nx_plane_ops;

		DRM_DEBUG_KMS("crtc.%d, %s\n", ovl->dp->module, ovl->name);
	}

	return num_overlays;
}

int nx_drm_encoder_init(struct drm_encoder *encoder)
{
	return 0;
}

static void nx_display_dump_par(struct nx_drm_display *display)
{
	struct nx_display_ctx *ctx = display_to_ctx(display);
	struct nx_display_par *dpp = &ctx->dpp;

	DRM_DEBUG_KMS("%s:\n", nx_panel_get_name(display->panel_type));
	DRM_DEBUG_KMS("SYNC -> LCD %d x %d mm\n",
		display->width_mm, display->height_mm);
	DRM_DEBUG_KMS("ha:%d, hs:%d, hb:%d, hf:%d\n",
	    display->vm.hactive, display->vm.hsync_len,
	    display->vm.hback_porch, display->vm.hfront_porch);
	DRM_DEBUG_KMS("va:%d, vs:%d, vb:%d, vf:%d\n",
		display->vm.vactive, display->vm.vsync_len,
	    display->vm.vback_porch, display->vm.vfront_porch);
	DRM_DEBUG_KMS("flags:0x%x\n", display->vm.flags);
	DRM_DEBUG_KMS("fmt:0x%x, inv:%d, swap:%d, yb:0x%x\n",
	    dpp->out_format, dpp->invert_field,
	    dpp->swap_rb, dpp->yc_order);
	DRM_DEBUG_KMS("dm:0x%x, drp:%d, dhs:%d, dvs:%d, dde:0x%x\n",
	    dpp->delay_mask, dpp->d_rgb_pvd,
	    dpp->d_hsync_cp1, dpp->d_vsync_frame, dpp->d_de_cp2);
	DRM_DEBUG_KMS("evs:%d, evb:%d, evf:%d\n",
	    dpp->evsync_len, dpp->evback_porch, dpp->evfront_porch);
	DRM_DEBUG_KMS("vso:%d, veo:%d, evso:%d, eveo:%d\n",
	    dpp->vstart_offs, dpp->vend_offs,
	    dpp->evstart_offs, dpp->evend_offs);
}

struct nx_drm_display *nx_drm_display_get(struct device *dev,
					  struct device_node *node,
					  struct drm_connector *connector,
					  enum nx_panel_type type)
{
	struct nx_drm_display *display;
	struct nx_display_ctx *ctx;

	display = kzalloc(sizeof(*display), GFP_KERNEL);
	if (!display)
		return NULL;

	switch (type) {
	#ifdef CONFIG_DRM_NEXELL_RGB
	case NX_PANEL_TYPE_RGB:
		ctx = nx_drm_display_rgb_get(dev, node, display);
		break;
	#endif
	#ifdef CONFIG_DRM_NEXELL_LVDS
	case NX_PANEL_TYPE_LVDS:
		ctx = nx_drm_display_lvds_get(dev, node, display);
		break;
	#endif
	default:
		DRM_ERROR("Failed support panel type: %d !!!\n", type);
		DRM_ERROR("Check config for panel: %s !!!\n",
			nx_panel_get_name(type));
		kfree(display);
		return NULL;
	}

	if (!ctx) {
		DRM_ERROR("Failed '%s' display interface !!!\n",
			nx_panel_get_name(type));
		kfree(display);
		return NULL;
	}

	BUG_ON(!display->context);

	ctx->panel_type = type;
	display->panel_type = type;
	display->connector = to_nx_connector(connector);

	DRM_DEBUG_KMS("get:%s [%d]\n", nx_panel_get_name(type), type);

	return display;
}

void nx_drm_display_put(struct device *dev, struct nx_drm_display *display)
{
	kfree(display->context);
	kfree(display);
}

int nx_drm_display_setup(struct nx_drm_display *display,
			 struct device_node *node, enum nx_panel_type type)
{
	struct nx_display_ctx *ctx = display_to_ctx(display);
	struct nx_display_par *dpp = &ctx->dpp;

	DRM_DEBUG_KMS("display :%s\n",
		nx_panel_get_name(nx_panel_get_type(display)));

	of_property_read_u32(node, "out-format", &dpp->out_format);
	of_property_read_u32(node, "invert-field", &dpp->invert_field);
	of_property_read_u32(node, "swap-rb", &dpp->swap_rb);
	of_property_read_u32(node, "yc-order", &dpp->yc_order);
	of_property_read_u32(node, "delay-mask", &dpp->delay_mask);
	of_property_read_u32(node, "delay-rgb-pvd", &dpp->d_rgb_pvd);
	of_property_read_u32(node, "delay-hs-cp1", &dpp->d_hsync_cp1);
	of_property_read_u32(node, "delay-vs-frame", &dpp->d_vsync_frame);
	of_property_read_u32(node, "delay-de-cp2", &dpp->d_de_cp2);
	of_property_read_u32(node, "evfront-porch", &dpp->evfront_porch);
	of_property_read_u32(node, "evback-porch", &dpp->evback_porch);
	of_property_read_u32(node, "evsync-len", &dpp->evsync_len);
	of_property_read_u32(node, "vsync-start-offs", &dpp->vstart_offs);
	of_property_read_u32(node, "vsync-end-offs", &dpp->vend_offs);
	of_property_read_u32(node, "evsync-start-offs", &dpp->evstart_offs);
	of_property_read_u32(node, "evsync-end-offs", &dpp->evend_offs);

	nx_display_dump_par(display);

	return 0;
}

dma_addr_t nx_drm_get_dma_addr(struct drm_plane *plane)
{
	struct nx_overlay *ovl;
	u32 addr;

	if (!plane)
		return 0;

	ovl = to_nx_plane(plane)->context;
	addr = nx_mlc_get_rgb_address(ovl->base, ovl->id);

	return (dma_addr_t)addr;
}
