// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/delay.h>
#include "low.h"

void nx_mlc_set_clock_pclk_mode(struct nx_mlc_reg *reg, enum nx_pclk_mode mode)
{
	u32 val;

	switch (mode) {
	case NX_PCLK_MODE_DYNAMIC:
		mode = 0;
		break;
	case NX_PCLK_MODE_ALWAYS:
		mode = 1;
		break;
	default:
		break;
	}

	val = readl(&reg->mlcclkenb) & ~(1 << 3);
	val |= (mode & 1) << 3;
	writel(val, &reg->mlcclkenb);
}

void nx_mlc_set_clock_bclk_mode(struct nx_mlc_reg *reg, enum nx_bclk_mode mode)
{
	u32 val;

	switch (mode) {
	case NX_BCLK_MODE_DISABLE:
		mode = 0;
		break;
	case NX_BCLK_MODE_DYNAMIC:
		mode = 2;
		break;
	case NX_BCLK_MODE_ALWAYS:
		mode = 3;
		break;
	default:
		break;
	}

	val = readl(&reg->mlcclkenb) & ~(0x3);
	val |= mode & 0x3;
	writel(val, &reg->mlcclkenb);
}

void nx_mlc_set_field_enable(struct nx_mlc_reg *reg, bool enb)
{
	u32 val = readl(&reg->mlccontrolt) & ~((1 << 0) | (1 << 3));

	writel(val | ((enb ? 1 : 0) << 0), &reg->mlccontrolt);
}

void nx_mlc_set_gamma_dither(struct nx_mlc_reg *reg, bool enb)
{
	u32 val = readl(&reg->mlcgammacont) & ~(1 << 0);

	writel(val | ((enb ? 1 : 0) << 0), &reg->mlcgammacont);
}

void nx_mlc_set_gamma_priority(struct nx_mlc_reg *reg, int priority)
{
	u32 val = readl(&reg->mlcgammacont) & ~(1 << 5);

	writel(val | (priority << 5), &reg->mlcgammacont);
}

void nx_mlc_set_power_mode(struct nx_mlc_reg *reg, bool power)
{
	int enb = power ? 1 : 0;
	u32 val;

	val = readl(&reg->mlccontrolt) & ~((1 << 11) | (1 << 10) | (1 << 3));
	writel(val | (enb << 11 | enb << 10), &reg->mlccontrolt);
}

void nx_mlc_set_background(struct nx_mlc_reg *reg, u32 color)
{
	writel(color, &reg->mlcbgcolor);
}

void nx_mlc_set_video_priority(struct nx_mlc_reg *reg,
			       enum nx_mlc_priority_vid priority)
{
	u32 val = readl(&reg->mlccontrolt) & ~((3 << 8) | (1 << 3));

	writel(val | (priority << 8), &reg->mlccontrolt);
}

void nx_mlc_set_screen_size(struct nx_mlc_reg *reg,
			    int width, int height)
{
	writel(((height - 1) << 16) | (width - 1), &reg->mlcscreensize);
}

void nx_mlc_set_enable(struct nx_mlc_reg *reg, bool enb)
{
	u32 val = readl(&reg->mlccontrolt) & ~((1 << 1) | (1 << 3));

	writel(val | ((enb ? 1 : 0) << 1), &reg->mlccontrolt);
}

void nx_mlc_set_dirty(struct nx_mlc_reg *reg)
{
	u32 val = readl(&reg->mlccontrolt);

	writel(val | (1 << 3), &reg->mlccontrolt);
}

void nx_mlc_set_layer_alpha(struct nx_mlc_reg *reg,
			    int layer, u32 alpha, bool enb)
{
	u32 val;

	if (layer == 0 || layer == 1) {
		val = readl(&reg->mlcrgblayer[layer].mlccontrol);
		val &= ~((1 << 2) | (1 << 4));
		val |= ((enb ? 1 : 0) << 2);
		writel(val, &reg->mlcrgblayer[layer].mlccontrol);

		val = readl(&reg->mlcrgblayer[layer].mlctpcolor) & ~(0xf << 28);
		val |= alpha << 28;
		writel(val, &reg->mlcrgblayer[layer].mlctpcolor);
	} else if (layer == 3) {
		val = readl(&reg->mlcvideolayer.mlccontrol);
		val &= ~((1 << 2) | (1 << 4));
		val |= ((enb ? 1 : 0) << 2);
		writel(val, &reg->mlcvideolayer.mlccontrol);
		writel(alpha << 28, &reg->mlcvideolayer.mlctpcolor);
	}
}

void nx_mlc_set_layer_lock_size(struct nx_mlc_reg *reg,
				int layer, u32 locksize)
{
	u32 val;

	if (layer != 0 && layer != 1)
		return;

	val = readl(&reg->mlcrgblayer[layer].mlccontrol);
	val &= ~((3 << 12) | (1 << 4));
	val |= ((locksize >> 3) << 12);
	writel(val, &reg->mlcrgblayer[layer].mlccontrol);
}

void nx_mlc_set_layer_position(struct nx_mlc_reg *reg,
			       int layer, int sx, int sy, int ex, int ey)
{
	void __iomem *lr, *tb;

	if (layer == 0 || layer == 1) {
		lr = &reg->mlcrgblayer[layer].mlcleftright;
		tb = &reg->mlcrgblayer[layer].mlctopbottom;
	} else if (layer == 2) {
		lr = &reg->mlcrgblayer2.mlcleftright;
		tb = &reg->mlcrgblayer2.mlctopbottom;
	} else if (layer == 3) {
		lr = &reg->mlcvideolayer.mlcleftright;
		tb = &reg->mlcvideolayer.mlctopbottom;
	} else {
		return;
	}

	writel(((sx & 0xffful) << 16) | (ex & 0xffful), lr);
	writel(((sy & 0xffful) << 16) | (ey & 0xffful), tb);
}

void nx_mlc_set_layer_enable(struct nx_mlc_reg *reg, int layer, bool enb)
{
	void __iomem *addr;
	u32 val;

	if (layer == 0 || layer == 1)
		addr = &reg->mlcrgblayer[layer].mlccontrol;
	else if (layer == 2)
		addr = &reg->mlcrgblayer2.mlccontrol;
	else if (layer == 3)
		addr = &reg->mlcvideolayer.mlccontrol;
	else
		return;

	val = readl(addr) & ~((1 << 5) | (1 << 4));
	val |= ((enb ? 1 : 0) << 5);
	writel(val, addr);
}

void nx_mlc_set_layer_dirty(struct nx_mlc_reg *reg, int layer, bool enb)
{
	void __iomem *addr;

	if (!enb)
		return;

	if (layer == 0 || layer == 1)
		addr = &reg->mlcrgblayer[layer].mlccontrol;
	else if (layer == 2)
		addr = &reg->mlcrgblayer2.mlccontrol;
	else if (layer == 3)
		addr = &reg->mlcvideolayer.mlccontrol;
	else
		return;

	writel(readl(addr) | (1 << 4), addr);
}

void nx_mlc_set_rgb_gamma_power(struct nx_mlc_reg *reg,
				int red, int green, int blue)
{
	u32 val = readl(&reg->mlcgammacont);

	val &= ~((1 << 11) | (1 << 9) | (1 << 3)); /* power */
	val &= ~((1 << 10) | (1 << 8) | (1 << 2)); /* sleep */

	val |= ((red << 3) | (green << 9) | (blue << 11)); /* power */
	val |= ((red << 2) | (green << 8) | (blue << 10)); /* sleep */

	writel(val, &reg->mlcgammacont);
}

void nx_mlc_set_rgb_gamma_enable(struct nx_mlc_reg *reg, bool enb)
{
	u32 val;

	val = readl(&reg->mlcgammacont) & ~(1 << 1);
	writel(val | ((enb ? 1 : 0) << 1), &reg->mlcgammacont);
}

void nx_mlc_set_rgb_transparency(struct nx_mlc_reg *reg,
				 int layer, u32 color, bool enb)
{
	u32 val;

	if (layer != 0 && layer != 1)
		return;

	val = readl(&reg->mlcrgblayer[layer].mlccontrol);
	val &= ~((1 << 0) | (1 << 4));
	val |= ((enb ? 1 : 0) << 0);
	writel(val, &reg->mlcrgblayer[layer].mlccontrol);

	val = readl(&reg->mlcrgblayer[layer].mlctpcolor);
	val &= ~(((1 << 24) - 1) << 0);
	val |= (color & (((1 << 24) - 1) << 0));

	writel(val, &reg->mlcrgblayer[layer].mlctpcolor);
}

void nx_mlc_set_rgb_color_inv(struct nx_mlc_reg *reg,
			      int layer, u32 color, bool enb)
{
	u32 val;

	if (layer != 0 && layer != 1)
		return;

	val = readl(&reg->mlcrgblayer[layer].mlccontrol);
	val &= ~((1 << 1) | (1 << 4));
	val |= ((enb ? 1 : 0) << 1);
	writel(val, &reg->mlcrgblayer[layer].mlccontrol);

	val = readl(&reg->mlcrgblayer[layer].mlcinvcolor);
	val &= ~(((1 << 24) - 1) << 0);
	val |= (color & (((1 << 24) - 1) << 0));
	writel(val, &reg->mlcrgblayer[layer].mlcinvcolor);
}

void nx_mlc_set_rgb_format(struct nx_mlc_reg *reg, int layer,
			   enum nx_mlc_format_rgb format)
{
	u32 val;
	u32 mask = 0xffff0000;

	if (layer != 0 && layer != 1)
		return;

	val = readl(&reg->mlcrgblayer[layer].mlccontrol);
	val &= ~(mask | (1 << 4));
	val |= (u32)format & mask;
	writel(val, &reg->mlcrgblayer[layer].mlccontrol);
}

void nx_mlc_set_rgb_invalid_position(struct nx_mlc_reg *reg,
				     int layer, int region,
				     int sx, int sy, int ex, int ey,
				     bool enb)
{
	if (layer != 0 && layer != 1)
		return;

	if (region == 0) {
		writel((((enb ? 1 : 0) << 28) |
			((sx & 0x7ff) << 16) | (ex & 0x7ff)),
		       &reg->mlcrgblayer[layer].mlcinvalidleftright0);
		writel((((sy & 0x7ff) << 16) | (ey & 0x7ff)),
		       &reg->mlcrgblayer[layer].mlcinvalidtopbottom0);
	} else {
		writel((((enb ? 1 : 0) << 28) |
			((sx & 0x7ff) << 16) | (ex & 0x7ff)),
		       &reg->mlcrgblayer[layer].mlcinvalidleftright1);
		writel((((sy & 0x7ff) << 16) | (ey & 0x7ff)),
		       &reg->mlcrgblayer[layer].mlcinvalidtopbottom1);
	}
}

void nx_mlc_set_rgb_stride(struct nx_mlc_reg *reg,
			   int layer, int hstride, int vstride)
{
	if (layer != 0 && layer != 1)
		return;

	writel(hstride, &reg->mlcrgblayer[layer].mlchstride);
	writel(vstride, &reg->mlcrgblayer[layer].mlcvstride);
}

void nx_mlc_set_rgb_address(struct nx_mlc_reg *reg,
			    int layer, u32 addr)
{
	if (layer != 0 && layer != 1)
		return;

	writel(addr, &reg->mlcrgblayer[layer].mlcaddress);
}

u32 nx_mlc_get_rgb_address(struct nx_mlc_reg *reg, int layer)
{
	return readl(&reg->mlcrgblayer[layer].mlcaddress);
}

void nx_mlc_set_vid_line_buffer_power(struct nx_mlc_reg *reg, bool power)
{
	u32 val;
	int enb = power ? 1 : 0;

	val = readl(&reg->mlcvideolayer.mlccontrol);
	val &= ~((1 << 15) | (1 << 14) | (1 << 4));
	val |= (enb << 15) | (enb << 14);

	writel(val, &reg->mlcvideolayer.mlccontrol);
}

void nx_mlc_set_vid_scale_filter(struct nx_mlc_reg *reg,
				 int bhlumaenb, int bhchromaenb, int bvlumaenb,
				 int bvchromaenb)
{
	u32 val;

	val = readl(&reg->mlcvideolayer.mlchscale) & ((1 << 23) - 1);
	val |= (bhlumaenb << 28) | (bhchromaenb << 29);
	writel(val, &reg->mlcvideolayer.mlchscale);

	val = readl(&reg->mlcvideolayer.mlcvscale) & ((1 << 23) - 1);
	val |= (bvlumaenb << 28) | (bvchromaenb << 29);
	writel(val, &reg->mlcvideolayer.mlcvscale);
}

void nx_mlc_get_vid_scale_filter(struct nx_mlc_reg *reg,
				 int *bhlumaenb,
				 int *bhchromaenb,
				 int *bvlumaenb,
				 int *bvchromaenb)
{
	u32 val;

	val = readl(&reg->mlcvideolayer.mlchscale);
	*bhlumaenb = (val >> 28) & 1;
	*bhchromaenb = (val >> 29) & 1;

	val = readl(&reg->mlcvideolayer.mlcvscale);
	*bvlumaenb = (val >> 28) & 1;
	*bvchromaenb = (val >> 29) & 1;
}

void nx_mlc_set_vid_format(struct nx_mlc_reg *reg,
			enum nx_mlc_format_vid format)
{
	u32 val;

	val = readl(&reg->mlcvideolayer.mlccontrol) & ~0xffff0000;
	val |= (u32)format;
	writel(val, &reg->mlcvideolayer.mlccontrol);
}

void nx_mlc_set_vid_scale(struct nx_mlc_reg *reg,
			  int sw, int sh, int dw, int dh,
			  int h_lu_enb, int h_ch_enb,
			  int v_lu_enb, int v_ch_enb)
{
	u32 hs = 0, vs = 0, cal;
	u32 val;

	if ((h_lu_enb || h_ch_enb) && (dw > sw))
		sw--, dw--;

	if (dw)
		hs = (sw << 11) / dw;

	if ((v_lu_enb || v_ch_enb) && (dh > sh)) {
		sh--, dh--;

		vs = (sh << 11) / dh;
		cal = ((vs * dh) >> 11);
		if (sh <= cal)
			vs--;
	} else {
		if (dh)
			vs = (sh << 11) / dh;
	}

	val = (h_lu_enb << 28) | (h_ch_enb << 29) | (hs & ((1 << 23) - 1));
	writel(val, &reg->mlcvideolayer.mlchscale);

	val = (v_lu_enb << 28) | (v_ch_enb << 29) | (vs & ((1 << 23) - 1));
	writel(val, &reg->mlcvideolayer.mlcvscale);
}

void nx_mlc_set_vid_address_yuyv(struct nx_mlc_reg *reg,
				 u32 addr, int stride)
{
	writel(addr, &reg->mlcvideolayer.mlcaddress);
	writel(stride, &reg->mlcvideolayer.mlcvstride);
}

void nx_mlc_set_vid_stride(struct nx_mlc_reg *reg,
			   int lu_stride, int cb_stride, int cr_stride)
{
	writel(lu_stride, &reg->mlcvideolayer.mlcvstride);
	writel(cb_stride, &reg->mlcvideolayer.mlcvstridecb);
	writel(cr_stride, &reg->mlcvideolayer.mlcvstridecr);
}

void nx_mlc_set_vid_address(struct nx_mlc_reg *reg,
			    u32 lu_addr, u32 cb_addr, u32 cr_addr)
{
	writel(lu_addr, &reg->mlcvideolayer.mlcaddress);
	writel(cb_addr, &reg->mlcvideolayer.mlcaddresscb);
	writel(cr_addr, &reg->mlcvideolayer.mlcaddresscr);
}

void nx_mlc_wait_vblank(struct nx_mlc_reg *reg, int layer)
{
	void __iomem *addr;
	int count = 30000;

	if (layer == 0 || layer == 1)
		addr = &reg->mlcrgblayer[layer].mlccontrol;
	else if (layer == 2)
		addr = &reg->mlcrgblayer2.mlccontrol;
	else if (layer == 3)
		addr = &reg->mlcvideolayer.mlccontrol;
	else
		return;

	while (readl(addr) & (1 << 5)) {
		if (0 > --count || !(readl(addr) & (1 << 4)))
			break;
		udelay(1);
	}
}

void nx_dpc_clear_interrupt_pending_all(struct nx_dpc_reg *reg)
{
	u32 val;

	val = readl(&reg->dpcctrl0) | 1 << 10;
	writel(val, &reg->dpcctrl0);
}

void nx_dpc_set_interrupt_enable_all(struct nx_dpc_reg *reg, bool enb)
{
	u32 val;

	val = readl(&reg->dpcctrl0) & ~((1 << 11) | (1 << 10));
	val |= (enb ? 1 : 0) << 11;

	writel(val, &reg->dpcctrl0);
}

int nx_dpc_get_interrupt_enable(struct nx_dpc_reg *reg)
{
	u32 val = readl(&reg->dpcctrl0) & (1 << 11);

	return (int)(val >> 11);
}

int nx_dpc_get_interrupt_pending(struct nx_dpc_reg *reg)
{
	u32 val = readl(&reg->dpcctrl0) & (1 << 10);

	return (int)(val >> 10);
}

void nx_dpc_set_mode(struct nx_dpc_reg *reg,
		     enum nx_dpc_outformat outformat,
		     bool interlace,
		     bool rgbmode, bool swaprb,
		     bool embsync,
		     enum nx_dpc_ycorder ycorder)
{
	u32 val;
	const u32 format_table[] = {
		[NX_DPC_FORMAT_RGB555] = (0<<0),
		[NX_DPC_FORMAT_RGB565] = (1<<0),
		[NX_DPC_FORMAT_RGB666] = (2<<0),
		[NX_DPC_FORMAT_RGB888] = (3<<0),
		[NX_DPC_FORMAT_MRGB555A] = (4<<0),
		[NX_DPC_FORMAT_MRGB555B] = (5<<0),
		[NX_DPC_FORMAT_MRGB565] = (6<<0),
		[NX_DPC_FORMAT_MRGB666] = (7<<0),
		[NX_DPC_FORMAT_MRGB888A] = (8<<0),
		[NX_DPC_FORMAT_MRGB888B] = (9<<0),
		[NX_DPC_FORMAT_BGR555] = (0<<0)|(1<<7),
		[NX_DPC_FORMAT_BGR565] = (1<<0)|(1<<7),
		[NX_DPC_FORMAT_BGR666] = (2<<0)|(1<<7),
		[NX_DPC_FORMAT_BGR888] = (3<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR555A] = (4<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR555B] = (5<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR565] = (6<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR666] = (7<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR888A] = (8<<0)|(1<<7),
		[NX_DPC_FORMAT_MBGR888B] = (9<<0)|(1<<7),
		[NX_DPC_FORMAT_CCIR656] = (10<<0),
		[NX_DPC_FORMAT_CCIR601_8] = (11<<0),
		[NX_DPC_FORMAT_CCIR601_16A] = (12<<0),
		[NX_DPC_FORMAT_CCIR601_16B] = (13<<0),
		[NX_DPC_FORMAT_SRGB888] = (14<<0),
		[NX_DPC_FORMAT_SRGBD8888] = (15<<0)
	};

	val = readl(&reg->dpcctrl0);
	val &= ~((1 << 12) | (1 << 9) | (1 << 8));

	if (rgbmode)
		val |= (1 << 12);

	if (interlace)
		val |= (1 << 9);

	if (embsync)
		val |= (1 << 8);

	writel(val, &reg->dpcctrl0);

	val = readl(&reg->dpcctrl1) & 0x30ff;
	val &= ~((1 << 15) | (1 << 13) | (1 << 6));

	if (swaprb)
		val |=  (1 << 15);

	if (!embsync)
		val |= (1 << 13);

	val |= ((ycorder & 0x3) << 6);
	val |= (format_table[outformat] << 8);
	writel(val, &reg->dpcctrl1);
}

void nx_dpc_set_hsync(struct nx_dpc_reg *reg, u32 avwidth, u32 hsw,
		      u32 hfp, u32 hbp, bool invhsync)
{
	u32 val;

	writel((hsw + hbp + avwidth + hfp - 1), &reg->dpchtotal);
	writel((hsw - 1), &reg->dpchswidth);
	writel((hsw + hbp - 1), &reg->dpchastart);
	writel((hsw + hbp + avwidth - 1), &reg->dpchaend);

	val = readl(&reg->dpcctrl0) & ~(1 << 10);

	if (invhsync)
		val |=  (1 << 0);
	else
		val &= ~(1 << 0);

	writel(val, &reg->dpcctrl0);
}

void nx_dpc_set_vsync(struct nx_dpc_reg *reg,
		      u32 avheight, u32 vsw, u32 vfp, u32 vbp,
		      u32 eavheight, u32 evsw, u32 evfp,
		      u32 evbp, bool invvsync)
{
	u32 val;

	writel((vsw + vbp + avheight + vfp - 1), &reg->dpcvtotal);
	writel((vsw - 1), &reg->dpcvswidth);
	writel((vsw + vbp - 1), &reg->dpcvastart);
	writel((vsw + vbp + avheight - 1), &reg->dpcvaend);
	writel((evsw + evbp + eavheight + evfp - 1), &reg->dpcevtotal);
	writel((evsw - 1), &reg->dpcevswidth);
	writel((evsw + evbp - 1), &reg->dpcevastart);
	writel((evsw + evbp + eavheight - 1), &reg->dpcevaend);

	val = readl(&reg->dpcctrl0) & ~(1 << 10);

	if (invvsync)
		val |= (1 << 1);
	else
		val &= ~(1 << 1);

	writel(val, &reg->dpcctrl0);
}

void nx_dpc_set_vsync_offset(struct nx_dpc_reg *reg,
			     u32 vssoffset, u32 vseoffset,
			     u32 evssoffset, u32 evseoffset)
{
	writel(vseoffset, &reg->dpcvseoffset);
	writel(vssoffset, &reg->dpcvssoffset);
	writel(evseoffset, &reg->dpcevseoffset);
	writel(evssoffset, &reg->dpcevssoffset);
}

void nx_dpc_set_sync(struct nx_dpc_reg *reg, bool interlace,
		     int avwidth, int avheight,
		     int hsw, int hfp, int hbp, int vsw, int vfp, int vbp,
		     int vso, int veo,
		     enum nx_dpc_polarity field_pol,
		     enum nx_dpc_polarity hs_pol,
		     enum nx_dpc_polarity vs_pol,
		     int evsw, int evfp, int evbp,
		     int evso, int eveo)
{
	u32 val;
	int htotal = hfp + hsw + hbp + avwidth - 1;
	int vtotal = vfp + vsw + vbp + avheight - 1;

	writel(htotal, &reg->dpchtotal);
	writel(hsw - 1, &reg->dpchswidth);
	writel(hsw + hbp - 1, &reg->dpchastart);
	writel(hsw + hbp + avwidth - 1, &reg->dpchaend);

	writel(vtotal, &reg->dpcvtotal);
	writel(vsw - 1, &reg->dpcvswidth);
	writel(vsw + vbp - 1, &reg->dpcvastart);
	writel(vsw + vbp + avheight - 1, &reg->dpcvaend);

	writel(veo, &reg->dpcvseoffset);
	writel(htotal - vso, &reg->dpcvssoffset);

	if (interlace) {
		writel(eveo, &reg->dpcevseoffset);
		writel(htotal - evso, &reg->dpcevssoffset);
		writel((evfp + evsw + evbp + avheight - 1), &reg->dpcevtotal);
		writel((evsw - 1), &reg->dpcevswidth);
		writel((evsw + evbp - 1), &reg->dpcevastart);
		writel((evsw + evbp + avheight - 1), &reg->dpcevaend);
	}

	val = readl(&reg->dpcctrl0) & 0xfff0;
	val |= ((field_pol << 2) | (vs_pol << 1) | (hs_pol << 0));
	writel(val, &reg->dpcctrl0);
}

void nx_dpc_set_delay(struct nx_dpc_reg *reg,
		      int delay_rgb_pvd, int delay_hs_cp1,
		      int delay_vs_frm, int delay_de_cp2)
{
	u32 val = readl(&reg->dpcctrl0) & ~((1 << 10) | (0xf << 4));

	val |= (delay_rgb_pvd << 4);
	writel(val, &reg->dpcctrl0);

	writel((delay_vs_frm << 8) | (delay_hs_cp1 << 0), &reg->dpcdelay0);
	writel((delay_de_cp2 << 0), &reg->dpcdelay1);
}

void nx_dpc_set_dither(struct nx_dpc_reg *reg, enum nx_dpc_dither dither_r,
		       enum nx_dpc_dither dither_g,
		       enum nx_dpc_dither dither_b)
{
	u32 val = readl(&reg->dpcctrl1) & ~0x3f;

	val |= (dither_b << 4) | (dither_g << 2) | (dither_r << 0);
	writel(val, &reg->dpcctrl1);
}

void nx_dpc_set_reg_flush(struct nx_dpc_reg *reg)
{
	u32 val = readl(&reg->dpcdataflush);

	writel(val | (1 << 4), &reg->dpcdataflush);
}

void nx_dpc_set_enable(struct nx_dpc_reg *reg, bool enb)
{
	u32 val = readl(&reg->dpcctrl0) & ~((1 << 10) | (1 << 15));

	val |= (enb ? 1 : 0) << 15;
	writel(val, &reg->dpcctrl0);
}

int nx_dpc_get_enable(struct nx_dpc_reg *reg)
{
	u32 val = readl(&reg->dpcctrl0) & (1 << 15);

	return (int)(val >> 15);
}
