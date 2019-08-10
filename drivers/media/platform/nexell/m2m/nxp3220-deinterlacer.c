// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/mutex.h>

#include <uapi/linux/nx-deinterlacer.h>

#define NX_DEINTERLACER_DEV_NAME "nx-deinterlacer"

#define TIMEOUT_STOP	(HZ/10) /* 100ms */

#ifdef FILE_DUMP
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#endif

enum {
	PROCESSING_STOP,
	PROCESSING_START
};

struct nx_deinterlace_register_set {
	u32 start;
	u32 mode;
	u32 intenb;
	u32 intpend;
	u32 tspara;
	u32 tmpara;
	u32 tipara;
	u32 tpara;
	u32 blendpara;
	u32 reserved_0[(0x100 - 0x024) / 4];

	u32 srcsize_y;
	u32 srcaddrp_y;
	u32 srcaddrc_y;
	u32 srcaddrn_y;
	u32 srcstrd_y;
	u32 destaddrf_y;
	u32 destaddrd_y;
	u32 deststrd_y;
	u32 reserved_1[(0x200 - 0x120) / 4];

	u32 srcsize_cb;
	u32 srcaddrc_cb;
	u32 srcstrd_cb;
	u32 destaddrf_cb;
	u32 destaddrd_cb;
	u32 deststrd_cb;
	u32 reserved_2[(0x300 - 0x218) / 4];

	u32 srcsize_cr;
	u32 srcaddrc_cr;
	u32 srcstrd_cr;
	u32 destaddrf_cr;
	u32 destaddrd_cr;
	u32 deststrd_cr;
};

struct nx_deinterlacer {
	atomic_t open_count;
	atomic_t status;

	wait_queue_head_t wq_start;
	wait_queue_head_t wq_end;

	int irq;
	void *base;
	struct clk *clk_axi;
	struct clk *clk_apb;
	struct platform_device *pdev;
	char irq_name[20];

	struct mutex mutex;
	struct miscdevice	miscdev;

	struct nx_deinterlace_register_set *reg;
};

#define DUMP_REG(dev, name) \
	dev_dbg(dev, "\t"#name "\t(0x%08x) = 0x%08x\n", \
	*((unsigned int *)&(pReg->name)), readl(&(pReg->name)))

void dump_deinterlace_register(struct nx_deinterlacer *me)
{
	struct device *dev = &me->pdev->dev;
	struct nx_deinterlace_register_set *pReg =
		(struct nx_deinterlace_register_set *)me->base;
	dev_dbg(dev, "====================================================\n");
	dev_dbg(dev, "                DEINTERLACE REGISTER DUMP\n");
	dev_dbg(dev, "====================================================\n");
	DUMP_REG(dev, start);
	DUMP_REG(dev, mode);
	DUMP_REG(dev, intenb);
	DUMP_REG(dev, intpend);
	DUMP_REG(dev, tspara);
	DUMP_REG(dev, tmpara);
	DUMP_REG(dev, tipara);
	DUMP_REG(dev, tpara);
	DUMP_REG(dev, blendpara);
	DUMP_REG(dev, srcsize_y);
	DUMP_REG(dev, srcaddrp_y);
	DUMP_REG(dev, srcaddrc_y);
	DUMP_REG(dev, srcaddrn_y);
	DUMP_REG(dev, srcstrd_y);
	DUMP_REG(dev, destaddrf_y);
	DUMP_REG(dev, destaddrd_y);
	DUMP_REG(dev, deststrd_y);
	DUMP_REG(dev, srcsize_cb);
	DUMP_REG(dev, srcaddrc_cb);
	DUMP_REG(dev, srcstrd_cb);
	DUMP_REG(dev, destaddrf_cb);
	DUMP_REG(dev, destaddrd_cb);
	DUMP_REG(dev, deststrd_cb);
	DUMP_REG(dev, srcsize_cr);
	DUMP_REG(dev, srcaddrc_cr);
	DUMP_REG(dev, srcstrd_cr);
	DUMP_REG(dev, destaddrf_cr);
	DUMP_REG(dev, destaddrd_cr);
	DUMP_REG(dev, deststrd_cr);
}

static int get_phy_addr_from_fd(struct device *dev, int fd, bool is_src,
				dma_addr_t *phy_addr)
{
	struct dma_buf	*dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	u32 direction;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		dev_err(dev, "%s: can't get dambuf : fd = %d\n", __func__, fd);
		return -EINVAL;
	}

	attach = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attach)) {
		dev_err(dev, "fail to attach dmabuf\n");
		return -EINVAL;
	}

	if (is_src)
		direction = DMA_TO_DEVICE;
	else
		direction = DMA_FROM_DEVICE;

	sgt = dma_buf_map_attachment(attach, direction);
	if (IS_ERR(sgt)) {
		dev_err(dev, "Error getting dmabuf scatterlist\n");
		return -EINVAL;
	}

	*phy_addr = sg_dma_address(sgt->sgl);

	dma_buf_unmap_attachment(attach, sgt, direction);
	dma_buf_detach(dmabuf, attach);
	dma_buf_put(dmabuf);

	return 0;
}

static void nx_deinterlace_set_ysrc_image_size(
		struct nx_deinterlace_register_set *reg,
		u32 ysrcheight,
		u32 ysrcwidth)
{
	u32 temp;
	const u32 yheight_bitpos = 16;
	const u32 ywidth_bitpos = 0;

	temp = (ysrcheight << yheight_bitpos) | (ysrcwidth << ywidth_bitpos);

	writel(temp, &reg->srcsize_y);
}

static void nx_deinterlace_set_ysrc_addr_prev(
		struct nx_deinterlace_register_set *reg, u32 ysrcaddrprev)
{
	writel(ysrcaddrprev, &reg->srcaddrp_y);
}

static void nx_deinterlace_set_ysrc_addr_curr(
		struct nx_deinterlace_register_set *reg, u32 ysrcaddrcurr)
{
	writel(ysrcaddrcurr, &reg->srcaddrc_y);
}

static void nx_deinterlace_set_ysrc_addr_next(
		struct nx_deinterlace_register_set *reg, u32 ysrcaddrnext)
{
	writel(ysrcaddrnext, &reg->srcaddrn_y);
}

static void nx_deinterlace_set_ysrc_stride(
		struct nx_deinterlace_register_set *reg, u32 ysrcstride)
{
	writel(ysrcstride, &reg->srcstrd_y);
}

static void nx_deinterlace_set_ydest_addr_dit(
		struct nx_deinterlace_register_set *reg, u32 ydestaddrdit)
{
	writel(ydestaddrdit, &reg->destaddrd_y);
}

static void nx_deinterlace_set_ydest_stride(
		struct nx_deinterlace_register_set *reg, u32 ydeststride)
{
	writel(ydeststride, &reg->deststrd_y);
}

static void nx_deinterlace_set_cbsrc_image_size(
		struct nx_deinterlace_register_set *reg,
		u32 cbsrcheight, u32 cbsrcwidth)
{
	const u32 cbheight_bitpos = 16;
	const u32 cbwidth_bitpos = 0;
	u32 temp;

	temp =
	    (cbsrcheight << cbheight_bitpos) | (cbsrcwidth << cbwidth_bitpos);

	writel(temp, &reg->srcsize_cb);
}

static void nx_deinterlace_set_cbsrc_addr_curr(
		struct nx_deinterlace_register_set *reg,
		u32 cbsrcaddrcurr)
{
	writel(cbsrcaddrcurr, &reg->srcaddrc_cb);
}

static void nx_deinterlace_set_cbsrc_stride(
		struct nx_deinterlace_register_set *reg, u32 cbsrcstride)
{
	writel(cbsrcstride, &reg->srcstrd_cb);
}

static void nx_deinterlace_set_cbdest_addr_dit(
		struct nx_deinterlace_register_set *reg,
		u32 cbdestaddrdit)
{
	writel(cbdestaddrdit, &reg->destaddrd_cb);
}

static void nx_deinterlace_set_cbdest_addr_fil(
		struct nx_deinterlace_register_set *reg,
		u32 cbdestaddrfil)
{
	writel(cbdestaddrfil, &reg->destaddrf_cb);
}

static void nx_deinterlace_set_cbdest_stride(
		struct nx_deinterlace_register_set *reg, u32 cbdeststride)
{
	writel(cbdeststride, &reg->deststrd_cb);
}

static void nx_deinterlace_set_crsrc_image_size(
		struct nx_deinterlace_register_set *reg, u32 crsrcheight,
		u32 crsrcwidth)
{
	const u32 crheight_bitpos = 16;
	const u32 crwidth_bitpos = 0;
	u32 temp;

	temp =
	    (crsrcheight << crheight_bitpos) | (crsrcwidth << crwidth_bitpos);

	writel(temp, &reg->srcsize_cr);
}

static void nx_deinterlace_set_crsrc_addr_curr(
		struct nx_deinterlace_register_set *reg, u32 crsrcaddrcurr)
{
	writel(crsrcaddrcurr, &reg->srcaddrc_cr);
}

static void nx_deinterlace_set_crsrc_stride(
		struct nx_deinterlace_register_set *reg, u32 crsrcstride)
{
	writel(crsrcstride, &reg->srcstrd_cr);
}

static void nx_deinterlace_set_crdest_addr_dit(
		struct nx_deinterlace_register_set *reg, u32 crdestaddrdit)
{
	writel(crdestaddrdit, &reg->destaddrd_cr);
}

static void nx_deinterlace_set_crdest_addr_fil(
		struct nx_deinterlace_register_set *reg, u32 crdestaddrfil)
{
	writel(crdestaddrfil, &reg->destaddrf_cr);
}

static void nx_deinterlace_set_crdest_stride(
		struct nx_deinterlace_register_set *reg, u32 crdeststride)
{
	writel(crdeststride, &reg->deststrd_cr);
}

static void nx_deinterlace_set_asparameter(
		struct nx_deinterlace_register_set *reg, u32 dwts1, u32 dwts2)
{
	const u32 ts2_bitpos = 16;

	writel(((dwts2 << ts2_bitpos) | dwts1), &reg->tspara);
}

static void nx_deinterlace_set_mdsadparameter(
		struct nx_deinterlace_register_set *reg, u32 dwtm1, u32 dwtm2)
{
	const u32 tm2_bitpos = 16;

	writel(((dwtm2 << tm2_bitpos) | dwtm1), &reg->tmpara);
}

static void nx_deinterlace_set_miparameter(
		struct nx_deinterlace_register_set *reg, u32 dwti1, u32 dwti2)
{
	const u32 ti2_bitpos = 16;

	writel(((dwti2 << ti2_bitpos) | dwti1), &reg->tipara);
}

static void nx_deinterlace_set_ysparameter(
		struct nx_deinterlace_register_set *reg, u32 dwt1, u32 dwt2)
{
	const u32 t2_bitpos = 16;

	writel(((dwt2 << t2_bitpos) | dwt1), &reg->tpara);
}

static void nx_deinterlace_set_blendparameter(
		struct nx_deinterlace_register_set *reg, u32 dwshift)
{
	writel(dwshift, &reg->blendpara);
}

static void nx_deinterlace_set_ydest_addr_fil(
		struct nx_deinterlace_register_set *reg, u32 ydestaddrfil)
{
	writel(ydestaddrfil, &reg->destaddrf_y);
}

static void nx_deinterlace_set_ycbcrenable(
		struct nx_deinterlace_register_set *reg, int yenable,
		int cbenable, int crenable)
{
	const u32 start_mask = (1 << 0);
	const u32 idle_mask = (1 << 1);
	const u32 enb_mask = (0x7ul << 8);
	const u32 yenb_mask = (0x01ul << 8);
	const u32 cbenb_mask = (0x01ul << 9);
	const u32 crenb_mask = (0x01ul << 10);
	u32 temp;
	struct nx_deinterlace_register_set *pregister;

	pregister = reg;
	temp = pregister->mode;
	temp &= ~(enb_mask | start_mask | idle_mask);

	if (true == yenable)
		temp |= yenb_mask;

	if (true == cbenable)
		temp |= cbenb_mask;

	if (true == crenable)
		temp |= crenb_mask;

	writel(temp, &pregister->mode);
}

static void nx_deinterlace_set_ycbcrfield(
				struct nx_deinterlace_register_set *reg,
				enum nx_deinter_src_field yfield,
				enum nx_deinter_src_field cbfield,
				enum nx_deinter_src_field crfield)
{
	const u32 start_mask = (1 << 0);
	const u32 idle_mask = (1 << 1);
	const u32 field_mask = (0x7 << 12);
	const u32 y_bitpos = 12;
	const u32 cb_bitpos = 13;
	const u32 cr_bitpos = 14;
	u32 temp;
	struct nx_deinterlace_register_set *pregister;

	pregister = reg;
	temp = pregister->mode;
	temp &= ~(field_mask | start_mask | idle_mask);
	temp |= ((u32)yfield << y_bitpos) | ((u32)cbfield << cb_bitpos) |
		((u32)crfield << cr_bitpos);

	writel(temp, &pregister->mode);
}

static void nx_deinterlace_deinterlace_start(
		struct nx_deinterlace_register_set *reg)
{
	const u32 start_mask = (0x01 << 0);
	const u32 idle_mask = (0x01 << 1);
	u32 temp;
	struct nx_deinterlace_register_set *pregister;

	pregister = reg;
	temp = pregister->mode;
	temp = (temp & ~idle_mask) | start_mask;

	writel(temp, &pregister->start);
}

static void nx_deinterlace_set_interrupt_enable_all(
		struct nx_deinterlace_register_set *reg, bool enable)
{
	struct nx_deinterlace_register_set *pregister;
	const u32 intenb_mask = 0xf;

	pregister = reg;

	if (true == enable)
		writel(intenb_mask, &pregister->intenb);
	else
		writel(0x00, &pregister->intenb);
}

static void nx_deinterlace_clear_interrupt_pending_all(
		struct nx_deinterlace_register_set *reg)
{
	const u32 intpend_mask = 0xf;

	writel(intpend_mask, &reg->intpend);
}

static void nx_deinterlace_set_interrupt_enable(
		struct nx_deinterlace_register_set *reg, int32_t int_num,
		bool enable)
{
	u32 read_value;

	read_value = reg->intenb;
	read_value &= ~(1 << int_num);
	read_value |= ((u32)enable << int_num);

	writel(read_value, &reg->intenb);
}

static void _set_deinterlacer(struct nx_deinterlacer *me, u16 height,
		u16 width, u32 y_prev_addr, u32 y_curr_addr, u32 y_next_addr,
		u32 y_src_stride, u32 y_dst_addr, u32 y_dst_stride,
		u32 cb_curr_addr, u32 cb_src_stride, u32 cb_dst_addr,
		u32 cb_dst_stride, u32 cr_curr_addr, u32 cr_src_stride,
		u32 cr_dst_addr, u32 cr_dst_stride, int is_odd)
{
	enum nx_deinter_src_field nx_deinter_src_field_odd = FIELD_ODD;
	enum nx_deinter_src_field nx_deinter_src_field_even = FIELD_EVEN;

	u32 ydst_field_stride	= (y_dst_stride * 2);
	u32 cb_dst_field_stride = (cb_dst_stride * 2);
	u32 cr_dst_field_stride = (cr_dst_stride * 2);
	u16 cwidth = (u16)(width / 2);
	u32 cheight = (u16)(height / 2);

	/*	Y Register Setting	*/
	nx_deinterlace_set_ysrc_image_size(me->reg, height, width);
	nx_deinterlace_set_ysrc_addr_prev(me->reg, y_prev_addr);
	nx_deinterlace_set_ysrc_addr_curr(me->reg, y_curr_addr);
	nx_deinterlace_set_ysrc_addr_next(me->reg, y_next_addr);
	nx_deinterlace_set_ysrc_stride(me->reg, y_src_stride);
	nx_deinterlace_set_ydest_stride(me->reg, ydst_field_stride);

	/*	CB Regiseter Setting	*/
	nx_deinterlace_set_cbsrc_image_size(me->reg, cheight, cwidth);
	nx_deinterlace_set_cbsrc_addr_curr(me->reg, cb_curr_addr);
	nx_deinterlace_set_cbsrc_stride(me->reg, cb_src_stride);
	nx_deinterlace_set_cbdest_stride(me->reg, cb_dst_field_stride);

	/*	CR Regiseter Setting	*/
	nx_deinterlace_set_crsrc_image_size(me->reg, cheight, cwidth);
	nx_deinterlace_set_crsrc_addr_curr(me->reg, cr_curr_addr);
	nx_deinterlace_set_crsrc_stride(me->reg, cr_src_stride);
	nx_deinterlace_set_crdest_stride(me->reg, cr_dst_field_stride);

	/*	Parameter Setting	*/
	nx_deinterlace_set_asparameter(me->reg, 10, 18);
	nx_deinterlace_set_mdsadparameter(me->reg, 8, 16);
	nx_deinterlace_set_miparameter(me->reg, 50, 306);
	nx_deinterlace_set_ysparameter(me->reg, 434, 466);
	nx_deinterlace_set_blendparameter(me->reg, 3);

	if (is_odd) {
		/*	Y Register Set	*/
		nx_deinterlace_set_ydest_addr_dit(me->reg, (y_dst_addr +
			y_dst_stride));
		nx_deinterlace_set_ydest_addr_fil(me->reg, y_dst_addr);
		/*	CB Register Set	*/
		nx_deinterlace_set_cbdest_addr_dit(me->reg, (cb_dst_addr +
			cb_dst_stride));
		nx_deinterlace_set_cbdest_addr_fil(me->reg, cb_dst_addr);
		/*	CR Register Set	*/
		nx_deinterlace_set_crdest_addr_dit(me->reg, (cr_dst_addr +
			cr_dst_stride));
		nx_deinterlace_set_crdest_addr_fil(me->reg, cr_dst_addr);
		/*	Start	*/
		nx_deinterlace_set_ycbcrenable(me->reg, true, true, true);
		nx_deinterlace_set_ycbcrfield(me->reg,
				nx_deinter_src_field_even,
				nx_deinter_src_field_even,
				nx_deinter_src_field_even);
	} else {
		/*	Y Register Set	*/
		nx_deinterlace_set_ydest_addr_dit(me->reg, y_dst_addr);
		nx_deinterlace_set_ydest_addr_fil(me->reg, (y_dst_addr +
					y_dst_stride));
		/*	CB Register Set	*/
		nx_deinterlace_set_cbdest_addr_dit(me->reg, cb_dst_addr);
		nx_deinterlace_set_cbdest_addr_fil(me->reg, (cb_dst_addr +
			cb_dst_stride));
		/*	CR Register Set	*/
		nx_deinterlace_set_crdest_addr_dit(me->reg, cr_dst_addr);
		nx_deinterlace_set_crdest_addr_fil(me->reg, (cr_dst_addr +
			cr_dst_stride));
		/*	Start	*/
		nx_deinterlace_set_ycbcrenable(me->reg, true, true, true);
		nx_deinterlace_set_ycbcrfield(me->reg,
				nx_deinter_src_field_odd,
				nx_deinter_src_field_odd,
				nx_deinter_src_field_odd);
	}
}

#ifdef FILE_DUMP
static struct dma_buf *mmap_vaddr_from_fd(int fd, void **virt)
{
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		dev_err(dev, "%s: can't get dambuf : fd{%d}\n", __func__, fd);
		return NULL;
	}

	*virt = dma_buf_vmap(dmabuf);

	return dmabuf;
}

static void munmap_vaddr_from_fd(struct dma_buf *dmabuf, void *virt)
{
	dma_buf_vunmap(dmabuf, virt);
	dma_buf_put(dmabuf);
}

void write_file(char *filename, char *data, long lFileSize)
{
	struct file *file;
	loff_t pos = 0;
	int fd;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_WRONLY|O_CREAT, 0644);

	if (fd >= 0) {
		sys_write(fd, data, lFileSize);
		file = fget(fd);

		if (file) {
			vfs_write(file, data, lFileSize, &pos);
			fput(file);
		}
		sys_close(fd);
	}
	set_fs(old_fs);
}

static void remove_stride(int w, int h, int align_factor, void *dst, void *src)
{
	int i;
	uint32_t y_stride, h_stride, c_stride, w_align, h_align = 16;
	uint32_t y_src_pos, cb_src_pos, cr_src_pos;
	uint32_t y_dst_pos, cb_dst_pos, cr_dst_pos;

	w_align = align_factor;

	y_stride = ALIGN(w, w_align);
	c_stride = ALIGN(w >> 1, w_align >> 1);
	h_stride = ALIGN(h, h_align);

	y_src_pos = 0;
	cb_src_pos = y_src_pos + (y_stride * h_stride);
	cr_src_pos = cb_src_pos + (c_stride * ALIGN(h >> 1, h_align >> 1));

	y_dst_pos = 0;
	cb_dst_pos = y_dst_pos + (w * h);
	cr_dst_pos = cb_dst_pos + ((w >> 1) * (h >> 1));

	for (i = 0; i < h; i++) {
		/* y */
		memcpy(dst + y_dst_pos, src + y_src_pos, w);
		y_src_pos += y_stride;
		y_dst_pos += w;

		if (i < (h >> 1)) {
			/* cb */
			memcpy(dst + cb_dst_pos, src + cb_src_pos,
					(w >> 1));
			cb_src_pos += c_stride;
			cb_dst_pos += (w >> 1);

			/* cr */
			memcpy(dst + cr_dst_pos, src + cr_src_pos,
					(w >> 1));
			cr_src_pos += c_stride;
			cr_dst_pos += (w >> 1);
		}
	}
}
#endif


static void _set_and_run(struct nx_deinterlacer *me,
		struct frame_data_info *frame)
{
	struct device *dev = &me->pdev->dev;
	unsigned long src_prev_y_data_phy, src_curr_y_data_phy;
	unsigned long src_next_y_data_phy, src_curr_cb_data_phy;
	unsigned long src_curr_cr_data_phy;
	unsigned long dst_y_data_phy, dst_cb_data_phy, dst_cr_data_phy;
	int width, height, src_y_stride, src_c_stride;
	int dst_y_stride, dst_c_stride;
	int dst_y_data_size = 0;
	int dst_cb_data_size = 0;
	int dst_cr_data_size = 0;

	width = frame->width;
	height = (frame->height / 2);

	src_y_stride = frame->src_bufs[0].plane3.src_stride[0];
	src_c_stride = frame->src_bufs[0].plane3.src_stride[1];

	dst_y_stride = frame->dst_bufs[0].plane3.dst_stride[0];
	dst_c_stride = frame->dst_bufs[0].plane3.dst_stride[1];

	src_prev_y_data_phy = frame->src_bufs[0].plane3.phys[0];
	src_curr_y_data_phy = frame->src_bufs[1].plane3.phys[0];
	src_next_y_data_phy = frame->src_bufs[2].plane3.phys[0];

	src_curr_cb_data_phy = frame->src_bufs[1].plane3.phys[1];
	src_curr_cr_data_phy = frame->src_bufs[1].plane3.phys[2];

	dst_y_data_phy = frame->dst_bufs[0].plane3.phys[0];
	dst_cb_data_phy	= frame->dst_bufs[0].plane3.phys[1];
	dst_cr_data_phy	= frame->dst_bufs[0].plane3.phys[2];

	dst_y_data_size = frame->dst_bufs[0].plane3.sizes[0];
	dst_cb_data_size = frame->dst_bufs[0].plane3.sizes[1];
	dst_cr_data_size = frame->dst_bufs[0].plane3.sizes[2];

	_set_deinterlacer(me, height, width,
			src_prev_y_data_phy, src_curr_y_data_phy,
			src_next_y_data_phy, src_y_stride, dst_y_data_phy,
			dst_y_stride, src_curr_cb_data_phy, src_c_stride,
			dst_cb_data_phy, dst_c_stride, src_curr_cr_data_phy,
			src_c_stride, dst_cr_data_phy, dst_c_stride,
			frame->src_bufs[1].frame_num % 2);

	/*	dump_deinterlace_register(me);	*/
	nx_deinterlace_deinterlace_start(me->reg);

	dev_dbg(dev, "SIZES - PREV SRC Y  : %ld\n",
			frame->src_bufs[0].plane3.sizes[0]);
	dev_dbg(dev, "SIZES - PREV SRC CB : %ld\n",
			frame->src_bufs[0].plane3.sizes[1]);
	dev_dbg(dev, "SIZES - PREV SRC CR : %ld\n",
			frame->src_bufs[0].plane3.sizes[2]);
	dev_dbg(dev, "SIZES - CURR SRC Y  : %ld\n",
			frame->src_bufs[1].plane3.sizes[0]);
	dev_dbg(dev, "SIZES - CURR SRC CB : %ld\n",
			frame->src_bufs[1].plane3.sizes[1]);
	dev_dbg(dev, "SIZES - CURR SRC CR : %ld\n",
			frame->src_bufs[1].plane3.sizes[2]);
	dev_dbg(dev, "SIZES - NEXT SRC Y  : %ld\n",
			frame->src_bufs[2].plane3.sizes[0]);
	dev_dbg(dev, "SIZES - NEXT SRC CB : %ld\n",
			frame->src_bufs[2].plane3.sizes[1]);
	dev_dbg(dev, "SIZES - NEXT SRC CR : %ld\n",
			frame->src_bufs[2].plane3.sizes[2]);
	dev_dbg(dev, "SIZES - NEXT DST Y  : %d\n",
			dst_y_data_size);
	dev_dbg(dev, "SIZES - NEXT DST CB : %d\n",
			dst_cb_data_size);
	dev_dbg(dev, "SIZES - NEXT DST CR : %d\n",
			dst_cr_data_size);

	dev_dbg(dev, "\nRESOLUTION - WIDTH : %d, HEIGHT : %d\n",
			width, height);
	dev_dbg(dev, "[%s]PHY - SRC PREV Y  : 0x%lX\n",
			__func__, src_prev_y_data_phy);
	dev_dbg(dev, "[%s]PHY - SRC CURR Y  : 0x%lX\n",
			__func__, src_curr_y_data_phy);
	dev_dbg(dev, "[%s]PHY - SRC NEXT Y  : 0x%lX\n",
			__func__, src_next_y_data_phy);
	dev_dbg(dev, "[%s]PHY - SRC CURR CB : 0x%lX\n",
			__func__, src_curr_cb_data_phy);
	dev_dbg(dev, "[%s]PHY - SRC CURR CR : 0x%lX\n",
			__func__, src_curr_cr_data_phy);

	dev_dbg(dev, "\n[%s]PHY - DST Y  : 0x%lX\n",
			__func__, dst_y_data_phy);
	dev_dbg(dev, "[%s]PHY - DST CB : 0x%lX\n",
			__func__, dst_cb_data_phy);
	dev_dbg(dev, "[%s]PHY - DST CR : 0x%lX\n",
			__func__, dst_cr_data_phy);

	dev_dbg(dev, "\n[%s]STRIDE - S Y : %d, S C : %d, D Y : %d, D C : %d\n\n",
			__func__,
			src_y_stride,
			src_c_stride,
			dst_y_stride,
			dst_c_stride);
}

static inline int _get_status(struct nx_deinterlacer *me)
{
	return atomic_read(&me->status);
}

static inline void _set_status(struct nx_deinterlacer *me, int status)
{
	atomic_set(&me->status, status);
}

static irqreturn_t deinterlacer_irq_handler(int irq, void *param)
{
	struct nx_deinterlacer *me = (struct nx_deinterlacer *)param;

	nx_deinterlace_clear_interrupt_pending_all(me->reg);
	_set_status(me, PROCESSING_STOP);
	wake_up_interruptible(&me->wq_end);

	return IRQ_HANDLED;
}

static inline int _wait_stop(struct nx_deinterlacer *me)
{
	struct device *dev = &me->pdev->dev;

	while (_get_status(me) != PROCESSING_STOP) {
		if (!wait_event_interruptible_timeout(me->wq_end,
			_get_status(me) == PROCESSING_STOP, TIMEOUT_STOP)) {

			dev_err(dev, "wait timeout for stop.\n");

			return -EBUSY;
		}
	}

	return 0;
}

static int _set_phys_from_fd(struct nx_deinterlacer *me,
		struct frame_data_info *frame_info) {
	struct device *dev = &me->pdev->dev;
	int ret;

	int width, height, r_height;
	int src0_plane_num, src1_plane_num;
	int src0_y_dma_fd, src0_cb_dma_fd, src0_cr_dma_fd;
	int src1_y_dma_fd, src1_cb_dma_fd, src1_cr_dma_fd;
	int dst_y_dma_fd = 0, dst_cb_dma_fd = 0, dst_cr_dma_fd = 0;
	int src0_y_stride, src0_c_stride, src1_y_stride, src1_c_stride;
	int y_offset, c_offset;
	unsigned int y_stride = 0, c_stride = 0;

	dma_addr_t src0_y_addr, src0_cb_addr, src0_cr_addr;
	dma_addr_t src1_y_addr, src1_cb_addr, src1_cr_addr;
	dma_addr_t dst_y_addr, dst_cb_addr, dst_cr_addr;

	struct frame_data *src_buf0, *src_buf1, *dst_buf;
	struct frame_data *src_frame0, *src_frame1, *src_frame2;

	unsigned long src0_y_phy = 0, src0_cb_phy = 0, src0_cr_phy = 0;
	unsigned long src1_y_phy = 0, src1_cb_phy = 0, src1_cr_phy = 0;

	width = frame_info->width;
	height = (frame_info->height / 2);
	r_height = frame_info->height;

	src_buf0 = &frame_info->src_bufs[0];
	src_buf1 = &frame_info->src_bufs[2];

	src0_plane_num = ((struct frame_data *)src_buf0)->plane_num;
	src1_plane_num = ((struct frame_data *)src_buf1)->plane_num;

	if (src0_plane_num != 1 && src0_plane_num != 3) {
		dev_err(dev, "The plane numer of the src0 frame plane num must be 1 or 3.\n");
		return -EINVAL;
	}

	if (src1_plane_num != 1 && src1_plane_num != 3) {
		dev_err(dev, "The plane numer of the src1 frame must be 1 or 3.\n");
		return -EINVAL;
	}

	src0_y_dma_fd = ((struct frame_data *)src_buf0)->plane3.fds[0];
	get_phy_addr_from_fd(&me->pdev->dev, src0_y_dma_fd, true,
			&src0_y_addr);
	if (src0_plane_num == 3) {
		src0_cb_dma_fd = ((struct frame_data *)src_buf0)->plane3.fds[1];
		get_phy_addr_from_fd(&me->pdev->dev, src0_cb_dma_fd, true,
				&src0_cb_addr);

		src0_cr_dma_fd = ((struct frame_data *)src_buf0)->plane3.fds[2];
		get_phy_addr_from_fd(&me->pdev->dev, src0_cr_dma_fd, true,
				&src0_cr_addr);
	}

	src1_y_dma_fd = ((struct frame_data *)src_buf1)->plane3.fds[0];
	get_phy_addr_from_fd(&me->pdev->dev, src1_y_dma_fd, true,
			&src1_y_addr);

	if (src1_plane_num == 3) {
		src1_cb_dma_fd = ((struct frame_data *)src_buf1)->plane3.fds[1];
		get_phy_addr_from_fd(&me->pdev->dev, src1_cb_dma_fd, true,
				&src1_cb_addr);

		src1_cr_dma_fd = ((struct frame_data *)src_buf1)->plane3.fds[2];
		get_phy_addr_from_fd(&me->pdev->dev, src1_cr_dma_fd, true,
				&src1_cr_addr);
	}

	src_frame0 = &frame_info->src_bufs[0];
	src_frame1 = &frame_info->src_bufs[1];
	src_frame2 = &frame_info->src_bufs[2];

	src0_y_stride = (frame_info->src_type == SRC_TYPE_MIPI) ?
			src_buf0->plane3.src_stride[0] :
			src_buf0->plane3.src_stride[0] / 2;
	src0_c_stride = (frame_info->src_type == SRC_TYPE_MIPI) ?
			src_buf0->plane3.src_stride[1] :
			src_buf0->plane3.src_stride[1] / 2;
	src1_y_stride = (frame_info->src_type == SRC_TYPE_MIPI) ?
			src_buf1->plane3.src_stride[0] :
			src_buf1->plane3.src_stride[0] / 2;
	src1_c_stride = (frame_info->src_type == SRC_TYPE_MIPI) ?
			src_buf1->plane3.src_stride[1] :
			src_buf1->plane3.src_stride[1] / 2;

#ifdef FILE_DUMP
	void *psrc0, *psrc1;
	char f0_name[200], f1_name[200];
	static int f0_idx, f1_idx;
	int w = width, h = r_height;

	int fsize = (w * h) + ((w/2) * (h/2) * 2);
	void *pdst0 = kmalloc(fsize, GFP_KERNEL);
	void *pdst1 = kmalloc(fsize, GFP_KERNEL);

	sprintf(f0_name, "/data/k_src0_%d.yuv", f0_idx++);
	sprintf(f1_name, "/data/k_src1_%d.yuv", f1_idx++);

	struct dma_buf *dma_buf0 = mmap_vaddr_from_fd(src0_y_dma_fd, &psrc0);
	struct dma_buf *dma_buf1 = mmap_vaddr_from_fd(src1_y_dma_fd, &psrc1);

	dev_dbg(dev, "%s: virt src0 : 0x%p\n", __func__, psrc0);
	dev_dbg(dev, "%s: virt src1 : 0x%p\n", __func__, psrc1);

	remove_stride(w, h, 64, pdst0, psrc0);
	write_file(f0_name, pdst0, fsize);

	remove_stride(w, h, 64, pdst1, psrc1);
	write_file(f1_name, pdst1, fsize);

	munmap_vaddr_from_fd(dma_buf0, psrc0);
	munmap_vaddr_from_fd(dma_buf1, psrc1);

	kfree(pdst0);
	kfree(pdst1);

	pdst0 = NULL;
	pdst1 = NULL;
#endif

	src0_y_phy = src0_y_addr;
	if (src0_plane_num == 1) {
		src0_cb_phy = src0_y_phy + (src0_y_stride * r_height);
		src0_cr_phy = src0_cb_phy +
			(src0_c_stride * (r_height >> 1));
	} else if (src0_plane_num == 3) {
		src0_cb_phy = src0_cb_addr;
		src0_cr_phy = src0_cr_addr;
	}

	src1_y_phy = src1_y_addr;
	if (src1_plane_num == 1) {
		src1_cb_phy = src1_y_phy + (src1_y_stride * r_height);
		src1_cr_phy = src1_cb_phy
			+ (src1_c_stride * (r_height >> 1));
	} else if (src1_plane_num == 3) {
		src1_cb_phy = src1_cb_addr;
		src1_cr_phy = src1_cr_addr;
	}

	if (frame_info->src_type == SRC_TYPE_MIPI) {
		y_offset = (src0_y_stride * height);
		c_offset = (src0_c_stride * (height >> 1));

		if (frame_info->src_field == FIELD_EVEN) {
			/*	odd - even - odd	*/

			/* odd	*/
			src_frame0->plane3.phys[0] = src0_y_phy;
			src_frame0->plane3.phys[1] = src0_cb_phy;
			src_frame0->plane3.phys[2] = src0_cr_phy;

			/* even	*/
			src_frame1->plane3.phys[0] = src0_y_phy + y_offset;
			src_frame1->plane3.phys[1] = src0_cb_phy + c_offset;
			src_frame1->plane3.phys[2] = src0_cr_phy + c_offset;

			/* odd	*/
			src_frame2->plane3.phys[0] = src1_y_phy;
			src_frame2->plane3.phys[1] = src1_cb_phy;
			src_frame2->plane3.phys[2] = src1_cr_phy;
		} else {
			/*	even - odd - even	*/

			/* even	*/
			src_frame0->plane3.phys[0] = src0_y_phy + y_offset;
			src_frame0->plane3.phys[1] = src0_cb_phy + c_offset;
			src_frame0->plane3.phys[2] = src0_cr_phy + c_offset;

			/* odd	*/
			src_frame1->plane3.phys[0] = src1_y_phy;
			src_frame1->plane3.phys[1] = src1_cb_phy;
			src_frame1->plane3.phys[2] = src1_cr_phy;

			/* even	*/
			src_frame2->plane3.phys[0] = src1_y_phy + y_offset;
			src_frame2->plane3.phys[1] = src1_cb_phy + c_offset;
			src_frame2->plane3.phys[2] = src1_cr_phy + c_offset;
		}
	} else {
		/* SRC_TYPE_PARALLEL */
		if (frame_info->src_field == FIELD_EVEN) {
			/*	odd - even - odd	*/

			/* odd	*/
			src_frame0->plane3.phys[0] = src0_y_phy;
			src_frame0->plane3.phys[1] = src0_cb_phy;
			src_frame0->plane3.phys[2] = src0_cr_phy;

			/* even	*/
			src_frame1->plane3.phys[0] =
				src0_y_phy + src0_y_stride;
			src_frame1->plane3.phys[1] =
				src0_cb_phy + src0_c_stride;
			src_frame1->plane3.phys[2] =
				src0_cr_phy + src0_c_stride;

			/* odd	*/
			src_frame2->plane3.phys[0] = src1_y_phy;
			src_frame2->plane3.phys[1] = src1_cb_phy;
			src_frame2->plane3.phys[2] = src1_cr_phy;
		} else {
			/*	even - odd - even	*/

			/* even	*/
			src_frame0->plane3.phys[0] =
				src0_y_phy + src0_y_stride;
			src_frame0->plane3.phys[1] =
				src0_cb_phy + src0_c_stride;
			src_frame0->plane3.phys[2] =
				src0_cr_phy + src0_c_stride;

			/* odd	*/
			src_frame1->plane3.phys[0] = src1_y_phy;
			src_frame1->plane3.phys[1] = src1_cb_phy;
			src_frame1->plane3.phys[2] = src1_cr_phy;

			/* even	*/
			src_frame2->plane3.phys[0] =
				src1_y_phy + src1_y_stride;
			src_frame2->plane3.phys[1] =
				src1_cb_phy + src1_c_stride;
			src_frame2->plane3.phys[2] =
				src1_cr_phy + src1_c_stride;
		}
	}

	dev_dbg(dev, "plane_num src0 = %d, src1 = %d\n",
			src0_plane_num, src1_plane_num);
	dev_dbg(dev, "FD src0 = %d, src1 = %d\n",
			src0_y_dma_fd, src1_y_dma_fd);
	dev_dbg(dev, "src type = %s\n", (frame_info->src_type == SRC_TYPE_MIPI) ?
			"MIPI" : "PALLALEL");
	dev_dbg(dev, "src field = %s\n",
			(frame_info->src_field == FIELD_EVEN) ? "EVEN" : "ODD");
	dev_dbg(dev, "width = %d, height = %d, real height = %d\n",
			width, height, r_height);
	dev_dbg(dev, "stride src0 - y_stride = %d, c_stride = %d\n",
			src0_y_stride, src0_c_stride);
	dev_dbg(dev, "stride src1 - y_stride = %d, c_stride = %d\n",
			src1_y_stride, src1_c_stride);
	dev_dbg(dev, "frame0 - y = 0x%08lx, cb = 0x%08lx, cr = 0x%08lx\n",
			src_frame0->plane3.phys[0],
			src_frame0->plane3.phys[1],
			src_frame0->plane3.phys[2]);
	dev_dbg(dev, "frame1 - y = 0x%08lx, cb = 0x%08lx, cr = 0x%08lx\n",
			src_frame1->plane3.phys[0],
			src_frame1->plane3.phys[1],
			src_frame1->plane3.phys[2]);
	dev_dbg(dev, "frame2 - y = 0x%08lx, cb = 0x%08lx, cr = 0x%08lx\n",
			src_frame2->plane3.phys[0],
			src_frame2->plane3.phys[1],
			src_frame2->plane3.phys[2]);

	/* set pysical address from DMA fd for destination buffers. */
	dst_buf = &frame_info->dst_bufs[0];
	if (dst_buf->plane_num == 1) {
		dst_y_dma_fd = dst_buf->plane3.fds[0];
		y_stride = dst_buf->plane3.dst_stride[0];
		c_stride = dst_buf->plane3.dst_stride[1];

		if (dst_y_dma_fd > 0) {
			ret = get_phy_addr_from_fd(dev, dst_y_dma_fd,
					true, &dst_y_addr);
			dst_buf->plane3.phys[0] = dst_y_addr;
			dst_buf->plane3.phys[1] = dst_y_addr +
				(y_stride * r_height);
			dst_buf->plane3.phys[2] =
				dst_buf->plane3.phys[1] +
				(c_stride * (r_height >> 1));
		}
	} else if (dst_buf->plane_num == 3) {
		dst_y_dma_fd = dst_buf->plane3.fds[0];
		dst_cb_dma_fd = dst_buf->plane3.fds[1];
		dst_cr_dma_fd = dst_buf->plane3.fds[2];

		if (dst_y_dma_fd > 0) {
			ret = get_phy_addr_from_fd(dev, dst_y_dma_fd,
				true, &dst_y_addr);
			dst_buf->plane3.phys[0] = dst_y_addr;
		}

		if (dst_cb_dma_fd > 0) {
			ret = get_phy_addr_from_fd(dev, dst_cb_dma_fd,
				true, &dst_cb_addr);
			dst_buf->plane3.phys[1] = dst_cb_addr;
		}

		if (dst_cr_dma_fd > 0) {
			ret = get_phy_addr_from_fd(dev, dst_cr_dma_fd,
				true, &dst_cr_addr);
			dst_buf->plane3.phys[2] = dst_cr_addr;
		}
	}

	dev_dbg(dev, "dst plane_num = %d\n", dst_buf->plane_num);
	dev_dbg(dev, "dst FD y = %d\n", dst_y_dma_fd);
	dev_dbg(dev, "dst stride - y_stride = %d, c_stride = %d\n",
			y_stride, c_stride);
	dev_dbg(dev, "dst frame - y = 0x%08lx, cb = 0x%08lx, cr = 0x%08lx\n",
			dst_buf->plane3.phys[0],
			dst_buf->plane3.phys[1],
			dst_buf->plane3.phys[2]);
	dev_dbg(dev, "\n");

	return ret;
}

static int _handle_set_and_run(struct nx_deinterlacer *me, unsigned long arg)
{
	struct frame_data_info frame_info;
	struct device *dev = &me->pdev->dev;
	int ret;

	if (copy_from_user(&frame_info, (struct frame_data_info *)arg,
		sizeof(struct frame_data_info))) {
		dev_err(dev, "%s: failed to copy_from_user.\n", __func__);
		_set_status(me, PROCESSING_STOP);

		return -EFAULT;
	}

	_set_phys_from_fd(me, &frame_info);
	_set_status(me, PROCESSING_START);
	_set_and_run(me, &frame_info);
	ret = _wait_stop(me);

	if (ret < 0) {
		dev_err(dev, "failed to _wait_stop\n");
		_set_status(me, PROCESSING_STOP);
		return ret;
	}

#ifdef FILE_DUMP
	void *pSrc = NULL;
	char f_name[200] = {0,};
	static int f_idx;
	int w = frame_info.width;
	int h = frame_info.height;
	int fsize = (w * h) + ((w/2) * (h/2) * 2);
	void *pDst = kmalloc(fsize, GFP_KERNEL);
	int dst_y_dma_fd = 0;

	dst_y_dma_fd = frame_info.dst_bufs[0].plane3.fds[0];

	sprintf(f_name, "/data/k_dst_%d.yuv", f_idx++);

	struct dma_buf *dma_buf = mmap_vaddr_from_fd(dst_y_dma_fd, &pSrc);

	dev_dbg(dev, "%s: virt dst : 0x%p\n", __func__, pDst);

	remove_stride(w, h, 512, pDst, pSrc);
	write_file(f_name, pDst, fsize);

	munmap_vaddr_from_fd(dma_buf, pSrc);

	kfree(pDst);

	pDst = NULL;
#endif

	return true;
}

static long deinterlacer_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct nx_deinterlacer *me = container_of(file->private_data,
			struct nx_deinterlacer, miscdev);
	int ret = -EINVAL;

	switch (cmd) {
	case IOCTL_DEINTERLACE_SET_AND_RUN:
		mutex_lock(&me->mutex);
		ret = _handle_set_and_run(me, arg);
		mutex_unlock(&me->mutex);
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
struct compat_frame_data {
	compat_int_t frame_num;
	compat_int_t plane_num;
	compat_int_t frame_type;
	compat_int_t frame_factor;

	union {
		struct {
			compat_uptr_t virt[MAX_BUFFER_PLANES];
			compat_ulong_t sizes[MAX_BUFFER_PLANES];
			compat_ulong_t src_stride[MAX_BUFFER_PLANES];
			compat_ulong_t dst_stride[MAX_BUFFER_PLANES];

			compat_int_t fds[MAX_BUFFER_PLANES];
			compat_ulong_t phys[MAX_BUFFER_PLANES];
		} plane3;
	};
};

struct compat_frame_data_info {
	compat_int_t command;
	compat_int_t width;
	compat_int_t height;
	compat_int_t plane_mode;
	compat_int_t src_type;
	compat_int_t src_field;

	struct compat_frame_data dst_bufs[DST_BUFFER_COUNT];
	struct compat_frame_data src_bufs[SRC_BUFFER_COUNT];
};

static int compat_get_frame_data(struct compat_frame_data *data32,
		struct frame_data *data)
{
	compat_int_t i32;
	compat_ulong_t ul;
	int err;
	int i;

	err  = get_user(i32, &data32->frame_num);
	err |= put_user(i32, &data->frame_num);
	err |= get_user(i32, &data32->plane_num);
	err |= put_user(i32, &data->plane_num);
	err |= get_user(i32, &data32->frame_type);
	err |= put_user(i32, &data->frame_type);
	err |= get_user(i32, &data32->frame_factor);
	err |= put_user(i32, &data->frame_factor);

	for (i = 0; i < MAX_BUFFER_PLANES; i++) {
		err |= get_user(ul, &data32->plane3.sizes[i]);
		err |= put_user(ul, &data->plane3.sizes[i]);
	}

	for (i = 0; i < MAX_BUFFER_PLANES; i++) {
		err |= get_user(ul, &data32->plane3.src_stride[i]);
		err |= put_user(ul, &data->plane3.src_stride[i]);
	}

	for (i = 0; i < MAX_BUFFER_PLANES; i++) {
		err |= get_user(ul, &data32->plane3.dst_stride[i]);
		err |= put_user(ul, &data->plane3.dst_stride[i]);
	}

	for (i = 0; i < MAX_BUFFER_PLANES; i++) {
		err |= get_user(ul, &data32->plane3.fds[i]);
		err |= put_user(ul, &data->plane3.fds[i]);
	}

	for (i = 0; i < MAX_BUFFER_PLANES; i++) {
		err |= get_user(ul, &data32->plane3.phys[i]);
		err |= put_user(ul, &data->plane3.phys[i]);
	}

	return err;
}

static int compat_get_frame_data_info(struct compat_frame_data_info *data32,
		struct frame_data_info *data)
{
	compat_int_t i32;
	int i;
	int err;

	err = get_user(i32, &data32->command);
	err |= put_user(i32, &data->command);
	err |= get_user(i32, &data32->width);
	err |= put_user(i32, &data->width);
	err |= get_user(i32, &data32->height);
	err |= put_user(i32, &data->height);
	err |= get_user(i32, &data32->plane_mode);
	err |= put_user(i32, &data->plane_mode);
	err |= get_user(i32, &data32->src_type);
	err |= put_user(i32, &data->src_type);
	err |= get_user(i32, &data32->src_field);
	err |= put_user(i32, &data->src_field);

	for (i = 0; i < DST_BUFFER_COUNT; i++) {
		err |= compat_get_frame_data(&data32->dst_bufs[i],
				&data->dst_bufs[i]);
	}

	for (i = 0; i < SRC_BUFFER_COUNT; i++) {
		err |= compat_get_frame_data(&data32->src_bufs[i],
				&data->src_bufs[i]);
	}

	return err;
}

static long deinterlacer_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct nx_deinterlacer *me = container_of(filp->private_data,
			struct nx_deinterlacer, miscdev);
	struct device *dev = &me->pdev->dev;
	int ret = -EINVAL;

	switch (cmd) {
	case IOCTL_DEINTERLACE_SET_AND_RUN:
		{
			struct compat_frame_data_info __user *data32;
			struct frame_data_info __user *data;
			int err;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (!data) {
				dev_err(dev, "%s:failed to alloc_user_space\n",
						__func__);
				ret = -EFAULT;
			}

			ret = compat_get_frame_data_info(data32, data);
			if (err) {
				dev_err(dev, "%s:failed to get_frame_data\n",
						__func__);
				break;
			}

			ret = filp->f_op->unlocked_ioctl(filp,
					IOCTL_DEINTERLACE_SET_AND_RUN,
					(unsigned long)data);
			break;
		}
	}

	return ret;
}
#endif

static void set_deinterlacer_interrupt(struct nx_deinterlacer *me, bool enable)
{
	nx_deinterlace_set_interrupt_enable_all(me->reg, false);
	nx_deinterlace_clear_interrupt_pending_all(me->reg);

	if (enable)
		nx_deinterlace_set_interrupt_enable(me->reg, 3, true);
}

static int nx_deinterlacer_irq_register(struct nx_deinterlacer *me)
{
	int ret = -EINVAL;
	struct device *dev = &me->pdev->dev;

	if (me->irq) {
		ret = devm_request_irq(dev, me->irq,
			&deinterlacer_irq_handler, IRQF_SHARED,
			me->irq_name, me);
		if (ret) {
			dev_err(dev, "failed to request irq.\n");
			return ret;
		}
	}

	return ret;
}

static int nx_deinterlacer_clock_enable(struct nx_deinterlacer *me, bool enable)
{
	struct device *dev = &me->pdev->dev;
	u8 ret;

	if (enable) {
		ret = clk_prepare_enable(me->clk_axi);
		if (ret) {
			dev_err(dev, "AXI clk_prepare_enable(%s) failed\n",
				__clk_get_name(me->clk_axi));
			return ret;
		}

		ret = clk_prepare_enable(me->clk_apb);
		if (ret) {
			dev_err(dev, "APB clk_prepare_enable(%s) failed\n",
				__clk_get_name(me->clk_apb));
			return ret;
		}

		return ret;
	}

	clk_disable_unprepare(me->clk_apb);
	clk_disable_unprepare(me->clk_axi);

	return 0;
}

static int nx_deinterlacer_parse_dt(struct nx_deinterlacer *me)
{
	int ret;
	struct device *dev = &me->pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource res;

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get base address\n");
		return -ENXIO;
	}

	me->base = devm_ioremap_nocache(dev, res.start, resource_size(&res));
	if (!me->base) {
		dev_err(dev, "failed to ioremap\n");
		return -ENOMEM;
	}
	me->reg = (struct nx_deinterlace_register_set *)me->base;

	ret = platform_get_irq(me->pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get irq num\n");
		return -EINVAL;
	}
	me->irq = ret;

	me->clk_axi = devm_clk_get(dev, "axi");
	if (IS_ERR(me->clk_axi)) {
		dev_err(dev, "failed to devm_clk_get for axi\n");
		return PTR_ERR(me->clk_axi);
	}

	me->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(me->clk_apb)) {
		dev_err(dev, "failed to devm_clk_get for apb\n");
		return PTR_ERR(me->clk_apb);
	}

	return 0;
}

static int deinterlacer_open(struct inode *inode, struct file *file)
{
	struct nx_deinterlacer *me = container_of(file->private_data,
			struct nx_deinterlacer, miscdev);
	struct device *dev = &me->pdev->dev;

	if (atomic_read(&me->open_count) > 0) {
		atomic_inc(&me->open_count);
		dev_err(dev, "%s: open count %d\n",
				__func__, atomic_read(&me->open_count));

		return 0;
	}

	atomic_inc(&me->open_count);
	atomic_set(&me->status, PROCESSING_STOP);

	set_deinterlacer_interrupt(me, true);

	return 0;
}

static int deinterlacer_close(struct inode *inode, struct file *file)
{
	struct nx_deinterlacer *me = container_of(file->private_data,
			struct nx_deinterlacer, miscdev);
	struct device *dev = &me->pdev->dev;

	atomic_dec(&me->open_count);

	if (atomic_read(&me->open_count) == 0) {
		dev_dbg(dev, "%s ==> open count 0\n", __func__);
		atomic_set(&me->status, PROCESSING_STOP);

		set_deinterlacer_interrupt(me, false);
	}

	return 0;
}

static const struct file_operations nx_deinterlacer_fops = {
	.open		= deinterlacer_open,
	.release	= deinterlacer_close,
	.unlocked_ioctl	= deinterlacer_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = deinterlacer_compat_ioctl,
#endif
};

static struct miscdevice nx_deinterlacer_misc_device = {
	.minor			=	MISC_DYNAMIC_MINOR,
	.name			=	NX_DEINTERLACER_DEV_NAME,
	.fops			=	&nx_deinterlacer_fops,
};

/* pm ops */
#ifdef CONFIG_PM_SLEEP
static int nx_deinterlacer_suspend(struct device *dev)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct nx_deinterlacer *me = platform_get_drvdata(pdev);

	set_deinterlacer_interrupt(me, false);
	nx_deinterlacer_clock_enable(me, false);

	return 0;
}

static int nx_deinterlacer_resume(struct device *dev)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct nx_deinterlacer *me = platform_get_drvdata(pdev);

	nx_deinterlacer_clock_enable(me, true);
	set_deinterlacer_interrupt(me, true);

	return 0;
}

static const struct dev_pm_ops nx_deinterlacer_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nx_deinterlacer_suspend,
			nx_deinterlacer_resume)
};
#endif

static int nx_deinterlacer_probe(struct platform_device *pdev)
{
	int ret;
	struct nx_deinterlacer *me;

	me = devm_kzalloc(&pdev->dev, sizeof(*me), GFP_KERNEL);
	if (!me)
		return -ENOMEM;

	me->miscdev = nx_deinterlacer_misc_device;
	ret = misc_register(&me->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to misc_register\n", __func__);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, me);
	dev_set_drvdata(&pdev->dev, me);

	me->pdev = pdev;

	ret = nx_deinterlacer_parse_dt(me);
	if (ret)
		goto misc_deregister;

	init_waitqueue_head(&(me->wq_start));
	init_waitqueue_head(&(me->wq_end));
	mutex_init(&me->mutex);
	sprintf(me->irq_name, "%s", "nx-deinterlace");

	nx_deinterlacer_clock_enable(me, true);
	nx_deinterlacer_irq_register(me);

	pr_info("Deinterlacer device registered as /dev/%s\n",
			NX_DEINTERLACER_DEV_NAME);

	return 0;

misc_deregister:
	misc_deregister(&me->miscdev);

	return ret;
}

static int nx_deinterlacer_remove(struct platform_device *pdev)
{
	struct nx_deinterlacer *me = platform_get_drvdata(pdev);

	if (me) {
		nx_deinterlacer_clock_enable(me, false);
		misc_deregister(&me->miscdev);
		me = NULL;
	}

	return 0;
}

static struct platform_device_id nx_deinterlacer_id_table[] = {
	{ NX_DEINTERLACER_DEV_NAME, 0 },
	{},
};

static const struct of_device_id nx_deinterlacer_dt_match[] = {
	{ .compatible = "nexell,nxp3220-deinterlacer" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_deinterlacer_dt_match);

static struct platform_driver nx_deinterlacer_driver = {
	.probe		= nx_deinterlacer_probe,
	.remove		= nx_deinterlacer_remove,
	.id_table	= nx_deinterlacer_id_table,
	.driver	= {
		.name	= NX_DEINTERLACER_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nx_deinterlacer_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm	= &nx_deinterlacer_pm_ops,
#endif
	},
};

#ifdef CONFIG_DEFERRED_UP_DEINTERLACER
static int __init nx_deinterlacer_init(void)
{
        return platform_driver_register(&nx_deinterlacer_driver);
}

subsys_initcall(nx_deinterlacer_init);
#else
module_platform_driver(nx_deinterlacer_driver);
#endif

MODULE_AUTHOR("JongKeun Choi<jkchoi@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell deinterlace devicer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("v0.1");
MODULE_ALIAS("platform:nxp3220-deinterlacer");
