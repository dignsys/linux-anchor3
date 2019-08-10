// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell G2D driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <drm/drmP.h>
#include <drm/nexell_drm.h>

#include "nexell_drv.h"
#include "nexell_gem.h"
#include "nxp3220_g2d.h"

struct nx_g2d_reg {
	u32 axi_parm;
	u32 out_stand;
	u32 status;
	u32 __reserved[(((0x40 - 0x08) / 4)  - 1)];
	u32 src_ctrl;
	u32 src_base;
	u32 src_stride;
	u32 src_blksize;
	u32 solid_color;
	u32 size;
	u32 dst_ctrl;
	u32 dst_base;
	u32 dst_stride;
	u32 dst_blksize;
	u32 blend_color;
	u32 blend_equat_alpha;
	u32 run;
};

static const u32 cmd_reg_offs[] = {
	[NX_G2D_CMD_SRC_CTRL] = 0x40,
	[NX_G2D_CMD_SRC_ADDR] = 0x44,
	[NX_G2D_CMD_SRC_STRIDE] = 0x48,
	[NX_G2D_CMD_SRC_BLKSIZE] = 0x4c,
	[NX_G2D_CMD_SOLID_COLOR] = 0x50,
	[NX_G2D_CMD_SIZE] = 0x54,
	[NX_G2D_CMD_DST_CTRL] = 0x58,
	[NX_G2D_CMD_DST_ADDR] = 0x5c,
	[NX_G2D_CMD_DST_STRIDE] = 0x60,
	[NX_G2D_CMD_DST_BLKSIZE] = 0x64,
	[NX_G2D_CMD_BLEND_COLOR] = 0x68,
	[NX_G2D_CMD_BLEND_EQUAT_ALPHA] = 0x6c,
	[NX_G2D_CMD_RUN] = 0x7c,
};

#define NX_G2D_INTC_G2D_ENB		BIT(0)
#define NX_G2D_INTC_G2D_MASK		BIT(1)
#define NX_G2D_INTC_G2D_PEND		BIT(2)
#define NX_G2D_INTC_EMPTY_ENB		BIT(4)
#define NX_G2D_INTC_EMPTY_MASK		BIT(5)
#define NX_G2D_INTC_EMPTY_PEND		BIT(6)

#define NX_G2D_AXI_BURST_LEN		(2)
#define NX_G2D_AXI_CACHE_SIGNAL		(0)
#define NX_G2D_AXI_PROT_SIGNAL		(0)
#define NX_G2D_MAX_OUTSTAND		(32)

#define NX_G2D_IRQ_G2D_ENB	\
	(NX_G2D_INTC_G2D_ENB | NX_G2D_INTC_G2D_MASK | \
	NX_G2D_INTC_G2D_PEND)

#define NX_G2D_IRQ_EMPTY_ENB	\
	(NX_G2D_INTC_EMPTY_ENB | NX_G2D_INTC_EMPTY_MASK | \
	NX_G2D_INTC_EMPTY_PEND)

struct nx_g2d_data {
	struct nx_drm_subdrv subdrv;
	struct device *dev;
	struct nx_g2d_reg *reg;
	struct clk *axi;
	int irq;
	struct mutex mutex;
	struct completion complete;
	int major, minor;
	int axi_burst, axi_cache, axi_prot;
	int out_stand;
};

static void nx_g2d_dump_cmd(struct nx_g2d_data *g2d, struct nx_g2d_cmd *cmd)
{
	int i;

	dev_dbg(g2d->dev, "================================================\n");
	for (i = 0; i < NX_G2D_CMD_NR; i++) {
		if (cmd->cmd_mask & (1 << i))
			dev_dbg(g2d->dev, "[0x%2x] 0x%08x\n",
				cmd_reg_offs[i], cmd->cmd[i]);
	}
	dev_dbg(g2d->dev, "================================================\n");
}

static inline void nx_g2d_set_axiparam(struct nx_g2d_data *g2d)
{
	struct nx_g2d_reg *reg = g2d->reg;
	u32 val;

	val = readl(&reg->axi_parm) & ~(0xf << 7 | 0xf << 3 | 0x7);
	val |= ((g2d->axi_burst & 0xf) << 7) |
	       ((g2d->axi_cache & 0xf) << 3) |
		(g2d->axi_prot & 0x7);

	writel(val, &reg->axi_parm);
}

static inline void nx_g2d_set_outstand(struct nx_g2d_data *g2d)
{
	struct nx_g2d_reg *reg = g2d->reg;
	int out_stand = g2d->out_stand;

	writel(out_stand, &reg->out_stand);
}

static void nx_g2d_exec_cmd(struct nx_g2d_data *g2d, struct nx_g2d_cmd *cmd)
{
	struct nx_g2d_reg *reg = g2d->reg;
	void __iomem *base = g2d->reg;
	u32 val;
	int i;

	cmd->cmd[NX_G2D_CMD_RUN] = 0;

	for (i = 0; i < NX_G2D_CMD_NR; i++) {
		if (cmd->cmd_mask & (1 << i))
			writel(cmd->cmd[i], base + cmd_reg_offs[i]);
	}

	val = readl(&reg->status) | (1 << 31);
	val |= NX_G2D_IRQ_G2D_ENB | NX_G2D_IRQ_EMPTY_ENB;
	writel(val, &reg->status);

	/* run */
	writel(1, base + cmd_reg_offs[NX_G2D_CMD_RUN]);
}

static int nx_g2d_setup_cmd(struct drm_device *drm, struct drm_file *file,
			    struct nx_g2d_cmd *req)
{
	struct nx_g2d_data *g2d = file->driver_priv;
	struct nx_g2d_buf *buf;
	dma_addr_t addr;

	if (req->cmd_nr > NX_G2D_CMD_MAX_SIZE) {
		dev_err(g2d->dev,
			"Failed commands %d is over max fifo %d\n",
			req->cmd_nr, NX_G2D_CMD_MAX_SIZE);
		return -EINVAL;
	}

	/*
	 * get source buffer
	 */
	buf = &req->src;
	if (buf->type != NX_G2D_BUF_TYPE_NONE && buf->handle) {
		if (buf->type == NX_G2D_BUF_TYPE_GEM)
			addr = nx_drm_gem_get_dma_addr(drm, buf->handle, file);
		else
			addr = buf->handle;

		if (!addr)
			return -ENOMEM;

		req->cmd[NX_G2D_CMD_SRC_ADDR] = (u32)addr + buf->offset;
		req->cmd_mask |= 1 << NX_G2D_CMD_SRC_ADDR;
		dev_dbg(g2d->dev,
			"SRC:0x%08x\n", req->cmd[NX_G2D_CMD_SRC_ADDR]);
	}

	/*
	 * get destinatioin buffer
	 */
	buf = &req->dst;
	if (buf->type == NX_G2D_BUF_TYPE_NONE || !buf->handle) {
		dev_err(g2d->dev,
			"Failed to destination buffer type:%d, hnd:%d\n",
			buf->type, buf->handle);
		return -EINVAL;
	}

	if (buf->type == NX_G2D_BUF_TYPE_GEM)
		addr = nx_drm_gem_get_dma_addr(drm, buf->handle, file);
	else
		addr = buf->handle;

	if (!addr)
		return -ENOMEM;

	req->cmd[NX_G2D_CMD_DST_ADDR] = (u32)addr + buf->offset;
	req->cmd_mask |= 1 << NX_G2D_CMD_DST_ADDR;
	req->cmd_nr = hweight_long(req->cmd_mask);

	dev_dbg(g2d->dev,
		"DST:0x%08x\n", req->cmd[NX_G2D_CMD_DST_ADDR]);

	nx_g2d_dump_cmd(g2d, req);

	return 0;
}

static irqreturn_t nx_g2d_irq_handler(int irq, void *dev_id)
{
	struct nx_g2d_data *g2d = dev_id;
	struct nx_g2d_reg *reg = g2d->reg;
	u32 pend = readl(&reg->status);

	dev_dbg(g2d->dev, "IRQ:%s/%s\n",
		pend | NX_G2D_INTC_G2D_PEND ? "G2D" : "None",
		pend | NX_G2D_INTC_EMPTY_PEND ? "EMPTY" : "None");

	if (pend & NX_G2D_INTC_G2D_PEND)
		pend |= NX_G2D_IRQ_G2D_ENB;

	if (pend & NX_G2D_INTC_EMPTY_PEND)
		pend |= NX_G2D_IRQ_EMPTY_ENB;

	writel(pend, &reg->status);
	complete(&g2d->complete);

	return IRQ_HANDLED;
}

int nx_drm_g2d_get_version(struct drm_device *drm_dev, void *data,
			 struct drm_file *file)
{
	struct nx_g2d_data *g2d = file->driver_priv;
	struct nx_g2d_ver *ver = data;

	if (WARN_ON(!g2d))
		return -ENODEV;

	if (WARN_ON(!ver))
		return -EINVAL;

	ver->major = g2d->major;
	ver->minor = g2d->minor;

	return 0;
}

int nx_drm_g2d_exec_ioctl(struct drm_device *drm, void *data,
			  struct drm_file *file)
{
	struct nx_g2d_data *g2d = file->driver_priv;
	struct nx_g2d_cmd *req = data;
	long long ts = ktime_to_ms(ktime_get());
	int ret;

	if (WARN_ON(!g2d))
		return -ENODEV;

	if (WARN_ON(!req))
		return -EINVAL;

	ret = nx_g2d_setup_cmd(drm, file, req);
	if (ret)
		return ret;

	if (!req->cmd_nr)
		return 0;

	mutex_lock(&g2d->mutex);

	nx_g2d_exec_cmd(g2d, req);

	mutex_unlock(&g2d->mutex);

	wait_for_completion(&g2d->complete);

	dev_dbg(g2d->dev, "[%lldms]\n", ktime_to_ms(ktime_get()) - ts);

	return ret;
}

int nx_drm_g2d_sync_ioctl(struct drm_device *drm_dev, void *data,
			 struct drm_file *file)
{
	return 0;
}

static int nx_g2d_prepare(struct nx_g2d_data *g2d)
{
	init_completion(&g2d->complete);
	mutex_init(&g2d->mutex);

	clk_prepare_enable(g2d->axi);

	g2d->major = NX_G2D_DRIVER_VER_MAJOR;
	g2d->minor = NX_G2D_DRIVER_VER_MINOR;

	g2d->axi_burst = NX_G2D_AXI_BURST_LEN;
	g2d->axi_cache = NX_G2D_AXI_CACHE_SIGNAL;
	g2d->axi_prot = NX_G2D_AXI_PROT_SIGNAL;
	g2d->out_stand = NX_G2D_MAX_OUTSTAND;

	nx_g2d_set_axiparam(g2d);
	nx_g2d_set_outstand(g2d);

	return 0;
}

static int nx_g2d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct nx_g2d_data *g2d;
	int ret;

	g2d = devm_kzalloc(dev, sizeof(*g2d), GFP_KERNEL);
	if (!g2d)
		return -ENOMEM;

	g2d->dev = dev;
	g2d->subdrv.driver_data = g2d;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	g2d->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(g2d->reg))
		return PTR_ERR(g2d->reg);

	g2d->axi = devm_clk_get(dev, "axi");
	if (IS_ERR(g2d->axi))
		return PTR_ERR(g2d->axi);

	g2d->irq = platform_get_irq(pdev, 0);
	if (g2d->irq < 0)
		return g2d->irq;

	ret = devm_request_irq(dev, g2d->irq,
			       nx_g2d_irq_handler, 0, "nx-g2d", g2d);
	if (ret < 0)
		return ret;

	nx_g2d_prepare(g2d);

	ret = nx_drm_subdrv_register(&g2d->subdrv, dev);
	if (ret < 0) {
		DRM_ERROR("Failed to register G2D sub drvier!!!\n");
		return ret;
	}

	platform_set_drvdata(pdev, g2d);

	DRM_INFO("Loaded G2D Version: %d-%d\n", g2d->major, g2d->minor);

	return 0;
}

static int nx_g2d_remove(struct platform_device *pdev)
{
	struct nx_g2d_data *g2d = platform_get_drvdata(pdev);

	return nx_drm_subdrv_unregister(&g2d->subdrv);
}

#ifdef CONFIG_PM_SLEEP
static int nx_g2d_suspend(struct device *dev)
{
	struct nx_g2d_data *g2d = dev_get_drvdata(dev);

	if (!__clk_is_enabled(g2d->axi))
		clk_disable_unprepare(g2d->axi);

	return 0;
}

static int nx_g2d_resume(struct device *dev)
{
	struct nx_g2d_data *g2d = dev_get_drvdata(dev);

	if (__clk_is_enabled(g2d->axi))
		clk_prepare_enable(g2d->axi);

	nx_g2d_set_axiparam(g2d);
	nx_g2d_set_outstand(g2d);

	return 0;
}
#endif

static const struct dev_pm_ops g2d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nx_g2d_suspend, nx_g2d_resume)
};

static const struct of_device_id nx_g2d_match[] = {
	{ .compatible = "nexell,nxp3220-drm-g2d" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_g2d_match);

struct platform_driver drm_g2d_driver = {
	.probe		= nx_g2d_probe,
	.remove		= nx_g2d_remove,
	.driver		= {
		.name	= "nexell,display_drm_g2d",
		.owner	= THIS_MODULE,
		.pm	= &g2d_pm_ops,
		.of_match_table = nx_g2d_match,
	},
};
