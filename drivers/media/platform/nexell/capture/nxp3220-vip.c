// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/interrupt.h>

#include <dt-bindings/media/nxp3220-vip.h>

#include "../nx-v4l2.h"
#include "nxp3220-vip-primitive.h"
#include "nxp3220-vip.h"

#define NX_VIP_DEV_NAME		"nx-vip"

/* if defined, when vip enabled, register of VIP are dumped */
/*	#define DUMP_REGISTER	*/

struct nx_vip {
	u32 module;
	void *base;
	int irq;
	struct clk *clk_axi;
	struct clk *clk_apb;
	struct clk *clk_padout0;
	struct clk *clk_padout1;

	struct reset_control *rst;
	int resets_num;

	atomic_t running_bitmap;

	spinlock_t lock;
	struct list_head irq_entry_list;
	int irq_entry_count;

	bool clipper_enable;
	bool decimator_enable;
	char irq_name[12];
};

static struct nx_vip *_nx_vip_object[NUMBER_OF_VIP_MODULE];

/**
 * static functions
 */
static int nx_vip_parse_dt(struct platform_device *pdev, struct nx_vip *me)
{
	int ret;
	struct device *dev = &pdev->dev;
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
		return -EBUSY;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get irq num\n");
		return -EBUSY;
	}
	me->irq = ret;

	if (of_property_read_u32(np, "module", &me->module)) {
		dev_err(dev, "failed to get module\n");
		return -EINVAL;
	}
	if (me->module >= NUMBER_OF_VIP_MODULE) {
		dev_err(dev, "invalid module: %d\n", me->module);
		return -EINVAL;
	}

	me->clk_axi = devm_clk_get(dev, "vip_axi");
	if (IS_ERR(me->clk_axi)) {
		dev_err(dev, "failed to devm_clk_get for vip_axi\n");
		return -ENODEV;
	}

	me->clk_apb = devm_clk_get(dev, "vip_apb");
	if (IS_ERR(me->clk_apb)) {
		dev_err(dev, "failed to devm_clk_get for vip_apb\n");
		return -ENODEV;
	}

	me->clk_padout0 = devm_clk_get(dev, "vip_padout0");
	if (IS_ERR(me->clk_padout0))
		me->clk_padout0 = NULL;

	me->clk_padout1 = devm_clk_get(dev, "vip_padout1");
	if (IS_ERR(me->clk_padout1))
		me->clk_padout1 = NULL;

	me->rst = devm_reset_control_get(dev, "vip-reset");
	if (IS_ERR(me->rst)) {
		dev_err(dev, "failed to get reset control\n");
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t vip_irq_handler(int irq, void *desc)
{
	struct nx_vip *me = desc;
	struct nx_v4l2_irq_entry *e;
	unsigned long flags;

	nx_vip_clear_interrupt_pending_all(me->module);

	spin_lock_irqsave(&me->lock, flags);
	if (!list_empty(&me->irq_entry_list))
		list_for_each_entry(e, &me->irq_entry_list, entry)
			e->handler(e->priv);
	spin_unlock_irqrestore(&me->lock, flags);

	return IRQ_HANDLED;
}

static void hw_child_enable(struct nx_vip *me, u32 child)
{
	bool clipper_enable = false;
	bool decimator_enable = false;

	if (child & VIP_CLIPPER)
		clipper_enable = true;

	if (child & VIP_DECIMATOR)
		decimator_enable = true;

	if (me->clipper_enable != clipper_enable ||
	    me->decimator_enable != decimator_enable) {
		if (clipper_enable || decimator_enable) {
			nx_vip_set_interrupt_enable_all(me->module, false);
			nx_vip_clear_interrupt_pending_all(me->module);
			nx_vip_set_vipenable(me->module, true, true,
					     clipper_enable, decimator_enable);
			nx_vip_set_interrupt_enable(me->module,
						    VIP_OD_INT, true);
		} else {
			nx_vip_set_vipenable(me->module, false, false, false,
					     false);
		}
		/*	nx_vip_dump_register(me->module);	*/
		me->clipper_enable = clipper_enable;
		me->decimator_enable = decimator_enable;
	}
}


/**
 * public functions
 */
bool nx_vip_is_valid(u32 module)
{
	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return false;
	}

	if (_nx_vip_object[module])
		return true;
	return false;
}
EXPORT_SYMBOL_GPL(nx_vip_is_valid);

int nx_vip_reset(u32 module)
{
        struct nx_vip *me;
        int ret = 0;

        if (module >= NUMBER_OF_VIP_MODULE) {
                pr_err("[nx vip] invalid module num %d\n", module);
                return -ENODEV;
        }
        me = _nx_vip_object[module];

        if (reset_control_status(me->rst))
		reset_control_assert(me->rst);

	reset_control_deassert(me->rst);
        nx_vip_clear_input_fifo(module);

        return ret;
}
EXPORT_SYMBOL_GPL(nx_vip_reset);

int nx_vip_clock_enable(u32 module, bool enable)
{
	struct nx_vip *me;
	u8 axi_val, apb_val;
	u8 ret_axi = 0, ret_apb = 0;
	u8 ret = 0;

	ret_axi = 0x1 << 1;
	ret_apb = 0x1 << 0;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	if (enable) {
		axi_val = clk_prepare_enable(me->clk_axi);
		apb_val = clk_prepare_enable(me->clk_apb);

		ret = axi_val ? (ret | ret_axi) : (ret & ~(ret_axi));
		ret = apb_val ? (ret | ret_apb) : (ret & ~(ret_apb));

		return ret;
	}

	clk_disable_unprepare(me->clk_axi);
	clk_disable_unprepare(me->clk_apb);

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_clock_enable);

int nx_vip_padout_clock_enable(u32 module, bool enable)
{
	struct nx_vip *me;
	u8 padout1_val, padout0_val;
	u8 ret_padout1 = 0, ret_padout0 = 0;
	u8 ret = 0;

	ret_padout1 = 0x1 << 1;
	ret_padout0 = 0x1 << 0;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	if (enable) {
		padout1_val = me->clk_padout1 ?
			clk_prepare_enable(me->clk_padout1) : 0;
		padout0_val = me->clk_padout0 ?
			clk_prepare_enable(me->clk_padout0) : 0;

		ret = padout1_val ? (ret | ret_padout1) :
			(ret & ~(ret_padout1));
		ret = padout0_val ? (ret | ret_padout0) :
			(ret & ~(ret_padout0));

		return ret;
	}

	if (me->clk_padout1)
		clk_disable_unprepare(me->clk_padout1);

	if (me->clk_padout0)
		clk_disable_unprepare(me->clk_padout0);

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_padout_clock_enable);

int nx_vip_register_irq_entry(u32 module, struct nx_v4l2_irq_entry *e)
{
	unsigned long flags;
	struct nx_vip *me;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	spin_lock_irqsave(&me->lock, flags);
	if (me->irq_entry_count >= 2) {
		WARN_ON(1);
		spin_unlock_irqrestore(&me->lock, flags);
		return 0;
	}

	list_add_tail(&e->entry, &me->irq_entry_list);
	me->irq_entry_count++;
	spin_unlock_irqrestore(&me->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_register_irq_entry);

int nx_vip_unregister_irq_entry(u32 module, struct nx_v4l2_irq_entry *e)
{
	unsigned long flags;
	struct nx_vip *me;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	spin_lock_irqsave(&me->lock, flags);
	if (me->irq_entry_count <= 0) {
		WARN_ON(1);
		spin_unlock_irqrestore(&me->lock, flags);
		return 0;
	}

	list_del(&e->entry);
	me->irq_entry_count--;
	spin_unlock_irqrestore(&me->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_unregister_irq_entry);

int nx_vip_is_running(u32 module, u32 child)
{
	struct nx_vip *me;
	int vip_enable = 0, sep_enable = 0;
	int clipper_enable = 0, decimator_enable = 0;
	int ret = 0;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return 0;
	}
	me = _nx_vip_object[module];

	nx_vip_get_vipenable(me->module, &vip_enable, &sep_enable,
			&clipper_enable, &decimator_enable);

	if (child & VIP_CLIPPER)
		ret |= clipper_enable ;
	if (child & VIP_DECIMATOR)
		ret |= decimator_enable;
	ret &= vip_enable;
	return ret;
}
EXPORT_SYMBOL_GPL(nx_vip_is_running);

int nx_vip_run(u32 module, u32 child)
{
	struct nx_vip *me;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	NX_ATOMIC_SET_MASK(child, &me->running_bitmap);
	hw_child_enable(me, NX_ATOMIC_READ(&me->running_bitmap));

#ifdef DUMP_REGISTER
	nx_vip_dump_register(module);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_run);

int nx_vip_stop(u32 module, u32 child)
{
	struct nx_vip *me;

	if (module >= NUMBER_OF_VIP_MODULE) {
		pr_err("[nx vip] invalid module num %d\n", module);
		return -ENODEV;
	}
	me = _nx_vip_object[module];

	NX_ATOMIC_CLEAR_MASK(child, &me->running_bitmap);
	hw_child_enable(me, NX_ATOMIC_READ(&me->running_bitmap));

	return 0;
}
EXPORT_SYMBOL_GPL(nx_vip_stop);

/**
 * supported formats
 */
static const struct nx_bus_fmt_map supported_bus_formats[] = {
	{
		.media_bus_fmt  = MEDIA_BUS_FMT_YUYV8_2X8,
		.nx_bus_fmt	= NX_VIN_Y0CBY1CR,
	}, {
		.media_bus_fmt	= MEDIA_BUS_FMT_YVYU8_2X8,
		.nx_bus_fmt	= NX_VIN_Y1CRY0CB,
	}, {
		.media_bus_fmt	= MEDIA_BUS_FMT_UYVY8_2X8,
		.nx_bus_fmt	= NX_VIN_CBY0CRY1,
	}, {
		.media_bus_fmt	= MEDIA_BUS_FMT_VYUY8_2X8,
		.nx_bus_fmt	= NX_VIN_CRY1CBY0,
	},
};

int nx_vip_find_nx_bus_format(u32 media_bus_fmt, u32 *found)
{
	int i;
	const struct nx_bus_fmt_map *entry = &supported_bus_formats[0];

	for (i = 0; i < ARRAY_SIZE(supported_bus_formats); i++, entry++) {
		if (entry->media_bus_fmt == media_bus_fmt)  {
			*found = entry->nx_bus_fmt;
			return 0;
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(nx_vip_find_nx_bus_format);

int nx_vip_find_mbus_format(u32 nx_bus_fmt, u32 *found)
{
	int i;
	const struct nx_bus_fmt_map *entry = &supported_bus_formats[0];

	for (i = 0; i < ARRAY_SIZE(supported_bus_formats); i++, entry++) {
		if (entry->nx_bus_fmt == nx_bus_fmt)  {
			*found = entry->media_bus_fmt;
			return 0;
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(nx_vip_find_mbus_format);

static const struct nx_mem_fmt_map supported_mem_formats[] = {
	{
		.pixel_fmt	= V4L2_PIX_FMT_YUYV,
		.media_bus_fmt	= MEDIA_BUS_FMT_YUYV8_1X16,
		.nx_mem_fmt	= nx_vip_format_yuyv,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_YUV420,
		.media_bus_fmt	= MEDIA_BUS_FMT_YUYV12_1X24,
		.nx_mem_fmt	= nx_vip_format_420,
	}, {
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_YVU420,
		.media_bus_fmt	= MEDIA_BUS_FMT_YVYU12_1X24,
		.nx_mem_fmt	= nx_vip_format_420,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_YUV422P,
		.media_bus_fmt	= MEDIA_BUS_FMT_YDYUYDYV8_1X16,
		.nx_mem_fmt	= nx_vip_format_422,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_YUV444,
		.media_bus_fmt	= MEDIA_BUS_FMT_AYUV8_1X32,
		.nx_mem_fmt	= nx_vip_format_444,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV12,
		.media_bus_fmt	= MEDIA_BUS_FMT_YUYV12_2X12,
		.nx_mem_fmt	= nx_vip_format_420_cbcr,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV16,
		.media_bus_fmt	= MEDIA_BUS_FMT_YUYV8_2X8,
		.nx_mem_fmt	= nx_vip_format_422_cbcr,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV24,
		.media_bus_fmt	= MEDIA_BUS_FMT_YUYV8_1_5X8,
		.nx_mem_fmt	= nx_vip_format_444_cbcr,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV21,
		.media_bus_fmt	= MEDIA_BUS_FMT_YVYU12_2X12,
		.nx_mem_fmt	= nx_vip_format_420_crcb,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV61,
		.media_bus_fmt	= MEDIA_BUS_FMT_YVYU8_2X8,
		.nx_mem_fmt	= nx_vip_format_422_crcb,
	}, {
		.pixel_fmt	= V4L2_PIX_FMT_NV42,
		.media_bus_fmt	= MEDIA_BUS_FMT_YVYU8_1_5X8,
		.nx_mem_fmt	= nx_vip_format_444_crcb,
	}
};

int nx_vip_find_nx_mem_format(u32 media_bus_fmt, u32 *found)
{
	int i;
	const struct nx_mem_fmt_map *entry = &supported_mem_formats[0];

	for (i = 0; i < ARRAY_SIZE(supported_mem_formats); i++, entry++) {
		if (entry->media_bus_fmt == media_bus_fmt)  {
			*found = entry->nx_mem_fmt;
			return 0;
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(nx_vip_find_nx_mem_format);

int nx_vip_find_mbus_mem_format(u32 nx_mem_fmt, u32 *found)
{
	int i;
	const struct nx_mem_fmt_map *entry = &supported_mem_formats[0];

	for (i = 0; i < ARRAY_SIZE(supported_mem_formats); i++, entry++) {
		if (entry->nx_mem_fmt == nx_mem_fmt)  {
			*found = entry->media_bus_fmt;
			return 0;
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL_GPL(nx_vip_find_mbus_mem_format);

/**
 * platform driver specific
 */
static int nx_vip_probe(struct platform_device *pdev)
{
	int ret;
	struct nx_vip *me;

	me = devm_kzalloc(&pdev->dev, sizeof(*me), GFP_KERNEL);
	if (!me) {
		WARN_ON(1);
		return -ENOMEM;
	}

	ret = nx_vip_parse_dt(pdev, me);
	if (ret)
		return ret;

	if (_nx_vip_object[me->module]) {
		dev_err(&pdev->dev, "already nx vip %d registered\n",
			me->module);
		return -EBUSY;
	}

	_nx_vip_object[me->module] = me;

	nx_vip_set_base_address(me->module, me->base);
	INIT_LIST_HEAD(&me->irq_entry_list);
	spin_lock_init(&me->lock);

	nx_vip_reset(me->module);
	nx_vip_clock_enable(me->module, true);
	snprintf(me->irq_name, sizeof(me->irq_name), "nx-vip%d", me->module);
	ret = devm_request_irq(&pdev->dev, me->irq, &vip_irq_handler,
			       IRQF_SHARED, me->irq_name, me);
	if (ret) {
		dev_err(&pdev->dev, "failed to devm_request_irq for vip %d\n",
			me->module);
		return ret;
	}

	platform_set_drvdata(pdev, me);

	return 0;
}

static int nx_vip_remove(struct platform_device *pdev)
{
	struct nx_vip *me = platform_get_drvdata(pdev);

	if (me) {
		nx_vip_clock_enable(me->module, false);
		_nx_vip_object[me->module] = NULL;
	}

	return 0;
}

static struct platform_device_id nx_vip_id_table[] = {
	{ NX_VIP_DEV_NAME, 0},
	{},
};

static const struct of_device_id nx_vip_dt_match[] = {
	{ .compatible = "nexell,nxp3220-vip" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_vip_dt_match);

static struct platform_driver nx_vip_driver = {
	.probe		= nx_vip_probe,
	.remove		= nx_vip_remove,
	.id_table	= nx_vip_id_table,
	.driver = {
		.name	= NX_VIP_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nx_vip_dt_match),
	},
};

#ifdef CONFIG_DEFERRED_UP_VIP
static int __init nx_vip_init(void)
{
        return platform_driver_register(&nx_vip_driver);
}

subsys_initcall(nx_vip_init);
#else
module_platform_driver(nx_vip_driver);
#endif

MODULE_AUTHOR("JongKeun Choi<jkchoi@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell NXP3220 SoC VIP device driver");
MODULE_LICENSE("GPL");
