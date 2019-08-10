// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell USB2 PHY driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define MAX_PHYS	4

struct nx_usb2_phy {
	struct phy *phy;
	void __iomem *base;
	const char *label;
	const char *vbus_gpio;
	const char *vbus_tune;
	int port;
	int bus_width;
	int vbus_tune_val;
	struct nx_usb2_phy_pdata *pdata;
	int (*probe)(struct nx_usb2_phy *p);
	int (*power_on)(struct nx_usb2_phy *p);
	int (*power_off)(struct nx_usb2_phy *p);
};

struct nx_usb2_phy_config {
	struct nx_usb2_phy *phys;
	int num_phys;
};

struct nx_usb2_phy_pdata {
	void __iomem *base;
	struct device *dev;
	spinlock_t lock;
	struct clk *clk_ahb, *clk_apb;
	struct regulator *phy_supply;
	atomic_t refcount;
	const struct nx_usb2_phy_config *cfg;
	struct nx_usb2_phy *phys[MAX_PHYS];
};

static int nx_usb2_phy_power_on(struct phy *phy)
{
	struct nx_usb2_phy *p = phy_get_drvdata(phy);
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	int ret = 0;

	dev_dbg(pdata->dev, "Request power on '%s'\n", p->label);

	if (atomic_read(&pdata->refcount) == 0) {
		if (pdata->phy_supply) {
			ret = regulator_enable(pdata->phy_supply);
			if (ret) {
				dev_err(pdata->dev,
					"Failed to enabling supply !!!\n");
				return ret;
			}
		}

		if (!IS_ERR(pdata->clk_ahb) &&
		    !__clk_is_enabled(pdata->clk_ahb)) {
			ret = clk_prepare_enable(pdata->clk_ahb);
			if (ret)
				return ret;
		}
		if (!IS_ERR(pdata->clk_apb) &&
		    !__clk_is_enabled(pdata->clk_apb)) {
			ret = clk_prepare_enable(pdata->clk_apb);
			if (ret)
				return ret;
		}
	}

	if (p->power_on)
		ret = p->power_on(p);

	if (!ret)
		atomic_inc(&pdata->refcount);

	return ret;
}

static int nx_usb2_phy_power_off(struct phy *phy)
{
	struct nx_usb2_phy *p = phy_get_drvdata(phy);
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	int ret = 0;

	dev_dbg(pdata->dev, "Request power off '%s'\n", p->label);

	if (p->power_off) {
		ret = p->power_off(p);
		if (ret)
			return ret;
	}

	if (!ret) {
		atomic_dec(&pdata->refcount);
		if (atomic_read(&pdata->refcount) == 0) {
			if (!IS_ERR(pdata->clk_ahb))
				clk_disable_unprepare(pdata->clk_ahb);

			if (!IS_ERR(pdata->clk_apb))
				clk_disable_unprepare(pdata->clk_apb);

			if (pdata->phy_supply)
				ret = regulator_enable(pdata->phy_supply);
		}
	}

	return ret;
}

static const struct phy_ops nx_usb2_phy_ops = {
	.power_on = nx_usb2_phy_power_on,
	.power_off = nx_usb2_phy_power_off,
	.owner = THIS_MODULE,
};

static struct phy *nx_usb2_phy_xlate(struct device *dev,
				     struct of_phandle_args *args)
{
	struct nx_usb2_phy_pdata *pdata = dev_get_drvdata(dev);
	struct nx_usb2_phy *p;
	int index = args->args[0];

	if (!pdata)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(index > (pdata->cfg->num_phys - 1)))
		return ERR_PTR(-ENODEV);

	p = pdata->phys[index];

	dev_dbg(dev, "xlate phy[%d]: %s\n", index, p->label);

	return p->phy;
}

#ifdef CONFIG_PHY_NXP3220_USB2
#include "phy-nxp3220-usb2.c"
#endif

static const struct of_device_id nx_usb2_phy_of_match[] = {
#ifdef CONFIG_PHY_NXP3220_USB2
	{
		.compatible = "nexell,nxp3220-usb2-phy",
		.data = &nxp3220_usb2_phy_cfg,
	},
#endif
	{ },
};
MODULE_DEVICE_TABLE(of, nx_usb2_phy_of_match);

static void nx_usb2_phy_vbus_gpio(struct device *dev, struct nx_usb2_phy *p)
{
	int gpio;

	if (!dev->of_node || !p->vbus_gpio)
		return;

	gpio = of_get_named_gpio(dev->of_node, p->vbus_gpio, 0);
	if (!gpio_is_valid(gpio))
		return;

	devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_HIGH, p->vbus_gpio);

	dev_info(dev, "gpio.%d vbus %s", gpio, p->vbus_gpio);
}

static int nx_usb2_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct resource *res;
	struct nx_usb2_phy_pdata *pdata;
	const struct nx_usb2_phy_config *cfg;
	int i, ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	cfg = of_device_get_match_data(dev);
	if (!cfg) {
		dev_err(dev, "Missing usb phy match config ...\n");
		return -EINVAL;
	}

	pdata->cfg = cfg;
	pdata->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->base))
		return PTR_ERR(pdata->base);

	pdata->clk_ahb = devm_clk_get(dev, "ahb");
	if (!IS_ERR(pdata->clk_ahb)) {
		ret = clk_prepare_enable(pdata->clk_ahb);
		if (ret)
			return ret;
	}

	pdata->clk_apb = devm_clk_get(dev, "apb");
	if (!IS_ERR(pdata->clk_apb)) {
		ret = clk_prepare_enable(pdata->clk_apb);
		if (ret)
			return ret;
	}

	pdata->phy_supply = devm_regulator_get(dev, "phy");
	if (pdata->phy_supply) {
		ret = regulator_enable(pdata->phy_supply);
		if (ret) {
			dev_err(pdata->dev,
				"Failed to enabling supply !!!\n");
			return ret;
		}
	}

	dev_info(dev, "phy mapped PA %08lx to VA %p\n",
			(unsigned long)res->start, pdata->base);

	for (i = 0; i < cfg->num_phys; i++) {
		struct nx_usb2_phy *p = &cfg->phys[i];

		if (i > (MAX_PHYS - 1)) {
			dev_err(dev, "Over max phy.%d num %d\n", i, MAX_PHYS);
			break;
		}

		p->pdata = pdata;
		p->phy = devm_phy_create(dev, NULL, &nx_usb2_phy_ops);
		if (IS_ERR(p->phy)) {
			dev_err(dev, "Failed to phy create '%s'\n", p->label);
			return PTR_ERR(p->phy);
		}

		nx_usb2_phy_vbus_gpio(dev, p);

		if (p->probe) {
			ret = p->probe(p);
			if (ret)
				continue;
		}

		if (p->vbus_tune)
			of_property_read_u32(dev->of_node,
					p->vbus_tune, &p->vbus_tune_val);

		pdata->phys[i] = p;
		phy_set_bus_width(p->phy, p->bus_width);
		phy_set_drvdata(p->phy, p);

		dev_dbg(dev, "phy %s bus %d\n", p->label, p->bus_width);
	}

	provider = devm_of_phy_provider_register(dev, nx_usb2_phy_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "Failed to register phy provider\n");
		return PTR_ERR(provider);
	}

	dev_set_drvdata(dev, pdata);
	spin_lock_init(&pdata->lock);

	return 0;
}

static struct platform_driver nx_usb2_phy_driver = {
	.probe	= nx_usb2_phy_probe,
	.driver = {
		.of_match_table	= nx_usb2_phy_of_match,
		.name = "nexell-usb2-phy",
	}
};

module_platform_driver(nx_usb2_phy_driver);

MODULE_DESCRIPTION("Nexell SoC USB2 PHY driver");
MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_LICENSE("GPL");
