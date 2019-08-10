// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell BUS DEVFREQ driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

#include "governor.h"

#define	MHZ(f)	(f/1000000)
#define	mV(v)	(v/1000)

struct nxp3220_busfreq {
	struct device *dev;
	struct devfreq *devfreq;
	const char *name;
	struct clk *clk;
	struct mutex lock;
	struct regulator *vdd;
	unsigned long rate;
	unsigned long volt;
	struct devfreq_dev_profile *profile;
	struct device_node *passive_node;
	void *private_data;	/* for governor data */
};

static int nxp3220_bus_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct nxp3220_busfreq *bus = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long target_volt, target_rate, old_rate;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	old_rate = bus->rate;
	target_rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	if (bus->rate == target_rate)
		return 0;

	mutex_lock(&bus->lock);

	if (!IS_ERR(bus->vdd) && old_rate < target_rate) {
		ret = regulator_set_voltage(bus->vdd, target_volt,
					    target_volt);
		if (ret) {
			dev_err(dev, "Cannot to set vol %lu uV\n", target_volt);
			goto out;
		}
	}

	ret = clk_set_rate(bus->clk, target_rate);
	if (ret) {
		dev_err(dev, "Cannot to set frequency %lu (%d)\n",
			target_rate, ret);
		if (!IS_ERR(bus->vdd))
			regulator_set_voltage(bus->vdd, bus->volt, bus->volt);
		goto out;
	}

	if (!IS_ERR(bus->vdd) && old_rate > target_rate) {
		ret = regulator_set_voltage(bus->vdd, target_volt, target_volt);
		if (ret) {
			dev_err(dev, "Cannot to set vol %lu uV\n", target_volt);
			goto out;
		}
	}

	dev_dbg(dev,
		"BUS: %s, %luMHz/%ldmV -> %lu(%ld)MHz/%ldmV [vol:%s]\n",
		bus->name, MHZ(bus->rate), mV(bus->volt),
		MHZ(target_rate), MHZ(clk_get_rate(bus->clk)), mV(target_volt),
		!IS_ERR(bus->vdd) ? "set" : "pass");

	bus->rate = target_rate;
	bus->volt = target_volt;

out:
	mutex_unlock(&bus->lock);

	return ret;
}

static int nxp3220_bus_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct nxp3220_busfreq *bus = dev_get_drvdata(dev);

	dev_dbg(dev, "get_dev_status: %s, %lu KHz, %ld v\n",
		bus->name, bus->rate, bus->volt);

	stat->current_frequency = bus->rate;
	stat->private_data = bus;

	return 0;
}

static int nxp3220_bus_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct nxp3220_busfreq *bus = dev_get_drvdata(dev);

	dev_dbg(dev, "get_cur_freq: %s, %lu KHz, %ld v\n",
		bus->name, bus->rate, bus->volt);

	*freq = bus->rate;

	return 0;
}

static int nxp3220_bus_suspend(struct device *dev)
{
	struct nxp3220_busfreq *bus = dev_get_drvdata(dev);
	int ret;

	ret = devfreq_suspend_device(bus->devfreq);
	if (ret < 0)
		dev_err(dev, "failed to suspend the devfreq devices\n");

	return 0;
}

static int nxp3220_bus_resume(struct device *dev)
{
	struct nxp3220_busfreq *bus = dev_get_drvdata(dev);
	int ret;

	ret = devfreq_resume_device(bus->devfreq);
	if (ret < 0)
		dev_err(dev, "failed to resume the devfreq devices\n");

	return ret;
}

static SIMPLE_DEV_PM_OPS(nxp3220_bus_pm,
		nxp3220_bus_suspend, nxp3220_bus_resume);

static int nxp3220_bus_parse_of(struct device *dev,
				struct nxp3220_busfreq *bus)
{
	int ret;

	of_property_read_string(dev->of_node, "bus-name", &bus->name);

	bus->clk = devm_clk_get(dev, "bus");
	if (IS_ERR(bus->clk)) {
		dev_err(dev, "Cannot get the clk %s\n", bus->name);
		return PTR_ERR(bus->clk);
	}
	bus->rate = clk_get_rate(bus->clk);

	bus->passive_node = of_parse_phandle(dev->of_node, "devfreq", 0);
	if (bus->passive_node) {
		bus->vdd = ERR_PTR(-ENOENT);
		of_node_put(bus->passive_node);
		return 0;
	}

	/* get regulator from "vdd-supply" node */
	bus->vdd = regulator_get_optional(dev, "vdd");
	if (!IS_ERR(bus->vdd)) {
		ret = regulator_enable(bus->vdd);
		if (ret < 0) {
			dev_err(dev,
				"failed to enable vdd for %s\n", bus->name);
			return ret;
		}
		bus->volt = regulator_get_voltage(bus->vdd);
	}

	if (IS_ERR(bus->vdd) &&
	    of_parse_phandle(dev->of_node, "vdd-supply", 0)) {
		dev_err(dev, "failed to get %s VDD regulator\n", bus->name);
		return -ENODEV;
	}

	return 0;
}

static int nxp3220_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_opp *opp;
	struct devfreq_dev_profile *profile;
	struct nxp3220_busfreq *bus;
	void *governor_data;
	const char *governor_name;
	int ret;

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	ret = nxp3220_bus_parse_of(dev, bus);
	if (ret < 0)
		return ret;

	profile = devm_kzalloc(dev, sizeof(*profile), GFP_KERNEL);
	if (!profile)
		return -ENOMEM;

	profile->target = nxp3220_bus_target;
	profile->get_dev_status = nxp3220_bus_get_dev_status;
	profile->get_cur_freq = nxp3220_bus_get_cur_freq;

	if (dev_pm_opp_of_add_table(dev)) {
		dev_err(dev,
			"Invalid %s operating-points in device tree.\n",
			bus->name);
		return -EINVAL;
	}

	opp = devfreq_recommended_opp(dev, &bus->rate, 0);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto err;
	}

	bus->rate = dev_pm_opp_get_freq(opp);
	bus->volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	mutex_init(&bus->lock);
	bus->dev = dev;
	bus->profile = profile;

	governor_name = "userspace";
	governor_data = bus;

	if (bus->passive_node) {
		struct devfreq *parent;
		struct devfreq_passive_data *passive;

		parent = devfreq_get_devfreq_by_phandle(dev, 0);
		if (IS_ERR(parent)) {
			ret = -EPROBE_DEFER;
			goto err;
		}

		passive = devm_kzalloc(dev, sizeof(*passive), GFP_KERNEL);
		if (!passive) {
			ret = -ENOMEM;
			goto err;
		}

		passive->parent = parent;
		bus->private_data = passive;

		governor_name = "passive";
		governor_data = passive;
	}

	bus->devfreq = devm_devfreq_add_device(dev,
					   bus->profile,
					   governor_name, governor_data);
	if (IS_ERR(bus->devfreq)) {
		ret = PTR_ERR(bus->devfreq);
		goto err;
	}

	bus->devfreq->previous_freq = bus->rate;
	devm_devfreq_register_opp_notifier(dev, bus->devfreq);

	platform_set_drvdata(pdev, bus);

	dev_info(dev, "%s, bus devfreq %ld MHz, %ldmV\n",
			bus->name, MHZ(bus->rate), mV(bus->volt));

	return 0;
err:
	dev_pm_opp_of_remove_table(dev);
	return ret;
}

static const struct of_device_id nxp3220_bus_of_match[] = {
	{ .compatible = "nexell,nxp3220-bus" },
	{ },
};
MODULE_DEVICE_TABLE(of, nxp3220_bus_of_match);

static struct platform_driver nxp3220_bus_driver = {
	.probe	= nxp3220_bus_probe,
	.driver = {
		.name	= "nexell-bus-devfreq",
		.pm	= &nxp3220_bus_pm,
		.of_match_table = nxp3220_bus_of_match,
	},
};
module_platform_driver(nxp3220_bus_driver);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell SoC bus devfreq driver");
MODULE_LICENSE("GPL");
