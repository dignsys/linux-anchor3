// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#define pr_fmt(fmt) "CPUidle-nexell: " fmt

#include <linux/cpuidle.h>
#include <asm/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/topology.h>

#include "dt_idle_states.h"

#define NEXELL_MAX_STATES		1

static int nexell_enter_idle(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();

	return index;
}

static struct cpuidle_driver nexell_idle_driver = {
	.name = "nexell_idle",
	.owner = THIS_MODULE,
	.states = {
		ARM_CPUIDLE_WFI_STATE,
	},
	.safe_state_index = 0,
	.state_count = NEXELL_MAX_STATES,
};

static const struct of_device_id nexell_idle_state_match[] __initconst = {
	{ .compatible = "nexell,idle-state",
	  .data = nexell_enter_idle },
	{ },
};

static int __init nexell_idle_init(void)
{
	int cpu, ret;
	struct cpuidle_driver *drv;
	struct cpuidle_device *dev;

	for_each_possible_cpu(cpu) {

		drv = kmemdup(&nexell_idle_driver, sizeof(*drv), GFP_KERNEL);
		if (!drv) {
			ret = -ENOMEM;
			goto out_fail;
		}

		drv->cpumask = (struct cpumask *)cpumask_of(cpu);

		/*
		 * Initialize idle states data, starting at index 1.  This
		 * driver is DT only, if no DT idle states are detected (ret
		 * == 0) let the driver initialization fail accordingly since
		 * there is no reason to initialize the idle driver if only
		 * wfi is supported.
		 */
		ret = dt_init_idle_driver(drv, nexell_idle_state_match, 1);
		if (ret <= 0) {
			ret = ret ? : -ENODEV;
			goto out_kfree_drv;
		}

		ret = cpuidle_register_driver(drv);
		if (ret) {
			pr_err("Failed to register cpuidle driver\n");
			goto out_kfree_drv;
		}

		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev) {
			pr_err("Failed to allocate cpuidle device\n");
			ret = -ENOMEM;
			goto out_unregister_drv;
		}
		dev->cpu = cpu;

		ret = cpuidle_register_device(dev);
		if (ret) {
			pr_err("Failed to register cpuidle device for CPU %d\n",
			       cpu);
			goto out_kfree_dev;
		}
	}

	return 0;

out_kfree_dev:
	kfree(dev);
out_unregister_drv:
	cpuidle_unregister_driver(drv);
out_kfree_drv:
	kfree(drv);
out_fail:
	while (--cpu >= 0) {
		dev = per_cpu(cpuidle_devices, cpu);
		drv = cpuidle_get_cpu_driver(dev);
		cpuidle_unregister_device(dev);
		cpuidle_unregister_driver(drv);
		kfree(dev);
		kfree(drv);
	}

	return ret;
}
device_initcall(nexell_idle_init);
