/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Youngbok, Park <ybpark@nexell.co.kr>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/pm_runtime.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define SDMMC_MODE			0x400
#define SDMMC_SRAM			0x404
#define SDMMC_DRV_PHASE			0x408
#define SDMMC_SMP_PHASE			0x40c

#define SRAM_AWAKE                      0x1
#define SRAM_SLEEP                      0x0

#define DWMMC_PRE_DIV			4

struct dw_mci_nexell_priv_data {
	u32	drv_phase;
	u32	smp_phase;
};

static int dw_mci_nexell_set_priv(struct dw_mci *host)
{
	struct dw_mci_nexell_priv_data *priv = host->priv;

	mci_writel(host, DRV_PHASE, priv->drv_phase);
	mci_writel(host, SMP_PHASE, priv->smp_phase);

	mci_writel(host, SRAM, SRAM_AWAKE);

	return 0;
}

static int dw_mci_nexell_priv_init(struct dw_mci *host)
{
	dw_mci_nexell_set_priv(host);

	host->bus_hz /= DWMMC_PRE_DIV;

	return 0;
}

static void dw_mci_nexell_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	u32 mode = 0;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		mode = 0;
		break;
	case MMC_BUS_WIDTH_4:
		mode = 1;
		break;
	case MMC_BUS_WIDTH_8:
		mode = 2;
		break;
	}

	mci_writel(host, MODE, mode);
}

#ifdef CONFIG_PM
static int dw_mci_nexell_runtime_resume(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = dw_mci_runtime_resume(dev);
	if (!ret)
		ret = dw_mci_nexell_set_priv(host);

	return ret;
}
#endif

static int dw_mci_nexell_parse_dt(struct dw_mci *host)
{
	struct dw_mci_nexell_priv_data *priv;
	struct device_node *np = host->dev->of_node;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (of_property_read_u32(np, "nexell,drive_shift", &priv->drv_phase))
		priv->drv_phase = 4;

	if (of_property_read_u32(np, "nexell,sample_shift", &priv->smp_phase))
		priv->smp_phase = 2;

	host->priv = priv;

	return 0;
}

/* Common capabilities of sip_s31nx SoC */
static unsigned long nexell_dwmmc_caps[4] = {
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
};

static const struct dw_mci_drv_data nexell_drv_data = {
	.caps			= nexell_dwmmc_caps,
	.num_caps		= ARRAY_SIZE(nexell_dwmmc_caps),
	.init			= dw_mci_nexell_priv_init,
	.parse_dt		= dw_mci_nexell_parse_dt,
	.set_ios		= dw_mci_nexell_set_ios,
};

static const struct of_device_id dw_mci_nexell_match[] = {
	{.compatible = "nexell,nxp3220-dw-mshc",
			.data = &nexell_drv_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_nexell_match);

static int dw_mci_nexell_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct dw_mci_nexell_priv_data *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	match = of_match_node(dw_mci_nexell_match, pdev->dev.of_node);
	drv_data = match->data;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = dw_mci_pltfm_register(pdev, drv_data);
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);

		return ret;
	}

	return 0;
}

static int dw_mci_nexell_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return dw_mci_pltfm_remove(pdev);
}

static const struct dev_pm_ops dw_mci_nexell_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(dw_mci_runtime_suspend,
				dw_mci_nexell_runtime_resume,
				NULL)
};

static struct platform_driver dw_mci_nexell_pltfm_driver = {
	.probe		= dw_mci_nexell_probe,
	.remove		= dw_mci_nexell_remove,
	.driver		= {
		.name		= "dwmmc_nexell",
		.of_match_table	= dw_mci_nexell_match,
		.pm		= &dw_mci_nexell_pmops,
	},
};
#if defined(CONFIG_DEFERRED_UP_MMC)
static int __init dw_mmc_nexell_init(void)
{
	return platform_driver_register(&dw_mci_nexell_pltfm_driver);
}
subsys_initcall(dw_mmc_nexell_init);
#else
module_platform_driver(dw_mci_nexell_pltfm_driver);
#endif

MODULE_DESCRIPTION("Nexell Specific DW-MSHC Driver Extension");
MODULE_AUTHOR("Youngbok Park <ybpark@nexell.co.kr");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwmmc_nexell");
