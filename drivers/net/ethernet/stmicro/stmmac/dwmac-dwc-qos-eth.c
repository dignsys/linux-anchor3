/*
 * Synopsys DWC Ethernet Quality-of-Service v4.10a linux driver
 *
 * Copyright (C) 2016 Joao Pinto <jpinto@synopsys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"
#include "dwmac4.h"

#define DWMAC_125MHZ	125000000
#define DWMAC_50MHZ	50000000
#define DWMAC_25MHZ	25000000
#define DWMAC_2_5MHZ	2500000

struct tegra_eqos {
	struct device *dev;
	void __iomem *regs;

	struct reset_control *rst;
	struct clk *clk_master;
	struct clk *clk_slave;
	struct clk *clk_tx;
	struct clk *clk_rx;

	struct gpio_desc *reset;
};

struct nxp3220_eqos {
	struct device *dev;
	void __iomem *regs;

	struct reset_control *rst;
	struct clk *clk_master;
	struct clk *clk_slave;
	struct clk *clk_tx;
	struct clk *ptp_ref;
	int wolopts;
	int phy_wol_irq;

	struct gpio_desc *phy_reset;
	struct gpio_desc *phy_intr;
	struct gpio_desc *phy_pme;
};

static void nxp3220_get_wol(struct net_device *ndev, struct ethtool_wolinfo *wol)
{
	struct stmmac_priv *stpriv = netdev_priv(ndev);
	wol->supported = WAKE_MAGIC;
	wol->wolopts = stpriv->wolopts;
}

static int nxp3220_set_wol(struct net_device *ndev, struct ethtool_wolinfo *wol)
{
	struct stmmac_priv *stpriv = netdev_priv(ndev);
	struct nxp3220_eqos *eqos = stpriv->plat->bsp_priv;
	u32 support = WAKE_MAGIC;
	int err;

	if (wol->wolopts & ~support)
		return -EOPNOTSUPP;

	err = phy_ethtool_set_wol(ndev->phydev, wol);
	if (err < 0) {
		dev_err(stpriv->device, "The PHY does not support set_wol\n");
		return -EOPNOTSUPP;
	}

	if (wol->wolopts)
		enable_irq_wake(eqos->phy_wol_irq);
	else
		disable_irq_wake(eqos->phy_wol_irq);

	mutex_lock(&stpriv->lock);
	stpriv->wolopts |= wol->wolopts;
	mutex_unlock(&stpriv->lock);

	return 0;
}

static int dwc_eth_dwmac_config_dt(struct platform_device *pdev,
				   struct plat_stmmacenet_data *plat_dat)
{
	struct device_node *np = pdev->dev.of_node;
	u32 burst_map = 0;
	u32 bit_index = 0;
	u32 a_index = 0;

	if (!plat_dat->axi) {
		plat_dat->axi = kzalloc(sizeof(struct stmmac_axi), GFP_KERNEL);

		if (!plat_dat->axi)
			return -ENOMEM;
	}

	plat_dat->axi->axi_lpi_en = of_property_read_bool(np, "snps,en-lpi");
	if (of_property_read_u32(np, "snps,write-requests",
				 &plat_dat->axi->axi_wr_osr_lmt)) {
		/**
		 * Since the register has a reset value of 1, if property
		 * is missing, default to 1.
		 */
		plat_dat->axi->axi_wr_osr_lmt = 1;
	} else {
		/**
		 * If property exists, to keep the behavior from dwc_eth_qos,
		 * subtract one after parsing.
		 */
		plat_dat->axi->axi_wr_osr_lmt--;
	}

	if (of_property_read_u32(np, "snps,read-requests",
				 &plat_dat->axi->axi_rd_osr_lmt)) {
		/**
		 * Since the register has a reset value of 1, if property
		 * is missing, default to 1.
		 */
		plat_dat->axi->axi_rd_osr_lmt = 1;
	} else {
		/**
		 * If property exists, to keep the behavior from dwc_eth_qos,
		 * subtract one after parsing.
		 */
		plat_dat->axi->axi_rd_osr_lmt--;
	}
	of_property_read_u32(np, "snps,burst-map", &burst_map);

	/* converts burst-map bitmask to burst array */
	for (bit_index = 0; bit_index < 7; bit_index++) {
		if (burst_map & (1 << bit_index)) {
			switch (bit_index) {
			case 0:
			plat_dat->axi->axi_blen[a_index] = 4; break;
			case 1:
			plat_dat->axi->axi_blen[a_index] = 8; break;
			case 2:
			plat_dat->axi->axi_blen[a_index] = 16; break;
			case 3:
			plat_dat->axi->axi_blen[a_index] = 32; break;
			case 4:
			plat_dat->axi->axi_blen[a_index] = 64; break;
			case 5:
			plat_dat->axi->axi_blen[a_index] = 128; break;
			case 6:
			plat_dat->axi->axi_blen[a_index] = 256; break;
			default:
			break;
			}
			a_index++;
		}
	}

	/* dwc-qos needs GMAC4, AAL, TSO and PMT */
	plat_dat->has_gmac4 = 1;
	plat_dat->dma_cfg->aal = 1;
	plat_dat->tso_en = 1;
	plat_dat->pmt = 1;

	return 0;
}

static void *dwc_qos_probe(struct platform_device *pdev,
			   struct plat_stmmacenet_data *plat_dat,
			   struct stmmac_resources *stmmac_res)
{
	int err;

	plat_dat->stmmac_clk = devm_clk_get(&pdev->dev, "apb_pclk");
	if (IS_ERR(plat_dat->stmmac_clk)) {
		dev_err(&pdev->dev, "apb_pclk clock not found.\n");
		return ERR_CAST(plat_dat->stmmac_clk);
	}

	err = clk_prepare_enable(plat_dat->stmmac_clk);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to enable apb_pclk clock: %d\n",
			err);
		return ERR_PTR(err);
	}

	plat_dat->pclk = devm_clk_get(&pdev->dev, "phy_ref_clk");
	if (IS_ERR(plat_dat->pclk)) {
		dev_err(&pdev->dev, "phy_ref_clk clock not found.\n");
		err = PTR_ERR(plat_dat->pclk);
		goto disable;
	}

	err = clk_prepare_enable(plat_dat->pclk);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to enable phy_ref clock: %d\n",
			err);
		goto disable;
	}

	return NULL;

disable:
	clk_disable_unprepare(plat_dat->stmmac_clk);
	return ERR_PTR(err);
}

static int dwc_qos_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	clk_disable_unprepare(priv->plat->pclk);
	clk_disable_unprepare(priv->plat->stmmac_clk);

	return 0;
}

#define SDMEMCOMPPADCTRL 0x8800
#define  SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD BIT(31)

#define AUTO_CAL_CONFIG 0x8804
#define  AUTO_CAL_CONFIG_START BIT(31)
#define  AUTO_CAL_CONFIG_ENABLE BIT(29)

#define AUTO_CAL_STATUS 0x880c
#define  AUTO_CAL_STATUS_ACTIVE BIT(31)

static void tegra_eqos_fix_speed(void *priv, unsigned int speed)
{
	struct tegra_eqos *eqos = priv;
	unsigned long rate = 125000000;
	bool needs_calibration = false;
	u32 value;
	int err;

	switch (speed) {
	case SPEED_1000:
		needs_calibration = true;
		rate = 125000000;
		break;

	case SPEED_100:
		needs_calibration = true;
		rate = 25000000;
		break;

	case SPEED_10:
		rate = 2500000;
		break;

	default:
		dev_err(eqos->dev, "invalid speed %u\n", speed);
		break;
	}

	if (needs_calibration) {
		/* calibrate */
		value = readl(eqos->regs + SDMEMCOMPPADCTRL);
		value |= SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD;
		writel(value, eqos->regs + SDMEMCOMPPADCTRL);

		udelay(1);

		value = readl(eqos->regs + AUTO_CAL_CONFIG);
		value |= AUTO_CAL_CONFIG_START | AUTO_CAL_CONFIG_ENABLE;
		writel(value, eqos->regs + AUTO_CAL_CONFIG);

		err = readl_poll_timeout_atomic(eqos->regs + AUTO_CAL_STATUS,
						value,
						value & AUTO_CAL_STATUS_ACTIVE,
						1, 10);
		if (err < 0) {
			dev_err(eqos->dev, "calibration did not start\n");
			goto failed;
		}

		err = readl_poll_timeout_atomic(eqos->regs + AUTO_CAL_STATUS,
						value,
						(value & AUTO_CAL_STATUS_ACTIVE) == 0,
						20, 200);
		if (err < 0) {
			dev_err(eqos->dev, "calibration didn't finish\n");
			goto failed;
		}

	failed:
		value = readl(eqos->regs + SDMEMCOMPPADCTRL);
		value &= ~SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD;
		writel(value, eqos->regs + SDMEMCOMPPADCTRL);
	} else {
		value = readl(eqos->regs + AUTO_CAL_CONFIG);
		value &= ~AUTO_CAL_CONFIG_ENABLE;
		writel(value, eqos->regs + AUTO_CAL_CONFIG);
	}

	err = clk_set_rate(eqos->clk_tx, rate);
	if (err < 0)
		dev_err(eqos->dev, "failed to set TX rate: %d\n", err);
}

static int tegra_eqos_init(struct platform_device *pdev, void *priv)
{
	struct tegra_eqos *eqos = priv;
	unsigned long rate;
	u32 value;

	rate = clk_get_rate(eqos->clk_slave);

	value = (rate / 1000000) - 1;
	writel(value, eqos->regs + GMAC_1US_TIC_COUNTER);

	return 0;
}

static void *tegra_eqos_probe(struct platform_device *pdev,
			      struct plat_stmmacenet_data *data,
			      struct stmmac_resources *res)
{
	struct tegra_eqos *eqos;
	int err;

	eqos = devm_kzalloc(&pdev->dev, sizeof(*eqos), GFP_KERNEL);
	if (!eqos) {
		err = -ENOMEM;
		goto error;
	}

	eqos->dev = &pdev->dev;
	eqos->regs = res->addr;

	eqos->clk_master = devm_clk_get(&pdev->dev, "master_bus");
	if (IS_ERR(eqos->clk_master)) {
		err = PTR_ERR(eqos->clk_master);
		goto error;
	}

	err = clk_prepare_enable(eqos->clk_master);
	if (err < 0)
		goto error;

	eqos->clk_slave = devm_clk_get(&pdev->dev, "slave_bus");
	if (IS_ERR(eqos->clk_slave)) {
		err = PTR_ERR(eqos->clk_slave);
		goto disable_master;
	}

	data->stmmac_clk = eqos->clk_slave;

	err = clk_prepare_enable(eqos->clk_slave);
	if (err < 0)
		goto disable_master;

	eqos->clk_rx = devm_clk_get(&pdev->dev, "rx");
	if (IS_ERR(eqos->clk_rx)) {
		err = PTR_ERR(eqos->clk_rx);
		goto disable_slave;
	}

	err = clk_prepare_enable(eqos->clk_rx);
	if (err < 0)
		goto disable_slave;

	eqos->clk_tx = devm_clk_get(&pdev->dev, "tx");
	if (IS_ERR(eqos->clk_tx)) {
		err = PTR_ERR(eqos->clk_tx);
		goto disable_rx;
	}

	err = clk_prepare_enable(eqos->clk_tx);
	if (err < 0)
		goto disable_rx;

	eqos->reset = devm_gpiod_get(&pdev->dev, "phy-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(eqos->reset)) {
		err = PTR_ERR(eqos->reset);
		goto disable_tx;
	}

	usleep_range(2000, 4000);
	gpiod_set_value(eqos->reset, 0);

	eqos->rst = devm_reset_control_get(&pdev->dev, "eqos");
	if (IS_ERR(eqos->rst)) {
		err = PTR_ERR(eqos->rst);
		goto reset_phy;
	}

	err = reset_control_assert(eqos->rst);
	if (err < 0)
		goto reset_phy;

	usleep_range(2000, 4000);

	err = reset_control_deassert(eqos->rst);
	if (err < 0)
		goto reset_phy;

	usleep_range(2000, 4000);

	data->fix_mac_speed = tegra_eqos_fix_speed;
	data->init = tegra_eqos_init;
	data->bsp_priv = eqos;

	err = tegra_eqos_init(pdev, eqos);
	if (err < 0)
		goto reset;

out:
	return eqos;

reset:
	reset_control_assert(eqos->rst);
reset_phy:
	gpiod_set_value(eqos->reset, 1);
disable_tx:
	clk_disable_unprepare(eqos->clk_tx);
disable_rx:
	clk_disable_unprepare(eqos->clk_rx);
disable_slave:
	clk_disable_unprepare(eqos->clk_slave);
disable_master:
	clk_disable_unprepare(eqos->clk_master);
error:
	eqos = ERR_PTR(err);
	goto out;
}

static int tegra_eqos_remove(struct platform_device *pdev)
{
	struct tegra_eqos *eqos = get_stmmac_bsp_priv(&pdev->dev);

	reset_control_assert(eqos->rst);
	gpiod_set_value(eqos->reset, 1);
	clk_disable_unprepare(eqos->clk_tx);
	clk_disable_unprepare(eqos->clk_rx);
	clk_disable_unprepare(eqos->clk_slave);
	clk_disable_unprepare(eqos->clk_master);

	return 0;
}

static void nxp3220_qos_fix_speed(void *priv, unsigned int speed)
{
	struct nxp3220_eqos *eqos = priv;
	unsigned long rate = DWMAC_125MHZ;
	unsigned long rate_r;
	int err;

	switch (speed) {
	case SPEED_1000:
		rate = DWMAC_125MHZ;
		break;

	case SPEED_100:
		rate = DWMAC_25MHZ;
		break;

	case SPEED_10:
		rate = DWMAC_2_5MHZ;
		break;

	default:
		dev_err(eqos->dev, "invalid speed %u\n", speed);
		break;
	}

	if (IS_ERR_OR_NULL(eqos->clk_tx))
		return;

	err = clk_set_rate(eqos->clk_tx, rate);
	if (err < 0)
		dev_err(eqos->dev, "failed to set TX rate: %d\n", err);

	rate_r = clk_get_rate(eqos->clk_tx);
	if (rate_r != rate)
		dev_err(eqos->dev, "failed to set TX rate to %lu(set %lu)\n",
				rate, rate_r);

}

static irqreturn_t wol_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int __nxp3220_clk_enable(struct nxp3220_eqos *eqos)
{
	int err;

	err = clk_prepare_enable(eqos->clk_master);
	if (err < 0) {
		dev_err(eqos->dev, "master clock enable failed\n");
		goto out;
	}

	err = clk_prepare_enable(eqos->clk_slave);
	if (err < 0) {
		dev_err(eqos->dev, "slave clock enable failed\n");
		goto disable_master;
	}

	err = clk_prepare_enable(eqos->ptp_ref);
	if (err < 0) {
		dev_err(eqos->dev, "ptp_ref clock error\n");
		goto disable_slave;
	}

	if (!IS_ERR_OR_NULL(eqos->clk_tx)) {
		err = clk_prepare_enable(eqos->clk_tx);
		if (err < 0) {
			dev_err(eqos->dev, "tx clock enable failed");
			goto disable_ptp_ref;
		}
	}

	/* eqos reset */
	if (!IS_ERR_OR_NULL(eqos->rst)) {
		err = reset_control_assert(eqos->rst);
		if (err < 0)
			goto disable_tx;

		usleep_range(2000, 4000);

		err = reset_control_deassert(eqos->rst);
		if (err < 0)
			goto disable_tx;

		usleep_range(2000, 4000);
	}

out:
	return err;

disable_tx:
	clk_disable_unprepare(eqos->clk_tx);
disable_ptp_ref:
	clk_disable_unprepare(eqos->ptp_ref);
disable_slave:
	clk_disable_unprepare(eqos->clk_slave);
disable_master:
	clk_disable_unprepare(eqos->clk_master);

	goto out;
}

static int nxp3220_qos_init(struct platform_device *pdev, void *priv)
{
	struct device *dev = &pdev->dev;
	struct nxp3220_eqos *eqos = priv;

	if (__nxp3220_clk_enable(eqos) < 0) {
		dev_err(dev, "clk failed\n");
		return -EINVAL;
	}

	dev_dbg(dev, " eqos slave : %lu\n", clk_get_rate(eqos->clk_slave));
	dev_dbg(dev, " eqos master : %lu\n", clk_get_rate(eqos->clk_master));
	dev_dbg(dev, " eqos ptp_ref : %lu\n", clk_get_rate(eqos->ptp_ref));

	/* FIFO wake */
	writel(1, eqos->regs + 0x4000);

	return 0;
}

static void nxp3220_qos_exit(struct platform_device *pdev, void *priv)
{
	struct nxp3220_eqos *eqos = priv;

	if (!IS_ERR(eqos->rst))
		reset_control_assert(eqos->rst);
	if (!IS_ERR_OR_NULL(eqos->clk_tx))
		clk_disable_unprepare(eqos->clk_tx);
	clk_disable_unprepare(eqos->ptp_ref);
	clk_disable_unprepare(eqos->clk_slave);
	clk_disable_unprepare(eqos->clk_master);
}

static int get_csr_rate(struct nxp3220_eqos *eqos)
{
	u32 clk_rate = clk_get_rate(eqos->clk_slave);
	int clk_csr;

	if (clk_rate < CSR_F_35M)
		clk_csr = STMMAC_CSR_20_35M;
	else if ((clk_rate >= CSR_F_35M) && (clk_rate < CSR_F_60M))
		clk_csr = STMMAC_CSR_35_60M;
	else if ((clk_rate >= CSR_F_60M) && (clk_rate < CSR_F_100M))
		clk_csr = STMMAC_CSR_60_100M;
	else if ((clk_rate >= CSR_F_100M) && (clk_rate < CSR_F_150M))
		clk_csr = STMMAC_CSR_100_150M;
	else if ((clk_rate >= CSR_F_150M) && (clk_rate < CSR_F_250M))
		clk_csr = STMMAC_CSR_150_250M;
	else if ((clk_rate >= CSR_F_250M) && (clk_rate < CSR_F_300M))
		clk_csr = STMMAC_CSR_250_300M;
	else
		clk_csr = 0;

	return clk_csr;
}

static void *nxp3220_qos_probe(struct platform_device *pdev,
			      struct plat_stmmacenet_data *data,
			      struct stmmac_resources *res)
{
	struct nxp3220_eqos *eqos;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int interface;
	int err = 0;
	int phy_wolirq;

	eqos = devm_kzalloc(dev, sizeof(*eqos), GFP_KERNEL);
	if (!eqos) {
		dev_err(dev, "eqos memory alloc failed\n");
		err = -ENOMEM;
		goto error;
	}

	eqos->dev = dev;
	eqos->regs = res->addr;

	/* Master(AXI) Clock */
	eqos->clk_master = devm_clk_get(dev, "master_bus");
	if (IS_ERR(eqos->clk_master)) {
		dev_dbg(dev, "master clock failed\n");
		err = PTR_ERR(eqos->clk_master);
		goto error;
	}

	/* Slave(CSR) Clock */
	eqos->clk_slave = devm_clk_get(dev, "slave_bus");
	if (IS_ERR(eqos->clk_slave)) {
		dev_err(dev, "slave clock get failed\n");
		err = PTR_ERR(eqos->clk_slave);
		goto disable_master;
	}

	/* PTP_REF Clock */
	eqos->ptp_ref = devm_clk_get(dev, "ptp_ref");
	if (IS_ERR(eqos->ptp_ref)) {
		dev_err(dev, "ptp_ref clock get failed\n");
		err = PTR_ERR(eqos->ptp_ref);
		goto disable_slave;
	}

	/* tx clk for rgmii only */
	interface = of_get_phy_mode(np);
	if (interface == PHY_INTERFACE_MODE_RGMII) {
		eqos->clk_tx = devm_clk_get(dev, "tx");
		if (IS_ERR(eqos->clk_tx)) {
			dev_err(dev, "tx clock get failed\n");
			err = PTR_ERR(eqos->clk_tx);
			goto disable_ptp_ref;
		}
	}

	/* phy reset */
	eqos->phy_reset =
		devm_gpiod_get(dev, "phy-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(eqos->phy_reset)) {
		dev_err(dev, "cannot get phy reset\n");
		err = PTR_ERR(eqos->phy_reset);
		goto disable_tx;
	}

	usleep_range(10000, 30000);
	gpiod_set_value(eqos->phy_reset, 0);
	usleep_range(10000, 30000);

	eqos->phy_intr = devm_gpiod_get(dev, "phy-intr", GPIOD_IN);
	eqos->phy_pme = devm_gpiod_get(dev, "phy-pme", GPIOD_IN);
	if (!IS_ERR(eqos->phy_pme)) {
		phy_wolirq = gpiod_to_irq(eqos->phy_pme);
		err = devm_request_irq(dev, phy_wolirq, wol_isr,
				IRQF_TRIGGER_FALLING,
				dev_name(dev), dev);
		if (err) {
			dev_err(&pdev->dev, "IRQ request returned %d\n", err);
			goto error;
		}
		eqos->phy_wol_irq = phy_wolirq;
	}

	eqos->rst = devm_reset_control_get(dev, "eqos");
	if (IS_ERR(eqos->rst))
		dev_dbg(dev, "no eqos reset provided\n");

	eqos->wolopts = 0;
	if (!IS_ERR_OR_NULL(eqos->phy_pme))
		data->phy_wol = true;

	nxp3220_qos_init(pdev, eqos);
	data->clk_csr = get_csr_rate(eqos);

	eqos->wolopts = 0;

	data->fix_mac_speed = nxp3220_qos_fix_speed;
	data->init = nxp3220_qos_init;
	data->exit = nxp3220_qos_exit;
	data->set_wol = nxp3220_set_wol;
	data->get_wol = nxp3220_get_wol;
	data->bsp_priv = eqos;

out:
	return eqos;

disable_tx:
	if (interface == PHY_INTERFACE_MODE_RGMII)
		clk_disable_unprepare(eqos->clk_tx);
disable_ptp_ref:
	clk_disable_unprepare(eqos->ptp_ref);
disable_slave:
	clk_disable_unprepare(eqos->clk_slave);
disable_master:
	clk_disable_unprepare(eqos->clk_master);
error:
	eqos = ERR_PTR(err);
	goto out;
}

static int nxp3220_qos_remove(struct platform_device *pdev)
{
	struct nxp3220_eqos *eqos = get_stmmac_bsp_priv(&pdev->dev);

	if (!IS_ERR(eqos->rst))
		reset_control_assert(eqos->rst);
	gpiod_set_value(eqos->phy_reset, 1);
	clk_disable_unprepare(eqos->clk_tx);
	clk_disable_unprepare(eqos->clk_master);

	return 0;
}

struct dwc_eth_dwmac_data {
	void *(*probe)(struct platform_device *pdev,
		       struct plat_stmmacenet_data *data,
		       struct stmmac_resources *res);
	int (*remove)(struct platform_device *pdev);
};

static const struct dwc_eth_dwmac_data dwc_qos_data = {
	.probe = dwc_qos_probe,
	.remove = dwc_qos_remove,
};

static const struct dwc_eth_dwmac_data tegra_eqos_data = {
	.probe = tegra_eqos_probe,
	.remove = tegra_eqos_remove,
};

static const struct dwc_eth_dwmac_data nxp3220_qos_data = {
	.probe = nxp3220_qos_probe,
	.remove = nxp3220_qos_remove,
};

static int dwc_eth_dwmac_probe(struct platform_device *pdev)
{
	const struct dwc_eth_dwmac_data *data;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct resource *res;
	void *priv;
	int ret;

	data = of_device_get_match_data(&pdev->dev);

	memset(&stmmac_res, 0, sizeof(struct stmmac_resources));

	/**
	 * Since stmmac_platform supports name IRQ only, basic platform
	 * resource initialization is done in the glue logic.
	 */
	stmmac_res.irq = platform_get_irq(pdev, 0);
	if (stmmac_res.irq < 0) {
		if (stmmac_res.irq != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"IRQ configuration information not found\n");

		return stmmac_res.irq;
	}
	stmmac_res.wol_irq = stmmac_res.irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	stmmac_res.addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(stmmac_res.addr))
		return PTR_ERR(stmmac_res.addr);

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	priv = data->probe(pdev, plat_dat, &stmmac_res);
	if (IS_ERR(priv)) {
		ret = PTR_ERR(priv);
		dev_err(&pdev->dev, "failed to probe subdriver: %d\n", ret);
		goto remove_config;
	}

	ret = dwc_eth_dwmac_config_dt(pdev, plat_dat);
	if (ret)
		goto remove;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto remove;

	return ret;

remove:
	data->remove(pdev);
remove_config:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int dwc_eth_dwmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	const struct dwc_eth_dwmac_data *data;
	int err;

	data = of_device_get_match_data(&pdev->dev);

	err = stmmac_dvr_remove(&pdev->dev);
	if (err < 0)
		dev_err(&pdev->dev, "failed to remove platform: %d\n", err);

	err = data->remove(pdev);
	if (err < 0)
		dev_err(&pdev->dev, "failed to remove subdriver: %d\n", err);

	stmmac_remove_config_dt(pdev, priv->plat);

	return err;
}

static const struct of_device_id dwc_eth_dwmac_match[] = {
	{ .compatible = "snps,dwc-qos-ethernet-4.10", .data = &dwc_qos_data },
	{ .compatible = "nvidia,tegra186-eqos", .data = &tegra_eqos_data },
	{ .compatible = "nexell,nxp3220-dwmac", .data = &nxp3220_qos_data },
	{ }
};
MODULE_DEVICE_TABLE(of, dwc_eth_dwmac_match);

static struct platform_driver dwc_eth_dwmac_driver = {
	.probe  = dwc_eth_dwmac_probe,
	.remove = dwc_eth_dwmac_remove,
	.driver = {
		.name           = "dwc-eth-dwmac",
		.pm             = &stmmac_pltfr_pm_ops,
		.of_match_table = dwc_eth_dwmac_match,
	},
};
#ifdef CONFIG_DEFERRED_DWC_ETHERNET
static int __init dwmac_pltfm_init(void)
{
	return platform_driver_register(&dwc_eth_dwmac_driver);
}
deferred_module_init(dwmac_pltfm_init)
#else
module_platform_driver(dwc_eth_dwmac_driver);
#endif

MODULE_AUTHOR("Joao Pinto <jpinto@synopsys.com>");
MODULE_DESCRIPTION("Synopsys DWC Ethernet Quality-of-Service v4.10a driver");
MODULE_LICENSE("GPL v2");
