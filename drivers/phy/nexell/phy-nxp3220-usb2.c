// SPDX-License-Identifier: GPL-2.0+
/*
 * NXP3220 USB2 PHY driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

static int nx_otg_phy_power_on(struct nx_usb2_phy *p)
{
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	void __iomem *base = pdata->base;
	int vbus_tune = p->vbus_tune_val;

	dev_dbg(pdata->dev, "power on '%s'\n", p->label);

	/*
	 * Must be enabled 'adb400 blk usb cfg' and 'data adb'
	 * 'adb400 blk usb cfg' and 'data adb' is shared EHCI
	 */

	/*
	 * Set OTGTUNE[9:6] :
	 * This bus adjusts the voltage level for the VBUS Valid threshold
	 * 111: + 9%
	 * 110: + 6%
	 * 101: + 3%
	 * 100: Design default
	 * 011: - 3%
	 * 010: - 6%
	 * 001: - 9%
	 * 000: - 12%
	 */
	writel(readl(base + 0x88) & ~(7 << 6), base + 0x88);
	writel(readl(base + 0x88) | ((vbus_tune & 7) << 6), base + 0x88);

	/*
	 * PHY POR release
	 * SYSREG_USB_USB20PHY_OTG0_i_POR/SYSREG_USB_USB20PHY_OTG0_i_POR_ENB
	 */
	writel(readl(base + 0x80) & ~(1 << 4), base + 0x80);
	writel(readl(base + 0x80) | (1 << 3), base + 0x80);

	udelay(50);

	/*
	 * PHY reset release in OTG LINK
	 * SYSREG_USB_OTG_i_nUtmiResetSync
	 */
	writel(readl(base + 0x60) | (1 << 8), base + 0x60);

	udelay(1);

	/*
	 * BUS reset release in OTG LINK
	 * SYSREG_USB_OTG_i_nResetSync
	 */
	writel(readl(base + 0x60) | (1 << 7), base + 0x60);

	udelay(1);

	return 0;
}

static int nx_otg_phy_power_off(struct nx_usb2_phy *p)
{
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	void __iomem *base = pdata->base;

	dev_dbg(pdata->dev, "power off '%s'\n", p->label);

	/*
	 * PHY reset in OTG LINK
	 * SYSREG_USB_OTG_i_nUtmiResetSync
	 */
	writel(readl(base + 0x60) & ~(1 << 8), base + 0x60);

	/*
	 * BUS reset in OTG LINK
	 * SYSREG_USB_OTG_i_nResetSync
	 */
	writel(readl(base + 0x60) & ~(1 << 7), base + 0x60);

	/*
	 * PHY POR
	 * SYSREG_USB_USB20PHY_OTG0_i_POR/SYSREG_USB_USB20PHY_OTG0_i_POR_ENB
	 */
	writel(readl(base + 0x80) | (1 << 4), base + 0x80);

	/*
	 * Don't disable 'adb400 blk usb cfg' and 'data adb'
	 * 'adb400 blk usb cfg' and 'data adb' is shared EHCI
	 */

	return 0;
}

static int nx_ehci_phy_power_on(struct nx_usb2_phy *p)
{
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	void __iomem *base = pdata->base;
	int port = p->port;

	dev_dbg(pdata->dev, "power on '%s' port.%d\n", p->label, p->port);

	/*
	 * Must be enabled 'adb400 blk usb cfg' and 'data adb'
	 * 'adb400 blk usb cfg' and 'data adb' is shared EHCI
	 */

	/*
	 * Adjuet frame length (default 0x20), must be set to adjust 125ms
	 * HOST_ss_fladj_val_n_i
	 */
	writel((readl(base + 0x28) &
		~(0x7 << 25) & ~(0x7 << 22) & ~(0x7 << 19) &
		~(0x7 << 16) & ~(0x7 << 13) & ~(0x7 << 10) & ~(0x3f << 4)) |
		((0x0 << 25) | (0x0 << 22) | (0x0 << 19) | (0x0 << 16) |
		 (0x0 << 13) | (0x7 << 10) | (0x20 << 4)),
		base + 0x28);

	/*
	 * Set Burst length (16 burst)
	 * SYSREG_USB_HOST_ss_ena_incr16_i
	 * SYSREG_USB_HOST_ss_ena_incr8_i
	 * SYSREG_USB_HOST_ss_ena_incr4_i
	 * HOST_ss_ena_incrx_align_i
	 */
	writel(readl(base + 0x2c) | (0xf << 0), base + 0x2c);

	/*
	 * Set 16bit interface for 60Mhz phy clock
	 * HOST_ss_word_if_i / HOST_ss_word_if_enb_i
	 * USB20PHY_HOST0_i_WORDINTERFACE / USB20PHY_HOST0_i_WORDINTERFACE_ENB
	 * USB20PHY_HOST1_i_WORDINTERFACE / USB20PHY_HOST1_i_WORDINTERFACE_ENB
	 */
	writel((readl(base + 0x20) & ~(0x3 << 26)) | (0x1 << 26), base + 0x20);
	writel((readl(base + 0x44) & ~(0x3 << 5)) | (0x1 << 5), base + 0x44);
	writel((readl(base + 0x54) & ~(0x3 << 5)) | (0x1 << 5), base + 0x54);

	/*
	 * HOST_i_nOhciClkCktRst
	 */
	writel(readl(base + 0x20) | (0x1 << 12), base + 0x20);

	/*
	 * POR(Power On Reset) of PHY fpr PORT0
	 * SYSREG_USB_USB20PHY_HOST0_i_POR/SYSREG_USB_USB20PHY_HOST0_i_POR_ENB
	 */
	writel((readl(base + 0x40) & ~(0x3 << 3)) | (0x1 << 3), base + 0x40);

	/* Wait clock of PHY: 40us */
	udelay(50);

	/*
	 *  Release utmi hub reset
	 * SYSREG_USB_HOST_i_nHostPhyResetSync
	 */
	writel(readl(base + 0x20) | (0x1 << 10), base + 0x20);

	/*
	 * Release utmi PORT0 reset
	 * SYSREG_USB_HOST_i_nHostUtmiResetSync0
	 */
	writel(readl(base + 0x20) | (0x1 << 11), base + 0x20);

	/*
	 * Release ahb reset of EHCI/OHCI
	 * SYSREG_USB_HOST_i_nResetSync/SYSREG_USB_HOST_i_nResetSync_ohci
	 * SYSREG_USB_HOST_i_nAuxWellResetSync
	 */
	writel(readl(base + 0x20) | (0x7 << 7), base + 0x20);

	if (port == 0)
		return 0;

	/*
	 * POR(Power On Reset) of PHY for PORT1
	 * SYSREG_USB_USB20PHY_HOST1_i_POR/SYSREG_USB_USB20PHY_HOST1_i_POR_ENB
	 */
	writel((readl(base + 0x50) & ~(0x3 << 3)) | (0x1 << 3), base + 0x50);

	/* Wait clock of PHY: 40us */
	udelay(50);

	/*
	 * Release utmi reset for PORT1
	 * SYSREG_USB_HOST_i_nHostUtmiResetSync1
	 */
	writel(readl(base + 0x20) | (1 << 31), base + 0x20);

	return 0;
}

static int nx_ehci_phy_power_off(struct nx_usb2_phy *p)
{
	struct nx_usb2_phy_pdata *pdata = p->pdata;
	void __iomem *base = pdata->base;
	int port = p->port;

	dev_dbg(pdata->dev, "power off '%s' port.%d\n", p->label, p->port);

	if (port == 1) {
		/*
		 * Reset utmi reset PORT1
		 * SYSREG_USB_HOST_i_nHostUtmiResetSync1
		 */
		writel(readl(base + 0x20) & ~(1 << 31), base + 0x20);

		/*
		 * Reset POR(Power On Reset) of PHY for PORT1
		 * SYSREG_USB_USB20PHY_HOST1_i_POR/
		 * SYSREG_USB_USB20PHY_HOST1_i_POR_ENB
		 */
		writel(readl(base + 0x50) | (0x3 << 3), base + 0x50);
	}

	/*
	 * Reset utmi reset PORT0
	 * SYSREG_USB_HOST_i_nHostUtmiResetSync0
	 */
	writel(readl(base + 0x20) & ~(1 << 11), base + 0x20);

	/*
	 * Reset ahb reset of EHCI
	 */
	writel(readl(base + 0x20) & ~(0x7 << 7), base + 0x20);

	/*
	 * Reset utmi reset
	 * SYSREG_USB_HOST_i_nHostPhyResetSync
	 */
	writel(readl(base + 0x20) & ~(1 << 10), base + 0x20);

	/*
	 * Reset PHY POR
	 * SYSREG_USB_USB20PHY_HOST0_i_POR/SYSREG_USB_USB20PHY_HOST0_i_POR_ENB
	 */
	writel(readl(base + 0x40) | (0x3 << 3), base + 0x40);

	return 0;
}

static struct nx_usb2_phy nxp3220_usb2_phys[] = {
	{
		.label = "dwc2otg",
		.vbus_gpio = "otg,vbus-gpio",
		.vbus_tune = "otg,vbus-tune",
		.bus_width = 16,
		.vbus_tune_val = 4,
		.power_on = nx_otg_phy_power_on,
		.power_off = nx_otg_phy_power_off,
	},
	{
		.label = "ehci0",
		.vbus_gpio = "ehci,vbus-gpio",
		.bus_width = 16,
		.port = 0,
		.power_on = nx_ehci_phy_power_on,
		.power_off = nx_ehci_phy_power_off,
	},
	{
		.label = "ehci1",
		.vbus_gpio = "ehci,vbus-gpio",
		.bus_width = 16,
		.port = 1,
		.power_on = nx_ehci_phy_power_on,
		.power_off = nx_ehci_phy_power_off,
	},
};

static const struct nx_usb2_phy_config nxp3220_usb2_phy_cfg = {
	.phys = nxp3220_usb2_phys,
	.num_phys = ARRAY_SIZE(nxp3220_usb2_phys),
};
