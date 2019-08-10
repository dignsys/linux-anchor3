// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>

#include "nxp3220_nand.h"

#define	DRV_NAME		"nxp3220_nfc"

#define	CHECK_RnB_INTERRUPT

static int nx_wait_for_irq(struct nxp3220_nfc *nfc, int nr_int);

#ifdef DEBUG
static void mem_dump(uint8_t *mem, int sectsize)
{
	int i;
	uint8_t buf[128];
	uint8_t *p = buf;

	memset(buf, 0x00, sizeof(buf));
	for (i = 0; i < sectsize; ) {
		p += sprintf(p, "%02x ", mem[i]);
		i++;
		if (!(i & 0xf) || i == sectsize)
			printk("%s\n", buf);
		if (!(i & 0xf))
			p = buf;
	}
}
#else
static inline void mem_dump(uint8_t *mem, int sectsize) { }
#endif

static void nfc_regs_dump(struct nxp3220_nfc *nfc)
{
	void __iomem *regs = nfc->regs;

	dev_dbg(nfc->dev,
		" ctrl 0x%x, bch mode 0x%x, dma ctrl 0x%x, dma addr 0x%x\n",
			readl(regs + NFC_CTRL),
			readl(regs + NFC_BCH_MODE),
			readl(regs + NFC_DMA_CTRL),
			readl(regs + NFC_DMA_ADDR));
	dev_dbg(nfc->dev,
		" dma size 0x%x, dma subp 0x%x, ddr ctrl 0x%x, cmd time 0x%x\n",
			readl(regs + NFC_DMA_SIZE),
			readl(regs + NFC_DMA_SUBP),
			readl(regs + NFC_DDR_CTRL),
			readl(regs + NFC_CMD_TIME));
	dev_dbg(nfc->dev,
		" data time 0x%x, ddr time 0x%x, rand 0x%x, status 0x%x\n",
			readl(regs + NFC_DATA_TIME),
			readl(regs + NFC_DDR_TIME),
			readl(regs + NFC_RAND),
			readl(regs + NFC_STATUS));
	dev_dbg(nfc->dev, " subpage 0x%x, error info 0x%x\n",
			readl(regs + NFC_SUBPAGE),
			readl(regs + NFC_ERR_INFO));
}

static inline struct nxp3220_nfc *mtd_to_nfc(struct mtd_info *mtd)
{
	return container_of(mtd_to_nand(mtd), struct nxp3220_nfc, chip);
}

/*
 * nand interface
 */

static void nx_nandc_set_irq_enable(void __iomem *regs, int nr_int, int enable)
{
	const uint32_t IRQRDYEN_POS    = 0;
	const uint32_t IRQRDYEN_MASK   = (1UL << IRQRDYEN_POS);

	const uint32_t IRQDMAEN_POS    = 4;
	const uint32_t IRQDMAEN_MASK   = (1UL << IRQDMAEN_POS);

	uint32_t val;

	val = readl(regs + NFC_STATUS);
	if (nr_int == NX_NANDC_INT_RDY) {
		val &= ~IRQRDYEN_MASK;
		val |= (uint32_t)(enable << IRQRDYEN_POS);
	} else {
		val &= ~IRQDMAEN_MASK;
		val |= (uint32_t)(enable << IRQDMAEN_POS);
	}

	writel(val, regs + NFC_STATUS);
	dmb();
}

static void nx_nandc_set_irq_mask(void __iomem *regs, int nr_int, int unmask)
{
	const uint32_t IRQRDYMS_POS    = 1;
	const uint32_t IRQRDYMS_MASK   = (1UL << IRQRDYMS_POS);

	const uint32_t IRQDMAMS_POS    = 5;
	const uint32_t IRQDMAMS_MASK   = (1UL << IRQDMAMS_POS);

	uint32_t val;
	int unmasked = !unmask; /* 1: unmasked, 0: masked */

	val = readl(regs + NFC_STATUS);
	if (nr_int == 0) {
		val &= ~IRQRDYMS_MASK;
		val |= (uint32_t)(unmasked << IRQRDYMS_POS);
	} else {
		val &= ~IRQDMAMS_MASK;
		val |= (uint32_t)(unmasked << IRQDMAMS_POS);
	}

	writel(val, regs + NFC_STATUS);
	dmb();
}

static int nx_nandc_get_irq_pending(void __iomem *regs, int nr_int)
{
	const u32 IRQRDY_POS    = 2;
	const u32 IRQDMA_POS    = 6;
	const u32 IRQRDY_MASK   = (1UL << IRQRDY_POS);
	const u32 IRQDMA_MASK   = (1UL << IRQDMA_POS);
	u32 IRQPEND_POS, IRQPEND_MASK;

	if (nr_int == NX_NANDC_INT_RDY) {
		IRQPEND_POS  = IRQRDY_POS;
		IRQPEND_MASK = IRQRDY_MASK;
	} else {
		IRQPEND_POS  = IRQDMA_POS;
		IRQPEND_MASK = IRQDMA_MASK;
	}

	return  (int)((readl(regs + NFC_STATUS) & IRQPEND_MASK) >> IRQPEND_POS);
}

static void nx_nandc_clear_irq_pending(void __iomem *regs, int nr_int)
{
	const uint32_t IRQRDY_POS    = 2;
	const uint32_t IRQDMA_POS    = 6;
	const uint32_t IRQRDY_MASK   = (1UL << IRQRDY_POS);
	const uint32_t IRQDMA_MASK   = (1UL << IRQDMA_POS);

	uint32_t IRQPEND_POS, IRQPEND_MASK;

	uint32_t val;

	if (nr_int == NX_NANDC_INT_RDY) {
		IRQPEND_POS  = IRQRDY_POS;
		IRQPEND_MASK = IRQRDY_MASK;
	} else {
		IRQPEND_POS  = IRQDMA_POS;
		IRQPEND_MASK = IRQDMA_MASK;
	}

	val = readl(regs + NFC_STATUS);
	val &= ~IRQPEND_MASK;
	val |= (0x1 << IRQPEND_POS);
	writel(val, regs + NFC_STATUS);
	dmb();
}

static void nx_nandc_set_cmd_timing(void __iomem *regs,
		int rhw, int setup, int width, int hold)
{
	uint32_t val;

	val = (rhw << 24 | setup << 16 | width << 8 | hold);
	writel(val, regs + NFC_CMD_TIME);
	dmb();
}

static void nx_nandc_set_data_timing(void __iomem *regs,
		int whr, int setup, int width, int hold)
{
	uint32_t val;

	val = (whr << 24 | setup << 16 | width << 8 | hold);
	writel(val, regs + NFC_DATA_TIME);
	dmb();
}

static void nx_nandc_set_ddr_timing(void __iomem *regs,
		int tcad, int tlast, int rd_dly, int dq_dly, int clk_period)
{
	uint32_t val;

	val = (tcad << 28 | tlast << 24 | rd_dly << 16 | dq_dly << 8 |
			clk_period);
	writel(val, regs + NFC_DDR_TIME);
	dmb();
}

static void nx_nandc_set_cs_enable(void __iomem *regs, uint32_t chipsel,
				   int enable)
{
	const uint32_t BIT_SIZE  = 3;
	const uint32_t BIT_POS   = 4;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_CTRL);
	val &= ~(BIT_MASK | 0x1);
	val |= (chipsel << BIT_POS) | enable;
	writel(val, regs + NFC_CTRL);
	dmb();
}

static uint32_t nx_nandc_get_ready(void __iomem *regs)
{
#ifdef CHECK_RnB_INTERRUPT
	return nx_nandc_get_irq_pending(regs, NX_NANDC_INT_RDY);
#else
	const uint32_t BIT_SIZE  = 1;
	const uint32_t BIT_POS   = 28;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	return readl(regs + NFC_STATUS) & BIT_MASK;
#endif
}

static void __maybe_unused nx_nandc_set_randseed(void __iomem *regs, int seed)
{
	const u32 BIT_SIZE  = 16;
	const u32 BIT_POS   = 0;
	const u32 BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	writel(seed & BIT_MASK, regs + NFC_RAND);
	dmb();
}

static void nx_nandc_set_bchmode(void __iomem *regs, uint32_t bchmode)
{
	const uint32_t BIT_SIZE  = 4;
	const uint32_t BIT_POS   = 0;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_BCH_MODE);
	val &= ~BIT_MASK;
	val |= (bchmode << BIT_POS);
	writel(val, regs + NFC_BCH_MODE);
	dmb();
}

static void nx_nandc_set_encmode(void __iomem *regs, int enable)
{
	const uint32_t BIT_SIZE  = 1;
	const uint32_t BIT_POS   = 4;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_BCH_MODE);
	val &= ~BIT_MASK;
	val |= (uint32_t)enable << BIT_POS;
	writel(val, regs + NFC_BCH_MODE);
	dmb();
}

static void nx_nandc_set_ddrmode(void __iomem *regs, int enable)
{
	const uint32_t BIT_POS  = 0;
	const uint32_t BIT_MASK = (1UL << BIT_POS);
	uint32_t val;

	val = readl(regs + NFC_DDR_CTRL);
	val &= ~BIT_MASK;
	val |= (enable << BIT_POS) & BIT_MASK;
	writel(val, regs + NFC_DDR_CTRL);
	dmb();
}

static void nx_nandc_set_ddrclock_enable(void __iomem *regs, int enable)
{
	const uint32_t BIT_POS  = 1;
	const uint32_t BIT_MASK = (1UL << BIT_POS);
	uint32_t val;

	val = readl(regs + NFC_DDR_CTRL);
	val &= ~BIT_MASK;
	val |= (enable << BIT_POS) & BIT_MASK;
	writel(val, regs + NFC_DDR_CTRL);
	dmb();
}

static void nx_nandc_set_dmamode(void __iomem *regs, uint32_t mode)
{
	const uint32_t BIT_POS  = 0;
	const uint32_t BIT_MASK = (1UL << BIT_POS);
	uint32_t val;

	val = readl(regs + NFC_DMA_CTRL);
	val &= ~BIT_MASK;
	val |= (mode << BIT_POS) & BIT_MASK;
	writel(val, regs + NFC_DMA_CTRL);
	dmb();
}

static void nx_nandc_set_eccsize(void __iomem *regs, int eccsize)
{
	const uint32_t BIT_SIZE  = 8;
	const uint32_t BIT_POS   = 16;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_DMA_SIZE);
	val &= ~BIT_MASK;
	val |= (uint32_t)eccsize << BIT_POS;
	writel(val, regs + NFC_DMA_SIZE);
	dmb();
}

static void nx_nandc_set_dmasize(void __iomem *regs, int dmasize)
{
	const uint32_t BIT_SIZE  = 16;
	const uint32_t BIT_POS   = 0;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_DMA_SIZE);
	val &= ~BIT_MASK;
	val |= dmasize << BIT_POS;
	writel(val, regs + NFC_DMA_SIZE);
	dmb();
}

static void nx_nandc_sel_subpage(void __iomem *regs, int sel)
{
	const uint32_t BIT_POS   = 0;
	uint32_t val;

	val = (uint32_t)sel << BIT_POS;
	writel(val, regs + NFC_SUBPAGE);
	dmb();
}

static int nx_nandc_get_errinfo(void __iomem *regs)
{
	const uint32_t BIT_SIZE  = 14;
	const uint32_t BIT_POS   = 0;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	return (readl(regs + NFC_ERR_INFO) & BIT_MASK) >> BIT_POS;
}

static void nx_nandc_set_subpage(void __iomem *regs, int subpage)
{
	const uint32_t BIT_SIZE  = 4;
	const uint32_t BIT_POS   = 16;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_DMA_SUBP);
	val &= ~BIT_MASK;
	val |= (uint32_t)subpage << BIT_POS;
	writel(val, regs + NFC_DMA_SUBP);
	dmb();
}

static void nx_nandc_set_subpage_size(void __iomem *regs, int subsize)
{
	const uint32_t BIT_SIZE  = 11;
	const uint32_t BIT_POS   = 0;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;

	uint32_t val;

	val = readl(regs + NFC_DMA_SUBP);
	val &= ~BIT_MASK;
	val |= subsize << BIT_POS;
	writel(val, regs + NFC_DMA_SUBP);
	dmb();
}

static void nx_nandc_set_dma_base(void __iomem *regs, uint32_t base)
{
	writel(base, regs + NFC_DMA_ADDR);
	dmb();
}

static void nx_nandc_set_sramsleep(void __iomem *regs, int enable)
{
	const uint32_t BIT_SIZE  = 1;
	const uint32_t BIT_POS   = 16;
	const uint32_t BIT_MASK  = ((1 << BIT_SIZE) - 1) << BIT_POS;
	uint32_t val;

	val = readl(regs + NFC_CTRL);
	val &= ~BIT_MASK;
	val |= (!enable << BIT_POS);
	writel(val, regs + NFC_CTRL);
	dmb();
}

static int nx_nandc_run_dma(struct nxp3220_nfc *nfc)
{
	void __iomem *regs = nfc->regs;
	int ret = 0;
	unsigned long flags;

	reinit_completion(&nfc->dma_done);

	spin_lock_irqsave(&nfc->irq_lock, flags);

	/* clear DMA interrupt pending */
	nx_nandc_clear_irq_pending(regs, NX_NANDC_INT_DMA);

	/* DMA run */
	nx_nandc_set_dmamode(regs, NX_NANDC_DMA_MODE);

	spin_unlock_irqrestore(&nfc->irq_lock, flags);

	ret = nx_wait_for_irq(nfc, NX_NANDC_INT_DMA);

	nx_nandc_set_dmamode(regs, NX_NANDC_CPU_MODE);

	return ret;
}

/* ***************************************************************************
 * interface functions
 *************************************************************************** */
static void nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;

	dev_dbg(nfc->dev, "%s, chipnr=%d\n", __func__, chipnr);

	if (chipnr > 4) {
		dev_err(nfc->dev, "does not support nand chip %d\n", chipnr);
		return;
	}

	if (chipnr == -1)
		nx_nandc_set_cs_enable(regs, 7, 1);
	else
		nx_nandc_set_cs_enable(regs, chipnr, 1);
}

static u16 nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	u16 tmp;

	ioread8_rep(chip->IO_ADDR_R, (u_char *)&tmp, sizeof(tmp));

	return tmp;
}

static void nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned long flags;

		spin_lock_irqsave(&nfc->irq_lock, flags);

		nx_nandc_clear_irq_pending(regs, NX_NANDC_INT_RDY);

		spin_unlock_irqrestore(&nfc->irq_lock, flags);
	}

	if (ctrl & NAND_CLE) {
		dev_dbg(nfc->dev, " command: %02x\n", (unsigned char)cmd);
		writeb(cmd, regs + NFC_CMD);
	} else if (ctrl & NAND_ALE) {
		dev_dbg(nfc->dev, " address: %02x\n", (unsigned char)cmd);
		writeb(cmd, regs + NFC_ADDR);
	}
}

static int nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;
	int ret = 0;

	ret = nx_nandc_get_ready(regs);
	dev_dbg(nfc->dev, " nfc RnB %s\n", ret ? "READY" : "BUSY");

	return ret;
}

/*
 * Enable NAND write protect
 */
static void nand_wp_enable(struct nxp3220_nfc *nfc)
{
	if (gpio_is_valid(nfc->wp_gpio))
		gpio_set_value(nfc->wp_gpio, 0);
}

/*
 * Disable NAND write protect
 */
static void nand_wp_disable(struct nxp3220_nfc *nfc)
{
	if (gpio_is_valid(nfc->wp_gpio))
		gpio_set_value(nfc->wp_gpio, 1);
}

/* convert to nano-seconds to nfc clock cycles */
#define ns2cycle(ns, clk)	(int)(((ns) * (clk / 1000000)) / 1000)
#define ns2cycle_r(ns, clk)	(int)(((ns) * (clk / 1000000) + 999) / 1000)

/* sdr timing saving and setting */
static void nxp3220_calc_sdr_timings(struct nxp3220_nfc *nfc,
				     const struct nand_sdr_timings *t)
{
	int tCLH = DIV_ROUND_UP(t->tCLH_min, 1000);
	int tWP = DIV_ROUND_UP(t->tWP_min, 1000);
	int tWH = DIV_ROUND_UP(t->tWH_min, 1000);
	int tCLS = DIV_ROUND_UP(t->tCLS_min, 1000);
	int tWC = DIV_ROUND_UP(t->tWC_min, 1000);
	int tRHW = DIV_ROUND_UP(t->tRHW_min, 1000);
	int tWHR = DIV_ROUND_UP(t->tWHR_min, 1000);

	int tDH = DIV_ROUND_UP(t->tDH_min, 1000);
	int tRC = DIV_ROUND_UP(t->tRC_min, 1000);
	int tRP = DIV_ROUND_UP(t->tRP_min, 1000);
	int tREH = DIV_ROUND_UP(t->tREH_min, 1000);
	int tREA = DIV_ROUND_UP(t->tREA_max, 1000);
	int clk_period;

	unsigned long clk_hz;
	int cmd_setup, cmd_width, cmd_hold, rhw;
	int write_setup, write_width, write_hold, whr;
	int read_setup, read_width, read_hold;

	clk_hz = clk_get_rate(nfc->core_clk);
	rhw = ns2cycle_r(tRHW, clk_hz);
	whr = ns2cycle_r(tWHR, clk_hz);

	clk_period = (u32)(1000000000UL / clk_hz);

	/* command write */
	cmd_setup = (tCLS-tWP) > tCLH ?
		DIV_ROUND_UP((tCLS-tWP),clk_period) :
		DIV_ROUND_UP(tCLH,clk_period);
	cmd_width = DIV_ROUND_UP(tWP,clk_period);
	cmd_hold = tCLH > tDH ? 0 : DIV_ROUND_UP(tDH,clk_period);

	nfc->time.cmd_setup = (cmd_setup > 0) ? cmd_setup - 1 : 0;
	nfc->time.cmd_width = (cmd_width > 0) ? cmd_width - 1 : 0;
	nfc->time.cmd_hold = (cmd_hold > 0) ? cmd_hold - 1 : 0;

	/* data write */
	write_setup = 0;
	write_width = DIV_ROUND_UP(tWP,clk_period);
	write_hold = (tWC-tWP) > tWH ?
		(((tWC-tWP) > tDH) ? DIV_ROUND_UP((tWC-tWP),clk_period) :
			DIV_ROUND_UP((tWC-tWP),clk_period)) :
		((tWH > tDH) ? DIV_ROUND_UP(tWH,clk_period) :
			DIV_ROUND_UP(tDH,clk_period));

	nfc->time.wr_setup = write_setup;
	nfc->time.wr_width = (write_width > 0) ? write_width - 1 : 0;
	nfc->time.wr_hold = (write_hold > 0) ? write_hold - 1 : 0;

	/* data read */
	read_setup = 0;
	read_width = (tRP > (tREA+clk_period)) ?
		DIV_ROUND_UP(tRP,clk_period) :
		DIV_ROUND_UP((tREA+clk_period),clk_period);
	read_hold = (tRC-tRP) > tREH ?
		DIV_ROUND_UP((tRC-tRP),clk_period) :
		DIV_ROUND_UP(tREH,clk_period);

	nfc->time.rd_setup = read_setup;
	nfc->time.rd_width = (read_width > 0) ? read_width - 1 : 0;
	nfc->time.rd_hold = (read_hold > 0) ? read_hold - 1 : 0;

	nfc->time.rhw = rhw > 0 ? rhw : 0;
	nfc->time.whr = whr > 0 ? whr : 0;

	dev_dbg(nfc->dev, " clk_hz %ld rhw:%u whr:%u\n",
		 clk_hz, nfc->time.rhw, nfc->time.whr);
	dev_dbg(nfc->dev, " cmd setup:%u width:%u hold:%u\n",
		 nfc->time.cmd_setup, nfc->time.cmd_width, nfc->time.cmd_hold);
	dev_dbg(nfc->dev, " read setup:%u width:%u hold:%u\n",
		nfc->time.rd_setup, nfc->time.rd_width, nfc->time.rd_hold);
	dev_dbg(nfc->dev, " write setup:%u width:%u hold:%u\n",
		 nfc->time.wr_setup, nfc->time.wr_width, nfc->time.wr_hold);
}

/* sdr timing settings */
static void nxp3220_set_sdr_timings(struct nxp3220_nfc *nfc,
				    int cmd, int read, int write)
{
	void __iomem *regs = nfc->regs;

	if (cmd) {
		u32 rhw = nfc->time.rhw;
		u32 setup = nfc->time.cmd_setup;
		u32 width = nfc->time.cmd_width;
		u32 hold = nfc->time.cmd_hold;

		nx_nandc_set_cmd_timing(regs, rhw, setup, width, hold);
	}

	if (read || write) {
		u32 whr = nfc->time.whr;
		u32 setup = (read) ? nfc->time.rd_setup : nfc->time.wr_setup;
		u32 width = (read) ? nfc->time.rd_width : nfc->time.wr_width;
		u32 hold = (read) ? nfc->time.rd_hold : nfc->time.wr_hold;

		nx_nandc_set_data_timing(regs, whr, setup, width, hold);
	}
}

static int nand_hw_init_timings(struct nand_chip *chip)
{
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	const struct nand_sdr_timings *timings;
	int mode;

	mode = onfi_get_async_timing_mode(chip);
	if (mode == ONFI_TIMING_MODE_UNKNOWN)
		mode = chip->onfi_timing_mode_default;
	else
		mode = fls(mode) - 1;
	if (mode < 0)
		mode = 0;
	dev_dbg(nfc->dev, "nand onfi timing mode: %x\n", mode);

	timings = onfi_async_timing_mode_to_sdr_timings(mode);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	nxp3220_calc_sdr_timings(nfc, timings);
	nxp3220_set_sdr_timings(nfc, 1, 1, 0);

	return 0;
}

static int nxp3220_setup_data_interface(struct mtd_info *mtd,
		int chipnr, const struct nand_data_interface *conf)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	const struct nand_sdr_timings *timings;

	timings = nand_get_sdr_timings(conf);
	if (IS_ERR(timings))
		return -ENOTSUPP;

	if (chipnr == NAND_DATA_IFACE_CHECK_ONLY)
		return 0;

	/* timing mode force */
	if (nfc->timing_force) {
		timings = onfi_async_timing_mode_to_sdr_timings(
				nfc->timing_mode);
		if (IS_ERR(timings))
			return PTR_ERR(timings);
	}

	nxp3220_calc_sdr_timings(nfc, timings);
	nxp3220_set_sdr_timings(nfc, 1, 1, 0);

	return 0;
}

static void nand_dev_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;

	nx_nandc_clear_irq_pending(regs, NX_NANDC_INT_RDY);
	nx_nandc_clear_irq_pending(regs, NX_NANDC_INT_DMA);

	/* sram sleep awake */
	nx_nandc_set_sramsleep(regs, 0);
	nx_nandc_set_cs_enable(regs, 0x7, 1);
	/* set ddr mode off */
	nx_nandc_set_ddrmode(regs, 0);
	/* ddr clock disable */
	nx_nandc_set_ddrclock_enable(regs, 0);
	/* ready interrupt enable */
	nx_nandc_set_irq_enable(regs, NX_NANDC_INT_RDY, 1);
	/* dma interrupt enable */
	nx_nandc_set_irq_enable(regs, NX_NANDC_INT_DMA, 1);
	/* ready interrupt mask enable */
	nx_nandc_set_irq_mask(regs, NX_NANDC_INT_RDY, 1);
	/* dma interrupt mask disable */
	nx_nandc_set_irq_mask(regs, NX_NANDC_INT_DMA, 0);

	nx_nandc_set_cmd_timing(regs, 45, 6, 20, 8);
	nx_nandc_set_data_timing(regs, 45, 5, 20, 8);
	nx_nandc_set_ddr_timing(regs, 3, 4, 0, 0, 3);

	nx_nandc_set_dmamode(regs, NX_NANDC_CPU_MODE);
}

/**
 * nand_hw_ecc_read_oob - OOB data read function for HW ECC
 *			    with syndromes
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to read
 */
static int nand_hw_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	int length = mtd->oobsize;
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsize = chip->ecc.size;
	uint8_t *bufpoi = chip->oob_poi;
	int i, toread, sndrnd = 0, pos;

	chip->cmdfunc(mtd, NAND_CMD_READ0, chip->ecc.size, page);
	for (i = 0; i < chip->ecc.steps; i++) {
		if (sndrnd) {
			pos = eccsize + i * (eccsize + chunk);
			if (mtd->writesize > 512)
				chip->cmdfunc(mtd, NAND_CMD_RNDOUT, pos, -1);
			else
				chip->cmdfunc(mtd, NAND_CMD_READ0, pos, page);
		} else {
			sndrnd = 1;
		}
		toread = min_t(int, length, chunk);
		chip->read_buf(mtd, bufpoi, toread);
		bufpoi += toread;
		length -= toread;
	}
	if (length > 0)
		chip->read_buf(mtd, bufpoi, length);

	return 0;
}

/**
 * nand_hw_ecc_write_oob - OOB data write function for HW ECC with syndrome
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to write
 */
static int nand_hw_ecc_write_oob(struct mtd_info *mtd,
				 struct nand_chip *chip, int page)
{
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsize = chip->ecc.size, length = mtd->oobsize;
	int i, len, pos, status = 0, sndcmd = 0, steps = chip->ecc.steps;
	const uint8_t *bufpoi = chip->oob_poi;

	/*
	 * data-ecc-data-ecc ... ecc-oob
	 * or
	 * data-pad-ecc-pad-data-pad .... ecc-pad-oob
	 */
	pos = eccsize;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);
	for (i = 0; i < steps; i++) {
		if (sndcmd) {
			if (mtd->writesize <= 512) {
				uint32_t fill = 0xFFFFFFFF;

				len = eccsize;
				while (len > 0) {
					int num = min_t(int, len, 4);
					chip->write_buf(mtd, (uint8_t *)&fill,
							num);
					len -= num;
				}
			} else {
				pos = eccsize + i * (eccsize + chunk);
				chip->cmdfunc(mtd, NAND_CMD_RNDIN, pos, -1);
			}
		} else {
			sndcmd = 1;
		}
		len = min_t(int, length, chunk);
		chip->write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
	}

	if (length > 0)
		chip->write_buf(mtd, bufpoi, length);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static int nand_hw_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	int ret = 0;

	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;

	int sectsize = nfc->sectsize;

	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize = chip->ecc.size;

	uint8_t *p = nfc->dmabuf;
	int spare, subp;
	int uncorr = 0;

	/* ecc setting */
	nx_nandc_set_bchmode(regs, nfc->bchmode);
	nx_nandc_set_encmode(regs, 0);

	/* setup DMA */
	nx_nandc_set_dma_base(regs, (uint32_t)(nfc->dmabuf));
	nx_nandc_set_eccsize(regs, eccbytes - 1);
	nx_nandc_set_dmasize(regs, sectsize * eccsteps - 1);
	nx_nandc_set_subpage(regs, eccsteps - 1);
	nx_nandc_set_subpage_size(regs, sectsize - 1);

	ret = nx_nandc_run_dma(nfc);
	if (ret < 0) {
		ret = -EIO;
		goto out;
	}

	spare = (eccsize * 2 - (eccsize + eccbytes)) * 8; /* data + meta */
	/* correction */
	for (subp = 0; subp < eccsteps; subp++, p += sectsize) {
		int errcnt;

		nx_nandc_sel_subpage(regs, subp);
		errcnt = nx_nandc_get_errinfo(regs);

		if (errcnt == 0x3f) {
			dev_dbg(nfc->dev, "page(%d) uncorrectable\n", page);
			uncorr = 1;
			break;
		} else if (errcnt) {
			int i;

			dev_dbg(nfc->dev, "errcnt:%d\n", errcnt);
			for (i = 0; i < errcnt; i++) {
				void __iomem *r = regs + NFC_SRAM +
					(i * sizeof(uint32_t));
				int elp = readl(r) & 0x3fff;

				dev_dbg(nfc->dev, "elp: %d -> %d\n",
						elp, elp - spare);
				elp -= spare;

				/* correct data range only */
				if (eccsize * 8 <= elp) {
					dev_dbg(nfc->dev,
						"step %d size %d elp %d\n",
						eccsteps, eccsize, elp);
					continue;
				}

				mem_dump(p, sectsize);
				/* correct bit */
				dev_dbg(nfc->dev, "p[elp(%d) >> 3] = %02x\n",
						elp, p[elp >> 3]);
				p[elp >> 3] ^= 1 << (7 - (elp & 0x7));
				dev_dbg(nfc->dev, "to = %02x\n", p[elp >> 3]);
			}
		}

		dev_dbg(nfc->dev, "read step=%d/%d page=%d, page size=%d\n",
				subp + 1, eccsteps, page, mtd->writesize);
	}

	p = nfc->dmabuf;
	if (uncorr) {
		/* data-ecc-data-ecc-...-ecc-oob */
		int stat = 0;
		unsigned int max_bitflips = 0;
		int prepad = chip->ecc.prepad;

		for (subp = 0; subp < eccsteps; subp++, p += sectsize) {
			stat = nand_check_erased_ecc_chunk(p, eccsize,
					p + prepad + eccsize, eccbytes,
					NULL, 0,
					chip->ecc.strength);
			if (stat < 0) {
				dev_err(nfc->dev, "ecc uncorrectable(%d/%d)\n",
						(subp + 1), eccsteps);
				mtd->ecc_stats.failed++;
				break;
			} else {
				mtd->ecc_stats.corrected += stat;
				max_bitflips =
					max_t(unsigned int, max_bitflips, stat);
				ret = max_bitflips;

				/* bitflip cleaning */
				memset(buf, 0xff, mtd->writesize);
			}
		}
	} else {
		for (subp = 0; subp < eccsteps; subp++) {
			memcpy(buf, p, eccsize);
			p += sectsize;
			buf += eccsize;
		}
	}

	dev_dbg(nfc->dev, "hw ecc read page return %d\n", ret);
out:
	return ret;
}

static int nand_hw_ecc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required, int page)
{
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);
	void __iomem *regs = nfc->regs;
	int ret;

	int sectsize = nfc->sectsize;

	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize = chip->ecc.size;

	uint8_t *p = nfc->dmabuf;
	int subp;

	memset(p, 0xff, nfc->databuf_size);

	for (subp = 0; subp < eccsteps; subp++) {
		memcpy(p, buf, eccsize);
		p += sectsize;
		buf += eccsize;
	}

	/* ecc setting */
	nx_nandc_set_bchmode(regs, nfc->bchmode);
	nx_nandc_set_encmode(regs, 1);

	/* setup DMA */
	nx_nandc_set_dma_base(regs, (uint32_t)(nfc->dmabuf));
	nx_nandc_set_eccsize(regs, eccbytes - 1);
	nx_nandc_set_dmasize(regs, sectsize * eccsteps - 1);
	nx_nandc_set_subpage(regs, eccsteps - 1);
	nx_nandc_set_subpage_size(regs, sectsize - 1);

	ret = nx_nandc_run_dma(nfc);
	if (ret < 0)
		ret = -EIO;

	return ret;
}

static int verify_config(struct mtd_info *mtd)
{
	struct nxp3220_nfc *nfc = mtd_to_nfc(mtd);
	struct nand_chip *chip = mtd_to_nand(mtd);
	int ecctotal = chip->ecc.total;
	int oobsize = mtd->oobsize;

	if (ecctotal > oobsize) {
		dev_err(nfc->dev, "==================================================\n");
		dev_err(nfc->dev, "error: %d bit hw ecc mode requires ecc %d byte\n",
				chip->ecc.strength, ecctotal);
		dev_err(nfc->dev, "       it's over the oob %d byte for page %d byte\n",
				oobsize, mtd->writesize);
		dev_err(nfc->dev, "==================================================\n");

		return -EINVAL;
	}

	return 0;
}

static int nand_hw_ecc_init_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nxp3220_nfc *nfc = nand_get_controller_data(chip);

	int eccbyte = 0;
	int eccbits = chip->ecc.strength;
	int eccsize = chip->ecc.size;

	int sectsize;
	int bchmode;

	switch (eccbits) {
	case 4:
		eccbyte = NX_NANDC_ECC_SZ_512_4;
		bchmode = NX_NANDC_BCH_512_4;
		break;
	case 8:
		eccbyte = NX_NANDC_ECC_SZ_512_8;
		bchmode = NX_NANDC_BCH_512_8;
		break;
	case 12:
		eccbyte = NX_NANDC_ECC_SZ_512_12;
		bchmode = NX_NANDC_BCH_512_12;
		break;
	case 16:
		eccbyte = NX_NANDC_ECC_SZ_512_16;
		bchmode = NX_NANDC_BCH_512_16;
		break;
	case 24:
		if (eccsize == 512) {
			eccbyte = NX_NANDC_ECC_SZ_512_24;
			bchmode = NX_NANDC_BCH_512_24;
		} else {
			eccbyte = NX_NANDC_ECC_SZ_1024_24;
			bchmode = NX_NANDC_BCH_1024_24;
		}
		break;
	case 40:
		eccbyte = NX_NANDC_ECC_SZ_1024_40;
		bchmode = NX_NANDC_BCH_1024_40;
		break;
	case 60:
		eccbyte = NX_NANDC_ECC_SZ_1024_60;
		bchmode = NX_NANDC_BCH_1024_60;
		break;
	default:
		goto _ecc_fail;
		break;
	}

	sectsize = eccsize + eccbyte;
	if (sectsize % 2) {
		sectsize = sectsize + 1;
		chip->ecc.postpad = 1;
	}
	nfc->sectsize = sectsize;
	nfc->bchmode = bchmode;

	chip->ecc.mode = NAND_ECC_HW_SYNDROME;
	chip->ecc.bytes = eccbyte;
	chip->ecc.read_page = nand_hw_ecc_read_page;
	chip->ecc.write_page = nand_hw_ecc_write_page;
	chip->ecc.read_oob = nand_hw_ecc_read_oob;
	chip->ecc.write_oob = nand_hw_ecc_write_oob;

	return 0;

_ecc_fail:
	dev_err(nfc->dev, "Fail: %dbit ecc for pagesize %d is not supported!\n",
			eccbits, eccsize);
	return -EINVAL;
}

static int nx_wait_for_irq(struct nxp3220_nfc *nfc, int nr_int)
{
	int ret;

	ret = wait_for_completion_timeout(&nfc->dma_done,
			msecs_to_jiffies(1000));
	if (!ret) {
		nfc_regs_dump(nfc);
		dev_err(nfc->dev, "nand DMA transfer timeout!\n");
		ret = -ETIMEDOUT;
	} else {
		dev_dbg(nfc->dev, "waiting DMA done irq %d jiffies\n", ret);
		ret = 0;
	}

	return ret;
}

static irqreturn_t nxp3220_nfc_irq(int irq, void *dev_id)
{
	struct nxp3220_nfc *nfc = dev_id;

	spin_lock(&nfc->irq_lock);

	/* clear DMA interrupt pending */
	nx_nandc_clear_irq_pending(nfc->regs, NX_NANDC_INT_DMA);
	complete(&nfc->dma_done);

	spin_unlock(&nfc->irq_lock);

	return IRQ_HANDLED;
}

static int nxp3220_ooblayout_ecc(struct mtd_info *mtd, int section,
				   struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;

	if (section >= chip->ecc.steps)
		return -ERANGE;

	oobregion->offset = chunk * section;
	oobregion->length = chunk;

	return 0;
}

static int nxp3220_ooblayout_free(struct mtd_info *mtd, int section,
		struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;

	if (section >= chip->ecc.steps)
		return -ERANGE;

	oobregion->offset = (chunk * chip->ecc.steps) + 2;
	oobregion->length = mtd->oobsize - oobregion->offset;

	return 0;
}

static const struct mtd_ooblayout_ops nxp3220_ooblayout_ops = {
	.ecc = nxp3220_ooblayout_ecc,
	.free = nxp3220_ooblayout_free,
};

static int nxp3220_nfc_init(struct nxp3220_nfc *nfc)
{
	struct nand_chip *chip = &nfc->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret = 0;

	nand_dev_init(mtd);

	chip->options |= NAND_NO_SUBPAGE_WRITE;

	chip->IO_ADDR_R = chip->IO_ADDR_W = (nfc->regs + NFC_DATA_BYPASS);

	/* nand callbacks */
	chip->read_word = nand_read_word;

	chip->cmd_ctrl = nand_cmd_ctrl;
	chip->dev_ready = nand_dev_ready;
	chip->select_chip = nand_select_chip;

	if (nand_scan_ident(mtd, 1, NULL)) {
		dev_err(nfc->dev, "nand scan ident failed\n");
		goto err;
	}

	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		chip->bbt_options |= NAND_BBT_NO_OOB | NAND_BBT_NO_OOB_BBM;

	nand_hw_init_timings(chip);
	chip->setup_data_interface = nxp3220_setup_data_interface;

	mtd_set_ooblayout(mtd, &nxp3220_ooblayout_ops);

	nfc->databuf_size = mtd->writesize + mtd->oobsize;

	nfc->dmabuf = dma_alloc_coherent(nfc->dev, nfc->databuf_size,
			&nfc->dmaaddr, GFP_KERNEL);
	if (!nfc->dmabuf) {
		ret = -ENOMEM;
		goto err;
	}

	if (nand_hw_ecc_init_device(mtd) < 0) {
		dev_err(nfc->dev, "hw ecc device init failed\n");
		goto err_free_dma;
	}

	if (nand_scan_tail(mtd)) {
		dev_err(nfc->dev, "nand scan tail failed\n");
		goto err_free_dma;
	}

	if (verify_config(mtd) < 0)
		goto err_free_dma;

	if (mtd_device_parse_register(mtd, NULL, NULL, NULL, 0)) {
		dev_err(nfc->dev, "mtd parse partition error\n");
		goto err_free_dma;
	}

	return ret;

err_free_dma:
	dma_free_coherent(nfc->dev, nfc->databuf_size,
			nfc->dmabuf, nfc->dmaaddr);

err:
	return ret;
}

static int of_get_nand_timing_conf(struct nxp3220_nfc *nfc,
				   struct device_node *np)
{
	const struct nand_sdr_timings *timings;
	struct nand_chip *chip = &nfc->chip;
	u32 mode;

	if (!of_property_read_u32(np, "nand-timing-mode", &mode)) {
		timings = onfi_async_timing_mode_to_sdr_timings(mode);
		if (IS_ERR(timings)) {
			dev_err(nfc->dev, "nand: wrong timing mode provided\n");
			return PTR_ERR(timings);
		}

		chip->onfi_timing_mode_default = mode;
		nfc->timing_mode = mode;

		if (of_property_read_bool(np, "nand-timing-force"))
			nfc->timing_force = 1;
	}

	return 0;
}

static int nxp3220_nfc_probe(struct platform_device *pdev)
{
	struct nxp3220_nfc *nfc;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq;

	int ret = 0;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	chip = &nfc->chip;
	mtd = nand_to_mtd(chip);

	mtd->owner = THIS_MODULE;
	mtd->dev.parent = nfc->dev;
	mtd->name = DRV_NAME;

	platform_set_drvdata(pdev, nfc);
	nand_set_controller_data(chip, nfc);
	nand_set_flash_node(chip, dev->of_node);

	/* nWP gpio settings */
	nfc->wp_gpio = of_get_named_gpio(dev->of_node, "wp-gpio", 0);
	if (gpio_is_valid(nfc->wp_gpio)) {
		if (devm_gpio_request(dev, nfc->wp_gpio, "nand_wp")) {
			dev_err(dev, "failed to request WP gpio\n");
			nfc->wp_gpio = -EBUSY;
		} else {
			nand_wp_disable(nfc);
		}
	}

	/* resource settings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(nfc->regs))
		return PTR_ERR(nfc->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return -EINVAL;
	}

	if (of_get_nand_timing_conf(nfc, dev->of_node) < 0)
		goto err;

	/* clock settings */
	nfc->apb_clk = devm_clk_get(dev, "nand_apb");
	if (IS_ERR(nfc->apb_clk))
		return PTR_ERR(nfc->apb_clk);

	nfc->core_clk = devm_clk_get(dev, "nand_core");
	if (IS_ERR(nfc->core_clk))
		return PTR_ERR(nfc->core_clk);

	ret = clk_prepare_enable(nfc->apb_clk);
	if (ret) {
		dev_err(dev, "failed to nand apb_clk enable\n");
		goto err;
	}

	ret = clk_prepare_enable(nfc->core_clk);
	if (ret) {
		dev_err(dev, "failed to nand core_clk enable\n");
		goto err;
	}

	init_completion(&nfc->dma_done);
	spin_lock_init(&nfc->irq_lock);

	/* request interrupt */
	ret = devm_request_irq(dev, irq, nxp3220_nfc_irq, 0, DRV_NAME, nfc);
	if (ret) {
		dev_err(dev, "failed to request IRQ\n");
		goto err_clk_off;
	}

	return nxp3220_nfc_init(nfc);

err_clk_off:
	clk_disable_unprepare(nfc->core_clk);
err:
	nand_wp_enable(nfc);

	return ret;
}

static int nxp3220_nfc_remove(struct platform_device *pdev)
{
	struct nxp3220_nfc *nfc = platform_get_drvdata(pdev);
	struct nand_chip *chip = &nfc->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);

	dma_free_coherent(nfc->dev, nfc->databuf_size,
			nfc->dmabuf, nfc->dmaaddr);
	nand_release(mtd);
	clk_disable_unprepare(nfc->core_clk);
	clk_disable_unprepare(nfc->apb_clk);
	nand_wp_enable(nfc);

	return 0;
}

static int nxp3220_nfc_suspend(struct device *dev)
{
	struct nxp3220_nfc *nfc = dev_get_drvdata(dev);

	/* Enable write protect for safety */
	nand_wp_enable(nfc);

	clk_disable_unprepare(nfc->core_clk);
	clk_disable_unprepare(nfc->apb_clk);

	return 0;
}

static int nxp3220_nfc_resume(struct device *dev)
{
	struct nxp3220_nfc *nfc = dev_get_drvdata(dev);
	struct nand_chip *chip = &nfc->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	ret = clk_prepare_enable(nfc->apb_clk);
	if (ret) {
		dev_err(dev, "failed to nand apb_clk enable\n");
		return ret;
	}

	ret = clk_prepare_enable(nfc->core_clk);
	if (ret) {
		dev_err(dev, "failed to nand core_clk enable\n");
		return ret;
	}

	nand_dev_init(mtd);
	nand_reset(chip, 0);

	/* Disable write protect */
	nand_wp_disable(nfc);

	return 0;
}

static SIMPLE_DEV_PM_OPS(nxp3220_nfc_pm_ops,
			 nxp3220_nfc_suspend, nxp3220_nfc_resume);

static const struct of_device_id nxp3220_nfc_ids[] = {
	{ .compatible = "nexell,nxp3220-nand" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nxp3220_nfc_ids);

static struct platform_driver nxp3220_nfc_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = nxp3220_nfc_ids,
		.pm	= &nxp3220_nfc_pm_ops,
	},
	.probe		= nxp3220_nfc_probe,
	.remove		= nxp3220_nfc_remove,
};

module_platform_driver(nxp3220_nfc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bon-gyu, KOO <freestyle@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell NXP3220 Flash Controller Driver");
