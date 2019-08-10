// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include <linux/platform_data/dma-dw.h>

#include "spi-dw.h"

#define RX_BUSY		0
#define TX_BUSY		1

struct dw_spi_nx {
	struct dw_spi		dws;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control	*rst;
	struct regmap		*regmap;
	u32			slave_sel_offset;
	void			*dummypage;
};

static struct dw_dma_slave nx_dma_tx;
static struct dw_dma_slave nx_dma_rx;

static int nx_spi_dma_init(struct dw_spi *dws)
{
	struct dw_spi_nx *dws_nx = container_of(dws, struct dw_spi_nx, dws);
	struct dw_dma_slave *tx = dws->dma_tx;
	struct dw_dma_slave *rx = dws->dma_rx;
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* 1. Init rx channel */
	rx->dma_dev = &dws->master->dev;
	dws->rxchan = dma_request_slave_channel_compat(mask, NULL, NULL,
						       rx->dma_dev, "rx");
	if (!dws->rxchan)
		goto err_exit;
	dws->master->dma_rx = dws->rxchan;

	/* 2. Init tx channel */
	tx->dma_dev = &dws->master->dev;
	dws->txchan = dma_request_slave_channel_compat(mask, NULL, NULL,
						       tx->dma_dev, "tx");
	if (!dws->txchan)
		goto free_rxchan;
	dws->master->dma_tx = dws->txchan;

	dws->dma_inited = 1;

	dws_nx->dummypage = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!dws_nx->dummypage)
		goto err_no_dummypage;

	return 0;
err_no_dummypage:
	dma_release_channel(dws->txchan);
free_rxchan:
	dma_release_channel(dws->rxchan);
err_exit:
	return -EBUSY;
}

static void nx_spi_dma_exit(struct dw_spi *dws)
{
	if (!dws->dma_inited)
		return;

	dmaengine_terminate_sync(dws->txchan);
	dma_release_channel(dws->txchan);

	dmaengine_terminate_sync(dws->rxchan);
	dma_release_channel(dws->rxchan);
}

static irqreturn_t nx_dma_transfer(struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);

	if (!irq_status)
		return IRQ_NONE;

	dw_readl(dws, DW_SPI_ICR);
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s: FIFO overrun/underrun\n", __func__);
	dws->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(dws->master);
	return IRQ_HANDLED;
}

static bool nx_spi_can_dma(struct spi_master *master, struct spi_device *spi,
		struct spi_transfer *xfer)
{
	struct dw_spi *dws = spi_master_get_devdata(master);

	if (!dws->dma_inited)
		return false;

	return xfer->len > dws->fifo_len;
}

static enum dma_slave_buswidth nx_convert_dma_width(u32 dma_width)
{
	if (dma_width == 1)
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (dma_width == 2)
		return DMA_SLAVE_BUSWIDTH_2_BYTES;

	return DMA_SLAVE_BUSWIDTH_UNDEFINED;
}

static void setup_dma_scatter(struct dw_spi *dws, void *buffer,
		unsigned int length, struct sg_table *sgtab)
{
	struct dw_spi_nx *dws_nx = container_of(dws, struct dw_spi_nx, dws);
	struct scatterlist *sg;
	int bytesleft	= length;
	void *bufp	= buffer;
	int mapbytes;
	int i;

	if (buffer) {
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(bufp);
			sg_set_page(sg, virt_to_page(bufp),
					mapbytes, offset_in_page(bufp));
			bufp += mapbytes;
			bytesleft -= mapbytes;
			pr_debug("set RX/TX page @ %p, %d bytes, %d left\n",
					bufp, mapbytes, bytesleft);
		}
	} else {
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			if (bytesleft < PAGE_SIZE)
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE;

			sg_set_page(sg, virt_to_page(dws_nx->dummypage),
				mapbytes, 0);
			bytesleft -= mapbytes;
			pr_debug("set RX/TX page @ %p, %d bytes, %d left\n",
					bufp, mapbytes, bytesleft);
		}
	}
	WARN_ON(bytesleft);
}

/*
 * dws->dma_chan_busy is set before the dma transfer starts, callback for tx
 * channel will clear a corresponding bit.
 */
static void nx_dw_spi_dma_tx_done(void *arg)
{
	struct dw_spi *dws = arg;

	clear_bit(TX_BUSY, &dws->dma_chan_busy);
	if (test_bit(RX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static struct dma_async_tx_descriptor *nx_dw_spi_dma_prepare_tx(
		struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_slave_config txconf;
	struct dma_async_tx_descriptor *txdesc;

	int ret;
	unsigned int pages;
	int tx_sglen;
	void *tx = (void *)xfer->tx_buf;

	if (!dws->txchan)
		return NULL;

	txconf.direction = DMA_MEM_TO_DEV;
	txconf.dst_addr = dws->dma_addr;
	txconf.dst_maxburst = 16;
	txconf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	txconf.dst_addr_width = nx_convert_dma_width(dws->dma_width);
	txconf.device_fc = false;

	dmaengine_slave_config(dws->txchan, &txconf);

	if (!xfer->tx_buf) {
		pages = DIV_ROUND_UP(xfer->len, PAGE_SIZE);
		ret = sg_alloc_table(&xfer->tx_sg, pages, GFP_KERNEL);
		if (ret)
			goto err_alloc_tx_sg;
		setup_dma_scatter(dws, tx, xfer->len, &xfer->tx_sg);
		tx_sglen = dma_map_sg(&dws->master->dev, xfer->tx_sg.sgl,
				xfer->tx_sg.nents, DMA_TO_DEVICE);
		if (!tx_sglen)
			goto err_tx_sgmap;

		xfer->tx_sg.nents = tx_sglen;
	}

	txdesc = dmaengine_prep_slave_sg(dws->txchan,
				xfer->tx_sg.sgl,
				xfer->tx_sg.nents,
				DMA_MEM_TO_DEV,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		return NULL;

	txdesc->callback = nx_dw_spi_dma_tx_done;
	txdesc->callback_param = dws;

	return txdesc;

err_tx_sgmap:
	dma_unmap_sg(&dws->master->dev, xfer->tx_sg.sgl,
		xfer->tx_sg.nents, DMA_TO_DEVICE);

err_alloc_tx_sg:
	sg_free_table(&xfer->tx_sg);

	return NULL;
}

/*
 * dws->dma_chan_busy is set before the dma transfer starts, callback for rx
 * channel will clear a corresponding bit.
 */
static void nx_dw_spi_dma_rx_done(void *arg)
{
	struct dw_spi *dws = arg;

	clear_bit(RX_BUSY, &dws->dma_chan_busy);
	if (test_bit(TX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static struct dma_async_tx_descriptor *nx_dw_spi_dma_prepare_rx(
		struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_slave_config rxconf;
	struct dma_async_tx_descriptor *rxdesc;

	if (!xfer->rx_buf)
		return NULL;

	rxconf.direction = DMA_DEV_TO_MEM;
	rxconf.src_addr = dws->dma_addr;
	rxconf.src_maxburst = 16;
	rxconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	rxconf.src_addr_width = nx_convert_dma_width(dws->dma_width);
	rxconf.device_fc = false;

	dmaengine_slave_config(dws->rxchan, &rxconf);

	rxdesc = dmaengine_prep_slave_sg(dws->rxchan,
				xfer->rx_sg.sgl,
				xfer->rx_sg.nents,
				DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		return NULL;

	rxdesc->callback = nx_dw_spi_dma_rx_done;
	rxdesc->callback_param = dws;

	return rxdesc;
}

static int nx_spi_dma_setup(struct dw_spi *dws, struct spi_transfer *xfer)
{
	u16 dma_ctrl = 0;

	dw_writel(dws, DW_SPI_DMARDLR, 0xf);
	dw_writel(dws, DW_SPI_DMATDLR, 0x10);

	dma_ctrl |= SPI_DMA_TDMAE;

	if (xfer->rx_buf)
		dma_ctrl |= SPI_DMA_RDMAE;
	dw_writel(dws, DW_SPI_DMACR, dma_ctrl);

	/* Set the interrupt mask */
	spi_umask_intr(dws, SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI);

	dws->transfer_handler = nx_dma_transfer;

	return 0;
}

static int nx_spi_dma_transfer(struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *txdesc, *rxdesc;

	/* Prepare the TX dma transfer */
	txdesc = nx_dw_spi_dma_prepare_tx(dws, xfer);

	/* Prepare the RX dma transfer */
	rxdesc = nx_dw_spi_dma_prepare_rx(dws, xfer);

	/* rx must be started before tx due to spi instinct */
	if (rxdesc) {
		set_bit(RX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(rxdesc);
		dma_async_issue_pending(dws->rxchan);
	}

	if (txdesc) {
		set_bit(TX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(txdesc);
		dma_async_issue_pending(dws->txchan);
	}

	return 0;
}

static void nx_spi_dma_stop(struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_sync(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}
	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_sync(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}
}

static const struct dw_spi_dma_ops nx_dma_ops = {
	.dma_init	= nx_spi_dma_init,
	.dma_exit	= nx_spi_dma_exit,
	.dma_setup	= nx_spi_dma_setup,
	.can_dma	= nx_spi_can_dma,
	.dma_transfer	= nx_spi_dma_transfer,
	.dma_stop	= nx_spi_dma_stop,
};

static int nx_dw_spi_dma_init(struct dw_spi *dws)
{
	dws->dma_tx = &nx_dma_tx;
	dws->dma_rx = &nx_dma_rx;
	dws->dma_ops = &nx_dma_ops;

	return 0;
}

#ifdef SUPPORT_TO_RO_MODE
static void nx_spi_cs_control(u32 command)
{
	/* Dummy callback */
}

static void nx_spi_prefare_transfer(struct dw_spi *dws)
{
	struct dw_spi_nx *dws_nx = container_of(dws, struct dw_spi_nx, dws);

	if (!dws->slave)
		return;

	reset_control_assert(dws_nx->rst);
	udelay(1);
	reset_control_deassert(dws_nx->rst);
}

static struct dw_spi_chip nx_spi_chip = {
	.ssi_max_xfer_size = 32,
	.cs_control = nx_spi_cs_control,
	.prefare_transfer = nx_spi_prefare_transfer,
};
#endif

static int nx_dw_spi_probe(struct platform_device *pdev)
{
	struct dw_spi_nx *dws_nx;
	struct dw_spi *dws;
	struct resource *mem;
	int ret, num_cs;

	dws_nx = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_nx),
			GFP_KERNEL);
	if (!dws_nx)
		return -ENOMEM;

	dws = &dws_nx->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->paddr = mem->start;

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return dws->irq; /* -ENXIO */
	}

	/* Optional: Reset control */
	dws_nx->rst = devm_reset_control_get_optional_exclusive(&pdev->dev,
								NULL);
	dws_nx->pclk = devm_clk_get(&pdev->dev, "apb");
	if (IS_ERR(dws_nx->pclk)) {
		return PTR_ERR(dws_nx->pclk);
	}
	ret = clk_prepare_enable(dws_nx->pclk);
	if (ret)
		return ret;

	dws_nx->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(dws_nx->clk)) {
		ret = PTR_ERR(dws_nx->clk);
		goto err_clk;
	}
	ret = clk_prepare_enable(dws_nx->clk);
	if (ret)
		goto err_clk;

	dws->bus_num = pdev->id;
	dws->max_freq = clk_get_rate(dws_nx->pclk);

	if (device_property_read_u32(&pdev->dev, "spi-mode", &dws->spi_mode))
		dws->spi_mode = 0;
	if (device_property_read_u32(&pdev->dev, "num-cs", &num_cs))
		dws->num_cs = 1;
	else
		dws->num_cs = (u16)num_cs;

	if (device_property_read_u32(&pdev->dev, "reg-io-width",
				&dws->reg_io_width))
		dws->reg_io_width = 4;

	if (device_property_read_bool(&pdev->dev, "spi-dma"))
		nx_dw_spi_dma_init(dws);

	if (device_property_read_bool(&pdev->dev, "spi-slave")) {
		struct device_node *of_node = pdev->dev.of_node;

		dws_nx->regmap = syscon_regmap_lookup_by_phandle(of_node,
								 "syscon0");
		if (IS_ERR(dws_nx->regmap)) {
			dev_err(&pdev->dev, "regmap lookup failed: %ld\n",
				PTR_ERR(dws_nx->regmap));
			goto err_clk;
		}

		if (device_property_read_u32(&pdev->dev, "slave-sel-offset",
					     &dws_nx->slave_sel_offset)) {
			dev_err(&pdev->dev, "cannot find slave-sel-offset\n");
			goto err_clk;
		}

		/* Set Slave Mode */
		regmap_update_bits(dws_nx->regmap, dws_nx->slave_sel_offset,
				   BIT(0), 0x1);
		dws->slave = true;
	}
#ifdef SUPPORT_TO_RO_MODE
	/* set chip info to support TR/RO/TO mode */
	dws->chip_info = &nx_spi_chip;
#endif

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto err_pclk;

	platform_set_drvdata(pdev, dws_nx);

	return 0;

err_clk:
	clk_disable_unprepare(dws_nx->clk);

err_pclk:
	clk_disable_unprepare(dws_nx->pclk);

	return ret;
}

static int nx_dw_spi_remove(struct platform_device *pdev)
{
	struct dw_spi_nx *dws_nx = platform_get_drvdata(pdev);

	if (dws_nx->dws.slave) {
		/* Back to Master Mode */
		regmap_update_bits(dws_nx->regmap, dws_nx->slave_sel_offset,
				   BIT(0), 0x0);
	}

	dw_spi_remove_host(&dws_nx->dws);
	clk_disable_unprepare(dws_nx->clk);
	clk_disable_unprepare(dws_nx->pclk);

	kfree(dws_nx->dummypage);

	return 0;
}

static const struct of_device_id nx_dw_spi_of_match[] = {
	{ .compatible = "nexell,nxp3220-dw-spi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, nx_dw_spi_of_match);

static struct platform_driver nx_dw_spi_driver = {
	.probe		= nx_dw_spi_probe,
	.remove		= nx_dw_spi_remove,
	.driver		= {
		.name	= "dw_spi_nx",
		.of_match_table = nx_dw_spi_of_match,
	},
};
module_platform_driver(nx_dw_spi_driver);

MODULE_AUTHOR("JongKeun Choi <jkchoi@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
