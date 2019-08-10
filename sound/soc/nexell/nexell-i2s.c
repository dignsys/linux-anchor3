// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound I2S driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "nexell-pcm.h"

struct i2s_reg {
	u32 con; /* 0x00 */
	u32 mod; /* 0x04 */
	u32 fic; /* 0x08 */
	u32 txd; /* 0x10 */
	u32 rxd; /* 0x14 */
};

#define	CON_TXDMA_ACT		(1<<2)
#define	CON_RXDMA_ACT		(1<<1)
#define	CON_DMA_MASK		(3<<1)
#define	CON_IIS_ACT		(1<<0)

#define MOD_BLC_16BIT		(0<<13)
#define MOD_BLC_8BIT		(1<<13)
#define MOD_BLC_24BIT		(2<<13)
#define	MOD_BLC_MASK		(3<<13)
#define	MOD_MLCK_NOUT		(1<<12)	/* 0:inner mclk, 1:external */
#define	MOD_IMS_MASTER		(0<<10)
#define	MOD_IMS_SLAVE		(2<<10)
#define	MOD_IMS_MASK		(3<<10)
#define MOD_TX_MODE		(0<<8)
#define MOD_RX_MODE		(1<<8)
#define MOD_TRX_MODE		(2<<8)
#define MOD_TXR_MASK		(3<<8)
#define MOD_LRP_LLOW		(0<<7)
#define MOD_LRP_RLOW		(1<<7)
#define MOD_LRP_MASK		(1<<7)
#define	MOD_SDF_IIS		(0<<5)
#define	MOD_SDF_MSB		(1<<5)
#define	MOD_SDF_LSB		(2<<5)
#define MOD_SDF_MASK		(3<<5)
#define MOD_RFS_256		(0<<3)
#define MOD_RFS_512		(1<<3)
#define MOD_RFS_384		(2<<3)
#define MOD_RFS_768		(3<<3)
#define	MOD_RFS_MASK		(3<<3)
#define MOD_BFS_32BIT		(0<<1)
#define MOD_BFS_48BIT		(1<<1)
#define MOD_BFS_16BIT		(2<<1)
#define MOD_BFS_24BIT		(3<<1)
#define	MOD_BFS_MASK		(3<<1)

#define FIC_TX_FLUSH		(1<<15)
#define FIC_RX_FLUSH		(1<<7)

struct clock_ratio {
	unsigned int sample_rate;
	unsigned int ratio_256;
	unsigned int ratio_384;
	unsigned int dfs_ratio_256;
	unsigned int dfs_ratio_384;
};

static struct clock_ratio clk_ratio[] = {
	{   8000,  2048000,  3072000, 614400000, 614400000 },
	{  11025,  2822400,  4233600, 677376000, 677376000 },
	{  16000,  4096000,  6144000, 614400000, 614400000 },
	{  22050,  5644800,  8467200, 677376000, 677376000 },
	{  32000,  8192000, 12288000, 638976000, 614400000 },
	{  44100, 11289600, 16934400, 677376000, 677376000 },
	{  48000, 12288000, 18432000, 614400000, 589824000 },
	{  64000, 16384000, 24576000, 589824000, 638976000 },
	{  88200, 22579200, 33868800, 677376000, 677376000 },
	{  96000, 24576000, 36864000, 638976000, 589824000 },
	{ 176400, 45158400, 67737600, 632217600, 677376000 },
	{ 192000, 49152000, 73728000, 589824000, 589824000 },
};

struct nx_i2s_match_data {
	int buswidth;
	int maxburst;
};

struct nx_i2s_data {
	struct device *dev;
	struct i2s_reg *base;
	dma_addr_t addr;
	struct clk *clk;
	struct clk *pclk;
	struct clk *pll;
	unsigned int sysclk_freq; /* set from `set_sysclk` */
	unsigned int mclk_freq;
	unsigned int sys_sample_rate;
	int rfs; /* root frame samples */
	int bfs; /* bit frame samples */

	struct nx_i2s_match_data *match;
	struct snd_soc_dai_driver dai_driver;
	struct nx_pcm_dma_param dma_params[2];

	/* properties */
	int master; /* frame-master/bitclock-master in simple-card */
	int sample_rate; /* fix sample rate range */
	int frame_bit; /* fix frame bit range */
	int external_frequency; /* when use external clock in master mode */
	int supply_mclk_always;
	int supply_lrck_always;

	/* susepnd */
	u32 iis_con;
	u32 iis_mod;
};

#define I2S_SND_SOC_FORMATS ( \
	SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8 | \
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U24_LE \
	)

#define I2S_SND_SOC_RATES	SNDRV_PCM_RATE_8000_192000

static inline void i2s_debug_show(struct nx_i2s_data *i2s)
{
	struct i2s_reg *reg = i2s->base;

	dev_dbg(i2s->dev, "i2s.0x%x con: 0x%08x\n",
		i2s->addr, readl(&reg->con));
	dev_dbg(i2s->dev, "i2s.0x%x mod: 0x%08x\n",
		i2s->addr, readl(&reg->mod));
	dev_dbg(i2s->dev, "i2s.0x%x fic: 0x%08x\n",
		i2s->addr, readl(&reg->fic));
}

static inline bool i2s_is_active(struct i2s_reg *reg)
{
	return readl(&reg->con) & CON_IIS_ACT ? true : false;
}

static void nx_i2s_fifo_flush(struct nx_i2s_data *i2s, int stream)
{
	struct i2s_reg *reg = i2s->base;
	u32 fic;

	fic = stream == SNDRV_PCM_STREAM_PLAYBACK ?
		FIC_TX_FLUSH : FIC_RX_FLUSH;

	/* On start, ensure that the FIFOs are cleared and reset. */
	writel(fic, &reg->fic);
	writel(0, &reg->fic);
}

static void nx_i2s_trx_start(struct nx_i2s_data *i2s, int stream)
{
	struct i2s_reg *reg = i2s->base;
	u32 mod = readl(&reg->mod);
	u32 con = readl(&reg->con) | CON_IIS_ACT;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		con |= CON_TXDMA_ACT;
	else
		con |= CON_RXDMA_ACT;

	writel(mod | MOD_TRX_MODE, &reg->mod);
	writel(con, &reg->con);

	i2s_debug_show(i2s);
}

static void nx_i2s_trx_stop(struct nx_i2s_data *i2s, int stream)
{
	struct i2s_reg *reg = i2s->base;
	u32 mod = readl(&reg->mod);
	u32 con = readl(&reg->con);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		con &= ~CON_TXDMA_ACT;
	else
		con &= ~CON_RXDMA_ACT;

	if (!i2s->supply_lrck_always && !(con & CON_DMA_MASK))
		con &= ~CON_IIS_ACT;

	writel(con, &reg->con);
	writel(mod, &reg->mod);
}

static void nx_i2s_clk_dir(struct nx_i2s_data *i2s)
{
	struct i2s_reg *reg = i2s->base;
	u32 mod = readl(&reg->mod);
	int dir = i2s->external_frequency;

	if (dir != -1) {
		if (i2s->master)
			mod &= ~MOD_MLCK_NOUT;
		else
			mod |= MOD_MLCK_NOUT;
	} else {
		mod &= ~MOD_MLCK_NOUT;
	}

	writel(mod, &reg->mod);
}

static struct clock_ratio *nx_i2s_clk_get_ratio(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clk_ratio); i++) {
		if (rate == clk_ratio[i].sample_rate)
			return &clk_ratio[i];
	}

	return NULL;
}

static int nx_i2s_clk_set_rate(struct nx_i2s_data *i2s,
			unsigned int rate, int bfs)
{
	struct clock_ratio *ratio = NULL;
	unsigned long pll, request, freq;
	int rfs;

	if (!i2s->master)
		return 0;

	if (i2s->external_frequency != -1) {
		i2s->sys_sample_rate = rate;
		i2s->mclk_freq = i2s->external_frequency;
		if (bfs == 16 || bfs == 32)
			i2s->rfs = i2s->external_frequency / rate;
		else
			i2s->rfs = 384;
		return 0;
	}

	if (i2s->sys_sample_rate == rate && i2s->bfs == bfs)
		return 0;

	ratio = nx_i2s_clk_get_ratio(rate);
	if (!ratio) {
		dev_err(i2s->dev,
			"%s: Not support i2s.0x%x sample rate:%d !\n",
			__func__, i2s->addr, rate);
		return -EINVAL;
	}

	if (bfs != 16 && bfs != 32 && bfs != 24 && bfs != 48) {
		dev_err(i2s->dev,
			"%s: Not support i2s.0x%x frame bit:%d !\n",
			__func__, i2s->addr, bfs);
		return -EINVAL;
	}

	rfs = 384;
	pll = ratio->dfs_ratio_384;
	request = rfs * ratio->sample_rate;

	if (!IS_ERR(i2s->pll))
		clk_set_rate(i2s->pll, pll);

	freq = clk_round_rate(i2s->clk, request);

	/* bfs 16/32 support 256 rfs */
	if (freq != request && (bfs == 16 || bfs == 32)) {
		unsigned long old = freq;

		rfs = 256;
		pll = ratio->dfs_ratio_256;
		request = rfs * ratio->sample_rate;

		if (!IS_ERR(i2s->pll))
			clk_set_rate(i2s->pll, pll);

		freq = clk_round_rate(i2s->clk, request);
		if (abs(request - freq) > abs(request - old)) {
			rfs = 384;
			pll = ratio->dfs_ratio_384;
			freq = old;
		}
	}

	if (!IS_ERR(i2s->pll)) {
		clk_set_rate(i2s->pll, pll);
		dev_dbg(i2s->dev, "i2s.0x%x clk pll:%ld (%ld)\n",
			i2s->addr, pll, clk_get_rate(i2s->pll));
	}

	i2s->mclk_freq = freq;
	i2s->sys_sample_rate = rate;
	i2s->rfs = rfs;
	i2s->bfs = bfs;

	clk_set_rate(i2s->clk, freq);

	dev_dbg(i2s->dev, "i2s.0x%x %d:%ld khz, rate:%ld, bfs:%d, rfs:%d\n",
		i2s->addr, rate, freq/rfs, freq, bfs, rfs);

	return 0;
}

static void nx_i2s_clk_enable(struct nx_i2s_data *i2s)
{
	if (!i2s_is_active(i2s->base))
		clk_prepare_enable(i2s->clk);
}

static void nx_i2s_clk_disable(struct nx_i2s_data *i2s)
{
	if (i2s->supply_mclk_always)
		return;

	if (!__clk_is_enabled(i2s->clk))
		return;

	if (!i2s_is_active(i2s->base))
		clk_disable_unprepare(i2s->clk);
}

static int nx_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_reg *reg = i2s->base;
	u32 mod, tmp = 0;

	dev_dbg(i2s->dev, "i2s.0x%x dai format 0x%x\n", i2s->addr, fmt);

	/* mclk dir */
	nx_i2s_clk_dir(i2s);

	mod = readl(&reg->mod);

	/* master / slave */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM: /* codec master */
		tmp = MOD_IMS_SLAVE;
		i2s->master = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* codec slave */
		tmp = MOD_IMS_MASTER;
		i2s->master = true;
		break;
	default:
		dev_err(i2s->dev,
			"%s: Invalid i2s.0x%x master/slave format:0x%x\n",
			__func__, i2s->addr,
			(fmt & SND_SOC_DAIFMT_MASTER_MASK));
		return -EINVAL;
	}

	/* mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		tmp |= MOD_LRP_RLOW;
		tmp |= MOD_SDF_MSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		tmp |= MOD_LRP_RLOW;
		tmp |= MOD_SDF_LSB;
		break;
	case SND_SOC_DAIFMT_I2S:
		tmp |= MOD_SDF_IIS;
		break;
	default:
		dev_err(i2s->dev,
			"%s: Invalid i2s.0x%x format:0x%x\n",
			__func__, i2s->addr,
			(fmt & SND_SOC_DAIFMT_FORMAT_MASK));
		return -EINVAL;
	}

	/* invert */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		if (tmp & MOD_LRP_RLOW)
			tmp &= ~MOD_LRP_RLOW;
		else
			tmp |= MOD_LRP_RLOW;
		break;
	default:
		break;
	}

	mod &= ~(MOD_SDF_MASK | MOD_LRP_MASK | MOD_IMS_MASK);
	mod |= tmp;

	writel(mod, &reg->mod);

	dev_info(i2s->dev,
		"%s, %dhz, %s, pol:%s dir:%s\n",
		(tmp & MOD_IMS_MASK) == MOD_IMS_MASTER ?
		"master" : "slave", i2s->sysclk_freq,
		(tmp & MOD_SDF_MASK) == MOD_SDF_IIS ? "iis" :
		(tmp & MOD_SDF_MASK) == MOD_SDF_LSB ? "left_j" :
		(tmp & MOD_SDF_MASK) == MOD_SDF_MSB ? "right_j" : "unknown",
		(tmp & MOD_LRP_RLOW) ? "invert" : "normal",
		(mod & MOD_MLCK_NOUT) ? "in" : "out");

	return 0;
}

static int nx_i2s_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	struct nx_pcm_dma_param *dmap;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dmap = &i2s->dma_params[0];
	else
		dmap = &i2s->dma_params[1];

	nx_i2s_clk_enable(i2s);

	snd_soc_dai_set_dma_data(dai, substream, dmap);

	return 0;
}

static void nx_i2s_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);

	nx_i2s_clk_disable(i2s);
}

static int nx_i2s_set_sysclk(struct snd_soc_dai *dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);

	dev_dbg(i2s->dev, "%s, i2s.0x%x, id:%d, freq:%u hz, dir:%d\n",
		 __func__, i2s->addr, clk_id, freq, dir);

	i2s->sysclk_freq = freq;

	return 0;
}

static int nx_i2s_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_reg *reg = i2s->base;
	struct nx_pcm_dma_param *dmap;
	u32 mod;
	unsigned int rate;
	int samples, frames, rfs;
	int ret;

	mod = readl(&reg->mod) & ~(MOD_BLC_MASK | MOD_BFS_MASK | MOD_RFS_MASK);

	rate = params_rate(params);
	samples = params_width(params);
	frames = snd_pcm_format_width(params_format(params)) * 2;
	rfs = i2s->sysclk_freq/rate;

	dev_dbg(i2s->dev, "i2s.0x%x params %d khz sample:%d, frame:%d, RFS:%d\n",
		i2s->addr, rate, samples, frames, rfs);

	switch (samples) {
	case 8:
		mod |= MOD_BLC_8BIT;
		break;
	case 16:
		mod |= MOD_BLC_16BIT;
		break;
	case 24:
		mod |= MOD_BLC_24BIT;
		break;
	default:
		dev_err(i2s->dev,
			"%s: Not support i2s.0x%x sample bit:%d\n",
			__func__, i2s->addr, samples);
		break;
		return -EINVAL;
	}

	switch (frames) {
	case 16:
		mod |= MOD_BFS_16BIT;
		break;
	case 32:
		mod |= MOD_BFS_32BIT;
		break;
	case 48:
		mod |= MOD_BFS_48BIT;
		break;
	default:
		dev_err(i2s->dev,
			"%s: Not support i2s.0x%x frame bit:%d\n",
			__func__, i2s->addr, frames);
		return -EINVAL;
	}

	if (!i2s->master) {
		writel(mod, &reg->mod);
		return 0;
	}

	ret = nx_i2s_clk_set_rate(i2s, rate, frames);
	if (ret)
		return ret;

	if (rfs && i2s->rfs != rfs) {
		dev_dbg(i2s->dev, "Check clock setting for %d Khz\n", rate);
		dev_dbg(i2s->dev, "Require %d/%d, IIS %d/%d\n",
			i2s->sysclk_freq, rfs, i2s->mclk_freq, i2s->rfs);
	}

	switch (i2s->rfs) {
	case 256:
		mod |= MOD_RFS_256;
		break;
	case 384:
		mod |= MOD_RFS_384;
		break;
	default:
		break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dmap = &i2s->dma_params[0];
	else
		dmap = &i2s->dma_params[1];

	dmap->clk_sample_rate = i2s->mclk_freq / i2s->rfs;

	writel(mod, &reg->mod);

	return 0;
}

static int nx_i2s_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	dev_dbg(i2s->dev, "i2s.0x%x:%s cmd:%d\n",
		i2s->addr, snd_pcm_stream_str(substream), cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		nx_i2s_fifo_flush(i2s, stream);

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		nx_i2s_trx_start(i2s, stream);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		nx_i2s_trx_stop(i2s, stream);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops nx_i2s_ops = {
	.set_fmt = nx_i2s_set_fmt,
	.startup = nx_i2s_startup,
	.shutdown = nx_i2s_shutdown,
	.set_sysclk = nx_i2s_set_sysclk,
	.hw_params = nx_i2s_hw_params,
	.trigger = nx_i2s_trigger,
};

static int nx_i2s_suspend(struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_reg *reg = i2s->base;

	if (i2s_is_active(i2s->base))
		clk_disable_unprepare(i2s->clk);

	i2s->iis_mod = readl(&reg->mod);
	i2s->iis_con = readl(&reg->con);

	clk_disable_unprepare(i2s->pclk);

	return 0;
}

static int nx_i2s_resume(struct snd_soc_dai *dai)
{
	struct nx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_reg *reg = i2s->base;

	clk_prepare_enable(i2s->pclk);

	writel(FIC_TX_FLUSH | FIC_RX_FLUSH, &reg->fic);
	writel(0, &reg->fic);
	writel(i2s->iis_mod, &reg->mod);
	writel(i2s->iis_con, &reg->con);

	if (i2s_is_active(i2s->base) || i2s->supply_mclk_always)
		clk_prepare_enable(i2s->clk);

	return 0;
}

static int nx_i2s_clock_setup(struct platform_device *pdev,
			struct nx_i2s_data *i2s)
{
	struct device_node *node = pdev->dev.of_node;
	struct i2s_reg *reg = i2s->base;

	i2s->clk = of_clk_get_by_name(node, "i2s");
	if (IS_ERR(i2s->clk)) {
		dev_err(&pdev->dev,
			"Failed to clk i2s.0x%x 'i2s'\n", i2s->addr);
		return PTR_ERR(i2s->clk);
	}

	i2s->pclk = of_clk_get_by_name(node, "pclk");
	if (IS_ERR(i2s->pclk)) {
		dev_err(&pdev->dev,
			"Failed to clk i2s.0x%x 'pclk'\n", i2s->addr);
		return PTR_ERR(i2s->pclk);
	}

	i2s->pll = of_clk_get_by_name(node, "pll");

	clk_prepare_enable(i2s->pclk);

	if (i2s->supply_lrck_always)
		writel(readl(&reg->con) | CON_IIS_ACT, &reg->con);

	dev_info(&pdev->dev,
		"i2s.0x%x: iis %lu hz [pclk:%lu hz] mclk:%s, lrck:%s\n",
		i2s->addr, clk_get_rate(i2s->clk), clk_get_rate(i2s->pclk),
		i2s->supply_mclk_always ? "always" : "runtime",
		i2s->supply_lrck_always ? "always" : "runtime");

	return 0;
}

static int nx_i2s_dai_driver_setup(struct platform_device *pdev,
			struct nx_i2s_data *i2s)
{
	struct snd_soc_dai_driver *dai_drv = &i2s->dai_driver;
	struct clock_ratio *ratio = clk_ratio;
	unsigned int format = 0, rate = 0;

	dai_drv->suspend = nx_i2s_suspend;
	dai_drv->resume = nx_i2s_resume;
	dai_drv->symmetric_rates = 1;
	dai_drv->ops = &nx_i2s_ops;

	/* playback */
	dai_drv->playback.stream_name = "Playback";
	dai_drv->playback.channels_min = 2;
	dai_drv->playback.channels_max = 2;
	dai_drv->playback.formats = I2S_SND_SOC_FORMATS;
	dai_drv->playback.rates = I2S_SND_SOC_RATES;
	dai_drv->playback.rate_min = 8000;
	dai_drv->playback.rate_max = 192000;

	/* capture */
	dai_drv->capture.stream_name = "Capture";
	dai_drv->capture.channels_min = 2;
	dai_drv->capture.channels_max = 2;
	dai_drv->capture.formats = I2S_SND_SOC_FORMATS;
	dai_drv->capture.rates = I2S_SND_SOC_RATES;
	dai_drv->capture.rate_min = 8000;
	dai_drv->capture.rate_max = 192000;

	if (i2s->frame_bit) {
		switch (i2s->frame_bit) {
		case 16:
			format = SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8;
			break;
		case 32:
			format = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_U16_LE;
			break;
		case 48:
			format = SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_U24_LE;
			break;
		default:
			dev_err(i2s->dev,
				"%s: Not support i2s.0x%x fix frame bit:%d\n",
				__func__, i2s->addr, i2s->frame_bit);
			break;
		}
	}

	if (i2s->sample_rate) {
		ratio = nx_i2s_clk_get_ratio(i2s->sample_rate);
		if (!ratio)
			dev_err(i2s->dev,
				"%s: Not support i2s.0x%x fix sample rate:%d\n",
				__func__, i2s->addr, i2s->sample_rate);
		else
			rate = ratio->sample_rate;
	}

	if (format) {
		dai_drv->playback.formats = format;
		dai_drv->capture.formats = format;
	}

	if (rate) {
		dai_drv->playback.rate_min = rate;
		dai_drv->capture.rate_min = rate;
		dai_drv->playback.rate_max = rate;
		dai_drv->capture.rate_max = rate;
	}

	return 0;
}

static const struct of_device_id of_i2s_match[];

static int nx_i2s_pcm_dma_setup(struct platform_device *pdev,
			struct nx_i2s_data *i2s)
{
	const struct of_device_id *match;
	struct nx_pcm_dma_param *dma_params = i2s->dma_params;
	int offset, i;

	match = of_match_node(of_i2s_match, pdev->dev.of_node);
	i2s->match = (struct nx_i2s_match_data *)match->data;

	for (offset = 0x10, i = 0; i < 2; i++, offset += 4) {
		dma_params[i].dev = &pdev->dev;
		dma_params[i].addr = i2s->addr + offset;
		dma_params[i].channel = i2s->addr;
		dma_params[i].buswidth = i2s->match->buswidth;
		dma_params[i].maxburst = i2s->match->maxburst;
		dma_params[i].filter_data = "i2s";
	}

	dev_info(&pdev->dev, "i2s.0x%x: dma:i2s, busw:%d, maxburst:%d\n",
		i2s->addr, i2s->match->buswidth, i2s->match->maxburst);

	return 0;
}

static int nx_i2s_parse_dt(struct platform_device *pdev,
			struct nx_i2s_data *i2s)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s: Can't get IO resource.\n", __func__);
		return -ENOENT;
	}

	i2s->dev = &pdev->dev;
	i2s->addr = res->start;
	i2s->base = devm_ioremap_resource(&pdev->dev, res);
	i2s->external_frequency = -1;

	of_property_read_u32(node, "frame-bit", &i2s->frame_bit);
	of_property_read_u32(node, "sample-rate", &i2s->sample_rate);
	of_property_read_u32(node, "external-mclk-frequency",
		&i2s->external_frequency);
	i2s->supply_lrck_always =
		of_property_read_bool(node, "supply-lrck-always");
	i2s->supply_mclk_always =
		of_property_read_bool(node, "supply-mclk-always");

	if (i2s->supply_lrck_always)
		i2s->supply_mclk_always = 1;

	return 0;
}

static const struct snd_soc_component_driver nx_i2s_comp = {
	.name = "nx-i2sc",
};

static int nx_i2s_probe(struct platform_device *pdev)
{
	struct nx_i2s_data *i2s;
	int ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	ret = nx_i2s_parse_dt(pdev, i2s);
	if (ret)
		goto err_out;

	nx_i2s_dai_driver_setup(pdev, i2s);
	nx_i2s_pcm_dma_setup(pdev, i2s);

	ret = nx_i2s_clock_setup(pdev, i2s);
	if (ret)
		goto err_out;

	ret = devm_snd_soc_register_component(&pdev->dev,
				&nx_i2s_comp, &i2s->dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed, i2s.0x%x snd register component !\n",
			i2s->addr);
		goto err_out;
	}

	ret = nx_pcm_snd_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed, i2s.0x%x snd register platform !\n",
			i2s->addr);
		goto err_out;
	}

	dev_set_drvdata(&pdev->dev, i2s);

	return ret;

err_out:
	devm_kfree(&pdev->dev, i2s);

	return ret;
}

static int nx_i2s_remove(struct platform_device *pdev)
{
	struct nx_i2s_data *i2s = dev_get_drvdata(&pdev->dev);

	nx_pcm_snd_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	if (!IS_ERR(i2s->clk))
		clk_put(i2s->clk);

	if (!IS_ERR(i2s->pll))
		clk_put(i2s->pll);

	if (!IS_ERR(i2s->pclk)) {
		clk_disable_unprepare(i2s->pclk);
		clk_put(i2s->pclk);
	}

	devm_kfree(&pdev->dev, i2s);

	return 0;
}

static struct nx_i2s_match_data nxp3220_i2s_port = {
	.buswidth = 4,
	.maxburst = 4,
};

static const struct of_device_id of_i2s_match[] = {
	{ .compatible = "nexell,nxp3220-i2s", .data = &nxp3220_i2s_port, },
	{ },
};
MODULE_DEVICE_TABLE(of, of_i2s_match);

static struct platform_driver nx_i2s_driver = {
	.probe  = nx_i2s_probe,
	.remove = nx_i2s_remove,
	.driver = {
		.name = "nexell-i2s",
		.of_match_table = of_match_ptr(of_i2s_match),
	},
};

#ifdef CONFIG_DEFERRED_SOUND_I2S
static int __init nx_i2s_driver_init(void)
{
	return platform_driver_register(&nx_i2s_driver);
}
deferred_module_init(nx_i2s_driver_init)
#else
module_platform_driver(nx_i2s_driver);
#endif

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound I2S driver for Nexell sound");
MODULE_LICENSE("GPL");
