// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound PDM/PCM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <sound/soc.h>

#define PDM_CLK_ENB		(1<<2 | 1<<0)
#define PDM_CLK_DIV_L(x)	((x & 0xFF) << 8)
#define PDM_CLK_DIV_H(x)	((x & 0xFF) << 16)
#define PDM_LOCK_CNT(x)		(x & ((1<<23) - 1))
#define PDM_LOCK_1ST_ENB	(1<<30)
#define PDM_LOCK_LRCK		(1<<31)
#define PDM_CIC_CH(x)		((x & 0x03) << 0)
#define PDM_CIC_COMB(x)		((x & 0x07) << 4)
#define PDM_CIC_INTG(x)		((x & 0x07) << 8)
#define PDM_CIC_SHIFT(x)	((x & 0x0f) << 16)
#define PDM_CIC_DIV(x)		((x & 0x3f) << 24)
#define PDM_SRC_PAD(x, ch)	((x & 0x3) << (ch * 4))
#define PDM_PARAMS_HBP(x)	(x ? 1<<30 : 0<<30)
#define PDM_PARAMS_OUT(x)	(x ? 1<<31 : 0<<31)
#define PDM_PARAMS_SIZE(s)	((s/4) & 0xFF)
#define IRQ_TOUT_ENB		(1<<1 | 1<<0)
#define IRQ_TOUT_PEND		(1<<2)
#define IRQ_LDET_ENB		(1<<5 | 1<<4) /* sync detect */
#define IRQ_LDET_PEND		(1<<6)
#define IRQ_DMA_ENB		(1<<9 | 1<<8) /* (0<<9 | 1<<8)  */
#define IRQ_DMA_MASK		(1<<9 | 1<<8) /* (1<<9 | 1<<8)  */
#define IRQ_DMA_PEND		(1<<10)

#define IRQ_MASK_PEND		(IRQ_TOUT_PEND | IRQ_LDET_PEND | IRQ_DMA_PEND)

#define	MEM_BURST_SIZE		(8 * 8) /* 64bit 8 burst :64byte */
#define	MEM_SIZE		(0x4000)
#define	MEM_CIC_SIZE		(199 * 4)
#define MAX_PDM_PADS		4
#define MAX_REF_CH		3
#define PDM_LOCK_MAX		(1<<(23 - 1))
#define _word_(x)		(x/4)

struct pdm_fir_reg {
	u32 rd_offs;	/* 0x100 */
	u32 rd_end;	/* 0x104 */
	u32 wr_offs;	/* 0x108 */
	u32 wr_end;	/* 0x10c */
	u32 coef_offs;	/* 0x110 */
	u32 coef_num;	/* 0x114 */
	u32 resv[2];
};

struct pdm_reg {
	u32 ctrl;	/* 0x00 */
	u32 pdm_clk;	/* 0x04 */
	u32 pdm_lock;	/* 0x08 */
	u32 pdm_tout;	/* 0x0c */
	u32 pdm_src;	/* 0x10 */
	u32 cic_cfg;	/* 0x14 */
	u32 cic_pos_v;	/* 0x18 */
	u32 cic_neg_v;	/* 0x1c */
	u32 dma_size;	/* 0x20 */
	u32 dma_addr0;	/* 0x24 */
	u32 dma_addr1;	/* 0x28 */
	u32 fir_num;	/* 0x2c */
	u32 cic_offs;	/* 0x30 */
	u32 cic_end;	/* 0x34 */
	u32 intr;	/* 0x38 */
	u32 mem_sleep;	/* 0x3c */
	u32 resv[50-2];
	struct pdm_fir_reg firs[6];
};

enum pdm_sync_type {
	pdm_sync_none, /* Not sync, source system clk */
	pdm_sync_mclk, /* sync iis mclk, source mclk */
	pdm_sync_lrck, /* sync iis lrck, source system clk */
};

struct pdm_cic_cfg {
	int comb_stages;
	int integrators;
	int shfit, divide;
	int posv, negv;
};

struct pdm_coeff_cfg {
	bool hbf; /* Halfband Filter or LowPass Filter */
	u32 *buffer;
	int size;
};

struct pdm_fir_cfg {
	struct pdm_coeff_cfg *coef;
	int num_coefs;
};

struct nx_pdm_data {
	struct device *dev;
	struct pdm_reg *base;
	void __iomem *mem;
	dma_addr_t addr;
	int irq;
	int channel;
	struct clk *clk;
	struct clk *clk_axi;
	struct regmap *syscon;
	enum pdm_sync_type sync_type;
	int ref_iis;
	unsigned int sys_freq;
	unsigned int ref_freq;
	int ref_sample_rate;
	int sample_rate;
	int strobe_hz;
	unsigned int timeout;
	int num_pads;
	int i_pads[MAX_PDM_PADS];
	struct pdm_cic_cfg cic;
	struct pdm_fir_cfg fir;
	bool running;
};

static struct strobe_filter {
	int clock;
	int rate[3];
	int snd_rates;
} strobe_filter[] = {
	{
		.clock = 1024000,
		/* LPF+HBF, LPF+HBFx2, LPF+HBFx3 */
		.rate = { 32000, 16000,  8000 },
		.snd_rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_8000,
	},
	{
		.clock = 2048000,
		 /* LPF+HBF, LPF+HBFx2, LPF+HBFx3 */
		.rate = { 64000, 32000, 16000 },
		.snd_rates = SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_16000,
	},
};

static unsigned int lpf_coef[] = {
	0x0000FFD9, 0x00000023, 0x000000C1, 0x000001BF,
	0x000002BA, 0x0000030F, 0x00000236, 0x0000002D,
	0x0000FDBB, 0x0000FC39, 0x0000FCDB, 0x0000FFD3,
	0x000003D7, 0x0000067F, 0x00000585, 0x0000004D,
	0x0000F8DE, 0x0000F37B, 0x0000F4D6, 0x0000FF98,
	0x00001285, 0x00002860, 0x000039CD, 0x00004071,
	0x000039CD, 0x00002860, 0x00001285, 0x0000FF98,
	0x0000F4D6, 0x0000F37B, 0x0000F8DE, 0x0000004D,
	0x00000585, 0x0000067F, 0x000003D7, 0x0000FFD3,
	0x0000FCDB, 0x0000FC39, 0x0000FDBB, 0x0000002D,
	0x00000236, 0x0000030F, 0x000002BA, 0x000001BF,
	0x000000C1, 0x00000023, 0x0000FFD9
};

static unsigned int hbf_coef[] = {
	0x0000FFFA, 0x00000000, 0x00000013, 0x00000000,
	0x0000FFD4, 0x00000000, 0x00000059, 0x00000000,
	0x0000FF5D, 0x00000000, 0x00000116, 0x00000000,
	0x0000FE3C, 0x00000000, 0x000002C7, 0x00000000,
	0x0000FBA7, 0x00000000, 0x00000708, 0x00000000,
	0x0000F31E, 0x00000000, 0x00002882, 0x00004000,
	0x00002882, 0x00000000, 0x0000F31E, 0x00000000,
	0x00000708, 0x00000000, 0x0000FBA7, 0x00000000,
	0x000002C7, 0x00000000, 0x0000FE3C, 0x00000000,
	0x00000116, 0x00000000, 0x0000FF5D, 0x00000000,
	0x00000059, 0x00000000, 0x0000FFD4, 0x00000000,
	0x00000013, 0x00000000, 0x0000FFFA
};

static struct pdm_coeff_cfg fir_coeff[] = {
	{
		.hbf = false,
		.buffer = lpf_coef,
		.size = ARRAY_SIZE(lpf_coef),
	},
	{
		.hbf = true,
		.buffer = hbf_coef,
		.size = ARRAY_SIZE(hbf_coef),
	},
};

#define	BUFFER_BYTES_MAX	(64 * 1024 * 2)
#define	PERIOD_BYTES_MAX	(4096)
#define	PERIOD_BYTES_MIN	(64) /* align 64bit * 8 burst */

#define PDM_SND_SOC_RATES ( \
		SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_96000 \
		)

static struct snd_pcm_hardware nx_pdm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 4,
	.buffer_bytes_max = BUFFER_BYTES_MAX,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = PERIOD_BYTES_MAX,
	.periods_min = 2,
	.periods_max = 64,
	.fifo_size = 32,
};

struct nx_pdm_runtime_data {
	struct nx_pdm_data *pdm;
	struct device *dev;
	unsigned int pos;
	int dma_bank;
	int period_msec;
	s64 msec;
};

#define	substream_to_rtd(s) \
	((struct snd_soc_pcm_runtime *)s->private_data)
#define	substream_to_prtd(s)	(s->runtime->private_data)

static irqreturn_t nx_pdm_pcm_irq_handler(int irq, void *arg)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nx_pdm_runtime_data *prtd = substream_to_prtd(substream);
	struct pdm_reg *reg = prtd->pdm->base;
	size_t period_bytes = snd_pcm_lib_period_bytes(substream);
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	unsigned int dma_offs;
	s64 duration, msec;
	u32 val;

	val = readl(&reg->intr);
	if (!(val & IRQ_DMA_PEND)) {
		writel(val | IRQ_MASK_PEND, &reg->intr);
		return IRQ_HANDLED;
	}

	prtd->pos += period_bytes;

	if (prtd->pos >= buffer_bytes)
		prtd->pos = 0;

	/* next dma address */
	dma_offs = prtd->pos + period_bytes;
	if (dma_offs >= buffer_bytes)
		dma_offs = 0;

	msec = ktime_to_ms(ktime_get());
	duration = msec - prtd->msec;
	if (prtd->period_msec + 10 < duration)
		dev_err(prtd->dev, "Timeout %lld:%dms\n",
			duration, prtd->period_msec);

	prtd->msec = msec;

	dev_dbg(prtd->dev,
		"DMA 0x%x, pos:%6d, dma:%6d,%d, addr:0x%x\n",
		runtime->dma_addr, prtd->pos, dma_offs, prtd->dma_bank,
		runtime->dma_addr + dma_offs);

	/* update dma address */
	writel(val | IRQ_DMA_PEND, &reg->intr);

	if (!prtd->dma_bank)
		writel(runtime->dma_addr + dma_offs, &reg->dma_addr0);
	else
		writel(runtime->dma_addr + dma_offs, &reg->dma_addr1);

	prtd->dma_bank = !prtd->dma_bank;

	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static int nx_pdm_pcm_request(struct snd_pcm_substream *substream)
{
	struct nx_pdm_runtime_data *prtd = substream_to_prtd(substream);
	unsigned long flags = IRQF_SHARED;
	int err;

	prtd->pdm = snd_soc_dai_get_dma_data(
				substream_to_rtd(substream)->cpu_dai,
				substream);

	err = request_irq(prtd->pdm->irq, nx_pdm_pcm_irq_handler,
				flags, dev_name(prtd->dev), substream);
	if (err) {
		dev_err(prtd->dev, "Failed, to request for pcm IRQ#%u: %d\n",
			prtd->pdm->irq, err);
		return err;
	}

	return 0;
}

static int nx_pdm_pcm_submit(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nx_pdm_runtime_data *prtd = substream_to_prtd(substream);
	struct pdm_reg *reg = prtd->pdm->base;
	dma_addr_t dma_addr = runtime->dma_addr;
	size_t period_bytes = snd_pcm_lib_period_bytes(substream);
	int dma_size = ALIGN(period_bytes, PERIOD_BYTES_MIN);

	if (period_bytes != ALIGN(period_bytes, PERIOD_BYTES_MIN)) {
		dev_err(prtd->dev,
			"Error, period bytes %d is not aligned %d\n",
			period_bytes, PERIOD_BYTES_MIN);
		return -EINVAL;
	}

	prtd->dma_bank = 0;
	prtd->pos = 0;
	prtd->msec = ktime_to_ms(ktime_get());
	prtd->period_msec = 1000 / ((runtime->rate * runtime->frame_bits/8) /
			snd_pcm_lib_period_bytes(substream));

	writel(dma_size / MEM_BURST_SIZE, &reg->dma_size);
	writel(dma_addr, &reg->dma_addr0);
	writel(dma_addr + period_bytes, &reg->dma_addr1);

	dev_dbg(prtd->dev, "DMA 0x%x, 0x%x\n", dma_addr, dma_addr + dma_size);
	dev_dbg(prtd->dev, "sample rate:%6d, bits:%2d, frame bytes:%2d\n",
		runtime->rate, runtime->sample_bits, runtime->frame_bits / 8);
	dev_dbg(prtd->dev, "buffer size:%6ld, byte:%6d\n",
		runtime->buffer_size, snd_pcm_lib_buffer_bytes(substream));
	dev_dbg(prtd->dev, "period size:%6ld, byte:%6d, periods:%2d\n",
		runtime->period_size, snd_pcm_lib_period_bytes(substream),
		runtime->periods);

	return 0;
}

static int nx_pdm_pcm_open(struct snd_pcm_substream *substream)
{
	static struct snd_pcm_hardware *pcm_hw = &nx_pdm_hardware;
	struct nx_pdm_runtime_data *prtd;
	int ret;

	ret = snd_pcm_hw_constraint_integer(
			substream->runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	substream->runtime->private_data = prtd;
	prtd->dev = substream->pcm->card->dev;

	ret = nx_pdm_pcm_request(substream);
	if (ret)
		return -EINVAL;

	/* set substream->runtime->hw */
	return snd_soc_set_runtime_hwparams(substream, pcm_hw);
}

static int nx_pdm_pcm_close(struct snd_pcm_substream *substream)
{
	struct nx_pdm_runtime_data *prtd = substream_to_prtd(substream);

	if (prtd)
		free_irq(prtd->pdm->irq, substream);

	kfree(substream_to_prtd(substream));

	return 0;
}

static int nx_pdm_pcm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	return 0;
}

static int nx_pdm_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int nx_pdm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	if (cmd == SNDRV_PCM_TRIGGER_START)
		return nx_pdm_pcm_submit(substream);

	return 0;
}

static snd_pcm_uframes_t nx_pdm_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct nx_pdm_runtime_data *prtd = substream_to_prtd(substream);

	return bytes_to_frames(substream->runtime, prtd->pos);
}

static struct snd_pcm_ops nx_pdm_pcm_ops = {
	.open = nx_pdm_pcm_open,
	.close = nx_pdm_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = nx_pdm_pcm_hw_params,
	.hw_free = nx_pdm_pcm_hw_free,
	.trigger = nx_pdm_pcm_trigger,
	.pointer = nx_pdm_pcm_pointer,
	.mmap = snd_pcm_lib_default_mmap,
};

static int nx_pdm_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

	return snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_DEV,
				NULL, BUFFER_BYTES_MAX,	BUFFER_BYTES_MAX);
}

static void nx_pdm_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

	snd_pcm_lib_preallocate_free(substream);
}

static struct snd_soc_platform_driver nx_pdm_platform = {
	.ops = &nx_pdm_pcm_ops,
	.pcm_new = nx_pdm_pcm_new,
	.pcm_free = nx_pdm_pcm_free,
};

static inline void nx_pdm_dump_reg(struct pdm_reg *reg)
{
	struct pdm_fir_reg *firs = &reg->firs[0];
	int num = readl(&reg->fir_num);
	int i;

	pr_debug("ctrl	: 0x%08x\n", readl(&reg->ctrl));
	pr_debug("clk	: 0x%08x\n", readl(&reg->pdm_clk));
	pr_debug("lock	: 0x%08x\n", readl(&reg->pdm_lock));
	pr_debug("tout	: 0x%08x\n", readl(&reg->pdm_tout));
	pr_debug("src	: 0x%08x\n", readl(&reg->pdm_src));
	pr_debug("c_cfg	: 0x%08x\n", readl(&reg->cic_cfg));
	pr_debug("pos_v	: 0x%08x\n", readl(&reg->cic_pos_v));
	pr_debug("neg_v	: 0x%08x\n", readl(&reg->cic_neg_v));
	pr_debug("size	: 0x%08x\n", readl(&reg->dma_size));
	pr_debug("dma0	: 0x%08x\n", readl(&reg->dma_addr0));
	pr_debug("dma1	: 0x%08x\n", readl(&reg->dma_addr1));
	pr_debug("firnr	: 0x%08x\n", readl(&reg->fir_num));
	pr_debug("c_off	: 0x%08x\n", readl(&reg->cic_offs));
	pr_debug("c_end	: 0x%08x\n", readl(&reg->cic_end));
	pr_debug("intr	: 0x%08x\n", readl(&reg->intr));
	pr_debug("sleep	: 0x%08x\n", readl(&reg->mem_sleep));

	for (i = 0; i < (num + 1); i++, firs++) {
		pr_debug("\n");
		pr_debug("rds	: 0x%08x\n", readl(&firs->rd_offs));
		pr_debug("rde	: 0x%08x\n", readl(&firs->rd_end));
		pr_debug("wrs	: 0x%08x\n", readl(&firs->wr_offs));
		pr_debug("wre	: 0x%08x\n", readl(&firs->wr_end));
		pr_debug("ces	: 0x%08x\n", readl(&firs->coef_offs));
		pr_debug("cen	: 0x%08x\n", readl(&firs->coef_num));
	}
}

static void nx_pdm_check_coeff(void *mem, void *buffer, int size)
{
	u32 *s = (u32 *)buffer;
	u32 *d = (u32 *)mem;
	int i;

	for (i = 0; i < size; i++) {
		if (d[i] != s[i])
			pr_err("[%4d] Err %p:0x%04x / %p:0x%04x\n",
				i, &d[i], d[i], &s[i], s[i]);
	}
}

static int nx_pdm_strobe_rate_step(int strobe, int rate)
{
	struct strobe_filter *f = strobe_filter;
	int i, n;

	for (i = 0; i < ARRAY_SIZE(strobe_filter); i++, f++) {
		if (strobe != f->clock)
			continue;

		for (n = 0; i < 3; n++) {
			if (rate == f->rate[n])
				return n + 2; /* add index + LPF */
		}
	}

	return -EINVAL;
}

static int nx_pdm_strobe_rate_range(int strobe,
				    int *min, int *max, int *snd_rates)
{
	struct strobe_filter *f = strobe_filter;
	int i;

	for (i = 0; i < ARRAY_SIZE(strobe_filter); i++, f++) {
		if (strobe == f->clock) {
			*max = f->rate[0];
			*min = f->rate[2];
			*snd_rates = f->snd_rates;
			return 0;
		}
	}

	return -EINVAL;
}

static int nx_pdm_strobe_check(int strobe)
{
	struct strobe_filter *f = strobe_filter;
	int i;

	for (i = 0; i < ARRAY_SIZE(strobe_filter); i++, f++) {
		if (strobe == f->clock)
			return 0;
	}

	return -EINVAL;
}

static void nx_pdm_start(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	u32 intr = readl(&reg->intr) & ~IRQ_DMA_MASK;
	u32 clk = readl(&reg->pdm_clk);

	writel(intr | IRQ_MASK_PEND | IRQ_DMA_ENB, &reg->intr);
	writel(clk | PDM_CLK_ENB, &reg->pdm_clk);
	writel(1, &reg->ctrl);

	nx_pdm_dump_reg(reg);
}

static void nx_pdm_stop(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	long loops = 100 * 1000;
	u32 intr = readl(&reg->intr) & ~IRQ_DMA_MASK;
	u32 clk = readl(&reg->pdm_clk);

	writel(intr & ~IRQ_DMA_ENB, &reg->intr);
	writel(0, &reg->ctrl);

	while (--loops) {
		if (!(1 & readl(&reg->ctrl)))
			break;
		udelay(1);
	}

	writel(clk & ~PDM_CLK_ENB, &reg->pdm_clk);
}

static void nx_pdm_sync_clk(struct nx_pdm_data *pdm)
{
	int val = 0;

	if (IS_ERR(pdm->syscon))
		return;

	val = pdm->ref_iis & 0x3;

	if (pdm->sync_type == pdm_sync_mclk)
		val |= 1 << 2;

	/*
	 * [2]   : clock source: 0=cmu, 1=mclk
	 * [1:0] : i2s source: 0=0, 1=1, 2=2, 3=3
	 */
	regmap_update_bits(pdm->syscon, 0x300, 0x7, val);
}

static int nx_pdm_sync_timeout(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	int strobe = pdm->strobe_hz;
	u32 val = 0;

	if (pdm->sync_type != pdm_sync_lrck) {
		writel(val, &reg->pdm_lock);
		return 0;
	}

	if (pdm->sync_type == pdm_sync_lrck) {
		unsigned int bclk = clk_get_rate(pdm->clk_axi);
		int count = strobe / 100; /* check per 10ms */

		val = PDM_LOCK_CNT(count) | PDM_LOCK_LRCK;

		writel(val, &reg->pdm_lock);

		pdm->timeout = bclk / (pdm->ref_sample_rate / 2);
		writel(pdm->timeout, &reg->pdm_tout);
	}

	dev_dbg(pdm->dev,
		"sync: iis.%d, lock:%s, tout:%d\n",
		pdm->ref_iis, pdm->sync_type == pdm_sync_mclk ?
		"MCLK" : "LRCLK", pdm->timeout);

	return 0;
}

static int nx_pdm_sync_setup(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	bool sys_clk = pdm->sync_type != pdm_sync_mclk ? true : false;
	long long sys_freq = pdm->sys_freq;
	long long freq, strobe;
	int div, mod;
	int dh = 0, dl = 0, ret;
	u32 val = 0;

	nx_pdm_sync_clk(pdm);

	ret = nx_pdm_sync_timeout(pdm);
	if (ret)
		return ret;

	if (sys_clk) {
		clk_set_rate(pdm->clk, pdm->sys_freq);
		sys_freq = clk_get_rate(pdm->clk);
	}

	clk_prepare_enable(pdm->clk);

	freq = sys_clk ? sys_freq : pdm->ref_freq;
	strobe = pdm->strobe_hz;

	div = div_s64(freq, strobe);
	mod = div_s64(freq * 1000, strobe);
	mod %= 1000;

	if (div > 1) {
		div += roundup(mod, 500)/1000;
		dh = div / 2;
		dl = div - dh;
	}

	val |= PDM_CLK_DIV_L(dl) | PDM_CLK_DIV_H(dh);
	writel(val, &reg->pdm_clk);

	if (sys_freq < 200000000)
		dev_warn(pdm->dev,
			"system clock more than 200Mhz (%lld Mhz)...\n",
			div_s64(freq, 1000*1000));

	dev_dbg(pdm->dev,
		"freq: %s, %lld/%d, storbe %lld, div:%d, mod:%d, [%d:%d]\n",
		sys_clk ? "SYS" : "MCLK", freq, pdm->sys_freq,
		strobe, div, mod, dh, dl);

	return 0;
}

static int nx_pdm_cic_setup(struct nx_pdm_data *pdm)
{
	struct pdm_cic_cfg *cic = &pdm->cic;
	struct pdm_reg *reg = pdm->base;
	int ch;
	u32 val;

	switch (pdm->channel) {
	case 2:
		ch = 1;
		break;
	case 4:
		ch = 3;
		break;
	default:
		dev_err(pdm->dev,
			"Failed, Invalid channel %d (2/4)\n",
			pdm->channel);
		return -EINVAL;
	}

	val = PDM_CIC_CH(ch) | PDM_CIC_COMB(cic->comb_stages) |
		PDM_CIC_INTG(cic->integrators) |
		PDM_CIC_SHIFT(cic->shfit) | PDM_CIC_DIV(cic->divide);

	dev_dbg(pdm->dev,
		"CIC : ch:%d, comb:%d, integ:%d, shift:%d, div:%d-0x%x\n",
		ch, cic->comb_stages, cic->integrators,
		cic->shfit, cic->divide, val);

	writel(val, &reg->cic_cfg);
	writel(cic->posv, &reg->cic_pos_v);
	writel(cic->negv, &reg->cic_neg_v);

	return 0;
}

static int nx_pdm_pad_setup(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	int *p = pdm->i_pads, i;
	u32 val;

	for (i = 0, val = 0; i < pdm->num_pads; i++)
		val |= PDM_SRC_PAD(p[i], i);

	dev_dbg(pdm->dev, "pad : %d,%d,%d,%d  - 0x%x\n",
		p[0], p[1], p[2], p[3], val);

	writel(val, &reg->pdm_src);

	return 0;
}

static int nx_pdm_fir_setup(struct nx_pdm_data *pdm, unsigned int sample_rate)
{
	struct pdm_reg *reg = pdm->base;
	struct pdm_fir_cfg *fir = &pdm->fir;
	void __iomem *mem = pdm->mem;
	int coefs = fir->num_coefs;
	int ws = 0, we = MEM_CIC_SIZE;
	int rs, re, cs, ce;
	int step, i;

	step = nx_pdm_strobe_rate_step(pdm->strobe_hz, sample_rate);
	if (step < 0) {
		dev_err(pdm->dev,
			"Not support strobe %d, sample rate %d\n",
			pdm->strobe_hz, pdm->sample_rate);
		return -EINVAL;
	}

	memset(mem, 0, MEM_SIZE);

	writel(0, &reg->cic_offs);
	writel(_word_(MEM_CIC_SIZE), &reg->cic_end);
	writel(step-1, &reg->fir_num); /* set hbf's step */

	dev_dbg(pdm->dev,
		"cic out: %p:0x0~0x%08x, strobe %dhz -> %dhz [%d]\n",
		mem, MEM_CIC_SIZE, pdm->strobe_hz, sample_rate, step);

	for (i = 0; i < step; i++) {
		struct pdm_fir_reg *firs = &reg->firs[i];
		struct pdm_coeff_cfg *coef;
		bool hbf, out = false;
		int sz;

		if (i < coefs)
			coef = &fir->coef[i];
		else
			coef = &fir->coef[coefs - 1];

		sz = ALIGN(coef->size, 4);
		rs = ws, re = we;
		cs = re + 4, ce = cs + (sz * 4);
		ws = ce, we = ws + MEM_CIC_SIZE;
		hbf = coef->hbf;

		if (i == step - 1) {
			out = true;
			ws = we = 0;
		}

		dev_dbg(pdm->dev, "step.%d: %s:%s, %p %2d (%d)\n",
			i, out ? "out" : "mem", hbf ? "HBF" : "LPF",
			coef->buffer, coef->size, sz);
		dev_dbg(pdm->dev,
			"[r:0x%04x~0x%04x, c:0x%04x~0x%04x, w:0x%04x~0x%04x]\n",
			rs, re, cs, ce, ws, we);

		/* RD OFFSET */
		writel(_word_(rs), &firs->rd_offs);
		writel(_word_(re), &firs->rd_end);

		/* Load coeff_ptr */
		memcpy(mem + cs, coef->buffer, (coef->size * 4));

		nx_pdm_check_coeff(mem + cs, coef->buffer, coef->size);

		/* WR OFFSET */
		writel(_word_(ws), &firs->wr_offs);
		writel(_word_(we), &firs->wr_end);

		/* set coeff and param */
		writel(_word_(cs), &firs->coef_offs);
		writel((coef->size - 1) |
			PDM_PARAMS_HBP(hbf) | PDM_PARAMS_OUT(out),
			&firs->coef_num);
	}

	pdm->sample_rate = sample_rate;

	return 0;
}

static int nx_pdm_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);

	return nx_pdm_fir_setup(pdm, params_rate(params));
}

static int nx_pdm_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	clk_prepare_enable(pdm->clk_axi);

	ret = nx_pdm_sync_setup(pdm);
	if (ret)
		return ret;

	ret = nx_pdm_cic_setup(pdm);
	if (ret)
		return ret;

	nx_pdm_pad_setup(pdm);

	pdm->running = true;

	snd_soc_dai_set_dma_data(dai, substream, pdm);

	return 0;
}

static void nx_pdm_shutdown(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);

	if (__clk_is_enabled(pdm->clk))
		clk_disable_unprepare(pdm->clk);

	if (__clk_is_enabled(pdm->clk_axi))
		clk_disable_unprepare(pdm->clk_axi);

	pdm->running = false;
}

static int nx_pdm_trigger(struct snd_pcm_substream *substream,
			  int cmd, struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_START:
		nx_pdm_start(pdm);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		nx_pdm_stop(pdm);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops nx_pdm_dai_ops = {
	.startup = nx_pdm_startup,
	.shutdown = nx_pdm_shutdown,
	.trigger = nx_pdm_trigger,
	.hw_params = nx_pdm_hw_params,
};

static int nx_pdm_suspend(struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);

	if (!pdm->running)
		return 0;

	if (__clk_is_enabled(pdm->clk))
		clk_disable_unprepare(pdm->clk);

	clk_disable_unprepare(pdm->clk_axi);

	return 0;
}

static int nx_pdm_resume(struct snd_soc_dai *dai)
{
	struct nx_pdm_data *pdm = snd_soc_dai_get_drvdata(dai);

	if (!pdm->running)
		return 0;

	clk_prepare_enable(pdm->clk_axi);

	if (!__clk_is_enabled(pdm->clk))
		clk_prepare_enable(pdm->clk);

	nx_pdm_fir_setup(pdm, pdm->sample_rate);
	nx_pdm_sync_setup(pdm);
	nx_pdm_cic_setup(pdm);
	nx_pdm_pad_setup(pdm);

	return 0;
}

static struct snd_soc_dai_driver nx_pdm_dai_driver = {
	.suspend = nx_pdm_suspend,
	.resume = nx_pdm_resume,
	.ops = &nx_pdm_dai_ops,
	.capture = {
		.stream_name = "Capture",
		.channels_min = 4,
		.channels_max = 4,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rates = PDM_SND_SOC_RATES,
	},
};

static const struct snd_soc_component_driver nx_pdm_component = {
	.name = "nexell-pdm",
};

static ssize_t select_pads_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nx_pdm_data *pdm = dev_get_drvdata(dev);
	int *pad = pdm->i_pads;

	return snprintf(buf, PAGE_SIZE,
			"%d,%d,%d,%d\n", pad[0], pad[1], pad[2], pad[3]);
}

static ssize_t select_pads_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct nx_pdm_data *pdm = dev_get_drvdata(dev);
	int *pad = pdm->i_pads;
	char *c, *s = (char *)buf;
	int num;
	int i = 0, ret;

	while (s) {
		c = strchr(s, ',');
		if (c)
			*c++ = '\0';

		ret = kstrtoint(s, 10, &num);
		if (ret != 0)
			return ret;

		pad[i++] = num;
		if (i > MAX_PDM_PADS-1)
			break;
		s = c;
	}

	return size;
}

static DEVICE_ATTR(select_pads, 0644, select_pads_show, select_pads_store);

static const struct attribute *pdm_attrs[] = {
	&dev_attr_select_pads.attr,
	NULL,
};

static const struct attribute_group pdm_attr_group = {
	.attrs = (struct attribute **)pdm_attrs,
};

static int nx_pdm_dai_init(struct nx_pdm_data *pdm)
{
	struct snd_soc_pcm_stream *capt = &nx_pdm_dai_driver.capture;
	int min, max, snd_rates;
	int ret;

	ret = nx_pdm_strobe_rate_range(pdm->strobe_hz,
					&min, &max, &snd_rates);
	if (ret)
		return ret;

	capt->rate_min = min;
	capt->rate_max = max;
	capt->rates = snd_rates;

	if (pdm->sample_rate) {
		capt->rate_min = pdm->sample_rate;
		capt->rate_max = pdm->sample_rate;
	}

	return 0;
}

static irqreturn_t nx_pdm_irq_handler(int irq, void *arg)
{
	struct nx_pdm_data *pdm = arg;
	struct pdm_reg *reg = pdm->base;
	u32 val = readl(&reg->intr);

	if ((val & IRQ_TOUT_PEND) && (val & IRQ_TOUT_ENB)) {
		val = readl(&reg->pdm_lock);
		val &= ~(PDM_LOCK_1ST_ENB | PDM_LOCK_LRCK);
		writel(val, &reg->pdm_lock);
		writel(val | IRQ_TOUT_PEND, &reg->intr);
	}

	if (val & IRQ_LDET_PEND)
		writel(val | IRQ_LDET_PEND, &reg->intr);

	return IRQ_HANDLED;
}

static int nx_pdm_irq_request(struct nx_pdm_data *pdm)
{
	struct pdm_reg *reg = pdm->base;
	unsigned long flags = IRQF_SHARED;
	int err;

	err = devm_request_irq(pdm->dev, pdm->irq,
			nx_pdm_irq_handler, flags, dev_name(pdm->dev), pdm);
	if (err) {
		dev_err(pdm->dev,
			"Failed, to request IRQ#%u: %d\n", pdm->irq, err);
		return -EINVAL;
	}

	writel(readl(&reg->intr) | IRQ_TOUT_ENB, &reg->intr);

	return 0;
}

static int nx_pdm_parse_res(struct platform_device *pdev,
			    struct nx_pdm_data *pdm)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res, *irq;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pdm");
	if (!res)
		return -ENOENT;

	pdm->addr = res->start;
	pdm->base = devm_ioremap_resource(&pdev->dev, res);

	dev_dbg(&pdev->dev, "PDM  mapped %08lx -> %p :%08lx\n",
		(unsigned long)res->start, pdm->base,
		(unsigned long)(res->end - res->start));

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!res)
		return -ENOMEM;

	pdm->mem = devm_ioremap_resource(&pdev->dev, res);
	if (!pdm->mem)
		return -ENOMEM;

	dev_dbg(&pdev->dev, "SRAM mapped %08lx -> %p :%08lx\n",
		(unsigned long)res->start, pdm->mem,
		(unsigned long)(res->end - res->start));

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq)
		return -ENOENT;

	pdm->irq = irq->start;

	pdm->clk_axi = of_clk_get_by_name(node, "axi");
	if (IS_ERR(pdm->clk_axi)) {
		dev_err(&pdev->dev, "Failed no clock found 'axi'\n");
		return PTR_ERR(pdm->clk_axi);
	}

	pdm->clk = of_clk_get_by_name(node, "core");
	if (IS_ERR(pdm->clk)) {
		dev_err(pdm->dev, "Failed no clock found 'core'\n");
		return PTR_ERR(pdm->clk);
	}

	pdm->syscon = syscon_regmap_lookup_by_phandle(node, "syscon");
	if (IS_ERR(pdm->syscon)) {
		dev_err(pdm->dev, "Failed no syscon found !!!\n");
		return PTR_ERR(pdm->syscon);
	}

	return 0;
}

static int nx_pdm_parse_of_coef(struct device *dev, struct pdm_fir_cfg *fir)
{
	struct device_node *node = dev->of_node;
	const __be32 *list;
	int size, i;

	/* load coefficient */
	list = of_get_property(node, "filter-coeff", &size);
	if (!list || !size)
		return -EINVAL;

	size = (list + size / sizeof(*list)) - list;
	fir->num_coefs = size;

	fir->coef = kcalloc(size, sizeof(*fir->coef), GFP_KERNEL);
	if (!fir->coef)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		struct pdm_coeff_cfg *coef = &fir->coef[i];
		struct device_node *np;
		int count;

		np = of_parse_phandle(node, "filter-coeff", i);
		if (!np) {
			dev_err(dev, "can't get 'filter-coeff:%d' node\n", i);
			return -ENOMEM;
		}

		count = of_property_count_u32_elems(np, "coeff");
		if (count > 0) {
			coef->buffer = kcalloc(count, sizeof(u32), GFP_KERNEL);
			if (!coef->buffer) {
				kfree(fir->coef);
				return -ENOMEM;
			}
			of_property_read_u32_array(
					np, "coeff", coef->buffer, count);
			coef->hbf = of_property_read_bool(np, "hbp-filter");
			coef->size = count;
			dev_dbg(dev, "Load: DT coeff.%d to %p size %d\n",
				i, coef->buffer, count);
		}
	}

	return 0;
}

static int nx_pdm_parse_coef(struct device *dev, struct pdm_fir_cfg *fir)
{
	int size = ARRAY_SIZE(fir_coeff); /* LPF, HBF */
	int i;

	fir->coef = kcalloc(size, sizeof(*fir->coef), GFP_KERNEL);
	if (!fir->coef)
		return -ENOMEM;

	fir->num_coefs = size;

	/* load from property */
	for (i = 0; i < size; i++) {
		struct pdm_coeff_cfg *coef = &fir->coef[i];
		struct pdm_coeff_cfg *p;

		p = &fir_coeff[i];
		coef->buffer = kcalloc(p->size, sizeof(u32), GFP_KERNEL);
		if (!coef->buffer) {
			kfree(fir->coef);
			return -ENOMEM;
		}

		memcpy(coef->buffer, p->buffer, p->size * sizeof(u32));

		coef->hbf = p->hbf;
		coef->size = p->size;
		dev_dbg(dev, "Load: static coeff.%d to %p size %d\n",
			i, coef->buffer, coef->size);
	}

	return 0;
}

static int nx_pdm_parse_of_dt(struct platform_device *pdev,
			      struct nx_pdm_data *pdm)
{
	struct device_node *node = pdev->dev.of_node;
	struct pdm_fir_cfg *fir = &pdm->fir;
	struct pdm_cic_cfg *cic = &pdm->cic;
	const __be32 *list;
	int count, step, ret;

	/* strobe clock and sample rate */
	of_property_read_u32(node, "ref-iis", &pdm->ref_iis);
	/* lock with iis mclk */
	of_property_read_u32(node, "ref-clk-frequency", &pdm->ref_freq);
	/* lock with iis lrck */
	of_property_read_u32(node, "ref-sample-rate", &pdm->ref_sample_rate);
	of_property_read_u32(node, "sample-rate", &pdm->sample_rate);
	of_property_read_u32(node, "strobe-hz", &pdm->strobe_hz);

	if (of_property_read_bool(node, "assigned-core-freqeuncy"))
		pdm->sys_freq = clk_get_rate(pdm->clk);

	step = nx_pdm_strobe_check(pdm->strobe_hz);
	if (step < 0) {
		dev_err(&pdev->dev,
			"Not support strobe %dhz clock\n", pdm->strobe_hz);
		return -EINVAL;
	}

	if (pdm->ref_freq)
		pdm->sync_type = pdm_sync_mclk;

	if (pdm->ref_sample_rate)
		pdm->sync_type = pdm_sync_lrck;

	if (pdm->sync_type != pdm_sync_none &&
		(pdm->ref_iis == -1 || pdm->ref_iis > MAX_REF_CH)) {
		dev_err(&pdev->dev,
			"Invalid reference iis.%d max:%d\n",
			pdm->ref_iis, MAX_REF_CH);
		return -EINVAL;
	}

	/* channel num_pads */
	count = of_property_count_u32_elems(node, "select-pads");
	if (count > 0 && count < MAX_PDM_PADS + 1) {
		of_property_read_u32_array(
				node, "select-pads", pdm->i_pads, count);
		pdm->num_pads = count;
	}

	/* load coefficient */
	list = of_get_property(node, "filter-coeff", NULL);
	if (list)
		ret = nx_pdm_parse_of_coef(&pdev->dev, fir);
	else
		ret = nx_pdm_parse_coef(&pdev->dev, fir);

	if (ret) {
		dev_err(&pdev->dev, "Can't get 'filter-coeff' !!!\n");
		return -EINVAL;
	}

	/* cic config */
	of_property_read_u32(node, "cic-combstage-tap", &cic->comb_stages);
	of_property_read_u32(node, "cic-integrator-tap", &cic->integrators);
	of_property_read_u32(node, "cic-shift", &cic->shfit);
	of_property_read_u32(node, "cic-divide", &cic->divide);
	of_property_read_u32(node, "cic-pos-value", &cic->posv);
	of_property_read_u32(node, "cic-neg-value", &cic->negv);

	dev_info(&pdev->dev, "PDM: sysclk:%d: storbe:%d, sample rate:%d\n",
		pdm->sync_type == pdm_sync_mclk ? pdm->ref_freq :
		pdm->sys_freq, pdm->strobe_hz, pdm->sample_rate);
	dev_info(&pdev->dev, "PDM: ref (iis.%d, sync:%s, rate:%d, freq:%d)\n",
		pdm->ref_iis, pdm->sync_type == pdm_sync_mclk ? "MCLK" :
		pdm->sync_type == pdm_sync_lrck ? "LRCLK" : "None",
		pdm->ref_sample_rate, pdm->ref_freq);
	dev_info(&pdev->dev, "PDM: filter:%d taps:%d,%d, sh:%d, div:%d, pv:%d, nv:%d\n",
		fir->num_coefs, cic->comb_stages, cic->integrators,
		cic->shfit, cic->divide, cic->posv, cic->negv);

	return 0;
}

static struct nx_pdm_data *nx_pdm_allocate(struct platform_device *pdev)
{
	struct nx_pdm_data *pdm;
	int i, ret;

	ret = sysfs_create_group(&pdev->dev.kobj, &pdm_attr_group);
	if (ret)
		return NULL;

	pdm = kzalloc(sizeof(*pdm), GFP_KERNEL);
	if (!pdm)
		return NULL;

	pdm->dev = &pdev->dev;
	pdm->ref_iis = -1;
	pdm->sys_freq = 12288000 * 10 * 2; /* 12.288*10*2, more than 200Mhz */
	pdm->sample_rate = 0;
	pdm->strobe_hz = 2048000;
	pdm->channel = 4;
	pdm->num_pads = MAX_PDM_PADS;
	pdm->sync_type = pdm_sync_none;

	for (i = 0; i < pdm->num_pads; i++)
		pdm->i_pads[i] = i;

	pdm->cic.comb_stages = 4;
	pdm->cic.integrators = 4;
	pdm->cic.shfit = 3;
	pdm->cic.divide = 15;
	pdm->cic.posv = 1;
	pdm->cic.negv = -1;

	return pdm;
}

static int nx_pdm_probe(struct platform_device *pdev)
{
	struct nx_pdm_data *pdm;
	int ret;

	pdm = nx_pdm_allocate(pdev);
	if (!pdm)
		return -ENOMEM;

	ret = nx_pdm_parse_res(pdev, pdm);
	if (ret)
		return ret;

	ret = nx_pdm_parse_of_dt(pdev, pdm);
	if (ret)
		goto err_out;

	ret = nx_pdm_irq_request(pdm);
	if (ret)
		goto err_out;

	nx_pdm_dai_init(pdm);

	ret = devm_snd_soc_register_component(&pdev->dev,
				&nx_pdm_component, &nx_pdm_dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed, pdm snd register component !\n");
		goto err_out;
	}

	ret = devm_snd_soc_register_platform(&pdev->dev, &nx_pdm_platform);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed, pdm snd register platform !\n");
		goto err_out;
	}

	dev_set_drvdata(&pdev->dev, pdm);

	return 0;

err_out:
	kfree(pdm->fir.coef);
	kfree(pdm);

	return ret;
}

static int nx_pdm_remove(struct platform_device *pdev)
{
	struct nx_pdm_data *pdm = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	clk_disable_unprepare(pdm->clk_axi);
	clk_put(pdm->clk_axi);
	clk_put(pdm->clk);

	kfree(pdm->fir.coef);
	kfree(pdm);

	return 0;
}

static const struct of_device_id of_pdm_match[] = {
	{ .compatible = "nexell,nxp3220-pdm" },
	{},
};

static struct platform_driver nx_pdm_driver = {
	.probe  = nx_pdm_probe,
	.remove = nx_pdm_remove,
	.driver = {
		.name	= "nexell-pdm",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_pdm_match),
	},
};
#ifdef CONFIG_DEFERRED_SOUND_PDM
static int __init nx_pdm_driver_init(void)
{
	return platform_driver_register(&nx_pdm_driver);
}
deferred_module_init(nx_pdm_driver_init)
#else
module_platform_driver(nx_pdm_driver);
#endif

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound PDM/PCM driver for Nexell sound");
MODULE_LICENSE("GPL");
