// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound PCM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "nexell-pcm.h"

#ifdef CONFIG_AMBA_PL08X
#include <linux/amba/pl08x.h>
#define FILTER_FN	pl08x_filter_id
#else
#define FILTER_FN	NULL
#endif

#define	BUFFER_BYTES_MAX		(64 * 1024 * 2)
#define	PERIOD_BYTES_MAX		(8192)

#define PCM_SND_SOC_FORMATS ( \
	SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8 | \
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_U32_LE)

static struct snd_pcm_hardware nx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_MMAP_VALID |
					SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= PCM_SND_SOC_FORMATS,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= 32,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= 2,
	.periods_max		= 64,
	.fifo_size		= 32,
};

#define	substream_to_rtd(s) \
	((struct snd_soc_pcm_runtime *)s->private_data)
#define	substream_to_prtd(s)	(s->runtime->private_data)

#define	dma_ch_name(s) \
	(s->stream == SNDRV_PCM_STREAM_PLAYBACK ? "tx":"rx")

#define PERIOD_TIME_US(r, s)	((1000000*1000) / ((r * 1000) / s))

#define SND_DMAENGINE_PCM_FLAG_NO_RESIDUE BIT(31)

static long preiod_time_us(struct snd_pcm_substream *substream)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	size_t period_size = substream->runtime->period_size;
	long rate = substream->runtime->rate;

	if (prtd->dma_param->clk_sample_rate)
		rate = prtd->dma_param->clk_sample_rate;

	return PERIOD_TIME_US(prtd->dma_param->clk_sample_rate, period_size);
}

static void nx_pcm_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);

	prtd->pos += snd_pcm_lib_period_bytes(substream);
	if (prtd->pos >= snd_pcm_lib_buffer_bytes(substream))
		prtd->pos = 0;

	snd_pcm_period_elapsed(substream);
}

static struct dma_chan *nx_pcm_dma_request(struct snd_pcm_substream *substream)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	dma_filter_fn filter_fn;
	void *filter_data;
	dma_cap_mask_t mask;
	const char *dma_name;

	prtd->dma_param = snd_soc_dai_get_dma_data(
				substream_to_rtd(substream)->cpu_dai,
				substream);

	filter_fn = FILTER_FN;
	filter_data = prtd->dma_param->filter_data;
	dma_name = prtd->dma_param->dma_name ? prtd->dma_param->dma_name :
			dma_ch_name(substream);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);

	return dma_request_slave_channel_compat(mask,
				filter_fn, filter_data,
				prtd->dma_param->dev, dma_name);
}

static void nx_pcm_dma_release(struct snd_pcm_substream *substream)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);

	if (prtd && prtd->dma_chan)
		dma_release_channel(prtd->dma_chan);
}

static int nx_pcm_dma_slave_config(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct nx_pcm_dma_param *dma_param = prtd->dma_param;
	struct dma_slave_config slave_config;

	memset(&slave_config, 0, sizeof(slave_config));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.direction = DMA_MEM_TO_DEV;
		slave_config.dst_addr = dma_param->addr;
	} else {
		slave_config.direction = DMA_DEV_TO_MEM;
		slave_config.src_addr = dma_param->addr;
	}

	slave_config.dst_addr_width = dma_param->buswidth;
	slave_config.src_addr_width = dma_param->buswidth;
	slave_config.src_maxburst = dma_param->maxburst / dma_param->buswidth;
	slave_config.dst_maxburst = dma_param->maxburst / dma_param->buswidth;
	slave_config.device_fc = false;

	dev_dbg(prtd->dev, "%s.%d:%s, addr:0x%x, busw:%d, maxburst:%d\n",
		substream_to_rtd(substream)->cpu_dai->name,
		prtd->dma_param->channel,
		snd_pcm_stream_str(substream),
		dma_param->addr, dma_param->buswidth, dma_param->maxburst);

	return dmaengine_slave_config(prtd->dma_chan, &slave_config);
}

static int nx_pcm_dma_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct dma_chan *chan = prtd->dma_chan;
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK;

	direction = snd_pcm_substream_to_dma_direction(substream);

	if (!runtime->no_period_wakeup)
		flags |= DMA_PREP_INTERRUPT;

	prtd->pos = 0;
	desc = dmaengine_prep_dma_cyclic(chan,
			runtime->dma_addr,
			snd_pcm_lib_buffer_bytes(substream),
			snd_pcm_lib_period_bytes(substream), direction, flags);
	if (!desc) {
		dev_err(prtd->dev, "Error: %s.%d dma prepare cyclic\n",
			substream_to_rtd(substream)->cpu_dai->name,
			prtd->dma_param->channel);
		return -ENOMEM;
	}

	desc->callback = nx_pcm_dma_complete;
	desc->callback_param = substream;
	prtd->cookie = dmaengine_submit(desc);

	dev_dbg(prtd->dev, "PCM: %s.%d\n",
		substream_to_rtd(substream)->cpu_dai->name,
		prtd->dma_param->channel);
	dev_dbg(prtd->dev, "sample rate:%6d (%6ld), bits:%2d, frame byte:%2d\n",
		runtime->rate, prtd->dma_param->clk_sample_rate,
		runtime->sample_bits, runtime->frame_bits / 8);
	dev_dbg(prtd->dev, "time per period: %3ld us\n",
		preiod_time_us(substream));
	dev_dbg(prtd->dev, "buffer size:%6ld, byte:%6d\n",
		runtime->buffer_size, snd_pcm_lib_buffer_bytes(substream));
	dev_dbg(prtd->dev, "period size:%6ld, byte:%6d, periods:%2d\n",
		runtime->period_size, snd_pcm_lib_period_bytes(substream),
		runtime->periods);

	return 0;
}
static bool nx_pcm_dma_can_report_residue(struct device *dev,
			struct dma_chan *chan)
{
	struct dma_slave_caps dma_caps;
	int ret;

	ret = dma_get_slave_caps(chan, &dma_caps);
	if (ret != 0) {
		dev_warn(dev, "Failed to get DMA channel capabilities\n");
		dev_warn(dev, "Falling back to period counting: %d\n", ret);
		return false;
	}

	if (dma_caps.residue_granularity == DMA_RESIDUE_GRANULARITY_DESCRIPTOR)
		return false;

	return true;
}

static snd_pcm_uframes_t nx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int buf_size;
	unsigned int pos = 0;

	if (prtd->flags & SND_DMAENGINE_PCM_FLAG_NO_RESIDUE)
		return bytes_to_frames(substream->runtime, prtd->pos);

	status = dmaengine_tx_status(prtd->dma_chan, prtd->cookie, &state);
	if (status == DMA_IN_PROGRESS || status == DMA_PAUSED) {
		buf_size = snd_pcm_lib_buffer_bytes(substream);
		if (state.residue > 0 && state.residue <= buf_size)
			pos = buf_size - state.residue;
	}

	return bytes_to_frames(substream->runtime, pos);
}

static int nx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	dev_dbg(prtd->dev, "%s.%d:%s cmd:%d\n",
		substream_to_rtd(substream)->cpu_dai->name,
		prtd->dma_param->channel, snd_pcm_stream_str(substream),
		cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = nx_pcm_dma_prepare_and_submit(substream);
		if (ret)
			return ret;
		dma_async_issue_pending(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dmaengine_resume(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (runtime->info & SNDRV_PCM_INFO_PAUSE)
			dmaengine_pause(prtd->dma_chan);
		else
			dmaengine_terminate_async(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_pause(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_async(prtd->dma_chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_hardware *pcm_hw = &nx_pcm_hardware;
	struct nx_pcm_runtime_data *prtd;
	struct dma_slave_caps dma_caps;
	int ret;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
				SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	substream->runtime->private_data = prtd;
	prtd->dev = substream->pcm->card->dev;

	prtd->dma_chan = nx_pcm_dma_request(substream);
	if (!prtd->dma_chan) {
		dev_err(prtd->dev, "Error: dma request for '%s:%s'\n",
			dev_name(prtd->dma_param->dev),
			dma_ch_name(substream));
		return -EINVAL;
	}

	/* check DMA_RESIDUE_GRANULARITY_DESCRIPTOR */
	if (!nx_pcm_dma_can_report_residue(prtd->dev, prtd->dma_chan)) {
		prtd->flags |= SND_DMAENGINE_PCM_FLAG_NO_RESIDUE;
		pcm_hw->info |= SNDRV_PCM_INFO_BATCH;
	}

	ret = dma_get_slave_caps(prtd->dma_chan, &dma_caps);
	if (!ret) {
		if (dma_caps.cmd_pause)
			pcm_hw->info |=
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME;

		if (dma_caps.residue_granularity <=
				DMA_RESIDUE_GRANULARITY_SEGMENT)
			pcm_hw->info |= SNDRV_PCM_INFO_BATCH;
	}

	if (prtd->dma_param->period_bytes_max)
		pcm_hw->period_bytes_max = prtd->dma_param->period_bytes_max;
	else
		pcm_hw->period_bytes_max = PERIOD_BYTES_MAX;

	/* set substream->runtime->hw */
	return snd_soc_set_runtime_hwparams(substream, pcm_hw);
}

static int nx_pcm_close(struct snd_pcm_substream *substream)
{
	nx_pcm_dma_release(substream);

	kfree(substream_to_prtd(substream));

	return 0;
}

static int nx_pcm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nx_pcm_runtime_data *prtd = substream_to_prtd(substream);
	int ret;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	ret = nx_pcm_dma_slave_config(substream, params);
	if (ret) {
		dev_err(prtd->dev, "Error: dma slave config !!!\n");
		dev_err(prtd->dev,
			"Error: %s.%d:%s, addr:0x%x, busw:%d, maxburst:%d\n",
			substream_to_rtd(substream)->cpu_dai->name,
			prtd->dma_param->channel, snd_pcm_stream_str(substream),
			prtd->dma_param->addr, prtd->dma_param->buswidth,
			prtd->dma_param->maxburst);
	}

	return ret;
}

static int nx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int nx_pcm_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	return snd_pcm_lib_default_mmap(substream, vma);
}

static struct snd_pcm_ops nx_pcm_ops = {
	.open		= nx_pcm_open,
	.close		= nx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= nx_pcm_hw_params,
	.hw_free	= nx_pcm_hw_free,
	.trigger	= nx_pcm_trigger,
	.pointer	= nx_pcm_pointer,
	.mmap		= nx_pcm_mmap,
};

static int nx_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	return snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
					SNDRV_DMA_TYPE_DEV,
					NULL, BUFFER_BYTES_MAX,
					BUFFER_BYTES_MAX);
}

static void nx_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static struct snd_soc_platform_driver nx_pcm_platform = {
	.ops = &nx_pcm_ops,
	.pcm_new = nx_pcm_new,
	.pcm_free = nx_pcm_free,
};

int nx_pcm_snd_platform_register(struct device *dev)
{
	return devm_snd_soc_register_platform(dev, &nx_pcm_platform);
}
EXPORT_SYMBOL_GPL(nx_pcm_snd_platform_register);

void nx_pcm_snd_platform_unregister(struct device *dev)
{
	return snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(nx_pcm_snd_platform_unregister);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound PCM driver for Nexell sound");
MODULE_LICENSE("GPL");
