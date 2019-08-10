// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound PCM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef __NEXELL_PCM_H__
#define __NEXELL_PCM_H_

#include <linux/dmaengine.h>

struct nx_pcm_dma_param {
	struct device *dev;
	dma_addr_t addr;
	unsigned int channel;
	int buswidth;
	int maxburst;
	char *filter_data;
	char *dma_name;
	size_t period_bytes_max;
	long clk_sample_rate;
};

struct nx_pcm_runtime_data {
	struct device *dev;
	unsigned int dma_area;
	unsigned int pos;
	struct dma_chan *dma_chan;
	struct nx_pcm_dma_param *dma_param;
	dma_cookie_t cookie;
	unsigned int flags;
};

int nx_pcm_snd_platform_register(struct device *dev);
void nx_pcm_snd_platform_unregister(struct device *dev);

#endif
