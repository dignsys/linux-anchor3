// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound Null codec driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define CODEC_SND_SOC_FORMATS (\
	SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8 | \
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_U32_LE \
	)

static struct snd_soc_dai_driver nx_null_codec_dai = {
	.name = "snd-null-codec",
	.playback = {
		.stream_name = "Snd Null Playback",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = CODEC_SND_SOC_FORMATS,
	},
	.capture = {
		.stream_name = "Snd Null Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = CODEC_SND_SOC_FORMATS,
	},
};

static struct snd_soc_codec_driver nx_null_codec_drv;

static int nx_null_codec_probe(struct platform_device *pdev)
{
	int ret = snd_soc_register_codec(&pdev->dev,
			&nx_null_codec_drv, &nx_null_codec_dai, 1);

	if (ret)
		dev_err(&pdev->dev,
			"Failed, register nexell null codec driver.\n");

	return ret;
}

static int nx_null_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id of_nx_null_codec[] = {
	{ .compatible = "nexell,snd-null-codec" },
	{},
};
MODULE_DEVICE_TABLE(of, of_nx_null_codec);

static struct platform_driver nx_null_codec_driver = {
	.probe = nx_null_codec_probe,
	.remove = nx_null_codec_remove,
	.driver = {
		.name = "nexell-snd-null-codec",
		.of_match_table = of_match_ptr(of_nx_null_codec),
	},
};

module_platform_driver(nx_null_codec_driver);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("ASoc null codec driver");
MODULE_LICENSE("GPL");
