// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell SoC sound PDM/PCM card driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <sound/soc.h>

struct nx_pdm_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
};

static int nx_pdm_card_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *pdm_np;
	struct nx_pdm_data *data;
	int ret;

	pdm_np = of_parse_phandle(node, "snd-controller", 0);
	if (!pdm_np) {
		dev_err(&pdev->dev, "failed to find snd-controller\n");
		ret = -EINVAL;
		goto end;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto end;
	}

	data->dai.name = "PDM/PCM";
	data->dai.stream_name = "PDM/PCM";
	data->dai.codec_dai_name = "snd-soc-dummy-dai";
	data->dai.codec_name = "snd-soc-dummy";
	data->dai.cpu_of_node = pdm_np;
	data->dai.platform_of_node = pdm_np;
	data->dai.capture_only = true;

	data->card.dev = &pdev->dev;
	data->card.name = "Sound PDM-PCM";
	data->card.dai_link = &data->dai;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret)
		dev_err(&pdev->dev,
			"snd_soc_register_card failed: %d\n", ret);

end:
	of_node_put(pdm_np);

	return ret;
}

static const struct of_device_id of_pdm_card[] = {
	{ .compatible = "nexell,nxp3220-pdm-card" },
	{},
};

static struct platform_driver nx_pdm_driver = {
	.probe  = nx_pdm_card_probe,
	.driver = {
		.name	= "nexell-pdm-card",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(of_pdm_card),
	},
};
#ifdef CONFIG_DEFERRED_SOUND_PDM
static int __init nx_pdm_card_driver_init(void)
{
	return platform_driver_register(&nx_pdm_driver);
}
deferred_module_init(nx_pdm_card_driver_init)
#else
module_platform_driver(nx_pdm_driver);
#endif

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound PDM/PCM card for Nexell sound");
MODULE_LICENSE("GPL");
