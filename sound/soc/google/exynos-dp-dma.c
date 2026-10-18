// SPDX-License-Identifier: GPL-2.0
/*
 * DMA front end for the Exynos DisplayPort audio FIFO.
 *
 * The DP link block has no bus master of its own: a PL330 channel writes
 * SST1_AUDIO_DATA_WRITE_FIFO one 32-bit word per beat, paced by the DP's
 * request line. Clocking, channel status and the infoframe are the DP
 * bridge's, reached through the hdmi-codec it registers.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DP_AUDIO_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000)

static int dp_dma_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_dmaengine_dai_dma_data *dma =
		snd_soc_dai_get_dma_data(dai, substream);

	/* words per frame; the DP's DMA_BURST_SEL is set to the same minus one */
	dma->maxburst = params_physical_width(params) * params_channels(params) / 32;
	return 0;
}

static int dp_dma_dai_probe(struct snd_soc_dai *dai)
{
	snd_soc_dai_init_dma_data(dai, snd_soc_dai_get_drvdata(dai), NULL);
	return 0;
}

static const struct snd_soc_dai_ops dp_dma_dai_ops = {
	.probe = dp_dma_dai_probe,
	.hw_params = dp_dma_hw_params,
};

static struct snd_soc_dai_driver dp_dma_dai = {
	.name = "dp-audio",
	.ops = &dp_dma_dai_ops,
	.playback = {
		.stream_name = "DP Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = DP_AUDIO_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
};

static const struct snd_soc_component_driver dp_dma_component = {
	.name = "exynos-dp-dma",
	.legacy_dai_naming = 1,
};

static int dp_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_dmaengine_dai_dma_data *dma;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	dma = devm_kzalloc(dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;
	dma->addr = res->start;
	dma->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma->maxburst = 1;
	dev_set_drvdata(dev, dma);

	ret = devm_snd_soc_register_component(dev, &dp_dma_component,
					      &dp_dma_dai, 1);
	if (ret)
		return ret;

	return devm_snd_dmaengine_pcm_register(dev, NULL, 0);
}

static const struct of_device_id dp_dma_of_match[] = {
	{ .compatible = "samsung,exynos910-dp-audio-dma" },
	{ }
};
MODULE_DEVICE_TABLE(of, dp_dma_of_match);

static struct platform_driver dp_dma_driver = {
	.driver = {
		.name = "exynos-dp-dma",
		.of_match_table = dp_dma_of_match,
	},
	.probe = dp_dma_probe,
};
module_platform_driver(dp_dma_driver);

MODULE_DESCRIPTION("Exynos DisplayPort audio DMA front end");
MODULE_LICENSE("GPL");
