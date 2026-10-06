// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "common.h"
#include "qdsp6/q6afe.h"

#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 16
#define WCD9335_DEFAULT_MCLK_RATE	9600000
/* 48 kHz, 2 slots of 16 bits */
#define MI2S_BCLK_RATE			1536000
#define MI2S_COUNT			4

struct apq8096_snd_data {
	/* Bit clock users per MI2S interface (RX and TX share the clock) */
	unsigned int mi2s_clk_count[MI2S_COUNT];
};

static const unsigned int apq8096_mi2s_bclk_ids[MI2S_COUNT] = {
	Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
	Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT,
	Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
	Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
};

static int apq8096_mi2s_index(unsigned int dai_id)
{
	switch (dai_id) {
	case PRIMARY_MI2S_RX ... QUATERNARY_MI2S_TX:
		return (dai_id - PRIMARY_MI2S_RX) / 2;
	default:
		return -EINVAL;
	}
}

static int apq8096_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* The MI2S bit clock is set up for 16-bit samples */
	if (apq8096_mi2s_index(snd_soc_rtd_to_cpu(rtd, 0)->id) >= 0) {
		snd_mask_none(fmt);
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	}

	return 0;
}

static int msm_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret = 0;

	ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt, rx_ch);
	if (ret != 0 && ret != -ENOTSUPP) {
		pr_err("failed to get codec chan map, err:%d\n", ret);
		goto end;
	} else if (ret == -ENOTSUPP) {
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  rx_ch_cnt, rx_ch);
	else
		ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt, tx_ch,
						  0, NULL);
	if (ret != 0 && ret != -ENOTSUPP)
		pr_err("Failed to set cpu chan map, err:%d\n", ret);
	else if (ret == -ENOTSUPP)
		ret = 0;
end:
	return ret;
}

static int apq8096_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct apq8096_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	int index, ret, i;

	index = apq8096_mi2s_index(cpu_dai->id);
	if (index < 0)
		return 0;

	if (data->mi2s_clk_count[index]++ == 0) {
		ret = snd_soc_dai_set_sysclk(cpu_dai,
					     apq8096_mi2s_bclk_ids[index],
					     MI2S_BCLK_RATE,
					     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret) {
			dev_err(rtd->dev, "failed to enable MI2S bit clock: %d\n",
				ret);
			data->mi2s_clk_count[index]--;
			return ret;
		}
	}

	/* The DSP provides bit and frame clock, the codec is consumer */
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);
	for_each_rtd_codec_dais(rtd, i, codec_dai)
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_BC_FC |
				    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S);

	return 0;
}

static void apq8096_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct apq8096_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int index;

	index = apq8096_mi2s_index(cpu_dai->id);
	if (index < 0)
		return;

	if (--data->mi2s_clk_count[index] == 0)
		snd_soc_dai_set_sysclk(cpu_dai, apq8096_mi2s_bclk_ids[index],
				       0, SNDRV_PCM_STREAM_PLAYBACK);
}

static const struct snd_soc_ops apq8096_ops = {
	.startup = apq8096_snd_startup,
	.shutdown = apq8096_snd_shutdown,
	.hw_params = msm_snd_hw_params,
};

static int apq8096_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	/*
	 * Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {144, 145, 146, 147, 148, 149,
					150, 151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
					tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	snd_soc_dai_set_sysclk(codec_dai, 0, WCD9335_DEFAULT_MCLK_RATE,
				SNDRV_PCM_STREAM_PLAYBACK);

	return 0;
}

static void apq8096_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->be_hw_params_fixup = apq8096_be_hw_params_fixup;
			link->init = apq8096_init;
			link->ops = &apq8096_ops;
		}
	}
}

static int apq8096_platform_probe(struct platform_device *pdev)
{
	struct apq8096_snd_data *data;
	struct snd_soc_card *card;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->driver_name = "apq8096";
	card->dev = dev;
	card->owner = THIS_MODULE;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	apq8096_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id msm_snd_apq8096_dt_match[] = {
	{.compatible = "qcom,apq8096-sndcard"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_snd_apq8096_dt_match);

static struct platform_driver msm_snd_apq8096_driver = {
	.probe  = apq8096_platform_probe,
	.driver = {
		.name = "msm-snd-apq8096",
		.of_match_table = msm_snd_apq8096_dt_match,
	},
};
module_platform_driver(msm_snd_apq8096_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8096 ASoC Machine Driver");
MODULE_LICENSE("GPL");
