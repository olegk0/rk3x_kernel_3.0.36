/*
 *  (c) 2014 olegvedi@gmail.com
 *  Audio for Asus MyPal a620
 *
 *  based on:
 *
 * SoC audio for HTC Magician
 *
 * Copyright (c) 2006 Philipp Zabel <philipp.zabel@gmail.com>
 *
 * based on spitz.c,
 * Authors: Liam Girdwood <lrg@slimlogic.co.uk>
 *          Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/uda1380.h>
#include <sound/jack.h>

#include <mach/asus620.h>
#include <asm/mach-types.h>
#include "../codecs/uda1380.h"
#include "pxa2xx-i2s.h"

static struct snd_soc_jack hs_jack;

/* Headphones jack detection DAPM pin */
static struct snd_soc_jack_pin hs_jack_pin[] = {
    {
	.pin	= "Headphone Jack",
	.mask	= SND_JACK_HEADPHONE,
    },
    {
	.pin	= "Speaker",
	/* disable speaker when hp jack is inserted */
	.mask   = SND_JACK_HEADPHONE,
	.invert	= 1,
    },
};

/* Headphones jack detection GPIO */
static struct snd_soc_jack_gpio hs_jack_gpio = {
    .gpio		= GPIO12_A620_HEARPHONE_N,
    .invert		= true,
    .name		= "hp-gpio",
    .report		= SND_JACK_HEADPHONE,
    .debounce_time	= 200,
};


static int a620_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

//	mutex_lock(&codec->mutex);

//	mutex_unlock(&codec->mutex);
//    runtime->hw.rate_min = 1;
//    runtime->hw.rate_max = 2;
//    runtime->hw.rates = SNDRV_PCM_RATE_KNOT;
	return 0;
}

/*
 * I2S for play and record.
 */
static int a620_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
			SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

/*        ret = snd_soc_dai_set_sysclk(codec_dai, 0, 0,
                                     SND_SOC_CLOCK_IN);
        if (ret < 0)
                return ret;
*/
	/* set the I2S system clock as output */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
			SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops a620_snd_ops = {
	.startup = a620_startup,
	.hw_params = a620_hw_params,
};

static int a620_spk_power(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
    if(SND_SOC_DAPM_EVENT_ON(event)){
	asus620_gpo_set(GPO_A620_SOUND);
    }
    else{
	asus620_gpo_clear(GPO_A620_SOUND);
    }

	return 0;
}


static int a620_mic_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
    if (SND_SOC_DAPM_EVENT_ON(event)){
	asus620_gpo_set(GPO_A620_MICROPHONE);
    }
    else{
	asus620_gpo_clear(GPO_A620_MICROPHONE);
    }
	return 0;
}

/* a620 machine dapm widgets */
static const struct snd_soc_dapm_widget uda1380_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", a620_spk_power),
	SND_SOC_DAPM_MIC("Mic", a620_mic_bias),
};

/* a620 machine audio_map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Headphone connected to */
	{"Headphone Jack", NULL, "VOUTLHP"},//VOUTLHP
	{"Headphone Jack", NULL, "VOUTRHP"},//VOUTRHP

	/* Speaker connected to */
	{"Speaker", NULL, "VOUTL"},
	{"Speaker", NULL, "VOUTR"},

	/* Mic connected to */
	{"VINM", NULL, "Mic"},
};

/*
 * Logic for a uda1380 as connected on a a620
 */
static int a620_uda1380_init(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_soc_codec *codec = rtd->codec;
    struct snd_soc_dapm_context *dapm = &codec->dapm;
    int err=0;

    /* FIXME: is anything connected here? */
    snd_soc_dapm_nc_pin(dapm, "VINL");
    snd_soc_dapm_nc_pin(dapm, "VINR");


    snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
    snd_soc_dapm_enable_pin(dapm, "Speaker");
    snd_soc_dapm_enable_pin(dapm, "Mic");

    /* Jack detection API stuff */
    err = snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE, &hs_jack);
    if (err)
        return err;
    
    err = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pin),hs_jack_pin);
    if (err)
        return err;
    err = snd_soc_jack_add_gpios(&hs_jack, 1, &hs_jack_gpio);

    return err;
}

/* a620 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link a620_dai[] = {
{
	.name = "UDA1380",
	.stream_name = "UDA1380",
	.cpu_dai_name = "pxa2xx-i2s",
	.codec_dai_name = "uda1380-hifi",
	.platform_name = "pxa-pcm-audio",
	.codec_name = "uda1380-codec.0-001a",
	.init = a620_uda1380_init,
	.ops = &a620_snd_ops,
},
};

/* a620 audio machine driver */
static struct snd_soc_card snd_soc_card_a620 = {
	.name = "a620-snd",
	.owner = THIS_MODULE,
	.dai_link = a620_dai,
	.num_links = ARRAY_SIZE(a620_dai),

    .dapm_widgets = uda1380_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(uda1380_dapm_widgets),
    .dapm_routes = audio_map,
    .num_dapm_routes = ARRAY_SIZE(audio_map),
};

static int __devinit a620_audio_probe(struct platform_device *pdev)
{
    int ret;

    snd_soc_card_a620.dev = &pdev->dev;
    ret = snd_soc_register_card(&snd_soc_card_a620);
    if (ret)
	goto err_pdev;

    return 0;

err_pdev:

    return ret;
}

static int __devexit a620_audio_remove(struct platform_device *pdev)
{
    snd_soc_jack_free_gpios(&hs_jack, 1, &hs_jack_gpio);
    snd_soc_unregister_card(&snd_soc_card_a620);
    return 0;
}

static struct platform_driver a620_audio_driver = {
    .driver	= {
	.name = "a620-audio",
	.owner = THIS_MODULE,
	.pm = &snd_soc_pm_ops,
    },
    .probe	= a620_audio_probe,
    .remove	= __devexit_p(a620_audio_remove),
};

module_platform_driver(a620_audio_driver);

MODULE_DESCRIPTION("ALSA SoC ASUS a620");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:a620-audio");

