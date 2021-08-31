/************************************************************************************
** Copyright (C), 2000-2016, OPPO Mobile Comm Corp., Ltd
** All rights reserved.
**
** VENDOR_EDIT
**
** Description: -
**      oppo audio custom configuration
**
**
** --------------------------- Revision History: --------------------------------
** <author>                      <date>    <version >    <desc>
** ------------------------------------------------------------------------------
** Jianfeng.Qiu@Swdp.Multimedia  2016/08/10   1.0	    create file
** ------------------------------------------------------------------------------
**
************************************************************************************/

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
//#include <linux/stddef.h>
#include <sound/soc.h>
#include <linux/regulator/consumer.h>

#include "oppo_audio_custom.h"

struct oppo_audio_data {
    int spk_pa_en;
    int spk_boost_en;
    int hp_pa_en;
    int spk_pa_on;
    int hp_pa_on;
    struct regulator *hp_pa_vdd;
};

struct oppo_audio_data *audio_pdata = NULL;

int oppo_audio_init(struct platform_device *pdev)
{
    const char *spk_pa_str = "oppo-spk-pa-en";
    const char *spk_boost_str = "oppo-spk-boost-en";
    const char *hp_pa_str = "oppo-hp-pa-en";
    const char *hp_vdd_str = "oppo-hp-vdd";
    int ret = 0;

    pr_info("%s:Enter\n", __func__);
    if (!audio_pdata) {
        audio_pdata = kzalloc(sizeof(struct oppo_audio_data), GFP_KERNEL);
        if (!audio_pdata) {
            pr_err("%s: fail to init\n", __func__);
            return -ENOMEM;
        }

        audio_pdata->spk_pa_en = of_get_named_gpio(pdev->dev.of_node, spk_pa_str, 0);
        if (audio_pdata->spk_pa_en < 0) {
            dev_dbg(&pdev->dev, "%s: missing %s in dt node\n", __func__, spk_pa_str);
        } else {
            if (!gpio_is_valid(audio_pdata->spk_pa_en)) {
                pr_err("%s: Invalid external speaker gpio: %d\n", __func__, audio_pdata->spk_pa_en);
            } else {
                ret = gpio_request(audio_pdata->spk_pa_en, "spk_pa_en_gpio");
                if (ret) {
                    pr_err("%s: cannot request spk pa gpio\n", __func__);
                }
            }
        }

        audio_pdata->spk_boost_en = of_get_named_gpio(pdev->dev.of_node, spk_boost_str, 0);
        if (audio_pdata->spk_boost_en < 0) {
            dev_dbg(&pdev->dev, "%s: missing %s in dt node\n", __func__, spk_boost_str);
        } else {
            if (!gpio_is_valid(audio_pdata->spk_boost_en)) {
                pr_err("%s: Invalid boost gpio: %d\n", __func__, audio_pdata->spk_boost_en);
            } else {
                ret = gpio_request(audio_pdata->spk_boost_en, "spk_boost_en_gpio");
                if (ret) {
                    pr_err("%s: cannot request spk boost gpio\n", __func__);
                }
            }
        }

        audio_pdata->hp_pa_en = of_get_named_gpio(pdev->dev.of_node, hp_pa_str, 0);
        if (audio_pdata->hp_pa_en < 0) {
            dev_dbg(&pdev->dev, "%s: missing %s in dt node\n", __func__, hp_pa_str);
        } else {
            if (!gpio_is_valid(audio_pdata->hp_pa_en)) {
                pr_err("%s: Invalid boost gpio: %d\n", __func__, audio_pdata->hp_pa_en);
            } else {
                ret = gpio_request(audio_pdata->hp_pa_en, "hp_pa_en_gpio");
                if (ret) {
                    pr_err("%s: cannot request headphone pa gpio\n", __func__);
                }
            }
        }

        audio_pdata->hp_pa_vdd = regulator_get(&pdev->dev, hp_vdd_str);
        if (IS_ERR(audio_pdata->hp_pa_vdd)) {
            pr_err("%s: can not get regulator for %s\n", __func__, hp_vdd_str);
        } else {
            if (regulator_count_voltages(audio_pdata->hp_pa_vdd) > 0) {
                ret = regulator_set_voltage(audio_pdata->hp_pa_vdd, 2950000, 2950000);
                if (ret) {
                    pr_err("%s: regulator set hp vdd failed ret=%d\n", __func__, ret);
                } else {
                    ret = regulator_enable(audio_pdata->hp_pa_vdd);
                    if (ret) {
                        pr_err("%s: regulator enable hp vdd failed ret=%d\n", __func__, ret);
                    }
                }
            }
        }
    }

    return 0;
}

int oppo_speaker_pa_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if (!audio_pdata) {
        return 0;
    }

    ucontrol->value.integer.value[0] = audio_pdata->spk_pa_on;

    return 0;
}

int oppo_speaker_pa_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if (!audio_pdata || !gpio_is_valid(audio_pdata->spk_boost_en) || !gpio_is_valid(audio_pdata->spk_pa_en)) {
        pr_err("%s: missing configuration\n", __func__);
        return -ENOSYS;
    }

    switch (ucontrol->value.integer.value[0]) {
    case 1:
        pr_info("%s: enable pa\n", __func__);
        gpio_direction_output(audio_pdata->spk_boost_en, 1);
        usleep_range(1000, 1050);
        gpio_direction_output(audio_pdata->spk_pa_en, 1);
        audio_pdata->spk_pa_on = 1;
        break;
    case 0:
    default:
        pr_info("%s: disable pa\n", __func__);
        gpio_direction_output(audio_pdata->spk_pa_en, 0);
        usleep_range(1000, 1050);
        gpio_direction_output(audio_pdata->spk_boost_en, 0);
        audio_pdata->spk_pa_on = 0;
        break;
    }

    return 0;
}

int oppo_hp_pa_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if (!audio_pdata) {
        return 0;
    }

    ucontrol->value.integer.value[0] = audio_pdata->hp_pa_on;

    return 0;
}

int oppo_hp_pa_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if (!audio_pdata || !gpio_is_valid(audio_pdata->hp_pa_en)) {
        pr_err("%s: missing configuration\n", __func__);
        return -ENOSYS;
    }

    switch (ucontrol->value.integer.value[0]) {
    case 1:
        pr_info("%s: enable pa\n", __func__);
        gpio_direction_output(audio_pdata->hp_pa_en, 1);
        audio_pdata->hp_pa_on = 1;
        break;
    case 0:
    default:
        pr_info("%s: disable pa\n", __func__);
        gpio_direction_output(audio_pdata->hp_pa_en, 0);
        audio_pdata->hp_pa_on = 0;
        break;
    }

    return 0;
}

int oppo_spk_pa_on(void)
{
    if (!audio_pdata) {
        return 0;
    }

    return audio_pdata->spk_pa_on;
}

int oppo_hp_pa_on(void)
{
    if (!audio_pdata) {
        return 0;
    }

    return audio_pdata->hp_pa_on;
}

void oppo_spk_pa_enable(int enable)
{
    if (!audio_pdata || !gpio_is_valid(audio_pdata->spk_boost_en) || !gpio_is_valid(audio_pdata->spk_pa_en)) {
        pr_debug("%s: missing configuration\n", __func__);
        return;
    }

    if (audio_pdata->spk_pa_on == enable) {
        pr_info("%s: pa is already %s\n", __func__, enable ? "enable" : "disable");
        return;
    }

    if (enable) {
        pr_info("%s: enable pa\n", __func__);
        gpio_direction_output(audio_pdata->spk_boost_en, 1);
        usleep_range(1000, 1050);
        gpio_direction_output(audio_pdata->spk_pa_en, 1);
        audio_pdata->spk_pa_on = 1;
    } else {
        pr_info("%s: disable pa\n", __func__);
        gpio_direction_output(audio_pdata->spk_pa_en, 0);
        usleep_range(1000, 1050);
        gpio_direction_output(audio_pdata->spk_boost_en, 0);
        audio_pdata->spk_pa_on = 0;
    }
}

void oppo_hp_pa_enable(int enable)
{
    if (!audio_pdata || !gpio_is_valid(audio_pdata->hp_pa_en)) {
        pr_debug("%s: missing configuration\n", __func__);
        return;
    }

    if (audio_pdata->hp_pa_on == enable) {
        pr_info("%s: pa is already %s\n", __func__, enable ? "enable" : "disable");
        return;
    }

    if (enable) {
        pr_info("%s: enable pa\n", __func__);
        gpio_direction_output(audio_pdata->hp_pa_en, 1);
        audio_pdata->hp_pa_on = 1;
    } else {
        pr_info("%s: disable pa\n", __func__);
        gpio_direction_output(audio_pdata->hp_pa_en, 0);
        audio_pdata->hp_pa_on = 0;
    }
}
