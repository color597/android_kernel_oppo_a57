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

#ifndef OPPO_AUDIO_CUSTOM_H
#define OPPO_AUDIO_CUSTOM_H

int oppo_audio_init(struct platform_device *pdev);
int oppo_speaker_pa_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oppo_speaker_pa_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oppo_hp_pa_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oppo_hp_pa_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oppo_spk_pa_on(void);
int oppo_hp_pa_on(void);
void oppo_spk_pa_enable(int enable);
void oppo_hp_pa_enable(int enable);

#endif //OPPO_AUDIO_CUSTOM_H