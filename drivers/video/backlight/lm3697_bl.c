/*
 * TI LM3697 Backlight Driver
 *
 * Copyright 2014 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/lm3697_bl.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <soc/oppo/mmkey_log.h>

#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/boot_mode.h>

/* Registers */
#define LM3697_REG_OUTPUT_CFG       0x10

#define LM3697_REG_BRT_CFG      0x16

#define LM3697_REG_BOOST_CTL    0x1A
#define LM3697_REG_BOOST        0x06

#define LM3697_REG_PWM_CFG      0x1C

#define LM3697_REG_IMAX_A       0x17
#define LM3697_REG_IMAX_B       0x18

#define LM3697_REG_BRT_A_LSB        0x20
#define LM3697_REG_BRT_A_MSB        0x21
#define LM3697_REG_BRT_B_LSB        0x22
#define LM3697_REG_BRT_B_MSB        0x23
#define LM3697_BRT_LSB_MASK     (BIT(0) | BIT(1) | BIT(2))
#define LM3697_BRT_MSB_SHIFT        3

#define LM3697_REG_ENABLE       0x24

/* Other definitions */
#define LM3697_PWM_ID           1
#define LM3697_MAX_REGISTERS        0xB4
#define LM3697_MAX_STRINGS      3
#define LM3697_MAX_BRIGHTNESS       2047 //2047
#define LM3697_MIN_BRIGHTNESS       4
#define LM3697_PWM_CLOSE_BRIGHTNESS 150
#define LM3697_IMAX_OFFSET      6
#define LM3697_DEFAULT_NAME     "lcd-backlight"
#define LM3697_DEFAULT_PWM      "lm3697-backlight"

#define LM3697_EXPONENTIAL      1
#define LM3697_LINEAR           0

enum lm3697_bl_ctrl_mode {
    BL_REGISTER_BASED,
    BL_PWM_BASED,
};

/*
 * struct lm3697_bl_chip
 * @dev: Parent device structure
 * @regmap: Used for I2C register access
 * @pdata: LM3697 platform data
 */
struct lm3697_bl_chip {
    struct device *dev;
    struct lm3697_platform_data *pdata;
    struct regmap *regmap;

};

/*
 * struct lm3697_bl
 * @bank_id: Control bank ID. BANK A or BANK A and B
 * @bl_dev: Backlight device structure
 * @chip: LM3697 backlight chip structure for low level access
 * @bl_pdata: LM3697 backlight platform data
 * @mode: Backlight control mode
 * @pwm: PWM device structure. Only valid in PWM control mode
 * @pwm_name: PWM device name
 */
struct lm3697_bl {
    int bank_id;
    struct backlight_device *bl_dev;
    struct lm3697_bl_chip *chip;
    struct lm3697_backlight_platform_data *bl_pdata;
    enum lm3697_bl_ctrl_mode mode;
    struct pwm_device *pwm;
    char pwm_name[20];
};

static struct lm3697_bl_chip *lm3697_pchip;
static unsigned int pt_times = 0;
static bool pwm_set_flag = false;

void set_lm3697_backlight_gpio(unsigned int value)
{
  struct lm3697_bl_chip *pchip = lm3697_pchip;
  int ret = 0;
  pr_err("%s bl_en_gpio  value =%d\n", __func__, value);
//LiPing@MultiMedia.Display.LCD.Stability,2017/2/17,add for case#927814
  if(!pchip || !lm3697_pchip){
        pr_err("%s  pchip is NULL \n", __func__);
        return;
  }
  if(pchip->pdata == NULL){
        pr_err("%s  pchip->pdata is NULL \n", __func__);
        return;
  }
  if(value==1){
  if (!gpio_is_valid(pchip->pdata->en_gpio))
      ret = gpio_request(pchip->pdata->en_gpio, "backlight_enable");
    if (ret) {
        pr_err("request enable gpio failed, ret=%d\n", ret);
    }
    pr_err("%s bl_en_gpio=%d\n", __func__, pchip->pdata->en_gpio);
    if (gpio_is_valid(pchip->pdata->en_gpio)){
        gpio_set_value((pchip->pdata->en_gpio), 1);
        gpio_direction_output((pchip->pdata->en_gpio), 1);
    }
  }else{
    if (gpio_is_valid(pchip->pdata->en_gpio)){
        gpio_set_value(pchip->pdata->en_gpio, 0);
    }
  }
}

int set_lm3697_backlight_scale_current(void)
{
    struct lm3697_bl_chip *pchip = lm3697_pchip;
    int ret = 0;
    ret = regmap_write(pchip->regmap, 0x17, 0x10);  //Bank A Full-scale current (17.8mA)
        if (ret < 0)
            goto out;
    ret = regmap_write(pchip->regmap, 0x18, 0x10); //Bank B Full-scale current (17.8mA)
        if (ret < 0)
            goto out;
    return ret;
out:
    dev_err(pchip->dev, "i2c failed to access register\n");
    return ret;
}
int init_lm3697_backlight_reg(void)
{
    struct lm3697_bl_chip *pchip = lm3697_pchip;
    int ret = 0;

    ret = regmap_write(pchip->regmap, 0x10, 0x04);  //HVLED1, 2, 3 enable
        if (ret < 0)
        goto out;

#ifdef VENDOR_EDIT
//Shengjun.Gou@Phone.Multimedia.Driver.Stability, 2016/09/26,
//modify for 16027 VOP adaption and EXPONENTIAL MODE
    if (is_project(OPPO_16027))
    {
        ret = regmap_write(pchip->regmap, 0x1A, 0x04);    //OVP 36V, Freq 500kh
        if (ret < 0)
            goto out;
        ret = regmap_write(pchip->regmap, 0x17, 0x10);  //Bank A Full-scale current (17.8mA)
        if (ret < 0)
            goto out;
        ret = regmap_write(pchip->regmap, 0x18, 0x10);    //Bank B Full-scale current (17.8mA)
        if (ret < 0)
            goto out;
    } else {
        ret = regmap_write(pchip->regmap, 0x1A, 0x02);    //OVP 24V, Freq 500kh
    }
    if (ret < 0)
        goto out;

    if(LM3697_EXPONENTIAL){
        ret = regmap_write(pchip->regmap, 0x16, 0x00);  //Exponential Mapping Mode
    }else{
        ret = regmap_write(pchip->regmap, 0x16, 0x01);  //0X01 Linear Mapping Mode 0X00 Index Mapping Mode
    }
    if (ret < 0)
        goto out;
#endif /*VENDOR_EDIT*/

    ret = regmap_write(pchip->regmap, 0x19, 0x03);  //Linear Mapping Mode
    if (ret < 0)
        goto out;

    ret = regmap_write(pchip->regmap, 0x24, 0x01);  //Enable Bank A / Disable Bank B
        if (ret < 0)
        goto out;
    ret = regmap_write(pchip->regmap, 0x1c, 0x0D);  //Set PWM Open
        if (ret < 0)
        goto out;
    return ret;
out:
    dev_err(pchip->dev, "i2c failed to access register\n");
    return ret;
}


int lm3697_lcd_backlight_set_level(unsigned int bl_level)
{
    struct lm3697_bl_chip *pchip = lm3697_pchip;
    unsigned int BL_MSB =0;
    unsigned int BL_LSB =0;
    int ret = 0;

    if(pt_times<5){
        pr_err("%s: bl=%d \n", __func__,bl_level);
        pt_times ++;
    }

    if(!pchip || !lm3697_pchip){
        dev_err(pchip->dev, "lm3697_lcd_backlight_set_level pchip is null\n");
        return 0;
    }

    if (!pchip->regmap || !lm3697_pchip->regmap) {
        pr_err("%s  pchip->regmap is NULL.\n", __func__);
        return 0;
    }

#ifdef VENDOR_EDIT
//Shengjun.Gou@Multimedia.Driver.Power, 2016/10/24,
//add for 16027 ftm brightness too low
    if ((get_boot_mode() == MSM_BOOT_MODE__FACTORY)
        && (bl_level <= 255)
        && (is_project(OPPO_16027))) {
//LiPing@MultiMedia.Display.LCD.Stability,2017/2/17,add for case#859933
        if((bl_level == 180)&& LM3697_EXPONENTIAL){
            bl_level = 1800;
            pr_err("%s: factory mode set bl=%d \n", __func__,bl_level);
        }else{
            bl_level = bl_level * 8;
        }
    }
#endif /*VENDOR_EDIT*/

    /* liping-m@PhoneSW.Multimedia, 2016/09/06 ADD for lcm cabc*/
    if (is_project(OPPO_16061)) {
        if((bl_level < LM3697_PWM_CLOSE_BRIGHTNESS)&&(LM3697_MIN_BRIGHTNESS <= bl_level))
        {
            ret = regmap_write(pchip->regmap, 0x1c, 0x0c);
            if (ret < 0)
                goto out;
            pwm_set_flag=true;
            bl_level = LM3697_MIN_BRIGHTNESS+(bl_level-4)*140/LM3697_PWM_CLOSE_BRIGHTNESS;
            if(bl_level < LM3697_MIN_BRIGHTNESS)
                bl_level = LM3697_MIN_BRIGHTNESS;
            //pr_err("%s LCD close pwm \n", __func__);
        }else if((bl_level >= LM3697_PWM_CLOSE_BRIGHTNESS)&&(pwm_set_flag==true)){
            ret = regmap_write(pchip->regmap, 0x1c, 0x0D);
            if (ret < 0)
                goto out;
            pwm_set_flag=false;
            //pr_err("%s LCD open pwm \n", __func__);
        }
    }

    if(LM3697_EXPONENTIAL){
        BL_MSB = (backlight_buf[bl_level] >>3) & 0xFF;
        BL_LSB = backlight_buf[bl_level] & 0x07;
    }else{
        BL_MSB = (bl_level >>3) & 0xFF;
        BL_LSB = bl_level & 0x07;
    }

    /* brightness 0 means disable */
    if (bl_level==0) {
        ret = regmap_write(pchip->regmap, 0x20, BL_LSB);
        if (ret < 0)
            goto out;
        ret = regmap_write(pchip->regmap, 0x21, BL_MSB);
        if (ret < 0)
            goto out;
        pt_times = 0;
    }else{
        ret = regmap_write(pchip->regmap, 0x24, 01);
            if (ret < 0)
                goto out;

        ret = regmap_write(pchip->regmap, 0x20, BL_LSB);
            if (ret < 0)
                goto out;

        ret = regmap_write(pchip->regmap, 0x21, BL_MSB);
            if (ret < 0)
                goto out;
    }

    return ret;
out:
    pr_err("%s  set lcd backlight failed.\n", __func__);
    return ret;
}

EXPORT_SYMBOL(lm3697_lcd_backlight_set_level);



static int lm3697_dt(struct device *dev, struct lm3697_platform_data *pdata)
{
    struct device_node *np = dev->of_node;

    pdata->en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio", 0);

    pr_err("%s bl_en_gpio=%d\n", __func__, pdata->en_gpio);

    if (!gpio_is_valid(pdata->en_gpio))
            pr_err("%s:%d, Backlight_en gpio not specified\n", __func__, __LINE__);

    return 0;
}

static int lm3697_chip_init(struct lm3697_bl_chip *pchip){
    int ret = 0;

    ret = regmap_write(pchip->regmap, 0x10, 0x04);  //HVLED1, 2, 3 enable
        if (ret < 0)
        goto out;

#ifdef VENDOR_EDIT
//Shengjun.Gou@Phone.Multimedia.Driver.Stability, 2016/09/26,
//modify for 16027 VOP adaption and LM3697_EXPONENTIAL MODE
    if (is_project(OPPO_16027))
    {
        ret = regmap_write(pchip->regmap, 0x1A, 0x04);  //OVP 36V, Freq 500kh
    } else {
        ret = regmap_write(pchip->regmap, 0x1A, 0x02);  //OVP 24V, Freq 500kh
    }
    if (ret < 0)
        goto out;

    if(LM3697_EXPONENTIAL){
        ret = regmap_write(pchip->regmap, 0x16, 0x00);    //Exponential Mapping Mode
    }else{
        ret = regmap_write(pchip->regmap, 0x16, 0x01);    //0X01 Linear Mapping Mode 0X00 Index Mapping Mode
    }
    if (ret < 0)
        goto out;
#endif /*VENDOR_EDIT*/

    ret = regmap_write(pchip->regmap, 0x19, 0x03);  //Linear Mapping Mode
    if (ret < 0)
        goto out;

    if (is_project(OPPO_16027))
    {
        ret = regmap_write(pchip->regmap, 0x17, 0x10);  //Bank A Full-scale current (17.8mA)
        if (ret < 0)
            goto out;
        ret = regmap_write(pchip->regmap, 0x18, 0x10);    //Bank B Full-scale current (17.8mA)
        if (ret < 0)
            goto out;

    }
    ret = regmap_write(pchip->regmap, 0x24, 0x01);  //Enable Bank A / Disable Bank B
        if (ret < 0)
        goto out;
    ret = regmap_write(pchip->regmap, 0x1c, 0x0D);  //Set PWM Open
        if (ret < 0)
        goto out;

#ifdef VENDOR_EDIT
/* Xiaori.Yuan@Mobile Phone Software Dept.Driver, 2015/06/03  Add for silence mode */
    if(MSM_BOOT_MODE__SILENCE == get_boot_mode())
        return 0;
#endif /*VENDOR_EDIT*/

    return ret;

out:
    dev_err(pchip->dev, "i2c failed to access register\n");
    return ret;
}

static struct regmap_config lm3697_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = LM3697_MAX_REGISTERS,
};

static int lm3697_bl_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
    struct lm3697_platform_data *pdata = client->dev.platform_data;
    struct lm3697_bl_chip *pchip;
    unsigned int revision;
    static char *temp;
    int ret = 0;

    pr_err("%s Enter\n", __func__);
    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
            sizeof(struct lm3697_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }

        ret = lm3697_dt(&client->dev, pdata);
        if (ret)
            return ret;
    } else
        pdata = client->dev.platform_data;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "fail : i2c functionality check...\n");
        return -EOPNOTSUPP;
    }

    if (pdata == NULL) {
        dev_err(&client->dev, "fail : no platform data.\n");
        return -ENODATA;
    }

    pchip = devm_kzalloc(&client->dev, sizeof(struct lm3697_bl_chip),
                 GFP_KERNEL);
    if (!pchip)
        return -ENOMEM;

    pchip->pdata = pdata;
    pchip->dev = &client->dev;
//LiPing@MultiMedia.Display.LCD.Stability,2017/2/17,add for case#927814
	lm3697_pchip = pchip;

    ret = gpio_request(pdata->en_gpio, "backlight_enable");
    if (ret) {
        pr_err("request enable gpio failed, ret=%d\n", ret);
    }
    pr_err("%s bl_en_gpio=%d\n", __func__, pdata->en_gpio);


    if (gpio_is_valid(pdata->en_gpio)){
        gpio_set_value((pdata->en_gpio), 1);
        gpio_direction_output((pdata->en_gpio), 1);
    }

    pchip->regmap = devm_regmap_init_i2c(client, &lm3697_regmap);
    if (IS_ERR(pchip->regmap)) {
        ret = PTR_ERR(pchip->regmap);
        dev_err(&client->dev, "fail : allocate register map: %d\n",
            ret);
        return ret;
    }

    i2c_set_clientdata(client, pchip);

    /* chip initialize */
    ret = lm3697_chip_init(pchip);
    if (ret < 0) {
        dev_err(&client->dev, "fail : init chip\n");
        mm_keylog_write("No Backlight\n", "Backlight device probe failed\n", TYPE_BL_EXCEPTION);
        goto error_enable;
    }

    regmap_read(pchip->regmap,0x00,&revision);
    if (revision == 0x02) {
        temp = "02";
    }else if (revision == 0x00) {
        temp = "00";
    }else {
        temp = "unknown";
    }

    pr_err("%s :revision = %s\n", __func__, temp);


    register_device_proc("backlight", temp, "LM3697");


    pr_info("%s : probe done\n", __func__);

    return 0;


error_enable:
    /* chip->pdata and chip->pdata->bl_pdata
     * are allocated in lm3697_bl_parse_dt() by devm_kzalloc()
     */

    devm_kfree(&client->dev, pchip->pdata);
    devm_kfree(&client->dev, pchip);
    pr_err("%s : probe failed\n", __func__);
    return ret;
}

static int lm3697_bl_remove(struct i2c_client *client){
    struct lm3697_bl_chip *pchip = i2c_get_clientdata(client);
    int ret = 0;

    ret = regmap_write(pchip->regmap, LM3697_REG_BRT_A_LSB, 0);
    if (ret < 0)
        dev_err(pchip->dev, "i2c failed to access register\n");

    ret = regmap_write(pchip->regmap, LM3697_REG_BRT_A_MSB, 0);
    if (ret < 0)
        dev_err(pchip->dev, "i2c failed to access register\n");

    if (gpio_is_valid(pchip->pdata->en_gpio)){
        gpio_set_value(pchip->pdata->en_gpio, 0);
        gpio_free(pchip->pdata->en_gpio);
    }

    return 0;
}

static const struct i2c_device_id lm3697_bl_ids[] = {
    { "lm3697", 0 },
    { }
};


static struct i2c_driver lm3697_i2c_driver = {
    .probe = lm3697_bl_probe,
    .remove = lm3697_bl_remove,
    .driver = {
        .name = "lm3697",
        .owner = THIS_MODULE,
    },
    .id_table = lm3697_bl_ids,
};

module_i2c_driver(lm3697_i2c_driver);


MODULE_DESCRIPTION("TI LM3697 Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
