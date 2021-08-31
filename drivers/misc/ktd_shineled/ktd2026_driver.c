/************************************************************************************
** File: - android\kernel\drivers\misc\ktd_shineled\ktd2026_driver.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**      breath-led KTD2026's driver for oppo
**      can support three kinds of leds if HW allows.
** Version: 1.0
** Date created: 18:35:46,11/11/2014
** Author: Tong.han@Bsp.group.TP&BL
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Tong.han@Bsp.group.TP&BL ,2014-11-11 , Add this driver for breath-LED for project 14051.
************************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h> 
#include <linux/cdev.h>
#include <linux/leds.h>

#define KTD_I2C_NAME			"ktd2026"

struct i2c_client *ktd20xx_client;
static struct class *ktd20xx_cls;
struct class *breathled_class;
struct device *breathled_dev;
struct device *ktd22xx_dev;

int led_on_off = 255;
unsigned long value = 0;
int breath_leds = 0;//1--breath_led,255--turn on 0--turn off
static int ktd22xx_max_brightness = 0xFF;

static int major;
static int onMS = 0;
static int totalMS = 0;
static int color_D1, color_D2, color_D3;

#define LED1_OUT_ALON  0x1   //always on
#define LED1_OUT_PWM1 0x2
#define LED2_OUT_ALON  0x1<<2
#define LED2_OUT_PWM1 0x2<<2
#define LED3_OUT_ALON  0x1<<4
#define LED3_OUT_PWM1 0x2<<4

#define LED_OUT_ALON  (LED2_OUT_ALON)
#define LED_OUT_PWM1 (LED2_OUT_PWM1)

void ktd22xx_lowbattery_breath_leds(void){
	/*
	 * flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0xaa);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x12);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_PWM1);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd22xx_events_breath_leds(void){
	/* 
	 * flash time period: 5s, rise/fall 2s, sleep 1s 
	 * reg5 = 0xaa, reg1 = 0x30	
	 */
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0xaa);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x30);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_PWM1);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd2xx_led_on(void){
	//turn on led when 0x00 is 0x00
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);//set sleep mode
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x2f);//set current is 10mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_ALON);//turn om all of leds
}

void ktd2xx_led_off(void){
	//turn on led when 0x02 is not 0x00,there is set to same as breath leds
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0.125mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);//turn off leds
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x28);//set sleep mode
}

void ktd22xx_breath_leds_time(int blink){
	//set breath led flash time from blink
	int period, flashtime;
	period = (blink >> 8) & 0xff;
	flashtime = blink & 0xff;
	printk("ktd20xx led write blink = %x, period = %x, flashtime = %x\n", blink, period, flashtime);
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, period);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, flashtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_PWM1);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)

}

//EXPORT_SYMBOL(ktd22xx_breath_leds);
EXPORT_SYMBOL(ktd2xx_led_on);
EXPORT_SYMBOL(ktd2xx_led_off);
EXPORT_SYMBOL(breath_leds);
EXPORT_SYMBOL(led_on_off);

static ssize_t Breathled_switch_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "%d\n", breath_leds);
}
static ssize_t Breathled_switch_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
    if (NULL == buf)
        return -EINVAL;

    if (0 == count)
        return 0;

	 if(buf[0] == '0'){
		breath_leds = 0;
		led_on_off = 0;
		ktd2xx_led_off();
	 }
	 else if( buf[0] == '1'){
		breath_leds = 1;
		led_on_off = 1;
		ktd22xx_lowbattery_breath_leds();
	 }
	 else if( buf[0] == '2'){
		breath_leds = 2;
		led_on_off = 2;
		ktd22xx_events_breath_leds();
	 }
     else{
		breath_leds = 255;
		led_on_off = 3;
		ktd2xx_led_on();

	 }
	
	return count;

}

static ssize_t Breathled_switch_show2 ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "value :%ld\n", value);
}
static ssize_t Breathled_switch_store2 ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
	//unsigned long value = 0;
	int ret;
	
    if (NULL == buf)
        return -EINVAL;
	 
	ret = kstrtoul(buf, 0, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	//sprintf ( buf, "%d\n", sn3191_breath_led);
	ktd22xx_breath_leds_time(value);
	breath_leds = 1;
	
	return count;

}

static void lcds_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
       //pr_err("%s ktd20xx name:%s value:%d \n", __func__, led_cdev->name, value);
	if(value > ktd22xx_max_brightness)
	 value = ktd22xx_max_brightness;
	if (!strcmp(led_cdev->name, "red")) {
        color_D1 = value;
    }else if (!strcmp(led_cdev->name, "green")) {
        if (LED_OUT_PWM1 & LED2_OUT_PWM1)  //D2 output
        {
			//modify for rendong 2015/11/21
             color_D2 = color_D1; 
			 color_D2 = value;
        }
        else
        {
            color_D2 = value;
        }
    }else if (!strcmp(led_cdev->name, "blue")) { 
        color_D3 = value;
    }else if (!strcmp(led_cdev->name, "white")) { 
        color_D2 = value;
    }
	
	
}

static ssize_t sled_grppwm_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
	onMS = value * totalMS / 255;
	if(onMS > 9000)
	 onMS = 9000;
    return count;
}

static ssize_t sled_grpfreq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
	totalMS = value * 50;
    if(totalMS > 16000)
        totalMS = 16000;
    return count;
}

static ssize_t sled_blink_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
	int totalMS_reg = 0;
	int onMS_reg = 0;
	int stableMS_reg = 0;
	if (~(~value) && totalMS && onMS) {
		totalMS_reg =	(totalMS-256)/128;
		onMS_reg = (onMS/3/192)&0x0F;
		stableMS_reg = onMS*250/3/totalMS*2;
		onMS_reg = onMS_reg|(onMS_reg << 4);
		pr_err("total_MS:%d,onMS:%d,totalMS_reg:%d,onMS_reg:%d,stableMS_reg:%x\n",totalMS,onMS,totalMS_reg,onMS_reg,stableMS_reg);
              pr_err("%s ktd20xx color_D1:%d color_D2:%d color_D3:%d \n", __func__, color_D1, color_D2, color_D3);
		i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// initialization LED off
		i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
		i2c_smbus_write_byte_data(ktd20xx_client, 0x06, color_D1);//set current is D1mA
		i2c_smbus_write_byte_data(ktd20xx_client, 0x07, color_D2);//set current is D2mA
		i2c_smbus_write_byte_data(ktd20xx_client, 0x08, color_D3);//set current is D3mA
		i2c_smbus_write_byte_data(ktd20xx_client, 0x05, onMS_reg);//rase time
		i2c_smbus_write_byte_data(ktd20xx_client, 0x01, totalMS_reg);//dry flash period
		i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter stableMS_reg
		i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_PWM1);//allocate led1 to timer1
		i2c_smbus_write_byte_data(ktd20xx_client, 0x02, stableMS_reg);//led flashing(curerent ramp-up and down countinuously)
	}else {
		if((color_D1+color_D2+color_D3) == 0) {
	  		ktd2xx_led_off();		
		}else {
			i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// initialization LED off
			i2c_smbus_write_byte_data(ktd20xx_client, 0x06, color_D1);//set current is D1mA
			i2c_smbus_write_byte_data(ktd20xx_client, 0x07, color_D2);//set current is D2mA
			i2c_smbus_write_byte_data(ktd20xx_client, 0x08, color_D3);//set current is D3mA
			i2c_smbus_write_byte_data(ktd20xx_client, 0x04, LED_OUT_ALON);//turn om all of leds
		}
	} 
    return count;
}

static ssize_t sled_showing(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    if (value >= 100) {
        return count;
    } else if ( (value < 100 ) && ( value > 20) ) {
        return count;
    } else {
        return count;
    }
}
static ssize_t sled_charging(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);

    if (value >= 100) {
        return count;
    } else {
        return count;
    }
}

static ssize_t sled_reset(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    if (value == 1) {
        return count;
    } else {
        return count;
    }
}

static ssize_t sled_test(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    return count;
}

static DEVICE_ATTR(led, S_IWUSR | S_IRUGO, Breathled_switch_show, Breathled_switch_store);
static DEVICE_ATTR(breath_led, S_IWUSR | S_IRUGO, Breathled_switch_show2, Breathled_switch_store2);
static DEVICE_ATTR(ledtest, S_IWUSR | S_IRUGO, NULL, sled_test);
static DEVICE_ATTR(ledreset, S_IWUSR | S_IRUGO, NULL, sled_reset);
static DEVICE_ATTR(showing, S_IWUSR | S_IRUGO, NULL, sled_showing);
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO, NULL, sled_charging);
static DEVICE_ATTR(grppwm, S_IWUSR | S_IRUGO, NULL, sled_grppwm_store);
static DEVICE_ATTR(grpfreq, S_IWUSR | S_IRUGO, NULL, sled_grpfreq_store);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, NULL, sled_blink_store);

static struct attribute *blink_attributes[] = {
    &dev_attr_grppwm.attr,
    &dev_attr_grpfreq.attr,
    &dev_attr_blink.attr,
    &dev_attr_charging.attr,
    &dev_attr_showing.attr,
    &dev_attr_ledreset.attr,
    &dev_attr_ledtest.attr,

    NULL
};

static const struct attribute_group blink_attr_group = {
    .attrs = blink_attributes,
};

static struct led_classdev KTD2026_lcds[] = {
	#if 0
	//shirendong remove for 15103 only one light
    {
        .name		= "red",
        //.brightness = MAX_BACKLIGHT_BRIGHTNESS,
        .brightness_set = lcds_set_brightness,
    },
    {
        .name		= "green",
        //.brightness = MAX_BACKLIGHT_BRIGHTNESS,
        .brightness_set = lcds_set_brightness,
    },
    {
        .name		= "blue",
        //.brightness = MAX_BACKLIGHT_BRIGHTNESS,
        .brightness_set = lcds_set_brightness,
    },
	#endif
	{
        .name		= "white",
        //.brightness = MAX_BACKLIGHT_BRIGHTNESS,
        .brightness_set = lcds_set_brightness,
    },
	
};
 
static ssize_t ktd20xx_read(struct file *file, char __user *buf, size_t size, loff_t * offset)
{
	return 0;
}

static ssize_t ktd20xx_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{

	static unsigned char rec_data[2];
	memset(rec_data, 0, 2);  
	
	if (copy_from_user(rec_data, buf, 1))  
	{  
		return -EFAULT;  
	}  
	
	if(buf[0] == '0'){
	   breath_leds = 0;
	   led_on_off = 0;
	   ktd2xx_led_off();
	}
	else if( buf[0] == '1'){
	   breath_leds = 1;
	   led_on_off = 1;
	   ktd22xx_lowbattery_breath_leds();
	}
	else if( buf[0] == '2'){
	   breath_leds = 2;
	   led_on_off = 2;
	   ktd22xx_events_breath_leds();
	}
	else{
	   breath_leds = 255;
	   led_on_off = 3;
	   ktd2xx_led_on();
	}
	
    return 1;
}


static struct file_operations ktd20xx_fops = {
	.owner = THIS_MODULE,
	.read  = ktd20xx_read,
	.write = ktd20xx_write,
};

static int ktd20xx_probe(struct i2c_client *client, const struct i2c_device_id *id){

	int ret = -1;
	int i, tmp = 0;
	color_D1 = 0;
	color_D2 = 0;
	color_D3 = 0;
	
	pr_err("oppo_led_debug %s is probe start \n", __func__);

	ktd20xx_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		ret = -ENODEV;
		goto err_i2c_check;
	}
	/* create i2c struct, when receve and transmit some byte */
	//printk("ktd2026 address is %x \n",ktd20xx_client->addr);

    	ret = of_property_read_u32(client->dev.of_node, "ktd2026,max_brightness", &tmp);
	if (!ret) 
       {
           //printk(KERN_ERR"%s val:%d \n", __func__, tmp);
	    ktd22xx_max_brightness = tmp;
       }

	/*************Added by Tong.han for same interface with 14037*******************/
	for(i = 0; i < 1; i ++ ) {
        if (led_classdev_register(&client->dev, &KTD2026_lcds[i])) {
            printk(KERN_ERR "led_classdev_register failed of KTD2026_lcds!\n");    
        }
    }
	ret = sysfs_create_group(&client->dev.kobj, &blink_attr_group);
    if (ret) {
        pr_err( "%s : sysfs_create_group failed!\n", __func__);
        goto err_group_register;
    }	

	//ktd2xx_led_off(); //turn off led when first start ktd
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x05);//reset reg
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0.125mA
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);//turn off leds	
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//set sleep mode	
	if(ret < 0){
		printk("can't find ktd2026 led control ic!");
		goto err_group_register;
	}
	
	major = register_chrdev(0, "ktd20xx", &ktd20xx_fops);
	/* create device node sys/class/ktd20xx/ktd2206/led
	 * led: 0:breath led 1:open led  2:close led
	 */	
	ktd20xx_cls = class_create(THIS_MODULE, "ktd20xx");
	if (IS_ERR(ktd20xx_cls)) {
		ret = IS_ERR(ktd20xx_cls);
		pr_err("perfmap: Error in class_create ktd20xx_cls\n");
		goto err_class_create;

	ktd22xx_dev = device_create(ktd20xx_cls, NULL, MKDEV(major, 0), NULL, "ktd2026"); /* /dev/ktd20xx */

	if (device_create_file(ktd22xx_dev, &dev_attr_led) < 0)
	    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
	if (device_create_file(ktd22xx_dev, &dev_attr_breath_led) < 0)
	    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
	}
	pr_err("oppo_led_debug %s is probe success end \n", __func__);
	return 0;
err_class_create:
	class_destroy(ktd20xx_cls);
err_group_register:
    for(i = 0; i < 1; i ++ )
        led_classdev_unregister(&KTD2026_lcds[i]);	
err_i2c_check:	
	ktd20xx_client = NULL;
	return ret;
}

static int ktd20xx_remove(struct i2c_client *client){

	printk("[%s]: Enter!\n", __func__);
	
	unregister_chrdev(0, "ktd20xx");
	class_destroy(ktd20xx_cls);
		
	return 0;
}
	

static const struct i2c_device_id ktd2xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktd20xx_match_table[] = {
        { .compatible = "ktd,ktd2026",},
        { },
};

static struct i2c_driver ktd20xx_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd20xx_match_table,
	},
	.probe = ktd20xx_probe,
	.remove = ktd20xx_remove,
	.id_table = ktd2xx_id,
};

static int __init ktd20xx_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&ktd20xx_driver);
	if (err) {
		printk(KERN_WARNING "ktd20xx driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd20xx_driver.driver.name);
	}
	return err;
}

static void __exit ktd20xx_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&ktd20xx_driver);
}

module_init(ktd20xx_init);
module_exit(ktd20xx_exit);

MODULE_LICENSE("GPL");

