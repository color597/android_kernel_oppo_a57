/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oppo77_12015\kernel\battery\battery
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      for dc-dc sn111008 charg
** 
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
************************************************************************************************************/


#ifdef CONFIG_OPPO_CHARGER_MTK

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/xlog.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <mach/mt_boot_common.h>

#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>


#endif

#include "../oppo_charger.h"
#include "../oppo_gauge.h"
#include "../oppo_vooc.h"
#include "oppo_vooc_fw.h"

int g_hw_version = 0;
void init_hw_version(void)
{
	

}

void oppo_vooc_data_irq_init(struct oppo_vooc_chip *chip);

// wenbin.liu@BSP.CHG.Vooc, 2016/10/20
// Add for vooc batt 4.40
void oppo_vooc_fw_type_dt(struct oppo_vooc_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc;
	
	chip->batt_type_4400mv = of_property_read_bool(node, "qcom,oppo_batt_4400mv");
	rc = of_property_read_u32(node, "qcom,vooc-fw-type", &chip->vooc_fw_type);
    if (rc) {
        chip->vooc_fw_type = VOOC_FW_TYPE_INVALID;
    }
}

int oppo_vooc_gpio_dt_init(struct oppo_vooc_chip *chip)
{

 	int rc;
    struct device_node *node = chip->dev->of_node;

    
    if (!node) {
        dev_err(chip->dev, "device tree info. missing\n");
        return -EINVAL;
    }

	rc = of_property_read_u32(node, "qcom,vooc_switch1_gpio", &chip->vooc_gpio.switch1_gpio);
    if (rc) {
        chip->vooc_gpio.switch1_gpio = OPPO_VOOC_SW_CTRL_EVT;
    }

	rc = of_property_read_u32(node, "qcom,vooc_switch2_gpio", &chip->vooc_gpio.switch2_gpio);
    if (rc) {
        chip->vooc_gpio.switch2_gpio = OPPO_VOOC_SW_CTRL_DVT;
    }

	rc = of_property_read_u32(node, "qcom,vooc_reset_gpio", &chip->vooc_gpio.reset_gpio);
    if (rc) {
        chip->vooc_gpio.reset_gpio = OPPO_VOOC_RESET_MCU_EN;
    }
	
	rc = of_property_read_u32(node, "qcom,vooc_clock_gpio", &chip->vooc_gpio.clock_gpio);
    if (rc) {
        chip->vooc_gpio.clock_gpio = OPPO_VOOC_MCU_AP_CLK;
    }
	
	rc = of_property_read_u32(node, "qcom,vooc_data_gpio", &chip->vooc_gpio.data_gpio);
    if (rc) {
        chip->vooc_gpio.data_gpio = OPPO_VOOC_MCU_AP_DATA;
    }
	
	oppo_vooc_data_irq_init(chip);
	
	chg_debug( " switch1_gpio = %d,switch2_gpio = %d,reset_gpio = %d,clock_gpio = %d,data_gpio = %d,,data_irq = %d\n", chip->vooc_gpio.switch1_gpio, chip->vooc_gpio.switch2_gpio, chip->vooc_gpio.reset_gpio, chip->vooc_gpio.clock_gpio, chip->vooc_gpio.data_gpio, chip->vooc_gpio.data_irq);
	return rc;
 }

void opchg_set_clock_active(struct oppo_vooc_chip *chip)
{
	if(chip->mcu_boot_by_gpio) {
		chg_debug( " mcu_boot_by_gpio,return\n" );
		return ;
	}
	mt_set_gpio_mode(chip->vooc_gpio.clock_gpio, GPIO_MODE_00);	
	mt_set_gpio_dir(chip->vooc_gpio.clock_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.clock_gpio, 0);
}

void opchg_set_clock_sleep(struct oppo_vooc_chip *chip)
{
	if(chip->mcu_boot_by_gpio) {
		chg_debug( " mcu_boot_by_gpio,return\n" );
		return ;
	}	
	mt_set_gpio_mode(chip->vooc_gpio.clock_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(chip->vooc_gpio.clock_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.clock_gpio, 1);
}

void opchg_set_data_active(struct oppo_vooc_chip *chip)
{
	mt_set_gpio_mode(chip->vooc_gpio.data_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(chip->vooc_gpio.data_gpio, GPIO_DIR_IN);
}

void opchg_set_data_sleep(struct oppo_vooc_chip *chip)
{
	mt_set_gpio_mode(chip->vooc_gpio.data_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(chip->vooc_gpio.data_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.data_gpio, 1);
}

void opchg_set_reset_active(struct oppo_vooc_chip *chip)
{
	if(chip->adapter_update_real == ADAPTER_FW_NEED_UPDATE || chip->btb_temp_over
		|| chip->mcu_update_ing) {
		chg_debug( " adapter_fw_need_update:%d,btb_temp_over:%d,mcu_update_ing:%d,return\n",
			  chip->adapter_update_real, chip->btb_temp_over, chip->mcu_update_ing);
		return ;
	}
	mt_set_gpio_mode(chip->vooc_gpio.reset_gpio, GPIO_MODE_00);	//Set GPIO P9.3 as Output
	mt_set_gpio_dir(chip->vooc_gpio.reset_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.reset_gpio, 0);
	msleep(5);
	mt_set_gpio_out(chip->vooc_gpio.reset_gpio, 1);
	msleep(10);
	mt_set_gpio_out(chip->vooc_gpio.reset_gpio, 0);	
	msleep(5);
	chg_debug( " call !\n" );
}

int oppo_vooc_get_reset_gpio_val(struct oppo_vooc_chip *chip)
{
	return mt_get_gpio_out(chip->vooc_gpio.reset_gpio);
}

bool oppo_is_power_off_charging(struct oppo_vooc_chip *chip)
{
	if(get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT)
		return true;
	else
		return false;
}
bool oppo_is_charger_reboot(struct oppo_vooc_chip *chip)
{
	return false;
}
static void delay_reset_mcu_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_vooc_chip *chip = container_of(dwork,struct oppo_vooc_chip, delay_reset_mcu_work);

	opchg_set_clock_sleep(chip);
	opchg_set_reset_active(chip);
}

void oppo_vooc_delay_reset_mcu_init(struct oppo_vooc_chip *chip)
{
	INIT_DELAYED_WORK(&chip->delay_reset_mcu_work, delay_reset_mcu_work_func);
}

static void oppo_vooc_delay_reset_mcu(struct oppo_vooc_chip *chip)
{
	schedule_delayed_work(&chip->delay_reset_mcu_work, 
				round_jiffies_relative(msecs_to_jiffies(1500)));
}

static bool is_allow_fast_chg_real(void)
{
	//bool auth = false,low_temp_full = false;
	int temp = 0,cap = 0,chg_type = 0;
	
	temp = oppo_chg_get_chg_temperature();
	cap = oppo_chg_get_ui_soc();
	chg_type = oppo_chg_get_chg_type();
	//low_temp_full = oppo_vooc_get_fastchg_low_temp_full();
	//chg_err("temp:%d,cap:%d,chg_type:%d\n",temp,cap,chg_type);

	if(chg_type != POWER_SUPPLY_TYPE_USB_DCP)
		return false;

	if(temp < 165){
		return false;
	}


	if(temp > 430){
		return false;
	}
	if(cap < 1)
		return false;
	if(cap > 85){
		return false;
	}

	if(oppo_vooc_get_fastchg_to_normal() == true){
		chg_debug( "  oppo_vooc_get_fastchg_to_normal is true\n" );
		return false;
	}
	return true;
}

static bool is_allow_fast_chg_dummy(void)
{
	int chg_type = 0;
	bool allow_real = false;

	chg_type = oppo_chg_get_chg_type();
	//chg_err(" chg_type:%d\n",  chg_type);
	if(chg_type != POWER_SUPPLY_TYPE_USB_DCP) 
		return false;
	
	if(oppo_vooc_get_fastchg_to_normal() == true){
		chg_debug( " fast_switch_to_noraml is true\n" );
		return false;
	}

	allow_real = is_allow_fast_chg_real();
	if(oppo_vooc_get_fastchg_dummy_started() == true && (!allow_real)) {
		chg_debug( " dummy_started true,allow_real false\n" );
		return false;
	}
	oppo_vooc_set_fastchg_allow(allow_real);
	return true;
}

void switch_fast_chg(struct oppo_vooc_chip *chip)
{
	bool allow_real = false;
	
	//if(mt_get_gpio_out(chip->vooc_gpio.switch1_gpio) == 1)
	if(chip->dpdm_switch_mode == VOOC_CHARGER_MODE
		&& mt_get_gpio_out(chip->vooc_gpio.switch1_gpio) == 1)
	{
		if(oppo_vooc_get_fastchg_started() == false) {
			allow_real = is_allow_fast_chg_real();
			oppo_vooc_set_fastchg_allow(allow_real);
		}
		return;
	}

	if(is_allow_fast_chg_dummy() == true) {
		if(oppo_vooc_get_adapter_update_status() == ADAPTER_FW_UPDATE_FAIL) {
			opchg_set_switch_mode(chip, VOOC_CHARGER_MODE);
			oppo_vooc_delay_reset_mcu(chip);
		} else {
			if(oppo_vooc_get_fastchg_allow() == false && oppo_vooc_get_fastchg_to_warm() == true) {
				chg_debug( "fastchg_allow false, to_warm true, don't switch to vooc mode\n");
			} else {
				opchg_set_switch_mode(chip, VOOC_CHARGER_MODE);
				opchg_set_clock_sleep(chip);
				opchg_set_reset_active(chip);
			}
		}
	}
	chg_debug( " end,allow_real:%d\n", oppo_vooc_get_fastchg_allow());
}

int oppo_vooc_get_ap_clk_gpio_val(struct oppo_vooc_chip *chip)
{
	return mt_get_gpio_out(chip->vooc_gpio.clock_gpio);
}

int opchg_get_gpio_ap_data(struct oppo_vooc_chip *chip)
{
	return mt_get_gpio_in(chip->vooc_gpio.data_gpio);
}

int opchg_read_ap_data(struct oppo_vooc_chip *chip)
{
	int bit = 0;
	mt_set_gpio_mode(chip->vooc_gpio.clock_gpio, GPIO_MODE_00);	//Set GPIO P9.3 as Output
	mt_set_gpio_mode(chip->vooc_gpio.data_gpio, GPIO_MODE_00);	//Set GPIO P9.3 as Output
	mt_set_gpio_dir(chip->vooc_gpio.clock_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.clock_gpio, 0);
	usleep_range(1000,1000);
	mt_set_gpio_out(chip->vooc_gpio.clock_gpio, 1);
	usleep_range(19000,19000);
	mt_set_gpio_dir(chip->vooc_gpio.data_gpio, GPIO_DIR_IN);
	bit = mt_get_gpio_in(chip->vooc_gpio.data_gpio); 
	return bit;
}

void opchg_reply_mcu_data(struct oppo_vooc_chip *chip, int ret_info,int device_type)
{
	int i = 0;
	
	for(i = 0; i < 3; i++) {
		if(i == 0){	//tell mcu1503 device_type
			mt_set_gpio_out(chip->vooc_gpio.data_gpio, ret_info >> 1);
		} else if(i == 1){
			mt_set_gpio_out(chip->vooc_gpio.data_gpio, ret_info & 0x1);
		} else {
			mt_set_gpio_out(chip->vooc_gpio.data_gpio,device_type);
			chg_debug( "device_type = %d\n",device_type);
		}
		opchg_set_clock_active(chip);
		usleep_range(1000,1000);
		opchg_set_clock_sleep(chip);
		usleep_range(19000,19000);
	}
}

static void opchg_set_switch_fast_charger(struct oppo_vooc_chip *chip)
{
	mt_set_gpio_mode(chip->vooc_gpio.switch1_gpio, GPIO_MODE_00);	//Set GPIO P9.3 as Output
	mt_set_gpio_dir(chip->vooc_gpio.switch1_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.switch1_gpio, 1);
}

static void opchg_set_switch_normal_charger(struct oppo_vooc_chip *chip)
{
	mt_set_gpio_mode(chip->vooc_gpio.switch1_gpio, GPIO_MODE_00);	//Set GPIO P9.3 as Output
	mt_set_gpio_dir(chip->vooc_gpio.switch1_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(chip->vooc_gpio.switch1_gpio, 0);
}

static void opchg_set_switch_earphone(struct oppo_vooc_chip *chip)
{

}


void opchg_set_switch_mode(struct oppo_vooc_chip *chip, int mode)
{
	//chg_err("-------------------------mode = %d\r\n",mode);
	if(chip->adapter_update_real == ADAPTER_FW_NEED_UPDATE || chip->btb_temp_over) {
		chg_debug( " adapter_fw_need_update:%d,btb_temp_over:%d\n",
			  chip->adapter_update_real, chip->btb_temp_over);
		return ;
	}
	if(mode == VOOC_CHARGER_MODE && chip->mcu_update_ing) {
		chg_debug( " mcu_update_ing,don't switch to vooc mode\n" );
		return ;
	}
    switch(mode) {
        case VOOC_CHARGER_MODE:	//11
			opchg_set_switch_fast_charger(chip);
			chg_debug( " vooc mode,switch1_gpio:%d\n" ,mt_get_gpio_out(chip->vooc_gpio.switch1_gpio));
			break;
            
        case HEADPHONE_MODE:		//10
			opchg_set_switch_earphone(chip);
			chg_debug( " headphone mode,switch1_gpio:%d\n" ,mt_get_gpio_out(chip->vooc_gpio.switch1_gpio));
			break;
            
        case NORMAL_CHARGER_MODE:	//01
        default:
			opchg_set_switch_normal_charger(chip);
			chg_debug( " normal mode,switch1_gpio:%d\n" ,mt_get_gpio_out(chip->vooc_gpio.switch1_gpio));
			break;
    }
	chip->dpdm_switch_mode = mode;
}

int oppo_vooc_get_switch_gpio_val(struct oppo_vooc_chip *chip)
{
	return mt_get_gpio_out(chip->vooc_gpio.switch1_gpio);
}

void reset_fastchg_after_usbout(struct oppo_vooc_chip *chip)
{
	if(oppo_vooc_get_fastchg_started() == false) {
		chg_debug( " switch off fastchg\n" );
		opchg_set_switch_mode(chip, NORMAL_CHARGER_MODE);
	}

	oppo_vooc_set_fastchg_to_normal_false();
	oppo_vooc_set_fastchg_to_warm_false();
	oppo_vooc_set_fastchg_low_temp_full_false();
	oppo_vooc_set_fastchg_dummy_started_false();
}

static irqreturn_t irq_rx_handler(int irq, void *dev_id)
{
	oppo_vooc_shedule_fastchg_work();
	return IRQ_HANDLED;
}

void oppo_vooc_data_irq_init(struct oppo_vooc_chip *chip)
{
	struct device_node *node = NULL;
	u32 intr[2] = {0,0};
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, VOOC_AP_DATA-eint");
	if(node){
		of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
		chg_debug( " intr[0]  = %d, intr[1]  = %d\r\n",intr[0] ,intr[1] );
		chip->vooc_gpio.data_irq = irq_of_parse_and_map(node, 0);
	}else{
		chg_err(" node not exist!\r\n");
		chip->vooc_gpio.data_irq = CUST_EINT_MCU_AP_DATA;
	}	
}

void oppo_vooc_eint_register(struct oppo_vooc_chip *chip)
{
	static int register_status = 0;
	int ret = 0;
	if(!register_status){
	
	opchg_set_data_active(chip);
	ret = request_irq(chip->vooc_gpio.data_irq, (irq_handler_t)irq_rx_handler, IRQF_TRIGGER_RISING, "VOOC_AP_DATA-eint",  NULL);
	if(ret){
		chg_err("ret = %d, oppo_vooc_eint_register failed to request_irq \n", ret);
	}
	register_status = 1;
	}
	else{
	chg_debug( " enable_irq!\r\n");
	enable_irq(chip->vooc_gpio.data_irq);
	}
}

void oppo_vooc_eint_unregister(struct oppo_vooc_chip *chip)
{
		chg_debug( " disable_irq!\r\n");
	disable_irq(chip->vooc_gpio.data_irq);
	//free_irq(chip->vooc_gpio.data_irq, chip);
}


