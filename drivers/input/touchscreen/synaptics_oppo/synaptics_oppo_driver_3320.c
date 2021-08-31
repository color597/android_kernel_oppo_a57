/************************************************************************************
** File: - /android/kernel/drivers/input/touchscreen/synaptic_s3203_13095/synaptics_s3203_13095.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,18/01/2012
** Author: Yixue.Ge@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
**  chenggang.li@BSP.TP modified for oppo 2014-07-30 14005 tp_driver
**  Ping.Liu@BSP.Group.Tp,       2016/09/28,       modify for key event, temp force baseline test pass.
**  Ping.Liu@BSP.Group.Tp,       2016/10/02,       modify for that it need to power off when ftm mode.
**  Ping.Liu@BSP.Group.Tp,       2016/10/07,       modify for baseline test ,include virtual key.
**  Ping.Liu@BSP.Group.Tp,       2016/12/15,       modify for hard reset when i2c error in resume , for esd issue.
************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
//#include <mach/device_info.h>
#include <soc/oppo/boot_mode.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/device_info.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#ifdef CONFIG_FB
	#include <linux/fb.h>
	#include <linux/notifier.h>
#endif

#include "oppo_tp_devices.h"

#include <linux/input/mt.h>

#include "synaptics_redremote.h"

/*------------------------------------------------Global Define--------------------------------------------*/
#define PAGESIZE 512
#define VKNUMBER 3
#define TPD_USE_EINT

#define TPD_DEVICE "synaptics-s3320"

#define IMAGE_AREA_OFFSET 0x100

//#define KEY_USE

//#define SUPPORT_14017
#define SUPPORT_GESTURE
#define RESET_ONESECOND
//#define SUPPORT_GLOVES_MODE

#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation
#define TP_FW_NAME_MAX_LEN 128

#define TEST_MAGIC1 0x494D494C
#define TEST_MAGIC2 0x474D4954
#define FINGER_PROTECT

#define I2C_ERROR_MAX_TIME 5


#undef TP_TEST_STATION   //guomingqiang@phone.bsp 2016-4-15 add for tp test beforce assemble a mobile phone 

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
	#define ENABLE_UNICODE  0x40
	#define ENABLE_VEE      0x20
	#define ENABLE_CIRCLE   0x08
	#define ENABLE_SWIPE    0x02
	#define ENABLE_DTAP     0x01

	#define UNICODE_DETECT  0x0b
	#define VEE_DETECT      0x0a
	#define CIRCLE_DETECT   0x08
	#define SWIPE_DETECT    0x07
	#define DTAP_DETECT     0x03
	

	#define UnkownGestrue       0
	#define DouTap              1   // double tap
	#define UpVee               2   // V
	#define DownVee             3   // ^
	#define LeftVee             4   // >
	#define RightVee            5   // <
	#define Circle              6   // O
	#define DouSwip             7   // ||
	#define Left2RightSwip      8   // -->
	#define Right2LeftSwip      9   // <--
	#define Up2DownSwip         10  // |v
	#define Down2UpSwip         11  // |^
	#define Mgestrue            12  // M
	#define Wgestrue            13  // W
#endif

/*********************for Debug LOG switch*******************/
#define TPD_DAILY(a,arg...)\
	do{\
		if(tp_debug_daily_build)\
			pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)	


#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)	
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)	

/*---------------------------------------------Global Variable----------------------------------------------*/
static int baseline_ret = 0;
static int TP_FW;
static int tp_dev = 2;
static unsigned int tp_debug = 0;
static int button_map[3];
static int tx_rx_num[2];
static int16_t Rxdata[30][30];
static int16_t delta_baseline[30][30];	
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y = 0;
static int force_update = 0;
extern int is_touch;
static int LCD_WIDTH ;
static int LCD_HEIGHT ;
static struct workqueue_struct *speedup_resume_wq = NULL;
#ifdef FINGER_PROTECT
static int finger_trigger = 0;
static int i2c_ready = 1;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct task_struct *thread = NULL;
static int report_finger_protect = 0;
enum finger_protect_status{
	FINGER_PROTECT_TOUCH_UP,
	FINGER_PROTECT_TOUCH_DOWN,
	FINGER_PROTECT_NOTREADY
};
enum finger_protect_trigger_status{
	FINGER_PROTECT_NOTTRIGGER,
	FINGER_PROTECT_ISTRIGGER
};
enum i2c_ready_status{
	I2C_NOTREADY,
	I2C_READY
};
#endif

#if defined(CONFIG_OPPO_DAILY_BUILD)
	static int tp_debug_daily_build = 1;
#else
	static int tp_debug_daily_build = 0;
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static int sleep_enable;
#endif

static struct manufacture_info tp_info;
static struct synaptics_ts_data *ts_g = NULL;
static struct workqueue_struct *synaptics_wq = NULL;
static struct proc_dir_entry *prEntry_tp = NULL; 
//mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
static int touch_major_report_num  = 0;
//end 

#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static uint32_t gesture_upload;

/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F12_2D_QUERY_BASE;
static int F12_2D_CMD_BASE;
static int F12_2D_CTRL_BASE;
static int F12_2D_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;
static int F01_RMI_CTRL02;

static int F12_2D_CTRL08;
static int F12_2D_CTRL32;
static int F12_2D_DATA04;
static int F12_2D_DATA38;
static int F12_2D_DATA39;
static int F12_2D_CMD00;	
static int F12_2D_CTRL20;
static int F12_2D_CTRL27;

static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_CTRL31;
static int F51_CUSTOM_DATA04;
static int F51_CUSTOM_DATA11;

#if TP_TEST_ENABLE
	static int F54_ANALOG_QUERY_BASE;//0x73
	static int F54_ANALOG_COMMAND_BASE;//0x72
	static int F54_ANALOG_CONTROL_BASE;//0x0d
	static int F54_ANALOG_DATA_BASE;//0x00
#endif

/***********for example of key event***********/
#ifdef KEY_USE
static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU, 90, 2050, 180, 100},
	{KEY_HOME, 500, 2050, 180, 100},
	{KEY_BACK, 855, 2050, 180, 100},
};
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static void speedup_synaptics_resume(struct work_struct *work);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev, char *buf, bool savefile)	;
static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);
static int synaptics_free_fingers(struct synaptics_ts_data *ts);

static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data);
		
static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
	unsigned char addr, unsigned short length, unsigned char const *data);
	
static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr);

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data);

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr);
		
static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data);

#ifdef TPD_USE_EINT
	static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB)
	static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

extern void judge_key_press_all_the_time(void);
static void checkCMD(void);

/*-------------------------------Using Struct----------------------------------*/
struct point_info {
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
	int status; //mingqiang.guo@phone.bsp.tp  add for auto report (0,0)
};

static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptics_match_table[] = {
	{ .compatible = TPD_DEVICE,},//"oppo,synaptics14045",},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_FB
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#endif
};	



static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,                         
	.id_table	= synaptics_ts_id,
	.driver = {
//		.owner  = THIS_MODULE,
		.name	= TPD_DEVICE,
		.of_match_table =  synaptics_match_table,
		.pm = &synaptic_pm_ops,
	},
};



struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	bool glove_mode_enabled;
	bool black_gesture_support;
	bool charger_pump_support;
	int irq;
	int irq_gpio;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int enable1v8_gpio;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;	
	int boot_mode;
	uint32_t irq_flags;
	uint32_t max_x;
    uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
    uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct input_dev *kpd;
	struct work_struct  work;
	struct work_struct speed_up_work;
	struct work_struct close_edge_product;
	struct input_dev *input_dev;	
	struct hrtimer timer;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	/******gesture*******/
	int double_enable;
	int gesture_enable;
	int glove_enable;
	int is_suspended;
	
	/********test*******/
	int i2c_device_test;
	
	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;
	
	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;	
	struct pinctrl_state 				*id_pullup;
	struct pinctrl_state 				*id_pulldown;

	/*******for FW update*******/
	bool suspended;
	bool loading_fw;
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[12];
	#ifdef FINGER_PROTECT
        uint8_t raw_data[1536];
	#endif
};

/*Virtual Keys Setting Start*/
static struct kobject *syna_properties_kobj;
static struct kobj_attribute qrd_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *qrd_properties_attrs[] = {
    &qrd_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group qrd_properties_attr_group = {
    .attrs = qrd_properties_attrs,
};

/*Virtual Keys Setting End*/
static struct device_attribute attrs_oppo[] = {
//	__ATTR(baseline_test, 0664, synaptics_rmi4_baseline_show, NULL),
	__ATTR(vendor_id, 0664, synaptics_rmi4_vendor_id_show, NULL),
 };
 
/*---------------------------------------------Fuction Apply------------------------------------------------*/
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
	return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
        "\n",LCD_WIDTH/6,button_map[2],button_map[0],button_map[1],LCD_WIDTH/2,button_map[2],button_map[0],button_map[1],LCD_WIDTH*5/6,button_map[2],button_map[0],button_map[1]);
}

static int synaptics_tpd_button_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
	ts->kpd = input_allocate_device();
    if( ts->kpd == NULL ){
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
    ts->kpd->id.vendor  = 0x0001;
    ts->kpd->id.product = 0x0001;
    ts->kpd->id.version = 0x0100;
	
	if(input_register_device(ts->kpd)){
        TPDTM_DMESG("input_register_device failed.(kpd)\n");	
		input_unregister_device(ts->kpd);
		input_free_device(ts->kpd);
	}
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
    syna_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if( syna_properties_kobj )
        ret = sysfs_create_group(syna_properties_kobj, &qrd_properties_attr_group);
    if( !syna_properties_kobj || ret )
		printk("failed to create board_properties\n");	

	err_input_dev_alloc_failed:		
		return ret;
}

static int Dot_report_down = 0;
static void tpd_down(struct synaptics_ts_data *ts,int raw_x, int raw_y, int x, int y, int p) 
{
	static int last_report_abs_mt_width_major;
	if( ts && ts->input_dev ){
        input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);
		
		if(ts->boot_mode == MSM_BOOT_MODE__RECOVERY)
		{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
		}
		else
		{
			 //if( (touch_major_report_num < 10) || !(touch_major_report_num&0x1ff) )
			 if( touch_major_report_num == 1){
				 last_report_abs_mt_width_major = (raw_x+raw_y)/2;
				 input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, last_report_abs_mt_width_major); //add 1 , ABS_MT_WIDTH_MAJOR change ,can report to user space 
			 }
			 else if(!(touch_major_report_num&0x7f))
			 {   //down and keep long time , auto repeat per 1.28 seconds ,for weixing
			 	 if(last_report_abs_mt_width_major == (raw_x+raw_y)/2) //make sure ABS_MT_WIDTH_MAJOR change
		 	 	 {
					last_report_abs_mt_width_major = (raw_x+raw_y)/2+1;
		 	 	 }
				 else
				 {
					last_report_abs_mt_width_major = (raw_x+raw_y)/2;
				 }
				 input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, last_report_abs_mt_width_major);
			 }
		}

	    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		TPD_DEBUG("Synaptics:Down[%4d %4d %4d]\n", x, y, p);
		if( Dot_report_down == 150 ){
			TPD_ERR("Synaptics:Down[%4d %4d %4d]\n", x, y, p);
			Dot_report_down = 0;
		}else{
			Dot_report_down++;
		} 
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(ts->input_dev);
#endif	
    }  
}

static int Dot_report_up = 0;
static void tpd_up(struct synaptics_ts_data *ts, int raw_x, int raw_y, int x, int y, int p) {	
	if( ts && ts->input_dev ){
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
		TPD_DEBUG("Up[%4d %4d %4d]\n", x, y, p);	
		if( Dot_report_up == 150 ){
			TPD_ERR("Up[%4d %4d %4d]\n", x, y, p);	
			Dot_report_up = 0;
		}else{
			Dot_report_up++;
		}
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(ts->input_dev);
#endif
    }  
}

#ifdef KEY_USE
static void tpd_button(struct synaptics_ts_data *ts,
    unsigned int x, unsigned int y, unsigned int down){
    int i;
	if( ts->max_x == 1145 ){
	    if( down ){
	        for(i = 0; i < VKNUMBER; i++){
	            if( x >= tpd_keys[i][1] &&
	                x <= tpd_keys[i][1]+tpd_keys[i][3] &&
	                y >= tpd_keys[i][2]&&
	                y <= tpd_keys[i][2]+tpd_keys[i][4] &&
	                !( ts->btn_state & (1 << i) ) ) {
	                input_report_key( ts->input_dev, tpd_keys[i][0], 1);
	                ts->btn_state |= ( 1<<i );
	            }
	        }
	    }else{
	        for(i=0; i<4; i++){
	            if( ts->btn_state&( 1<<i ) ){
	                input_report_key(ts->input_dev, tpd_keys[i][0], 0);
	            }
	        }
	        ts->btn_state = 0;
	    }
	}
}
#endif

int is_gesture_enable_for_lcd(void)
{
    int support_gesture_for_lcd = 0;

    if (ts_g == NULL)
    {
        support_gesture_for_lcd = 0;
        return support_gesture_for_lcd;
    }

    #ifdef SUPPORT_GESTURE
    if((1 == ts_g->double_enable) || (2 == ts_g->double_enable)) {
        support_gesture_for_lcd = 1;
    } else {
        support_gesture_for_lcd = 0;
    }
    #else
        support_gesture_for_lcd = 0;
    #endif

    TPD_DEBUG("lcd get gesture type, support_gesture_for_lcd is %d\n", support_gesture_for_lcd);
    return support_gesture_for_lcd;
}

static int tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;

	/***enable the 2v8 power*****/
	if (!IS_ERR(ts->vdd_2v8)) {
		//regulator_set_optimum_mode(ts->vdd_2v8,100000);
		rc = regulator_enable(ts->vdd_2v8);
		if(rc){
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
				//return rc;
		}
	}

	msleep(1);
	
	if( ts->enable2v8_gpio > 0 ) {	
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 1);		
	}
	
	if( ts->enable1v8_gpio > 0 ) {	
		TPD_DEBUG("synaptics:enable the enable1v8_gpio\n");
		gpio_direction_output(ts->enable1v8_gpio, 1);		
	}
	
	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		//regulator_set_optimum_mode(ts->vcc_i2c_1v8,100000);
		rc = regulator_enable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	
	if( ts->reset_gpio > 0 ) {	
		TPD_DAILY("synaptics:enable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 1);		
	}
	
	msleep(80); //wanghao@phone.bsp , 2016/7/11 modify synaptics tp vender need delay 80ms 
	
	return rc;
}

static int tpd_hw_pwroff(struct synaptics_ts_data *ts)
{ 
	int rc = 0;
	if( ts->reset_gpio > 0 ) {
		TPD_DEBUG("synaptics:disable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 0);
	}
	
	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		//regulator_set_optimum_mode(ts->vcc_i2c_1v8,100);	
		rc = regulator_disable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	
	if (!IS_ERR(ts->vdd_2v8)) {
		//regulator_set_optimum_mode(ts->vdd_2v8,100);
		rc = regulator_disable(ts->vdd_2v8);
		if (rc) {
			dev_err(&ts->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			//return rc;
		}
	}

   	if( ts->enable1v8_gpio > 0 ) {	
		TPD_DEBUG("synaptics:disable the enable1v8_gpio\n");
		gpio_direction_output(ts->enable1v8_gpio, 0);		
		mdelay(5);
		TPD_ERR("enable1v8_gpio: %d\n", __gpio_get_value(ts->enable1v8_gpio));
	}
	
	if( ts->enable2v8_gpio > 0 ) {	
		TPD_DEBUG("synaptics:disable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 0);		
		mdelay(5);
		TPD_ERR("enable2v8_gpio: %d\n", __gpio_get_value(ts->enable2v8_gpio));
	}
	return rc;
}

static int tpd_power(struct synaptics_ts_data *ts, unsigned int on)
{
	int ret;
	if(on){
		ret = tpd_hw_pwron(ts);		
	}else{
		ret = tpd_hw_pwroff(ts);
	}		
	return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];   
	int ret;
	memset(buf, 0, sizeof(buf));
   	ret = synaptics_rmi4_i2c_write_byte( ts->client, 0xff, 0x0 ); 
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}

	F12_2D_QUERY_BASE = buf[0];
	F12_2D_CMD_BASE = buf[1];
	F12_2D_CTRL_BASE = buf[2]; 
	F12_2D_DATA_BASE = buf[3];

	TPD_DEBUG("F12_2D_QUERY_BASE = %x \n \
			F12_2D_CMD_BASE  = %x \n\
			F12_2D_CTRL_BASE	= %x \n\
			F12_2D_DATA_BASE	= %x \n\
			",F12_2D_QUERY_BASE,F12_2D_CMD_BASE,F12_2D_CTRL_BASE,F12_2D_DATA_BASE);

	
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE3, 4, &(buf[0x0]));    
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2]; 
	F01_RMI_DATA_BASE = buf[3];
    TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			   F01_RMI_CMD_BASE  = %x \n\
		       F01_RMI_CTRL_BASE	= %x \n\
		       F01_RMI_DATA_BASE	= %x \n\
		       ", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL_BASE, F01_RMI_DATA_BASE);

	ret = synaptics_rmi4_i2c_read_block( ts->client, 0xE9, 4, &(buf[0x0]) );	  
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2]; 
	F34_FLASH_DATA_BASE = buf[3];
	TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
			  F34_FLASH_CMD_BASE	= %x \n\
				F34_FLASH_CTRL_BASE	= %x \n\
				F34_FLASH_DATA_BASE	= %x \n\
			   ", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);
	
	F01_RMI_QUERY11 = F12_2D_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CTRL02 = F01_RMI_CTRL_BASE + 2;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1; 
	
	F12_2D_CTRL08 = F12_2D_CTRL_BASE;
	F12_2D_CTRL32 = F12_2D_CTRL_BASE + 15;
	F12_2D_DATA38 = F12_2D_DATA_BASE + 54;
	F12_2D_DATA39 = F12_2D_DATA_BASE + 55;
	F12_2D_CMD00 = F12_2D_CMD_BASE;
	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x0c;


	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
	
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4); 
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0])); 	   
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2]; 
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA04 = F51_CUSTOM_DATA_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;
	F51_CUSTOM_CTRL31 = F51_CUSTOM_CTRL_BASE + 11;

	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			   F51_CUSTOM_CMD_BASE  = %x \n\
			   F51_CUSTOM_CTRL_BASE    = %x \n\
			   F51_CUSTOM_DATA_BASE    = %x \n\
			  ", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);  	
			  
#if TP_TEST_ENABLE	
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01); 
	if(ret < 0) {
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0])); 
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_DEBUG("F54_QUERY_BASE = %x \n\
			 F54_CMD_BASE  = %x \n\
			 F54_CTRL_BASE	= %x \n\
			 F54_DATA_BASE	= %x \n\
				   ", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE , F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);	
#endif
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00); 	
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	unsigned char reportbuf[3];
	unsigned char val[4];
	TPD_DEBUG("%s is called\n", __func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0); 
	if( ret < 0 ) {
		TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ) {
		TPD_DEBUG("read reg F12_2D_CTRL20 failed\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL27, 4, &(val[0x0]) );
	val[0] = 0xef;
	ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL27, 4, &(val[0x0]) );
	
	
	if( enable ) {
		reportbuf[2] |= 0x02 ;
	} else {
		reportbuf[2] &= 0xfd ;
	}
	TPD_DEBUG("%s:reportbuf[2] = 0x%x\n", __func__, reportbuf[2]);
	
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0); 
	if( ret < 0 ) {
		TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}	
	ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("%s :Failed to write report buffer\n", __func__);
		return -1;
	}		
	gesture = UnkownGestrue;	
	ts->gesture_enable = enable;
	return 0;	
}
#endif

#ifdef SUPPORT_GLOVES_MODE
static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;	
	TPD_DEBUG("glove mode enable\n");
	/* page select = 0x4 */	
	if( 1 == ts->glove_enable)  {	       
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, 0x001E);
		//TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n", ret, ret|0x20);
		ret = i2c_smbus_write_byte_data(ts->client, 0x001E, ret | 0x20);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}else{
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, 0x001E);
		ret = i2c_smbus_write_byte_data(ts->client, 0x001E, ret & 0xdf);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}			
	}
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		goto GLOVE_ENABLE_END;
	}
	
GLOVE_ENABLE_END: 
	return ret;
}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	if( 1 == sleep_enable ){	    
	/*0x00:enable glove mode,0x02:disable glove mode,*/
	    TPDTM_DMESG("sleep mode enable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01 ); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}else{		  
	    TPDTM_DMESG("sleep mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); //chenggang.li@BSP add for oppo TP NO SLEEP set 1
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	
SLEEP_ENABLE_END:
	return ret;
}
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;
	
	memset(buf1, 0 , sizeof(buf1));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY_BASE+11, 8, &(buf1[0x0]));
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));	
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	return 0;
}

static int limit_enable = 1;
static int limit_area = 1;
static int left_x1, left_x2;
static int right_x1, right_x2;
static int in_which_area = 0;
static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
	int ret = 0;
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret < 0){
        TPD_ERR("%s error ret=%d\n",__func__,ret);
    }

	msleep(20);

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x04);
	if(0 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x46);
	}
	else if(1 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x47);
	}
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0); 

    TPD_DAILY("%s !!!\n",__func__);
    return ret;
}

static int synaptics_soft_reset_for_prepare(struct synaptics_ts_data *ts)
{
    int ret = 0;
    int i2c_error_number = 0;

    ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
    while(ret && i2c_error_number < I2C_ERROR_MAX_TIME){
        ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
        if (ret){
            i2c_error_number++;
            TPD_ERR("%s i2c error i2c_error_number:%d \n",__func__, i2c_error_number);
        }
    }

    if(i2c_error_number >= I2C_ERROR_MAX_TIME && ts->reset_gpio > 0){
        TPD_ERR("%s reseet gpio \n",__func__);
        gpio_direction_output(ts->reset_gpio, 0);
        msleep(10);
        gpio_direction_output(ts->reset_gpio, 1);
        msleep(10);

        ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
        if (ret){
            TPD_ERR("%s after reset gpio, i2c error!!!!!!! \n",__func__);
        }
    }

    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret < 0){
        TPD_ERR("%s error ret=%d\n",__func__,ret);
    }

    TPD_DAILY("%s !!!\n",__func__);
    return ret;
}


static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	int read_value;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/\
	read_value = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00);
    	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, (uint8_t)(read_value)|0x80);// chenggang.li @BSP change 0x80 to 0x84
	if( ret < 0 ){
		msleep(150);
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, (uint8_t)(read_value)|0x80); 
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	} 

	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0); 
	if( ret < 0 ) {
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
	if( enable ) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;		
	}	
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00+1);
	return 0;	
}
/*
static void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;
    for(i = 0; i < w_ms; i++) {
        for (j = 0; j < 1000; j++) {
            udelay(1);
        }
    }
}
*/
static void int_state(struct synaptics_ts_data *ts)
{
    int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	if(ret) {
		TPD_ERR("int_state:cannot reset touch panel \n");
		return;
	}
	msleep(170);
#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts);
#endif

	synaptics_init_panel(ts);
	if( ret < 0 ) {
		TPD_DEBUG("int_state: control tm1400 to sleep failed\n");
		return;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_DEBUG("int_state:cannot  enable interrupt \n");
		return;
	}
	if(ts->gesture_enable == 1) {
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
		if (ret < 0){
			TPD_ERR("%s page select failed\n", __func__);
		}else{
			ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL02, 0x03);
		}	
		synaptics_enable_interrupt_for_gesture(ts, 1);	
	}
}
	
//Added for larger than 32 length read!
static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf;	
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for( retry = 0; retry < 2; retry++ ) {
		if( i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if( retry == 2 ) {
		dev_err(&client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		//rst_flag_counter = 1;//reset tp		
		retval = -5;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
	unsigned char addr, unsigned short length, unsigned char const *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr & 0xff;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if (retry == 2) {
		//rst_flag_counter = 1;//rest tp
		retval = -EIO;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr)
{
	int retval = 0;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,1,buf);
	if(retval >= 0)
		retval = buf[0]&0xff;	
	return retval;
}	

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data)
{
	int retval;
	unsigned char data_send = data;
	retval = synaptics_rmi4_i2c_write_block(client,addr,1,&data_send);
	return retval;
}

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr)
{
	int retval;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];	
	return retval;
}

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data)
{
	int retval;
	unsigned char buf[2] = {data&0xff,(data>>8)&0xff};
	retval = synaptics_rmi4_i2c_write_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];	
	return retval;
}		

//chenggang.li@BSP.TP modified for oppo 2014-08-05 gesture_judge
/***************start****************/
#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
    int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11, 8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16, 8, &(coordinate_buf[16])); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24, 1, &(coordinate_buf[24]));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
    }	
	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (ts->max_y);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (ts->max_y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 :
                                    (coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
	TPD_DAILY("%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture,
               Point_start.x, Point_start.y, Point_end.x, Point_end.y,
               Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
               Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
               clockwise);
}

static void gesture_judge(struct synaptics_ts_data *ts)
{
	int ret = 0,gesture_sign, regswipe;
	uint8_t gesture_buffer[10];
	unsigned char reportbuf[3];
	F12_2D_DATA04 = 0x0007;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_read_i2c_block_data(ts->client,  F12_2D_DATA04, 5, &(gesture_buffer[0]));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
	regswipe = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA04+0x18);
	TPDTM_DMESG(" gesture_buffer[0] = 0x%x, regswipe = 0x%x,gesture_buffer[1] = 0x%x, gesture_buffer[4] = 0x%x\n", gesture_buffer[0], regswipe, gesture_buffer[1], gesture_buffer[4]);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	gesture_sign = gesture_buffer[0];
	//detect the gesture mode
	switch (gesture_sign) {
		case DTAP_DETECT:
			gesture = DouTap;
			break;
		case SWIPE_DETECT:
			gesture =   (regswipe == 0x41) ? Left2RightSwip   :
						(regswipe == 0x42) ? Right2LeftSwip   :
						(regswipe == 0x44) ? Up2DownSwip      :
						(regswipe == 0x48) ? Down2UpSwip      :
						(regswipe == 0x80) ? DouSwip          :
						UnkownGestrue;
			break;
		case CIRCLE_DETECT:
			gesture = Circle;
			break;
		case VEE_DETECT:
			gesture =   (gesture_buffer[2] == 0x01) ? DownVee  :
						(gesture_buffer[2] == 0x02) ? UpVee    :
						(gesture_buffer[2] == 0x04) ? RightVee :
						(gesture_buffer[2] == 0x08) ? LeftVee  : 
						UnkownGestrue;
			break;
		case UNICODE_DETECT:
			gesture =   (gesture_buffer[2] == 0x77 && gesture_buffer[3] == 0x00) ? Wgestrue :
						(gesture_buffer[2] == 0x6d && gesture_buffer[3] == 0x00) ? Mgestrue :
						UnkownGestrue;
			break;
		default:
			gesture = UnkownGestrue;
	} 

	TPD_ERR("detect %s gesture\n", gesture == DouTap ? "double tap" :
													gesture == UpVee ? "up vee" :
													gesture == DownVee ? "down vee" :
													gesture == LeftVee ? "(>)" :
													gesture == RightVee ? "(<)" :
													gesture == Circle ? "circle" :
													gesture == DouSwip ? "(||)" :
													gesture == Left2RightSwip ? "(-->)" :
													gesture == Right2LeftSwip ? "(<--)" :
													gesture == Up2DownSwip ? "up to down |" :
													gesture == Down2UpSwip ? "down to up |" :
													gesture == Mgestrue ? "(M)" :
													gesture == Wgestrue ? "(W)" : "unknown");
    if(gesture != UnkownGestrue ){
			synaptics_get_coordinate_point(ts);
			gesture_upload = gesture;
			input_report_key(ts->input_dev, KEY_F4, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F4, 0);
			input_sync(ts->input_dev);
    }else{
		ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
		ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		ret = reportbuf[2] & 0x20;
		if(ret == 0)
			reportbuf[2] |= 0x02 ;
		ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) ); //enable gesture
		if( ret < 0 ){
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
			return;
		}
	}	   
} 
#endif
/***************end****************/



static void int_touch(struct synaptics_ts_data *ts)
{
 	int ret = -1,i = 0;
    uint8_t buf[80];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
	int obj_attention =0;
    struct point_info points;
	
    memset(buf, 0, sizeof(buf));
	points.x = 0;
	points.y = 0;
	points.z = 0;
	obj_attention = i2c_smbus_read_word_data(ts->client, F12_2D_DATA_BASE+0x03);
	if( (obj_attention&0x03FF) != 0 ){	
		/* mingqiang.guo add  2015-11-19  Start checking from the highest bit */	
		unsigned char fingers_to_process;
		fingers_to_process =  ts->max_num;
		for( i = 9;  ; i--)
		{
			if (  (obj_attention&0x03FF)>>i  || i==0 )
				break;
			else
				fingers_to_process--;	
		}
		ret = synaptics_rmi4_i2c_read_block(ts->client, F12_2D_DATA_BASE, 8*fingers_to_process, buf);
		if (ret < 0) {
			TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
			return;
		}		
		for( i = 0; i < ts->max_num; i++ ) { 
			points.status = buf[i*8]; //mingqiang.guo@phone.bsp.tp add for auto report (0,0), point buf[0]==0 ,no point 
		
		   if(((obj_attention&0x03FF) >> i )&0x01   &&  ( points.status == 0 )  ) // buf[0] ==0 is wrong point ,no process
		   		continue;
			if(((obj_attention&0x03FF) >> i )&0x01   &&  ( points.status != 0 )  ) //mingqiang.guo modify for reslove dirty touch point
			{
				points.x = ((buf[i*8+2]&0x0f)<<8) | (buf[i*8+1] & 0xff);
				points.raw_x = buf[i*8+6] & 0x0f;
				points.y = ((buf[i*8+4]&0x0f)<<8) | (buf[i*8+3] & 0xff);
				points.raw_y = buf[i*8+7] & 0x0f;
				points.z = buf[i*8+5];
				
				if(limit_enable)
				{
					if(points.x > left_x2 && points.x < right_x2)		//area 3
					{
						if(1 == in_which_area )
						{
							synaptics_free_fingers(ts);
							//TPD_ERR("in_which_area is 1 and this is 3 so up all finger\n");
						}
						in_which_area = 3;
						//TPD_ERR("in_which_area is %d\n", in_which_area);
					}

					if((points.x > left_x1 && points.x < left_x2) || (points.x >right_x2 && points.x < right_x1))	//area2
					{
						if(1 == in_which_area ){
							in_which_area = 2;
						}
						//TPD_ERR("in_which_area is %d\n", in_which_area);
					}
					
					if(points.x < left_x1 || points.x > right_x1)		//area 1
					{
						if(2 == in_which_area)
						{
							in_which_area = 1;
							continue;
						}
						if(3 == in_which_area )
							continue;
						in_which_area = 1;
						//TPD_ERR("in_which_area is %d\n", in_which_area);
					}
				}
				
#ifdef TYPE_B_PROTOCOL
/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 1);
#endif				
					//mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
				    touch_major_report_num++;
					tpd_down(ts, points.raw_x, points.raw_y, points.x, points.y, points.z);		
					finger_num++;
					
				}
		#ifdef TYPE_B_PROTOCOL
				else{
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
				}	
		#endif		
		}
	}else{
	    //mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
		touch_major_report_num = 0; 
		//end 
#ifdef TYPE_B_PROTOCOL
		for (i = 0; i < ts->max_num; i++) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
		}
#endif
		tpd_up(ts, points.raw_x, points.raw_y, points.x, points.y, points.z);	
		in_which_area = 0;
	}			
	is_touch = finger_num;
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state; 
	
#ifdef SUPPORT_GESTURE
	if( ts->gesture_enable == 1) {
		if(ts->is_suspended == 1) {
			gesture_judge(ts);
		} else {
			TPD_ERR("synaptics touchpanel only goes this for get in gesture mode after resume\n");
			ts->gesture_enable = 0;
			int_state(ts);
		}
	}
#endif	

//mingqiang.guo@phone.bsp  2016-1-7 add for judge key press all time ,can not touch up, need  force cal 
#if 0 //close this function  
	if(is_touch)
		judge_key_press_all_the_time();
#endif 
//end

}

static void int_key_report(struct synaptics_ts_data *ts)
{
    int ret= 0;
	int F1A_0D_DATA00=0x00;
	int F51_CUSTOM_DATA32=0x19;

	TPD_DEBUG("%s is called!\n",__func__);
	if (ts->gesture_enable == 1) {
		i2c_smbus_write_byte_data(ts->client, 0xff, 0x04);
		ret = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA32);
		if( (ret & 0x07) != 0 ){
			gesture_upload = DouTap;
			input_report_key(ts->input_dev, KEY_F4, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F4, 0);
			input_sync(ts->input_dev);
		}
	}else{
		i2c_smbus_write_byte_data(ts->client, 0xff, 0x02);
		ret = i2c_smbus_read_byte_data(ts->client, F1A_0D_DATA00);
        TPD_ERR("%s ret = %d pre_btn_state = %d\n", __func__, ret, ts->pre_btn_state);
		if((ret & 0x01) && !(ts->pre_btn_state & 0x01))//menu
		{
			if( 0 == is_touch ){
				input_report_key(ts->input_dev, KEY_MENU, 1);
				input_sync(ts->input_dev);
			}
		}else if(!(ret & 0x01) && (ts->pre_btn_state & 0x01)){
			input_report_key(ts->input_dev, KEY_MENU, 0);
			input_sync(ts->input_dev);
		}

		if((ret & 0x02) && !(ts->pre_btn_state & 0x02))//back
		{
			if( 0 == is_touch ){
				input_report_key(ts->input_dev, KEY_BACK, 1);
				input_sync(ts->input_dev);
			}

		}else if(!(ret & 0x02) && (ts->pre_btn_state & 0x02)){
			input_report_key(ts->input_dev, KEY_BACK, 0);
			input_sync(ts->input_dev);
		}

		if((ret & 0x04) && !(ts->pre_btn_state & 0x04))//home
		{
			if( 0 == is_touch ){
				input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);
				input_sync(ts->input_dev);
			}
		}else if(!(ret & 0x04) && (ts->pre_btn_state & 0x04)){
			input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
			input_sync(ts->input_dev);
		}
	}
	ts->pre_btn_state = ret & 0x07;
	input_sync(ts->input_dev);

	synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
}

static void synaptics_ts_work_func(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[8];
    	uint8_t status = 0;
	uint8_t inte = 0;
	if( ts->enable_remote) {
		TPD_DEBUG("%s:enable_remote",__func__);
		goto END;
	}
	memset(buf, 0, sizeof(buf));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00 ); 
	ret = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);
	if( ret < 0 ) {
		TPDTM_DMESG("Synaptic:ret = %d\n", ret);
		goto END;
	}
	status = ret & 0xff;
	inte = (ret & 0x7f00)>>8;
	TPD_DEBUG(" %s inte=%d  status=%d\n",__func__,inte,status);
	if(inte & 0x2) {
		TPDTM_DMESG("Synaptic:ret = %d, TP reset detected!\n", ret);
	}
	if(status) {
		synaptics_free_fingers(ts);
		int_state(ts);
		goto END;
	}
	if( inte & 0x04 ) {
		int_touch(ts);
	}	
	if( inte & 0x10 ) {
		int_key_report(ts);
	}
END:
	return;
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	mutex_lock(&ts->mutex);
	synaptics_ts_work_func(ts);
	mutex_unlock(&ts->mutex);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)dev_id;
	mutex_lock(&ts->mutex);
	synaptics_ts_work_func(ts);
	mutex_unlock(&ts->mutex);
	return IRQ_HANDLED;
}
#endif

static ssize_t limit_area_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	ssize_t ret =0;
	char page[PAGESIZE] = {0};

	TPD_DAILY("limit_area is: %d\n", limit_area);
	ret = sprintf(page, "limit_area = %d left_x1 = %d right_x1 = %d left_x2 = %d right_x2 = %d\n", limit_area, left_x1, right_x1, left_x2, right_x2);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_area_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp;

	if(copy_from_user(buf, buffer, count))
	{
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf, "%d", &temp);
	if(temp < 0 || temp >10)
		return count;
	
	limit_area = temp;
	left_x1 = (limit_area*1000)/100;
	right_x1 = LCD_WIDTH - left_x1;
	left_x2 = 2 * left_x1;
	right_x2 = LCD_WIDTH -2*left_x1;
	TPD_DAILY("limit_area = %d left_x1 = %d right_x1 = %d left_x2 =%d right_x2 = %d\n", limit_area, left_x1, right_x1, left_x2, right_x2);
	return count;
}

static ssize_t limit_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	ssize_t ret =0;
	char page[PAGESIZE] = {0};

	TPD_DEBUG("limit_enable is: %d\n", limit_enable);
	ret = sprintf(page, "%d\n", limit_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};

	if( count > 2 || ts_g == NULL ) 
		count = 2;

	if(copy_from_user(buf, buffer, count))
	{
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	if('0' == buf[0]){
		limit_enable = 0;
	}else if('1' == buf[0]){
		limit_enable = 1;
	}

	queue_work(synaptics_wq, &ts_g->close_edge_product);
	//TPD_ERR("temp = %d\n", temp);
	return count;
}

void close_edge_protect_function_s3320(struct work_struct *work)
{
	int ret = 0;

	if(ts_g == NULL)
	{
		TPD_ERR("ts_g is NULL!\n");
		return;
	}
	TPD_DAILY("%s limit_enable =%d !\n", __func__,limit_enable);	
	mutex_lock(&ts_g->mutex);
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4); 
	if(0 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x46);
	}
	else if(1 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x47);
	}
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0); 
	mutex_unlock(&ts_g->mutex);
}

static const struct file_operations proc_limit_area =
{
	.read = limit_area_read,
	.write = limit_area_write,
	.open = simple_open,
	.owner = THIS_MODULE,	
};

static const struct file_operations proc_limit_enable =
{
	.read = limit_enable_read,
	.write = limit_enable_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	char page[PAGESIZE] = {0};
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return baseline_ret;
	if(baseline_ret == 0){
		count = synaptics_rmi4_baseline_show(ts->dev,page,1);				
		baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));		
	}else{
		baseline_ret = 0;
	}
	return baseline_ret;
}
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

static ssize_t i2c_device_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] ={0};
	struct synaptics_ts_data *ts = ts_g;
	if(!ts_g)
		return ret;
	TPD_DEBUG("double tap enable is: %d\n", ts->double_enable);
	ret = sprintf(page, "%d\n", ts->i2c_device_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

#ifdef SUPPORT_GESTURE
static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("double tap enable is: %d\n", ts->double_enable);
	ret = sprintf(page, "%d\n", ts->double_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

static ssize_t tp_double_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int read_value;
	char buf[10] = {0};
	struct synaptics_ts_data *ts = ts_g;
	if( count > 2)
		return count;
	if( copy_from_user(buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf, "%d", &ret);
	if(!ts)
		return count;

	TPD_ERR("%s: ret=%d is_suspended=%d\n",__func__,ret,ts->is_suspended);
	mutex_lock(&ts->mutex);
	if ((ret == 0 ) || (ret == 1) || (ret == 2))
		ts->double_enable = ret;
	if (ts->is_suspended == 1) {
		switch(ret) {
			case 0:
			case 2:
				//TPDTM_DMESG("%s will be disable\n",__func__);
				ret = synaptics_enable_interrupt_for_gesture(ts, 0);
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts, 0);
				ret = synaptics_enable_interrupt(ts, 0);
				if(ret){
					TPD_DEBUG("%s: cannot disable interrupt\n", __func__);
					break;
				}
				ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01);
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					break;
				}
				break;
			case 1:
				//TPDTM_DMESG("%s will be enable\n",__func__);
				ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0); 
				if( ret < 0 ) {
					TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
					break;
				}
				read_value = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00);
				ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					break;
				}
				ret = synaptics_enable_interrupt_for_gesture(ts, 1); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts, 1); 
                			ret = synaptics_enable_interrupt(ts, 1);
				if(ret){
					TPD_DEBUG("%s: cannot enable interrupt\n", __func__);
					break;
				}
				break;
			default:
				TPDTM_DMESG("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	mutex_unlock(&ts->mutex);
	return count;
}

static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] ={0};
	TPD_DEBUG("%s:gesture_upload = %d \n",__func__,gesture_upload);
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;	
}


// chenggang.li@BSP.TP modified for oppo 2014-08-08 create node 
/******************************start****************************/
static const struct file_operations tp_double_proc_fops = {
	.write = tp_double_write_func,
	.read =  tp_double_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

//chenggang.li@bsp add 14045
static char buffer_data[128];
static void synaptics_select_regieter(uint16_t address, int count , char *buffer_data)
{
	int i,len,tot=0;
	uint8_t ret;
	struct synaptics_ts_data *ts = ts_g;
	if(count>8)
		count = 8;

	if(!ts)
		return;
		
	for(i=0;i<count;i++){
		ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,0xF9);
		ret = synaptics_rmi4_i2c_read_byte(ts->client,0xA3+i);
		len = sprintf(buffer_data+tot,"0x%02x  ",ret);
		tot+=len;
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,address>>8);
	ret = synaptics_rmi4_i2c_read_byte(ts->client,address&0xff);
	len = sprintf(buffer_data+tot,"the address is 0x%x = %02x\n",address,ret);
	tot+=len;

	ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,0);
	buffer_data[tot]='\0';
}

static ssize_t read_base_register_address(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	char page[PAGESIZE] = {0};
	ret = sprintf(page,"%s", buffer_data);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t write_base_register_address(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[24] = {0};
	int data,cnt;
	int off;

	strcpy (buf,buffer);
	cnt = sscanf(buf,"%x %d",&off, &data);
//	TPD_ERR("off is 0x%zx; data is %d\n",off,data);
	synaptics_select_regieter(off,data,buffer_data);
	return count;
}

#ifdef SUPPORT_GLOVES_MODE
static ssize_t tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	printk("glove mode enable is: %d\n", ts->glove_enable);
	ret = sprintf(page, "%d\n", ts->glove_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;
	char buf[10] = {0};

	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	if(!ts)
		return count;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		ts->glove_enable = ret;
		synaptics_glove_mode_enable(ts);
	}	
	switch(ret){	
		case 0:	
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:	
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;		
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
	return count;
}
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static ssize_t tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE] = {0};
	printk("sleep mode enable is: %d\n", sleep_enable);
	ret = sprintf(page, "%d\n", sleep_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	char buf[10] = {0};
    struct synaptics_ts_data *ts = ts_g;
    int ret = 0 ;
	if( count > 10 )
		return count;
	if(!ts)
		return count;
	if( copy_from_user( buf, buffer, count) ) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret);
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ) {
		sleep_enable = ret;
		synaptics_sleep_mode_enable(ts);		 
	}
	switch(ret) {
		case 0:
			TPDTM_DMESG("tp_sleep_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_sleep_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}
#endif

static ssize_t tp_show(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char * kernel_buf  ;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return 0;

	if(ts->is_suspended == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	if(!ts)
		return 0;

	kernel_buf = kmalloc(4096,GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error\n");
		return 0;
	}
	
	ret = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);
	if( ret < 0 )
		printk("tp_show read i2c err\n");	
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA01);
	if( ret < 0 )
		printk("tp_show read i2c err\n");
	
     sprintf(kernel_buf, "0x13=0x%x  %s \n", ret,"\r\n");
	 
     ret = simple_read_from_buffer(buf, count, ppos, kernel_buf, strlen(kernel_buf)); 
     kfree(kernel_buf);
     return ret;

}

static ssize_t store_tp(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int tmp = 0;
	if( 1 == sscanf(buf, "%d", &tmp) ){
		tp_debug = tmp;
	} else {
		TPDTM_DMESG("invalid content: '%s', length = %zd\n", buf, count);
	}	
	return count;
}

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1); 
	if( ret < 0 ) {
		TPDTM_DMESG("synaptics_rmi4_i2c_write_byte failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	return 0;
}

static void checkCMD(void)
{
	int ret;
	int flag_err = 0;
	struct synaptics_ts_data *ts = ts_g;
	do {
		msleep(30); //wait 30ms
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_COMMAND_BASE);
		ret = ret & 0x01;
		flag_err++;
    }while( (ret > 0x00) && (flag_err < 30) );
	if( ret > 0x00 )
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
}
#endif

static ssize_t tp_baseline_show(struct file *file, char __user *buf, size_t t, loff_t *ppos)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t tmp_old = 0;
	//uint16_t tmp_new = 0;
	uint16_t count = 0;
	int8_t TransCap[4] = {6, 6, 0 ,0};
	int8_t TransCap_tmp[4] = {0};
	char * kernel_buf;

	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;


	kernel_buf = kmalloc(4096,GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("%s kmalloc error\n",__func__);
		return 0;
	}
	
	memset(delta_baseline,0,sizeof(delta_baseline));
    /*disable irq when read data from IC*/
    disable_irq_nosync(ts->client->irq);	
	mutex_lock(&ts->mutex); 
	synaptics_read_register_map_page1(ts);

	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");

	/*step 1:check raw capacitance*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("Before write to the register, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
	ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap);    //F54_ANALOG_CTRL91 relate to the baseline result
	//ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+0x28);
	tmp_old = ret & 0xff;
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+0x28, (tmp_old & 0xdf));
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation

	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	//TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	//TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
	//printk("\n[%d]", x);
		num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%2d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h << 8)|tmp_l;
			//printk("%d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%4d ", delta_baseline[x][y]);
		}
	}
	TPDTM_DMESG("\nread all is oK\n");
	ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);    //After test TransCap_data should return
	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("After return, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
	msleep(60);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);

    synaptics_enable_interrupt(ts,1);
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);

	num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s ","\r\n");
	ret=simple_read_from_buffer(buf,  t, ppos,kernel_buf,strlen(kernel_buf));

	kfree(kernel_buf);
	
	return ret;
}

static ssize_t tp_self_rawdata_show(struct file *file, char __user *buf, size_t t, loff_t *ppos)
{
    int ret = 0, x = 0, y = 0;
    ssize_t num_read_chars = 0;
    uint8_t tmp_l = 0, tmp_h = 0;
    int16_t temp_delta = 0;
    char * kernel_buf;
    struct synaptics_ts_data *ts = ts_g;

    if(!ts)
        return 0;

    kernel_buf = kmalloc(4096,GFP_KERNEL);
    if(kernel_buf == NULL)
    {
        TPD_ERR("%s kmalloc error\n",__func__);
        return 0;
    }

    //prepare RT58 Test
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);     //select page
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_DATA_BASE, 0x42);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_DATA_BASE, 0xE1);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x80);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0x00, 0x4F);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0x00, 0x59);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);

    /*disable irq when read data from IC*/
    disable_irq_nosync(ts->client->irq);
    mutex_lock(&ts->mutex);
    synaptics_read_register_map_page1(ts);

    ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_DATA_BASE, 0x3A);//select report type 0x02
    ret = synaptics_rmi4_i2c_write_word(ts->client, F54_ANALOG_DATA_BASE + 1, 0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
    msleep(10);
    checkCMD();

    for (x = 0; x < 1; x++) {
        num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%2d]", x);
        for( y = 0; y < 53; y++ ){
            ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
            tmp_l = ret&0xff;
            ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
            tmp_h = ret&0xff;
            temp_delta = (tmp_h << 8) | tmp_l;
            num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%4d ", temp_delta);
        }
    }
    msleep(60);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01);
    if (ret < 0) {
         TPD_ERR("%s: failed for page select\n", __func__);
    }

    mutex_unlock(&ts->mutex);
    enable_irq(ts->client->irq);
    num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s ","\r\n");
    ret=simple_read_from_buffer(buf,  t, ppos,kernel_buf,strlen(kernel_buf));
    kfree(kernel_buf);
    return ret;
}


static ssize_t tp_rawdata_show(struct file *file, char __user *buf, size_t t, loff_t *ppos)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0;
	uint16_t count = 0;
	char * kernel_buf;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return 0;

	kernel_buf = kmalloc(4096,GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("%s kmalloc error\n",__func__);
		return 0;
	}

	
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
    disable_irq_nosync(ts->client->irq);	
	mutex_lock(&ts->mutex);	
	synaptics_read_register_map_page1(ts);
	
	//TPD_DEBUG("\nstep 2:report type2 delta image\n");	
	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
	ret = synaptics_rmi4_i2c_write_word(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%2d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;       		
			//printk("%3d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%4d ", delta_baseline[x][y]);
		}	 
	}
	//ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
     msleep(60);
   // ret = synaptics_soft_reset(ts);
   //delay_qt_ms(60);

    synaptics_enable_interrupt(ts, 1);
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);

	num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s ","\r\n");
	ret=simple_read_from_buffer(buf,  t, ppos,kernel_buf,strlen(kernel_buf));
	kfree(kernel_buf);
	return ret;
}

static ssize_t tp_delta_store(struct file *file, const char __user *page, size_t count, loff_t *lo)
{
	  TPDTM_DMESG("tp_test_store is not support\n");
	  return count;
}


//guomingqiang@phone.bsp, 2016-06-27, add for tp test step2
int  pow1_s3320(uint8_t  n)
{	
	return 1<<n;
}

static ssize_t synaptics_rmi4_baseline_show_s3508(struct device *dev, char *buf, bool savefile)	
{
	ssize_t num_read_chars = 0;
#if TP_TEST_ENABLE
    int ret = 0;
	uint8_t x,y;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t count = 0;
	int error_count = 0;
	uint8_t buffer[9];
	int16_t *baseline_data_test;
	int enable_cbc = 0;

//guomingqiang@phone.bsp 2016-6-27 add setp 0 for TP EINT direct short to Low
	int eint_status, eint_count = 0,read_gpio_num;
	char eint_buf[64];
//end

//guomingqiang@phone.bsp, 2016-06-27, add for tp test step2
	uint8_t i,j;
	uint8_t buffer_rx[RX_NUM];
	uint8_t buffer_tx[3] = {32, 34, 36};    //for key only need test tx 32, 34, 36
	int sum_line_error = 0;
	unsigned int pow_num=0;
//end

	int fd = -1;
	int8_t TransCap[4] = {6, 6, 0, 0};
	int8_t TransCap_tmp[4] = {0};
	struct timespec   now_time;
	struct rtc_time   rtc_now_time;
	uint8_t  data_buf[64];
	mm_segment_t old_fs;

	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
#ifdef TP_TEST_STATION
	uint8_t CBC_TransCap[28],temp_CBC[28];
	uint8_t val[28] = {0};
	int err_line_num = -1;
	static int F54_ANALOG_CTRL96;
#endif

	ret = request_firmware(&fw, ts->test_limit_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts->test_limit_name, ret);
		error_count++;
		num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
		num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
		return num_read_chars;
	}

	ph = (struct test_header *)(fw->data);

	mutex_lock(&ts->mutex);
	disable_irq_nosync(ts->client->irq);

	memset(Rxdata, 0, sizeof(Rxdata));
	synaptics_read_register_map_page1(ts);
	//TPDTM_DMESG("step 1:select report type 0x03\n");

	if(savefile) {
        getnstimeofday(&now_time);
        rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
        sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv", 
                (rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
                rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0) {
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
            set_fs(old_fs);
        }
    }

//guomingqiang@phone.bsp 2016-6-27 add setp 0 for TP EINT direct short to Low
	ret = synaptics_enable_interrupt(ts, 0);
	TPD_ERR("synaptics_rmi4_baseline_show_s3508  step 0:check TP EINT direct short\n");
       	eint_count = 0;
	read_gpio_num = 10;
	while(read_gpio_num--)
	{
		msleep(5);
		eint_status = gpio_get_value(ts_g->irq_gpio);
		if(eint_status == 1)
			eint_count--;
		else
			eint_count++;
	}
	
	TPD_ERR("eint_count=%d\n",eint_count);
    
	if(eint_count == 10)
	{
		TPD_ERR("error :  TP EINT PIN direct short!\n");
		error_count++;
		num_read_chars += sprintf(&(buf[num_read_chars]), "eint_status is low, TP EINT direct stort,\n");
		sprintf(&eint_buf[0], "eint_status is low, TP EINT direct stort,\n");
              	sys_write(fd, eint_buf, strlen(eint_buf));
		eint_count = 0;
		goto END;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("Before send TransCap data, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
	ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap);    //F54_ANALOG_CTRL91 relate to the baseline result

//step 1:check raw capacitance.
TEST_WITH_CBC_s3508:
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}
	//ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28);
	tmp_old = ret&0xff;
    
	if(enable_cbc){
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28,(tmp_old | 0x20));
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28);
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		TPD_DEBUG("Test open cbc\n");
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limitcbc_offset);
	}else{
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xdf));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28,(tmp_old & 0xdf));
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28);
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limit_offset);
	}
	/******write No Relax to 1******/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	TPD_DEBUG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();
			
	count = 0;	
	for( x = 0; x < TX_NUM; x++ ){
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			if (fd >= 0){
                sprintf(data_buf, "%d,", baseline_data);
                sys_write(fd, data_buf, strlen(data_buf));
            }
			TPD_DEBUG("baseline_data is %d x:%d y:%d  \n", baseline_data, x, y);
			if( (y < (RX_NUM - 2) ) && (x < (TX_NUM - 1) ) ){ 
				TPD_DEBUG("RX_NUM is %d,TX_NUM is %d baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n", RX_NUM, TX_NUM, baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
				//printk("%d,",baseline_data);
				if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
						TPD_ERR("Synaptic:touchpanel failed---------------;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
						num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
						error_count++;
						goto END;
					} 
			}	
			
				//test virtual key  ----- the last two data.
			if( (y > RX_NUM-3) && (x == TX_NUM - 1) ){
					TPD_DEBUG("synaptics:test virtual key,y= %d ,x = %d\n",y,x);
					TPD_DEBUG("Synaptic:test virtual key;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
					if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
						TPD_ERR("Synaptic:test virtual key failed------------;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
						num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
						error_count++;
						goto END;
					} 
			}
	
			count++;
		}
		if (fd >= 0){
            sys_write(fd, "\n", 1);
        }
		printk("\n");
   	}
	
	if(!enable_cbc){
		enable_cbc = 1;
		TPD_ERR("test cbc baseline again\n");
			goto TEST_WITH_CBC_s3508;
	}
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	checkCMD();

//step 2 :check tx-to-tx and tx-to-vdd
	TPD_ERR("step 2:check TRx-TRx & TRx-Vdd short\n" );
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x1);
	//ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x1A);//select report type 26
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	//msleep(100);
	checkCMD();
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_DATA_BASE+3, 5, buffer);

	//guomingqiang@phone.bsp, 2016-06-27, add for tp test step2
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x3);
	i2c_smbus_read_i2c_block_data(ts->client, 0x01, RX_NUM, buffer_rx); 		//save the used rx

	TPD_ERR("RX_NUM : ");
	for(i = 0;i < RX_NUM;i++)
		printk("%d,",buffer_rx[i]);
	printk("\n");

	TPD_ERR("TX_NUM : ");
	for(i = 0;i < 3;i++)
		printk("%d,",buffer_tx[i]);
	printk("\n");

	for(x = 0;x < 5; x++)
	{
		if(buffer[x]!=0)
		{
			for(i=0;i<8;i++)
			{
				pow_num=pow1_s3320(i);
				if((pow_num & buffer[x])!=0)
				{
						sum_line_error = 8*x+i;
						for(j = 0;j < RX_NUM;j++)
						{
								if(sum_line_error == buffer_rx[j])
								{
										error_count++;
										TPD_ERR(" step 2 :check tx-to-tx and tx-to-vdd error ,  error_line is rx =%d\n",buffer_rx[j]);
										num_read_chars += sprintf(&(buf[num_read_chars]), " step 2 :check tx-to-tx and tx-to-vdd error ,  error_line is rx =%d\n",buffer_rx[j]);
										goto END;
								}
						}
						for(j = 0;j < 3;j++)
						{
								if(sum_line_error == buffer_tx[j])
								{
										error_count++;
										TPD_ERR(" step 2 :check tx-to-tx and tx-to-vdd error , error_line is tx =%d\n",buffer_tx[j]);
										num_read_chars += sprintf(&(buf[num_read_chars]), " step 2 :check tx-to-tx and tx-to-vdd error , error_line is rx =%d\n",buffer_rx[j]);
										goto END;
								}
						}
				}
			}
		}
	}
	num_read_chars += sprintf(buf, "1");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	checkCMD();

	ret = synaptics_soft_reset(ts);	//software reset TP
	msleep(50);

	//Step 3 : Check the broken line
	TPD_ERR("Step 3 : Check the broken line\n" );
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x1);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x82);//select report type 0x82
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set LSB 
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE+2, 0x00);//set MSB
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();

	for( y = 0; y < RX_NUM; y++ ) {
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
		tmp_l = ret&0xff;
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
		tmp_h = ret&0xff;
		baseline_data = (tmp_h<<8)|tmp_l;
		TPD_ERR("Step 3 data[%d] is %d\n", y, baseline_data);
		if(baseline_data > 100)
		{
				error_count++;
				TPD_ERR(" step 3 :check the broken line, error_line is y =%d\n",y);
				num_read_chars += sprintf(&(buf[num_read_chars]), " step 3 :check the broken line, error_line is y =%d\n",y);
				goto END;
		}
	}

	msleep(20);     //bit 2 of F54_ANALOG_COMMAND_BASE will return 0 in 20ms
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_COMMAND_BASE);
	TPD_ERR("Step 3 after read ret is %d\n", ret);

#ifdef TP_TEST_STATION
//Step 3 : Check the broken line
	TPD_ERR("Step 3 : Check the broken line\n" );
	ret = synaptics_soft_reset(ts);	//software reset TP
    msleep(150);
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x1);
	F54_ANALOG_CTRL96= F54_ANALOG_CONTROL_BASE + 0x21;
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE, 0x22);		//No Scan
	ret = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_CTRL96, 28, &(CBC_TransCap[0]));		//save CBC setting
	for(x = 0 ; x < RX_NUM; x++)
	{
		val[x] = 0;
	}
	ret = i2c_smbus_write_i2c_block_data( ts_g->client, F54_ANALOG_CTRL96, 28, &(val[0]) );	//val is zero
	checkCMD();
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	ret = i2c_smbus_read_i2c_block_data( ts->client, F54_ANALOG_CTRL96, 28, &(temp_CBC[0]) );	//val is zero
	for(x = 0 ; x < 28; x++)
		TPD_DEBUG("temp_CBC [%d] = %d\n", x, *(temp_CBC + x));

	TPD_ERR("TX_NUM =%d  RX_NUM = %d\n ",TX_NUM,RX_NUM );
	//read baseline_data
	for(x = 0 ; x < RX_NUM; x++)
	{
		val[x] = 0x0f;
	//	for(int i = 0; i< RX_NUM; i++)
	//		TPD_DEBUG("val[%d] = %d\n", i, val[i]);
		ret = i2c_smbus_write_i2c_block_data( ts->client, F54_ANALOG_CTRL96, RX_NUM, &(val[0]) );
		checkCMD();
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
		checkCMD();
		ret = i2c_smbus_read_i2c_block_data( ts->client, F54_ANALOG_CTRL96, RX_NUM, &(temp_CBC[0]) );
	//	for(int i = 0 ; i < RX_NUM; i++)
	//		TPD_DEBUG("temp_CBC [%d] = %d\n", i, *(temp_CBC + i));

		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23);
		tmp_old = ret&0xff;

		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old | 0x10));
		ret = i2c_smbus_write_word_data(ts->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret | 0x20;
		i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		ret = i2c_smbus_write_word_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
		checkCMD();
		TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
		checkCMD();
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
		ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
		checkCMD();
		
		for(y = 0; y < RX_NUM; y++) 
		{
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			if(x == y)		//self, should be in [1500, 1650]
			{
				TPD_DEBUG("x = %d , y = %d, baseline_data = %d\n", x, y, baseline_data);
				if(baseline_data <1500 || baseline_data > 1650)
				{
					TPD_ERR("Broken test fail! x = %d y = %d baseline_data = %d\n", x , y, baseline_data);
					sprintf(data_buf, "%d,%d,%d\n", x, y, baseline_data);
                	sys_write(fd, data_buf, strlen(data_buf));
					error_count++;
					num_read_chars += sprintf(&(buf[num_read_chars]), "TPD error RX_x = %d RX_y = %d baseline_data=%d \n", x, y,baseline_data);
				}
			}
			else		//others, should be less than 50
			{
				if(abs(baseline_data) > 50)
				{
					TPD_ERR("Broken test fail! x = %d y = %d baseline_data = %d\n", x , y, baseline_data);
					sprintf(data_buf, "%d,%d,%d\n", x, y, baseline_data);
                				sys_write(fd, data_buf, strlen(data_buf));
					error_count++;
					num_read_chars += sprintf(&(buf[num_read_chars]), "TPD error RX_x = %d RX_y = %d baseline_data=%d \n", x, y,baseline_data);
				}
			}
		}

		if (fd >= 0 && y == RX_NUM - 1)
		{
			sys_write(fd, "\n", 1);
		}
		err_line_num = x;
		val[x] = 0;
	}

	ret = i2c_smbus_read_i2c_block_data( ts->client, F54_ANALOG_CTRL96, 32, &(CBC_TransCap[0]) );
	checkCMD();
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE, 0x00);		//Cancel No Scan
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
#endif  /*TP_TEST_STATION*/	
 	
END:
	if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }
	release_firmware(fw);
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n", TP_FW, TP_FW);
	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	TPD_ERR(" TP auto test %d error(s). %s\n", error_count, error_count?"":"All test passed.");
		
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	msleep(60);
	ret = synaptics_soft_reset(ts);	//software reset TP
	msleep(150);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	//ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);    //After test these data should return
	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("After soft reset, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
		
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif	
    synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts, 1);
	enable_irq(ts->client->irq);
	printk("\n\nstep5 reset and open irq complete\n");
    mutex_unlock(&ts->mutex);
#endif
	return num_read_chars;
}


static ssize_t tp_baseline_show_with_cbc(struct file *file, char __user *buf, size_t t, loff_t *ppos)
{
	int ret;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint8_t tmp_old;
	uint16_t count = 0;
	int8_t TransCap[4] = {6, 6, 0, 0};
	int8_t TransCap_tmp[4] = {0};
	struct synaptics_ts_data *ts = ts_g;
	char * kernel_buf  ;
	if(ts->is_suspended == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	if(!ts)
		return 0;

	kernel_buf = kmalloc(4096,GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error\n");
		return 0;
	}
	
	/*disable irq when read data from IC*/	
    disable_irq_nosync(ts->client->irq);	
	mutex_lock(&ts->mutex);	
	synaptics_read_register_map_page1(ts);
	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	//step 1:check raw capacitance.

	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("Before write register, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
	ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap);    //F54_ANALOG_CTRL91 relate to the baseline result
	//ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28);
	tmp_old = ret&0xff;
	TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
	ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+0x28,(tmp_old | 0x20));
	ret = i2c_smbus_write_word_data(ts->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	//TPD_DEBUG("open CBC oK\n");
	//ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
	//tmp_new = ret | 0x20;
	//i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
	//ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);

	
	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	checkCMD();
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	
	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
		
	ret = synaptics_rmi4_i2c_write_word(ts->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {   	
		TPD_DEBUG("\n[%d]",x);
		num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%2d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			TPD_DEBUG("%d,",delta_baseline[x][y]);	
			num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%4d ",delta_baseline[x][y]);
		}
   	}
	
	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
	ret = synaptics_rmi4_i2c_write_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);    //After test TransCap_data should return
	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_CONTROL_BASE+0x2A, 4, TransCap_tmp);
	TPD_ERR("After return, TransCap is [%d %d %d %d]\n", TransCap_tmp[0], TransCap_tmp[1], TransCap_tmp[2], TransCap_tmp[3]);
    msleep(60);
    synaptics_enable_interrupt(ts,1);
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);

     TPD_ERR("num_read_chars = %zd , t = %zd\n",num_read_chars,t);
     num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%s ","\r\n");
     ret = simple_read_from_buffer(buf, t, ppos, kernel_buf, strlen(kernel_buf)); 
     kfree(kernel_buf);
     return ret;
	  
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,	char *buf, bool savefile)	
{
	return synaptics_rmi4_baseline_show_s3508(dev,buf,savefile);	
}

static ssize_t tp_test_store(struct file *file, const char __user *page, size_t count, loff_t *lo)
{
   TPDTM_DMESG("tp_test_store is not support\n");
   return count;
}

static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,struct device_attribute *attr, char *buf)
{	
	if( (tp_dev == TP_G2Y) || (tp_dev == TP_TPK) )
		return sprintf(buf, "%d\n", TP_TPK); 
	if(tp_dev == TP_TRULY)
		return sprintf(buf, "%d\n", TP_TRULY); 
    if(tp_dev == TP_OFILM)
		return sprintf(buf, "%d\n", TP_OFILM);	
	return sprintf(buf, "%d\n", tp_dev); 
}


static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int attr_count = 0;
	int ret = 0;
	
	TPD_DEBUG("%s is called\n",__func__);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return -1;
	}
	ts->input_dev->name = TPD_DEVICE;;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
	set_bit(KEY_MENU , ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE , ts->input_dev->keybit);
	set_bit(KEY_BACK , ts->input_dev->keybit);
#endif
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif				
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE , ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);
	
	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);			
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);	
		return -1;
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++) {
		ret = sysfs_create_file(&ts->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
		if (ret < 0) {
			dev_err(&ts->client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			for (attr_count--; attr_count >= 0; attr_count--) {
				sysfs_remove_file(&ts->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
			}
			return -1;
		}
	}		
	return 0;
}

static int synaptics_free_fingers(struct synaptics_ts_data *ts)
{
	int i = 0;

#ifdef TYPE_B_PROTOCOL
	for (i = 0; i < ts->max_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);
#else 
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
#endif

	is_touch = 0; //guomingqiang@phone.bsp 2016/07/12 , clear touch flag  
	
	return 0;
}
/*********************FW Update Func******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
    int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0 ;
	
	TPD_DAILY("synatpitcs_fw_update: fw_name = %s\n", ts->fw_name);
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");	
		return 0;
	}

	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
						ts->fw_name, ret);
		return ret;
	}
	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force); 
	if(ret < 0){
		TPD_ERR("FW update not success try again\n");			
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);	
	
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);	
	ret = synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
	#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
	#endif	
    synaptics_free_fingers(ts);
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts,1);	
	return 0;
}

static ssize_t synaptics_update_fw_write(struct file *file, const char __user *page, size_t size, loff_t *lo)				
{
	struct synaptics_ts_data *ts = ts_g;
	int val = 0;

	if (size> 2)
		return -EINVAL;

	sscanf(page, "%d", &val);
	if(!val)
		val = force_update;

	disable_irq_nosync(ts->client->irq);
	mutex_lock(&ts->mutex);
	ts->loading_fw = true;
	synatpitcs_fw_update(ts->dev, val);
	ts->loading_fw = false;
	mutex_unlock(&ts->mutex);
	enable_irq(ts->client->irq);
	force_update = 0;
	return size;
}
/*********************FW Update Func End*************************************/

static ssize_t synaptics_test_limit_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts = ts_g;
	int ret = 0;
	uint16_t *prow = NULL; 
	uint16_t *prowcbc = NULL; 
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	int i = 0;
	int temp = 0;
	static int cat_cbc_change = 0;

	char * kernel_buf = kmalloc(4096,GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("%s kmalloc error\n",__func__);
		return 0;
	}
	
	ret = request_firmware(&fw, ts->test_limit_name, ts_g->dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
						ts->test_limit_name, ret);
		temp = temp + sprintf(&kernel_buf[temp],"Request failed,Check the path %d",temp);
		goto GO_RETRUN;
	}
		
	ph = (struct test_header *)(fw->data);
	prow = (uint16_t *)(fw->data + ph->array_limit_offset);

	prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);

	TPD_DEBUG("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x\n",
		ph->array_limit_offset,ph->array_limitcbc_offset);
	
	TPD_DEBUG("test begin:\n");
	if(cat_cbc_change == 0 || ph->withCBC == 0) {
		temp += sprintf(kernel_buf, "Without cbc:");
		for(i = 0 ;i < (ph->array_limit_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
			   temp += sprintf(&(kernel_buf[temp]), "\n[%2d] ",(i/RX_NUM)/2);
			temp += sprintf(&kernel_buf[temp],"%4d,",prow[i]);
			printk("%d,",prow[i]);						   
		}
		cat_cbc_change = 1;
	}else{
		temp += sprintf(kernel_buf, "With cbc:");
		cat_cbc_change = 0;
		if( ph->withCBC == 0){
		  goto GO_RETRUN;
		}
		for(i = 0 ;i < (ph->array_limitcbc_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
			   temp += sprintf(&(kernel_buf[temp]), "\n[%2d] ",(i/RX_NUM)/2);
			temp += sprintf(&kernel_buf[temp],"%4d,",prowcbc[i]);
			printk("%d,",prowcbc[i]);
		}
	}
	release_firmware(fw);
GO_RETRUN:
	temp += sprintf(&(kernel_buf[temp]), "%s ","\r\n");
	ret=simple_read_from_buffer(buf,  count, ppos,kernel_buf,strlen(kernel_buf));

	kfree(kernel_buf);
	return ret;	
}

static ssize_t synaptics_test_limit_store(struct file *file, const char __user *page, size_t count, loff_t *lo)
{
	return count;
}

static ssize_t tp_write_func (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	return count;
}

#ifdef FINGER_PROTECT
static ssize_t read_finger_protect_result(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[16] = {0};
    struct synaptics_ts_data *ts = ts_g;
    if(!ts)
        return 0;

    TPD_DAILY("%s report_finger_protect = %d\n", __func__, report_finger_protect);
    ret = sprintf(page, "%d\n", report_finger_protect);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
    return ret;
}

static int synaptics_finger_entrance(void);
static ssize_t trigger_finger_protect(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int op = 0;

    if( 1 == sscanf(buffer, "%d", &op) ) {
        if (op == 1) {
             synaptics_finger_entrance();
        }
    } else {
        printk("invalid content: '%s', length = %zd\n", buffer, count);
        return -EINVAL;
    }

    return count;
}

#endif

static const struct file_operations proc_test_limit =
{
	.write = synaptics_test_limit_store,
	.read = synaptics_test_limit_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations proc_oppo_tp_baseline_image =
{
	.write = tp_delta_store,
	.read= tp_baseline_show,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations proc_oppo_tp_baseline_image_with_cbc =
{
	.write = tp_test_store,
	.read = tp_baseline_show_with_cbc,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations proc_oppo_tp_delta_image =
{
	.read=tp_rawdata_show,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations proc_oppo_tp_selfdelta_image =
{
    .read=tp_self_rawdata_show,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations proc_oppo_tp_debug =
{
	.write = store_tp,
	.read=tp_show,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations proc_firmware_update =
{
	.write = synaptics_update_fw_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};


//chenggang.li@bsp add for 14045
static const struct file_operations base_register_address= {
	.write = write_base_register_address,
	.read =  read_base_register_address,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#ifdef FINGER_PROTECT
//wanghao@bsp add for finger protect
static const struct file_operations finger_protect_result= {
        .read =  read_finger_protect_result,
        .open = simple_open,
        .owner = THIS_MODULE,
};

static const struct file_operations finger_protect_trigger= {
        .write =  trigger_finger_protect,
        .open = simple_open,
        .owner = THIS_MODULE,
};

#endif

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations i2c_device_test_fops = {	
	.read =  i2c_device_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations tp_baseline_test_proc_fops = {	
	.read =  tp_baseline_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

#ifdef SUPPORT_GLOVES_MODE
static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static const struct file_operations sleep_mode_enable_proc_fops = {
	.write = tp_sleep_write_func,
	.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations tp_reset_proc_fops = {
	.write = tp_write_func,
	//.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_synaptics_proc(struct synaptics_ts_data *ts)
{
	int ret = 0;	
	struct proc_dir_entry *prEntry_tmp  = NULL; 
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create TP proc entry\n");
	}

	prEntry_tmp = proc_create("test_limit", 0644, prEntry_tp,&proc_test_limit);
	prEntry_tmp = proc_create("oppo_tp_baseline_image", 0644, prEntry_tp,&proc_oppo_tp_baseline_image);
	prEntry_tmp = proc_create("oppo_tp_baseline_image_with_cbc", 0644, prEntry_tp,&proc_oppo_tp_baseline_image_with_cbc);
	prEntry_tmp = proc_create("oppo_tp_delta_image", 0644, prEntry_tp,&proc_oppo_tp_delta_image);
	prEntry_tmp = proc_create("oppo_tp_debug", 0644, prEntry_tp,&proc_oppo_tp_debug);
	prEntry_tmp = proc_create("tp_fw_update", 0644, prEntry_tp,&proc_firmware_update);
	prEntry_tmp = proc_create("oppo_tp_limit_area", 0664, prEntry_tp, &proc_limit_area);
	prEntry_tmp = proc_create("oppo_tp_limit_enable", 0664, prEntry_tp, &proc_limit_enable);
	prEntry_tmp = proc_create("oppo_tp_selfdelta_image", 0664, prEntry_tp, &proc_oppo_tp_selfdelta_image);
    	
#ifdef SUPPORT_GESTURE
	if(ts->black_gesture_support) {
		prEntry_tmp = proc_create( "double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
		if(prEntry_tmp == NULL){
			ret = -ENOMEM;
			printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		}	

		prEntry_tmp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
		if(prEntry_tmp == NULL){	   
			ret = -ENOMEM;	   
			printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		}
	}
#endif	 

#ifdef SUPPORT_GLOVES_MODE
	if(ts->glove_mode_enabled) {
		prEntry_tmp = proc_create( "glove_mode_enable", 0666, prEntry_tp,&glove_mode_enable_proc_fops);
		if(prEntry_tmp == NULL) {
			ret = -ENOMEM;
			printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		}
	}
#endif	
	
#ifdef SUPPORT_TP_SLEEP_MODE
	prEntry_tmp = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if( prEntry_tmp == NULL ){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif	
		
#ifdef RESET_ONESECOND
	prEntry_tmp = proc_create( "tp_reset", 0666, prEntry_tp, &tp_reset_proc_fops);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create tp reset proc entry\n");
	}
#endif
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
	prEntry_tmp = proc_create( "baseline_test", 0666, prEntry_tp, &tp_baseline_test_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\i2c_device_test"  begin
	prEntry_tmp = proc_create( "i2c_device_test", 0666, prEntry_tp, &i2c_device_test_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}	
	
	prEntry_tmp = proc_create( "synaptics_register_address", 0777, prEntry_tp, &base_register_address);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}	
    #ifdef FINGER_PROTECT
    prEntry_tmp = proc_create( "finger_protect_result", 0777, prEntry_tp, &finger_protect_result);
    if(prEntry_tmp == NULL){
        ret = -ENOMEM;
        printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
    }
    prEntry_tmp = proc_create("finger_protect_trigger", 0777, prEntry_tp, &finger_protect_trigger);
    if(prEntry_tmp == NULL) {
        ret = -ENOMEM;
        printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
    }
    #endif
    
	return ret;
}
/******************************end****************************/

/****************************S3203*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT(struct i2c_client *client)
{	  
    uint8_t buf[8];
    i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
    SynaF34DataBase = buf[3];
    SynaF34QueryBase = buf[0];
    i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
    SynaF01DataBase = buf[3];
    SynaF01CommandBase = buf[1];
    i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);
	
    SynaF34Reflash_BlockNum = SynaF34DataBase;
    SynaF34Reflash_BlockData = SynaF34DataBase + 1;
    SynaF34ReflashQuery_BootID = SynaF34QueryBase;
    SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
    SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
    SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
    SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
    SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
    i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
    SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
    TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
    SynaF34_FlashControl = SynaF34DataBase + 2;
}
struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

static int checkFlashState(struct i2c_client *client)
{
	int ret ;
	int count = 0;
	ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
		count++;
    } 
	if(count == 8)
		return 1;
	else
		return 0;
}

static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;
	int max_x_ic = 0;
	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);	
		return -1;
	}
	
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
   	/*read max_x ,max_y*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
			return -1;
		}
	}
	
	i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL08, 4, buf);
	printk("buf[0] = 0x%x, buf[1] = 0x%x\n", buf[0], buf[1]);
	max_x_ic = ( (buf[1]<<8)&0xffff ) | (buf[0]&0xffff);       
	max_y_ic = ( (buf[3]<<8)&0xffff ) | (buf[2]&0xffff); 		
	
	TPD_DAILY("max_x = %d,max_y = %d\n max_x_ic = %d,max_y_ic = %d\n",ts->max_x,ts->max_y,max_x_ic,max_y_ic);
	if((ts->max_x == 0) ||(ts->max_y ==0 )) {
		ts->max_x = max_x_ic;
		ts->max_y = max_y_ic;
	}
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 	
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);	
	
 	if((max_x_ic == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");		
		return -1;
	}
	return 0;
}

static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	int ret,j;
	uint8_t buf[8];
	uint8_t bootloder_id[10];
	uint16_t block,firmware,configuration;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	const uint8_t *Config_Data = NULL;
	const uint8_t *Firmware_Data = NULL;
	struct image_header_data header;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);	
	
	TPD_DEBUG("%s is called\n",__func__);
	if(!client)
		return -1;
		
	parse_header(&header,data);	
	if((header.firmware_size + header.config_size + 0x100) > data_len) {
		TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
		return -1;
	}

	Firmware_Data = data + 0x100;
	Config_Data = Firmware_Data + header.firmware_size;
	ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);	

	ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
	TPD_DAILY("CURRENT_FIRMWARE_ID is %x-----------, FIRMWARE_ID is %x-----------\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);
	//TPD_ERR("synaptics force is %d\n", force);	
	if(!force) {
		if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
			return 0;
		}	
	}	
	re_scan_PDT(client);	
	block = 16;
	TPD_DEBUG("block is %d \n",block);
	firmware = (header.firmware_size)/16;
    TPD_DEBUG("firmware is %d \n",firmware);
    configuration = (header.config_size)/16;
    TPD_DEBUG("configuration is %d \n",configuration);


	ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));  	//00e9->0026
	TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);		
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));	//00ec->0000
	TPDTM_DMESG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);		//00ee->0034
	
	synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x0F);	//00ee->0034
	msleep(10);	
	TPD_DEBUG("attn step 4\n");
	ret=checkFlashState(client);
	if(ret > 0) {
		TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
			return -1;
	}
	ret = i2c_smbus_read_byte_data(client,0x04);
	TPD_DEBUG("The status(device state) is %x\n",ret);
	ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
	TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
	ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);		//no sleep
	/********************get into prog end************/
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));		//00ec
	TPD_DEBUG("ret is %d\n",ret);	 
	re_scan_PDT(client);
	i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);	//00e9
	i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
	msleep(2000);
	ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
	TPDTM_DMESG("going to flash firmware area synaF34_FlashControl %d\n",ret);	

	TPD_DEBUG("update-----------------firmware ------------------update!\n");
	TPD_DEBUG("cnt %d\n",firmware);
	for(j=0; j<firmware; j++) {
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;		
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]); 
		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x02);
		ret=checkFlashState(client);		
		if(ret > 0) {
			TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}		
	}
	//step 7 configure data
	//TPD_ERR("going to flash configuration area\n");
	//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
	//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
	TPD_DEBUG("update-----------------configuration ------------------update!\n");
	for(j=0;j<configuration;j++) {
		//a)write SynaF34Reflash_BlockNum to access
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
		//b) write data
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);
		//c) issue write
		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x06);
		//d) wait attn		
		ret = checkFlashState(client);	
		if(ret > 0) {
			TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	
	//step 1 issue reset
	synaptics_rmi4_i2c_write_byte(client,SynaF01CommandBase,0x01);
	//step2 wait ATTN
	//delay_qt_ms(1000);
	mdelay(1500);
    synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}	
	TPD_DEBUG("Firmware self check Ok\n");
	return 0;
}
/****************************S3203*****end!!!!!**********************************/


static int charge_plug_status = 0 ;//0: no   1: slow  3:  quickly charge
static atomic_t charge_plug_in_flag;
extern void (*synaptics_chg_mode_enable)(int , int);
static void synaptics_charge_mode_enable(int enable, int is_fast_charge)
{
	int ret;

    	charge_plug_status = enable ;
	if(ts_g == NULL){
		TPD_ERR("tp do not register\n");
		return;
	}
    	if(ts_g->loading_fw)
	{
		TPD_ERR("  updtae tp firmware now,can not write tp reg \n");
		return;
	}

       mutex_lock(&ts_g->mutex);


	atomic_set(&charge_plug_in_flag,enable);
	TPD_DAILY("%s, page 4  F51_CUSTOM_CTRL31=0x%x write charge_plug_status=%d is_fast_charge=%d\n",__func__,F51_CUSTOM_CTRL31,charge_plug_status, is_fast_charge);
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 4);
	if( ret < 0 ){
		TPD_ERR("%s i2c error\n", __func__);
		goto END ;
	}
	ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL31&0x00ff, charge_plug_status);
	if( ret < 0 ){
		TPD_ERR("%s i2c error\n", __func__);
		goto END ;
	}
       ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0);
	if( ret < 0 ){
		TPDTM_DMESG("%s i2c error\n", __func__);
              goto END ;
	}

END:
       mutex_unlock(&ts_g->mutex);
	return ;
}

static  void get_tp_id(int TP_ID1,int TP_ID2,int TP_ID3, struct synaptics_ts_data *ts)
{
	int ret,id1 = -1,id2 = -1,id3 = -1;
	if(TP_ID1  >= 0) {
		ret = gpio_request(TP_ID1,"TP_ID1");
	    id1=gpio_get_value(TP_ID1);	
	}
	if(TP_ID2  >=  0) {
		ret = gpio_request(TP_ID2,"TP_ID2");
		id2=gpio_get_value(TP_ID2);	
	}	
	if(TP_ID3  >= 0) {
		ret = gpio_request(TP_ID3,"TP_ID3");
		id3=gpio_get_value(TP_ID3);
	}
	
	if((id1 == 1)&&(id2 == 1)&&(id3 == 0)){
		TPD_DEBUG("%s::OFILM\n",__func__);			
		tp_dev=TP_OFILM;
	}else if((id1 == 0)&&(id2 == 0)&&(id3 == 0)){
		TPD_DEBUG("%s::TP_TPK\n",__func__);
		tp_dev=TP_TPK;
	}else if((id1 == 0)&&(id2 == 0)&&(id3 == 0)){
		TPD_DEBUG("%s::TP_TRULY\n",__func__);
		tp_dev=TP_TRULY;
	}else{
		TPD_DEBUG("%s::TP_UNKNOWN\n",__func__);
		tp_dev=TP_TRULY;	
	}
	//printk("%s::id1:%d id2:%d id3:%d\n",__func__,id1,id2,id3);		
}

/*int lcd_id;
static int get_lcd_id(char *lcd_id_char)
{

	sscanf(lcd_id_char, "0x%x", &lcd_id);
	TPD_DEBUG("TP get lcd_id=%d ",lcd_id);
	return 1;
}
__setup("lcd_id=", get_lcd_id);*/

static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
	int rc, ret = 0;
	struct device_node *np;
	int temp_array[2];
	
	
	np = dev->of_node;
	ts->glove_mode_enabled = of_property_read_bool(np,
				"synaptics,glove-mode-enabled");
	ts->black_gesture_support = of_property_read_bool(np,
				"synaptics,black-gesture-enabled");
	ts->charger_pump_support = of_property_read_bool(np,
				"synaptics,charger-pump-support");
	TPD_DAILY("charger_pump_support %s \n",ts->charger_pump_support?"true":"false");
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_ERR("ts->irq_gpio not specified\n");	
	}
	ret = gpio_direction_input(ts->irq_gpio);
	
	ts->id1_gpio = of_get_named_gpio(np, "synaptics,id1-gpio", 0);
	if( ts->id1_gpio < 0 ){
		TPD_DEBUG("ts->id1_gpio not specified\n");	
	}
	TPD_DEBUG("synaptic:irq_gpio %d\n",ts->irq_gpio);
	ts->id2_gpio = of_get_named_gpio(np, "synaptics,id2-gpio", 0);
	if( ts->id2_gpio < 0 ){
		TPD_DEBUG("ts->id2_gpio not specified\n");	
	}
	ts->id3_gpio = of_get_named_gpio(np, "synaptics,id3-gpio", 0);	
	if( ts->id3_gpio < 0 ){
		TPD_DEBUG("ts->id3_gpio not specified\n");	
	}
	get_tp_id(ts->id1_gpio,ts->id2_gpio,ts->id3_gpio,ts);
	
	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);	
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");	
	}
	
	ts->enable2v8_gpio = of_get_named_gpio(np, "synaptics,enable2v8-gpio", 0);	
	if( ts->enable2v8_gpio < 0 ){
		TPD_DEBUG("ts->enable2v8_gpio not specified\n");	
	}

	ts->enable1v8_gpio = of_get_named_gpio(np, "synaptics,enable1v8-gpio", 0);	
	if( ts->enable1v8_gpio < 0 ){
		TPD_DEBUG("ts->enable1v8_gpio not specified\n");	
	}

	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_DEBUG("ts->max_num not specified\n");
		ts->max_num = 10;
	}
	
	rc = of_property_read_u32_array(np, "synaptics,button-map", button_map, 3);
	if(rc){
		TPD_DEBUG("button-map not specified\n");
		button_map[0] = 180;
		button_map[1] = 180;
		button_map[2] = 2021;
	}
	TPD_DEBUG("synaptics:button map readed is %d %d %d\n", button_map[0], button_map[1], button_map[2]);
	
	rc = of_property_read_u32_array(np, "synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  13;	
	    RX_NUM =  23;	
	}else{
        TX_NUM =  tx_rx_num[0];	
	    RX_NUM =  tx_rx_num[1];	
	}
	TPD_DEBUG("synaptics,tx-rx-num is %d %d \n", TX_NUM,RX_NUM);	
	
	rc = of_property_read_u32_array(np, "synaptics,display-coords", temp_array, 2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 720;
		LCD_HEIGHT = 1280;
	}else{
		LCD_WIDTH = temp_array[0];
		LCD_HEIGHT = temp_array[1];	
	}
	rc = of_property_read_u32_array(np, "synaptics,panel-coords", temp_array, 2);
	if(rc){
		ts->max_x = 0;
		ts->max_y = 0;		
	}else{
		ts->max_x = temp_array[0];
		ts->max_y = temp_array[1];	
	}
	TPD_DEBUG("synaptic:ts->irq_gpio:%d irq_flags:%u id1_gpio:%d id2_gpio:%d id3_gpio:%d max_num %d\n"
				,ts->irq_gpio, ts->irq_flags ,ts->id1_gpio, ts->id2_gpio, ts->id3_gpio, ts->max_num);
				
/***********power regulator_get****************/
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if( IS_ERR(ts->vdd_2v8) ){
		rc = PTR_ERR(ts->vdd_2v8);
		TPD_ERR("Regulator get failed vdd rc=%d\n", rc);	
	}	
	
	if (!IS_ERR(ts->vdd_2v8))
	{
		if(regulator_count_voltages(ts->vdd_2v8) > 0) {
			rc = regulator_set_voltage(ts->vdd_2v8, 2850000,2850000);
			if (rc) {
				dev_err(&ts->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
					//return rc;
			}
		}	
	}
    
	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if( IS_ERR(ts->vcc_i2c_1v8) ){
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		TPD_DEBUG("Regulator get failed vcc_i2c rc=%d\n", rc);
	}
	
	if (!IS_ERR(ts->vcc_i2c_1v8)){
		if(regulator_count_voltages(ts->vcc_i2c_1v8) > 0) {
			rc = regulator_set_voltage(ts->vcc_i2c_1v8, 1800000,
						   1800000);
			if (rc) {
				dev_err(&ts->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				//return rc;
			}
		}
	}
	
	if( ts->reset_gpio > 0){
		if( gpio_is_valid(ts->reset_gpio) ){
			rc = gpio_request(ts->reset_gpio, "rmi4-reset-gpio");
			if(rc){
				TPD_ERR("1 unable to request gpio [%d]\n", ts->reset_gpio);
			}
		}
	}
	
	if( ts->enable2v8_gpio > 0){
		if( gpio_is_valid(ts->enable2v8_gpio) ){
			rc = gpio_request(ts->enable2v8_gpio, "rmi4-enable2v8-gpio");
			if(rc){
				TPD_ERR("2 unable to request gpio [%d]\n", ts->enable2v8_gpio);
			}
		}
	}
	
	return rc;
}

#ifdef FINGER_PROTECT
int lcd_resume_ok = 0;
void lcd_wakeup_finger_protect(bool wakeup)
{
    TPD_ERR("%s wakeup = %d\n", __func__, wakeup);

    if(wakeup){
        lcd_resume_ok = 1;
        wake_up_interruptible(&waiter);
    } else {
        lcd_resume_ok = 0;
    }
}

static int synaptics_finger_entrance(void)
{
    TPD_DAILY("-->%s\n", __func__);
    finger_trigger = FINGER_PROTECT_ISTRIGGER;
    report_finger_protect = FINGER_PROTECT_NOTREADY;
    wake_up_interruptible(&waiter);
    TPD_DAILY("<--%s\n", __func__);
    return 0;
}

static int finger_protect_handler(void *unused)
{
    int ret = 0;
    uint8_t tmp[1536] = {0};
    int16_t baseline_data = 0, baseline_data_base = 0, delta_data;
    int x, y, z;
    int error_count = 0;

    do {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, finger_trigger != FINGER_PROTECT_NOTTRIGGER && lcd_resume_ok && i2c_ready);
        set_current_state(TASK_RUNNING);
        finger_trigger = FINGER_PROTECT_NOTTRIGGER;
        report_finger_protect = FINGER_PROTECT_NOTREADY;

        mutex_lock(&ts_g->mutex); 
        TPD_ERR("lcd_resume_ok = %d\n", lcd_resume_ok);
        ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0);
        ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, 0x80);
	    msleep(5);
        ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x1);
        ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
        ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x00);//set LSB 
        ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_DATA_BASE+2, 0x00);//set MSB
        ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
        //checkCMD();
        msleep(5);

        ret = synaptics_rmi4_i2c_read_block(ts_g->client, F54_ANALOG_DATA_BASE+3, RX_NUM*TX_NUM*2, tmp);        //read data
        for(x = 1; x < TX_NUM - 2; x++) {
            TPD_DEBUG("[%d]: ", x);
            for(y = 20; y < RX_NUM - 3; y++) {
                z = 2*RX_NUM*x + 2*y;
                baseline_data = (tmp[z+1]<<8) |tmp[z];
                baseline_data_base = (ts_g->raw_data[z+1]<<8) | ts_g->raw_data[z];
                delta_data = baseline_data - baseline_data_base;
                printk("%d [%d %d], ", delta_data, baseline_data, baseline_data_base);
                if((delta_data + 60) < 0) {
                    TPD_DEBUG("delta_data too large, delta_data = %d\n", delta_data);
                    error_count++;
                }
            }
            printk("\n");
            if(error_count > 2) {
                report_finger_protect = FINGER_PROTECT_TOUCH_DOWN;
                TPD_ERR("finger protect trigger!report_finger_protect = %d\n", report_finger_protect);
                error_count = 0;
                break;
            } else {
                report_finger_protect = FINGER_PROTECT_TOUCH_UP;
            }
        }
        
        //ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);    
        mutex_unlock(&ts_g->mutex); 
    } while (!kthread_should_stop());
    return 0;
}
#endif

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	#ifdef CONFIG_SYNAPTIC_RED
	struct remotepanel_data *premote_data = NULL;
	#endif
	struct synaptics_ts_data *ts = NULL;
	int ret = -1;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;	
    #ifdef FINGER_PROTECT
    int16_t baseline_data = 0;
    int x, y, z;
    #endif

	TPD_DEBUG("%s  3320 is called\n",__func__);
	is_touch= 0;

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if( ts == NULL ) {
		TPD_ERR("ts  kzalloc error\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	memset(ts, 0, sizeof(*ts));	

	atomic_set(&charge_plug_in_flag, 0);
	ts_g = ts;
	ts->boot_mode = get_boot_mode();
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
	speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
	if( !speedup_resume_wq ){
		TPD_ERR("create speedup_resume_wq error\n");	
       		 ret = -ENOMEM;
		 goto err_check_functionality_failed;
   	 }   
	INIT_WORK(&ts->speed_up_work,speedup_synaptics_resume);
	synaptics_parse_dts(&client->dev, ts);
	
	/***power_init*****/
	ret = tpd_power(ts, 1);	
	if( ret < 0 )
		TPD_ERR("regulator_enable is called\n");
	
	mutex_init(&ts->mutex);	
	

	ts->is_suspended = 0;
	ts->gesture_enable = 0;
	/*****power_end*********/
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ){
		TPD_ERR("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}	
	
    ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
	if( ret < 0 ) {
		ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
		if( ret < 0 ) {	
			TPD_ERR("synaptics is no exist!\n");
			ret = -ENODEV;
			goto err_check_functionality_failed;
		}
	}

	ts->i2c_device_test = ret;

	INIT_WORK(&ts->close_edge_product, close_edge_protect_function_s3320);
	synaptics_read_register_map(ts);

    if( (ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN) )
	{
        TPD_ERR("ts->boot_mode is %d\n", ts->boot_mode);
        ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00, 0x01);      //enter sleep mode
        msleep(5);
        ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00);
        TPD_ERR("TP sleep mode is %d\n", ret);
        
		return 0;
	}
    
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	printk("synaptics:before fw update,bootloader_mode = 0x%x\n", bootloader_mode);
	
	synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_DAILY("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);		
	TP_FW = CURRENT_FIRMWARE_ID;		
	sprintf(ts->fw_id,"0x%x",TP_FW);
	tp_info.version = ts->fw_id;
	
	memset(ts->fw_name,TP_FW_NAME_MAX_LEN,0);
	memset(ts->test_limit_name,TP_FW_NAME_MAX_LEN,0);

	if( (tp_dev == TP_G2Y) || (tp_dev == TP_TPK) )
		sprintf(ts->manu_name, "TP_TPK");
	if(tp_dev == TP_TRULY)
		sprintf(ts->manu_name, "TP_TRULY");
    if(tp_dev == TP_OFILM)
		sprintf(ts->manu_name, "TP_OFILM");

	tp_info.manufacture = ts->manu_name;

	tp_info.manufacture = "SAMSUNG_Default";

	tp_info.manufacture = "SYNAPTICS";
	strcpy(ts->fw_name,"tp/16027/16027_FW_S3320_JDI.img");
	strcpy(ts->test_limit_name,"tp/16027/16027_Limit_JDI.img");


	TPD_DEBUG("synatpitcs_fw: fw_name = %s \n",ts->fw_name);
	register_device_proc("tp", tp_info.version, tp_info.manufacture);

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if( !synaptics_wq ){
		TPD_ERR("creat synaptics_wq error\n");
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	} 

   // ret = synaptics_soft_reset(ts);
	ret = synaptics_init_panel(ts); // switch to normal work mode , can not go to sleep when power on
	if (ret < 0) {
		TPD_ERR("synaptics_init_panel failed\n");	
	}
	
	//Detect whether TP FW is error, max_x,max_y may be incoorect while it has been damaged!
	ret = synaptics_fw_check(ts);
	if(ret < 0 ) {	
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");		
	} else {
		force_update = 0;	
	}
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ) {
		TPD_ERR(" synaptics_ts_probe: disable interrupt failed\n");
	}
	ret = synaptics_input_init(ts);
	if(ret < 0) {
		TPD_ERR("synaptics_input_init failed!\n");			
	}
	ret = synaptics_tpd_button_init(ts);	
	if(ret < 0) {
		TPD_ERR("synaptics_tpd_button_init failed!\n");			
	}
#if defined(CONFIG_FB)
	ts->suspended = 0;
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);
#endif

	left_x1 = (limit_area * 1000)/100;
	right_x1 = LCD_WIDTH - left_x1;
	left_x2 = 2 * left_x1;
	right_x2 = LCD_WIDTH - 2 * left_x1;

#ifndef TPD_USE_EINT
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif 

#ifdef TPD_USE_EINT
	ret = synaptics_enable_interrupt(ts, 1);

	/****************
	shoud set the irq GPIO
	*******************/		
	if (gpio_is_valid(ts->irq_gpio)) {
		/* configure touchscreen irq gpio */
		ret = gpio_request(ts->irq_gpio,"rmi4_irq_gpio");
		if (ret) {
			TPD_ERR(" 3 unable to request gpio [%d]\n",ts->irq_gpio);
		}
		msleep(100);
	}
	
	TPD_DEBUG("synaptic:ts->client->irq is %d\n",ts->client->irq);
	ret = request_threaded_irq(ts->client->irq, NULL,
				synaptics_irq_thread_fn,
				ts->irq_flags | IRQF_ONESHOT,
				TPD_DEVICE, ts);		
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
#endif
	
		
	//modify rendong for ts_g should be exit for some proc file
	init_synaptics_proc(ts);
	
	if(ts->charger_pump_support)
		synaptics_chg_mode_enable = synaptics_charge_mode_enable;
#ifdef CONFIG_SYNAPTIC_RED	
	premote_data = remote_alloc_panel_data();
	if(premote_data) {
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->kpd			= ts->kpd;
		premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device(premote_data);

	}
#endif

#ifdef FINGER_PROTECT
	mutex_lock(&ts->mutex); 
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02); //forcecal
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set LSB 
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE+2, 0x00);//set MSB
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	//checkCMD();
    msleep(5);

	ret = synaptics_rmi4_i2c_read_block(ts->client, F54_ANALOG_DATA_BASE+3, RX_NUM*TX_NUM*2, ts->raw_data);     //read data
	for(x = 0; x < TX_NUM; x++)
	{
	    TPD_DAILY("[%d]: ", x);
	    for(y = 0; y < RX_NUM; y++)
	    {
	        z = 2*RX_NUM*x + 2*y;
	        baseline_data = (ts->raw_data[z+1]<<8) |ts->raw_data[z];
            if(tp_debug_daily_build)
                printk("%d, ", baseline_data);
	    }
	    if(tp_debug_daily_build)
	    	printk("\n");
	}

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);    
	mutex_unlock(&ts->mutex);
	thread = kthread_run(finger_protect_handler, 0, TPD_DEVICE);
#endif

	TPD_DEBUG("synaptics_ts_probe 3203: normal end\n");
    	return 0;

//exit_init_failed:
	free_irq(client->irq,ts);
exit_createworkqueue_failed:
	destroy_workqueue(synaptics_wq);
	synaptics_wq = NULL;

err_check_functionality_failed:
	 //wanghao 2016/7/11 delete , tp and touchkey have one power supply  
	 //tpd_power(ts, 0);
err_alloc_data_failed:
	kfree(ts);
	ts = NULL;
	ts_g = NULL;
	TPD_ERR("synaptics_ts_probe error : not normal end ret = %d \n",ret);
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	int attr_count;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	
	TPD_ERR("%s is called\n",__func__);
#ifdef CONFIG_SYNAPTIC_RED
	unregister_remote_device();
#endif

#if defined(CONFIG_FB)
	if( fb_unregister_client(&ts->fb_notif) )
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
	
#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif	

	for(attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++){
		sysfs_remove_file(&ts->input_dev->dev.kobj, &attrs_oppo[attr_count].attr);
	}
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
  	//wanghao 2016/7/11 delete , tp and touchkey have one power supply  
  	//tpd_hw_pwroff(ts);
	return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DAILY("%s: is called\n", __func__);

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
			return -1;
	}
	if(ts->loading_fw) {
	    TPD_ERR("FW is updating while suspending");
			return -1;
    }

	mutex_lock(&ts->mutex);
	is_touch = 0;
	if (!ts->is_suspended) {
		ts->is_suspended = 1;
	} else {
		TPD_ERR("%s: do not suspend twice.\n", __func__);
		goto OUT;
	}
/***********report Up key when suspend********/
	input_report_key(ts->input_dev, KEY_MENU, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_BACK, 0);
	input_sync(ts->input_dev);
	synaptics_free_fingers(ts);

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if (ret < 0){
		TPD_ERR("%s page select failed\n", __func__);
	}else{
		ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL02, 0x03);
	}
#ifdef SUPPORT_GLOVES_MODE
	if( ts->glove_enable ){
		/* page select = 0x4 */
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			ret = 0;
			goto OUT;
		}
		//printk("glove mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00, 0x02 ); 	
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			ret = 0;
			goto OUT;
		}
	}
#endif

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
		if(ts->charger_pump_support && atomic_read(&charge_plug_in_flag) )
		{
			TPD_ERR("%s, page 4 F51_CUSTOM_CTRL31=0x%x write %d \n",__func__,F51_CUSTOM_CTRL31,0);//0x0424
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 4);
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
			}
			ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL31&0x00ff, 0 );
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
			}
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0);
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
			}

		}
//end

#ifdef SUPPORT_GESTURE
	if( ts->double_enable == 1){
		synaptics_enable_interrupt_for_gesture(ts, 1);
		TPD_ERR("synaptics:double_tap end suspend\n");
			ret = 0;
			goto OUT;
	}
#endif

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret){
		TPD_DEBUG("%s: cannot disable interrupt\n", __func__);
		ret = -1;
		goto OUT;
	}
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01);
	if( ret < 0 ){
		TPD_ERR("%s: control tm1400 to sleep failed\n", __func__);
		ret = -1;
		goto OUT;
	}
	//TPDTM_DMESG("%s:normal end\n", __func__);
    ret = 0 ;
OUT:
	mutex_unlock(&ts->mutex);
	return ret;
}

/**
 * synaptics_resume_prepare -   release inter pins before lcd resume flow(only gesture mode will hold those pins).
 * @chip_info: struct include i2c resource.
 * Return NULL.
 */
static void synaptics_resume_prepare(struct synaptics_ts_data *ts)
{
	synaptics_soft_reset_for_prepare(ts);
	msleep(10);
}

static int synaptics_ts_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s is called\n", __func__);
	is_touch = 0;
	if (!ts->is_suspended) {
	    TPD_ERR("%s: do not resume twice.\n", __func__);
	    return 0;
	}
	ts->is_suspended = 0;
	if(ts->loading_fw)
	    return 0;

	free_irq(ts->client->irq, ts);
	mutex_lock(&ts->mutex);
	synaptics_resume_prepare(ts);
	mutex_unlock(&ts->mutex);
	queue_work(speedup_resume_wq, &ts->speed_up_work);
	return 0;
}

static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = ts_g;
	TPD_DAILY("%s is called\n", __func__);
/***********report Up key when resume********/	
	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}
	in_which_area = 0;
	synaptics_free_fingers(ts);
	ts->pre_btn_state = 0;

	if( ts->reset_gpio > 0 ) {
		gpio_direction_output(ts->reset_gpio, 1);
	}

	//free_irq(ts->client->irq, ts);
	mutex_lock(&ts->mutex);
#ifndef TPD_USE_EINT
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
	/***Reset TP ******/ 
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPD_ERR("%s: failed for page select try again later\n", __func__);
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if( ret < 0 ){
			TPD_ERR("%s: failed for page select try again later\n", __func__);
		}
	}
	ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL02, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE);
	ret = synaptics_soft_reset(ts);
	msleep(50);
/*****Gesture Register********/
#ifdef SUPPORT_GESTURE
        if( ts->double_enable == 1){
            ret = synaptics_enable_interrupt_for_gesture(ts, 0);
            if(ret < 0)
                ret = synaptics_enable_interrupt_for_gesture(ts, 0);
        }
#endif

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
	if(ts->charger_pump_support && atomic_read(&charge_plug_in_flag) )
	{
			TPD_ERR("%s, page 4 F51_CUSTOM_CTRL31=0x%x write charge_plug_status=%d \n",__func__,F51_CUSTOM_CTRL31,charge_plug_status);//0x0424
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 4);  
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
				goto ERR_RESUME;
			}
			ret = i2c_smbus_write_byte_data(ts_g->client, F51_CUSTOM_CTRL31&0x00ff,charge_plug_status);  
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
			}
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0);  
			if( ret < 0 ){
				TPD_ERR("%s i2c error\n",__func__);
			}
	}
//end 

/*****Normal Init TP********/
    ret = synaptics_init_panel(ts);
	if (ret < 0){
		TPD_ERR("%s: TP init failed\n", __func__);
		goto ERR_RESUME;
	}	
		ret = request_threaded_irq(ts->client->irq, NULL,
				synaptics_irq_thread_fn,
				ts->irq_flags | IRQF_ONESHOT,
				TPD_DEVICE, ts);		
	if (ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
#endif
	ret = synaptics_enable_interrupt(ts, 1);
	if (ret){
		TPD_ERR("%s:can't  enable interrupt!\n", __func__);
		goto ERR_RESUME;
	}	
ERR_RESUME:
	TPD_DEBUG("%s:ERR_RESUME!\n", __func__);	
	mutex_unlock(&ts->mutex);
	return;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s: is called\n", __func__);	
	if (ts->gesture_enable == 1){		
		/*enable gpio wake system through intterrupt*/		
		enable_irq_wake(ts->client->irq);
	}
	disable_irq_nosync(ts->client->irq);	
    #ifdef FINGER_PROTECT
    i2c_ready = I2C_NOTREADY;
    TPD_ERR("%s call finger protect!i2c_ready = %d\n", __func__, i2c_ready);
    #endif
	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s is called\n", __func__);
	if (ts->gesture_enable == 1){
		/*disable gpio wake system through intterrupt*/	
		disable_irq_wake(ts->client->irq);
	}
	enable_irq(ts->client->irq);
    #ifdef FINGER_PROTECT
    i2c_ready = I2C_READY;
    TPD_ERR("%s call finger protect!i2c_ready = %d\n", __func__, i2c_ready);
    if(finger_trigger == FINGER_PROTECT_ISTRIGGER)
        wake_up_interruptible(&waiter);
    #endif
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	//int ret;
	struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);

    //to aviod some kernel bug (at fbmem.c some local veriable are not initialized)
	if(event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK)
		return 0;

    if((evdata) && (evdata->data) && (ts) && (ts->client)) {
        blank = evdata->data;
        if( (event == FB_EARLY_EVENT_BLANK ) && (*blank == FB_BLANK_UNBLANK)) {
            TPD_DEBUG("%s going TP resume\n", __func__);
			if(ts->suspended == 1){
				synaptics_ts_resume(&ts->client->dev);
				ts->suspended = 0;
			}
        } else if ((event == FB_EVENT_BLANK ) && (*blank == FB_BLANK_POWERDOWN)) {
            TPD_DEBUG("%s : going TP suspend\n", __func__);
			if(ts->suspended == 0) {
				ts->suspended = 1;
				synaptics_ts_suspend(&ts->client->dev);
			}
        }
    }

	return 0;
}
#endif

static int __init tpd_driver_init(void)
{
	TPD_ERR("Synaptics_3320:%s is called\n", __func__);
	 if( i2c_add_driver(&tpd_i2c_driver)!= 0 ){
        TPDTM_DMESG("unable to add i2c driver.\n");
        return -1;
    }	
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
 	i2c_del_driver(&tpd_i2c_driver);
	if(synaptics_wq ){
		destroy_workqueue(synaptics_wq);
		synaptics_wq = NULL;
	}	
	return;
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics S3203 Touchscreen Driver");
MODULE_LICENSE("GPL");

