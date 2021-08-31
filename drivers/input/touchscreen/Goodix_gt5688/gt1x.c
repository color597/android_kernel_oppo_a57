/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.4   
 * Release Date:  2015/07/10
 */
 
#include <linux/irq.h>
#include "gt1x.h"
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include <soc/oppo/boot_mode.h>
#include "circle_point.h"



static struct work_struct gt1x_work;
struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";
#ifdef GTP_CONFIG_OF
int gt1x_rst_gpio;
int gt1x_int_gpio;
int vdd_ana_supply;
#endif
//static int boot_mode;
static int gt1x_id1;
static int gt1x_id2;
static int gt1x_id3;
int config_id;
char manu_name[12];
int firmware_id;

struct mutex gt_suspend_lock;
struct mutex i2c_access;

#ifdef SUPPORT_GESTURE
static  Point Point_input[64];
static  Point Point_output[4];
struct Coordinate Point_start;
struct Coordinate Point_end;
struct Coordinate Point_1st;
struct Coordinate Point_2nd;
struct Coordinate Point_3rd;
struct Coordinate Point_4th;
uint32_t clockwise=1;

atomic_t double_enable;
atomic_t is_in_suspend;
static uint32_t gesture;
uint32_t gesture_upload;

#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76 
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x63 
#define RIGHT_VEE_DETECT     0x3e
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48 
#define DOUUPSWIP_DETECT     0x4E
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT			 0x6D
#define W_DETECT			 0x77


#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define RightVee            4   // >
#define LeftVee             5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W	
#define CustomGestrue       14  //Custom
#endif


static int gt1x_register_powermanger(void);
static int gt1x_unregister_powermanger(void);

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static spinlock_t irq_lock;
 s32 irq_is_disable = 0;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();
	
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

#ifndef GTP_CONFIG_OF
int gt1x_power_switch(s32 state)
{
    return 0;
}
#endif

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#if GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
#error Need to get charger status of your platform.
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_HANDLED: interrupt handled successfully
 */
irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	GTP_DEBUG_FUNC();
    gt1x_irq_disable();
	queue_work(gt1x_wq, &gt1x_work);
	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
	input_report_key(input_dev, BTN_TOUCH, 1);
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
	//input_report_key(input_dev, BTN_TOOL_FINGER, 1);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	//input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	//input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_report_key(input_dev, BTN_TOOL_FINGER, 0);

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

#ifdef SUPPORT_GESTURE
int ClockWise(Point *p,int n)
{

	int i,j,k;
	int count = 0;
	//double z;
	long int z;
	if (n < 3)
		return -1;
	for (i=0;i<n;i++) 
	{
		j = (i + 1) % n;
		k = (i + 2) % n;
		if( (p[i].x==p[j].x) && (p[j].x==p[j].y) )
		   continue;
		z = (p[j].x - p[i].x) * (p[k].y - p[j].y);
		z -= (p[j].y - p[i].y) * (p[k].x - p[j].x);
		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}
    
	printk("ClockWise count = %d\n",count);
	if (count > 0)
		return 1; 
	else if (count < 0)
		return 0;
	else
		return 0;
}
#endif

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };
	#if GTP_GESTURE_WAKEUP
	u8 doze_buf[3];
	u8 clear_buf[1];
	u8 coordinate_single[260];
	u8 coordinate_size;
	int i = 0, j = 0;
	#endif
	
    if (update_info.status) {
        GTP_DEBUG("Ignore interrupts during fw update.");
        return;
    }



  mutex_lock(&i2c_access);
#if GTP_GESTURE_WAKEUP
#ifdef SUPPORT_GESTURE
	if (DOZE_ENABLED == gesture_doze_status)
	{
		ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 2);
		GTP_DEBUG("0x814C = 0x%02X,ret=%d\n", doze_buf[0],ret);
		if (ret == 0 )
		{
			if(doze_buf[0] != 0)
			{
				memset(coordinate_single, 0, 260);
				coordinate_size=doze_buf[1];
				//GTP_ERROR("report gesture::original gesture=%d  coordinate_size=%d \n",doze_buf[0],coordinate_size);
				if(coordinate_size>64) //mingqiang.guo@phone.bsp add coordinate_size*4 can not large > 260
					coordinate_size = 64;
				ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE_DETAIL, coordinate_single, coordinate_size*4);
				switch (doze_buf[0])
				{
					case DTAP_DETECT:
						gesture = DouTap;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
						Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_1st.x = 0;
						Point_1st.y = 0;
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case UP_VEE_DETECT :
						gesture = UpVee;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
						Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case DOWN_VEE_DETECT :
						gesture = DownVee;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
						Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case LEFT_VEE_DETECT:
						gesture =  LeftVee;
 
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
						Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);
						Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
						Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case RIGHT_VEE_DETECT :
						gesture =  RightVee;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
						Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case CIRCLE_DETECT  :
						gesture =  Circle;
						j = 0;
						for(i = 0; i < coordinate_size;i++)
						{
							Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j+1] << 8);
							Point_input[i].y = coordinate_single[j+2] |  (coordinate_single[j+3] << 8);
							j = j+4;
							GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n",i,Point_input[i].x,i,Point_input[i].y);
						}

						clockwise = ClockWise(&Point_input[0],coordinate_size-2);
						GetCirclePoints(&Point_input[0], coordinate_size,Point_output);
						Point_start.x = Point_input[0].x;
						Point_start.y = Point_input[0].y;

						Point_end.x = Point_input[coordinate_size-1].x;
						Point_end.y = Point_input[coordinate_size-1].y;

						Point_1st.x = Point_output[0].x;
						Point_1st.y = Point_output[0].y;
 
						Point_2nd.x = Point_output[1].x;
						Point_2nd.y = Point_output[1].y;
 
						Point_3rd.x = Point_output[2].x;
						Point_3rd.y = Point_output[2].y;
 
						Point_4th.x = Point_output[3].x;
						Point_4th.y = Point_output[3].y;
					break;

					case DOUSWIP_DETECT  :
						gesture =  DouSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
						Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
						Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case DOUUPSWIP_DETECT:
						gesture =  DouSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
						Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
						Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case RIGHT_SLIDE_DETECT :
						gesture =  Left2RightSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
						Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = 0;
						Point_1st.y = 0;
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case LEFT_SLIDE_DETECT :
						gesture =  Right2LeftSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = 0;
						Point_1st.y = 0;
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case DOWN_SLIDE_DETECT  :
						gesture =  Up2DownSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = 0;
						Point_1st.y = 0;
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case UP_SLIDE_DETECT :
						gesture =  Down2UpSwip;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
						Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_1st.x = 0;
						Point_1st.y = 0;
						Point_2nd.x = 0;
						Point_2nd.y = 0;
						Point_3rd.x = 0;
						Point_3rd.y = 0;
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;	

					case M_DETECT  :
						gesture =  Mgestrue;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
						Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);
						Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
						Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
						Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;

					case W_DETECT :
						gesture =  Wgestrue;
						Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
						Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
						Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
						Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);
						Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
						Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
						Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
						Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);
						Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
						Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
						Point_4th.x = 0;
						Point_4th.y = 0;
						clockwise = 0 ;
					break;	

					default:
						if((doze_buf[0]>=1)&&(doze_buf[0]<=15))
						{
							gesture =  doze_buf[0] + OPPO_CUSTOM_GESTURE_ID_BASE;  //user custom gesture
							Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
							Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
							Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
							Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
							Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
							Point_2nd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
							Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
							Point_3rd.x = coordinate_single[16] |  (coordinate_single[17] << 8);
							Point_3rd.y = coordinate_single[18] |  (coordinate_single[19] << 8);
							Point_3rd.x = coordinate_single[20] |  (coordinate_single[21] << 8);
							Point_3rd.y = coordinate_single[22] |  (coordinate_single[23] << 8);
							Point_4th.x = 0;
							Point_4th.y = 0;
							clockwise = 0 ;
						}
						else
						{
							gesture =  doze_buf[0];
						}
					break;
				}
				GTP_INFO("report gesture::detect %s gesture\n", gesture == DouTap ? "double tap" :
                                                        gesture == UpVee ? "up vee" :
                                                        gesture == DownVee ? "down vee" :
                                                        gesture == LeftVee ? "(<)" :
                                                        gesture == RightVee ? "(>)" :
                                                        gesture == Circle ? "circle" :
															gesture == DouSwip ? "(||)" :
                                                        gesture == Left2RightSwip ? "(-->)" :
                                                        gesture == Right2LeftSwip ? "(<--)" :
                                                        gesture == Up2DownSwip ? "up to down |" :
                                                        gesture == Down2UpSwip ? "down to up |" :
                                                        gesture == Mgestrue ? "(M)" :
					gesture == Wgestrue ? "(W)" : "oppo custom gesture"); 
				if(gesture > OPPO_CUSTOM_GESTURE_ID_BASE )
				{
					ret = gesture_event_handler(input_dev);
					if (ret >= 0) 
					{
						gt1x_irq_enable();
						mutex_unlock(&i2c_access);
						goto exit_eint;
					}
				}
				else
				{
					gesture_upload=gesture;
					input_report_key(input_dev, KEY_F4, 1);
					input_sync(input_dev);
					input_report_key(input_dev, KEY_F4, 0);
					input_sync(input_dev);
					clear_buf[0] = 0x00;
					gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);

				}

			}
			else       
			{
				GTP_ERROR("report gesture::Unknow gesture!!!\n");
			}	
		}
		mutex_unlock(&i2c_access);	
		goto exit_eint;
	}
#endif
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend...");
		mutex_unlock(&i2c_access);
        		goto exit_eint;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		mutex_unlock(&i2c_access);
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00) {
		gt1x_request_event_handler();
	}

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			//GTP_ERROR("buffer not ready:0x%02x", finger);
			mutex_unlock(&i2c_access);
			goto exit_eint;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret) {
		goto exit_work_func;
	}
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0) {
		goto exit_work_func;
	}
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0) {
			GTP_ERROR("I2C write end_cmd  error!");
		}
	}
	mutex_unlock(&i2c_access);
exit_eint:
    gt1x_irq_enable();
    
}

/* 
 * Devices Tree support, 
*/
#ifdef GTP_CONFIG_OF

//static struct regulator *vdd_ana;
static struct regulator *vcc_i2c;

/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev)
{
	struct device_node *np;
    int ret = 0;

    if (!dev)
        return -ENODEV;
    
    np = dev->of_node;
	gt1x_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	gt1x_rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);
	vdd_ana_supply = of_get_named_gpio(np, "vdd_ana-supply", 0);
	gt1x_id1 = of_get_named_gpio(np, "gt1x_id1-gpio", 0);
	gt1x_id2 = of_get_named_gpio(np, "gt1x_id2-gpio", 0);
	gt1x_id3 = of_get_named_gpio(np, "gt1x_id3-gpio", 0);
		
    if (!gpio_is_valid(gt1x_int_gpio) || !gpio_is_valid(gt1x_rst_gpio) || !gpio_is_valid(vdd_ana_supply)) {
        GTP_ERROR("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
            gt1x_int_gpio, gt1x_rst_gpio);
        return -EINVAL;
    }
	if (!gpio_is_valid(gt1x_id1) || !gpio_is_valid(gt1x_id2) || !gpio_is_valid(gt1x_id3)) {
		GTP_ERROR("Invalid GPIO, gt1x_id1-gpio:%d, gt1x_id2-gpio:%d, gt1x_id3-gpio:%d", gt1x_id1, gt1x_id2, gt1x_id3);
		return -EINVAL;
	}

    /*vdd_ana = regulator_get(dev, "vdd_ana");
    if (IS_ERR(vdd_ana)) {
	    GTP_ERROR("regulator get of vdd_ana failed");
	    ret = PTR_ERR(vdd_ana);
	    vdd_ana = NULL;
	    return ret;
    }*/

	vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c)) {
		GTP_ERROR("regulator get of vcc_i2c failed");
		ret = PTR_ERR(vcc_i2c);
		vcc_i2c = NULL;
		goto ERR_GET_VCC;
	}
    return 0;
ERR_GET_VCC:
    //regulator_put(vdd_ana);
    //vdd_ana = NULL;
    return ret;

}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_power_switch(int on)
{

	int ret;
	struct i2c_client *client = gt1x_i2c_client;

    	if (!client || !vdd_ana_supply || !vcc_i2c)
    	{
    		GTP_ERROR("%s failed!\n", __func__);
        		return -1;
    	}	
	if (on) {
		GTP_DEBUG("GTP power on.");
		//ret = regulator_enable(vdd_ana);
		GTP_GPIO_OUTPUT(vdd_ana_supply, 1);
		udelay(2);
		ret = regulator_enable(vcc_i2c);
	} else {
		GTP_DEBUG("GTP power off.");
		ret = regulator_disable(vcc_i2c);
		udelay(2);
		//ret = regulator_disable(vdd_ana);
		GTP_GPIO_OUTPUT(vdd_ana_supply, 0);
	}
	mdelay(10);
	return ret;

}
#endif

static void gt1x_remove_gpio_and_power(void)
{
    if (gpio_is_valid(gt1x_int_gpio))
        gpio_free(gt1x_int_gpio);

    if (gpio_is_valid(gt1x_rst_gpio))
        gpio_free(gt1x_rst_gpio);
    
#ifdef GTP_CONFIG_OF      
    if (vcc_i2c)
        regulator_put(vcc_i2c);

    //if (vdd_ana)
        //regulator_put(vdd_ana);
    if (gpio_is_valid(vdd_ana_supply))
        gpio_free(vdd_ana_supply);
#endif

    if (gt1x_i2c_client && gt1x_i2c_client->irq){
		free_irq(gt1x_i2c_client->irq, gt1x_i2c_client);
	}

}


/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);

	ret = gpio_request(vdd_ana_supply, "vdd_ana_supply");
	if(ret < 0)
	{
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) vdd_ana_supply, ret);
		ret = -ENODEV;
	}

	ret = gpio_request(gt1x_id1, "GTP_TP_ID1");
	if(ret < 0)
	{
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) gt1x_id1, ret);
		ret = -ENODEV;
	}
	ret = gpio_request(gt1x_id2, "GTP_TP_ID2");
	if(ret < 0)
	{
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) gt1x_id2, ret);
		ret = -ENODEV;
	}
	ret = gpio_request(gt1x_id3, "GTP_TP_ID3");
	if(ret < 0)
	{
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) gt1x_id3, ret);
		ret = -ENODEV;
	}
	GTP_GPIO_AS_INPUT(gt1x_id1);
	GTP_GPIO_AS_INPUT(gt1x_id2);
	GTP_GPIO_AS_INPUT(gt1x_id3);

	if (ret < 0) {
		gpio_free(GTP_RST_PORT);
		gpio_free(GTP_INT_PORT);
		gpio_free(vdd_ana_supply);
	}

	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	//const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
		ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler, IRQ_TYPE_EDGE_RISING, gt1x_i2c_client->name, gt1x_i2c_client);
		if (ret) {
			GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			gpio_free(GTP_INT_PORT);

			return -1;
		} else {
			gt1x_irq_disable();
			return 0;
		}

	return  0;
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
    input_mt_init_slots(input_dev, 16, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(input_dev, 16); 
#endif
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_GES_REGULAR);
    	input_set_capability(input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}

static int gt1x_get_sensor_id(void)
{
	int ID1 = 0, ID2 = 0, ID3 = 0;

	GTP_INFO("%s", __func__);
	if (GTP_TP_ID1  >= 0)
		ID1 = gpio_get_value(GTP_TP_ID1);
	if(GTP_TP_ID2 >= 0)
		ID2 = gpio_get_value(GTP_TP_ID2);
	if(GTP_TP_ID3 >= 0)
		ID3 = gpio_get_value(GTP_TP_ID3);
	GTP_INFO("ID1 = %d ID2 = %d ID3 = %d", ID1, ID2, ID3);

	if(ID1 == 1 && ID2 == 0 && ID3 == 0) {
		GTP_INFO("TP_OFILM");
		firmware_id = 0xBC051100;
		config_id = 0;
		strcpy(manu_name, "TP_OFILM");
	} else if(ID1 == 0 && ID2 == 1 && ID3 == 0) {
		GTP_INFO("TP_BIEL");
		firmware_id = 0xBC051600;
		config_id = 1;
		strcpy(manu_name, "TP_BIEL");
	}
	return 0;
}


/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int boot_mode;
	s32 ret = -1;
#if GTP_AUTO_UPDATE
	struct task_struct *thread = NULL;
#endif
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);
		
	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	
#ifdef GTP_CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		gt1x_parse_dt(&client->dev);
	}
#endif
	
	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return ret;
	}

	gt1x_get_sensor_id();
	//wanghao@bsp 2016/08/08 add for drop power in factory mode
	boot_mode = get_boot_mode();
	if( (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) ) {
		GTP_INFO("boot_mode is %d, Drop power in FTM mode!\n", boot_mode);
		GTP_GPIO_OUTPUT(vdd_ana_supply, 0);
		msleep(20);
		gpio_get_value(vdd_ana_supply);
		GTP_INFO("vdd_ana_supply is %d\n", vdd_ana_supply);
		return 0;
	}

	mutex_init(&gt_suspend_lock);
	mutex_init(&i2c_access);

	ret = gt1x_init();
	if(ret < 0) {
		GTP_ERROR("gt1x_init failed, quit probe!");
		return ret;
	}
	INIT_WORK(&gt1x_work, gt1x_ts_work_func);
	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

#if GTP_GESTURE_WAKEUP
	enable_irq_wake(client->irq);
#endif

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_ERROR("Failed to create auto-update thread: %d.", ret);
	}
#endif
	gt1x_register_powermanger();
#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable,0); 
	atomic_set(&is_in_suspend,0);
#endif  

	return 0;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");
	gt1x_unregister_powermanger();

#if GTP_GESTURE_WAKEUP
	disable_irq_wake(client->irq);
#endif
    gt1x_deinit();
	input_unregister_device(input_dev);
    gt1x_remove_gpio_and_power();

    return 0;
}

#if   defined(CONFIG_FB)	
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block gt1x_fb_notifier;

static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

#if GTP_INCELL_PANEL
    #ifndef FB_EARLY_EVENT_BLANK
        #error Need add FB_EARLY_EVENT_BLANK to fbmem.c
    #endif
    
	if (ev_data && ev_data->data && event == FB_EARLY_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
        }
    }
#else
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			mutex_lock(&gt_suspend_lock);
			gt1x_rawdiff_mode=0;
			atomic_set(&is_in_suspend,0);
			GTP_INFO("is_in_suspend %d\n", atomic_read(&is_in_suspend));
			gt1x_resume();
			mutex_unlock(&gt_suspend_lock);
        }
    }
#endif

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			mutex_lock(&gt_suspend_lock);
			atomic_set(&is_in_suspend,1);
			GTP_INFO("is_in_suspend %d\n", atomic_read(&is_in_suspend));
			gt1x_suspend();
			mutex_unlock(&gt_suspend_lock);
		}
	}

	return 0;
}
#elif defined(CONFIG_PM)
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
	return gt1x_suspend();
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
	gt1x_rawdiff_mode=0;
	return gt1x_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_pm_suspend,
	.resume = gt1x_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_suspend();
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_resume();
}

static struct early_suspend gt1x_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = gt1x_ts_early_suspend,
	.resume = gt1x_ts_late_resume,
};
#endif


static int gt1x_register_powermanger(void)
{
#if   defined(CONFIG_FB)
	gt1x_fb_notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&gt1x_fb_notifier);
	
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&gt1x_early_suspend);
#endif	
	return 0;
}

static int gt1x_unregister_powermanger(void)
{
#if   defined(CONFIG_FB)
	fb_unregister_client(&gt1x_fb_notifier);
		
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

#ifdef GTP_CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
		{.compatible = "goodix,gt1x",},
		{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef GTP_CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		   .pm = &gt1x_ts_pm_ops,
#endif
		   },
};

/**   
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");

	
	
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}

	return i2c_add_driver(&gt1x_ts_driver);
}

/**   
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq) {
		destroy_workqueue(gt1x_wq);
	}
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
