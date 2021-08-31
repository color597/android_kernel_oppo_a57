/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.£¬
 * VENDOR_EDIT
 * File       : touchpanel_common_driver.c
 * Description: Source file for Touch common driver
 * Version   : 1.0
 * Date        : 2016-09-02
 * Author    : Tong.han@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 * Revision 1.1, 2016-09-09, Tong.han@Bsp.Group.Tp, modify based on gerrit review result(http://gerrit.scm.adc.com:8080/#/c/223721/)
 ****************************************************************/
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#ifndef TPD_USE_EINT
#include <linux/hrtimer.h>
#endif

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "touchpanel_common.h"
#include "util_interface/touch_interfaces.h"

/*******Part0:LOG TAG Declear************************/
#define TPD_PRINT_POINT_NUM 150
#define TPD_DEVICE "touchpanel"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_SPECIFIC_PRINT(count, a, arg...)\
    do{\
        if (count++ == TPD_PRINT_POINT_NUM || tp_debug) {\
            TPD_INFO(TPD_DEVICE ": " a, ##arg);\
            count = 0;\
        }\
    }while(0)

/*******Part1:Global variables Area********************/
unsigned int tp_debug = 0;
static struct touchpanel_data *g_tp = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

/*******Part2:declear Area********************************/
static void speedup_resume(struct work_struct *work);
void esd_handle_switch(struct esd_information *esd_info, bool flag);
static void tp_btnkey_release(struct touchpanel_data *ts);
static void tp_touch_release(struct touchpanel_data *ts);

#ifdef TPD_USE_EINT
static irqreturn_t tp_irq_thread_fn(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

/*******Part3:Function  Area********************************/
/**
 * operate_mode_switch - switch work mode based on current params
 * @ts: touchpanel_data struct using for common driver
 *
 * switch work mode based on current params(gesture_enable, limit_enable, glove_enable)
 * Do not care the result: Return void type
 */
void operate_mode_switch(struct touchpanel_data *ts)
{
    if (!ts->ts_ops->mode_switch) {
        TPD_INFO("not support ts_ops->mode_switch callback\n");
        return;
    }
    if (ts->is_suspended) {
        if (ts->black_gesture_support) {
            if (ts->gesture_enable == 1) {
                ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
                ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
            } else {
                ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, false);
                ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
            }
        } else
            ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
    } else {
        if (ts->black_gesture_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, false);

        if (ts->edge_limit_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_EDGE, ts->limit_enable);

        if (ts->glove_mode_support)
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, ts->glove_enable);

        ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
    }
}

static void tp_touch_down(struct touchpanel_data *ts, struct point_info points, int touch_report_num)
{
    static int last_width_major;
    static int point_num = 0;

    if (ts->input_dev == NULL)
        return;

    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);

    if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY) {
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, points.z);
    } else {
        if (touch_report_num == 1) {
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, points.width_major);
            last_width_major = points.width_major;
        } else if (!(touch_report_num & 0x7f)) {
            //if touch_report_num == 127, every 127 points, change width_major
            //down and keep long time, auto repeat per 5 seconds, for weixing
            if (last_width_major == points.width_major) {
                last_width_major = points.width_major + 1;
            } else {
                last_width_major = points.width_major;
            }
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, last_width_major);
        }
    }

    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, points.y);

    TPD_SPECIFIC_PRINT(point_num, "Touchpanel:Down[%4d %4d %4d]\n", points.x, points.y, points.z);

#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts->input_dev);
#endif
}

static void tp_touch_up(struct touchpanel_data *ts)
{
    if (ts->input_dev == NULL)
        return;

    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(ts->input_dev);
#endif
}

static void tp_exception_handle(struct touchpanel_data *ts)
{
    if (!ts->ts_ops->reset) {
        TPD_INFO("not support ts->ts_ops->reset callback\n");
        return;
    }

    ts->ts_ops->reset(ts->chip_data);    // after reset, all registers set to default
    operate_mode_switch(ts);

    tp_btnkey_release(ts);
    tp_touch_release(ts);
}

static void tp_geture_info_transform(struct gesture_info * gesture, struct resolution_info *resolution_info)
{
    gesture->Point_start.x = gesture->Point_start.x * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_start.y = gesture->Point_start.y * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_end.x   = gesture->Point_end.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_end.y   = gesture->Point_end.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_1st.x   = gesture->Point_1st.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_1st.y   = gesture->Point_1st.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_2nd.x   = gesture->Point_2nd.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_2nd.y   = gesture->Point_2nd.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_3rd.x   = gesture->Point_3rd.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_3rd.y   = gesture->Point_3rd.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
    gesture->Point_4th.x   = gesture->Point_4th.x   * resolution_info->LCD_WIDTH  / (resolution_info->max_x);
    gesture->Point_4th.y   = gesture->Point_4th.y   * resolution_info->LCD_HEIGHT / (resolution_info->max_y);
}

static void tp_gesture_handle(struct touchpanel_data *ts)
{
    struct gesture_info gesture_info_temp;

    if (!ts->ts_ops->get_gesture_info) {
        TPD_INFO("not support ts->ts_ops->get_gesture_info callback\n");
        return;
    }

    memset(&gesture_info_temp, 0, sizeof(struct gesture_info));
    ts->ts_ops->get_gesture_info(ts->chip_data, &gesture_info_temp);
    tp_geture_info_transform(&gesture_info_temp, &ts->resolution_info);

    TPD_INFO("detect %s gesture\n", gesture_info_temp.gesture_type == DouTap ? "double tap" :
            gesture_info_temp.gesture_type == UpVee ? "up vee" :
            gesture_info_temp.gesture_type == DownVee ? "down vee" :
            gesture_info_temp.gesture_type == LeftVee ? "(>)" :
            gesture_info_temp.gesture_type == RightVee ? "(<)" :
            gesture_info_temp.gesture_type == Circle ? "circle" :
            gesture_info_temp.gesture_type == DouSwip ? "(||)" :
            gesture_info_temp.gesture_type == Left2RightSwip ? "(-->)" :
            gesture_info_temp.gesture_type == Right2LeftSwip ? "(<--)" :
            gesture_info_temp.gesture_type == Up2DownSwip ? "up to down |" :
            gesture_info_temp.gesture_type == Down2UpSwip ? "down to up |" :
            gesture_info_temp.gesture_type == Mgestrue ? "(M)" :
            gesture_info_temp.gesture_type == Wgestrue ? "(W)" : "unknown");

    if (gesture_info_temp.gesture_type != UnkownGesture) {
        memcpy(&ts->gesture, &gesture_info_temp, sizeof(struct gesture_info));
        input_report_key(ts->input_dev, KEY_F4, 1);
        input_sync(ts->input_dev);
        input_report_key(ts->input_dev, KEY_F4, 0);
        input_sync(ts->input_dev);
    }
}

void tp_touch_btnkey_release(void)
{
    struct touchpanel_data *ts = g_tp;

    if (!ts) {
        TPD_INFO("ts is NULL\n");
        return ;
    }

    tp_touch_release(ts);
    tp_btnkey_release(ts);
}

static void tp_touch_release(struct touchpanel_data *ts)
{
    int i = 0;

#ifdef TYPE_B_PROTOCOL
    for (i = 0; i < ts->max_num; i++) {
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
    }
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
    input_sync(ts->input_dev);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
    input_mt_sync(ts->input_dev);
    input_sync(ts->input_dev);
#endif
    TPD_INFO("release all touch point and key, clear tp touch down flag\n");
    ts->view_area_touched = 0;  //realse all touch point,must clear this flag
}

static bool edge_point_process(struct touchpanel_data *ts, struct point_info points)
{
    if (ts->limit_enable) {
        if (points.x > ts->edge_limit.left_x2 && points.x < ts->edge_limit.right_x2) {
            if (ts->edge_limit.in_which_area == AREA_EDGE)
                tp_touch_release(ts);
            ts->edge_limit.in_which_area = AREA_NORMAL;
        } else if ((points.x > ts->edge_limit.left_x1 && points.x < ts->edge_limit.left_x2) || (points.x >ts->edge_limit.right_x2 && points.x < ts->edge_limit.right_x1)) {//area2
            if (ts->edge_limit.in_which_area == AREA_EDGE) {
                ts->edge_limit.in_which_area = AREA_CRITICAL;
            }
        } else if (points.x < ts->edge_limit.left_x1 || points.x > ts->edge_limit.right_x1) {        //area 1
            if (ts->edge_limit.in_which_area == AREA_CRITICAL) {
                ts->edge_limit.in_which_area = AREA_EDGE;
                return true;
            }
            if (ts->edge_limit.in_which_area ==  AREA_NORMAL)
                return true;

            ts->edge_limit.in_which_area = AREA_EDGE;
        }
    }

    return false;
}

static void tp_touch_handle(struct touchpanel_data *ts)
{
    int i = 0;
    uint8_t finger_num = 0;
    int obj_attention = 0;
    struct point_info points[ts->max_num];
    static int touch_report_num = 0;

    if (!ts->ts_ops->get_touch_points) {
        TPD_INFO("not support ts->ts_ops->get_touch_points callback\n");
        return;
    }

    memset(points, 0, sizeof(points));

    obj_attention = ts->ts_ops->get_touch_points(ts->chip_data, points, ts->max_num);
    if ((obj_attention & TOUCH_BIT_CHECK) != 0) {
        for (i = 0; i < ts->max_num; i++) {
            if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01 && (points[i].status == 0)) // buf[0] == 0 is wrong point, no process
                continue;
            if (((obj_attention & TOUCH_BIT_CHECK) >> i) & 0x01 && (points[i].status != 0)) {
                //Edge process before report abs
                if (ts->edge_limit_support) {
                    if (edge_point_process(ts, points[i]))
                        continue;
                }
#ifdef TYPE_B_PROTOCOL
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#endif
                touch_report_num++;
                tp_touch_down(ts, points[i], touch_report_num);
                finger_num++;
            }
#ifdef TYPE_B_PROTOCOL
            else {
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
            }
#endif
        }
    } else {
        touch_report_num = 0;
#ifdef TYPE_B_PROTOCOL
        for (i = 0; i < ts->max_num; i++) {
            input_mt_slot(ts->input_dev, i);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
        }
#endif
        tp_touch_up(ts);
        if (ts->edge_limit_support)
            ts->edge_limit.in_which_area = AREA_NOTOUCH;
    }
    ts->view_area_touched = finger_num;
    TPD_DEBUG("%s:%d ts->view_area_touched=%d finger_num=%d\n",__func__,__LINE__,ts->view_area_touched,finger_num);
    input_sync(ts->input_dev);
}

static void tp_btnkey_release(struct touchpanel_data *ts) {
    if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
        input_report_key_oppo(ts->input_dev, KEY_MENU, 0);
    if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
        input_report_key_oppo(ts->input_dev, KEY_HOMEPAGE, 0);
    if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
        input_report_key_oppo(ts->input_dev, KEY_BACK, 0);
    input_sync(ts->input_dev);
}

static void tp_btnkey_handle(struct touchpanel_data *ts)
{
    u8 touch_state = 0;

    if (ts->vk_type != TYPE_AREA_SEPRATE) {
        TPD_DEBUG("TP vk_type not proper, checktouchpanel, button-type\n");

        return;
    }
    if (!ts->ts_ops->get_keycode) {
        TPD_INFO("not support ts->ts_ops->get_keycode callback\n");

        return;
    }
    touch_state = ts->ts_ops->get_keycode(ts->chip_data);
    TPD_INFO("%s touc_state is %d\n", __func__, touch_state);

    if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
        input_report_key_oppo(ts->input_dev, KEY_MENU, CHK_BIT(touch_state, BIT_MENU));
    if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
        input_report_key_oppo(ts->input_dev, KEY_HOMEPAGE, CHK_BIT(touch_state, BIT_HOME));
    if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
        input_report_key_oppo(ts->input_dev, KEY_BACK, CHK_BIT(touch_state, BIT_BACK));
    input_sync(ts->input_dev);
}

static void tp_config_handle(struct touchpanel_data *ts)
{
    int ret = 0;

    ret = ts->ts_ops->fw_handle(ts->chip_data);
}

static void tp_work_func(struct touchpanel_data *ts)
{
    u8 cur_event = 0;

    if (!ts->ts_ops->trigger_reason) {
        TPD_INFO("not support ts_ops->trigger_reason callback\n");
        return;
    }
    /*
     *  trigger_reason:this callback determine which trigger reason should be
     *  The value returned has some policy!
     *  1.IRQ_EXCEPTION /IRQ_GESTURE /IRQ_IGNORE /IRQ_FW_CONFIG --->should be only reported  individually
     *  2.IRQ_TOUCH && IRQ_BTN_KEY --->should depends on real situation && set correspond bit on trigger_reason
     */
    cur_event = ts->ts_ops->trigger_reason(ts->chip_data, ts->gesture_enable, ts->is_suspended);
    if (CHK_BIT(cur_event, IRQ_TOUCH) || CHK_BIT(cur_event, IRQ_BTN_KEY)) {
        if (CHK_BIT(cur_event, IRQ_TOUCH)) {
            tp_touch_handle(ts);
        }
        if (CHK_BIT(cur_event, IRQ_BTN_KEY)) {
            tp_btnkey_handle(ts);
        }
    } else if (CHK_BIT(cur_event, IRQ_GESTURE)) {
        tp_gesture_handle(ts);
    } else if (CHK_BIT(cur_event, IRQ_EXCEPTION)) {
        tp_exception_handle(ts);
    } else if (CHK_BIT(cur_event, IRQ_FW_CONFIG)) {
        tp_config_handle(ts);
    } else {
        TPD_DEBUG("unknown irq trigger reason\n");
    }
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart touchpanel_timer_func(struct hrtimer *timer)
{
    struct touchpanel_data *ts = container_of(timer, struct touchpanel_data, timer);

    mutex_lock(&ts->mutex);
    tp_work_func(ts);
    mutex_unlock(&ts->mutex);
    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}
#else
static irqreturn_t tp_irq_thread_fn(int irq, void *dev_id)
{
    struct touchpanel_data *ts = (struct touchpanel_data *)dev_id;

    mutex_lock(&ts->mutex);
    tp_work_func(ts);
    mutex_unlock(&ts->mutex);

    return IRQ_HANDLED;
}
#endif

/**
 * tp_gesture_enable_flag -   expose gesture control status for other module.
 * Return gesture_enable status.
 */
int tp_gesture_enable_flag(void)
{
    if (!g_tp)
        return 0;

    return g_tp->gesture_enable;
}


/*
 *    gesture_enable = 0 : disable gesture
 *    gesture_enable = 1 : enable gesture when ps is far away
 *    gesture_enable = 2 : disable gesture when ps is near
 */
static ssize_t proc_gesture_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
    char buf[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return count;
    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &value);
    if (value > 2)
        return count;

    mutex_lock(&ts->mutex);
    if (ts->gesture_enable != value) {
        ts->gesture_enable = value;
        TPD_INFO("%s: gesture_enable = %d, is_suspended = %d\n", __func__, ts->gesture_enable, ts->is_suspended);
        if (ts->is_suspended)
            operate_mode_switch(ts);
    }else {
        TPD_INFO("%s: do not do same operator :%d\n", __func__, value);
    }
    mutex_unlock(&ts->mutex);

    return count;
}

static ssize_t proc_gesture_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[4] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("double tap enable is: %d\n", ts->gesture_enable);
    ret = sprintf(page, "%d\n", ts->gesture_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_coordinate_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_DEBUG("%s:gesture_type = %d\n", __func__, ts->gesture.gesture_type);
    ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", ts->gesture.gesture_type,
            ts->gesture.Point_start.x, ts->gesture.Point_start.y, ts->gesture.Point_end.x, ts->gesture.Point_end.y,
            ts->gesture.Point_1st.x,   ts->gesture.Point_1st.y,   ts->gesture.Point_2nd.x, ts->gesture.Point_2nd.y,
            ts->gesture.Point_3rd.x,   ts->gesture.Point_3rd.y,   ts->gesture.Point_4th.x, ts->gesture.Point_4th.y,
            ts->gesture.clockwise);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_gesture_control_fops = {
    .write = proc_gesture_control_write,
    .read  = proc_gesture_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations proc_coordinate_fops = {
    .read  = proc_coordinate_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_glove_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0 ;
    char buf[3] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;
    if (count > 2)
        return count;
    if (!ts->ts_ops->mode_switch) {
        TPD_INFO("not support ts_ops->mode_switch callback\n");
        return count;
    }
    if (copy_from_user(buf, buffer, count)) {
        TPD_INFO("%s: read proc input error.\n", __func__);
        return count;
    }
    sscanf(buf, "%d", &ret);
    TPD_INFO("%s:buf = %d, ret = %d\n", __func__, *buf, ret);
    if ((ret == 0) || (ret == 1)) {
        mutex_lock(&ts->mutex);
        ts->glove_enable = ret;
        ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_GLOVE, ts->glove_enable);
        if (ret < 0) {
            TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
        }
        mutex_unlock(&ts->mutex);
    }
    switch(ret) {
        case 0:
            TPD_DEBUG("tp_glove_func will be disable\n");
            break;
        case 1:
            TPD_DEBUG("tp_glove_func will be enable\n");
            break;
        default:
            TPD_DEBUG("Please enter 0 or 1 to open or close the glove function\n");
    }

    return count;
}

static ssize_t proc_glove_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[PAGESIZE] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_INFO("glove mode enable is: %d\n", ts->glove_enable);
    ret = sprintf(page, "%d\n", ts->glove_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_glove_control_fops = {
    .write = proc_glove_control_write,
    .read =  proc_glove_control_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct button_map *button_map;
    if (!g_tp)
        return sprintf(buf, "not support");

    button_map = &g_tp->button_map;
    return sprintf(buf,
            __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
            "\n", button_map->coord_menu.x, button_map->coord_menu.y, button_map->width_x, button_map->height_y, \
            button_map->coord_home.x, button_map->coord_home.y, button_map->width_x, button_map->height_y, \
            button_map->coord_back.x, button_map->coord_back.y, button_map->width_x, button_map->height_y);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static ssize_t proc_debug_control_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[PAGESIZE];

    TPD_INFO("%s: tp_debug = %d.\n", __func__, tp_debug);
    sprintf(page, "%d", tp_debug);
    ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_debug_control_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
    int tmp = 0;
    if (1 == sscanf(buf, "%d", &tmp)) {
        tp_debug = tmp;
    } else {
        TPD_DEBUG("invalid content: '%s', length = %zd\n", buf, count);
    }

    return count;
}

static const struct file_operations proc_debug_control_ops =
{
    .write = proc_debug_control_write,
    .read  = proc_debug_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_limit_area_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;
    TPD_DEBUG("limit_area is: %d\n", ts->edge_limit.limit_area);
    ret = sprintf(page, "limit_area = %d left_x1 = %d right_x1 = %d left_x2 = %d right_x2 = %d\n",
            ts->edge_limit.limit_area, ts->edge_limit.left_x1, ts->edge_limit.right_x1, ts->edge_limit.left_x2, ts->edge_limit.right_x2);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_limit_area_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8];
    int temp;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    if (copy_from_user(buf, buffer, count)) {
        TPD_DEBUG("%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%d", &temp);
    if (temp < 0 || temp > 10)
        return count;

    ts->edge_limit.limit_area = temp;
    ts->edge_limit.left_x1    = (ts->edge_limit.limit_area*1000)/100;
    ts->edge_limit.right_x1   = ts->resolution_info.LCD_WIDTH - ts->edge_limit.left_x1;
    ts->edge_limit.left_x2    = 2 * ts->edge_limit.left_x1;
    ts->edge_limit.right_x2   = ts->resolution_info.LCD_WIDTH - (2 * ts->edge_limit.left_x1);

    TPD_INFO("limit_area = %d; left_x1 = %d; right_x1 = %d; left_x2 = %d; right_x2 = %d\n",
            ts->edge_limit.limit_area, ts->edge_limit.left_x1, ts->edge_limit.right_x1, ts->edge_limit.left_x2, ts->edge_limit.right_x2);

    return count;
}

static const struct file_operations proc_limit_area_ops =
{
    .read  = proc_limit_area_read,
    .write = proc_limit_area_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_limit_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    TPD_INFO("limit_enable is: %d\n", ts->limit_enable);
    ret = sprintf(page, "%d\n", ts->limit_enable);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static ssize_t proc_limit_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int ret;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (!ts)
        return count;

    if (count > 2)
        count = 2;
    if (copy_from_user(buf, buffer, count)) {
        TPD_DEBUG("%s: read proc input error.\n", __func__);
        return count;
    }

    mutex_lock(&ts->mutex);

    if ('0' == buf[0]) {
        ts->limit_enable = 0;
    }else if ('1' == buf[0]) {
        ts->limit_enable = 1;
    }

    TPD_INFO("%s: limit_enable = %d \n", __func__, ts->limit_enable);
    if (ts->is_suspended == 0) {
        ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_EDGE, ts->limit_enable);
        if (ret < 0) {
            TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
        }
    }
    mutex_unlock(&ts->mutex);

    return count;
}

static const struct file_operations proc_limit_control_ops =
{
    .read  = proc_limit_control_read,
    .write = proc_limit_control_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_fw_update_write(struct file *file, const char __user *page, size_t size, loff_t *lo)
{
    const struct firmware *fw = NULL;
    int ret;
    int val = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    int count_tmp = 0;

    if (!ts)
        return size;
    if (size> 2)
        return -EINVAL;

    if (!ts->ts_ops->fw_check || !ts->ts_ops->reset) {
        TPD_INFO("not support ts_ops->fw_check callback\n");
        return size;
    }

    sscanf(page, "%d", &val);

    if (!val)
        val = ts->force_update;

    // val = 1 : must update ; val = 0 : depend on the firmware id
    TPD_INFO("%s, val = %d\n", __func__, val);

    mutex_lock(&ts->mutex);
    disable_irq_nosync(ts->irq);
    ts->loading_fw = true;

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    if (ts->ts_ops->fw_update) {
        TPD_INFO("%s: fw_name = %s\n", __func__, ts->panel_data.fw_name);
        ret = request_firmware(&fw, ts->panel_data.fw_name, ts->dev);
        if (!ret) {
            do {
                count_tmp++;

                ret = ts->ts_ops->fw_update(ts->chip_data, fw, val);
                if (ret == FW_NO_NEED_UPDATE) {
                    break;
                }

                ret |= ts->ts_ops->reset(ts->chip_data);
                ret |= ts->ts_ops->get_chip_info(ts->chip_data);
                ret |= ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);

            } while((count_tmp < 2) && (ret != 0));
            release_firmware(fw);
        } else {
            TPD_INFO("%s: fw_name request failed %s\n", __func__, ts->panel_data.fw_name);
            goto EXIT;
        }
    }

    tp_touch_release(ts);
    tp_btnkey_release(ts);
    ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);

EXIT:
    ts->loading_fw = false;

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, true);
    }

    enable_irq(ts->irq);
    mutex_unlock(&ts->mutex);

    ts->force_update = 0;

    return size;
}

static const struct file_operations proc_fw_update_ops =
{
    .write = proc_fw_update_write,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_finger_protect_result_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    uint8_t ret = 0;
    char page[16] = {0};
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    if(!ts)
        return 0;

    TPD_INFO("%s report_finger_protect = %d\n", __func__, ts->spuri_fp_touch.fp_touch_st);
    ret = sprintf(page, "%d\n", ts->spuri_fp_touch.fp_touch_st);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}


static const struct file_operations proc_finger_protect_result= {
    .read  =  proc_finger_protect_result_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_finger_protect_trigger_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int op = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));

    if (count > 2)
        return -EINVAL;

    if(1 == sscanf(buffer, "%d", &op)) {
        if (op == 1) {
            TPD_INFO("-->%s\n", __func__);
            ts->spuri_fp_touch.fp_trigger= true;
            ts->spuri_fp_touch.fp_touch_st = FINGER_PROTECT_NOTREADY;
            wake_up_interruptible(&waiter);
            TPD_INFO("<--%s\n", __func__);
        }
    } else {
        TPD_INFO("invalid content: '%s', length = %zd\n", buffer, count);
        return -EINVAL;
    }

    return count;
}

static const struct file_operations proc_finger_protect_trigger= {
    .write = proc_finger_protect_trigger_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static int finger_protect_handler(void *data)
{
    struct touchpanel_data *ts = (struct touchpanel_data *)data;
    if (!ts) {
        TPD_INFO("ts is null should nerver get here!\n");
        return 0;
    };
    if (!ts->ts_ops->spurious_fp_check) {
        TPD_INFO("not support spurious_fp_check call back\n");
        return 0;
    }

    do {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, ts->spuri_fp_touch.fp_trigger && ts->i2c_ready);
        set_current_state(TASK_RUNNING);
        ts->spuri_fp_touch.fp_trigger = false;
        ts->spuri_fp_touch.fp_touch_st = FINGER_PROTECT_NOTREADY;

        mutex_lock(&ts->mutex);
        ts->spuri_fp_touch.fp_touch_st = ts->ts_ops->spurious_fp_check(ts->chip_data);
        mutex_unlock(&ts->mutex);
    } while (!kthread_should_stop());
    return 0;
}

/**
 * init_touchpanel_proc - Using for create proc interface
 * @ts: touchpanel_data struct using for common driver
 *
 * we need to set touchpanel_data struct as private_data to those file_inode
 * Returning zero(success) or negative errno(failed)
 */
static int init_touchpanel_proc(struct touchpanel_data *ts)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_tp = NULL;
    struct proc_dir_entry *prEntry_tmp = NULL;

    TPD_INFO("%s entry\n", __func__);

    //proc files-step1:/proc/devinfo/tp  (touchpanel device info)
    register_device_proc("tp", ts->panel_data.manufacture_info.version, ts->panel_data.manufacture_info.manufacture);

    //proc files-step2:/proc/touchpanel
    prEntry_tp = proc_mkdir("touchpanel", NULL);
    if (prEntry_tp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create TP proc entry\n", __func__);
    }

    //proc files-step2-1:/proc/touchpanel/tp_debug_log_level (log control interface)
    prEntry_tmp = proc_create("tp_debug_log_level", 0644, prEntry_tp, &proc_debug_control_ops);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc files-step2-2:/proc/touchpanel/oppo_tp_fw_update (FW update interface)
    prEntry_tmp = proc_create_data("tp_fw_update", 0644, prEntry_tp, &proc_fw_update_ops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    //proc files-step2-3:/proc/touchpanel/oppo_tp_fw_update (edge limit control interface)
    if (ts->edge_limit_support) {
        prEntry_tmp = proc_create_data("oppo_tp_limit_area", 0664, prEntry_tp, &proc_limit_area_ops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("oppo_tp_limit_enable", 0664, prEntry_tp, &proc_limit_control_ops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-4:/proc/touchpanel/double_tap_enable (black gesture related interface)
    if (ts->black_gesture_support) {
        prEntry_tmp = proc_create_data("double_tap_enable", 0666, prEntry_tp, &proc_gesture_control_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("coordinate", 0444, prEntry_tp, &proc_coordinate_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

    //proc files-step2-5:/proc/touchpanel/double_tap_enable (Glove mode related interface)
    if (ts->glove_mode_support) {
        prEntry_tmp = proc_create_data("glove_mode_enable", 0666, prEntry_tp, &proc_glove_control_fops, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }

   //proc files-step2-5:/proc/touchpanel/double_tap_enable (Glove mode related interface)
    if (ts->spurious_fp_support) {
        prEntry_tmp = proc_create_data("finger_protect_result", 0666, prEntry_tp, &proc_finger_protect_result, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
        prEntry_tmp = proc_create_data("finger_protect_trigger", 0666, prEntry_tp, &proc_finger_protect_trigger, ts);
        if (prEntry_tmp == NULL) {
            ret = -ENOMEM;
            TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
        }
    }
    ts->prEntry_tp = prEntry_tp;

    return ret;
}

/**
 * init_input_device - Using for register input device
 * @ts: touchpanel_data struct using for common driver
 *
 * we should using this function setting input report capbility && register input device
 * Returning zero(success) or negative errno(failed)
 */
static int init_input_device(struct touchpanel_data *ts)
{
    int ret = 0;
    struct kobject *vk_properties_kobj;

    TPD_INFO("%s is called\n", __func__);
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        TPD_INFO("Failed to allocate input device\n");
        return ret;
    }

    ts->input_dev->name = TPD_DEVICE;;
    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
    set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    if (ts->black_gesture_support) {
        set_bit(KEY_F4, ts->input_dev->keybit);
    }

    switch(ts->vk_type) {
        case TYPE_PROPERTIES :
            {
                TPD_DEBUG("Type 1: using board_properties\n");
                vk_properties_kobj = kobject_create_and_add("board_properties", NULL);
                if (vk_properties_kobj)
                    ret = sysfs_create_group(vk_properties_kobj, &properties_attr_group);
                if (!vk_properties_kobj || ret)
                    TPD_DEBUG("failed to create board_properties\n");
                break;
            }
        case TYPE_AREA_SEPRATE:
            {
                TPD_DEBUG("Type 2:using same IC (button zone &&  touch zone are seprate)\n");
                if (CHK_BIT(ts->vk_bitmap, BIT_MENU))
                    set_bit(KEY_MENU, ts->input_dev->keybit);
                if (CHK_BIT(ts->vk_bitmap, BIT_HOME))
                    set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
                if (CHK_BIT(ts->vk_bitmap, BIT_BACK))
                    set_bit(KEY_BACK, ts->input_dev->keybit);
                break;
            }
        default :
            break;
    }

#ifdef TYPE_B_PROTOCOL
    input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->resolution_info.max_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->resolution_info.max_y, 0, 0);
    input_set_drvdata(ts->input_dev, ts);

    if (input_register_device(ts->input_dev)) {
        TPD_INFO("%s: Failed to register input device\n", __func__);
        input_free_device(ts->input_dev);
        return -1;
    }

    return 0;
}

/**
 * init_parse_dts - parse dts, get resource defined in Dts
 * @dev: i2c_client->dev using to get device tree
 * @ts: touchpanel_data, using for common driver
 *
 * If there is any Resource needed by chip_data, we can add a call-back func in this function
 * Do not care the result : Returning void type
 */
static void init_parse_dts(struct device *dev, struct touchpanel_data *ts)
{
    int rc;
    struct device_node *np;
    int temp_array[8];
    int tx_rx_num[2];

    np = dev->of_node;

    ts->register_is_16bit   = of_property_read_bool(np, "register-is-16bit");
    ts->edge_limit_support  = of_property_read_bool(np, "edge_limit_support");
    ts->glove_mode_support  = of_property_read_bool(np, "glove_mode_support");
    ts->esd_handle_support  = of_property_read_bool(np, "esd_handle_support");
    ts->spurious_fp_support = of_property_read_bool(np, "spurious_fingerprint_support");
    ts->charger_pump_support  = of_property_read_bool(np, "charger_pump_support");
    ts->black_gesture_support = of_property_read_bool(np, "black_gesture_support");

    // irq gpio
    ts->hw_res.irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, &(ts->irq_flags));
    if (gpio_is_valid(ts->hw_res.irq_gpio)) {
        rc= gpio_request(ts->hw_res.irq_gpio, "tp_irq_gpio");
        if (rc) {
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.irq_gpio);
        } else {
        	TPD_INFO("irq-gpio not specified in dts\n");
    	}
     }

    // reset gpio
    ts->hw_res.reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (gpio_is_valid(ts->hw_res.reset_gpio)) {
        rc = gpio_request(ts->hw_res.reset_gpio, "reset-gpio");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.reset_gpio);
    } else {
        TPD_INFO("ts->reset-gpio not specified\n");
    }

    TPD_INFO("%s : irq_gpio = %d, irq_flags = 0x%x, reset_gpio = %d\n",
            __func__, ts->hw_res.irq_gpio, ts->irq_flags, ts->hw_res.reset_gpio);

    // tp type gpio
    ts->hw_res.id1_gpio = of_get_named_gpio(np, "id1-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id1_gpio)) {
        rc = gpio_request(ts->hw_res.id1_gpio, "TP_ID1");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id1_gpio);
    } else {
        TPD_INFO("id1_gpio not specified\n");
    }

    ts->hw_res.id2_gpio = of_get_named_gpio(np, "id2-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id2_gpio)) {
        rc = gpio_request(ts->hw_res.id2_gpio, "TP_ID2");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id2_gpio);
    } else {
        TPD_INFO("id2_gpio not specified\n");
    }

    ts->hw_res.id3_gpio = of_get_named_gpio(np, "id3-gpio", 0);
    if (gpio_is_valid(ts->hw_res.id3_gpio)) {
        rc = gpio_request(ts->hw_res.id3_gpio, "TP_ID3");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.id3_gpio);
    } else {
        TPD_INFO("id3_gpio not specified\n");
    }

    ts->hw_res.pinctrl= devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(ts->hw_res.pinctrl)) {
       TPD_INFO("Getting pinctrl handle failed");
    }

    ts->hw_res.pin_set_high = pinctrl_lookup_state(ts->hw_res.pinctrl, "pin_set_high");
    if (IS_ERR_OR_NULL(ts->hw_res.pin_set_high)) {
        TPD_INFO ("Failed to get the high state pinctrl handle");
    }

    ts->hw_res.pin_set_low = pinctrl_lookup_state(ts->hw_res.pinctrl, "pin_set_low");
    if (IS_ERR_OR_NULL(ts->hw_res.pin_set_low)) {
        TPD_INFO(" Failed to get the low state pinctrl handle");
    }

    ts->hw_res.pin_set_nopull = pinctrl_lookup_state(ts->hw_res.pinctrl, "pin_set_nopull");
    if (IS_ERR_OR_NULL(ts->hw_res.pin_set_nopull)) {
        TPD_INFO("Failed to get the input state pinctrl handle");
    }

    ts->hw_res.enable2v8_gpio = of_get_named_gpio(np, "enable2v8_gpio", 0);
    if (ts->hw_res.enable2v8_gpio < 0) {
        TPD_INFO("ts->hw_res.enable2v8_gpio not specified\n");
    } else {
        if (gpio_is_valid(ts->hw_res.enable2v8_gpio)) {
            rc = gpio_request(ts->hw_res.enable2v8_gpio, "vdd2v8-gpio");
            if (rc) {
                TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.enable2v8_gpio);
            }
        }
    }

    ts->hw_res.enable1v8_gpio = of_get_named_gpio(np, "enable1v8_gpio", 0);
    if (ts->hw_res.enable1v8_gpio < 0) {
        TPD_INFO("ts->hw_res.enable1v8_gpio not specified\n");
    } else {
        if (gpio_is_valid(ts->hw_res.enable1v8_gpio)) {
            rc = gpio_request(ts->hw_res.enable1v8_gpio, "vcc1v8-gpio");
            if (rc) {
                TPD_INFO("unable to request gpio [%d]\n", ts->hw_res.enable1v8_gpio);
            }
        }
    }

    // resolution info
    rc = of_property_read_u32(np, "touchpanel,max-num-support", &ts->max_num);
    if (rc) {
        TPD_INFO("ts->max_num not specified\n");
        ts->max_num = 10;
    }

    rc = of_property_read_u32_array(np, "touchpanel,tx-rx-num", tx_rx_num, 2);
    if (rc) {
        TPD_INFO("tx-rx-num not set\n");
        ts->hw_res.TX_NUM = 0;
        ts->hw_res.RX_NUM = 0;
    }else{
        ts->hw_res.TX_NUM = tx_rx_num[0];
        ts->hw_res.RX_NUM = tx_rx_num[1];
    }
    TPD_INFO("TX_NUM = %d, RX_NUM = %d \n", ts->hw_res.TX_NUM, ts->hw_res.RX_NUM);

    rc = of_property_read_u32_array(np, "touchpanel,display-coords", temp_array, 2);
    if (rc) {
        TPD_INFO("Lcd size not set\n");
        ts->resolution_info.LCD_WIDTH = 0;
        ts->resolution_info.LCD_HEIGHT = 0;
    }else{
        ts->resolution_info.LCD_WIDTH = temp_array[0];
        ts->resolution_info.LCD_HEIGHT = temp_array[1];
    }

    rc = of_property_read_u32_array(np, "touchpanel,panel-coords", temp_array, 2);
    if (rc) {
        ts->resolution_info.max_x = 0;
        ts->resolution_info.max_y = 0;
    }else{
        ts->resolution_info.max_x = temp_array[0];
        ts->resolution_info.max_y = temp_array[1];
    }
    TPD_INFO("LCD_WIDTH = %d, LCD_HEIGHT = %d, max_x = %d, max_y = %d\n",
            ts->resolution_info.LCD_WIDTH, ts->resolution_info.LCD_HEIGHT, ts->resolution_info.max_x, ts->resolution_info.max_y);

    // virturl key Related
    rc = of_property_read_u32_array(np, "touchpanel,button-type", temp_array, 2);
    if (rc < 0) {
        TPD_INFO("error:button-type should be setting in dts!");
    } else {
        ts->vk_type = temp_array[0];
        ts->vk_bitmap = temp_array[1] & 0xFF;
        if (ts->vk_type == TYPE_PROPERTIES) {
            rc = of_property_read_u32_array(np, "touchpanel,button-map", temp_array, 8);
            if (rc) {
                TPD_INFO("button-map not set\n");
            }else{
                ts->button_map.coord_menu.x = temp_array[0];
                ts->button_map.coord_menu.y = temp_array[1];
                ts->button_map.coord_home.x = temp_array[2];
                ts->button_map.coord_home.y = temp_array[3];
                ts->button_map.coord_back.x = temp_array[4];
                ts->button_map.coord_back.y = temp_array[5];
                ts->button_map.width_x = temp_array[6];
                ts->button_map.height_y = temp_array[7];
            }
        }
    }
    // We can Add callback fuction here if necessary £¨seprate some dts config for chip_data£©
}

int init_power_control(struct touchpanel_data *ts)
{
    int ret;

    // i2c 1.8v
    ts->hw_res.vcc_1v8 = regulator_get(ts->dev, "vcc_1v8");
    if (IS_ERR(ts->hw_res.vcc_1v8)) {
        TPD_INFO("Regulator get failed vcc_1v8, ret = %d\n", ret);
    } else {
        if (regulator_count_voltages(ts->hw_res.vcc_1v8) > 0) {
            ret = regulator_set_voltage(ts->hw_res.vcc_1v8, 1800000, 1800000);
            if (ret) {
                dev_err(ts->dev, "Regulator set_vtg failed vcc_i2c rc = %d\n", ret);
                goto regulator_vcc_1v8_put;
            }
        }
    }
    // vdd 2.8v
    ts->hw_res.vdd_2v8 = regulator_get(ts->dev, "vdd_2v8");
    if (IS_ERR(ts->hw_res.vdd_2v8)) {
        TPD_INFO("Regulator vdd2v8 get failed, ret = %d\n", ret);
    } else {
        if (regulator_count_voltages(ts->hw_res.vdd_2v8) > 0) {
            ret = regulator_set_voltage(ts->hw_res.vdd_2v8, 2850000, 2850000);
            if (ret) {
                dev_err(ts->dev, "Regulator set_vtg failed vdd rc = %d\n", ret);
                goto regulator_vdd_2v8_put;
            }
        }
    }

    return 0;

regulator_vdd_2v8_put:
    regulator_put(ts->hw_res.vdd_2v8);
regulator_vcc_1v8_put:
    if (IS_ERR(ts->hw_res.vcc_1v8))
        regulator_put(ts->hw_res.vcc_1v8);

    return ret;
}

int tp_powercontrol_1v8(struct hw_resource *hw_res, bool on)
{
    int ret = 0;

    if (on) {// 1v8 power on
        if (!IS_ERR(hw_res->vcc_1v8)) {
            TPD_INFO("Enable the Regulator1v8.\n");
            ret = regulator_enable(hw_res->vcc_1v8);
            if (ret) {
                TPD_INFO("Regulator vcc_i2c enable failed ret = %d\n", ret);
                return ret;
            }
        } else {
            if (hw_res->enable1v8_gpio > 0) {
                TPD_INFO("Enable the 1v8_gpio\n");
                ret = gpio_direction_output(hw_res->enable1v8_gpio, 1);
                if (ret) {
                    TPD_INFO("enable the enable1v8_gpio failed.\n");
                    return ret;
                }
            }
        }
    } else {// 1v8 power off
        if (!IS_ERR(hw_res->vcc_1v8)) {
            ret = regulator_disable(hw_res->vcc_1v8);
            if (ret) {
                TPD_INFO("Regulator vcc_i2c enable failed rc = %d\n", ret);
                return ret;
            }
        } else {
            if (hw_res->enable1v8_gpio > 0) {
                TPD_INFO("disable the 1v8_gpio\n");
                ret = gpio_direction_output(hw_res->enable1v8_gpio, 0);
                if (ret) {
                    TPD_INFO("disable the enable2v8_gpio failed.\n");
                    return ret;
                }
            }
        }
    }

    return 0;
}

int tp_powercontrol_2v8(struct hw_resource *hw_res, bool on)
{
    int ret = 0;

    if (on) {// 2v8 power on
        if (!IS_ERR(hw_res->vdd_2v8)) {
            TPD_INFO("Enable the Regulator2v8.\n");
            ret = regulator_enable(hw_res->vdd_2v8);
            if (ret) {
                TPD_INFO("Regulator vdd enable failed ret = %d\n", ret);
                return ret;
            }
        } else {
            if (hw_res->enable2v8_gpio > 0) {
                TPD_INFO("Enable the 2v8_gpio\n");
                ret = gpio_direction_output(hw_res->enable2v8_gpio, 1);
                if (ret) {
                    TPD_INFO("enable the enable2v8_gpio failed.\n");
                    return ret;
                }
            }
        }
    } else {// 2v8 power off
        if (!IS_ERR(hw_res->vdd_2v8)) {
            ret = regulator_disable(hw_res->vdd_2v8);
            if (ret) {
                TPD_INFO("Regulator vdd disable failed rc = %d\n", ret);
                return ret;
            }
        } else {
            if (hw_res->enable2v8_gpio > 0) {
                TPD_INFO("disable the 2v8_gpio\n");
                ret = gpio_direction_output(hw_res->enable2v8_gpio, 0);
                if (ret) {
                    TPD_INFO("disable the enable2v8_gpio failed.\n");
                    return ret;
                }
            }
        }
    }
    return ret;
}


static void esd_handle_func(struct work_struct *work)
{
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
            esd_info.esd_check_work.work);

    if (ts->loading_fw) {
        TPD_INFO("FW is updating, stop esd handle!\n");
        return;
    }

    mutex_lock(&ts->esd_info.esd_lock);
    if (!ts->esd_info.esd_running_flag) {
        TPD_INFO("Esd protector has stopped!\n");
        goto ESD_END;
    }

    if (ts->is_suspended == 1) {
        TPD_INFO("Touch panel has suspended!\n");
        goto ESD_END;
    }

    if (!ts->ts_ops->esd_handle) {
        TPD_INFO("not support ts_ops->esd_handle callback\n");
        goto ESD_END;
    }

    ts->ts_ops->esd_handle(ts->chip_data);

    if (ts->esd_info.esd_running_flag)
        queue_delayed_work(ts->esd_info.esd_workqueue, &ts->esd_info.esd_check_work, ts->esd_info.esd_work_time);
    else
        TPD_INFO("Esd protector suspended!");

ESD_END:
    mutex_unlock(&ts->esd_info.esd_lock);
    return;
}

/**
 * esd_handle_switch - open or close esd thread
 * @esd_info: touchpanel_data, using for common driver resource
 * @on: bool variable using for  indicating open or close esd check function.
 *     true:open;
 *     false:close;
 */
void esd_handle_switch(struct esd_information *esd_info, bool on)
{
    mutex_lock(&esd_info->esd_lock);

    if (on) {
        if (!esd_info->esd_running_flag) {
            esd_info->esd_running_flag = 1;

            TPD_INFO("Esd protector started!\n");
            queue_delayed_work(esd_info->esd_workqueue, &esd_info->esd_check_work, esd_info->esd_work_time);
        }
    } else {
        if (esd_info->esd_running_flag) {
            esd_info->esd_running_flag = 0;

            TPD_INFO("Esd protector stoped!\n");
            cancel_delayed_work(&esd_info->esd_check_work);
        }
    }

    mutex_unlock(&esd_info->esd_lock);
}

/**
 * register_common_touch_device - parse dts, get resource defined in Dts
 * @pdata: touchpanel_data, using for common driver
 *
 * entrance of common touch Driver
 * Returning zero(sucess) or negative errno(failed)
 */
int register_common_touch_device(struct touchpanel_data *pdata)
{
    struct touchpanel_data *ts = pdata;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);
    //step1 : dts parse
    init_parse_dts(ts->dev, ts);

    //step2 : IIC interfaces init
    init_touch_interfaces(ts->dev, ts->register_is_16bit);

    //step3 : mutex init
    mutex_init(&ts->mutex);

    //step4 : Power init && setting
    ret = init_power_control(ts);
    if (ret) {
        TPD_INFO("%s: tp power init failed.\n", __func__);
        return -1;
    }
    if (!ts->ts_ops->power_control) {
        ret = -EINVAL;
        TPD_INFO("tp power_control NULL!\n");
        goto power_control_failed;
    }
    ret = ts->ts_ops->power_control(ts->chip_data, true);
    if (ret) {
        TPD_INFO("%s: tp power init failed.\n", __func__);
        goto power_control_failed;
    }

    //step5 : I2C function check
    if (!i2c_check_functionality(ts->client->adapter, I2C_FUNC_I2C)) {
        TPD_INFO("%s: need I2C_FUNC_I2C\n", __func__);
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    //step6 : FTM process
    ts->boot_mode = get_boot_mode();
    if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN)) {
        ts->ts_ops->ftm_process(ts->chip_data);
        ret = -ENODEV;
        TPD_INFO("%s: not int normal mode, return.\n", __func__);
        goto power_control_failed;
    }

    //step7 : Alloc fw_name/devinfo memory space
    ts->panel_data.fw_name = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
    if (ts->panel_data.fw_name == NULL) {
        ret = -ENOMEM;
        TPD_INFO("panel_data.fw_name kzalloc error\n");
        goto err_check_functionality_failed;
    }

    ts->panel_data.manufacture_info.version = kzalloc(MAX_DEVICE_VERSION_LENGTH, GFP_KERNEL);
    if (ts->panel_data.manufacture_info.version == NULL) {
        ret = -ENOMEM;
        TPD_INFO("manufacture_info.version kzalloc error\n");
        goto manu_version_alloc_err;
    }

    ts->panel_data.manufacture_info.manufacture = kzalloc(MAX_DEVICE_MANU_LENGTH, GFP_KERNEL);
    if (ts->panel_data.manufacture_info.manufacture == NULL) {
        ret = -ENOMEM;
        TPD_INFO("panel_data.fw_name kzalloc error\n");
        goto manu_info_alloc_err;
    }

    //step8 : touchpanel vendor
    ts->ts_ops->get_vendor(ts->chip_data, &ts->panel_data);

    //step9:get chip info
    if (!ts->ts_ops->get_chip_info) {
        ret = -EINVAL;
        TPD_INFO("tp get_chip_info NULL!\n");
        goto err_check_functionality_failed;
    }
    ret = ts->ts_ops->get_chip_info(ts->chip_data);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("tp get_chip_info failed!\n");
        goto err_check_functionality_failed;
    }

    //step10 : touchpanel Fw check
    if (!ts->ts_ops->fw_check) {
        ret = -EINVAL;
        TPD_INFO("tp fw_check NULL!\n");
        goto err_check_functionality_failed;
    }
    ret = ts->ts_ops->fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);
    if (ret == FW_ABNORMAL) {
        ts->force_update = 1;
        TPD_INFO("This FW need to be updated!\n");
    } else {
        ts->force_update = 0;
    }

    //step11 : touch input dev init
    ret = init_input_device(ts);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("tp_input_init failed!\n");
        goto err_check_functionality_failed;
    }

    //step12 : enable touch ic irq output ability    
    if (!ts->ts_ops->mode_switch) {
        ret = -EINVAL;
        TPD_INFO("tp mode_switch NULL!\n");
        goto free_touch_panel_input;
    }
    ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_NORMAL, true);
    if (ret < 0) {
        ret = -EINVAL;
        TPD_INFO("%s:modem switch failed!\n", __func__);
        goto free_touch_panel_input;
    }    

    //step13 : irq request setting
#ifdef TPD_USE_EINT
    if (gpio_is_valid(ts->hw_res.irq_gpio)) {
        TPD_INFO("%s, irq_gpio is %d, ts->irq is %d\n", __func__, ts->hw_res.irq_gpio, ts->irq);

        ret = request_threaded_irq(ts->irq, NULL,
                tp_irq_thread_fn,
                ts->irq_flags | IRQF_ONESHOT,
                TPD_DEVICE, ts);
        if (ret < 0) {
            TPD_INFO("%s request_threaded_irq ret is %d\n", __func__, ret);
            goto free_touch_panel_input;
        }
    }
#else
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = touchpanel_timer_func;
    hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

    //step14 : suspend && resume fuction register
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    /***Mark: need to add early_suspend register***/
#else
#ifdef CONFIG_FB
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret) {
        TPD_INFO("Unable to register fb_notifier: %d\n", ret);
    }
#endif/*CONFIG_FB*/
#endif/*CONFIG_TOUCHPANEL_MTK_PLATFORM*/

    //step15 : workqueue create(speedup_resume)
    ts->speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
    if (!ts->speedup_resume_wq) {
        ret = -ENOMEM;
        goto threaded_irq_free;
    }
    INIT_WORK(&ts->speed_up_work, speedup_resume);

    //step 16 : short edge shield
    if (ts->edge_limit_support) {
        ts->limit_enable = 1;
        ts->edge_limit.limit_area = 1;
        ts->edge_limit.in_which_area = AREA_NOTOUCH;
        ts->edge_limit.left_x1  = (ts->edge_limit.limit_area * 1000)/100;
        ts->edge_limit.right_x1 = ts->resolution_info.LCD_WIDTH - ts->edge_limit.left_x1;
        ts->edge_limit.left_x2  = 2 * ts->edge_limit.left_x1;
        ts->edge_limit.right_x2 = ts->resolution_info.LCD_WIDTH - (2 * ts->edge_limit.left_x1);
    }

    //step 17:esd recover support
    if (ts->esd_handle_support) {
        ts->esd_info.esd_workqueue = create_singlethread_workqueue("esd_workthread");
        INIT_DELAYED_WORK(&ts->esd_info.esd_check_work, esd_handle_func);

        mutex_init(&ts->esd_info.esd_lock);

        ts->esd_info.esd_running_flag = 0;
        ts->esd_info.esd_work_time = 2 * HZ; // HZ: clock ticks in 1 second generated by system
        TPD_INFO("Clock ticks for an esd cycle: %d\n", ts->esd_info.esd_work_time);

        esd_handle_switch(&ts->esd_info, true);
    }

    //step 18:spurious_fingerprint support
    if (ts->spurious_fp_support) {
        ts->spuri_fp_touch.thread = kthread_run(finger_protect_handler, ts, "touchpanel_fp");
        if (IS_ERR(ts->spuri_fp_touch.thread)) {
            TPD_INFO("spurious fingerprint thread create failed\n");
        }
    }

    //step 19 : createproc proc files interface
    init_touchpanel_proc(ts);

    //step 20 : Other****
    ts->i2c_ready = true;
    ts->loading_fw = false;
    ts->is_suspended = 0;
    ts->gesture_enable = 0;
    ts->glove_enable = 0;
    ts->view_area_touched = 0;
    g_tp = ts;
    TPD_INFO("Touch panel probe : normal end\n");
    return 0;

    destroy_workqueue(ts->speedup_resume_wq);
    ts->speedup_resume_wq = NULL;

threaded_irq_free:
    free_irq(ts->irq, ts);

free_touch_panel_input:
    input_unregister_device(ts->input_dev);

manu_info_alloc_err:
    kfree(ts->panel_data.manufacture_info.version);

manu_version_alloc_err:
    kfree(ts->panel_data.fw_name);

err_check_functionality_failed:
    ts->ts_ops->power_control(ts->chip_data, false);

power_control_failed:

    if (IS_ERR(ts->hw_res.vdd_2v8))
        gpio_free(ts->hw_res.enable2v8_gpio);
    else
        regulator_put(ts->hw_res.vdd_2v8);

    if (IS_ERR(ts->hw_res.vcc_1v8))
        gpio_free(ts->hw_res.enable1v8_gpio);
    else
        regulator_put(ts->hw_res.vcc_1v8);

    return ret;
}

/**
 * touchpanel_ts_suspend - touchpanel suspend function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * suspend function bind to LCD on/off status
 * Returning zero(sucess) or negative errno(failed)
 */
static int tp_suspend(struct device *dev)
{
    int ret;
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: start.\n", __func__);

    //step1:detect whether we need to do suspend
    if (ts->input_dev == NULL) {
        TPD_INFO("input_dev  registration is not complete\n");
        return -1;
    }
    if (ts->loading_fw) {
        TPD_INFO("FW is updating while suspending");
        return -1;
    }

#ifndef TPD_USE_EINT
        hrtimer_cancel(&ts->timer);
#endif

    //step2:get mutex && start process suspend flow
    mutex_lock(&ts->mutex);
    if (!ts->is_suspended) {
        ts->is_suspended = 1;
    } else {
        TPD_INFO("%s: do not suspend twice.\n", __func__);
        goto EXIT;
    }

    //step3:Release key && touch event before suspend
    tp_btnkey_release(ts);
    tp_touch_release(ts);

    //step4:cancel esd test
    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    //step5:gesture mode status process
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            ts->ts_ops->mode_switch(ts->chip_data, MODE_GESTURE, true);
            goto EXIT;
        }
    }

    //step6:switch mode to sleep
    ret = ts->ts_ops->mode_switch(ts->chip_data, MODE_SLEEP, true);
    if (ret < 0) {
        TPD_INFO("%s, Touchpanel operate mode switch failed\n", __func__);
    }

EXIT:
    TPD_INFO("%s: end.\n", __func__);
    mutex_unlock(&ts->mutex);

    return 0;
}

/**
 * touchpanel_ts_suspend - touchpanel resume function
 * @dev: i2c_client->dev using to get touchpanel_data resource
 *
 * resume function bind to LCD on/off status, this fuction start thread to speedup screen on flow.
 * Do not care the result: Return void type
 */
static void tp_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s start.\n", __func__);

    if (!ts->is_suspended) {
        TPD_INFO("%s: do not resume twice.\n", __func__);
        return ;
    }
    if (ts->loading_fw)
        return ;
    if(ts->ts_ops->resume_prepare) {
        mutex_lock(&ts->mutex);
        ts->ts_ops->resume_prepare(ts->chip_data);
        mutex_unlock(&ts->mutex);
    }

    queue_work(ts->speedup_resume_wq, &ts->speed_up_work);
}

/**
 * speedup_resume - speedup resume thread process
 * @work: work struct using for this thread
 *
 * do actully resume function
 * Do not care the result: Return void type
 */
static void speedup_resume(struct work_struct *work)
{
    int ret = 0;
    struct touchpanel_data *ts = container_of(work, struct touchpanel_data,
            speed_up_work);

    TPD_INFO("%s is called\n", __func__);

    //step1:Free irq && get mutex for locking i2c acess flow
    free_irq(ts->irq, ts);  // free irq
    mutex_lock(&ts->mutex);

    //step2:before Resume clear All of touch/key event Reset some flag to default satus
    ts->is_suspended = 0;
    if (ts->edge_limit_support)
        ts->edge_limit.in_which_area = AREA_NOTOUCH;
    tp_btnkey_release(ts);
    tp_touch_release(ts);

    //step3:Reset IC && switch work mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, true);
    }

    //step4:Request irq again
#ifndef TPD_USE_EINT
    hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
    ret = request_threaded_irq(ts->irq, NULL,
            tp_irq_thread_fn,
            ts->irq_flags | IRQF_ONESHOT,
            TPD_DEVICE, ts);        // request irq again
    if (ret < 0) {
        TPD_INFO("%s request_threaded_irq failed, ret is %d\n", __func__, ret);
    }
#endif

    //step5:Unlock  && exit
    TPD_INFO("%s: end!\n", __func__);
    mutex_unlock(&ts->mutex);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    int *blank;
    struct fb_event *evdata = data;
    struct touchpanel_data *ts = container_of(self, struct touchpanel_data, fb_notif);

    //to aviod some kernel bug (at fbmem.c some local veriable are not initialized)
    if(event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK)
        return 0;

    if ((evdata) && (evdata->data) && (event == FB_EARLY_EVENT_BLANK)  && (ts) && (ts->chip_data)) {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK) {
            TPD_INFO("%s :TP resume\n", __func__);
            tp_resume(ts->dev);
        } else if (*blank == FB_BLANK_POWERDOWN) {
            TPD_INFO("%s :TP suspend\n", __func__);
            tp_suspend(ts->dev);
        }
    }
    return 0;
}
#endif

void tp_i2c_suspend(struct touchpanel_data *ts)
{
    ts->i2c_ready = false;
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*enable gpio wake system through interrupt*/
            enable_irq_wake(ts->irq);
        }
    }
    disable_irq_nosync(ts->irq);
}

void tp_i2c_resume(struct touchpanel_data *ts)
{
    if (ts->black_gesture_support) {
        if (ts->gesture_enable == 1) {
            /*disable gpio wake system through intterrupt*/
            disable_irq_wake(ts->irq);
        }
    }
    enable_irq(ts->irq);
    ts->i2c_ready = true;
    if (ts->spurious_fp_support && ts->spuri_fp_touch.fp_trigger) {
        wake_up_interruptible(&waiter);
    }
}

struct touchpanel_data *common_touch_data_alloc(void)
{
    if (g_tp) {
        TPD_INFO("%s:common panel struct has alloc already!\n", __func__);
        return NULL;
    }
    return kzalloc(sizeof(struct touchpanel_data), GFP_KERNEL);
}

int common_touch_data_free(struct touchpanel_data *pdata)
{
    if (pdata)
        kfree(pdata);
    return 0;
}

/**
 * input_report_key_oppo - Using for report virtual key
 * @work: work struct using for this thread
 *
 * before report virtual key, detect whether touch_area has been touched
 * Do not care the result: Return void type
 */
void input_report_key_oppo(struct input_dev *dev, unsigned int code, int value)
{
    if (value) {//report Key[down]
        if (g_tp) {
            if (g_tp->view_area_touched == 0) {
                input_report_key(dev, code, value);
            } else {
                TPD_INFO("sorry, tp is touch down, can not report touch key\n");
            }
        }
    } else {
        input_report_key(dev, code, value);
    }
}
