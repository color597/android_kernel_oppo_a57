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
#ifndef _TOUCHPANEL_COMMON_H_
#define _TOUCHPANEL_COMMON_H_

/*********PART1:Head files**********************/
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <soc/oppo/boot_mode.h>
#include <soc/oppo/device_info.h>
#include <linux/delay.h>

#include "util_interface/touch_interfaces.h"
#include "tp_devices.h"

/*********PART2:Define Area**********************/
#define TPD_USE_EINT
#define TYPE_B_PROTOCOL

#define PAGESIZE 512
#define MAX_GESTURE_COORD 6

#define UnkownGesture       0
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

/* bit operation */
#define SET_BIT(data, flag) ((data) |= (flag))
#define CLR_BIT(data, flag) ((data) &= ~(flag))
#define CHK_BIT(data, flag) ((data) & (flag))
#define VK_TAB {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}

#define TOUCH_BIT_CHECK           0x3FF  //max support 10 point report.using for detect non-valid points
#define MAX_FW_NAME_LENGTH        60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    16

/*********PART3:Struct Area**********************/
typedef enum {
    TYPE_PROPERTIES = 1,   /*using board_properties*/
    TYPE_AREA_SEPRATE,     /*using same IC (button zone &&  touch zone are seprate)*/
    TYPE_DIFF_IC,          /*using diffrent IC (button zone &&  touch zone are seprate)*/
    TYPE_NO_NEED,          /*No need of virtual key process*/
}vk_type;

typedef enum {
    AREA_NOTOUCH,
    AREA_EDGE,
    AREA_CRITICAL,
    AREA_NORMAL,
}touch_area;

typedef enum {
    MODE_NORMAL,
    MODE_SLEEP,
    MODE_EDGE,
    MODE_GESTURE,
    MODE_GLOVE,
}work_mode;

typedef enum {
    FW_NORMAL,     /*fw might update, depend on the fw id*/
    FW_ABNORMAL,   /*fw abnormal, need update*/
}fw_check_state;

typedef enum {
    FW_UPDATE_SUCCESS,
    FW_NO_NEED_UPDATE,
    FW_UPDATE_ERROR,
}fw_update_state;

typedef enum IRQ_TRIGGER_REASON {
    IRQ_TOUCH       = 0x01,
    IRQ_GESTURE     = 0x02,
    IRQ_BTN_KEY     = 0x04,
    IRQ_EXCEPTION   = 0x08,
    IRQ_FW_CONFIG   = 0x10,
    IRQ_IGNORE      = 0x00,
}irq_reason;

typedef enum vk_bitmap{
    BIT_reserve    = 0x08,
    BIT_BACK       = 0x04,
    BIT_HOME       = 0x02,
    BIT_MENU       = 0x01,
}vk_bitmap;

typedef enum finger_protect_status {
    FINGER_PROTECT_TOUCH_UP,
    FINGER_PROTECT_TOUCH_DOWN,
    FINGER_PROTECT_NOTREADY,
}fp_touch_state;

struct Coordinate {
    int x;
    int y;
};

struct gesture_info {
    uint32_t gesture_type;
    uint32_t clockwise;
    struct Coordinate Point_start;
    struct Coordinate Point_end;
    struct Coordinate Point_1st;
    struct Coordinate Point_2nd;
    struct Coordinate Point_3rd;
    struct Coordinate Point_4th;
};

struct point_info {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t  width_major;
    uint8_t  status;
};

struct panel_info {
    char    *fw_name;                               /*FW name*/
    char    *test_limit_name;                       /*test limit name*/
    uint32_t TP_FW;                                 /*FW Version Read from IC*/
    tp_dev  tp_type;
    struct manufacture_info manufacture_info;       /*touchpanel device info*/
};

struct hw_resource {
    //gpio
    int id1_gpio;
    int id2_gpio;
    int id3_gpio;

    int irq_gpio;                                   /*irq GPIO num*/
    int reset_gpio;                                 /*Reset GPIO*/

    int enable2v8_gpio;                             /*vdd_2v8 enable GPIO*/
    int enable1v8_gpio;                             /*vcc_1v8 enable GPIO*/

    //TX&&RX Num
    int TX_NUM;
    int RX_NUM;

    //power
    struct regulator *vdd_2v8;                      /*power 2v8*/
    struct regulator *vcc_1v8;                      /*power 1v8*/

    //pinctrl
    struct pinctrl          *pinctrl;
    struct pinctrl_state    *pin_set_high;
    struct pinctrl_state    *pin_set_low;
    struct pinctrl_state    *pin_set_nopull;
};

struct edge_limit {
    int limit_area;
    int left_x1;
    int right_x1;
    int left_x2;
    int right_x2;
    touch_area in_which_area;
};

struct button_map {
    int width_x;                                        /*width of each key area */
    int height_y;                                       /*height of each key area*/
    struct Coordinate coord_menu;                       /*Menu centre coordinates*/
    struct Coordinate coord_home;                       /*Home centre coordinates*/
    struct Coordinate coord_back;                       /*Back centre coordinates*/
};

struct resolution_info {
    uint32_t max_x;                                     /*touchpanel width */
    uint32_t max_y;                                     /*touchpanel height*/
    uint32_t LCD_WIDTH;                                 /*LCD WIDTH        */
    uint32_t LCD_HEIGHT;                                /*LCD HEIGHT       */
};

struct esd_information {
    bool esd_running_flag;
    int  esd_work_time;
    struct mutex             esd_lock;
    struct workqueue_struct *esd_workqueue;
    struct delayed_work      esd_check_work;
};

struct spurious_fp_touch {
    bool fp_trigger;                                    /*thread only turn into runnning state by fingerprint kick proc/touchpanel/finger_protect_trigger*/
    fp_touch_state fp_touch_st;                         /*provide result to fingerprint of touch status data*/
    struct task_struct *thread;                         /*tread use for fingerprint susprious touch check*/
};

struct touchpanel_data {
    bool register_is_16bit;                             /*register is 16bit*/
    bool glove_mode_support;                            /*glove_mode support feature*/
    bool black_gesture_support;                         /*black_gesture support feature*/
    bool charger_pump_support;                          /*charger_pump support feature*/
    bool edge_limit_support;                            /*edge_limit support feature*/
    bool esd_handle_support;                            /*esd handle support feature*/
    bool spurious_fp_support;                           /*avoid fingerprint spurious trrigger feature*/

    bool i2c_ready;                                     /*i2c resume status*/
    bool loading_fw;                                    /*touchpanel FW updating*/
	
    u8   vk_bitmap ;                                     /*every bit declear one state of key "reserve(keycode)|home(keycode)|menu(keycode)|back(keycode)"*/
    vk_type  vk_type;                                   /*virtual_key type*/
	
    uint32_t irq_flags;                                 /*irq setting flag*/
    int irq;                                            /*irq num*/

    int gesture_enable;                                 /*control state of black gesture*/
    int glove_enable;                                   /*control state of glove gesture*/
    int limit_enable;                                   /*control state of edge limit*/
    int is_suspended;                                   /*suspend/resume flow exec flag*/

    int boot_mode;                                      /*boot up mode */
    int view_area_touched;                              /*view area touched flag*/
    int force_update;                                   /*force update flag*/
    int max_num;                                        /*max muti-touch num supportted*/

#if defined(TPD_USE_EINT)
    struct hrtimer         timer;                       /*using polling instead of IRQ*/
#endif
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;                     /*register to control suspend/resume*/
#endif

    struct mutex           mutex;                       /*mutex for lock i2c related flow*/
    struct panel_info      panel_data;                  /*GPIO control(id && pinctrl && tp_type)*/
    struct hw_resource     hw_res;                      /*hw resourc information*/
    struct edge_limit      edge_limit;                  /*edge limit*/
    struct button_map      button_map;                  /*virtual_key button area*/
    struct resolution_info resolution_info;             /*resolution of touchpanel && LCD*/
    struct gesture_info    gesture;                     /*gesture related info*/

    struct work_struct     speed_up_work;               /*using for speedup resume*/
    struct workqueue_struct *speedup_resume_wq;         /*using for touchpanel speedup resume wq*/

    struct esd_information  esd_info;
    struct spurious_fp_touch spuri_fp_touch;           /*spurious_finger_support*/

    struct device         *dev;                         /*used for i2c->dev*/
    struct i2c_client     *client;
    struct input_dev      *input_dev;
    struct input_dev      *kpd_input_dev;

    struct oppo_touchpanel_operations *ts_ops;          /*call_back function*/
    struct proc_dir_entry *prEntry_tp;                  /*struct proc_dir_entry of "/proc/touchpanel"*/

    void                  *chip_data;                   /*Chip Related data*/
    void                  *private_data;                /*Reserved Private data*/
};

struct oppo_touchpanel_operations {
    int  (*get_chip_info)        (void *chip_data);                                           /*return 0:success;other:failed*/
    int  (*mode_switch)          (void *chip_data, work_mode mode, bool flag);                /*return 0:success;other:failed*/
    int  (*get_touch_points)     (void *chip_data, struct point_info *points, int max_num);   /*return point bit-map*/
    int  (*get_gesture_info)     (void *chip_data, struct gesture_info * gesture);            /*return 0:success;other:failed*/
    int  (*ftm_process)          (void *chip_data);                                           /*ftm boot mode process*/
    int  (*get_vendor)           (void *chip_data, struct panel_info  *panel_data);           /*distingush which panel we use, (TRULY/OFLIM/BIEL/TPK)*/
    int  (*reset)                (void *chip_data);                                           /*Reset Touchpanel*/
    fw_check_state  (*fw_check)  (void *chip_data, struct resolution_info *resolution_info,
                                                  struct panel_info *panel_data);             /*return < 0 :failed; 0 sucess*/
    fw_update_state (*fw_update) (void *chip_data, const struct firmware *fw, bool force);    /*return 0 normal; return -1:update failed;*/
    int  (*power_control)        (void *chip_data, bool enable);                              /*return 0:success;other:abnormal, need to jump out*/
    u8   (*trigger_reason)       (void *chip_data, int gesture_enable, int is_suspended);     /*clear innterrupt reg && detect irq trigger reason*/
    u8   (*get_keycode)          (void *chip_data);                                           /*get touch-key code*/
    void (*esd_handle)           (void *chip_data);
    int  (*fw_handle)            (void *chip_data);                                           /*return 0 normal; return -1:update failed;*/
    void (*resume_prepare)       (void *chip_data);                                           /*using for operation before resume flow, 
                                                                                                eg:incell 3320 need to disable gesture to release inter pins for lcd resume*/
    fp_touch_state (*spurious_fp_check) (void *chip_data);                                    /*spurious fingerprint check*/
};

/*********PART3:function or variables for other files**********************/
extern unsigned int tp_debug ;                                                            /*using for print debug log*/

struct touchpanel_data *common_touch_data_alloc(void);

int  common_touch_data_free(struct touchpanel_data *pdata);
int  register_common_touch_device(struct touchpanel_data *pdata);

void tp_i2c_suspend(struct touchpanel_data *ts);
void tp_i2c_resume (struct touchpanel_data *ts);

int  tp_powercontrol_1v8(struct hw_resource *hw_res, bool on);
int  tp_powercontrol_2v8(struct hw_resource *hw_res, bool on);

void operate_mode_switch  (struct touchpanel_data *ts);
void input_report_key_oppo(struct input_dev *dev, unsigned int code, int value);
void esd_handle_switch(struct esd_information *esd_info, bool on);
void tp_touch_btnkey_release(void);

#endif

