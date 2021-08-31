/************************************************************************************
** File: - MSM8953_LA_1_0\android\kernel\include\soc\oppo\oppo_fp_common.h
** VENDOR_EDIT
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd
**
** Description:
**      fingerprint compatibility configuration 
**
** Version: 1.0
** Date created: 18:03:11,23/07/2016
** Author: Ziqing.guo@Prd.BaseDrv
**
** --------------------------- Revision History: --------------------------------
** 	<author>	     <data>			<desc>
**  Ziqing.guo   2016/07/23       create the file
************************************************************************************/

#ifndef _OPPO_FP_COMMON_H_
#define _OPPO_FP_COMMON_H_

#include <linux/platform_device.h>

#define FP_ID_MAX_LENGTH 10 //the length of /proc/fp_id should less than FP_ID_MAX_LENGTH !!!

typedef enum {
    FP_GOODIX,
    FP_FPC ,
    FP_UNKNOWN,
} fp_vendor_t;

enum {
    FP_OK,
    FP_ERROR_GPIO,
    FP_ERROR_GENERAL,
} ;

struct fp_data { 
	struct device *dev;
	int gpio_id0;
	int gpio_id1;
	int gpio_id2;
	int gpio_id3;
	int fp_id1;
	int fp_id2;
	int fp_id3;
	fp_vendor_t fpsensor_type; 
};

fp_vendor_t get_fpsensor_type(void);
#endif

