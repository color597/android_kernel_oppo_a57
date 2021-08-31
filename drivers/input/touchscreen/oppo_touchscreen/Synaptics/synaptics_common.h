/***************************************************
 * File:synaptics_common.h
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             synaptics common driver
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef SYNAPTICS_H
#define SYNAPTICS_H
#define CONFIG_SYNAPTIC_RED

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"

/*********PART2:Define Area**********************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

#define DiagonalUpperLimit  1100
#define DiagonalLowerLimit  900

/*********PART3:Struct Area**********************/
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


struct test_header {
    unsigned int magic1;
    unsigned int magic2;
    unsigned int withCBC;
    unsigned int array_limit_offset;
    unsigned int array_limit_size;
    unsigned int array_limitcbc_offset;
    unsigned int array_limitcbc_size;
};

struct syna_testdata{
    int TX_NUM;
    int RX_NUM;
    int fd;
    int irq_gpio;
    uint64_t  TP_FW;     
    const struct firmware *fw;
};

struct synaptics_proc_operations {
    void (*auto_test)    (struct seq_file *s, void *chip_data, struct syna_testdata *syna_testdata);
    void (*limit_read)   (struct seq_file *s, struct touchpanel_data *ts);
    void (*delta_read)   (struct seq_file *s, void *chip_data);
    void (*baseline_read)(struct seq_file *s, void *chip_data);
};

void synaptics_limit_read(struct seq_file *s, struct touchpanel_data *ts);
int  synaptics_create_proc(struct touchpanel_data *ts, struct synaptics_proc_operations *syna_ops);
void parse_header(struct image_header_data *header, const unsigned char *fw_image);

#endif
