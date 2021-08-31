/***************************************************
 * File:synaptics_common.c
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

#include "../touchpanel_common.h"
#include "synaptics_common.h"

/*******Part0:LOG TAG Declear********************/

#define TPD_DEVICE "synaptics_common"
#define TPD_INFO(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)    
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err(TPD_DEVICE ": " a, ##arg);\
    }while(0) 

/*******Part1:Call Back Function implement*******/

unsigned int extract_uint_le(const unsigned char *ptr)
{
    return (unsigned int)ptr[0] + 
        (unsigned int)ptr[1] * 0x100 + 
        (unsigned int)ptr[2] * 0x10000 + 
        (unsigned int)ptr[3] * 0x1000000;
}

void parse_header(struct image_header_data *header, const unsigned char *fw_image)
{
    struct image_header *data = (struct image_header *)fw_image;

    header->checksum = extract_uint_le(data->checksum);
    TPD_DEBUG(" checksume is %x", header->checksum);

    header->bootloader_version = data->bootloader_version;
    TPD_DEBUG(" bootloader_version is %d\n", header->bootloader_version);

    header->firmware_size = extract_uint_le(data->firmware_size);
    TPD_DEBUG(" firmware_size is %x\n", header->firmware_size);

    header->config_size = extract_uint_le(data->config_size);
    TPD_DEBUG(" header->config_size is %x\n", header->config_size);

    memcpy(header->product_id, data->product_id, sizeof(data->product_id));
    header->product_id[sizeof(data->product_id)] = 0;

    memcpy(header->product_info, data->product_info, sizeof(data->product_info));

    header->contains_firmware_id = data->options_firmware_id;
    TPD_DEBUG(" header->contains_firmware_id is %x\n", header->contains_firmware_id);   
    if (header->contains_firmware_id)
        header->firmware_id = extract_uint_le(data->firmware_id);

    return;
}

void synaptics_limit_read(struct seq_file *s, struct touchpanel_data *ts)
{
    int ret = 0;
    uint16_t *prow = NULL;
    uint16_t *prowcbc = NULL;
    const struct firmware *fw = NULL;
    struct test_header *ph = NULL;
    int i = 0;
    int temp = 0;

    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "Request failed, Check the path %d", temp);
        return;
    }

    ph = (struct test_header *)(fw->data);
    prow = (uint16_t *)(fw->data + ph->array_limit_offset);
    prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);
    TPD_DEBUG("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x\n", 
            ph->array_limit_offset, ph->array_limitcbc_offset);
    TPD_DEBUG("test begin:\n");
    seq_printf(s, "Without cbc:");

    for (i = 0 ; i < (ph->array_limit_size / 2); i++) {
        if (i % (2 * ts->hw_res.RX_NUM) == 0)
            seq_printf(s, "\n[%2d] ", (i / ts->hw_res.RX_NUM) / 2);
        seq_printf(s, "%4d, ", prow[i]);
        TPD_DEBUG("%d, ", prow[i]);
    }
    if (ph->withCBC == 1) {
        seq_printf(s, "\nWith cbc:");
        for (i = 0 ; i < (ph->array_limitcbc_size / 2); i++) {
            if (i % (2 * ts->hw_res.RX_NUM) == 0)
                seq_printf(s, "\n[%2d] ", (i / ts->hw_res.RX_NUM) / 2);
            seq_printf(s, "%4d, ", prowcbc[i]);
            TPD_DEBUG("%d, ", prowcbc[i]);
        }
    }

    seq_printf(s, "\n");
    release_firmware(fw);
}

//proc/touchpanel/synaptics/delta
static int tp_delta_debug_read_func(struct seq_file *s, void *v)
{    
    struct touchpanel_data *ts = s->private;
    struct synaptics_proc_operations *syna_ops;

    if (!ts)
        return 0; 
    syna_ops = (struct synaptics_proc_operations *)ts->private_data;

    if (!syna_ops)
        return 0;
    if (!syna_ops->delta_read) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }
    disable_irq_nosync(ts->client->irq);
    mutex_lock(&ts->mutex);
    syna_ops->delta_read(s, ts->chip_data);
    mutex_unlock(&ts->mutex);
    enable_irq(ts->client->irq);

    return 0;
}

static int data_delta_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_delta_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_delta_data_proc_fops = {    
    .owner = THIS_MODULE, 
    .open  = data_delta_open, 
    .read  = seq_read, 
    .release = single_release, 
};

//proc/touchpanel/synaptics/baseline
static int tp_baseline_debug_read_func(struct seq_file *s, void *v)
{    
    struct touchpanel_data *ts = s->private;
    struct synaptics_proc_operations *syna_ops;

    if (!ts)
        return 0; 
    syna_ops = (struct synaptics_proc_operations *)(ts->private_data);
    if (!syna_ops) {
        return 0;
    }
    if (!syna_ops->baseline_read) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }
    disable_irq_nosync(ts->client->irq);
    mutex_lock(&ts->mutex);
    syna_ops->baseline_read(s, ts->chip_data);

    //step6: return to normal mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    mutex_unlock(&ts->mutex);
    enable_irq(ts->client->irq);

    return 0;
}

static int data_baseline_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_baseline_debug_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_baseline_data_proc_fops = {    
    .owner = THIS_MODULE, 
    .open = data_baseline_open, 
    .read = seq_read, 
    .release = single_release, 
};

//proc/touchpanel/synaptics/data_limit
static int tp_limit_data_read_func(struct seq_file *s, void *v)
{    
    struct touchpanel_data *ts = s->private;
    struct synaptics_proc_operations *syna_ops;

    if (!ts)
        return 0; 
    syna_ops = (struct synaptics_proc_operations *)ts->private_data;
    if (!syna_ops)
        return 0;
    if (!syna_ops->limit_read) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }
    syna_ops->limit_read(s, ts);

    return 0;
}

static int limit_data_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_limit_data_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_limit_data_proc_fops = {    
    .owner = THIS_MODULE, 
    .open  = limit_data_open, 
    .read  = seq_read, 
    .release = single_release, 
};

//proc/touchpanel/baseline_test
static int tp_auto_test_read_func(struct seq_file *s, void *v)
{    
    struct touchpanel_data *ts = s->private;
    struct synaptics_proc_operations *syna_ops;
    struct timespec now_time;
    struct rtc_time rtc_now_time;
    const struct firmware *fw = NULL;
    mm_segment_t old_fs;
    uint8_t data_buf[64];
    int ret = 0;
    int fd = -1;

    struct syna_testdata syna_testdata = 
    {
        .TX_NUM = 0, 
        .RX_NUM = 0, 
        .fd = -1, 
        .irq_gpio = -1, 
        .TP_FW = 0, 
        .fw = NULL, 
    };

    if (!ts)
        return 0; 
    syna_ops = (struct synaptics_proc_operations *)ts->private_data;
    if (!syna_ops)
        return 0;
    if (!syna_ops->auto_test) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }

    //step1:disable_irq && get mutex locked
    disable_irq_nosync(ts->client->irq);
    mutex_lock(&ts->mutex);

    //step2: create a file to store test data in /sdcard/Tp_Test
    getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv", 
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday, 
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {
        TPD_INFO("Open log file '%s' failed.\n", data_buf);
        set_fs(old_fs);
        mutex_unlock(&ts->mutex);
        enable_irq(ts->client->irq);

        return 0;    
    }

    //step3:request test limit data from userspace
    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "No limit IMG\n");
        mutex_unlock(&ts->mutex);
        enable_irq(ts->client->irq);
        return 0;    
    }

    //step4:init syna_testdata
    syna_testdata.fd = fd;
    syna_testdata.TX_NUM = ts->hw_res.TX_NUM;
    syna_testdata.RX_NUM = ts->hw_res.RX_NUM;
    syna_testdata.irq_gpio = ts->hw_res.irq_gpio;
    syna_testdata.TP_FW = ts->panel_data.TP_FW;
    syna_testdata.fw = fw;

    syna_ops->auto_test(s, ts->chip_data, &syna_testdata);

    //step5: close file && release test limit firmware
    if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }    
    release_firmware(fw);

    //step6: return to normal mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    //step7: unlock the mutex && enable irq trigger
    mutex_unlock(&ts->mutex);
    enable_irq(ts->client->irq);

    return 0;
}

static int baseline_autotest_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_auto_test_proc_fops = {    
    .owner = THIS_MODULE, 
    .open  = baseline_autotest_open, 
    .read  = seq_read, 
    .release = single_release, 
};

int synaptics_create_proc(struct touchpanel_data *ts, struct synaptics_proc_operations *syna_ops)
{    
    int ret = 0;

    // touchpanel_auto_test interface
    struct proc_dir_entry *prEntry_synaptics = NULL; 
    struct proc_dir_entry *prEntry_tmp = NULL; 
    ts->private_data = syna_ops;
    prEntry_tmp = proc_create_data("baseline_test", 0666, ts->prEntry_tp, &tp_auto_test_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
    prEntry_synaptics = proc_mkdir("synaptics", ts->prEntry_tp);
    if (prEntry_synaptics == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create TP proc entry\n", __func__);
    }
    // show limit data interface
    prEntry_tmp = proc_create_data("data_limit", 0666, prEntry_synaptics, &tp_limit_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    // show baseline data interface
    prEntry_tmp = proc_create_data("baseline", 0666, prEntry_synaptics, &tp_baseline_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
    // show delta interface
    prEntry_tmp = proc_create_data("delta", 0666, prEntry_synaptics, &tp_delta_data_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }    

    return ret;
}
