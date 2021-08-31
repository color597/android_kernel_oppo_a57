/************************************************************************************
 ** File: - MSM8953_LA_1_0\android\kernel\drivers\soc\oppo\oppo_fp_common\oppo_fp_common.c
 ** VENDOR_EDIT
 ** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd
 **
 ** Description:
 **      fp_common compatibility configuration
 **
 ** Version: 1.0
 ** Date created: 18:03:11,23/07/2016
 ** Author: Ziqing.guo@Prd.BaseDrv
 **
 ** --------------------------- Revision History: --------------------------------
 **  <author>         <data>         <desc>
 **  Ziqing.guo       2016/07/23     create the file
 **  Haitao.zhou      2016/08/16     fix file format
 **  Ziqing.guo       2016/09/19     add support for project 16027
 **  Ziqing.guo       2016/11/04     add Turly for project 16017
 **  Ziqing.guo       2017/02/24     add Turly for project 16027
 ************************************************************************************/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/oppo_fp_common.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>


#define CHIP_GOODIX            "goodix"
#define CHIP_OFILM_1140        "a_ofilm"
#define CHIP_CT_1140           "a_CT"
#define CHIP_PRIMAX_1140       "a_primax"
#define CHIP_QTECH_1140        "a_Qtech"
#define CHIP_Turly_1140        "a_Turly"
#define CHIP_FINGERCHIP_1260   "b_fgchip"
#define CHIP_UNKNOWN    "unknown"

#define CHIP_OFILM_1140        "a_ofilm"
#define CHIP_FINGERCHIP_1260   "b_fgchip"
#define CHIP_CT_1140           "a_CT"
#define CHIP_PRIMAX_1140       "a_primax"

static struct proc_dir_entry *fp_id_dir = NULL;
static char* fp_id_name = "fp_id";
static char* fp_manu = CHIP_UNKNOWN ; // the length of this string should be less than FP_ID_MAX_LENGTH
static struct fp_data *fp_data_ptr = NULL;

static int fp_request_named_gpio(struct fp_data *fp_data,
        const char *label, int *gpio)
{
    struct device *dev = fp_data->dev;
    struct device_node *np = dev->of_node;

    int ret = of_get_named_gpio(np, label, 0);
    if (ret < 0) {
        dev_err(dev, "failed to get '%s'\n", label);
        return FP_ERROR_GPIO;
    }

    *gpio = ret;
    ret = devm_gpio_request(dev, *gpio, label);
    if (ret) {
        dev_err(dev, "failed to request gpio %d\n", *gpio);
        devm_gpio_free(dev, *gpio);
        return FP_ERROR_GPIO;
    }

    dev_info(dev, "%s - gpio: %d\n", label, *gpio);
    return FP_OK;
}

static int fp_gpio_parse_dts(struct fp_data *fp_data)
{
    int ret =-1;

    if (!fp_data) {
        dev_err(fp_data->dev,"fp_data is NULL\n");
        return FP_ERROR_GENERAL;
    }

    ret = fp_request_named_gpio(fp_data, "oppo,fp-id0",
            &fp_data->gpio_id0);
    if (ret)
        return FP_ERROR_GPIO;

    ret = fp_request_named_gpio(fp_data, "oppo,fp-id1",
            &fp_data->gpio_id1);
    if (ret)
        return FP_ERROR_GPIO;

    ret = fp_request_named_gpio(fp_data, "oppo,fp-id2",
            &fp_data->gpio_id2);
    if (ret)
        return FP_ERROR_GPIO;

    switch(get_project()) {
        case OPPO_16027:
            ret = fp_request_named_gpio(fp_data, "oppo,fp-id3",
                    &fp_data->gpio_id3);
            if (ret)
                return FP_ERROR_GPIO;
            break;
        default:
            break;
    }

    return FP_OK;
}

static ssize_t fp_id_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[10];
    char *p = page;
    int len = 0;

    p += sprintf(p, "%s", fp_manu);
    len = p - page;
    if (len > *pos)
        len -= *pos;
    else
        len = 0;

    if (copy_to_user(buf, page,len < count ? len  : count))
        return -EFAULT;

    *pos = *pos + (len < count ? len  : count);

    return len < count ? len  : count;
}

static struct file_operations fp_id_node_ctrl = {
    .read = fp_id_node_read,
};

static int fp_register_proc_fs(struct fp_data *fp_data)
{
    fp_data->fp_id1 = gpio_get_value(fp_data->gpio_id1);
    fp_data->fp_id2 = gpio_get_value(fp_data->gpio_id2);

    switch(get_project()) {
        case OPPO_16017:
            {
                if ((fp_data->fp_id1 == 0) && (fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_PRIMAX_1140; //fp_manu should be shorter than FP_ID_MAX_LENGTH here !!!
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 0)&&(fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_Turly_1140;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 1)&&(fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_CT_1140;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 1)&&(fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_OFILM_1140;
                    fp_data->fpsensor_type = FP_FPC;
                } else {
                    fp_manu = CHIP_UNKNOWN;
                    fp_data->fpsensor_type = FP_UNKNOWN;
                }
                break;
            }
        case OPPO_16061:
            {
                if ((fp_data->fp_id1 == 0) && (fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_PRIMAX_1140; //fp_manu should be shorter than FP_ID_MAX_LENGTH here !!!
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 0)&&(fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_FINGERCHIP_1260;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 1)&&(fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_CT_1140;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id1 == 1)&&(fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_OFILM_1140;
                    fp_data->fpsensor_type = FP_FPC;
                } else {
                    fp_manu = CHIP_UNKNOWN;
                    fp_data->fpsensor_type = FP_UNKNOWN;
                }
                break;
            }
        case OPPO_16027:
            {
                fp_data->fp_id3 = gpio_get_value(fp_data->gpio_id3); //for 16027 fp_id3 is ID0 from HW Config
                if ((fp_data->fp_id3 == 1) && (fp_data->fp_id1 == 0) && (fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_PRIMAX_1140 ; //fp_manu should be shorter than FP_ID_MAX_LENGTH here !!!
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id3 == 1) && (fp_data->fp_id1 == 0) && (fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_GOODIX ;
                    fp_data->fpsensor_type = FP_GOODIX;
                } else if ((fp_data->fp_id3 == 1) && (fp_data->fp_id1 == 1) && (fp_data->fp_id2 == 0)) {
                    fp_manu = CHIP_QTECH_1140 ;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id3 == 1) && (fp_data->fp_id1 == 1) && (fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_OFILM_1140 ;
                    fp_data->fpsensor_type = FP_FPC;
                } else if ((fp_data->fp_id3 == 0) && (fp_data->fp_id1 == 0) && (fp_data->fp_id2 == 1)) {
                    fp_manu = CHIP_Turly_1140 ;
                    fp_data->fpsensor_type = FP_FPC;
                } else {
                    fp_manu = CHIP_UNKNOWN ;
                    fp_data->fpsensor_type = FP_UNKNOWN;
                }
                break;
            }
        default:
            {
                fp_manu = CHIP_UNKNOWN;
                fp_data->fpsensor_type = FP_UNKNOWN;
                break;
            }
    }

    if (strlen(fp_manu) >= FP_ID_MAX_LENGTH) {
        dev_err(fp_data->dev,"fp_name should be shorter than %d\n",FP_ID_MAX_LENGTH);
        return FP_ERROR_GENERAL;
    }

    fp_id_dir = proc_create(fp_id_name, 0664, NULL, &fp_id_node_ctrl);
    if (fp_id_dir == NULL) {
        return FP_ERROR_GENERAL;
    }

    return FP_OK;
}

fp_vendor_t get_fpsensor_type(void)
{
    fp_vendor_t fpsensor_type = FP_UNKNOWN;

    if (FP_FPC == fp_data_ptr->fpsensor_type || FP_GOODIX == fp_data_ptr->fpsensor_type ) {
        fpsensor_type = fp_data_ptr->fpsensor_type;
        return fpsensor_type;
    }

    return fpsensor_type;
}

static int oppo_fp_common_probe(struct platform_device *fp_dev)
{
    int ret = 0;
    struct device *dev = &fp_dev->dev;
    struct fp_data *fp_data = NULL;
    fp_vendor_t fpsensor_type = FP_UNKNOWN;

    fp_data = devm_kzalloc(dev,sizeof(struct fp_data), GFP_KERNEL);
    if (fp_data == NULL) {
        dev_err(dev,"fp_data kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    fp_data->dev = dev;
    fp_data_ptr = fp_data;
    ret = fp_gpio_parse_dts(fp_data);
    if (ret)
        goto exit;

    ret = fp_register_proc_fs(fp_data);
    if (ret)
        goto exit;

    fpsensor_type = get_fpsensor_type();

    if (FP_FPC == fpsensor_type) {
        dev_info(dev, "found fpc sensor\n");
    } else if (FP_GOODIX == fpsensor_type) {
        dev_info(dev, "found gdx sensor\n");
    } else {
        dev_err(dev, "found error\n");
        return FP_ERROR_GENERAL;
    }

    return FP_OK;

exit:

    dev_err(dev,"fp_data probe failed ret = %d\n",ret);
    if(fp_data)
        devm_kfree(dev, fp_data);

    return ret;
}

static int oppo_fp_common_remove(struct platform_device *pdev)
{
    return FP_OK;
}

static struct of_device_id oppo_fp_common_match_table[] = {
    {   .compatible = "oppo,fp_common", },
    {}
};

static struct platform_driver oppo_fp_common_driver = {
    .probe = oppo_fp_common_probe,
    .remove = oppo_fp_common_remove,
    .driver = {
        .name = "oppo_fp_common",
        .owner = THIS_MODULE,
        .of_match_table = oppo_fp_common_match_table,
    },
};

static int __init oppo_fp_common_init(void)
{
    return platform_driver_register(&oppo_fp_common_driver);
}

static void __exit oppo_fp_common_exit(void)
{
    platform_driver_unregister(&oppo_fp_common_driver);
}

subsys_initcall(oppo_fp_common_init);
module_exit(oppo_fp_common_exit)

