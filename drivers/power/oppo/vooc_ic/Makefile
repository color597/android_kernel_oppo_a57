ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-y      += oppo_vooc_fw_mtk.o
else
obj-y	+= oppo_vooc_fw_qcom.o
endif

obj-y	+= oppo_pic16f.o
obj-y	+= oppo_stm8s.o
obj-y	+= oppo_adapter.o
