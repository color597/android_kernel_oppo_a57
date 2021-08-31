ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable
endif

obj-y      += charger_ic/
obj-y      += gauge_ic/
obj-y      += vooc_ic/

obj-y	+= oppo_charger.o
obj-y	+= oppo_gauge.o
obj-y	+= oppo_vooc.o
obj-y	+= oppo_short.o