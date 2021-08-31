dts-dirs += apm
dts-dirs += arm
dts-dirs += cavium
#ifndef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2016/6/11 for boot mode 
#dts-dirs += qcom
#else
#ifeq ($(OPPO_TARGET_DEVICE),MSM_16017)
dts-dirs += 16017
#else ifeq ($(OPPO_TARGET_DEVICE),MSM_16027)
dts-dirs += 16027
#else
dts-dirs += 16061
#endif
#endif //VENDOR_EDIT

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
