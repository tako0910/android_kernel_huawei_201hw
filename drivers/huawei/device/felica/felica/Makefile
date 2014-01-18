#
# Makefile for FeliCa control driver.
#
# target module name
DRV_NAME = felica_ctrl

## Defined compiling option
EXTRA_CFLAGS :=
EXTRA_CFLAGS += -Idrivers/felica/common
EXTRA_CFLAGS += -Idrivers/felica/inc

#
# Compile options for release version.
#

obj-$(CONFIG_HUAWEI_FEATURE_FELICA_T6ND5)   := $(DRV_NAME).o
$(DRV_NAME)-objs = \
        src/felica_ctrl.o 
## End of Makefile

#******************************************************************************
#* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             *
#******************************************************************************
