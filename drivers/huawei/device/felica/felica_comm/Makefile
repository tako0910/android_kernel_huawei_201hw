#
# Makefile for FeliCa control driver.
#
# target module name
DRV_NAME = felica_comm

## Defined compiling option
EXTRA_CFLAGS :=
EXTRA_CFLAGS += -Idrivers/felica_comm/inc

#
# Compile options for release version.
#

obj-$(CONFIG_HUAWEI_FEATURE_FELICA_T6ND5)   := $(DRV_NAME).o
$(DRV_NAME)-objs = \
	src/felica_comm.o
## End of Makefile

#******************************************************************************
#* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             *
#******************************************************************************
