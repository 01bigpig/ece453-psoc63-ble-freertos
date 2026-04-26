################################################################################
# Integrated IR Tracking System — Team 10
# Combines: CapSense + ToF + IMU + AFE(ADC) + PWM
################################################################################

MTB_TYPE=COMBINED

TARGET=APP_CY8CPROTO-063-BLE

APPNAME=team10_ir_tracker

TOOLCHAIN=GCC_ARM

CONFIG=Debug

VERBOSE=

################################################################################
# Advanced Configuration
################################################################################

COMPONENTS=FREERTOS

DISABLE_COMPONENTS=

# Pull in source files from sibling projects
SOURCES=\
    ../IMU/lsm6dsm_reg.c \
    ../IMU/source/app_hw/i2c.c \
    ../ToF/vl53l3cx/core/src/vl53lx_api.c \
    ../ToF/vl53l3cx/core/src/vl53lx_api_calibration.c \
    ../ToF/vl53l3cx/core/src/vl53lx_api_core.c \
    ../ToF/vl53l3cx/core/src/vl53lx_api_debug.c \
    ../ToF/vl53l3cx/core/src/vl53lx_api_preset_modes.c \
    ../ToF/vl53l3cx/core/src/vl53lx_core.c \
    ../ToF/vl53l3cx/core/src/vl53lx_core_support.c \
    ../ToF/vl53l3cx/core/src/vl53lx_dmax.c \
    ../ToF/vl53l3cx/core/src/vl53lx_hist_algos_gen3.c \
    ../ToF/vl53l3cx/core/src/vl53lx_hist_algos_gen4.c \
    ../ToF/vl53l3cx/core/src/vl53lx_hist_char.c \
    ../ToF/vl53l3cx/core/src/vl53lx_hist_core.c \
    ../ToF/vl53l3cx/core/src/vl53lx_hist_funcs.c \
    ../ToF/vl53l3cx/core/src/vl53lx_nvm.c \
    ../ToF/vl53l3cx/core/src/vl53lx_register_funcs.c \
    ../ToF/vl53l3cx/core/src/vl53lx_sigma_estimate.c \
    ../ToF/vl53l3cx/core/src/vl53lx_silicon_core.c \
    ../ToF/vl53l3cx/core/src/vl53lx_wait.c \
    ../ToF/vl53l3cx/core/src/vl53lx_xtalk.c \
    ../ToF/vl53l3cx/platform/src/vl53lx_platform.c \
    ../ToF/vl53l3cx/platform/src/vl53lx_platform_init.c \
    ../ToF/vl53l3cx/platform/src/vl53lx_platform_ipp.c \
    ../ToF/vl53l3cx/platform/src/vl53lx_platform_log.c

# Include paths for all module headers
INCLUDES=\
    . \
    source \
    ../IMU \
    ../IMU/source \
    ../IMU/source/app_hw \
    ../ToF/vl53l3cx/core/inc \
    ../ToF/vl53l3cx/platform/inc

DEFINES=

VFP_SELECT=

CFLAGS=

CXXFLAGS=

ASFLAGS=

LDFLAGS=-u _printf_float

LDLIBS=

LINKER_SCRIPT=

PREBUILD=

POSTBUILD=

################################################################################
# Paths
################################################################################

CY_APP_PATH=

CY_GETLIBS_SHARED_PATH=../
CY_GETLIBS_SHARED_NAME=mtb_shared

CY_COMPILER_GCC_ARM_DIR=

CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

CY_TOOLS_PATHS+=

CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
