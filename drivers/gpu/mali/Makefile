# Configuration for rk3066

ifdef CONFIG_ARCH_RK30
CONFIG:=rk30-m400-4
TARGET_PLATFORM:=rk30
#MALI_PLATFORM_FILE:=platform/${MALI_PLATFORM}/${TARGET_PLATFORM}.c
MALI_PLATFORM=rk30
export EXTRA_DEFINES += -DMALI_VOLTAGE_LOCK=0 -DMALI_POWER_MGMT_TEST_SUITE=0
else
#$(error No mali cfg for current arch)
endif

export CONFIG TARGET_PLATFORM MALI_PLATFORM

obj-$(CONFIG_MALI) += ump/ mali/ 

obj-y += rk_ump/
