#rkwifi packed Makefile

obj-y += bcmdhd/

rkwifi-objs += bcmdhd/bcmdhd.o wifi_sys/rkwifi_sys_iface.o rk_wifi_config.o


obj-$(CONFIG_RKWIFI_BLD)	+= rkwifi.o

.PHONY: clean

clean:
	find . -name '*.o*' -exec rm -f {} \; 
