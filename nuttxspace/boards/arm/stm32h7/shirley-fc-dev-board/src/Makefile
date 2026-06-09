include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_bringup.c stm32_appinit.c stm32_spi.c

ifeq ($(CONFIG_SENSORS_ICM40609D),y)
CSRCS += icm40609d.c
VPATH += :$(TOPDIR)/../drivers/sensors
DEPPATH += --dep-path $(TOPDIR)/../drivers/sensors
endif

include $(TOPDIR)/boards/Board.mk