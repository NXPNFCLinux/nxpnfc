#
# Makefile for nfc devices
#
obj-$(CONFIG_NXP_NFC_I2C)		+= nxpnfc_i2c.o
nxpnfc_i2c-objs				:= common.o i2c_drv.o
obj-$(CONFIG_NXP_NFC_SPI)		+= nxpnfc_spi.o
nxpnfc_spi-objs				:= common.o spi_drv.o
#ccflags-y                      += -DDEBUG
