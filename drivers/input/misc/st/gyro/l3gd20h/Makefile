#
# Makefile for the input misc l3gd20h driver.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT_L3GD20H) += l3gd20h_gyr.o
obj-$(CONFIG_INPUT_L3GD20H_I2C) += l3gd20h_gyr_i2c.o
obj-$(CONFIG_INPUT_L3GD20H_SPI) += l3gd20h_gyr_spi.o

ccflags-y += -DL3GD20H_DEBUG
