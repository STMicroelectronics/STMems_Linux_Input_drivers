#
# Makefile for the input misc lsm303agr driver.
#

# Each configuration option enables a list of files.

lsm303agr-y += lsm303agr_acc.o lsm303agr_mag.o
obj-$(CONFIG_INPUT_LSM303AGR) += lsm303agr.o
obj-$(CONFIG_INPUT_LSM303AGR_I2C) += lsm303agr_acc_i2c.o lsm303agr_mag_i2c.o
obj-$(CONFIG_INPUT_LSM303AGR_SPI) += lsm303agr_acc_spi.o lsm303agr_mag_spi.o
