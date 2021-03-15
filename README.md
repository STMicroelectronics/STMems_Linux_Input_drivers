Index
=======
	* Introduction
	* Integration details
	* More information
	* Copyright


Introduction
==============
This repository contains Linux kernel (v3.10, v3.4) with STMicroelectronics MEMS sensor support. STM sensor drivers are located under the directory [drivers/input/misc/st](https://github.com/STMicroelectronics/STMems_Linux_Input_drivers/tree/linux-3.10.y-gh/drivers/input/misc/st)  organized by sensor type:

### Inertial Module Unit (IMU):

> LSM330, LSM330DLC, LSM6DS0,
> LSM6DS3, LSM6DSL, LSM6DSM, LSM9DS0, LSM9DS1, ISM330DLC
> ASM330LHH

### eCompass:

> LSM303AGR, LSM303AH, LSM303C, LSM303D, LSM303DLHC, ISM303DAC

### Accelerometer:

> AIS328DQ, AIS3624DQ, H3LIS100DL, H3LIS331DL, LIS2DE, LIS2DE12,
> LIS2DH, LIS2DH12, LIS2DS12, LIS2HH12, LIS331DLH, LIS331HH,
> LIS3DE, LIS3DH, LIS3DSH, LIS2DW12, LIS3DHH, IIS3DHHC, IIS2DH

### Gyroscope:

> A3G4250D, L3G4200D, L3GD20, L3GD20H

### Magnetometer:

> LIS3MDL, LIS2MDL, IIS2MDC

### Humidity:

> HTS221

### Pressure:

> LPS22HB, LPS22HD, LPS25H, LPS25HB, LPS33HW, LPS35HW, LPS22HH

### Ultraviolet:

> UVIS25


Data collected by STM sensors are pushed to userland through the Linux kernel Input framework using *EV_MSC* events. User space applications can get sensor events by reading the related input device created in the /dev directory. Please see [Input][1] for more information.

All STM MEMS sensors support *I2C/SPI* digital interface. Please refer to [I2C][2] and [SPI][3] for detailed documentation.


Integration details
=====================

In order to explain how to integrate STM sensors in a different kernel, please consider the following *LSM6DSM* IMU example

### Source code integration

> * Copy driver source code into the target directory (e.g. *drivers/input/misc*)
> * Edit related Kconfig (e.g. *drivers/input/misc/Kconfig*) to include *LSM6DSM* support:

>         source "drivers/input/misc/lsm6dsm/Kconfig"

> * Edit related Makefile (e.g. *drivers/input/misc/Makefile*) adding the following line:

>         obj-y += lsm6dsm/

### Device Tree configuration

> To enable driver probing, add the lsm6dsm node to the platform device tree as described below.

> **Required properties:**

> *- compatible*: "st,lsm6dsm"

> *- reg*: the I2C address or SPI chip select the device will respond to

> *- interrupt-parent*: phandle to the parent interrupt controller as documented in [interrupts][4]

> *- interrupts*: interrupt mapping for IRQ as documented in [interrupts][4]

>
>**Recommended properties for SPI bus usage:**

> *- spi-max-frequency*: maximum SPI bus frequency as documented in [SPI][3]
>
> **Optional properties:**

> *- st,drdy-int-pin*: MEMS sensor interrupt line to use (default 1)

> I2C example (based on Raspberry PI 3):

>		&i2c0 {
>			status = "ok";
>			#address-cells = <0x1>;
>			#size-cells = <0x0>;
>			lsm6dsm@6b {
>				compatible = "st,lsm6dsm";
>				reg = <0x6b>;
>				interrupt-parent = <&gpio>;
>				interrupts = <26 IRQ_TYPE_EDGE_RISING>;
>		};

> SPI example (based on Raspberry PI 3):

>		&spi0 {
>			status = "ok";
>			#address-cells = <0x1>;
>			#size-cells = <0x0>;
>			lsm6dsm@0 {
>				spi-max-frequency = <500000>;
>				compatible = "st,lsm6dsm";
>				reg = <0>;
>				interrupt-parent = <&gpio>;
>				interrupts = <26 IRQ_TYPE_EDGE_RISING>;
>			};


### Kernel configuration

Configure kernel with *make menuconfig* (alternatively use *make xconfig* or *make qconfig*)

>		Device Drivers  --->
>			Input device support  --->
>			[*]   Miscellaneous devices  --->
>				<*>   STM MEMs Device Drivers  --->
>					<M>   Inertial motion unit  --->
>						<M>   STMicroelectronics LSM6DSM sensor
>


More Information
=================
[http://st.com](http://st.com)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi)

[https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bings/interrupt-controller/interrupts.txt](https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt)


Copyright
===========
Copyright (C) 2016 STMicroelectronics

This software is distributed under the GNU General Public License - see the accompanying COPYING file for more details.

[1]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/input "Input"
[2]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c "I2C"
[3]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi "SPI"
[4]: https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt "interrupts"
