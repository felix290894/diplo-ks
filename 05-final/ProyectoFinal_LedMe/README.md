README:

Este repositorio presenta una propuesta básica de driver del acelerómetro ADXL345 utilizando el protocolo de comunicación serial i2c para la tarjeta de desarrollo Raspberry PI3.

EL proyecto incluye los siguientes archivos:
-) adxl.c: driver acelerómetro ADXL345
-) Makefile: compilación del driver
-) sript.py: programa para probar la adquisición de datos del sensor desde espacio de usuario
-) screenshots/ muestran la evidencia de funcionamiento del driver

El driver se desarrolló utilizando la siguiente configuración en la Raspberry Pi3:
-)Versión de Kernel de linux: 4.4.21-v7+
-)Versión de sistema operativo Raspbian: 2016-09-23-raspbian-jessie


Referencias:

linux-headers-rpi . https://www.niksula.hut.fi/~mhiienka/Rpi/linux-headers-rpi/ 
Exploring Raspberry Pi : Interfacing to the Real World with Embedded Linux, Derek Molloy. Chapter 16
ADXL345 Datasheet. https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
I2C Driver for Linux Based Embedded System. http://invo-tronics.com/i2c-driver-for-linux-based-embedded-system/

Embedded Linux training (Free-electrons). http://free-electrons.com/doc/training/embedded-linux/embedded-linux-slides.pdf

Embedded Linux kernel and driver development training (Free-electrons). http://free-electrons.com/doc/training/linux-kernel/linux-kernel-slides.pdf

Equipo LedMe:
Esau Rojas,
Samuel Quiroz, 
Hector Reyes.


