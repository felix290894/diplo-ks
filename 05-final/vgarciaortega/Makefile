obj-m += proy-3.o

all:
#	Para compilar el módulo de kernel en PC con Ubuntu:
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

#	Para compilar el módulo de kernel con el compilador cruzado para la Raspberry PI 3:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /home/victor/diplomadoLinux/linux_rasp2/linux M=$(PWD) modules

clean:
#	Para compilar el módulo de kernel en PC con Ubuntu:
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

#	Para compilar el módulo de kernel con el compilador cruzado para la Raspberry PI 3:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C /home/victor/diplomadoLinux/linux_rasp2/linux M=$(PWD) clean
