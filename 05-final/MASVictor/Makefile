obj-m := gps_usb.o
KVERSION = $(shell uname -r)

KERNELDIR ?= ../../Kernel_build/linux-3.14/
#KERNELDIR ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

default:
	    $(MAKE) -C $(KERNELDIR) M=$(PWD) modules 
clean:
	    $(MAKE) -C $(KERNELDIR) M=$(PWD) clean
