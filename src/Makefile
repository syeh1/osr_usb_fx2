.PHONY: clean

osr_usb_fx2-objs := osr_usb_drv.o osr_usb_attrib.o

obj-m = osr_usb_fx2.o

KVERSION = $(shell uname -r)

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean

