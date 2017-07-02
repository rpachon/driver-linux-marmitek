ifneq ($(KERNELRELEASE),)

obj-m    += marmitek_remote.o

else  # We were called from command line

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
clean:
	rm -f *.mod.c *.ko *.o *~ core *.i
endif
