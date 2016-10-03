VERSION = 2
PATCHLEVEL = 6
SUBLEVEL = 35
EXTRAVERSION = .13-ubuntu1
NAME=Crazed Snow-Weasel

ifneq ($(KERNELRELEASE),)
#
# Make rules for use from within 2.6 kbuild system
#
obj-m    += rc-marmitek-x10.o

EXTRA_CFLAGS=-I/home/stan/linux-source-3.8.0/include/media

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
