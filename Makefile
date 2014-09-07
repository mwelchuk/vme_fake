ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := vme_fake.o

else
# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	#ln -s $(KDIR)/drivers/vme/vme_bridge.h vme_bridge.h
	$(MAKE) -C $(KDIR) M=$$PWD

endif
