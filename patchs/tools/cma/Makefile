# Kernel modules
#
# To compile for ARM:
# make ARCH=arm CC=arm-none-linux-gnueabi-gcc
#
obj-m	+= cma_test.o

build: kernel_modules

kernel_modules:
	${MAKE} -C $(CURDIR)/../.. M=$(CURDIR)
clean:
	${MAKE} -C $(CURDIR)/../.. M=$(CURDIR) clean
