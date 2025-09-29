obj-m := virtual_geforce_pci.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Add compiler flags to disable SSE and fix other issues
ccflags-y := -mno-sse -mno-sse2 -mno-mmx -mno-3dnow -msoft-float -fno-strict-aliasing -O0

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

.PHONY: default clean install
