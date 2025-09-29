# Virtual GeForce PCI Device Driver Makefile
# Based on Bochs GeForce emulation with 3D pipeline support

# Module name
obj-m := virtual_geforce_pci.o

# Kernel build directory - adjust path as needed
KERNEL_DIR := /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Compiler flags for the module
ccflags-y += -DDEBUG
ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable

# Enable 3D pipeline by default
ccflags-y += -DENABLE_3D_PIPELINE=1

# GeForce card type selection (can be overridden)
# 0x20 = GeForce3, 0x35 = GeForce FX 5900, 0x40 = GeForce 6800
GEFORCE_CARD_TYPE ?= 0x20
ccflags-y += -DDEFAULT_CARD_TYPE=$(GEFORCE_CARD_TYPE)

# VRAM size selection (in MB)
GEFORCE_VRAM_SIZE ?= 64
ccflags-y += -DDEFAULT_VRAM_SIZE=$(GEFORCE_VRAM_SIZE)

# Build targets
all:
	@echo "Building Virtual GeForce PCI Driver with 3D support..."
	@echo "Card Type: $(GEFORCE_CARD_TYPE) (0x20=GF3, 0x35=FX5900, 0x40=6800)"
	@echo "VRAM Size: $(GEFORCE_VRAM_SIZE) MB"
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	@echo "Cleaning build files..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f *.symvers *.order *.mod.c

install: all
	@echo "Installing Virtual GeForce PCI Driver..."
	sudo $(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	sudo depmod -a
	@echo "Driver installed. Load with: sudo modprobe virtual_geforce_pci"

uninstall:
	@echo "Uninstalling Virtual GeForce PCI Driver..."
	sudo modprobe -r virtual_geforce_pci || true
	sudo rm -f /lib/modules/$(shell uname -r)/extra/virtual_geforce_pci.ko
	sudo depmod -a

load: all
	@echo "Loading Virtual GeForce PCI Driver..."
	sudo insmod virtual_geforce_pci.ko card_type=$(GEFORCE_CARD_TYPE) enable_3d=1

unload:
	@echo "Unloading Virtual GeForce PCI Driver..."
	sudo rmmod virtual_geforce_pci || true

reload: unload load

# Test targets
test-gf3:
	$(MAKE) GEFORCE_CARD_TYPE=0x20 GEFORCE_VRAM_SIZE=64 reload

test-fx5900:
	$(MAKE) GEFORCE_CARD_TYPE=0x35 GEFORCE_VRAM_SIZE=128 reload

test-6800:
	$(MAKE) GEFORCE_CARD_TYPE=0x40 GEFORCE_VRAM_SIZE=256 reload

# Development targets
debug: ccflags-y += -DDEBUG_VERBOSE=1
debug: all

check:
	@echo "Checking kernel module..."
	modinfo virtual_geforce_pci.ko || echo "Module not built yet"

status:
	@echo "Driver status:"
	@lsmod | grep virtual_geforce || echo "Driver not loaded"
	@ls -la /dev/geforce_emu 2>/dev/null || echo "Device node not found"
	@lspci | grep -i nvidia || echo "No NVIDIA devices visible"

# Show 3D pipeline features
features:
	@echo "Virtual GeForce 3D Pipeline Features:"
	@echo "- Hardware-accelerated vertex transformation"
	@echo "- Multi-texture support (up to 16 texture units)"
	@echo "- Programmable vertex and pixel shaders"
	@echo "- Hardware lighting (up to 8 lights)"
	@echo "- Depth testing and Z-buffering"
	@echo "- Alpha blending with multiple blend modes"
	@echo "- Triangle rasterization with barycentric interpolation"
	@echo "- VRAM management with DMA coherent memory"
	@echo "- Interrupt-driven command processing"
	@echo "- Multi-channel FIFO command queue"

# Help target
help:
	@echo "Virtual GeForce PCI Driver Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build the kernel module"
	@echo "  clean      - Clean build files"
	@echo "  install    - Install module system-wide"
	@echo "  uninstall  - Remove installed module"
	@echo "  load       - Load module with default parameters"
	@echo "  unload     - Unload module"
	@echo "  reload     - Unload and reload module"
	@echo "  debug      - Build with verbose debugging"
	@echo "  check      - Show module information"
	@echo "  status     - Show driver status"
	@echo "  features   - Show 3D pipeline features"
	@echo ""
	@echo "Card-specific tests:"
	@echo "  test-gf3     - Test GeForce3 Ti 500 (64MB)"
	@echo "  test-fx5900  - Test GeForce FX 5900 (128MB)"
	@echo "  test-6800    - Test GeForce 6800 GT (256MB)"
	@echo ""
	@echo "Variables:"
	@echo "  GEFORCE_CARD_TYPE - Card type (0x20, 0x35, 0x40)"
	@echo "  GEFORCE_VRAM_SIZE - VRAM size in MB"
	@echo ""
	@echo "Examples:"
	@echo "  make GEFORCE_CARD_TYPE=0x35 GEFORCE_VRAM_SIZE=128"
	@echo "  make load card_type=0x40 enable_3d=1"

.PHONY: all clean install uninstall load unload reload test-gf3 test-fx5900 test-6800 debug check status features help