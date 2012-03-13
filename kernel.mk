KERNEL_BUILD_DIR=$(BASE_DIR)/
KERNEL_CONFIG=$(KERNEL_BUILD_DIR)/linux-ics/.config
KERNEL_CONFIG_SRC=$(KERNEL_SRC_REAL_DIR)/linux.config

ifeq ($(ARCH),arm)
KERNEL=$(KERNEL_BUILD_DIR)/linux-ics/arch/arm/boot/zImage
else
KERNEL=
endif

KERNEL_SRC_REAL_DIR:=$(BASE_DIR)/../linux-ics
KERNEL_SRC_DIR:=$(KERNEL_SRC_REAL_DIR)

$(KERNEL_SRC_DIR):
ifneq ($(KERNEL_SRC_DIR),$(KERNEL_SRC_REAL_DIR))
	mkdir -p $(KERNEL_SRC_DIR)
	tar -C $(KERNEL_SRC_REAL_DIR) --exclude=.svn -cpf - . | tar -C $(KERNEL_SRC_DIR) -xf -
else
	@echo "Use unpatched kernel"
endif

$(KERNEL_BUILD_DIR)/linux-ics: $(KERNEL_SRC_DIR)
	@mkdir -p $(KERNEL_BUILD_DIR)/linux-ics

$(KERNEL_CONFIG): $(KERNEL_CONFIG_SRC)
	cp $(KERNEL_CONFIG_SRC) $(KERNEL_CONFIG)
	@(PATH=$(shell pwd)/$(TOOLCHAIN_PATH):$(PATH) $(MAKE) -C $(KERNEL_SRC_DIR) oldconfig O=$(KERNEL_BUILD_DIR)/linux-ics ARCH=arm CROSS_COMPILE=arm-linux-)

$(KERNEL): $(KERNEL_BUILD_DIR)/linux-ics $(KERNEL_CONFIG) FORCE
	(PATH=$(shell pwd)/$(TOOLCHAIN_PATH):$(PATH) $(MAKE) $(MAKE_J) -C $(KERNEL_SRC_DIR) O=$(KERNEL_BUILD_DIR)/linux-ics ARCH=arm CROSS_COMPILE=arm-linux-) 
	
kernel-clean:
	@(PATH=$(shell pwd)/$(TOOLCHAIN_PATH):$(PATH) $(MAKE) -C $(KERNEL_SRC_DIR) O=$(KERNEL_BUILD_DIR)/linux-ics ARCH=arm CROSS_COMPILE=arm-linux- clean)

kernel-cleaner: kernel-clean
	rm -rf $(KERNEL_BUILD_DIR)/linux $(KERNEL_BUILD_DIR)/linux-ics

kernel: $(KERNEL)

.PHONY: kernel-clean kernel-cleaner FORCE

TARGETS	+= kernel

