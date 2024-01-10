ifneq ($(KERNELRELEASE),)

# object files to build
obj-m += nmac.o
nmac-y += nmac_main.o
nmac-y += nmac_reg_block.o
nmac-y += nmac_irq.o
nmac-y += nmac_dev.o
nmac-y += nmac_if.o
nmac-y += nmac_port.o
nmac-y += nmac_netdev.o
nmac-y += nmac_sched_block.o
nmac-y += nmac_scheduler.o
nmac-y += nmac_ptp.o
nmac-y += nmac_i2c.o
nmac-y += nmac_board.o
nmac-y += nmac_clk_info.o
nmac-y += nmac_stats.o
nmac-y += nmac_tx.o
nmac-y += nmac_rx.o
nmac-y += nmac_cq.o
nmac-y += nmac_eq.o
nmac-y += nmac_ethtool.o

else

ifneq ($(KERNEL_SRC),)
# alternatively to variable KDIR accept variable KERNEL_SRC as used in
# PetaLinux/Yocto for example
KDIR ?= $(KERNEL_SRC)
endif

KDIR ?= /lib/modules/$(shell uname -r)/build

all: modules

help modules modules_install clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) $@

install: modules_install

endif
