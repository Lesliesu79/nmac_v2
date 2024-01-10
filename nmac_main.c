#include "nmac.h"
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
#include <linux/pci-aspm.h>
#endif

MODULE_DESCRIPTION("nmac driver");
MODULE_AUTHOR("Alex Forencich");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

unsigned int nmac_num_ev_queue_entries = 1024;
unsigned int nmac_num_tx_queue_entries = 1024;
unsigned int nmac_num_rx_queue_entries = 1024;

module_param_named(num_ev_queue_entries, nmac_num_ev_queue_entries, uint, 0444);
MODULE_PARM_DESC(num_ev_queue_entries, "number of entries to allocate per event queue (default: 1024)");
module_param_named(num_tx_queue_entries, nmac_num_tx_queue_entries, uint, 0444);
MODULE_PARM_DESC(num_tx_queue_entries, "number of entries to allocate per transmit queue (default: 1024)");
module_param_named(num_rx_queue_entries, nmac_num_rx_queue_entries, uint, 0444);
MODULE_PARM_DESC(num_rx_queue_entries, "number of entries to allocate per receive queue (default: 1024)");

unsigned int nmac_link_status_poll = NMAC_LINK_STATUS_POLL_MS;

module_param_named(link_status_poll, nmac_link_status_poll, uint, 0444);
MODULE_PARM_DESC(link_status_poll,
		 "link status polling interval, in ms (default: 1000; 0 to turn off)");


#ifdef CONFIG_PCI
static const struct pci_device_id nmac_pci_id_table[] = {
	{PCI_DEVICE(0x1234, 0x1001)},
	{PCI_DEVICE(0x5543, 0x1001)},
	{PCI_DEVICE(0x10ee, 0x1068)},
	{0 /* end */ }
};

MODULE_DEVICE_TABLE(pci, nmac_pci_id_table);
#endif

#ifdef CONFIG_OF
static struct of_device_id nmac_of_id_table[] = {
	{ .compatible = "corundum,nmac" },
	{ },
};
MODULE_DEVICE_TABLE(of, nmac_of_id_table);
#endif

static LIST_HEAD(nmac_devices);
static DEFINE_SPINLOCK(nmac_devices_lock);

static unsigned int nmac_get_free_id(void)
{
	struct nmac_dev *nmac;
	unsigned int id = 0;
	bool available = false;

	while (!available) {
		available = true;
		list_for_each_entry(nmac, &nmac_devices, dev_list_node) {
			if (nmac->id == id) {
				available = false;
				id++;
				break;
			}
		}
	}

	return id;
}

static void nmac_assign_id(struct nmac_dev *nmac)
{
	spin_lock(&nmac_devices_lock);
	nmac->id = nmac_get_free_id();
	list_add_tail(&nmac->dev_list_node, &nmac_devices);
	spin_unlock(&nmac_devices_lock);

	snprintf(nmac->name, sizeof(nmac->name), DRIVER_NAME "%d", nmac->id);
}

static void nmac_free_id(struct nmac_dev *nmac)
{
	spin_lock(&nmac_devices_lock);
	list_del(&nmac->dev_list_node);
	spin_unlock(&nmac_devices_lock);
}

static int nmac_common_setdma(struct nmac_dev *nmac)
{
	int ret;
	struct device *dev = nmac->dev;

	// Set mask
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_warn(dev, "Warning: failed to set 64 bit PCI DMA mask");
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "Failed to set PCI DMA mask");
			return ret;
		}
	}

	// Set max segment size
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));

	return ret;
}

#ifdef CONFIG_OF
static int nmac_platform_get_mac_address(struct nmac_dev *nmac)
{
	int ret;
	struct device *dev = nmac->dev;
	char mac_base[ETH_ALEN];
	struct device_node *np;
	u32 inc_idx;
	u32 inc;
	int k;

	/* NOTE: Not being able to get a (base) MAC address shall not be an
	 *       error to fail on intentionally. Thus we are warning, only.
	 */
	ret = eth_platform_get_mac_address(dev, mac_base);
	if (ret) {
		dev_warn(dev, "Unable to get MAC address\n");
		return 0;
	}

	np = nmac->dev->of_node;
	if (!np)
		return 0;

	if (of_property_read_u32(np, NMAC_PROP_MAC_ADDR_INC_BYTE, &inc_idx))
		inc_idx = 5;
	if ((inc_idx < 3) || (inc_idx > 5)) {
		dev_err(dev, "Invalid property \"" NMAC_PROP_MAC_ADDR_INC_BYTE "\"\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, NMAC_PROP_MAC_ADDR_INC, &inc);
	if (ret == -EINVAL) {
		inc = 0;
	} else if (ret) {
		dev_err(dev, "Invalid property \"" NMAC_PROP_MAC_ADDR_INC "\"\n");
		return ret;
	}

	if (of_property_read_bool(np, NMAC_PROP_MAC_ADDR_LOCAL))
		mac_base[0] |= BIT(1);

	nmac->mac_count = nmac->if_count;
	for (k = 0; k < nmac->mac_count; k++) {
		memcpy(nmac->mac_list[k], mac_base, ETH_ALEN);
		nmac->mac_list[k][inc_idx] += inc + k;
	}

	return 0;
}

static void nmac_platform_module_eeprom_put(struct nmac_dev *nmac)
{
	int k;

	for (k = 0; k < nmac->if_count; k++)
		if (nmac->mod_i2c_client)
			put_device(&nmac->mod_i2c_client[k]->dev);
}

static int nmac_platform_module_eeprom_get(struct nmac_dev *nmac)
{
	int ret;
	struct device *dev = nmac->dev;
	int k;

	ret = 0;

	if (!dev->of_node)
		return 0;

	for (k = 0; k < nmac->if_count; k++) {
		struct device_node *np;
		struct i2c_client *cl;

		/* NOTE: Not being able to get a phandle for module EEPROM shall
		 *       not be an error to fail on intentionally. Thus we are
		 *       warning, only.
		 */
		np = of_parse_phandle(dev->of_node, NMAC_PROP_MODULE_EEPROM, k);
		if (!np) {
			dev_warn(dev, "Missing phandle to module EEPROM for interface %d\n", k);
			continue;
		}

		cl = of_find_i2c_device_by_node(np);
		if (!cl) {
			ret = -ENOENT;
			dev_err(dev, "Failed to find I2C device for module of interface %d\n", k);
			of_node_put(np);
			break;
		} else {
			nmac->mod_i2c_client[k] = cl;
			nmac->mod_i2c_client_count++;
		}
		of_node_put(np);
	}

	if (ret)
		nmac_platform_module_eeprom_put(nmac);

	return ret;
}
#endif

static void nmac_common_remove(struct nmac_dev *nmac);

#ifdef CONFIG_AUXILIARY_BUS
static void nmac_adev_release(struct device *dev)
{
	struct nmac_adev *nmac_adev = container_of(dev, struct nmac_adev, adev.dev);

	if (nmac_adev->ptr)
		*nmac_adev->ptr = NULL;
	kfree(nmac_adev);
}
#endif

static int nmac_common_probe(struct nmac_dev *nmac)
{
	int ret = 0;
	struct device *dev = nmac->dev;
	struct nmac_reg_block *rb;
	struct rtc_time tm;

	int k = 0, l = 0;

	// Enumerate registers
	nmac->rb_list = nmac_enumerate_reg_block_list(nmac->hw_addr, 0, nmac->hw_regs_size);
	if (!nmac->rb_list) {
		dev_err(dev, "Failed to enumerate blocks");
		return -EIO;
	}

	dev_info(dev, "Device-level register blocks:");
	for (rb = nmac->rb_list; rb->regs; rb++)
		dev_info(dev, " type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
				(rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

	// Read ID registers
	nmac->fw_id_rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_FW_ID_TYPE, NMAC_RB_FW_ID_VER, 0);

	if (!nmac->fw_id_rb) {
		ret = -EIO;
		dev_err(dev, "Error: FW ID block not found");
		goto fail_rb_init;
	}

	nmac->fpga_id = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_FPGA_ID);
	nmac->fw_id = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_FW_ID);
	nmac->fw_ver = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_FW_VER);
	nmac->board_id = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_BOARD_ID);
	nmac->board_ver = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_BOARD_VER);
	nmac->build_date = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_BUILD_DATE);
	nmac->git_hash = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_GIT_HASH);
	nmac->rel_info = ioread32(nmac->fw_id_rb->regs + NMAC_RB_FW_ID_REG_REL_INFO);

	rtc_time64_to_tm(nmac->build_date, &tm);

	dev_info(dev, "FPGA ID: 0x%08x", nmac->fpga_id);
	dev_info(dev, "FW ID: 0x%08x", nmac->fw_id);
	dev_info(dev, "FW version: %d.%d.%d.%d", nmac->fw_ver >> 24,
			(nmac->fw_ver >> 16) & 0xff,
			(nmac->fw_ver >> 8) & 0xff,
			nmac->fw_ver & 0xff);
	dev_info(dev, "Board ID: 0x%08x", nmac->board_id);
	dev_info(dev, "Board version: %d.%d.%d.%d", nmac->board_ver >> 24,
			(nmac->board_ver >> 16) & 0xff,
			(nmac->board_ver >> 8) & 0xff,
			nmac->board_ver & 0xff);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
	dev_info(dev, "Build date: %ptRd %ptRt UTC (raw: 0x%08x)", &tm, &tm, nmac->build_date);
#else
	dev_info(dev, "Build date: %04d-%02d-%02d %02d:%02d:%02d UTC (raw: 0x%08x)",
			tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, nmac->build_date);
#endif
	dev_info(dev, "Git hash: %08x", nmac->git_hash);
	dev_info(dev, "Release info: %08x", nmac->rel_info);

	rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_APP_INFO_TYPE, NMAC_RB_APP_INFO_VER, 0);

	if (rb) {
		nmac->app_id = ioread32(rb->regs + NMAC_RB_APP_INFO_REG_ID);
		dev_info(dev, "Application ID: 0x%08x", nmac->app_id);
	}

	nmac_clk_info_init(nmac);
	nmac_stats_init(nmac);

	nmac->phc_rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_PHC_TYPE, NMAC_RB_PHC_VER, 0);

	// Enumerate interfaces
	nmac->if_rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_IF_TYPE, NMAC_RB_IF_VER, 0);

	if (!nmac->if_rb) {
		ret = -EIO;
		dev_err(dev, "Error: interface block not found");
		goto fail_rb_init;
	}

	nmac->if_offset = ioread32(nmac->if_rb->regs + NMAC_RB_IF_REG_OFFSET);
	nmac->if_count = ioread32(nmac->if_rb->regs + NMAC_RB_IF_REG_COUNT);
	nmac->if_stride = ioread32(nmac->if_rb->regs + NMAC_RB_IF_REG_STRIDE);
	nmac->if_csr_offset = ioread32(nmac->if_rb->regs + NMAC_RB_IF_REG_CSR_OFFSET);

	dev_info(dev, "IF offset: 0x%08x", nmac->if_offset);
	dev_info(dev, "IF count: %d", nmac->if_count);
	dev_info(dev, "IF stride: 0x%08x", nmac->if_stride);
	dev_info(dev, "IF CSR offset: 0x%08x", nmac->if_csr_offset);

	// check BAR size
	if (nmac->if_count * nmac->if_stride > nmac->hw_regs_size) {
		ret = -EIO;
		dev_err(dev, "Invalid BAR configuration (%d IF * 0x%x > 0x%llx)",
				nmac->if_count, nmac->if_stride, nmac->hw_regs_size);
		goto fail_bar_size;
	}

	if (nmac->pfdev) {
#ifdef CONFIG_OF
		ret = nmac_platform_get_mac_address(nmac);
		if (ret)
			goto fail_board;

		ret = nmac_platform_module_eeprom_get(nmac);
		if (ret)
			goto fail_board;
#endif
	} else {
		// Board-specific init
		ret = nmac_board_init(nmac);
		if (ret) {
			dev_err(dev, "Failed to initialize board");
			goto fail_board;
		}
	}

	// register PHC
	if (nmac->phc_rb)
		nmac_register_phc(nmac);

	mutex_init(&nmac->state_lock);

	// Set up interfaces
	nmac->dev_port_max = 0;
	nmac->dev_port_limit = NMAC_MAX_IF;

	nmac->if_count = min_t(u32, nmac->if_count, NMAC_MAX_IF);

	for (k = 0; k < nmac->if_count; k++) {
		dev_info(dev, "Creating interface %d", k);
		ret = nmac_create_interface(nmac, &nmac->interface[k], k, nmac->hw_addr + k * nmac->if_stride);
		if (ret) {
			dev_err(dev, "Failed to create interface: %d", ret);
			goto fail_create_if;
		}
		nmac->dev_port_max = nmac->interface[k]->dev_port_max;
	}

	// pass module I2C clients to interface instances
	for (k = 0; k < nmac->if_count; k++) {
		struct nmac_if *interface = nmac->interface[k];
		interface->mod_i2c_client = nmac->mod_i2c_client[k];

		for (l = 0; l < interface->ndev_count; l++) {
			struct nmac_priv *priv = netdev_priv(interface->ndev[l]);
			priv->mod_i2c_client = nmac->mod_i2c_client[k];
		}
	}

fail_create_if:
	nmac->misc_dev.minor = MISC_DYNAMIC_MINOR;
	nmac->misc_dev.name = nmac->name;
	nmac->misc_dev.fops = &nmac_fops;
	nmac->misc_dev.parent = dev;

	ret = misc_register(&nmac->misc_dev);
	if (ret) {
		nmac->misc_dev.this_device = NULL;
		dev_err(dev, "misc_register failed: %d\n", ret);
		goto fail_miscdev;
	}

	dev_info(dev, "Registered device %s", nmac->name);

#ifdef CONFIG_AUXILIARY_BUS
	if (nmac->app_id) {
		nmac->app_adev = kzalloc(sizeof(*nmac->app_adev), GFP_KERNEL);
		if (!nmac->app_adev) {
			ret = -ENOMEM;
			goto fail_adev;
		}

		snprintf(nmac->app_adev->name, sizeof(nmac->app_adev->name), "app_%08x", nmac->app_id);

		nmac->app_adev->adev.id = nmac->id;
		nmac->app_adev->adev.name = nmac->app_adev->name;
		nmac->app_adev->adev.dev.parent = dev;
		nmac->app_adev->adev.dev.release = nmac_adev_release;
		nmac->app_adev->mdev = nmac;
		nmac->app_adev->ptr = &nmac->app_adev;

		ret = auxiliary_device_init(&nmac->app_adev->adev);
		if (ret) {
			kfree(nmac->app_adev);
			nmac->app_adev = NULL;
			goto fail_adev;
		}

		ret = auxiliary_device_add(&nmac->app_adev->adev);
		if (ret) {
			auxiliary_device_uninit(&nmac->app_adev->adev);
			nmac->app_adev = NULL;
			goto fail_adev;
		}

		dev_info(dev, "Registered auxiliary bus device " DRIVER_NAME ".%s.%d",
				nmac->app_adev->adev.name, nmac->app_adev->adev.id);
	}
#endif

	// probe complete
	return 0;

	// error handling
#ifdef CONFIG_AUXILIARY_BUS
fail_adev:
#endif
fail_miscdev:
fail_board:
fail_bar_size:
fail_rb_init:
	nmac_common_remove(nmac);
	return ret;
}

static void nmac_common_remove(struct nmac_dev *nmac)
{
	int k = 0;

#ifdef CONFIG_AUXILIARY_BUS
	if (nmac->app_adev) {
		auxiliary_device_delete(&nmac->app_adev->adev);
		auxiliary_device_uninit(&nmac->app_adev->adev);
	}
#endif

	if (nmac->misc_dev.this_device)
		misc_deregister(&nmac->misc_dev);

	for (k = 0; k < ARRAY_SIZE(nmac->interface); k++)
		if (nmac->interface[k])
			nmac_destroy_interface(&nmac->interface[k]);

	nmac_unregister_phc(nmac);
	if (nmac->pfdev) {
#ifdef CONFIG_OF
		nmac_platform_module_eeprom_put(nmac);
#endif
	} else {
		nmac_board_deinit(nmac);
	}
	if (nmac->rb_list)
		nmac_free_reg_block_list(nmac->rb_list);
}

#ifdef CONFIG_PCI
static int nmac_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	pr_info("lbj-test pci probe");
	int ret = 0;
	struct nmac_dev *nmac;
	struct device *dev = &pdev->dev;
	struct pci_dev *bridge = pci_upstream_bridge(pdev);

	dev_info(dev, DRIVER_NAME " PCI probe");
	dev_info(dev, " Vendor: 0x%04x", pdev->vendor);
	dev_info(dev, " Device: 0x%04x", pdev->device);
	dev_info(dev, " Subsystem vendor: 0x%04x", pdev->subsystem_vendor);
	dev_info(dev, " Subsystem device: 0x%04x", pdev->subsystem_device);
	dev_info(dev, " Class: 0x%06x", pdev->class);
	dev_info(dev, " PCI ID: %04x:%02x:%02x.%d", pci_domain_nr(pdev->bus),
			pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));

	if (pdev->pcie_cap) {
		u16 devctl;
		u32 lnkcap;
		u16 lnksta;

		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_DEVCTL, &devctl);
		pci_read_config_dword(pdev, pdev->pcie_cap + PCI_EXP_LNKCAP, &lnkcap);
		pci_read_config_word(pdev, pdev->pcie_cap + PCI_EXP_LNKSTA, &lnksta);

		dev_info(dev, " Max payload size: %d bytes",
				128 << ((devctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5));
		dev_info(dev, " Max read request size: %d bytes",
				128 << ((devctl & PCI_EXP_DEVCTL_READRQ) >> 12));
		dev_info(dev, " Link capability: gen %d x%d",
				lnkcap & PCI_EXP_LNKCAP_SLS, (lnkcap & PCI_EXP_LNKCAP_MLW) >> 4);
		dev_info(dev, " Link status: gen %d x%d",
				lnksta & PCI_EXP_LNKSTA_CLS, (lnksta & PCI_EXP_LNKSTA_NLW) >> 4);
		dev_info(dev, " Relaxed ordering: %s",
				devctl & PCI_EXP_DEVCTL_RELAX_EN ? "enabled" : "disabled");
		dev_info(dev, " Phantom functions: %s",
				devctl & PCI_EXP_DEVCTL_PHANTOM ? "enabled" : "disabled");
		dev_info(dev, " Extended tags: %s",
				devctl & PCI_EXP_DEVCTL_EXT_TAG ? "enabled" : "disabled");
		dev_info(dev, " No snoop: %s",
				devctl & PCI_EXP_DEVCTL_NOSNOOP_EN ? "enabled" : "disabled");
	}

#ifdef CONFIG_NUMA
	dev_info(dev, " NUMA node: %d", pdev->dev.numa_node);
#endif

	if (bridge) {
		dev_info(dev, " PCI ID (bridge): %04x:%02x:%02x.%d", pci_domain_nr(bridge->bus),
				bridge->bus->number, PCI_SLOT(bridge->devfn), PCI_FUNC(bridge->devfn));
	}

	if (bridge && bridge->pcie_cap) {
		u32 lnkcap;
		u16 lnksta;

		pci_read_config_dword(bridge, bridge->pcie_cap + PCI_EXP_LNKCAP, &lnkcap);
		pci_read_config_word(bridge, bridge->pcie_cap + PCI_EXP_LNKSTA, &lnksta);

		dev_info(dev, " Link capability (bridge): gen %d x%d",
				lnkcap & PCI_EXP_LNKCAP_SLS, (lnkcap & PCI_EXP_LNKCAP_MLW) >> 4);
		dev_info(dev, " Link status (bridge): gen %d x%d",
				lnksta & PCI_EXP_LNKSTA_CLS, (lnksta & PCI_EXP_LNKSTA_NLW) >> 4);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
	pcie_print_link_status(pdev);
#endif

	nmac = devm_kzalloc(dev, sizeof(*nmac), GFP_KERNEL);
	if (!nmac)
		return -ENOMEM;

	nmac->dev = dev;
	nmac->pdev = pdev;
	pci_set_drvdata(pdev, nmac);

	// assign ID and add to list
	nmac_assign_id(nmac);

	// Disable ASPM
	pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S |
			PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_CLKPM);

	// Enable device
	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(dev, "Failed to enable PCI device");
		goto fail_enable_device;
	}

	// Set DMA properties
	ret = nmac_common_setdma(nmac);
	if (ret)
		goto fail_regions;

	// Reserve regions
	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret) {
		dev_err(dev, "Failed to reserve regions");
		goto fail_regions;
	}

	nmac->hw_regs_size = pci_resource_len(pdev, 0);
	nmac->hw_regs_phys = pci_resource_start(pdev, 0);
	nmac->app_hw_regs_size = pci_resource_len(pdev, 2);
	nmac->app_hw_regs_phys = pci_resource_start(pdev, 2);
	nmac->ram_hw_regs_size = pci_resource_len(pdev, 4);
	nmac->ram_hw_regs_phys = pci_resource_start(pdev, 4);

	// Map BARs
	dev_info(dev, "Control BAR size: %llu", nmac->hw_regs_size);
	nmac->hw_addr = pci_ioremap_bar(pdev, 0);
	if (!nmac->hw_addr) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to map control BAR");
		goto fail_map_bars;
	}

	if (nmac->app_hw_regs_size) {
		dev_info(dev, "Application BAR size: %llu", nmac->app_hw_regs_size);
		nmac->app_hw_addr = pci_ioremap_bar(pdev, 2);
		if (!nmac->app_hw_addr) {
			ret = -ENOMEM;
			dev_err(dev, "Failed to map application BAR");
			goto fail_map_bars;
		}
	}

	if (nmac->ram_hw_regs_size) {
		dev_info(dev, "RAM BAR size: %llu", nmac->ram_hw_regs_size);
		nmac->ram_hw_addr = pci_ioremap_bar(pdev, 4);
		if (!nmac->ram_hw_addr) {
			ret = -ENOMEM;
			dev_err(dev, "Failed to map RAM BAR");
			goto fail_map_bars;
		}
	}

	// Check if device needs to be reset
	if (ioread32(nmac->hw_addr+4) == 0xffffffff) {
		ret = -EIO;
		dev_err(dev, "Device needs to be reset");
		goto fail_reset;
	}

	// Set up interrupts
	ret = nmac_irq_init_pcie(nmac);
	if (ret) {
		dev_err(dev, "Failed to set up interrupts");
		goto fail_init_irq;
	}

	// Enable bus mastering for DMA
	pci_set_master(pdev);

	// Common init
	ret = nmac_common_probe(nmac);
	if (ret)
		goto fail_common;

	// probe complete
	return 0;

	// error handling
fail_common:
	pci_clear_master(pdev);
	nmac_irq_deinit_pcie(nmac);
fail_reset:
fail_init_irq:
fail_map_bars:
	if (nmac->hw_addr)
		pci_iounmap(pdev, nmac->hw_addr);
	if (nmac->app_hw_addr)
		pci_iounmap(pdev, nmac->app_hw_addr);
	if (nmac->ram_hw_addr)
		pci_iounmap(pdev, nmac->ram_hw_addr);
	pci_release_regions(pdev);
fail_regions:
	pci_disable_device(pdev);
fail_enable_device:
	nmac_free_id(nmac);
	return ret;
}

static void nmac_pci_remove(struct pci_dev *pdev)
{
	struct nmac_dev *nmac = pci_get_drvdata(pdev);

	dev_info(&pdev->dev, DRIVER_NAME " PCI remove");

	nmac_common_remove(nmac);

	pci_clear_master(pdev);
	nmac_irq_deinit_pcie(nmac);
	if (nmac->hw_addr)
		pci_iounmap(pdev, nmac->hw_addr);
	if (nmac->app_hw_addr)
		pci_iounmap(pdev, nmac->app_hw_addr);
	if (nmac->ram_hw_addr)
		pci_iounmap(pdev, nmac->ram_hw_addr);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	nmac_free_id(nmac);
}

static void nmac_pci_shutdown(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, DRIVER_NAME " PCI shutdown");

	nmac_pci_remove(pdev);
}

static struct pci_driver nmac_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = nmac_pci_id_table,
	.probe = nmac_pci_probe,
	.remove = nmac_pci_remove,
	.shutdown = nmac_pci_shutdown
};
#endif /* CONFIG_PCI */

static int nmac_platform_probe(struct platform_device *pdev)
{
	pr_info("lbj-test namc_platform_probe running");
	int ret;
	struct nmac_dev *nmac;
	struct device *dev = &pdev->dev;
	struct resource *res;

	dev_info(dev, DRIVER_NAME " platform probe");

#ifdef CONFIG_NUMA
	dev_info(dev, " NUMA node: %d", pdev->dev.numa_node);
#endif

	nmac = devm_kzalloc(dev, sizeof(*nmac), GFP_KERNEL);
	if (!nmac)
		return -ENOMEM;

	nmac->dev = dev;
	nmac->pfdev = pdev;
	platform_set_drvdata(pdev, nmac);

	// assign ID and add to list
	nmac_assign_id(nmac);

	// Set DMA properties
	ret = nmac_common_setdma(nmac);
	if (ret)
		goto fail;

	// Reserve and map regions
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nmac->hw_regs_size = resource_size(res);
	nmac->hw_regs_phys = res->start;

	dev_info(dev, "Control BAR size: %llu", nmac->hw_regs_size);
	nmac->hw_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(nmac->hw_addr)) {
		ret = PTR_ERR(nmac->hw_addr);
		dev_err(dev, "Failed to map control BAR");
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		void __iomem *hw_addr;

		nmac->app_hw_regs_size = resource_size(res);
		nmac->app_hw_regs_phys = res->start;

		dev_info(dev, "Application BAR size: %llu", nmac->app_hw_regs_size);
		hw_addr = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(hw_addr)) {
			ret = PTR_ERR(hw_addr);
			dev_err(dev, "Failed to map application BAR");
			goto fail;
		}
		nmac->app_hw_addr = hw_addr;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res) {
		void __iomem *hw_addr;

		nmac->ram_hw_regs_size = resource_size(res);
		nmac->ram_hw_regs_phys = res->start;

		dev_info(dev, "RAM BAR size: %llu", nmac->ram_hw_regs_size);
		hw_addr = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(hw_addr)) {
			ret = PTR_ERR(hw_addr);
			dev_err(dev, "Failed to map RAM BAR");
			goto fail;
		}
		nmac->ram_hw_addr = hw_addr;
	}

	// Set up interrupts
	ret = nmac_irq_init_platform(nmac);
	if (ret) {
		dev_err(dev, "Failed to set up interrupts");
		goto fail;
	}

	// Common init
	ret = nmac_common_probe(nmac);
	if (ret)
		goto fail;

	// probe complete
	return 0;

	// error handling
fail:
	nmac_free_id(nmac);
	return ret;
}

static int nmac_platform_remove(struct platform_device *pdev)
{
	struct nmac_dev *nmac = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, DRIVER_NAME " platform remove");

	nmac_common_remove(nmac);

	nmac_free_id(nmac);
	return 0;
}

static struct platform_driver nmac_platform_driver = {
	.probe = nmac_platform_probe,
	.remove = nmac_platform_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(nmac_of_id_table),
	},
};

static int __init nmac_init(void)
{
	int rc;

#ifdef CONFIG_PCI
	rc = pci_register_driver(&nmac_pci_driver);
	if (rc)
		return rc;
#endif

	rc = platform_driver_register(&nmac_platform_driver);
	if (rc)
		goto err;

	return 0;

err:
#ifdef CONFIG_PCI
	pci_unregister_driver(&nmac_pci_driver);
#endif
	return rc;
}

static void __exit nmac_exit(void)
{
	platform_driver_unregister(&nmac_platform_driver);

#ifdef CONFIG_PCI
	pci_unregister_driver(&nmac_pci_driver);
#endif
}

module_init(nmac_init);
module_exit(nmac_exit);
