#include "nmac.h"

#include <linux/module.h>
#include <linux/i2c-mux.h>
#include <linux/version.h>

static const struct property_entry i2c_mux_props[] = {
	PROPERTY_ENTRY_BOOL("i2c-mux-idle-disconnect"),
	{}
};

static struct i2c_client *create_i2c_client(struct i2c_adapter *adapter,
		const char *type, int addr, const struct property_entry *props)
{
	struct i2c_client *client;
	struct i2c_board_info board_info;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
	struct software_node sw_node;
#endif
	int err;

	if (!adapter)
		return NULL;

	memset(&board_info, 0, sizeof(board_info));
	strscpy(board_info.type, type, I2C_NAME_SIZE);
	board_info.addr = addr;
	if (props) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
		memset(&sw_node, 0, sizeof(sw_node));
		sw_node.properties = props;
		board_info.swnode = &sw_node;
#else
		board_info.properties = props;
#endif
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	client = i2c_new_client_device(adapter, &board_info);
#else
	client = i2c_new_device(adapter, &board_info);
#endif

	if (!client)
		return NULL;

	// force driver load (mainly for muxes so we can talk to downstream devices)
	err = device_attach(&client->dev);
	if (err < 0)
		goto err_free_client;

	return client;

err_free_client:
	i2c_unregister_device(client);
	return NULL;
}

static struct i2c_adapter *get_i2c_mux_channel(struct i2c_client *mux, u32 chan_id)
{
	struct i2c_mux_core *muxc;

	if (!mux)
		return NULL;

	muxc = i2c_get_clientdata(mux);

	if (!muxc || chan_id >= muxc->num_adapters)
		return NULL;

	return muxc->adapter[chan_id];
}

static int init_mac_list_from_base_mac(struct nmac_dev *nmac, int count, char *mac)
{
	int k;

	count = min(count, NMAC_MAX_IF);

	if (!is_valid_ether_addr(mac)) {
		dev_warn(nmac->dev, "Base MAC is not valid");
		return -1;
	}

	nmac->mac_count = count;
	for (k = 0; k < nmac->mac_count; k++) {
		memcpy(nmac->mac_list[k], mac, ETH_ALEN);
		nmac->mac_list[k][ETH_ALEN - 1] += k;
	}

	return count;
}

static int read_mac_from_eeprom(struct nmac_dev *nmac,
		struct i2c_client *eeprom, int offset, char *mac)
{
	int ret;

	if (!eeprom) {
		dev_warn(nmac->dev, "Failed to read MAC from EEPROM; no EEPROM I2C client registered");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(eeprom, offset, ETH_ALEN, mac);
	if (ret < 0) {
		dev_warn(nmac->dev, "Failed to read MAC from EEPROM");
		return -1;
	}

	return 0;
}

static int read_mac_from_eeprom_hex(struct nmac_dev *nmac,
		struct i2c_client *eeprom, int offset, char *mac)
{
	int ret;
	char mac_hex[3*ETH_ALEN];

	if (!eeprom) {
		dev_warn(nmac->dev, "Failed to read MAC from EEPROM; no EEPROM I2C client registered");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(eeprom, offset, 3 * ETH_ALEN - 1, mac_hex);
	mac_hex[3*ETH_ALEN-1] = 0;
	if (ret < 0 || !mac_pton(mac_hex, mac)) {
		dev_warn(nmac->dev, "Failed to read MAC from EEPROM");
		return -1;
	}

	return 0;
}

static int init_mac_list_from_eeprom_base(struct nmac_dev *nmac,
		struct i2c_client *eeprom, int offset, int count)
{
	int ret;
	char mac[ETH_ALEN];

	ret = read_mac_from_eeprom(nmac, eeprom, offset, mac);
	if (ret < 0)
		return ret;

	if (!is_valid_ether_addr(mac)) {
		dev_warn(nmac->dev, "EEPROM does not contain a valid base MAC");
		return -1;
	}

	return init_mac_list_from_base_mac(nmac, count, mac);
}

static int init_mac_list_from_eeprom_base_hex(struct nmac_dev *nmac,
		struct i2c_client *eeprom, int offset, int count)
{
	int ret;
	char mac[ETH_ALEN];

	ret = read_mac_from_eeprom_hex(nmac, eeprom, offset, mac);
	if (ret < 0)
		return ret;

	if (!is_valid_ether_addr(mac)) {
		dev_warn(nmac->dev, "EEPROM does not contain a valid base MAC");
		return -1;
	}

	return init_mac_list_from_base_mac(nmac, count, mac);
}

static int nmac_generic_board_init(struct nmac_dev *nmac)
{
	struct i2c_adapter *adapter;
	struct i2c_client *mux;
	int ret = 0;

	nmac->mod_i2c_client_count = 0;

	if (nmac_i2c_init(nmac)) {
		dev_err(nmac->dev, "Failed to initialize I2C subsystem");
		return -1;
	}

	switch (nmac->board_id) {
	case NMAC_BOARD_ID_NETFPGA_SUME:
		// FPGA IC12
		//   TCA9548 IC31 0x74
		//     CH0: SFP1 IC3 0x50
		//     CH1: SFP2 IC5 0x50
		//     CH2: SFP3 IC6 0x50
		//     CH3: SFP4 IC8 0x50
		//     CH4: DDR3 IC27 0x51
		//          DDR3 IC28 0x52
		//          SI5324 IC20 0x68
		//     CH5: FMC
		//     CH6: PCON
		//     CH7: PMOD J11

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// IC31 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x74, i2c_mux_props);

		// IC3 SFP1
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 0), "24c02", 0x50, NULL);

		// IC5 SFP2
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 1), "24c02", 0x50, NULL);

		// IC6 SFP3
		nmac->mod_i2c_client[2] = create_i2c_client(get_i2c_mux_channel(mux, 2), "24c02", 0x50, NULL);

		// IC8 SFP4
		nmac->mod_i2c_client[3] = create_i2c_client(get_i2c_mux_channel(mux, 3), "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 4;

		break;
	case NMAC_BOARD_ID_VCU108:
		// FPGA U1
		//   TCA9548 U28 0x74
		//     CH0: SI570 Osc U32 0x5D
		//     CH1: TCA6416 Port Exp U89 0x21
		//     CH2: QSFP U145 0x50
		//     CH3: NC
		//     CH4: SI5328 U57 0x68
		//     CH5: HDMI U52 0x39
		//     CH6: SYSMON U1 0x32
		//     CH7: NC
		//   PCA9544 U80 0x75
		//     CH0: PMBUS
		//     CH1: FMC_HPC0 J22
		//     CH2: FMC_HPC1 J2
		//     CH3: M24C08 EEPROM U12 0x54

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U28 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x74, i2c_mux_props);

		// U145 QSFP
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 2), "24c02", 0x50, NULL);

		// U80 PCA9544 I2C MUX
		mux = create_i2c_client(adapter, "pca9544", 0x75, i2c_mux_props);

		// U12 I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(get_i2c_mux_channel(mux, 3), "24c08", 0x54, NULL);

		nmac->mod_i2c_client_count = 1;

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0x20, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_VCU118:
		// FPGA U1
		//   TCA9548 U28 0x74
		//     CH0: SI570 Osc U32 0x5D
		//     CH1: NC
		//     CH2: QSFP1 U145 0x50
		//     CH3: QSFP2 U123 0x50
		//     CH4: SI5328 U57 0x68
		//     CH5: SI570 Osc U18 0x5D
		//     CH6: SYSMON U1 0x32
		//     CH7: FIREFLY J6 0x50
		//   TCA9548 U80 0x75
		//     CH0: PMBUS
		//     CH1: FMCP_HSPC J22
		//     CH2: FMC_HPC1 J2
		//     CH3: M24C08 EEPROM U12 0x54
		//     CH4: INA_PMBUS
		//     CH5: SI570 Osc U38 0x5D
		//     CH6: NC
		//     CH7: NC

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U28 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x74, i2c_mux_props);

		// U145 QSFP1
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 2), "24c02", 0x50, NULL);

		// U123 QSFP2
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 3), "24c02", 0x50, NULL);

		// U80 PCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x75, i2c_mux_props);

		// U12 I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(get_i2c_mux_channel(mux, 3), "24c08", 0x54, NULL);

		nmac->mod_i2c_client_count = 2;

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0x20, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_VCU1525:
		// FPGA U13
		//   PCA9546 U28 0x74
		//     CH0: QSFP0 J7 0x50
		//     CH1: QSFP1 J9 0x50
		//     CH2: M24C08 EEPROM U62 0x54
		//          SI570 Osc U14 0x5D
		//     CH3: SYSMON U13 0x32

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U28 TCA9546 I2C MUX
		mux = create_i2c_client(adapter, "pca9546", 0x74, i2c_mux_props);

		// J7 QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 0), "24c02", 0x50, NULL);

		// J9 QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 1), "24c02", 0x50, NULL);

		// U12 I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(get_i2c_mux_channel(mux, 2), "24c08", 0x54, NULL);

		nmac->mod_i2c_client_count = 2;

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0x20, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_ZCU106:
		// FPGA U1 / MSP430 U41 I2C0
		//   TCA6416 U61 0x21
		//   TCA6416 U97 0x20
		//   PCA9544 U60 0x75
		//     CH0: PS_PMBUS
		//     CH1: PL_PMBUS
		//     CH2: MAXIM_PMBUS
		//     CH3: SYSMON U1 0x32
		// FPGA U1 / MSP430 U41 I2C1
		//   TCA9548 U34 0x74
		//     CH0: M24C08 EEPROM U23 0x54
		//     CH1: SI5341 U69 0x36
		//     CH2: SI570 Osc U42 0x5D
		//     CH3: SI570 Osc U56 0x5D
		//     CH4: SI5328 U20 0x68
		//     CH5: NC
		//     CH6: NC
		//     CH7: NC
		//   TCA9548 U135 0x75
		//     CH0: FMC_HPC0 J5
		//     CH1: FMC_HPC1 J4
		//     CH2: SYSMON U1 0x32
		//     CH3: DDR4 SODIMM 0x51
		//     CH4: NC
		//     CH5: NC
		//     CH6: SFP1 P2 0x50
		//     CH7: SFP0 P1 0x50

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U34 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x74, i2c_mux_props);

		// U23 I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(get_i2c_mux_channel(mux, 0), "24c08", 0x54, NULL);

		// U135 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x75, i2c_mux_props);

		// P1 SFP0
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 7), "24c02", 0x50, NULL);

		// P2 SFP1
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 6), "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0x20, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_DE10_AGILEX:

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// QSFP-DD A
		nmac->mod_i2c_client[0] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// QSFP-DD B
		nmac->mod_i2c_client[1] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		break;
	case NMAC_BOARD_ID_250SOC:
		// FPGA I2C
		//   TCA9548 U28 0x72
		//     CH0: J6 (OCuLink ch 0) A
		//     CH1: J6 (OCuLink ch 0) B
		//     CH2: J7 (OCuLink ch 1) A
		//     CH3: J8 (OCuLink ch 2) A
		//     CH4: J9 (OCuLink ch 3) A
		//     CH5: J9 (OCuLink ch 3) B
		//     CH6: QSFP0
		//     CH7: QSFP1
		// FPGA SMBUS
		//   AT24C16C U51 0x54
		//   TMP431C U52 0x4C

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U34 TCA9548 I2C MUX
		mux = create_i2c_client(adapter, "pca9548", 0x70, i2c_mux_props);

		// QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 6), "24c02", 0x50, NULL);

		// QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 7), "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 2);

		// I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(adapter, "24c16", 0x50, NULL);

		break;
	case NMAC_BOARD_ID_XUPP3R:

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 2);

		// QSFP2
		nmac->mod_i2c_client[2] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 3);

		// QSFP3
		nmac->mod_i2c_client[3] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 4;

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 4);

		// I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(adapter, "24c04", 0x50, NULL);

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base_hex(nmac, nmac->eeprom_i2c_client, 4, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_NEXUS_K35_S:
	case NMAC_BOARD_ID_NEXUS_K3P_S:
	case NMAC_BOARD_ID_ADM_PCIE_9V3:

		request_module("at24");

		// create I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_NEXUS_K3P_Q:

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 2);

		// I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// read MACs from EEPROM
		init_mac_list_from_eeprom_base(nmac, nmac->eeprom_i2c_client, 0, NMAC_MAX_IF);

		break;
	case NMAC_BOARD_ID_DNPCIE_40G_KU:

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 2);

		// I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(adapter, "24c256", 0x50, NULL);

		// read MACs from EEPROM
		// init_mac_list_from_eeprom(nmac, nmac->eeprom_i2c_client, 0x000E, NMAC_MAX_IF);

		break;
	default:
		dev_warn(nmac->dev, "Unknown board ID, not performing any board-specific init");
	}

	return ret;
}

static void nmac_generic_board_deinit(struct nmac_dev *nmac)
{
	int k;

	// unregister I2C clients
	for (k = 0; k < ARRAY_SIZE(nmac->mod_i2c_client); k++) {
		if (nmac->mod_i2c_client[k]) {
			i2c_unregister_device(nmac->mod_i2c_client[k]);
			nmac->mod_i2c_client[k] = NULL;
		}
	}

	if (nmac->eeprom_i2c_client) {
		i2c_unregister_device(nmac->eeprom_i2c_client);
		nmac->eeprom_i2c_client = NULL;
	}

	nmac_i2c_deinit(nmac);
}

static struct nmac_board_ops generic_board_ops = {
	.init = nmac_generic_board_init,
	.deinit = nmac_generic_board_deinit
};

static u32 nmac_alveo_bmc_reg_read(struct nmac_dev *nmac, struct nmac_reg_block *rb, u32 reg)
{
	iowrite32(reg, rb->regs + NMAC_RB_ALVEO_BMC_REG_ADDR);
	ioread32(rb->regs + NMAC_RB_ALVEO_BMC_REG_DATA); // dummy read
	return ioread32(rb->regs + NMAC_RB_ALVEO_BMC_REG_DATA);
}

static void nmac_alveo_bmc_reg_write(struct nmac_dev *nmac, struct nmac_reg_block *rb, u32 reg, u32 val)
{
	iowrite32(reg, rb->regs + NMAC_RB_ALVEO_BMC_REG_ADDR);
	iowrite32(val, rb->regs + NMAC_RB_ALVEO_BMC_REG_DATA);
	ioread32(rb->regs + NMAC_RB_ALVEO_BMC_REG_DATA); // dummy read
}

static int nmac_alveo_bmc_read_mac(struct nmac_dev *nmac, struct nmac_reg_block *rb, int index, char *mac)
{
	u32 reg = 0x0281a0 + index * 8;
	u32 val;

	val = nmac_alveo_bmc_reg_read(nmac, rb, reg);
	mac[0] = (val >> 8) & 0xff;
	mac[1] = val & 0xff;

	val = nmac_alveo_bmc_reg_read(nmac, rb, reg + 4);
	mac[2] = (val >> 24) & 0xff;
	mac[3] = (val >> 16) & 0xff;
	mac[4] = (val >> 8) & 0xff;
	mac[5] = val & 0xff;

	return 0;
}

static int nmac_alveo_bmc_read_mac_list(struct nmac_dev *nmac, struct nmac_reg_block *rb, int count)
{
	int ret, k;
	char mac[ETH_ALEN];

	count = min(count, NMAC_MAX_IF);

	nmac->mac_count = 0;
	for (k = 0; k < count; k++) {
		ret = nmac_alveo_bmc_read_mac(nmac, rb, k, mac);
		if (ret) {
			dev_warn(nmac->dev, "Failed to read MAC from Alveo BMC");
			return -1;
		}

		if (is_valid_ether_addr(mac)) {
			memcpy(nmac->mac_list[nmac->mac_count], mac, ETH_ALEN);
			nmac->mac_count++;
		}
	}

	dev_info(nmac->dev, "Read %d MACs from Alveo BMC", nmac->mac_count);

	if (nmac->mac_count == 0)
		dev_warn(nmac->dev, "Failed to read any valid MACs from Alveo BMC");

	return nmac->mac_count;
}

static int nmac_alveo_board_init(struct nmac_dev *nmac)
{
	struct i2c_adapter *adapter;
	struct i2c_client *mux;
	struct nmac_reg_block *rb;
	int ret = 0;

	nmac->mod_i2c_client_count = 0;

	if (nmac_i2c_init(nmac)) {
		dev_err(nmac->dev, "Failed to initialize I2C subsystem");
		return -1;
	}

	switch (nmac->board_id) {
	case NMAC_BOARD_ID_AU200:
	case NMAC_BOARD_ID_AU250:
		// FPGA U13
		//   PCA9546 U28 0x74
		//     CH0: QSFP0 J7 0x50
		//     CH1: QSFP1 J9 0x50
		//     CH2: M24C08 EEPROM U62 0x54
		//          SI570 Osc U14 0x5D
		//     CH3: SYSMON U13 0x32

		request_module("i2c_mux_pca954x");
		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// U28 TCA9546 I2C MUX
		mux = create_i2c_client(adapter, "pca9546", 0x74, i2c_mux_props);

		// J7 QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(get_i2c_mux_channel(mux, 0), "24c02", 0x50, NULL);

		// J9 QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(get_i2c_mux_channel(mux, 1), "24c02", 0x50, NULL);

		// U12 I2C EEPROM
		nmac->eeprom_i2c_client = create_i2c_client(get_i2c_mux_channel(mux, 2), "24c08", 0x54, NULL);

		nmac->mod_i2c_client_count = 2;

		break;
	case NMAC_BOARD_ID_AU50:
	case NMAC_BOARD_ID_AU280:
		// no I2C interfaces

		break;
	default:
		dev_warn(nmac->dev, "Unknown Alveo board ID");
	}

	// init BMC
	rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_ALVEO_BMC_TYPE, NMAC_RB_ALVEO_BMC_VER, 0);

	if (rb) {
		if (nmac_alveo_bmc_reg_read(nmac, rb, 0x020000) == 0 ||
		    nmac_alveo_bmc_reg_read(nmac, rb, 0x028000) != 0x74736574) {
			dev_info(nmac->dev, "Resetting Alveo CMS");

			nmac_alveo_bmc_reg_write(nmac, rb, 0x020000, 0);
			nmac_alveo_bmc_reg_write(nmac, rb, 0x020000, 1);
			msleep(200);
		}

		if (nmac_alveo_bmc_reg_read(nmac, rb, 0x028000) != 0x74736574)
			dev_warn(nmac->dev, "Alveo CMS not responding");
		else
			nmac_alveo_bmc_read_mac_list(nmac, rb, 8);
	} else {
		dev_warn(nmac->dev, "Alveo CMS not found");
	}

	return ret;
}

static struct nmac_board_ops alveo_board_ops = {
	.init = nmac_alveo_board_init,
	.deinit = nmac_generic_board_deinit
};

static int nmac_gecko_bmc_read(struct nmac_dev *nmac, struct nmac_reg_block *rb)
{
	u32 val;
	int timeout = 200;

	while (1) {
		val = ioread32(rb->regs + NMAC_RB_GECKO_BMC_REG_STATUS);
		if (val & BIT(19)) {
			if (val & BIT(18)) {
				// timed out
				dev_warn(nmac->dev, "Timed out waiting for Gecko BMC response");
				msleep(20);
				return -2;
			}
			return val & 0xffff;
		}

		timeout--;
		if (timeout == 0) {
			dev_warn(nmac->dev, "Timed out waiting for Gecko BMC interface");
			return -1;
		}
		usleep_range(1000, 100000);
	}

	return -1;
}

static int nmac_gecko_bmc_write(struct nmac_dev *nmac, struct nmac_reg_block *rb, u16 cmd, u32 data)
{
	int ret;

	ret = nmac_gecko_bmc_read(nmac, rb);

	if (ret == -1)
		return ret;

	iowrite32(data, rb->regs + NMAC_RB_GECKO_BMC_REG_DATA);
	iowrite32(cmd << 16, rb->regs + NMAC_RB_GECKO_BMC_REG_CMD);

	return 0;
}

static int nmac_gecko_bmc_query(struct nmac_dev *nmac, struct nmac_reg_block *rb, u16 cmd, u32 data)
{
	int ret;

	ret = nmac_gecko_bmc_write(nmac, rb, cmd, data);

	if (ret)
		return ret;

	return nmac_gecko_bmc_read(nmac, rb);
}

static int nmac_gecko_bmc_read_mac(struct nmac_dev *nmac, struct nmac_reg_block *rb, int index, char *mac)
{
	int i;
	u16 val;

	for (i = 0; i < ETH_ALEN; i += 2) {
		val = nmac_gecko_bmc_query(nmac, rb, 0x2003, 0 + index * ETH_ALEN + i);
		if (val < 0)
			return val;
		mac[i] = val & 0xff;
		mac[i + 1] = (val >> 8) & 0xff;
	}

	return 0;
}

static int nmac_gecko_bmc_read_mac_list(struct nmac_dev *nmac, struct nmac_reg_block *rb, int count)
{
	int ret, k;
	char mac[ETH_ALEN];

	count = min(count, NMAC_MAX_IF);

	nmac->mac_count = 0;
	for (k = 0; k < count; k++) {
		ret = nmac_gecko_bmc_read_mac(nmac, rb, k, mac);
		if (ret) {
			dev_warn(nmac->dev, "Failed to read MAC from Gecko BMC");
			return -1;
		}

		if (is_valid_ether_addr(mac)) {
			memcpy(nmac->mac_list[nmac->mac_count], mac, ETH_ALEN);
			nmac->mac_count++;
		}
	}

	dev_info(nmac->dev, "Read %d MACs from Gecko BMC", nmac->mac_count);

	if (nmac->mac_count == 0)
		dev_warn(nmac->dev, "Failed to read any valid MACs from Gecko BMC");

	return nmac->mac_count;
}

static int nmac_gecko_board_init(struct nmac_dev *nmac)
{
	struct i2c_adapter *adapter;
	struct nmac_reg_block *rb;
	int ret = 0;

	nmac->mod_i2c_client_count = 0;

	if (nmac_i2c_init(nmac)) {
		dev_err(nmac->dev, "Failed to initialize I2C subsystem");
		return -1;
	}

	switch (nmac->board_id) {
	case NMAC_BOARD_ID_FB2CG_KU15P:
		// FPGA U1 I2C0
		//     QSFP0 J3 0x50
		// FPGA U1 I2C1
		//     QSFP1 J4 0x50

		request_module("at24");

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 0);

		// QSFP0
		nmac->mod_i2c_client[0] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		// I2C adapter
		adapter = nmac_i2c_adapter_create(nmac, 1);

		// QSFP1
		nmac->mod_i2c_client[1] = create_i2c_client(adapter, "24c02", 0x50, NULL);

		nmac->mod_i2c_client_count = 2;

		break;
	default:
		dev_warn(nmac->dev, "Unknown board ID (Silicom Gecko BMC)");
	}

	// init BMC
	rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_GECKO_BMC_TYPE, NMAC_RB_GECKO_BMC_VER, 0);

	if (rb) {
		if (nmac_gecko_bmc_query(nmac, rb, 0x7006, 0) <= 0) {
			dev_warn(nmac->dev, "Gecko BMC not responding");
		} else {
			u16 v_l = nmac_gecko_bmc_query(nmac, rb, 0x7005, 0);
			u16 v_h = nmac_gecko_bmc_query(nmac, rb, 0x7006, 0);

			dev_info(nmac->dev, "Gecko BMC version %d.%d.%d.%d",
					(v_h >> 8) & 0xff, v_h & 0xff, (v_l >> 8) & 0xff, v_l & 0xff);

			nmac_gecko_bmc_read_mac_list(nmac, rb, 8);
		}
	} else {
		dev_warn(nmac->dev, "Gecko BMC not found");
	}

	return ret;
}

static struct nmac_board_ops gecko_board_ops = {
	.init = nmac_gecko_board_init,
	.deinit = nmac_generic_board_deinit
};

int nmac_board_init(struct nmac_dev *nmac)
{
	switch (nmac->board_id) {
	case NMAC_BOARD_ID_AU50:
	case NMAC_BOARD_ID_AU200:
	case NMAC_BOARD_ID_AU250:
	case NMAC_BOARD_ID_AU280:
		nmac->board_ops = &alveo_board_ops;
		break;
	case NMAC_BOARD_ID_FB2CG_KU15P:
		nmac->board_ops = &gecko_board_ops;
		break;
	default:
		nmac->board_ops = &generic_board_ops;
	}

	if (!nmac->board_ops)
		return -1;

	return nmac->board_ops->init(nmac);
}

void nmac_board_deinit(struct nmac_dev *nmac)
{
	if (!nmac->board_ops)
		return;

	nmac->board_ops->deinit(nmac);
	nmac->board_ops = NULL;
}
