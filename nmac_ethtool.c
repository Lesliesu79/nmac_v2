#include "nmac.h"

#include <linux/ethtool.h>
#include <linux/version.h>

#define SFF_MODULE_ID_SFP        0x03
#define SFF_MODULE_ID_QSFP       0x0c
#define SFF_MODULE_ID_QSFP_PLUS  0x0d
#define SFF_MODULE_ID_QSFP28     0x11

static void nmac_get_drvinfo(struct net_device *ndev,
		struct ethtool_drvinfo *drvinfo)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	struct nmac_dev *mdev = priv->mdev;

	strscpy(drvinfo->driver, DRIVER_NAME, sizeof(drvinfo->driver));
	strscpy(drvinfo->version, DRIVER_VERSION, sizeof(drvinfo->version));

	snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version), "%d.%d.%d.%d",
			mdev->fw_ver >> 24, (mdev->fw_ver >> 16) & 0xff,
			(mdev->fw_ver >> 8) & 0xff, mdev->fw_ver & 0xff);
	strscpy(drvinfo->bus_info, dev_name(mdev->dev), sizeof(drvinfo->bus_info));
}

static int nmac_get_ts_info(struct net_device *ndev,
		struct ethtool_ts_info *info)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	struct nmac_dev *mdev = priv->mdev;

	ethtool_op_get_ts_info(ndev, info);

	if (mdev->ptp_clock)
		info->phc_index = ptp_clock_index(mdev->ptp_clock);

	if (!(priv->if_features & NMAC_IF_FEATURE_PTP_TS) || !mdev->ptp_clock)
		return 0;

	info->so_timestamping |= SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);

	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_ALL);

	return 0;
}

static int nmac_read_module_eeprom(struct net_device *ndev,
		u16 offset, u16 len, u8 *data)
{
	struct nmac_priv *priv = netdev_priv(ndev);

	if (!priv->mod_i2c_client)
		return -EINVAL;

	if (len > I2C_SMBUS_BLOCK_MAX)
		len = I2C_SMBUS_BLOCK_MAX;

	return i2c_smbus_read_i2c_block_data(priv->mod_i2c_client, offset, len, data);
}

static int nmac_write_module_eeprom(struct net_device *ndev,
		u16 offset, u16 len, u8 *data)
{
	struct nmac_priv *priv = netdev_priv(ndev);

	if (!priv->mod_i2c_client)
		return -EINVAL;

	if (len > I2C_SMBUS_BLOCK_MAX)
		len = I2C_SMBUS_BLOCK_MAX;

	return i2c_smbus_write_i2c_block_data(priv->mod_i2c_client, offset, len, data);
}

static int nmac_query_module_id(struct net_device *ndev)
{
	int ret;
	u8 data;

	ret = nmac_read_module_eeprom(ndev, 0, 1, &data);

	if (ret < 0)
		return ret;

	return data;
}

static int nmac_query_module_eeprom_by_page(struct net_device *ndev,
		u8 i2c_addr, u16 page, u16 bank, u16 offset, u16 len, u8 *data)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	int module_id;
	u8 d;

	module_id = nmac_query_module_id(ndev);

	if (module_id < 0) {
		dev_err(priv->dev, "%s: Failed to read module ID (%d)", __func__, module_id);
		return module_id;
	}

	switch (module_id) {
	case SFF_MODULE_ID_SFP:
		if (page > 0 || bank > 0)
			return -EINVAL;
		break;
	case SFF_MODULE_ID_QSFP:
	case SFF_MODULE_ID_QSFP_PLUS:
	case SFF_MODULE_ID_QSFP28:
		if (page > 3 || bank > 0)
			return -EINVAL;
		break;
	default:
		dev_err(priv->dev, "%s: Unknown module ID (0x%x)", __func__, module_id);
		return -EINVAL;
	}

	if (i2c_addr != 0x50)
		return -EINVAL;

	// set page
	switch (module_id) {
	case SFF_MODULE_ID_SFP:
		break;
	case SFF_MODULE_ID_QSFP:
	case SFF_MODULE_ID_QSFP_PLUS:
	case SFF_MODULE_ID_QSFP28:
		if (offset+len >= 128) {
			// select page
			d = page;
			nmac_write_module_eeprom(ndev, 127, 1, &d);
			msleep(1);
		}
		break;
	default:
		dev_err(priv->dev, "%s: Unknown module ID (0x%x)", __func__, module_id);
		return -EINVAL;
	}

	// read data
	return nmac_read_module_eeprom(ndev, offset, len, data);
}

static int nmac_query_module_eeprom(struct net_device *ndev,
		u16 offset, u16 len, u8 *data)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	int module_id;
	u8 i2c_addr = 0x50;
	u16 page = 0;
	u16 bank = 0;

	module_id = nmac_query_module_id(ndev);

	if (module_id < 0) {
		dev_err(priv->dev, "%s: Failed to read module ID (%d)", __func__, module_id);
		return module_id;
	}

	switch (module_id) {
	case SFF_MODULE_ID_SFP:
		i2c_addr = 0x50;
		page = 0;
		if (offset > 256) {
			offset -= 256;
			i2c_addr = 0x51;
		}
		break;
	case SFF_MODULE_ID_QSFP:
	case SFF_MODULE_ID_QSFP_PLUS:
	case SFF_MODULE_ID_QSFP28:
		i2c_addr = 0x50;
		if (offset < 256) {
			page = 0;
		} else {
			page = 1 + ((offset - 256) / 128);
			offset -= page * 128;
		}
		break;
	default:
		dev_err(priv->dev, "%s: Unknown module ID (0x%x)", __func__, module_id);
		return -EINVAL;
	}

	// clip request to end of page
	if (offset + len > 256)
		len = 256 - offset;

	return nmac_query_module_eeprom_by_page(ndev, i2c_addr,
			page, bank, offset, len, data);
}

static int nmac_get_module_info(struct net_device *ndev,
		struct ethtool_modinfo *modinfo)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	int read_len = 0;
	u8 data[16];

	// read module ID and revision
	read_len = nmac_read_module_eeprom(ndev, 0, 2, data);

	if (read_len < 0)
		return read_len;

	if (read_len < 2)
		return -EIO;

	// check identifier byte at address 0
	switch (data[0]) {
	case SFF_MODULE_ID_SFP:
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
		break;
	case SFF_MODULE_ID_QSFP:
		modinfo->type = ETH_MODULE_SFF_8436;
		modinfo->eeprom_len = ETH_MODULE_SFF_8436_LEN;
		break;
	case SFF_MODULE_ID_QSFP_PLUS:
		// check revision at address 1
		if (data[1] >= 0x03) {
			modinfo->type = ETH_MODULE_SFF_8636;
			modinfo->eeprom_len = ETH_MODULE_SFF_8636_LEN;
		} else {
			modinfo->type = ETH_MODULE_SFF_8436;
			modinfo->eeprom_len = ETH_MODULE_SFF_8436_LEN;
		}
		break;
	case SFF_MODULE_ID_QSFP28:
		modinfo->type = ETH_MODULE_SFF_8636;
		modinfo->eeprom_len = ETH_MODULE_SFF_8636_LEN;
		break;
	default:
		dev_err(priv->dev, "%s: Unknown module ID (0x%x)", __func__, data[0]);
		return -EINVAL;
	}

	return 0;
}

static int nmac_get_module_eeprom(struct net_device *ndev,
		struct ethtool_eeprom *eeprom, u8 *data)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	int i = 0;
	int read_len;

	if (eeprom->len == 0)
		return -EINVAL;

	memset(data, 0, eeprom->len);

	while (i < eeprom->len) {
		read_len = nmac_query_module_eeprom(ndev, eeprom->offset + i,
				eeprom->len - i, data + i);

		if (read_len == 0)
			return 0;

		if (read_len < 0) {
			dev_err(priv->dev, "%s: Failed to read module EEPROM (%d)", __func__, read_len);
			return read_len;
		}

		i += read_len;
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
static int nmac_get_module_eeprom_by_page(struct net_device *ndev,
		const struct ethtool_module_eeprom *eeprom,
		struct netlink_ext_ack *extack)
{
	struct nmac_priv *priv = netdev_priv(ndev);
	int i = 0;
	int read_len;

	if (eeprom->length == 0)
		return -EINVAL;

	memset(eeprom->data, 0, eeprom->length);

	while (i < eeprom->length) {
		read_len = nmac_query_module_eeprom_by_page(ndev, eeprom->i2c_address,
				eeprom->page, eeprom->bank, eeprom->offset + i,
				eeprom->length - i, eeprom->data + i);

		if (read_len == 0)
			return 0;

		if (read_len < 0) {
			dev_err(priv->dev, "%s: Failed to read module EEPROM (%d)", __func__, read_len);
			return read_len;
		}

		i += read_len;
	}

	return i;
}
#endif

const struct ethtool_ops nmac_ethtool_ops = {
	.get_drvinfo = nmac_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_ts_info = nmac_get_ts_info,
	.get_module_info = nmac_get_module_info,
	.get_module_eeprom = nmac_get_module_eeprom,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
	.get_module_eeprom_by_page = nmac_get_module_eeprom_by_page,
#endif
};
