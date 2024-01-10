#include "nmac.h"

int nmac_create_port(struct nmac_if *interface, struct nmac_port **port_ptr,
		int index, struct nmac_reg_block *port_rb)
{
	struct device *dev = interface->dev;
	struct nmac_port *port;
	struct nmac_reg_block *rb;
	u32 offset;
	int ret = 0;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	*port_ptr = port;

	port->dev = dev;
	port->interface = interface;

	port->index = index;

	port->port_rb = port_rb;

	offset = ioread32(port_rb->regs + NMAC_RB_SCHED_BLOCK_REG_OFFSET);

	port->rb_list = nmac_enumerate_reg_block_list(interface->hw_addr, offset, interface->hw_regs_size - offset);

	if (!port->rb_list) {
		ret = -EIO;
		dev_err(dev, "Failed to enumerate blocks");
		goto fail;
	}

	dev_info(dev, "Port-level register blocks:");
	for (rb = port->rb_list; rb->regs; rb++)
		dev_info(dev, " type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
				(rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

	port->port_ctrl_rb = nmac_find_reg_block(port->rb_list, NMAC_RB_PORT_CTRL_TYPE, NMAC_RB_PORT_CTRL_VER, 0);

	if (!port->port_ctrl_rb) {
		ret = -EIO;
		dev_err(dev, "Port control register block not found");
		goto fail;
	}

	port->port_features = ioread32(port->port_ctrl_rb->regs + NMAC_RB_PORT_CTRL_REG_FEATURES);

	dev_info(dev, "Port features: 0x%08x", port->port_features);

	dev_info(dev, "Port TX status: 0x%08x", nmac_port_get_tx_status(port));
	dev_info(dev, "Port RX status: 0x%08x", nmac_port_get_rx_status(port));

	return 0;

fail:
	nmac_destroy_port(port_ptr);
	return ret;
}

void nmac_destroy_port(struct nmac_port **port_ptr)
{
	struct nmac_port *port = *port_ptr;

	if (port->rb_list)
		nmac_free_reg_block_list(port->rb_list);

	*port_ptr = NULL;
	kfree(port);
}

u32 nmac_port_get_tx_status(struct nmac_port *port)
{
	return ioread32(port->port_ctrl_rb->regs + NMAC_RB_PORT_CTRL_REG_TX_STATUS);
}
EXPORT_SYMBOL(nmac_port_get_tx_status);

u32 nmac_port_get_rx_status(struct nmac_port *port)
{
	return ioread32(port->port_ctrl_rb->regs + NMAC_RB_PORT_CTRL_REG_RX_STATUS);
}
EXPORT_SYMBOL(nmac_port_get_rx_status);
