#include "nmac.h"

static void nmac_i2c_set_scl(void *data, int state)
{
	struct nmac_i2c_bus *bus = data;

	if (state)
		iowrite32(ioread32(bus->scl_out_reg) | bus->scl_out_mask, bus->scl_out_reg);
	else
		iowrite32(ioread32(bus->scl_out_reg) & ~bus->scl_out_mask, bus->scl_out_reg);
}

static void nmac_i2c_set_sda(void *data, int state)
{
	struct nmac_i2c_bus *bus = data;

	if (state)
		iowrite32(ioread32(bus->sda_out_reg) | bus->sda_out_mask, bus->sda_out_reg);
	else
		iowrite32(ioread32(bus->sda_out_reg) & ~bus->sda_out_mask, bus->sda_out_reg);
}

static int nmac_i2c_get_scl(void *data)
{
	struct nmac_i2c_bus *bus = data;

	return !!(ioread32(bus->scl_in_reg) & bus->scl_in_mask);
}

static int nmac_i2c_get_sda(void *data)
{
	struct nmac_i2c_bus *bus = data;

	return !!(ioread32(bus->sda_in_reg) & bus->sda_in_mask);
}

struct nmac_i2c_bus *nmac_i2c_bus_create(struct nmac_dev *nmac, int index)
{
	struct nmac_i2c_bus *bus;
	struct i2c_algo_bit_data *algo;
	struct i2c_adapter *adapter;
	struct nmac_reg_block *rb;

	rb = nmac_find_reg_block(nmac->rb_list, NMAC_RB_I2C_TYPE, NMAC_RB_I2C_VER, index);

	if (!rb)
		return NULL;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);

	if (!bus)
		return NULL;

	// set private data
	bus->nmac = nmac;
	bus->scl_in_reg = rb->regs + NMAC_RB_I2C_REG_CTRL;
	bus->scl_out_reg = rb->regs + NMAC_RB_I2C_REG_CTRL;
	bus->sda_in_reg = rb->regs + NMAC_RB_I2C_REG_CTRL;
	bus->sda_out_reg = rb->regs + NMAC_RB_I2C_REG_CTRL;
	bus->scl_in_mask = NMAC_REG_GPIO_I2C_SCL_IN;
	bus->scl_out_mask = NMAC_REG_GPIO_I2C_SCL_OUT;
	bus->sda_in_mask = NMAC_REG_GPIO_I2C_SDA_IN;
	bus->sda_out_mask = NMAC_REG_GPIO_I2C_SDA_OUT;

	// bit-bang algorithm setup
	algo = &bus->algo;
	algo->udelay = 5;
	algo->timeout = usecs_to_jiffies(2000);
	algo->setsda = nmac_i2c_set_sda;
	algo->setscl = nmac_i2c_set_scl;
	algo->getsda = nmac_i2c_get_sda;
	algo->getscl = nmac_i2c_get_scl;
	algo->data = bus;

	// adapter setup
	adapter = &bus->adapter;
	adapter->owner = THIS_MODULE;
	adapter->algo_data = algo;
	adapter->dev.parent = nmac->dev;
	snprintf(adapter->name, sizeof(adapter->name), "%s I2C%d", nmac->name,
			nmac->i2c_adapter_count);

	if (i2c_bit_add_bus(adapter)) {
		dev_err(nmac->dev, "Failed to register I2C adapter");
		goto err_free_bus;
	}

	list_add_tail(&bus->head, &nmac->i2c_bus);

	nmac->i2c_adapter_count++;

	return bus;

err_free_bus:
	kfree(bus);
	return NULL;
}

struct i2c_adapter *nmac_i2c_adapter_create(struct nmac_dev *nmac, int index)
{
	struct nmac_i2c_bus *bus = nmac_i2c_bus_create(nmac, index);

	if (!bus)
		return NULL;

	return &bus->adapter;
}

void nmac_i2c_bus_release(struct nmac_i2c_bus *bus)
{
	struct nmac_dev *nmac;

	if (!bus)
		return;

	nmac = bus->nmac;

	nmac->i2c_adapter_count--;

	i2c_del_adapter(&bus->adapter);
	list_del(&bus->head);
	kfree(bus);
}

void nmac_i2c_adapter_release(struct i2c_adapter *adapter)
{
	struct nmac_i2c_bus *bus;

	if (!adapter)
		return;

	bus = container_of(adapter, struct nmac_i2c_bus, adapter);
	nmac_i2c_bus_release(bus);
}

int nmac_i2c_init(struct nmac_dev *nmac)
{
	INIT_LIST_HEAD(&nmac->i2c_bus);

	return 0;
}

void nmac_i2c_deinit(struct nmac_dev *nmac)
{
	struct nmac_i2c_bus *bus;

	while (!list_empty(&nmac->i2c_bus)) {
		bus = list_first_entry(&nmac->i2c_bus, typeof(*bus), head);
		nmac_i2c_bus_release(bus);
	}
}
