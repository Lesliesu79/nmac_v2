#include "nmac.h"

int nmac_create_sched_block(struct nmac_if *interface, struct nmac_sched_block **block_ptr,
		int index, struct nmac_reg_block *block_rb)
{
	struct device *dev = interface->dev;
	struct nmac_sched_block *block;
	struct nmac_reg_block *rb;
	u32 offset;
	int ret = 0;

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	*block_ptr = block;

	block->dev = dev;
	block->interface = interface;

	block->index = index;

	block->tx_queue_count = interface->tx_queue_count;

	block->block_rb = block_rb;

	offset = ioread32(block_rb->regs + NMAC_RB_SCHED_BLOCK_REG_OFFSET);

	block->rb_list = nmac_enumerate_reg_block_list(interface->hw_addr, offset, interface->hw_regs_size - offset);

	if (!block->rb_list) {
		ret = -EIO;
		dev_err(dev, "Failed to enumerate blocks");
		goto fail;
	}

	dev_info(dev, "Scheduler block-level register blocks:");
	for (rb = block->rb_list; rb->regs; rb++)
		dev_info(dev, " type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
				(rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

	block->sched_count = 0;
	for (rb = block->rb_list; rb->regs; rb++) {
		if (rb->type == NMAC_RB_SCHED_RR_TYPE && rb->version == NMAC_RB_SCHED_RR_VER) {
			ret = nmac_create_scheduler(block, &block->sched[block->sched_count],
					block->sched_count, rb);

			if (ret)
				goto fail;

			block->sched_count++;
		}
	}

	dev_info(dev, "Scheduler count: %d", block->sched_count);

	nmac_deactivate_sched_block(block);

	return 0;

fail:
	nmac_destroy_sched_block(block_ptr);
	return ret;
}

void nmac_destroy_sched_block(struct nmac_sched_block **block_ptr)
{
	struct nmac_sched_block *block = *block_ptr;
	int k;

	nmac_deactivate_sched_block(block);

	for (k = 0; k < ARRAY_SIZE(block->sched); k++)
		if (block->sched[k])
			nmac_destroy_scheduler(&block->sched[k]);

	if (block->rb_list)
		nmac_free_reg_block_list(block->rb_list);

	*block_ptr = NULL;
	kfree(block);
}

int nmac_activate_sched_block(struct nmac_sched_block *block)
{
	int k;

	// enable schedulers
	for (k = 0; k < ARRAY_SIZE(block->sched); k++)
		if (block->sched[k])
			nmac_scheduler_enable(block->sched[k]);

	return 0;
}
EXPORT_SYMBOL(nmac_activate_sched_block);

void nmac_deactivate_sched_block(struct nmac_sched_block *block)
{
	int k;

	// disable schedulers
	for (k = 0; k < ARRAY_SIZE(block->sched); k++)
		if (block->sched[k])
			nmac_scheduler_disable(block->sched[k]);
}
EXPORT_SYMBOL(nmac_deactivate_sched_block);
