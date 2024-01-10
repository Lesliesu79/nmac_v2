#include "nmac.h"

int nmac_create_scheduler(struct nmac_sched_block *block, struct nmac_sched **sched_ptr,
		int index, struct nmac_reg_block *rb)
{
	struct device *dev = block->dev;
	struct nmac_sched *sched;

	sched = kzalloc(sizeof(*sched), GFP_KERNEL);
	if (!sched)
		return -ENOMEM;

	*sched_ptr = sched;

	sched->dev = dev;
	sched->interface = block->interface;
	sched->sched_block = block;

	sched->index = index;

	sched->rb = rb;

	sched->type = rb->type;
	sched->offset = ioread32(rb->regs + NMAC_RB_SCHED_RR_REG_OFFSET);
	sched->channel_count = ioread32(rb->regs + NMAC_RB_SCHED_RR_REG_CH_COUNT);
	sched->channel_stride = ioread32(rb->regs + NMAC_RB_SCHED_RR_REG_CH_STRIDE);

	sched->hw_addr = block->interface->hw_addr + sched->offset;

	dev_info(dev, "Scheduler type: 0x%08x", sched->type);
	dev_info(dev, "Scheduler offset: 0x%08x", sched->offset);
	dev_info(dev, "Scheduler channel count: %d", sched->channel_count);
	dev_info(dev, "Scheduler channel stride: 0x%08x", sched->channel_stride);

	nmac_scheduler_disable(sched);

	return 0;
}

void nmac_destroy_scheduler(struct nmac_sched **sched_ptr)
{
	struct nmac_sched *sched = *sched_ptr;
	*sched_ptr = NULL;

	nmac_scheduler_disable(sched);

	kfree(sched);
}

int nmac_scheduler_enable(struct nmac_sched *sched)
{
	int k;

	// enable scheduler
	iowrite32(1, sched->rb->regs + NMAC_RB_SCHED_RR_REG_CTRL);

	// enable queues
	for (k = 0; k < sched->channel_count; k++)
		iowrite32(3, sched->hw_addr + k * sched->channel_stride);

	return 0;
}
EXPORT_SYMBOL(nmac_scheduler_enable);

void nmac_scheduler_disable(struct nmac_sched *sched)
{
	// disable scheduler
	iowrite32(0, sched->rb->regs + NMAC_RB_SCHED_RR_REG_CTRL);
}
EXPORT_SYMBOL(nmac_scheduler_disable);
