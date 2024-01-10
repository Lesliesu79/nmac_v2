#include "nmac.h"

void nmac_stats_init(struct nmac_dev *mdev)
{
	mdev->stats_rb = nmac_find_reg_block(mdev->rb_list, NMAC_RB_STATS_TYPE, NMAC_RB_STATS_VER, 0);

	if (!mdev->stats_rb)
		return;

	mdev->stats_offset = ioread32(mdev->stats_rb->regs + NMAC_RB_STATS_REG_OFFSET);
	mdev->stats_count = ioread32(mdev->stats_rb->regs + NMAC_RB_STATS_REG_COUNT);
	mdev->stats_stride = ioread32(mdev->stats_rb->regs + NMAC_RB_STATS_REG_STRIDE);
	mdev->stats_flags = ioread32(mdev->stats_rb->regs + NMAC_RB_STATS_REG_FLAGS);
}

u64 nmac_stats_read(struct nmac_dev *mdev, int index)
{
	u64 val;

	if (!mdev->stats_rb || index < 0 || index >= mdev->stats_count)
		return 0;

	val = (u64)ioread32(mdev->hw_addr + mdev->stats_offset + index*8 + 0);
	val |= (u64)ioread32(mdev->hw_addr + mdev->stats_offset + index*8 + 4) << 32;

	return val;
}
EXPORT_SYMBOL(nmac_stats_read);
