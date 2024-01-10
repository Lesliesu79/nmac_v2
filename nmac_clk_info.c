#include "nmac.h"

void nmac_clk_info_init(struct nmac_dev *mdev)
{
	u32 val;

	mdev->clk_info_rb = nmac_find_reg_block(mdev->rb_list, NMAC_RB_CLK_INFO_TYPE, NMAC_RB_CLK_INFO_VER, 0);

	if (!mdev->clk_info_rb)
		return;

	val = ioread32(mdev->clk_info_rb->regs + NMAC_RB_CLK_INFO_REF_NOM_PER);

	mdev->ref_clk_nom_per_ns_num = val >> 16;
	mdev->ref_clk_nom_per_ns_denom = val & 0xffff;
	mdev->ref_clk_nom_freq_hz = (mdev->ref_clk_nom_per_ns_denom * 1000000000ull) / mdev->ref_clk_nom_per_ns_num;

	val = ioread32(mdev->clk_info_rb->regs + NMAC_RB_CLK_INFO_CLK_NOM_PER);

	mdev->core_clk_nom_per_ns_num = val >> 16;
	mdev->core_clk_nom_per_ns_denom = val & 0xffff;
	mdev->core_clk_nom_freq_hz = (mdev->core_clk_nom_per_ns_denom * 1000000000ull) / mdev->core_clk_nom_per_ns_num;

	mdev->clk_info_channels = ioread32(mdev->clk_info_rb->regs + NMAC_RB_CLK_INFO_COUNT);
}

u32 nmac_get_core_clk_nom_freq_hz(struct nmac_dev *mdev)
{
	return mdev->core_clk_nom_freq_hz;
}
EXPORT_SYMBOL(nmac_get_core_clk_nom_freq_hz);

u32 nmac_get_ref_clk_nom_freq_hz(struct nmac_dev *mdev)
{
	return mdev->ref_clk_nom_freq_hz;
}
EXPORT_SYMBOL(nmac_get_ref_clk_nom_freq_hz);

u32 nmac_get_core_clk_freq_hz(struct nmac_dev *mdev)
{
	if (!mdev->clk_info_rb)
		return 0;

	return ioread32(mdev->clk_info_rb->regs + NMAC_RB_CLK_INFO_CLK_FREQ);
}
EXPORT_SYMBOL(nmac_get_core_clk_freq_hz);

u32 nmac_get_clk_freq_hz(struct nmac_dev *mdev, int ch)
{
	if (!mdev->clk_info_rb || ch < 0 || ch >= mdev->clk_info_channels)
		return 0;

	return ioread32(mdev->clk_info_rb->regs + NMAC_RB_CLK_INFO_FREQ_BASE + ch*4);
}
EXPORT_SYMBOL(nmac_get_clk_freq_hz);

u64 nmac_core_clk_cycles_to_ns(struct nmac_dev *mdev, u64 cycles)
{
	if (!mdev->clk_info_rb || !mdev->core_clk_nom_per_ns_denom)
		return 0;

	return (cycles * (u64)mdev->core_clk_nom_per_ns_num) / (u64)mdev->core_clk_nom_per_ns_denom;
}
EXPORT_SYMBOL(nmac_core_clk_cycles_to_ns);

u64 nmac_core_clk_ns_to_cycles(struct nmac_dev *mdev, u64 ns)
{
	if (!mdev->clk_info_rb || !mdev->core_clk_nom_per_ns_num)
		return 0;

	return (ns * (u64)mdev->core_clk_nom_per_ns_denom) / (u64)mdev->core_clk_nom_per_ns_num;
}
EXPORT_SYMBOL(nmac_core_clk_ns_to_cycles);

u64 nmac_ref_clk_cycles_to_ns(struct nmac_dev *mdev, u64 cycles)
{
	if (!mdev->clk_info_rb || !mdev->ref_clk_nom_per_ns_denom)
		return 0;

	return (cycles * (u64)mdev->ref_clk_nom_per_ns_num) / (u64)mdev->ref_clk_nom_per_ns_denom;
}
EXPORT_SYMBOL(nmac_ref_clk_cycles_to_ns);

u64 nmac_ref_clk_ns_to_cycles(struct nmac_dev *mdev, u64 ns)
{
	if (!mdev->clk_info_rb || !mdev->ref_clk_nom_per_ns_num)
		return 0;

	return (ns * (u64)mdev->ref_clk_nom_per_ns_denom) / (u64)mdev->ref_clk_nom_per_ns_num;
}
EXPORT_SYMBOL(nmac_ref_clk_ns_to_cycles);
