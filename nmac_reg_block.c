#include "nmac.h"

struct nmac_reg_block *nmac_enumerate_reg_block_list(u8 __iomem *base, size_t offset, size_t size)
{
	int max_count = 8;
	struct nmac_reg_block *reg_block_list = kzalloc(max_count * sizeof(struct nmac_reg_block), GFP_KERNEL);
	int count = 0;
	int k;

	u8 __iomem *ptr;

	u32 rb_type;
	u32 rb_version;

	if (!reg_block_list)
		return NULL;

	while (1) {
		reg_block_list[count].type = 0;
		reg_block_list[count].version = 0;
		reg_block_list[count].base = 0;
		reg_block_list[count].regs = 0;

		if ((offset == 0 && count != 0) || offset >= size)
			break;

		ptr = base + offset;

		for (k = 0; k < count; k++)
		{
			if (ptr == reg_block_list[k].regs)
			{
				pr_err("Register blocks form a loop");
				goto fail;
			}
		}

		rb_type = ioread32(ptr + NMAC_RB_REG_TYPE);
		rb_version = ioread32(ptr + NMAC_RB_REG_VER);
		offset = ioread32(ptr + NMAC_RB_REG_NEXT_PTR);

		reg_block_list[count].type = rb_type;
		reg_block_list[count].version = rb_version;
		reg_block_list[count].base = base;
		reg_block_list[count].regs = ptr;

		count++;

		if (count >= max_count) {
			struct nmac_reg_block *tmp;
			max_count += 4;
			tmp = krealloc(reg_block_list, max_count * sizeof(struct nmac_reg_block), GFP_KERNEL);
			if (!tmp)
				goto fail;
			reg_block_list = tmp;
		}
	}

	return reg_block_list;
fail:
	kfree(reg_block_list);
	return NULL;
}
EXPORT_SYMBOL(nmac_enumerate_reg_block_list);

struct nmac_reg_block *nmac_find_reg_block(struct nmac_reg_block *list, u32 type, u32 version, int index)
{
	struct nmac_reg_block *rb = list;

	while (rb->regs) {
		if (rb->type == type && (!version || rb->version == version)) {
			if (index > 0)
				index--;
			else
				return rb;
		}

		rb++;
	}

	return NULL;
}
EXPORT_SYMBOL(nmac_find_reg_block);

void nmac_free_reg_block_list(struct nmac_reg_block *list)
{
	kfree(list);
}
EXPORT_SYMBOL(nmac_free_reg_block_list);
