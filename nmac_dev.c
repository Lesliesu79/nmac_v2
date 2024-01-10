#include "nmac.h"
#include "nmac_ioctl.h"

#include <linux/uaccess.h>

static int nmac_open(struct inode *inode, struct file *file)
{
	// struct miscdevice *miscdev = file->private_data;
	// struct nmac_dev *nmac = container_of(miscdev, struct nmac_dev, misc_dev);

	return 0;
}

static int nmac_release(struct inode *inode, struct file *file)
{
	// struct miscdevice *miscdev = file->private_data;
	// struct nmac_dev *nmac = container_of(miscdev, struct nmac_dev, misc_dev);

	return 0;
}

static int nmac_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct miscdevice *miscdev = file->private_data;
	struct nmac_dev *nmac = container_of(miscdev, struct nmac_dev, misc_dev);
	int index;
	u64 pgoff, req_len, req_start;

	index = vma->vm_pgoff >> (40 - PAGE_SHIFT);
	req_len = vma->vm_end - vma->vm_start;
	pgoff = vma->vm_pgoff & ((1U << (40 - PAGE_SHIFT)) - 1);
	req_start = pgoff << PAGE_SHIFT;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	switch (index) {
	case 0:
		if (req_start + req_len > nmac->hw_regs_size)
			return -EINVAL;

		return io_remap_pfn_range(vma, vma->vm_start,
				(nmac->hw_regs_phys >> PAGE_SHIFT) + pgoff,
				req_len, pgprot_noncached(vma->vm_page_prot));
	case 1:
		if (req_start + req_len > nmac->app_hw_regs_size)
			return -EINVAL;

		return io_remap_pfn_range(vma, vma->vm_start,
				(nmac->app_hw_regs_phys >> PAGE_SHIFT) + pgoff,
				req_len, pgprot_noncached(vma->vm_page_prot));
	case 2:
		if (req_start + req_len > nmac->ram_hw_regs_size)
			return -EINVAL;

		return io_remap_pfn_range(vma, vma->vm_start,
				(nmac->ram_hw_regs_phys >> PAGE_SHIFT) + pgoff,
				req_len, pgprot_noncached(vma->vm_page_prot));
	default:
		dev_err(nmac->dev, "%s: Tried to map an unknown region at page offset 0x%lx",
				__func__, vma->vm_pgoff);
		return -EINVAL;
	}

	return -EINVAL;
}

static long nmac_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct nmac_dev *nmac = container_of(miscdev, struct nmac_dev, misc_dev);
	size_t minsz;

	if (cmd == NMAC_IOCTL_GET_API_VERSION) {
		// Get API version
		return NMAC_IOCTL_API_VERSION;
	} else if (cmd == NMAC_IOCTL_GET_DEVICE_INFO) {
		// Get device information
		struct nmac_ioctl_device_info info;

		minsz = offsetofend(struct nmac_ioctl_device_info, num_irqs);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		info.flags = 0;
		info.fw_id = nmac->fw_id;
		info.fw_ver = nmac->fw_ver;
		info.board_id = nmac->board_id;
		info.board_ver = nmac->board_ver;
		info.build_date = nmac->build_date;
		info.git_hash = nmac->git_hash;
		info.rel_info = nmac->rel_info;
		info.num_regions = 3;
		info.num_irqs = 0;

		return copy_to_user((void __user *)arg, &info, minsz) ? -EFAULT : 0;

	} else if (cmd == NMAC_IOCTL_GET_REGION_INFO) {
		// Get region information
		struct nmac_ioctl_region_info info;

		minsz = offsetofend(struct nmac_ioctl_region_info, name);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		info.flags = 0;
		info.type = NMAC_REGION_TYPE_UNIMPLEMENTED;
		info.next = 0;
		info.child = 0;
		info.size = 0;
		info.offset = ((u64)info.index) << 40;
		info.name[0] = 0;

		switch (info.index) {
		case 0:
			info.type = NMAC_REGION_TYPE_NIC_CTRL;
			info.next = 1;
			info.child = 0;
			info.size = nmac->hw_regs_size;
			info.offset = ((u64)info.index) << 40;
			strlcpy(info.name, "ctrl", sizeof(info.name));
			break;
		case 1:
			info.type = NMAC_REGION_TYPE_APP_CTRL;
			info.next = 2;
			info.child = 0;
			info.size = nmac->app_hw_regs_size;
			info.offset = ((u64)info.index) << 40;
			strlcpy(info.name, "app", sizeof(info.name));
			break;
		case 2:
			info.type = NMAC_REGION_TYPE_RAM;
			info.next = 3;
			info.child = 0;
			info.size = nmac->ram_hw_regs_size;
			info.offset = ((u64)info.index) << 40;
			strlcpy(info.name, "ram", sizeof(info.name));
			break;
		default:
			return -EINVAL;
		}

		return copy_to_user((void __user *)arg, &info, minsz) ? -EFAULT : 0;

	}

	return -EINVAL;
}

const struct file_operations nmac_fops = {
	.owner = THIS_MODULE,
	.open = nmac_open,
	.release = nmac_release,
	.mmap = nmac_mmap,
	.unlocked_ioctl = nmac_ioctl,
};
