#ifndef NMAC_IOCTL_H
#define NMAC_IOCTL_H

#include <linux/types.h>

#define NMAC_IOCTL_API_VERSION 0

#define NMAC_IOCTL_TYPE 0x88
#define NMAC_IOCTL_BASE 0xC0

enum {
	NMAC_REGION_TYPE_UNIMPLEMENTED = 0x00000000,
	NMAC_REGION_TYPE_CTRL = 0x00001000,
	NMAC_REGION_TYPE_NIC_CTRL = 0x00001001,
	NMAC_REGION_TYPE_APP_CTRL = 0x00001002,
	NMAC_REGION_TYPE_RAM = 0x00002000
};

// get API version
#define NMAC_IOCTL_GET_API_VERSION _IO(NMAC_IOCTL_TYPE, NMAC_IOCTL_BASE + 0)

// get device information
struct nmac_ioctl_device_info {
	__u32 argsz;
	__u32 flags;
	__u32 fw_id;
	__u32 fw_ver;
	__u32 board_id;
	__u32 board_ver;
	__u32 build_date;
	__u32 git_hash;
	__u32 rel_info;
	__u32 num_regions;
	__u32 num_irqs;
};

#define NMAC_IOCTL_GET_DEVICE_INFO _IO(NMAC_IOCTL_TYPE, NMAC_IOCTL_BASE + 1)

// get region information
struct nmac_ioctl_region_info {
	__u32 argsz;
	__u32 flags;
	__u32 index;
	__u32 type;
	__u32 next;
	__u32 child;
	__u64 size;
	__u64 offset;
	__u8 name[32];
};

#define NMAC_IOCTL_GET_REGION_INFO _IO(NMAC_IOCTL_TYPE, NMAC_IOCTL_BASE + 2)

#endif /* NMAC_IOCTL_H */
