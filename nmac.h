#ifndef NMAC_H
#define NMAC_H

#include <linux/kernel.h>
#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif
#ifdef CONFIG_AUXILIARY_BUS
#include <linux/auxiliary_bus.h>
#endif
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timer.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#define DRIVER_NAME "nmac"
#define DRIVER_VERSION "0.1"

#include "nmac_hw.h"

#ifdef CONFIG_OF
/* platform driver OF-related definitions */
#define NMAC_PROP_MAC_ADDR_INC_BYTE "mac-address-increment-byte"
#define NMAC_PROP_MAC_ADDR_INC "mac-address-increment"
#define NMAC_PROP_MAC_ADDR_LOCAL "mac-address-local"
#define NMAC_PROP_MODULE_EEPROM "module-eeproms"
#endif

// default interval to poll port TX/RX status, in ms
#define NMAC_LINK_STATUS_POLL_MS 1000

extern unsigned int nmac_num_ev_queue_entries;
extern unsigned int nmac_num_tx_queue_entries;
extern unsigned int nmac_num_rx_queue_entries;

extern unsigned int nmac_link_status_poll;

struct nmac_dev;
struct nmac_if;

struct nmac_reg_block {
	u32 type;
	u32 version;
	u8 __iomem *regs;
	u8 __iomem *base;
};

struct nmac_board_ops {
	int (*init)(struct nmac_dev *nmac);
	void (*deinit)(struct nmac_dev *nmac);
};

struct nmac_i2c_bus {
	struct nmac_dev *nmac;

	u8 __iomem *scl_in_reg;
	u8 __iomem *scl_out_reg;
	u8 __iomem *sda_in_reg;
	u8 __iomem *sda_out_reg;

	u32 scl_in_mask;
	u32 scl_out_mask;
	u32 sda_in_mask;
	u32 sda_out_mask;

	struct list_head head;

	struct i2c_algo_bit_data algo;
	struct i2c_adapter adapter;
};

struct nmac_irq {
	int index;
	int irqn;
	char name[16 + 3];
	struct atomic_notifier_head nh;
};

#ifdef CONFIG_AUXILIARY_BUS
struct nmac_adev {
	struct auxiliary_device adev;
	struct nmac_dev *mdev;
	struct nmac_adev **ptr;
	char name[32];
};
#endif

struct nmac_dev {
	struct device *dev;
#ifdef CONFIG_PCI
	struct pci_dev *pdev;
#endif
	struct platform_device *pfdev;

	resource_size_t hw_regs_size;
	phys_addr_t hw_regs_phys;
	u8 __iomem *hw_addr;
	u8 __iomem *phc_hw_addr;

	resource_size_t app_hw_regs_size;
	phys_addr_t app_hw_regs_phys;
	u8 __iomem *app_hw_addr;

	resource_size_t ram_hw_regs_size;
	phys_addr_t ram_hw_regs_phys;
	u8 __iomem *ram_hw_addr;

	struct mutex state_lock;

	int mac_count;
	u8 mac_list[NMAC_MAX_IF][ETH_ALEN];

	char name[16];

	int irq_count;
	struct nmac_irq *irq[NMAC_MAX_IRQ];

	unsigned int id;
	struct list_head dev_list_node;

	struct miscdevice misc_dev;

#ifdef CONFIG_AUXILIARY_BUS
	struct nmac_adev *app_adev;
#endif

	struct nmac_reg_block *rb_list;
	struct nmac_reg_block *fw_id_rb;
	struct nmac_reg_block *if_rb;
	struct nmac_reg_block *stats_rb;
	struct nmac_reg_block *clk_info_rb;
	struct nmac_reg_block *phc_rb;

	int dev_port_max;
	int dev_port_limit;

	u32 fpga_id;
	u32 fw_id;
	u32 fw_ver;
	u32 board_id;
	u32 board_ver;
	u32 build_date;
	u32 git_hash;
	u32 rel_info;

	u32 app_id;

	u32 stats_offset;
	u32 stats_count;
	u32 stats_stride;
	u32 stats_flags;

	u32 core_clk_nom_per_ns_num;
	u32 core_clk_nom_per_ns_denom;
	u32 core_clk_nom_freq_hz;
	u32 ref_clk_nom_per_ns_num;
	u32 ref_clk_nom_per_ns_denom;
	u32 ref_clk_nom_freq_hz;
	u32 clk_info_channels;

	u32 if_offset;
	u32 if_count;
	u32 if_stride;
	u32 if_csr_offset;

	struct nmac_if *interface[NMAC_MAX_IF];

	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_info;

	struct nmac_board_ops *board_ops;

	struct list_head i2c_bus;
	int i2c_adapter_count;

	int mod_i2c_client_count;
	struct i2c_client *mod_i2c_client[NMAC_MAX_IF];
	struct i2c_client *eeprom_i2c_client;
};

struct nmac_frag {
	dma_addr_t dma_addr;
	u32 len;
};

struct nmac_tx_info {
	struct sk_buff *skb;
	DEFINE_DMA_UNMAP_ADDR(dma_addr);
	DEFINE_DMA_UNMAP_LEN(len);
	u32 frag_count;
	struct nmac_frag frags[NMAC_MAX_FRAGS - 1];
	int ts_requested;
};

struct nmac_rx_info {
	struct page *page;
	u32 page_order;
	u32 page_offset;
	dma_addr_t dma_addr;
	u32 len;
};

struct nmac_ring {
	// written on enqueue (i.e. start_xmit)
	u32 head_ptr;
	u64 bytes;
	u64 packets;
	u64 dropped_packets;
	struct netdev_queue *tx_queue;

	// written from completion
	u32 tail_ptr ____cacheline_aligned_in_smp;
	u32 clean_tail_ptr;
	u64 ts_s;
	u8 ts_valid;

	// mostly constant
	u32 size;
	u32 full_size;
	u32 size_mask;
	u32 stride;

	u32 cpl_index;

	u32 mtu;
	u32 page_order;

	u32 desc_block_size;
	u32 log_desc_block_size;

	size_t buf_size;
	u8 *buf;
	dma_addr_t buf_dma_addr;

	union {
		struct nmac_tx_info *tx_info;
		struct nmac_rx_info *rx_info;
	};

	struct device *dev;
	struct nmac_if *interface;
	struct nmac_priv *priv;
	int index;
	struct nmac_cq_ring *cq_ring;
	int active;

	u32 hw_ptr_mask;
	u8 __iomem *hw_addr;
	u8 __iomem *hw_head_ptr;
	u8 __iomem *hw_tail_ptr;
} ____cacheline_aligned_in_smp;

struct nmac_cq_ring {
	u32 head_ptr;

	u32 tail_ptr;

	u32 size;
	u32 size_mask;
	u32 stride;

	size_t buf_size;
	u8 *buf;
	dma_addr_t buf_dma_addr;

	struct device *dev;
	struct nmac_if *interface;
	struct napi_struct napi;
	int index;
	struct nmac_eq_ring *eq_ring;
	struct nmac_ring *src_ring;
	int eq_index;
	int active;

	void (*handler)(struct nmac_cq_ring *ring);

	u32 hw_ptr_mask;
	u8 __iomem *hw_addr;
	u8 __iomem *hw_head_ptr;
	u8 __iomem *hw_tail_ptr;
};

struct nmac_eq_ring {
	u32 head_ptr;

	u32 tail_ptr;

	u32 size;
	u32 size_mask;
	u32 stride;

	size_t buf_size;
	u8 *buf;
	dma_addr_t buf_dma_addr;

	struct device *dev;
	struct nmac_if *interface;
	int index;
	struct nmac_irq *irq;
	int irq_index;
	int active;

	struct notifier_block irq_nb;

	void (*handler)(struct nmac_eq_ring *ring);

	u32 hw_ptr_mask;
	u8 __iomem *hw_addr;
	u8 __iomem *hw_head_ptr;
	u8 __iomem *hw_tail_ptr;
};

struct nmac_sched {
	struct device *dev;
	struct nmac_if *interface;
	struct nmac_sched_block *sched_block;

	struct nmac_reg_block *rb;

	int index;

	u32 type;
	u32 offset;
	u32 channel_count;
	u32 channel_stride;

	u8 __iomem *hw_addr;
};

struct nmac_port {
	struct device *dev;
	struct nmac_if *interface;

	struct nmac_reg_block *port_rb;
	struct nmac_reg_block *rb_list;
	struct nmac_reg_block *port_ctrl_rb;

	int index;

	u32 port_features;
};

struct nmac_sched_block {
	struct device *dev;
	struct nmac_if *interface;

	struct nmac_reg_block *block_rb;
	struct nmac_reg_block *rb_list;

	int index;

	u32 tx_queue_count;

	u32 sched_count;
	struct nmac_sched *sched[NMAC_MAX_PORTS];
};

struct nmac_if {
	struct device *dev;
	struct nmac_dev *mdev;

	struct nmac_reg_block *rb_list;
	struct nmac_reg_block *if_ctrl_rb;
	struct nmac_reg_block *event_queue_rb;
	struct nmac_reg_block *tx_queue_rb;
	struct nmac_reg_block *tx_cpl_queue_rb;
	struct nmac_reg_block *rx_queue_rb;
	struct nmac_reg_block *rx_cpl_queue_rb;
	struct nmac_reg_block *rx_queue_map_rb;

	int index;

	int dev_port_base;
	int dev_port_max;
	int dev_port_limit;

	u32 if_features;

	u32 max_tx_mtu;
	u32 max_rx_mtu;

	u32 event_queue_offset;
	u32 event_queue_count;
	u32 event_queue_stride;
	struct nmac_eq_ring *event_ring[NMAC_MAX_EVENT_RINGS];

	u32 tx_queue_offset;
	u32 tx_queue_count;
	u32 tx_queue_stride;
	struct nmac_ring *tx_ring[NMAC_MAX_TX_RINGS];

	u32 tx_cpl_queue_offset;
	u32 tx_cpl_queue_count;
	u32 tx_cpl_queue_stride;
	struct nmac_cq_ring *tx_cpl_ring[NMAC_MAX_TX_CPL_RINGS];

	u32 rx_queue_offset;
	u32 rx_queue_count;
	u32 rx_queue_stride;
	struct nmac_ring *rx_ring[NMAC_MAX_RX_RINGS];

	u32 rx_cpl_queue_offset;
	u32 rx_cpl_queue_count;
	u32 rx_cpl_queue_stride;
	struct nmac_cq_ring *rx_cpl_ring[NMAC_MAX_RX_CPL_RINGS];

	u32 port_count;
	struct nmac_port *port[NMAC_MAX_PORTS];

	u32 sched_block_count;
	struct nmac_sched_block *sched_block[NMAC_MAX_PORTS];

	u32 max_desc_block_size;

	resource_size_t hw_regs_size;
	u8 __iomem *hw_addr;
	u8 __iomem *csr_hw_addr;

	u32 ndev_count;
	struct net_device *ndev[NMAC_MAX_PORTS];

	struct i2c_client *mod_i2c_client;
};

struct nmac_priv {
	struct device *dev;
	struct net_device *ndev;
	struct nmac_dev *mdev;
	struct nmac_if *interface;

	spinlock_t stats_lock;

	int index;
	bool registered;
	bool port_up;

	u32 if_features;

	unsigned int link_status;
	struct timer_list link_status_timer;

	u32 event_queue_count;
	struct nmac_eq_ring *event_ring[NMAC_MAX_EVENT_RINGS];

	u32 tx_queue_count;
	struct nmac_ring *tx_ring[NMAC_MAX_TX_RINGS];

	u32 tx_cpl_queue_count;
	struct nmac_cq_ring *tx_cpl_ring[NMAC_MAX_TX_CPL_RINGS];

	u32 rx_queue_count;
	struct nmac_ring *rx_ring[NMAC_MAX_RX_RINGS];

	u32 rx_cpl_queue_count;
	struct nmac_cq_ring *rx_cpl_ring[NMAC_MAX_RX_CPL_RINGS];

	u32 sched_block_count;
	struct nmac_sched_block *sched_block[NMAC_MAX_PORTS];

	u32 max_desc_block_size;

	struct hwtstamp_config hwts_config;

	struct i2c_client *mod_i2c_client;
};

// nmac_main.c

// nmac_reg_block.c
struct nmac_reg_block *nmac_enumerate_reg_block_list(u8 __iomem *base, size_t offset, size_t size);
struct nmac_reg_block *nmac_find_reg_block(struct nmac_reg_block *list, u32 type, u32 version, int index);
void nmac_free_reg_block_list(struct nmac_reg_block *list);

// nmac_irq.c
int nmac_irq_init_pcie(struct nmac_dev *mdev);
void nmac_irq_deinit_pcie(struct nmac_dev *mdev);
int nmac_irq_init_platform(struct nmac_dev *mdev);

// nmac_dev.c
extern const struct file_operations nmac_fops;

// nmac_if.c
int nmac_create_interface(struct nmac_dev *mdev, struct nmac_if **interface_ptr,
		int index, u8 __iomem *hw_addr);
void nmac_destroy_interface(struct nmac_if **interface_ptr);
u32 nmac_interface_get_tx_mtu(struct nmac_if *interface);
void nmac_interface_set_tx_mtu(struct nmac_if *interface, u32 mtu);
u32 nmac_interface_get_rx_mtu(struct nmac_if *interface);
void nmac_interface_set_rx_mtu(struct nmac_if *interface, u32 mtu);
u32 nmac_interface_get_rx_queue_map_offset(struct nmac_if *interface, int port);
void nmac_interface_set_rx_queue_map_offset(struct nmac_if *interface, int port, u32 val);
u32 nmac_interface_get_rx_queue_map_rss_mask(struct nmac_if *interface, int port);
void nmac_interface_set_rx_queue_map_rss_mask(struct nmac_if *interface, int port, u32 val);
u32 nmac_interface_get_rx_queue_map_app_mask(struct nmac_if *interface, int port);
void nmac_interface_set_rx_queue_map_app_mask(struct nmac_if *interface, int port, u32 val);

// nmac_port.c
int nmac_create_port(struct nmac_if *interface, struct nmac_port **port_ptr,
		int index, struct nmac_reg_block *port_rb);
void nmac_destroy_port(struct nmac_port **port_ptr);
u32 nmac_port_get_tx_status(struct nmac_port *port);
u32 nmac_port_get_rx_status(struct nmac_port *port);

// nmac_netdev.c
void nmac_update_stats(struct net_device *ndev);
int nmac_create_netdev(struct nmac_if *interface, struct net_device **ndev_ptr,
		int index, int dev_port);
void nmac_destroy_netdev(struct net_device **ndev_ptr);

// nmac_sched_block.c
int nmac_create_sched_block(struct nmac_if *interface, struct nmac_sched_block **block_ptr,
		int index, struct nmac_reg_block *rb);
void nmac_destroy_sched_block(struct nmac_sched_block **block_ptr);
int nmac_activate_sched_block(struct nmac_sched_block *block);
void nmac_deactivate_sched_block(struct nmac_sched_block *block);

// nmac_scheduler.c
int nmac_create_scheduler(struct nmac_sched_block *block, struct nmac_sched **sched_ptr,
		int index, struct nmac_reg_block *rb);
void nmac_destroy_scheduler(struct nmac_sched **sched_ptr);
int nmac_scheduler_enable(struct nmac_sched *sched);
void nmac_scheduler_disable(struct nmac_sched *sched);

// nmac_ptp.c
void nmac_register_phc(struct nmac_dev *mdev);
void nmac_unregister_phc(struct nmac_dev *mdev);
ktime_t nmac_read_cpl_ts(struct nmac_dev *mdev, struct nmac_ring *ring,
		const struct nmac_cpl *cpl);

// nmac_i2c.c
struct nmac_i2c_bus *nmac_i2c_bus_create(struct nmac_dev *nmac, int index);
struct i2c_adapter *nmac_i2c_adapter_create(struct nmac_dev *nmac, int index);
void nmac_i2c_bus_release(struct nmac_i2c_bus *bus);
void nmac_i2c_adapter_release(struct i2c_adapter *adapter);
int nmac_i2c_init(struct nmac_dev *nmac);
void nmac_i2c_deinit(struct nmac_dev *nmac);

// nmac_board.c
int nmac_board_init(struct nmac_dev *nmac);
void nmac_board_deinit(struct nmac_dev *nmac);

// nmac_clk_info.c
void nmac_clk_info_init(struct nmac_dev *mdev);
u32 nmac_get_core_clk_nom_freq_hz(struct nmac_dev *mdev);
u32 nmac_get_ref_clk_nom_freq_hz(struct nmac_dev *mdev);
u32 nmac_get_core_clk_freq_hz(struct nmac_dev *mdev);
u32 nmac_get_clk_freq_hz(struct nmac_dev *mdev, int ch);
u64 nmac_core_clk_cycles_to_ns(struct nmac_dev *mdev, u64 cycles);
u64 nmac_core_clk_ns_to_cycles(struct nmac_dev *mdev, u64 ns);
u64 nmac_ref_clk_cycles_to_ns(struct nmac_dev *mdev, u64 cycles);
u64 nmac_ref_clk_ns_to_cycles(struct nmac_dev *mdev, u64 ns);

// nmac_stats.c
void nmac_stats_init(struct nmac_dev *mdev);
u64 nmac_stats_read(struct nmac_dev *mdev, int index);

// nmac_eq.c
int nmac_create_eq_ring(struct nmac_if *interface, struct nmac_eq_ring **ring_ptr,
		int index, u8 __iomem *hw_addr);
void nmac_destroy_eq_ring(struct nmac_eq_ring **ring_ptr);
int nmac_alloc_eq_ring(struct nmac_eq_ring *ring, int size, int stride);
void nmac_free_eq_ring(struct nmac_eq_ring *ring);
int nmac_activate_eq_ring(struct nmac_eq_ring *ring, struct nmac_irq *irq);
void nmac_deactivate_eq_ring(struct nmac_eq_ring *ring);
bool nmac_is_eq_ring_empty(const struct nmac_eq_ring *ring);
bool nmac_is_eq_ring_full(const struct nmac_eq_ring *ring);
void nmac_eq_read_head_ptr(struct nmac_eq_ring *ring);
void nmac_eq_write_tail_ptr(struct nmac_eq_ring *ring);
void nmac_arm_eq(struct nmac_eq_ring *ring);
void nmac_process_eq(struct nmac_eq_ring *eq_ring);

// nmac_cq.c
int nmac_create_cq_ring(struct nmac_if *interface, struct nmac_cq_ring **ring_ptr,
		int index, u8 __iomem *hw_addr);
void nmac_destroy_cq_ring(struct nmac_cq_ring **ring_ptr);
int nmac_alloc_cq_ring(struct nmac_cq_ring *ring, int size, int stride);
void nmac_free_cq_ring(struct nmac_cq_ring *ring);
int nmac_activate_cq_ring(struct nmac_cq_ring *ring, struct nmac_eq_ring *eq_ring);
void nmac_deactivate_cq_ring(struct nmac_cq_ring *ring);
bool nmac_is_cq_ring_empty(const struct nmac_cq_ring *ring);
bool nmac_is_cq_ring_full(const struct nmac_cq_ring *ring);
void nmac_cq_read_head_ptr(struct nmac_cq_ring *ring);
void nmac_cq_write_tail_ptr(struct nmac_cq_ring *ring);
void nmac_arm_cq(struct nmac_cq_ring *ring);

// nmac_tx.c
int nmac_create_tx_ring(struct nmac_if *interface, struct nmac_ring **ring_ptr,
		int index, u8 __iomem *hw_addr);
void nmac_destroy_tx_ring(struct nmac_ring **ring_ptr);
int nmac_alloc_tx_ring(struct nmac_ring *ring, int size, int stride);
void nmac_free_tx_ring(struct nmac_ring *ring);
int nmac_activate_tx_ring(struct nmac_ring *ring, struct nmac_priv *priv,
		struct nmac_cq_ring *cq_ring);
void nmac_deactivate_tx_ring(struct nmac_ring *ring);
bool nmac_is_tx_ring_empty(const struct nmac_ring *ring);
bool nmac_is_tx_ring_full(const struct nmac_ring *ring);
void nmac_tx_read_tail_ptr(struct nmac_ring *ring);
void nmac_tx_write_head_ptr(struct nmac_ring *ring);
void nmac_free_tx_desc(struct nmac_ring *ring, int index, int napi_budget);
int nmac_free_tx_buf(struct nmac_ring *ring);
int nmac_process_tx_cq(struct nmac_cq_ring *cq_ring, int napi_budget);
void nmac_tx_irq(struct nmac_cq_ring *cq);
int nmac_poll_tx_cq(struct napi_struct *napi, int budget);
netdev_tx_t nmac_start_xmit(struct sk_buff *skb, struct net_device *dev);

// nmac_rx.c
int nmac_create_rx_ring(struct nmac_if *interface, struct nmac_ring **ring_ptr,
		int index, u8 __iomem *hw_addr);
void nmac_destroy_rx_ring(struct nmac_ring **ring_ptr);
int nmac_alloc_rx_ring(struct nmac_ring *ring, int size, int stride);
void nmac_free_rx_ring(struct nmac_ring *ring);
int nmac_activate_rx_ring(struct nmac_ring *ring, struct nmac_priv *priv,
		struct nmac_cq_ring *cq_ring);
void nmac_deactivate_rx_ring(struct nmac_ring *ring);
bool nmac_is_rx_ring_empty(const struct nmac_ring *ring);
bool nmac_is_rx_ring_full(const struct nmac_ring *ring);
void nmac_rx_read_tail_ptr(struct nmac_ring *ring);
void nmac_rx_write_head_ptr(struct nmac_ring *ring);
void nmac_free_rx_desc(struct nmac_ring *ring, int index);
int nmac_free_rx_buf(struct nmac_ring *ring);
int nmac_prepare_rx_desc(struct nmac_ring *ring, int index);
void nmac_refill_rx_buffers(struct nmac_ring *ring);
int nmac_process_rx_cq(struct nmac_cq_ring *cq_ring, int napi_budget);
void nmac_rx_irq(struct nmac_cq_ring *cq);
int nmac_poll_rx_cq(struct napi_struct *napi, int budget);

// nmac_ethtool.c
extern const struct ethtool_ops nmac_ethtool_ops;

#endif /* NMAC_H */
