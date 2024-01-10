#include "nmac.h"

int nmac_create_rx_ring(struct nmac_if *interface, struct nmac_ring **ring_ptr,
		int index, u8 __iomem *hw_addr)
{
	struct nmac_ring *ring;

	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring)
		return -ENOMEM;

	*ring_ptr = ring;

	ring->dev = interface->dev;
	ring->interface = interface;

	ring->index = index;
	ring->active = 0;

	ring->hw_addr = hw_addr;
	ring->hw_ptr_mask = 0xffff;
	ring->hw_head_ptr = hw_addr + NMAC_QUEUE_HEAD_PTR_REG;
	ring->hw_tail_ptr = hw_addr + NMAC_QUEUE_TAIL_PTR_REG;

	ring->head_ptr = 0;
	ring->tail_ptr = 0;
	ring->clean_tail_ptr = 0;

	// deactivate queue
	iowrite32(0, ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);

	return 0;
}

void nmac_destroy_rx_ring(struct nmac_ring **ring_ptr)
{
	struct nmac_ring *ring = *ring_ptr;

	nmac_free_rx_ring(ring);

	*ring_ptr = NULL;
	kfree(ring);
}

int nmac_alloc_rx_ring(struct nmac_ring *ring, int size, int stride)
{
	int ret = 0;

	if (ring->active || ring->buf)
		return -EINVAL;

	ring->size = roundup_pow_of_two(size);
	ring->size_mask = ring->size - 1;
	ring->stride = roundup_pow_of_two(stride);

	ring->desc_block_size = ring->stride / NMAC_DESC_SIZE;
	ring->log_desc_block_size = ring->desc_block_size < 2 ? 0 : ilog2(ring->desc_block_size - 1) + 1;
	ring->desc_block_size = 1 << ring->log_desc_block_size;

	ring->rx_info = kvzalloc(sizeof(*ring->rx_info) * ring->size, GFP_KERNEL);
	if (!ring->rx_info)
		return -ENOMEM;

	ring->buf_size = ring->size * ring->stride;
	ring->buf = dma_alloc_coherent(ring->dev, ring->buf_size, &ring->buf_dma_addr, GFP_KERNEL);
	if (!ring->buf) {
		ret = -ENOMEM;
		goto fail;
	}

	ring->head_ptr = 0;
	ring->tail_ptr = 0;
	ring->clean_tail_ptr = 0;

	// deactivate queue
	iowrite32(0, ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);
	// set base address
	iowrite32(ring->buf_dma_addr, ring->hw_addr + NMAC_QUEUE_BASE_ADDR_REG + 0);
	iowrite32(ring->buf_dma_addr >> 32, ring->hw_addr + NMAC_QUEUE_BASE_ADDR_REG + 4);
	// set completion queue index
	iowrite32(0, ring->hw_addr + NMAC_QUEUE_CPL_QUEUE_INDEX_REG);
	// set pointers
	iowrite32(ring->head_ptr & ring->hw_ptr_mask, ring->hw_addr + NMAC_QUEUE_HEAD_PTR_REG);
	iowrite32(ring->tail_ptr & ring->hw_ptr_mask, ring->hw_addr + NMAC_QUEUE_TAIL_PTR_REG);
	// set size
	iowrite32(ilog2(ring->size) | (ring->log_desc_block_size << 8),
			ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);

	return 0;

fail:
	nmac_free_rx_ring(ring);
	return ret;
}

void nmac_free_rx_ring(struct nmac_ring *ring)
{
	nmac_deactivate_rx_ring(ring);

	if (ring->buf) {
		nmac_free_rx_buf(ring);

		dma_free_coherent(ring->dev, ring->buf_size, ring->buf, ring->buf_dma_addr);
		ring->buf = NULL;
		ring->buf_dma_addr = 0;
	}

	if (ring->rx_info) {
		kvfree(ring->rx_info);
		ring->rx_info = NULL;
	}
}

int nmac_activate_rx_ring(struct nmac_ring *ring, struct nmac_priv *priv,
		struct nmac_cq_ring *cq_ring)
{
	nmac_deactivate_rx_ring(ring);

	if (!ring->buf || !priv || !cq_ring || cq_ring->handler || cq_ring->src_ring)
		return -EINVAL;

	ring->priv = priv;
	ring->cq_ring = cq_ring;
	cq_ring->src_ring = ring;
	cq_ring->handler = nmac_rx_irq;

	// deactivate queue
	iowrite32(0, ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);
	// set base address
	iowrite32(ring->buf_dma_addr, ring->hw_addr + NMAC_QUEUE_BASE_ADDR_REG + 0);
	iowrite32(ring->buf_dma_addr >> 32, ring->hw_addr + NMAC_QUEUE_BASE_ADDR_REG + 4);
	// set completion queue index
	iowrite32(cq_ring->index, ring->hw_addr + NMAC_QUEUE_CPL_QUEUE_INDEX_REG);
	// set pointers
	iowrite32(ring->head_ptr & ring->hw_ptr_mask, ring->hw_addr + NMAC_QUEUE_HEAD_PTR_REG);
	iowrite32(ring->tail_ptr & ring->hw_ptr_mask, ring->hw_addr + NMAC_QUEUE_TAIL_PTR_REG);
	// set size and activate queue
	iowrite32(ilog2(ring->size) | (ring->log_desc_block_size << 8) | NMAC_QUEUE_ACTIVE_MASK,
			ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);

	ring->active = 1;

	nmac_refill_rx_buffers(ring);

	return 0;
}

void nmac_deactivate_rx_ring(struct nmac_ring *ring)
{
	// deactivate queue
	iowrite32(ilog2(ring->size) | (ring->log_desc_block_size << 8),
			ring->hw_addr + NMAC_QUEUE_ACTIVE_LOG_SIZE_REG);

	if (ring->cq_ring) {
		ring->cq_ring->src_ring = NULL;
		ring->cq_ring->handler = NULL;
	}

	ring->priv = NULL;
	ring->cq_ring = NULL;

	ring->active = 0;
}

bool nmac_is_rx_ring_empty(const struct nmac_ring *ring)
{
	return ring->head_ptr == ring->clean_tail_ptr;
}

bool nmac_is_rx_ring_full(const struct nmac_ring *ring)
{
	return ring->head_ptr - ring->clean_tail_ptr >= ring->size;
}

void nmac_rx_read_tail_ptr(struct nmac_ring *ring)
{
	ring->tail_ptr += (ioread32(ring->hw_tail_ptr) - ring->tail_ptr) & ring->hw_ptr_mask;
}

void nmac_rx_write_head_ptr(struct nmac_ring *ring)
{
	iowrite32(ring->head_ptr & ring->hw_ptr_mask, ring->hw_head_ptr);
}

void nmac_free_rx_desc(struct nmac_ring *ring, int index)
{
	struct nmac_rx_info *rx_info = &ring->rx_info[index];
	struct page *page = rx_info->page;

	dma_unmap_page(ring->dev, dma_unmap_addr(rx_info, dma_addr),
			dma_unmap_len(rx_info, len), DMA_FROM_DEVICE);
	rx_info->dma_addr = 0;
	__free_pages(page, rx_info->page_order);
	rx_info->page = NULL;
}

int nmac_free_rx_buf(struct nmac_ring *ring)
{
	u32 index;
	int cnt = 0;

	while (!nmac_is_rx_ring_empty(ring)) {
		index = ring->clean_tail_ptr & ring->size_mask;
		nmac_free_rx_desc(ring, index);
		ring->clean_tail_ptr++;
		cnt++;
	}

	ring->head_ptr = 0;
	ring->tail_ptr = 0;
	ring->clean_tail_ptr = 0;

	return cnt;
}

int nmac_prepare_rx_desc(struct nmac_ring *ring, int index)
{
	struct nmac_rx_info *rx_info = &ring->rx_info[index];
	struct nmac_desc *rx_desc = (struct nmac_desc *)(ring->buf + index * ring->stride);
	struct page *page = rx_info->page;
	u32 page_order = ring->page_order;
	u32 len = PAGE_SIZE << page_order;
	dma_addr_t dma_addr;

	if (unlikely(page)) {
		dev_err(ring->dev, "%s: skb not yet processed on interface %d",
				__func__, ring->interface->index);
		return -1;
	}

	page = dev_alloc_pages(page_order);
	if (unlikely(!page)) {
		dev_err(ring->dev, "%s: failed to allocate memory on interface %d",
				__func__, ring->interface->index);
		return -1;
	}

	// map page
	dma_addr = dma_map_page(ring->dev, page, 0, len, DMA_FROM_DEVICE);

	if (unlikely(dma_mapping_error(ring->dev, dma_addr))) {
		dev_err(ring->dev, "%s: DMA mapping failed on interface %d",
				__func__, ring->interface->index);
		__free_pages(page, page_order);
		return -1;
	}

	// write descriptor
	rx_desc->len = cpu_to_le32(len);
	rx_desc->addr = cpu_to_le64(dma_addr);

	// update rx_info
	rx_info->page = page;
	rx_info->page_order = page_order;
	rx_info->page_offset = 0;
	rx_info->dma_addr = dma_addr;
	rx_info->len = len;

	return 0;
}

void nmac_refill_rx_buffers(struct nmac_ring *ring)
{
	u32 missing = ring->size - (ring->head_ptr - ring->clean_tail_ptr);

	if (missing < 8)
		return;

	for (; missing-- > 0;) {
		if (nmac_prepare_rx_desc(ring, ring->head_ptr & ring->size_mask))
			break;
		ring->head_ptr++;
	}

	// enqueue on NIC
	dma_wmb();
	nmac_rx_write_head_ptr(ring);
}

int nmac_process_rx_cq(struct nmac_cq_ring *cq_ring, int napi_budget)
{
	struct nmac_if *interface = cq_ring->interface;
	struct device *dev = interface->dev;
	struct nmac_ring *rx_ring = cq_ring->src_ring;
	struct nmac_priv *priv = rx_ring->priv;
	struct nmac_rx_info *rx_info;
	struct nmac_cpl *cpl;
	struct sk_buff *skb;
	struct page *page;
	u32 cq_index;
	u32 cq_tail_ptr;
	u32 ring_index;
	u32 ring_clean_tail_ptr;
	int done = 0;
	int budget = napi_budget;
	u32 len;

	if (unlikely(!priv || !priv->port_up))
		return done;

	// process completion queue
	// read head pointer from NIC
	nmac_cq_read_head_ptr(cq_ring);

	cq_tail_ptr = cq_ring->tail_ptr;
	cq_index = cq_tail_ptr & cq_ring->size_mask;

	mb(); // is a barrier here necessary?  If so, what kind?

	while (cq_ring->head_ptr != cq_tail_ptr && done < budget) {
		cpl = (struct nmac_cpl *)(cq_ring->buf + cq_index * cq_ring->stride);
		ring_index = le16_to_cpu(cpl->index) & rx_ring->size_mask;
		rx_info = &rx_ring->rx_info[ring_index];
		page = rx_info->page;

		if (unlikely(!page)) {
			dev_err(dev, "%s: ring %d null page at index %d",
					__func__, cq_ring->index, ring_index);
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 1,
					cpl, NMAC_CPL_SIZE, true);
			break;
		}

		skb = napi_get_frags(&cq_ring->napi);
		if (unlikely(!skb)) {
			dev_err(dev, "%s: ring %d failed to allocate skb",
					__func__, cq_ring->index);
			break;
		}

		// RX hardware timestamp
		if (interface->if_features & NMAC_IF_FEATURE_PTP_TS)
			skb_hwtstamps(skb)->hwtstamp = nmac_read_cpl_ts(interface->mdev, rx_ring, cpl);

		skb_record_rx_queue(skb, rx_ring->index);

		// RX hardware checksum
		if (priv->ndev->features & NETIF_F_RXCSUM) {
			skb->csum = csum_unfold((__sum16) cpu_to_be16(le16_to_cpu(cpl->rx_csum)));
			skb->ip_summed = CHECKSUM_COMPLETE;
		}

		// unmap
		dma_unmap_page(dev, dma_unmap_addr(rx_info, dma_addr),
				dma_unmap_len(rx_info, len), DMA_FROM_DEVICE);
		rx_info->dma_addr = 0;

		len = min_t(u32, le16_to_cpu(cpl->len), rx_info->len);

		dma_sync_single_range_for_cpu(dev, rx_info->dma_addr, rx_info->page_offset,
				rx_info->len, DMA_FROM_DEVICE);

		__skb_fill_page_desc(skb, 0, page, rx_info->page_offset, len);
		rx_info->page = NULL;

		skb_shinfo(skb)->nr_frags = 1;
		skb->len = len;
		skb->data_len = len;
		skb->truesize += rx_info->len;

		// hand off SKB
		napi_gro_frags(&cq_ring->napi);

		rx_ring->packets++;
		rx_ring->bytes += le16_to_cpu(cpl->len);

		done++;

		cq_tail_ptr++;
		cq_index = cq_tail_ptr & cq_ring->size_mask;
	}

	// update CQ tail
	cq_ring->tail_ptr = cq_tail_ptr;
	nmac_cq_write_tail_ptr(cq_ring);

	// process ring
	// read tail pointer from NIC
	nmac_rx_read_tail_ptr(rx_ring);

	ring_clean_tail_ptr = READ_ONCE(rx_ring->clean_tail_ptr);
	ring_index = ring_clean_tail_ptr & rx_ring->size_mask;

	while (ring_clean_tail_ptr != rx_ring->tail_ptr) {
		rx_info = &rx_ring->rx_info[ring_index];

		if (rx_info->page)
			break;

		ring_clean_tail_ptr++;
		ring_index = ring_clean_tail_ptr & rx_ring->size_mask;
	}

	// update ring tail
	WRITE_ONCE(rx_ring->clean_tail_ptr, ring_clean_tail_ptr);

	// replenish buffers
	nmac_refill_rx_buffers(rx_ring);

	return done;
}

void nmac_rx_irq(struct nmac_cq_ring *cq)
{
	napi_schedule_irqoff(&cq->napi);
}

int nmac_poll_rx_cq(struct napi_struct *napi, int budget)
{
	struct nmac_cq_ring *cq_ring = container_of(napi, struct nmac_cq_ring, napi);
	int done;

	done = nmac_process_rx_cq(cq_ring, budget);

	if (done == budget)
		return done;

	napi_complete(napi);

	nmac_arm_cq(cq_ring);

	return done;
}
