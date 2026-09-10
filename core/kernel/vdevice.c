// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2025, Linaro Limited
 */

#include <kernel/spinlock.h>
#include <kernel/vdevice.h>
#include <mm/core_mmu.h>	/* for dsb() via <arm.h> */
#include <string.h>
#include <trace.h>
#include <util.h>

/*
 * This core is a port of the QTEE vhost backend. Guest ring memory is already
 * mapped by the transport, so the QTEE guest-copy helpers reduce to memcpy and
 * the release/acquire barriers to dsb(). Feature sets fit in 64 bits, so a
 * plain uint64_t replaces the QTEE virtio_features_t bitset.
 */

static bool vdevice_has_feat(struct vdevice *vdev, int bit)
{
	return vdev->features_neg & BIT64(bit);
}

/*
 * vring_need_event() - has the used index crossed the driver's event index?
 *
 * The algorithm from the virtio specification (VIRTIO_F_EVENT_IDX): returns
 * true when @new_idx has passed @event_idx since @old_idx, using wrapping
 * unsigned 16-bit arithmetic. Only meaningful when EVENT_IDX is negotiated.
 */
static bool vring_need_event(uint16_t event_idx, uint16_t new_idx,
			     uint16_t old_idx)
{
	return (uint16_t)(new_idx - event_idx - 1) < (uint16_t)(new_idx -
							       old_idx);
}

static struct vdevice_vq *vdevice_get_vq(struct vdevice *vdev, int qid)
{
	if (qid < 0 || qid >= vdev->num_queues)
		return NULL;

	return &vdev->vqs[qid];
}

/*
 * Split-ring accessors. The ring pointers are host-accessible (mapped by the
 * transport); these wrap the accesses so ordering and the used_event /
 * avail_event trailing fields live in one place.
 */
static uint16_t vdevice_get_avail_flags(struct vdevice_vq *vq)
{
	return vq->avail->flags;
}

static uint16_t vdevice_get_avail_entry(struct vdevice_vq *vq, uint16_t i)
{
	return vq->avail->ring[i];
}

/* used_event follows avail->ring[queue_size] (virtio split-ring layout) */
static uint16_t vdevice_get_used_event(struct vdevice_vq *vq)
{
	return vq->avail->ring[vq->num];
}

static void vdevice_put_used_flags(struct vdevice_vq *vq, uint16_t flags)
{
	vq->used->flags = flags;
}

static void vdevice_put_used_idx(struct vdevice_vq *vq, uint16_t idx)
{
	vq->used->idx = idx;
}

static void vdevice_put_used_entry(struct vdevice_vq *vq, uint16_t i,
				   struct virtq_used_elem *el)
{
	vq->used->ring[i] = *el;
}

/* avail_event follows used->ring[queue_size] (virtio split-ring layout) */
static void vdevice_put_avail_event(struct vdevice_vq *vq, uint16_t ev)
{
	memcpy(&vq->used->ring[vq->num], &ev, sizeof(ev));
}

static struct virtq_desc vdevice_get_desc(struct vdevice_vq *vq, uint16_t i)
{
	return vq->desc[i];
}

static struct virtq_desc vdevice_get_indirect_desc(struct virtq_desc *table,
						   uint16_t i)
{
	return table[i];
}

static void *vdevice_map_guest(struct vdevice_vq *vq, uint64_t dma_addr,
			       size_t size)
{
	struct vdevice_dma *dma = &vq->vdev->dma;

	if (!dma->map)
		return NULL;

	return dma->map(vq->vdev, dma_addr, size);
}

static void vdevice_unmap_guest(struct vdevice_vq *vq, void *addr, size_t size)
{
	struct vdevice_dma *dma = &vq->vdev->dma;

	if (dma->unmap)
		dma->unmap(vq->vdev, addr, size);
}

/* Copy @len bytes from a guest DMA address into a host buffer */
static int vdevice_copy_from_guest_dma(struct vdevice_vq *vq, void *dst,
				       uint64_t src_dma, size_t len)
{
	void *ptr = NULL;

	if (!len)
		return 0;

	ptr = vdevice_map_guest(vq, src_dma, len);
	if (!ptr)
		return -1;

	memcpy(dst, ptr, len);
	vdevice_unmap_guest(vq, ptr, len);

	return 0;
}

/* Copy @len bytes from a host buffer into a guest DMA address */
static int vdevice_copy_to_guest_dma(struct vdevice_vq *vq, uint64_t dst_dma,
				     const void *src, size_t len)
{
	void *ptr = NULL;

	if (!len)
		return 0;

	ptr = vdevice_map_guest(vq, dst_dma, len);
	if (!ptr)
		return -1;

	memcpy(ptr, src, len);
	vdevice_unmap_guest(vq, ptr, len);

	return 0;
}

void vdevice_init(struct vdevice *vdev, struct vdevice_vq *vqs, int num_queues,
		  const struct vdevice_ops *ops)
{
	int n = 0;

	vdev->ops = ops;
	vdev->num_queues = num_queues;
	vdev->vqs = vqs;
	vdev->status = 0;
	vdev->started = false;

	/* Core default features; device-specific bits are added by the
	 * transport via ops->get_features().
	 */
	vdev->features = BIT64(VDEVICE_F_INDIRECT_DESC) |
			 BIT64(VDEVICE_F_EVENT_IDX) |
			 BIT64(VDEVICE_F_VERSION_1);
	vdev->features_neg = 0;

	memset(vqs, 0, num_queues * sizeof(*vqs));
	for (n = 0; n < num_queues; n++) {
		vqs[n].qid = n;
		vqs[n].lock = SPINLOCK_UNLOCK;
		vqs[n].vdev = vdev;
	}
}

int vdevice_check_features(struct vdevice *vdev __unused, uint64_t features)
{
	int bit = 0;

	/* Always require VIRTIO_F_VERSION_1 */
	if (!(features & BIT64(VDEVICE_F_VERSION_1)))
		return -1;

	for (bit = VDEVICE_F_START; bit < VDEVICE_F_END; bit++) {
		switch (bit) {
		/* Supported device-independent features */
		case VDEVICE_F_INDIRECT_DESC:
		case VDEVICE_F_EVENT_IDX:
		case VDEVICE_F_VERSION_1:
		case VDEVICE_F_ACCESS_PLATFORM:
		case VDEVICE_F_IN_ORDER:
		case VDEVICE_F_RING_RESET:
			break;
		default:
			/* Reject any reserved bit we do not implement */
			if (features & BIT64(bit))
				return -1;
		}
	}

	return 0;
}

uint8_t vdevice_status_read(struct vdevice *vdev)
{
	return vdev->status;
}

/* Reset all per-queue state (see vdevice_reset_queue for the device-type part) */
static void vdevice_vq_reset(struct vdevice_vq *vq)
{
	vq->ready = false;

	vq->f_version_1 = false;
	vq->f_event_idx = false;
	vq->f_indirect = false;
	vq->f_in_order = false;
	vq->f_reset_vq = false;

	vq->last_avail_idx = 0;
	vq->last_used_idx = 0;
	vq->last_signalled_used_idx = 0;
	vq->used_flags = 0;
	vq->avail_idx = 0;

	vq->desc = NULL;
	vq->avail = NULL;
	vq->used = NULL;
	vq->num = 0;
	vq->desc_ba = 0;
	vq->avail_ba = 0;
	vq->used_ba = 0;
}

static void vdevice_stop(struct vdevice *vdev)
{
	if (vdev->started && vdev->ops->stop)
		vdev->ops->stop(vdev);
	vdev->started = false;
}

static void vdevice_reset(struct vdevice *vdev)
{
	int n = 0;

	vdevice_stop(vdev);
	for (n = 0; n < vdev->num_queues; n++)
		vdevice_vq_reset(&vdev->vqs[n]);

	vdev->features_neg = 0;
	vdev->status = 0;
}

static int vdevice_start(struct vdevice *vdev)
{
	int res = 0;

	if (vdev->started)
		return 0;

	if (vdev->ops->start) {
		res = vdev->ops->start(vdev);
		if (res)
			return res;
	}
	vdev->started = true;

	return 0;
}

int vdevice_status_write(struct vdevice *vdev, uint8_t status)
{
	uint8_t old = vdev->status;

	/* See the virtio spec "Device Initialization" for the transitions */

	/* A write of 0 is a clean device reset; must not panic */
	if (!status) {
		vdevice_reset(vdev);
		return 0;
	}

	/* Already FAILED: accept no further change until reset */
	if (old & VDEVICE_STATUS_FAILED)
		return -1;

	/* The driver cannot set NEEDS_RESET */
	if (status & VDEVICE_STATUS_NEEDS_RESET)
		return -1;

	if (status & VDEVICE_STATUS_FAILED) {
		/* Latch FAILED and stop; wait for reset */
		vdev->status |= VDEVICE_STATUS_FAILED;
		vdevice_stop(vdev);
		return 0;
	}

	/* Status bits are only ever set, never cleared (except by reset) */
	if ((status & old) != old) {
		EMSG("Illegal status transition %#"PRIx8" -> %#"PRIx8,
		     old, status);
		return -1;
	}

	/* FEATURES_OK requires VERSION_1 among the negotiated features */
	if ((status & VDEVICE_STATUS_FEATURES_OK) &&
	    !(old & VDEVICE_STATUS_FEATURES_OK)) {
		if (!vdevice_has_feat(vdev, VDEVICE_F_VERSION_1))
			return -1;
	}

	/* DRIVER_OK only after FEATURES_OK */
	if ((status & VDEVICE_STATUS_DRIVER_OK) &&
	    !(status & VDEVICE_STATUS_FEATURES_OK))
		return -1;

	vdev->status = status;

	/* Bring the device live on the DRIVER_OK edge */
	if ((status & VDEVICE_STATUS_DRIVER_OK) &&
	    !(old & VDEVICE_STATUS_DRIVER_OK))
		vdevice_start(vdev);

	return 0;
}

int vdevice_set_ring(struct vdevice *vdev, int qid, void *desc, void *avail,
		     void *used, uint64_t desc_ba, uint64_t avail_ba,
		     uint64_t used_ba, uint16_t num)
{
	struct vdevice_vq *vq = vdevice_get_vq(vdev, qid);

	if (!vq || !desc || !avail || !used || !num)
		return -1;
	if (!IS_POWER_OF_TWO(num))
		return -1;

	/* Must be at FEATURES_OK so the negotiated features are final */
	if (!(vdev->status & VDEVICE_STATUS_FEATURES_OK))
		return -1;

	/* Do not (re)program a running queue */
	if (vq->ready)
		return -1;

	vq->desc = desc;
	vq->avail = avail;
	vq->used = used;
	vq->desc_ba = desc_ba;
	vq->avail_ba = avail_ba;
	vq->used_ba = used_ba;
	vq->num = num;

	/* Seed index and cache state from the guest rings */
	dsb();
	vq->avail_idx = vq->avail->idx;
	vq->last_avail_idx = vq->avail->idx;
	vq->last_used_idx = vq->used->idx;
	vq->last_signalled_used_idx = vq->used->idx;
	vq->used_flags = vq->used->flags;

	/* Cache negotiated features for the queue fast paths */
	vq->f_version_1 = vdevice_has_feat(vdev, VDEVICE_F_VERSION_1);
	vq->f_event_idx = vdevice_has_feat(vdev, VDEVICE_F_EVENT_IDX);
	vq->f_indirect = vdevice_has_feat(vdev, VDEVICE_F_INDIRECT_DESC);
	vq->f_in_order = vdevice_has_feat(vdev, VDEVICE_F_IN_ORDER);
	vq->f_reset_vq = vdevice_has_feat(vdev, VDEVICE_F_RING_RESET);

	return 0;
}

void vdevice_set_ring_ready(struct vdevice_vq *vq, bool ready)
{
	uint32_t exceptions = cpu_spin_lock_xsave(&vq->lock);

	/* A zero-size queue is never worth enabling */
	if (vq->num)
		vq->ready = ready;

	cpu_spin_unlock_xrestore(&vq->lock, exceptions);
}

int vdevice_reset_queue(struct vdevice *vdev, int qid)
{
	struct vdevice_vq *vq = vdevice_get_vq(vdev, qid);

	if (!vq)
		return -1;

	/* Only valid when VIRTIO_F_RING_RESET was negotiated */
	if (!vq->f_reset_vq)
		return -1;

	vdevice_set_ring_ready(vq, false);
	if (vdev->ops->reset_vq)
		vdev->ops->reset_vq(vdev, vq);
	vdevice_vq_reset(vq);

	return 0;
}

/*
 * Read and validate avail->idx. Returns 0 if new entries are available, 1 if
 * the ring is empty, or -1 if the index is out of the ring-size bound.
 */
static int vdevice_get_avail_idx(struct vdevice_vq *vq)
{
	vq->avail_idx = vq->avail->idx;

	if ((uint16_t)(vq->avail_idx - vq->last_avail_idx) > vq->num)
		return -1;

	if (vq->avail_idx == vq->last_avail_idx)
		return 1;

	/* avail_idx advanced: acquire the entries the driver published */
	dsb();

	return 0;
}

/* Fetch descriptor @idx from the main table (@table NULL) or an indirect one */
static int vdevice_chain_get_desc(struct vdevice_vq *vq,
				  struct virtq_desc *table, uint16_t limit,
				  uint16_t idx, struct virtq_desc *desc)
{
	if (idx >= limit)
		return -1;

	if (table)
		*desc = vdevice_get_indirect_desc(table, idx);
	else
		*desc = vdevice_get_desc(vq, idx);

	return 0;
}

/*
 * Walk a descriptor chain starting at @start in the table (@table, @limit) and
 * append each buffer into vq->sgs, enforcing the split-ring rule that all
 * device-readable (OUT) descriptors precede device-writable (IN) ones. A direct
 * chain may end in a single INDIRECT descriptor, which is recursed into once.
 * @out/@in/@sgs_idx accumulate across the (single) recursion.
 */
static int vdevice_chain_accum(struct vdevice_vq *vq, uint16_t start,
			       struct virtq_desc *table, uint16_t limit,
			       uint16_t *out, uint16_t *in, uint16_t *sgs_idx)
{
	uint16_t idx = start;
	struct virtq_desc desc;
	int res = 0;

	for (;;) {
		if (*sgs_idx >= VDEVICE_MAX_DESC_CHAIN)
			return -1;

		res = vdevice_chain_get_desc(vq, table, limit, idx, &desc);
		if (res)
			return res;

		if (desc.flags & VIRTQ_DESC_F_INDIRECT) {
			struct virtq_desc *tbl = NULL;
			uint16_t nent = 0;

			/* No nested indirect tables */
			if (table)
				return -1;
			/* An indirect desc must terminate the direct chain */
			if (desc.flags & VIRTQ_DESC_F_NEXT)
				return -1;
			if (!vq->f_indirect)
				return -1;
			if (!desc.len || desc.len % sizeof(struct virtq_desc))
				return -1;
			if (desc.addr & (VRING_DESC_ALIGN_SIZE - 1))
				return -1;

			tbl = vdevice_map_guest(vq, desc.addr, desc.len);
			if (!tbl)
				return -1;

			nent = desc.len / sizeof(struct virtq_desc);
			/* Recurse once; the nested call ends via the "no
			 * nested indirect" check above.
			 */
			res = vdevice_chain_accum(vq, 0, tbl, nent, out, in,
						  sgs_idx);
			vdevice_unmap_guest(vq, tbl, desc.len);

			return res;
		}

		if (desc.flags & VIRTQ_DESC_F_WRITE) {
			/* Device-writable (driver IN) */
			vq->sgs[*sgs_idx].addr = desc.addr;
			vq->sgs[*sgs_idx].len = desc.len;
			(*in)++;
		} else {
			/* Device-readable (driver OUT), must precede any IN */
			if (*in)
				return -1;
			vq->sgs[*sgs_idx].addr = desc.addr;
			vq->sgs[*sgs_idx].len = desc.len;
			(*out)++;
		}

		(*sgs_idx)++;
		if (!(desc.flags & VIRTQ_DESC_F_NEXT))
			break;
		idx = desc.next;
	}

	return 0;
}

int vdevice_get_vq_desc(struct vdevice_vq *vq, struct vdevice_chain *chain)
{
	uint16_t out_n = 0;
	uint16_t in_n = 0;
	uint16_t sgs_idx = 0;
	uint16_t head = 0;
	uint16_t idx = 0;
	struct virtq_desc desc;
	int res = 0;

	if (!vq->ready || !vq->desc || !vq->avail)
		return -1;

	if (vq->avail_idx == vq->last_avail_idx) {
		res = vdevice_get_avail_idx(vq);
		if (res)
			return res;	/* 1 empty, -1 error */
	}

	/* avail->idx is free-running; index the ring modulo queue size */
	idx = vq->last_avail_idx & (vq->num - 1);
	head = vdevice_get_avail_entry(vq, idx);
	if (head >= vq->num)
		return -1;

	desc = vdevice_get_desc(vq, head);
	if (desc.flags & VIRTQ_DESC_F_INDIRECT) {
		struct virtq_desc *tbl = NULL;
		uint16_t nent = 0;

		if (!vq->f_indirect)
			return -1;
		if (!desc.len || desc.len % sizeof(struct virtq_desc))
			return -1;
		if (desc.addr & (VRING_DESC_ALIGN_SIZE - 1))
			return -1;

		nent = desc.len / sizeof(struct virtq_desc);
		tbl = vdevice_map_guest(vq, desc.addr, desc.len);
		if (!tbl)
			return -1;

		res = vdevice_chain_accum(vq, 0, tbl, nent, &out_n, &in_n,
					  &sgs_idx);
		vdevice_unmap_guest(vq, tbl, desc.len);
	} else {
		res = vdevice_chain_accum(vq, head, NULL, vq->num, &out_n,
					  &in_n, &sgs_idx);
	}

	if (res)
		return res;

	memset(chain, 0, sizeof(*chain));
	chain->head = head;
	if (out_n) {
		chain->out.sg = &vq->sgs[0];
		chain->out.num = out_n;
	}
	if (in_n) {
		chain->in.sg = &vq->sgs[out_n];
		chain->in.num = in_n;
	}

	vq->last_avail_idx++;

	return 0;
}

/* Publish @n completions to the used ring, out of submission order */
static int vdevice_add_used_out_of_order(struct vdevice_vq *vq,
					 struct virtq_used_elem *elems, int n)
{
	uint16_t used_idx = 0;
	uint16_t idx = 0;
	int i = 0;

	for (i = 0; i < n; i++) {
		idx = (vq->last_used_idx + i) & (vq->num - 1);
		vdevice_put_used_entry(vq, idx, &elems[i]);
	}

	/* Publish the entries before advancing the visible index */
	dsb();

	used_idx = vq->last_used_idx + n;
	vdevice_put_used_idx(vq, used_idx);
	vq->last_used_idx = used_idx;

	return 0;
}

/*
 * Publish @n in-order batches: elems[i] is the representative entry for a batch
 * of nelems[i] buffers, written at the running used index (virtio spec 2.7.9).
 */
static int vdevice_add_used_in_order(struct vdevice_vq *vq,
				     struct virtq_used_elem *elems,
				     uint16_t *nelems, int n)
{
	uint16_t idx = vq->last_used_idx & (vq->num - 1);
	uint32_t batch = 0;
	uint16_t used_idx = 0;
	int i = 0;

	for (i = 0; i < n; i++)
		if (!nelems[i])
			return -1;

	for (i = 0; i < n; i++) {
		vdevice_put_used_entry(vq, idx, &elems[i]);
		idx += nelems[i];
		batch += nelems[i];
		if (idx >= vq->num)
			idx -= vq->num;
	}

	/* Publish the entries before advancing the visible index */
	dsb();

	used_idx = vq->last_used_idx + batch;
	vdevice_put_used_idx(vq, used_idx);
	vq->last_used_idx = used_idx;

	return 0;
}

int vdevice_add_used(struct vdevice_vq *vq, struct virtq_used_elem *elems,
		     uint16_t *nelems, int n)
{
	uint32_t exceptions = 0;
	int res = 0;

	if (!vq->used)
		return -1;

	exceptions = cpu_spin_lock_xsave(&vq->lock);
	if (vq->f_in_order)
		res = vdevice_add_used_in_order(vq, elems, nelems, n);
	else
		res = vdevice_add_used_out_of_order(vq, elems, n);
	cpu_spin_unlock_xrestore(&vq->lock, exceptions);

	return res;
}

int vdevice_add_used_one(struct vdevice_vq *vq, uint16_t head, uint32_t len)
{
	struct virtq_used_elem elem = { .id = head, .len = len };
	uint16_t nelems = 1;

	return vdevice_add_used(vq, &elem, &nelems, 1);
}

/* Decide whether the driver wants an interrupt now (best-effort suppression) */
static bool vdevice_should_signal(struct vdevice_vq *vq)
{
	uint16_t used_event = 0;
	uint16_t old_used = 0;
	uint16_t new_used = 0;

	/* Order the used->idx update against reading the driver's event */
	dsb();

	if (!vq->f_event_idx)
		return !(vdevice_get_avail_flags(vq) &
			 VIRTQ_AVAIL_F_NO_INTERRUPT);

	used_event = vdevice_get_used_event(vq);
	old_used = vq->last_signalled_used_idx;
	new_used = vq->last_used_idx;
	vq->last_signalled_used_idx = new_used;

	return vring_need_event(used_event, new_used, old_used);
}

void vdevice_signal(struct vdevice *vdev, struct vdevice_vq *vq)
{
	if (vdevice_should_signal(vq) && vdev->signal)
		vdev->signal(vdev, vq->qid);
}

bool vdevice_enable_notify(struct vdevice_vq *vq)
{
	if (!(vq->used_flags & VIRTQ_USED_F_NO_NOTIFY))
		return false;

	vq->used_flags &= ~VIRTQ_USED_F_NO_NOTIFY;

	if (!vq->f_event_idx)
		vdevice_put_used_flags(vq, vq->used_flags);
	else
		/* Don't ask again until the driver passes the current idx */
		vdevice_put_avail_event(vq, vq->avail_idx);

	/* Publish the update and re-observe avail->idx */
	dsb();

	/* 1 (empty) and -1 (error) both mean "nothing new" */
	if (vdevice_get_avail_idx(vq))
		return false;

	/* The driver added entries during the window: suppression failed */
	return true;
}

void vdevice_disable_notify(struct vdevice_vq *vq)
{
	if (vq->used_flags & VIRTQ_USED_F_NO_NOTIFY)
		return;

	vq->used_flags |= VIRTQ_USED_F_NO_NOTIFY;

	if (!vq->f_event_idx)
		vdevice_put_used_flags(vq, vq->used_flags);
}

/*
 * Copy @len bytes between a linear buffer and the segments of @sgl starting at
 * @cur, mapping each segment's driver DMA address on demand. @to selects the
 * direction: true copies from @buf into the (writable) segments, false copies
 * out of the (readable) segments into @buf.
 */
static int vdevice_sglist_copy(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			       struct vdevice_sglist_cursor *cur, void *buf,
			       size_t len, bool to)
{
	uint8_t *lin = buf;
	size_t done = 0;
	uint16_t seg = 0;
	size_t off = 0;

	if (!len)
		return 0;
	if (cur->seg >= sgl->num)
		return -1;

	seg = cur->seg;
	off = cur->off;

	while (done < len && seg < sgl->num) {
		struct vdevice_sg *s = &sgl->sg[seg];
		size_t chunk = 0;

		if (off >= s->len) {
			seg++;
			off = 0;
			continue;
		}

		chunk = MIN(len - done, s->len - off);
		if (to) {
			if (vdevice_copy_to_guest_dma(vq, s->addr + off,
						      lin + done, chunk))
				return -1;
		} else {
			if (vdevice_copy_from_guest_dma(vq, lin + done,
							s->addr + off, chunk))
				return -1;
		}

		done += chunk;
		off += chunk;
	}

	if (done != len)
		return -1;

	cur->seg = seg;
	cur->off = off;

	return 0;
}

int vdevice_sglist_copy_to(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			   struct vdevice_sglist_cursor *cur, const void *src,
			   size_t len)
{
	return vdevice_sglist_copy(vq, sgl, cur, (void *)src, len, true);
}

int vdevice_sglist_copy_from(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			     struct vdevice_sglist_cursor *cur, void *dst,
			     size_t len)
{
	return vdevice_sglist_copy(vq, sgl, cur, dst, len, false);
}
