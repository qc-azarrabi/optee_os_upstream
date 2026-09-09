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
 * vring_need_event() - has the used index crossed the driver's event index?
 *
 * The algorithm from the virtio specification (VIRTIO_F_EVENT_IDX): returns
 * true when @new_idx has passed @event_idx since @old_idx, using wrapping
 * unsigned 16-bit arithmetic.
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

void vdevice_init(struct vdevice *vdev, struct vdevice_vq *vqs, int num_queues,
		  const struct vdevice_ops *ops)
{
	int n = 0;

	vdev->ops = ops;
	vdev->num_queues = num_queues;
	vdev->vqs = vqs;
	vdev->status = 0;
	vdev->started = false;
	vdev->features_neg = 0;

	memset(vqs, 0, num_queues * sizeof(*vqs));
	for (n = 0; n < num_queues; n++) {
		vqs[n].qid = n;
		vqs[n].lock = SPINLOCK_UNLOCK;
	}

	vdev->features = 0;
	if (ops->get_features)
		ops->get_features(vdev, &vdev->features);
}

uint8_t vdevice_status_read(struct vdevice *vdev)
{
	return vdev->status;
}

int vdevice_status_write(struct vdevice *vdev, uint8_t status)
{
	uint8_t old = vdev->status;
	int n = 0;
	int res = 0;

	/*
	 * A write of 0 is a device reset (virtio spec "Device
	 * Initialization"). Stop the device and reset every queue. This must
	 * not panic: the driver may reset at any time.
	 */
	if (status == 0) {
		if (vdev->started && vdev->ops->stop)
			vdev->ops->stop(vdev);
		vdev->started = false;
		for (n = 0; n < vdev->num_queues; n++)
			vdevice_reset_queue(vdev, n);
		vdev->features_neg = 0;
		vdev->status = 0;
		return 0;
	}

	/* Status bits are only ever set, never cleared (except by reset) */
	if ((old & status) != old) {
		EMSG("Illegal status transition %#"PRIx8" -> %#"PRIx8,
		     old, status);
		return -1;
	}

	/* Driver just set FEATURES_OK: latch the negotiated features */
	if ((status & VDEVICE_STATUS_FEATURES_OK) &&
	    !(old & VDEVICE_STATUS_FEATURES_OK)) {
		if (vdev->ops->finalize_features) {
			res = vdev->ops->finalize_features(vdev,
							   vdev->features_neg);
			if (res)
				return res;
		}
	}

	/* Driver just set DRIVER_OK: bring the device live */
	if ((status & VDEVICE_STATUS_DRIVER_OK) &&
	    !(old & VDEVICE_STATUS_DRIVER_OK)) {
		if (vdev->ops->start) {
			res = vdev->ops->start(vdev);
			if (res)
				return res;
		}
		vdev->started = true;
	}

	vdev->status = status;

	return 0;
}

int vdevice_set_ring(struct vdevice *vdev, int qid, void *desc, void *avail,
		     void *used, uint64_t desc_ba, uint64_t avail_ba,
		     uint64_t used_ba, uint16_t num)
{
	struct vdevice_vq *vq = vdevice_get_vq(vdev, qid);

	if (!vq)
		return -1;
	if (!num || !IS_POWER_OF_TWO(num))
		return -1;

	vq->desc = desc;
	vq->avail = avail;
	vq->used = used;
	vq->desc_ba = desc_ba;
	vq->avail_ba = avail_ba;
	vq->used_ba = used_ba;
	vq->num = num;

	vq->last_avail_idx = 0;
	vq->last_used_idx = 0;
	vq->signalled_used_valid = false;

	vq->f_event_idx = vdev->features_neg & BIT64(VDEVICE_F_EVENT_IDX);
	vq->f_indirect = vdev->features_neg & BIT64(VDEVICE_F_INDIRECT_DESC);
	vq->f_in_order = vdev->features_neg & BIT64(VDEVICE_F_IN_ORDER);

	return 0;
}

void vdevice_set_ring_ready(struct vdevice_vq *vq, bool ready)
{
	vq->ready = ready;
}

int vdevice_reset_queue(struct vdevice *vdev, int qid)
{
	struct vdevice_vq *vq = vdevice_get_vq(vdev, qid);

	if (!vq)
		return -1;

	if (vdev->ops->reset_vq)
		vdev->ops->reset_vq(vdev, vq);

	vq->ready = false;
	vq->desc = NULL;
	vq->avail = NULL;
	vq->used = NULL;
	vq->num = 0;
	vq->last_avail_idx = 0;
	vq->last_used_idx = 0;
	vq->signalled_used_valid = false;

	return 0;
}

/*
 * Append one descriptor's buffer to the readable or writable sglist depending
 * on VIRTQ_DESC_F_WRITE. Returns 0 on success or -1 if a list would overflow.
 */
static int vdevice_sg_add(struct vdevice_chain *chain,
			  const struct virtq_desc *d)
{
	struct vdevice_sglist *sgl = NULL;

	if (d->flags & VIRTQ_DESC_F_WRITE)
		sgl = &chain->in;
	else
		sgl = &chain->out;

	if (sgl->num >= VDEVICE_MAX_DESC_CHAIN)
		return -1;

	sgl->sg[sgl->num].addr = d->addr;
	sgl->sg[sgl->num].len = d->len;
	sgl->num++;
	sgl->total += d->len;

	return 0;
}

int vdevice_get_vq_desc(struct vdevice_vq *vq, struct vdevice_chain *chain)
{
	struct virtq_desc *desc = vq->desc;
	uint16_t avail_idx = 0;
	uint16_t head = 0;
	uint16_t idx = 0;
	unsigned int count = 0;

	if (!vq->ready || !desc || !vq->avail)
		return -1;

	/* dsb() before reading avail->idx published by the driver */
	dsb();
	avail_idx = vq->avail->idx;
	if (avail_idx == vq->last_avail_idx)
		return 0;	/* ring empty */

	memset(chain, 0, sizeof(*chain));
	head = vq->avail->ring[vq->last_avail_idx % vq->num];
	chain->head = head;
	idx = head;

	/*
	 * Walk the chain. Indirect descriptors are not yet supported; a driver
	 * only uses them if the device offered VIRTQ_DESC_F_INDIRECT, which
	 * this core does not.
	 */
	for (;;) {
		if (idx >= vq->num)
			return -1;
		if (desc[idx].flags & VIRTQ_DESC_F_INDIRECT)
			return -1;
		if (count++ >= vq->num || count > VDEVICE_MAX_DESC_CHAIN)
			return -1;

		if (vdevice_sg_add(chain, &desc[idx]))
			return -1;

		if (!(desc[idx].flags & VIRTQ_DESC_F_NEXT))
			break;
		idx = desc[idx].next;
	}

	vq->last_avail_idx++;

	return 1;
}

int vdevice_add_used(struct vdevice_vq *vq, uint16_t head, uint32_t len)
{
	uint32_t exceptions = 0;
	uint16_t idx = 0;

	if (!vq->used)
		return -1;

	exceptions = cpu_spin_lock_xsave(&vq->lock);

	idx = vq->last_used_idx % vq->num;
	vq->used->ring[idx].id = head;
	vq->used->ring[idx].len = len;

	/* Publish the ring entry before advancing the visible index */
	dsb();
	vq->last_used_idx++;
	vq->used->idx = vq->last_used_idx;

	cpu_spin_unlock_xrestore(&vq->lock, exceptions);

	return 0;
}

/*
 * used_event is stored just past the avail ring (see the virtio spec split
 * virtqueue layout: le16 used_event after avail->ring[queue_size]).
 */
static uint16_t vdevice_used_event(struct vdevice_vq *vq)
{
	return vq->avail->ring[vq->num];
}

void vdevice_signal(struct vdevice *vdev, struct vdevice_vq *vq)
{
	bool notify = false;
	uint16_t old = 0;
	uint16_t new = 0;

	/* Ensure the used ring is visible before we test the driver's flags */
	dsb();

	if (vq->f_event_idx) {
		new = vq->last_used_idx;
		old = vq->signalled_used_idx;
		notify = vring_need_event(vdevice_used_event(vq), new, old) ||
			 !vq->signalled_used_valid;
		vq->signalled_used_idx = new;
		vq->signalled_used_valid = true;
	} else {
		notify = !(vq->avail->flags & VIRTQ_AVAIL_F_NO_INTERRUPT);
	}

	if (notify && vdev->signal)
		vdev->signal(vdev, vq->qid);
}

bool vdevice_enable_notify(struct vdevice_vq *vq)
{
	/*
	 * Ask the driver to notify us again and re-check the ring for entries
	 * that arrived in the meantime. Without EVENT_IDX there is nothing to
	 * write back; the "clear NO_NOTIFY" advisory lives in the used ring
	 * flags which the device owns.
	 */
	vq->used->flags &= ~VIRTQ_USED_F_NO_NOTIFY;
	dsb();

	return vq->avail->idx != vq->last_avail_idx;
}

void vdevice_disable_notify(struct vdevice_vq *vq)
{
	if (!vq->f_event_idx)
		vq->used->flags |= VIRTQ_USED_F_NO_NOTIFY;
}

/*
 * Copy @len bytes between a linear buffer and the segments of @sgl, mapping
 * each segment's driver bus address on demand. @to selects the direction:
 * true copies from @buf into the (writable) segments, false copies out of the
 * (readable) segments into @buf.
 */
static int vdevice_sglist_copy(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			       struct vdevice_sglist_cursor *cur, void *buf,
			       size_t len, bool to)
{
	uint8_t *lin = buf;
	size_t done = 0;

	while (done < len) {
		struct vdevice_sg *seg = NULL;
		size_t avail = 0;
		size_t chunk = 0;
		void *va = NULL;

		if (cur->seg >= sgl->num)
			return -1;

		seg = &sgl->sg[cur->seg];
		avail = seg->len - cur->off;
		if (!avail) {
			cur->seg++;
			cur->off = 0;
			continue;
		}

		chunk = MIN(avail, len - done);
		va = vq->dma.map(vq->dma.cookie, seg->addr + cur->off, chunk);
		if (!va)
			return -1;

		if (to)
			memcpy(va, lin + done, chunk);
		else
			memcpy(lin + done, va, chunk);

		vq->dma.unmap(vq->dma.cookie, va, chunk);

		done += chunk;
		cur->off += chunk;
	}

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
