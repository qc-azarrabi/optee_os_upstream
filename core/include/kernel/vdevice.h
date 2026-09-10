/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025, Linaro Limited
 */

#ifndef __KERNEL_VDEVICE_H
#define __KERNEL_VDEVICE_H

#include <kernel/spinlock.h>
#include <kernel/virtio_ring.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * vdevice - the virtio device (backend) core.
 *
 * This is the transport-agnostic heart of the virtio backend: it parses split
 * virtqueues, walks descriptor chains into scatter/gather lists, copies data
 * to and from driver-owned buffers and manages the used ring. It knows nothing
 * about the transport (virtio-msg, MMIO, ...) or about any particular device
 * type (vsock, net, ...).
 *
 * Naming: the virtio specification calls the two endpoints the "driver"
 * (frontend) and the "device" (backend). This core implements the device
 * side, hence "vdevice". It is intentionally *not* shared with a future
 * driver-side core (which would be a separate "vdriver" implementation),
 * mirroring Linux where the device side (vdpa/vringh) and the driver side
 * (virtio_config_ops/vring_virtqueue) are distinct interfaces.
 *
 * Ring memory is mapped once by the transport layer; per-buffer bus addresses
 * are translated to virtual addresses on demand through the vdevice_vq::dma
 * accessor, so this core never dereferences a driver-supplied payload address
 * directly.
 */

/* Maximum number of descriptors followed in a single chain */
#define VDEVICE_MAX_DESC_CHAIN		64

/* virtio device status field, see the virtio spec "Device Status Field" */
#define VDEVICE_STATUS_ACKNOWLEDGE	BIT(0)
#define VDEVICE_STATUS_DRIVER		BIT(1)
#define VDEVICE_STATUS_DRIVER_OK	BIT(2)
#define VDEVICE_STATUS_FEATURES_OK	BIT(3)
#define VDEVICE_STATUS_NEEDS_RESET	BIT(6)
#define VDEVICE_STATUS_FAILED		BIT(7)

/*
 * Transport-independent virtio feature bits. VDEVICE_F_START/END bound the
 * device-independent reserved bit range (virtio spec section 6), which
 * vdevice_check_features() scans to reject any feature this core does not
 * understand.
 */
#define VDEVICE_F_START			24	/* inclusive */
#define VDEVICE_F_END			42	/* exclusive */
#define VDEVICE_F_INDIRECT_DESC		28
#define VDEVICE_F_EVENT_IDX		29
#define VDEVICE_F_VERSION_1		32
#define VDEVICE_F_ACCESS_PLATFORM	33
#define VDEVICE_F_IN_ORDER		35
#define VDEVICE_F_RING_RESET		40

struct vdevice;

/*
 * One scatter-gather segment: a driver bus address and length. The payload is
 * mapped on demand via vdevice_vq::dma, never dereferenced directly.
 */
struct vdevice_sg {
	uint64_t addr;
	size_t len;
};

/*
 * struct vdevice_dma - per-device accessor to driver (guest) memory.
 * @cookie:	opaque context passed back to @map / @unmap (the transport
 *		endpoint that owns the mapped areas).
 * @map:	translate a driver bus address to a virtual address valid for
 *		at least @size bytes, or NULL on failure.
 * @unmap:	release a translation previously returned by @map.
 *
 * This is the only path through which the core touches driver payload memory,
 * keeping it independent of how the transport made that memory reachable. It is
 * shared by all of a device's queues (reached through vdevice_vq::vdev).
 */
struct vdevice_dma {
	void *cookie;
	void *(*map)(void *cookie, uint64_t bus_addr, size_t size);
	void (*unmap)(void *cookie, void *va, size_t size);
};

/*
 * struct vdevice_vq - one split virtqueue owned by a device.
 * @qid:		queue index.
 * @ready:		true once the ring pointers are set and the queue is
 *			enabled.
 * @lock:		protects all ring processing of this queue.
 * @desc/@avail/@used:	mapped ring pointers (set by vdevice_set_ring()).
 * @num:		number of descriptors in the ring (queue size).
 * @desc_ba/@avail_ba/@used_ba: bus addresses of the rings, kept for unmap.
 * @notify:		transport callback invoked when the driver kicks this
 *			queue (a "descriptor available" event).
 * @f_version_1/@f_event_idx/@f_indirect/@f_in_order/@f_reset_vq: negotiated
 *			feature cache, latched by vdevice_set_ring().
 * @last_avail_idx:	next avail ring entry the device will consume.
 * @last_used_idx:	device-side copy of used->idx.
 * @last_signalled_used_idx: used->idx at the last signal, for EVENT_IDX.
 * @used_flags:		device's cached copy of used->flags.
 * @avail_idx:		last observed avail->idx.
 * @sgs:		scratch scatter-gather array for one parsed chain; the
 *			sglists returned by vdevice_get_vq_desc() point into it.
 * @vdev:		owning device, for reaching the shared DMA accessor.
 */
struct vdevice_vq {
	int qid;
	bool ready;
	unsigned int lock;

	struct virtq_desc *desc;
	struct virtq_avail *avail;
	struct virtq_used *used;
	uint16_t num;

	uint64_t desc_ba;
	uint64_t avail_ba;
	uint64_t used_ba;

	void (*notify)(struct vdevice *vdev, struct vdevice_vq *vq);

	/* Negotiated feature cache */
	bool f_version_1;
	bool f_event_idx;
	bool f_indirect;
	bool f_in_order;
	bool f_reset_vq;

	uint16_t last_avail_idx;
	uint16_t last_used_idx;
	uint16_t last_signalled_used_idx;

	/* Ring caches */
	uint16_t used_flags;
	uint16_t avail_idx;

	struct vdevice *vdev;

	/*
	 * Scratch scatter-gather array for one parsed chain. The sglists
	 * returned by vdevice_get_vq_desc() point into this storage and are
	 * only valid until the next parse on this queue.
	 */
	struct vdevice_sg sgs[VDEVICE_MAX_DESC_CHAIN];
};

/*
 * struct vdevice_ops - device-type callbacks, provided by e.g. vsock.
 *
 * These mirror the device-side operations of the virtio configuration space
 * (compare Linux struct vdpa_config_ops, the device-side analog of
 * virtio_config_ops).
 *
 * @start:		driver reached DRIVER_OK, bring the device live.
 * @stop:		device is being reset/stopped, quiesce it.
 * @reset_vq:		reset a single queue's device-type state (only invoked
 *			when VIRTIO_F_RING_RESET was negotiated).
 * @gen_count:		return the config-space generation counter.
 * @get:		read @len bytes of config space at @offset.
 * @set:		write @len bytes of config space at @offset; the new
 *			generation counter is returned in *@gen.
 * @get_features:	OR the features the device offers into *@features.
 * @finalize_features:	the driver proposed @features; validate/latch them.
 *			Returns 0 to accept, negative to reject.
 */
struct vdevice_ops {
	int (*start)(struct vdevice *vdev);
	void (*stop)(struct vdevice *vdev);
	void (*reset_vq)(struct vdevice *vdev, struct vdevice_vq *vq);
	uint32_t (*gen_count)(struct vdevice *vdev);
	int (*get)(struct vdevice *vdev, size_t offset, void *buf, size_t len);
	int (*set)(struct vdevice *vdev, size_t offset, const void *buf,
		   size_t len, uint32_t *gen);
	void (*get_features)(struct vdevice *vdev, uint64_t *features);
	int (*finalize_features)(struct vdevice *vdev, uint64_t features);
};

/*
 * struct vdevice - a virtio device (backend) instance.
 * @dev_id/@vendor_id:	virtio device and vendor identifiers.
 * @ops:		device-type callbacks.
 * @signal:		transport callback used to notify the driver that the
 *			used ring advanced on queue @qid.
 * @features:		default features offered by the core (device-specific
 *			bits are added on top via ops->get_features()).
 * @features_neg:	features negotiated with the driver.
 * @num_queues:		number of queues in @vqs.
 * @vqs:		array of @num_queues queues.
 * @dma:		accessor to driver memory, shared by all queues.
 * @status:		virtio device status field.
 * @started:		true between a successful ops->start() and ops->stop().
 * @priv:		device-type private pointer.
 */
struct vdevice {
	uint32_t dev_id;
	uint32_t vendor_id;
	const struct vdevice_ops *ops;
	void (*signal)(struct vdevice *vdev, int qid);

	uint64_t features;
	uint64_t features_neg;

	int num_queues;
	struct vdevice_vq *vqs;
	struct vdevice_dma dma;

	uint8_t status;
	bool started;
	void *priv;
};

/*
 * A scatter-gather list over a parsed descriptor chain. @sg points into the
 * owning queue's scratch array (vdevice_vq::sgs) and is only valid until the
 * next vdevice_get_vq_desc() on that queue.
 */
struct vdevice_sglist {
	struct vdevice_sg *sg;
	uint16_t num;
};

/*
 * A parsed descriptor chain, split into device-readable (out) and
 * device-writable (in) segments, in that spec-mandated order.
 */
struct vdevice_chain {
	uint16_t head;
	struct vdevice_sglist out;	/* device-readable (driver OUT) */
	struct vdevice_sglist in;	/* device-writable (driver IN) */
};

/* Cursor for copying across the segments of a sglist */
struct vdevice_sglist_cursor {
	uint16_t seg;
	size_t off;
};

static inline void vdevice_sglist_cursor_reset(struct vdevice_sglist_cursor *c)
{
	c->seg = 0;
	c->off = 0;
}

/* Total number of bytes described by @sgl */
static inline size_t vdevice_sglist_len(const struct vdevice_sglist *sgl)
{
	size_t len = 0;
	uint16_t i = 0;

	for (i = 0; i < sgl->num; i++)
		len += sgl->sg[i].len;

	return len;
}

/*
 * vdevice_init() - initialise a device instance.
 * @vdev:	device to initialise.
 * @vqs:	caller-owned array of @num_queues queues.
 * @num_queues:	number of queues.
 * @ops:	device-type callbacks.
 *
 * Zeroes the queues, wires their @qid, stores @ops and seeds the core default
 * features (INDIRECT_DESC, EVENT_IDX, VERSION_1). Device-specific features are
 * added on top by the transport through ops->get_features().
 */
void vdevice_init(struct vdevice *vdev, struct vdevice_vq *vqs, int num_queues,
		  const struct vdevice_ops *ops);

/*
 * vdevice_check_features() - core validation of a proposed feature set.
 *
 * Requires VIRTIO_F_VERSION_1 and rejects any device-independent reserved bit
 * this core does not implement. Returns 0 if acceptable, negative otherwise.
 * The device's own finalize_features() is consulted separately (by the
 * transport) for device-specific constraints.
 */
int vdevice_check_features(struct vdevice *vdev, uint64_t features);

/* Read the device status field */
uint8_t vdevice_status_read(struct vdevice *vdev);

/*
 * vdevice_status_write() - drive the device status state machine.
 * @vdev:	device.
 * @status:	new status value written by the driver.
 *
 * Implements the virtio spec status transitions: a write of 0 is a clean reset
 * (stop + reset all queues, no panic); FAILED latches and stops; DRIVER_OK
 * starts the device. Returns 0 on success or a negative value on an illegal
 * transition.
 */
int vdevice_status_write(struct vdevice *vdev, uint8_t status);

/*
 * vdevice_set_ring() - install the mapped ring pointers of a queue.
 * @vdev:	device.
 * @qid:	queue index.
 * @desc/@avail/@used: mapped ring virtual addresses.
 * @desc_ba/@avail_ba/@used_ba: bus addresses of the rings (kept for unmap).
 * @num:	queue size (number of descriptors, must be a power of two).
 *
 * Must be called at FEATURES_OK. Seeds the index/cache state from the guest
 * rings and latches the negotiated per-queue feature bits. The queue is not
 * consumed until it is marked ready with vdevice_set_ring_ready().
 */
int vdevice_set_ring(struct vdevice *vdev, int qid, void *desc, void *avail,
		     void *used, uint64_t desc_ba, uint64_t avail_ba,
		     uint64_t used_ba, uint16_t num);

/* Mark a queue ready (or not) for the device to consume */
void vdevice_set_ring_ready(struct vdevice_vq *vq, bool ready);

/*
 * vdevice_reset_queue() - reset a single queue.
 *
 * Only valid when VIRTIO_F_RING_RESET was negotiated for the queue. Disables
 * the queue, calls the device-type reset_vq(), then clears core queue state.
 * Returns 0 on success or negative on error.
 */
int vdevice_reset_queue(struct vdevice *vdev, int qid);

/*
 * vdevice_get_vq_desc() - pop the next available descriptor chain.
 * @vq:		queue.
 * @chain:	filled in with the head index and out/in sglists (pointing
 *		into vq->sgs).
 *
 * Walks the chain following VIRTQ_DESC_F_NEXT, resolving a trailing
 * VIRTQ_DESC_F_INDIRECT table when INDIRECT_DESC was negotiated, splitting
 * segments into @chain->out (readable, first) and @chain->in (writable),
 * bounded by VDEVICE_MAX_DESC_CHAIN. Returns 0 when a chain was popped, 1 when
 * the ring is empty, or a negative value on a malformed chain.
 */
int vdevice_get_vq_desc(struct vdevice_vq *vq, struct vdevice_chain *chain);

/*
 * vdevice_add_used_one() - publish one consumed chain to the used ring.
 * @vq:		queue.
 * @head:	head descriptor index (from vdevice_get_vq_desc()).
 * @len:	total number of bytes written into the writable buffers.
 */
int vdevice_add_used_one(struct vdevice_vq *vq, uint16_t head, uint32_t len);

/*
 * vdevice_add_used() - publish @n completions to the used ring.
 * @vq:		queue.
 * @elems:	@n used elements (one representative per batch when in-order).
 * @nelems:	@n per-batch sizes (used only when VIRTIO_F_IN_ORDER negotiated).
 * @n:		number of entries.
 */
int vdevice_add_used(struct vdevice_vq *vq, struct virtq_used_elem *elems,
		     uint16_t *nelems, int n);

/*
 * vdevice_signal() - notify the driver if it wants to be, after add_used().
 *
 * Applies the used-event / VIRTQ_AVAIL_F_NO_INTERRUPT logic and, if a
 * notification is due, calls vdev->signal().
 */
void vdevice_signal(struct vdevice *vdev, struct vdevice_vq *vq);

/*
 * vdevice_enable_notify() - ask the driver to kick us on new descriptors.
 *
 * Returns true if the driver added entries in the race window, meaning the
 * caller should re-scan the queue.
 */
bool vdevice_enable_notify(struct vdevice_vq *vq);

/* Suppress driver kicks for this queue */
void vdevice_disable_notify(struct vdevice_vq *vq);

/*
 * Copy between a linear buffer and a parsed sglist, advancing @cur across
 * segments. copy_to() writes into device-writable (in) segments; copy_from()
 * reads from device-readable (out) segments. Return 0 on success or a negative
 * value if the sglist is exhausted before @len bytes.
 */
int vdevice_sglist_copy_to(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			   struct vdevice_sglist_cursor *cur, const void *src,
			   size_t len);
int vdevice_sglist_copy_from(struct vdevice_vq *vq, struct vdevice_sglist *sgl,
			     struct vdevice_sglist_cursor *cur, void *dst,
			     size_t len);

#endif /* __KERNEL_VDEVICE_H */
