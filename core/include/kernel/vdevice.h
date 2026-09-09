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
 * accessor, so this core never dereferences a driver-supplied address
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

/* Transport-independent virtio feature bits used by this core */
#define VDEVICE_F_INDIRECT_DESC		28
#define VDEVICE_F_EVENT_IDX		29
#define VDEVICE_F_VERSION_1		32
#define VDEVICE_F_IN_ORDER		35

struct vdevice;

/*
 * struct vdevice_dma - per-queue accessor to driver (guest) memory.
 * @cookie:	opaque context passed back to @map / @unmap (the transport
 *		endpoint that owns the mapped areas).
 * @map:	translate a driver bus address to a virtual address valid for
 *		at least @size bytes, or NULL on failure.
 * @unmap:	release a translation previously returned by @map.
 *
 * This is the only path through which the core touches driver memory, keeping
 * it independent of how the transport made that memory reachable.
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
 * @lock:		protects the used-ring producer state of this queue.
 * @desc/@avail/@used:	mapped ring pointers (set by vdevice_set_ring()).
 * @num:		number of descriptors in the ring (queue size).
 * @desc_ba/@avail_ba/@used_ba: bus addresses of the rings, kept for unmap.
 * @last_avail_idx:	next avail ring entry the device will consume.
 * @last_used_idx:	device-side copy of used->idx.
 * @signalled_used_idx:	used->idx at the last signal, for EVENT_IDX.
 * @signalled_used_valid: whether @signalled_used_idx has been set yet.
 * @f_event_idx/@f_indirect/@f_in_order: negotiated feature cache.
 * @notify:		transport callback invoked when the driver kicks this
 *			queue (a "descriptor available" event).
 * @dma:		accessor to driver memory for this queue's buffers.
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

	uint16_t last_avail_idx;
	uint16_t last_used_idx;
	uint16_t signalled_used_idx;
	bool signalled_used_valid;

	bool f_event_idx;
	bool f_indirect;
	bool f_in_order;

	void (*notify)(struct vdevice *vdev, struct vdevice_vq *vq);
	struct vdevice_dma dma;
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
 * @reset_vq:		reset a single queue's device-type state.
 * @gen_count:		return the config-space generation counter.
 * @get:		read @len bytes of config space at @offset.
 * @set:		write @len bytes of config space at @offset; the new
 *			generation counter is returned in *@gen.
 * @get_features:	return the features the device offers.
 * @finalize_features:	the driver accepted @features; latch them.
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
 * @features:		features offered by the device (from ops->get_features).
 * @features_neg:	features negotiated with the driver.
 * @num_queues:		number of queues in @vqs.
 * @vqs:		array of @num_queues queues.
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

	uint8_t status;
	bool started;
	void *priv;
};

/*
 * A parsed descriptor chain, split into device-readable (out) and
 * device-writable (in) segments. Each segment records a driver bus address
 * and length; the buffer is mapped on demand via vdevice_vq::dma.
 */
struct vdevice_sg {
	uint64_t addr;
	size_t len;
};

struct vdevice_sglist {
	struct vdevice_sg sg[VDEVICE_MAX_DESC_CHAIN];
	uint16_t num;
	size_t total;
};

struct vdevice_chain {
	uint16_t head;
	struct vdevice_sglist out;	/* device-readable */
	struct vdevice_sglist in;	/* device-writable */
};

/* Cursor for copying across the segments of a sglist */
struct vdevice_sglist_cursor {
	uint16_t seg;
	size_t off;
};

/*
 * vdevice_init() - initialise a device instance.
 * @vdev:	device to initialise.
 * @vqs:	caller-owned array of @num_queues queues.
 * @num_queues:	number of queues.
 * @ops:	device-type callbacks.
 *
 * Zeroes the queues, wires their @qid, stores @ops and queries the
 * device-offered features via ops->get_features().
 */
void vdevice_init(struct vdevice *vdev, struct vdevice_vq *vqs, int num_queues,
		  const struct vdevice_ops *ops);

/* Read the device status field */
uint8_t vdevice_status_read(struct vdevice *vdev);

/*
 * vdevice_status_write() - drive the device status state machine.
 * @vdev:	device.
 * @status:	new status value written by the driver.
 *
 * Latches negotiated features on FEATURES_OK, starts the device on DRIVER_OK
 * and, when @status is 0, performs a clean reset (stop + reset all queues).
 * Returns 0 on success or a negative value on an illegal transition.
 */
int vdevice_status_write(struct vdevice *vdev, uint8_t status);

/*
 * vdevice_set_ring() - install the mapped ring pointers of a queue.
 * @vdev:	device.
 * @qid:	queue index.
 * @desc/@avail/@used: mapped ring virtual addresses.
 * @desc_ba/@avail_ba/@used_ba: bus addresses of the rings (kept for unmap).
 * @num:	queue size (number of descriptors).
 *
 * Caches the negotiated per-queue feature bits. The queue is not consumed
 * until it is marked ready with vdevice_set_ring_ready().
 */
int vdevice_set_ring(struct vdevice *vdev, int qid, void *desc, void *avail,
		     void *used, uint64_t desc_ba, uint64_t avail_ba,
		     uint64_t used_ba, uint16_t num);

/* Mark a queue ready (or not) for the device to consume */
void vdevice_set_ring_ready(struct vdevice_vq *vq, bool ready);

/* Reset a single queue: device-type reset_vq() plus core producer state */
int vdevice_reset_queue(struct vdevice *vdev, int qid);

/*
 * vdevice_get_vq_desc() - pop the next available descriptor chain.
 * @vq:		queue.
 * @chain:	filled in with the head index and out/in sglists.
 *
 * Walks the chain following VIRTQ_DESC_F_NEXT and VIRTQ_DESC_F_INDIRECT,
 * splitting segments into @chain->out (readable) and @chain->in (writable),
 * bounded by VDEVICE_MAX_DESC_CHAIN. Returns 1 when a chain was popped, 0 when
 * the ring is empty, or a negative value on a malformed chain.
 */
int vdevice_get_vq_desc(struct vdevice_vq *vq, struct vdevice_chain *chain);

/*
 * vdevice_add_used() - publish a consumed chain to the used ring.
 * @vq:		queue.
 * @head:	head descriptor index (from vdevice_get_vq_desc()).
 * @len:	total number of bytes written into the writable buffers.
 */
int vdevice_add_used(struct vdevice_vq *vq, uint16_t head, uint32_t len);

/*
 * vdevice_signal() - notify the driver if it wants to be, after add_used().
 *
 * Applies the used-event / VIRTQ_USED_F_NO_NOTIFY logic and, if a notification
 * is due, calls vdev->signal().
 */
void vdevice_signal(struct vdevice *vdev, struct vdevice_vq *vq);

/* Toggle "please notify me on new descriptors" back towards the driver */
bool vdevice_enable_notify(struct vdevice_vq *vq);
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
