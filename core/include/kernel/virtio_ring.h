/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025, Linaro Limited
 */

#ifndef __KERNEL_VIRTIO_RING_H
#define __KERNEL_VIRTIO_RING_H

#include <stdint.h>

/*
 * The virtio split virtqueue layout (virtio spec v1.x, section "Split
 * Virtqueues"). These structures describe the on-ring ABI shared between a
 * virtio driver and device, independent of which side reads or writes each
 * ring. They are kept in this neutral header so that both the device
 * (backend) core and a possible future driver-side core can use them without
 * depending on either one.
 */

/* This marks a buffer as continuing via the next field. */
#define VIRTQ_DESC_F_NEXT	1
/* This marks a buffer as write-only (otherwise read-only). */
#define VIRTQ_DESC_F_WRITE	2
/* This means the buffer contains a list of buffer descriptors. */
#define VIRTQ_DESC_F_INDIRECT	4

/*
 * The device uses this in used->flags to advise the driver: don't kick me
 * when you add a buffer. It's unreliable, so it's simply an optimization.
 */
#define VIRTQ_USED_F_NO_NOTIFY		1
/*
 * The driver uses this in avail->flags to advise the device: don't interrupt
 * me when you consume a buffer. It's unreliable, so it's simply an
 * optimization.
 */
#define VIRTQ_AVAIL_F_NO_INTERRUPT	1

struct virtq_desc {
	uint64_t addr;	/* Buffer address */
	uint32_t len;	/* Buffer length */
	uint16_t flags;	/* Flags depending on descriptor type */
	uint16_t next;	/* Next field if flags & VIRTQ_DESC_F_NEXT */
};

struct virtq_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[];
	/* Only if VIRTIO_F_EVENT_IDX: uint16_t used_event; */
};

struct virtq_used_elem {
	uint32_t id;	/* Index of start of used descriptor chain */
	uint32_t len;	/* Total length written to the descriptor chain */
};

struct virtq_used {
	uint16_t flags;
	uint16_t idx;
	struct virtq_used_elem ring[];
	/* Only if VIRTIO_F_EVENT_IDX: uint16_t avail_event; */
};

#endif /* __KERNEL_VIRTIO_RING_H */
