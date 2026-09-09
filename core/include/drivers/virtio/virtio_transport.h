/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 */
#ifndef __DRIVERS_VIRTIO_VIRTIO_TRANSPORT_H
#define __DRIVERS_VIRTIO_VIRTIO_TRANSPORT_H

#include <tee_api_types.h>
#include <types_ext.h>

/*
 * Device-initiated event kinds, mirroring the virtio-msg EVENT_* opcodes.
 * Used with virtio_transport_ops::send_event().
 */
#define VIRTIO_TRANSPORT_EVENT_USED	0x1
#define VIRTIO_TRANSPORT_EVENT_CONFIG	0x2

/*
 * struct virtio_transport_ops - seam between the transport-agnostic virtio
 * backend core/codec and the transport binding (e.g. virtio-msg over FF-A).
 *
 * The backend core (virtio_dev.c) and the message codec (virtio_msg.c) never
 * reference the transport directly; all bus-address translation and mapping
 * goes through this table, provided by the transport binding (virtio_msg_ffa.c)
 * and threaded down into each virtqueue.
 *
 * @bus_addr_to_va:	Translate a bus address (area_id[63:48] | offset[47:0])
 *			into a mapped virtual address valid for @len bytes, or
 *			NULL on failure.
 * @map_area:		Take a reference on the mapping backing @bus_addr.
 *			Paired with @unmap_area.
 * @unmap_area:		Release a reference taken with @map_area.
 * @send_event:		Send a device->driver event. With the EVENT_POLL model
 *			this is a no-op initially; NULL is permitted.
 * @max_msg_payload:	Maximum message payload the transport can carry in a
 *			single message (96 for FF-A direct messaging).
 */
struct virtio_transport_ops {
	void *(*bus_addr_to_va)(const struct virtio_transport_ops *ops,
				uint64_t bus_addr, size_t len);
	TEE_Result (*map_area)(const struct virtio_transport_ops *ops,
			       uint64_t bus_addr);
	void (*unmap_area)(const struct virtio_transport_ops *ops,
			   uint64_t bus_addr);
	TEE_Result (*send_event)(const struct virtio_transport_ops *ops,
				 uint16_t dev_num, uint8_t event_type);
	size_t max_msg_payload;
};

#endif /*__DRIVERS_VIRTIO_VIRTIO_TRANSPORT_H*/
