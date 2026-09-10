/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025, Linaro Limited
 */

#ifndef __KERNEL_VIRTIO_MSG_H
#define __KERNEL_VIRTIO_MSG_H

#include <kernel/vdevice.h>
#include <stddef.h>
#include <stdint.h>

/*
 * virtio-msg transport and bus layer.
 *
 * This implements the transport-independent part of the virtio-msg protocol
 * (Arm DEN0153): it decodes the 8-byte message header, dispatches transport
 * requests (VIRTIO_MSG_*) onto the device core (struct vdevice) and bus
 * requests (VIRTIO_MSG_BUS_*) onto the per-bus device table. It knows about
 * struct vdevice but nothing about any particular device type (vsock, ...)
 * nor about the underlying carrier (FF-A, ...).
 *
 * A struct virtio_msg_bus represents one connection to a driver (for the FF-A
 * carrier, one non-secure endpoint). The carrier layer owns the bus, provides
 * the memory-area and event callbacks through struct virtio_msg_bus_ops and
 * feeds inbound messages to virtio_msg_recv().
 */

/* Transport message ids (DEN0153, "Virtio Transport Messages") */
#define VIRTIO_MSG_DEVICE_INFO		0x02
#define VIRTIO_MSG_GET_DEV_FEATURES	0x03
#define VIRTIO_MSG_SET_DRV_FEATURES	0x04
#define VIRTIO_MSG_GET_CONFIG		0x05
#define VIRTIO_MSG_SET_CONFIG		0x06
#define VIRTIO_MSG_GET_DEVICE_STATUS	0x07
#define VIRTIO_MSG_SET_DEVICE_STATUS	0x08
#define VIRTIO_MSG_GET_VQUEUE		0x09
#define VIRTIO_MSG_SET_VQUEUE		0x0a
#define VIRTIO_MSG_RESET_VQUEUE		0x0b
#define VIRTIO_MSG_GET_SHM		0x0c
#define VIRTIO_MSG_EVENT_CONFIG		0x40
#define VIRTIO_MSG_EVENT_AVAIL		0x41
#define VIRTIO_MSG_EVENT_USED		0x42

/* Bus message ids (DEN0153, "Virtio Bus Messages") */
#define VIRTIO_MSG_BUS_GET_DEVICES	0x02
#define VIRTIO_MSG_BUS_PING		0x03
#define VIRTIO_MSG_BUS_EVENT_DEVICE	0x40

/* type field: bit0 request(0)/response(1), bit1 transport(0)/bus(1) */
#define VIRTIO_MSG_TYPE_REQUEST		0x0
#define VIRTIO_MSG_TYPE_RESPONSE	0x1
#define VIRTIO_MSG_TYPE_BUS		0x2

#define MSG_TYPE_TRANSPORT_REQUEST	0x0
#define MSG_TYPE_TRANSPORT_RESPONSE	0x1
#define MSG_TYPE_BUS_REQUEST		0x2
#define MSG_TYPE_BUS_RESPONSE		0x3

/* Features are exchanged and stored in 32-bit blocks */
#define VIRTIO_MSG_FEATURE_BLOCK_BITS	32

/*
 * The largest virtio-msg message this backend produces or consumes. The
 * carrier reserves a buffer of at least this size for a request/response.
 */
#define VIRTIO_MSG_MAX_SIZE		96

/* Maximum number of devices attached to a single bus */
#define VIRTIO_MSG_BUS_MAX_DEVS		16

/* struct virtio_msg - 8-byte message header, DEN0153 little-endian layout */
struct virtio_msg {
	uint8_t type;
	uint8_t msg_id;
	uint16_t dev_id;
	uint16_t token;
	uint16_t msg_size;
	uint8_t payload[];
};

struct virtio_msg_bus;
struct virtio_msg_dev;

/*
 * struct virtio_msg_bus_ops - carrier callbacks, provided by the FF-A layer.
 * @map_area:	translate a driver bus address to a VA valid for @len bytes.
 * @unmap_area:	release a translation from @map_area.
 * @send_event_used: notify the driver that a queue's used ring advanced.
 */
struct virtio_msg_bus_ops {
	void *(*map_area)(void *cookie, uint64_t bus_addr, size_t len);
	void (*unmap_area)(void *cookie, void *va, size_t len);
	void (*send_event_used)(struct virtio_msg_dev *vmdev, int qid);
};

/*
 * struct virtio_msg_bus - one driver connection and its device table.
 * @devs:	devices attached to this bus, indexed by dev_id.
 * @ops:	carrier callbacks.
 * @ops_cookie:	opaque carrier context passed back to @ops (the endpoint).
 */
struct virtio_msg_bus {
	struct virtio_msg_dev *devs[VIRTIO_MSG_BUS_MAX_DEVS];
	const struct virtio_msg_bus_ops *ops;
	void *ops_cookie;
};

/*
 * struct virtio_msg_dev - a device as seen on a virtio-msg bus.
 * @vdev:	the device core; first member so container_of() works.
 * @dev_id:	index of this device in bus->devs[].
 * @bus:	owning bus (carries the carrier ops and cookie).
 */
struct virtio_msg_dev {
	struct vdevice vdev;
	uint16_t dev_id;
	struct virtio_msg_bus *bus;
};

/*
 * struct virtio_bus_driver - a device-type registration.
 *
 * The analog of QTEE's ffa_bus_driver. When a new bus (driver connection)
 * appears, the framework allocates one struct virtio_msg_dev per registered
 * driver, attaches it to the bus and calls init() with the device core it
 * wraps; deinit() tears that device down. The device type only ever sees
 * struct vdevice: it never needs to know about the virtio-msg transport.
 *
 * init() typically calls vdevice_init() on @vdev, sets @vdev->dev_id /
 * ->vendor_id and the per-queue notify callbacks, and stashes its private
 * state in @vdev->priv. It returns 0 on success or negative to decline.
 */
struct virtio_bus_driver {
	int (*init)(struct vdevice *vdev);
	void (*deinit)(struct vdevice *vdev);
};

/* Initialise an empty bus */
void virtio_msg_bus_init(struct virtio_msg_bus *bus,
			 const struct virtio_msg_bus_ops *ops, void *cookie);

/*
 * virtio_msg_bus_attach_driver() - instantiate one driver's device on a bus.
 *
 * Allocates a struct virtio_msg_dev, attaches it to @bus, wires the standard
 * used-ring signal bridge and calls drv->init() on the wrapped struct vdevice.
 * Returns 0 on success or negative if the driver declined or no slot/memory
 * was available.
 */
int virtio_msg_bus_attach_driver(struct virtio_msg_bus *bus,
				 struct virtio_bus_driver *drv);

/* Look up a device by its dev_id, or NULL */
struct virtio_msg_dev *virtio_msg_bus_device(struct virtio_msg_bus *bus,
					     uint16_t dev_id);

/*
 * virtio_msg_recv() - process one inbound message in place.
 * @msg:	message buffer, updated with the response.
 * @max_size:	usable size of @msg (>= VIRTIO_MSG_MAX_SIZE).
 * @bus:	the bus the message arrived on.
 *
 * Dispatches a transport or bus request and rewrites @msg as the response
 * (type response bit set, token echoed). Never panics on a bad or unknown
 * message: it returns a null/error response instead.
 */
void virtio_msg_recv(struct virtio_msg *msg, size_t max_size,
		     struct virtio_msg_bus *bus);

/*
 * virtio_msg_event_used() - device-originated used-ring notification.
 *
 * Called from a device's signal callback; forwards to the carrier's
 * send_event_used().
 */
void virtio_msg_event_used(struct virtio_msg_dev *vmdev, int qid);

#endif /* __KERNEL_VIRTIO_MSG_H */
