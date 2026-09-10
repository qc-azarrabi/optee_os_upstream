// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 */

#include <kernel/notif.h>
#include <kernel/vdevice.h>
#include <kernel/virtio_msg.h>
#include <malloc.h>
#include <string.h>
#include <trace.h>
#include <util.h>

/*
 * Payload layouts follow the virtio-msg specification (DEN0153). Every field
 * is little-endian; OP-TEE core is little-endian on all supported targets so
 * the packed structs are accessed directly.
 */

struct msg_bus_get_devices {
	uint16_t offset;
	uint16_t num;
} __packed;

struct msg_bus_get_devices_resp {
	uint16_t offset;
	uint16_t num;
	uint16_t next_offset;
	uint8_t devices[];
} __packed;

struct msg_bus_ping {
	uint32_t data;
} __packed;

struct msg_device_info_resp {
	uint32_t device_id;
	uint32_t vendor_id;
	uint32_t num_feature_bits;
	uint32_t config_size;
	uint32_t max_vq_count;
	uint16_t admin_vq_start_idx;
	uint16_t admin_vq_count;
} __packed;

struct msg_features {
	uint32_t index;
	uint32_t num;
	uint8_t features[];
} __packed;

struct msg_get_config {
	uint32_t offset;
	uint32_t size;
} __packed;

struct msg_config_resp {
	uint32_t generation;
	uint32_t offset;
	uint32_t size;
	uint8_t config[];
} __packed;

struct msg_set_config {
	uint32_t generation;
	uint32_t offset;
	uint32_t size;
	uint8_t config[];
} __packed;

struct msg_device_status {
	uint32_t status;
} __packed;

struct msg_get_vqueue {
	uint32_t index;
} __packed;

struct msg_get_vqueue_resp {
	uint32_t index;
	uint32_t max_size;
	uint32_t size;
	uint32_t reserved;
	uint64_t descriptor_addr;
	uint64_t driver_addr;
	uint64_t device_addr;
} __packed;

struct msg_set_vqueue {
	uint32_t index;
	uint32_t unused;
	uint32_t size;
	uint32_t reserved;
	uint64_t descriptor_addr;
	uint64_t driver_addr;
	uint64_t device_addr;
} __packed;

struct msg_reset_vqueue {
	uint32_t index;
} __packed;

struct msg_event_avail {
	uint32_t index;
	uint32_t next_offset_wrap;
} __packed;

struct msg_event_used {
	uint32_t index;
} __packed;

void virtio_msg_bus_init(struct virtio_msg_bus *bus,
			 const struct virtio_msg_bus_ops *ops, void *cookie)
{
	memset(bus, 0, sizeof(*bus));
	bus->ops = ops;
	bus->ops_cookie = cookie;
}

/*
 * Guest-memory accessor handed to the device core. The core passes back the
 * owning vdevice; recover the bus from it and defer to the carrier's area ops.
 */
static void *virtio_msg_dma_map(struct vdevice *vdev, uint64_t bus_addr,
				size_t size)
{
	struct virtio_msg_dev *vmdev =
		container_of(vdev, struct virtio_msg_dev, vdev);
	struct virtio_msg_bus *bus = vmdev->bus;

	return bus->ops->map_area(bus->ops_cookie, bus_addr, size);
}

static void virtio_msg_dma_unmap(struct vdevice *vdev, void *va, size_t size)
{
	struct virtio_msg_dev *vmdev =
		container_of(vdev, struct virtio_msg_dev, vdev);
	struct virtio_msg_bus *bus = vmdev->bus;

	bus->ops->unmap_area(bus->ops_cookie, va, size);
}

/* Attach a device to the first free bus slot; sets vmdev->dev_id and ->bus */
static int virtio_msg_bus_add(struct virtio_msg_bus *bus,
			      struct virtio_msg_dev *vmdev)
{
	uint16_t n = 0;

	for (n = 0; n < VIRTIO_MSG_BUS_MAX_DEVS; n++) {
		if (!bus->devs[n]) {
			bus->devs[n] = vmdev;
			vmdev->dev_id = n;
			vmdev->bus = bus;
			/* Give the device a memory accessor for its buffers */
			vmdev->vdev.dma.map = virtio_msg_dma_map;
			vmdev->vdev.dma.unmap = virtio_msg_dma_unmap;
			return 0;
		}
	}

	return -1;
}

/*
 * Bridge the device core's used-ring signal to the virtio-msg carrier. The
 * core only knows about struct vdevice; recover the transport wrapper and
 * forward to the carrier's EVENT_USED path.
 */
static void virtio_msg_dev_signal(struct vdevice *vdev, int qid)
{
	struct virtio_msg_dev *vmdev =
		container_of(vdev, struct virtio_msg_dev, vdev);

	virtio_msg_event_used(vmdev, qid);
}

int virtio_msg_bus_attach_driver(struct virtio_msg_bus *bus,
				 struct virtio_msg_bus_driver *drv)
{
	struct virtio_msg_dev *vmdev = calloc(1, sizeof(*vmdev));

	if (!vmdev)
		return -1;

	/*
	 * Own the transport wrapper here so the device type only deals with
	 * struct vdevice. Attach it to the bus, then let the driver initialise
	 * the device core it wraps (vdevice_init(), queues, private state).
	 */
	if (virtio_msg_bus_add(bus, vmdev)) {
		free(vmdev);
		return -1;
	}
	vmdev->vdev.signal = virtio_msg_dev_signal;

	if (drv->init(&vmdev->vdev)) {
		bus->devs[vmdev->dev_id] = NULL;
		free(vmdev);
		return -1;
	}

	return 0;
}

struct virtio_msg_dev *virtio_msg_bus_device(struct virtio_msg_bus *bus,
					     uint16_t dev_id)
{
	if (dev_id >= VIRTIO_MSG_BUS_MAX_DEVS)
		return NULL;

	return bus->devs[dev_id];
}

/* Build a minimal null response for a message we cannot service */
static void virtio_msg_null_resp(struct virtio_msg *msg)
{
	msg->dev_id = 0;
	msg->msg_size = sizeof(*msg);
}

/*
 * Bus requests
 */

static void handle_bus_get_devices(struct virtio_msg *msg, size_t max_size,
				   struct virtio_msg_bus *bus)
{
	struct msg_bus_get_devices *req = (void *)msg->payload;
	struct msg_bus_get_devices_resp *resp = (void *)msg->payload;
	uint16_t offset = 0;
	uint16_t num = 0;
	uint16_t n = 0;
	size_t bytes = 0;

	if (msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		virtio_msg_null_resp(msg);
		return;
	}

	offset = req->offset;
	num = req->num;
	/* Report device presence as a bitmap, one bit per dev_id */
	bytes = (num + 7) / 8;
	if (offset || sizeof(*msg) + sizeof(*resp) + bytes > max_size) {
		virtio_msg_null_resp(msg);
		return;
	}

	memset(resp, 0, sizeof(*resp) + bytes);
	for (n = 0; n < num && n < VIRTIO_MSG_BUS_MAX_DEVS; n++)
		if (bus->devs[n])
			resp->devices[n / 8] |= BIT(n % 8);

	resp->offset = 0;
	resp->num = num;
	resp->next_offset = 0;
	msg->msg_size = sizeof(*msg) + sizeof(*resp) + bytes;
}

static void handle_bus_ping(struct virtio_msg *msg)
{
	struct msg_bus_ping *req = (void *)msg->payload;

	/* Echo the ping payload back unchanged */
	if (msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		virtio_msg_null_resp(msg);
		return;
	}
	/* payload already in place, size unchanged */
}

static void virtio_msg_recv_bus(struct virtio_msg *msg, size_t max_size,
				struct virtio_msg_bus *bus)
{
	switch (msg->msg_id) {
	case VIRTIO_MSG_BUS_GET_DEVICES:
		handle_bus_get_devices(msg, max_size, bus);
		break;
	case VIRTIO_MSG_BUS_PING:
		handle_bus_ping(msg);
		break;
	default:
		DMSG("Unhandled bus msg_id %#"PRIx8, msg->msg_id);
		virtio_msg_null_resp(msg);
		break;
	}
}

/*
 * Transport requests
 */

static void handle_device_info(struct virtio_msg *msg,
			       struct virtio_msg_dev *vmdev)
{
	struct msg_device_info_resp *resp = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;

	memset(resp, 0, sizeof(*resp));
	resp->device_id = vdev->dev_id;
	resp->vendor_id = vdev->vendor_id;
	resp->num_feature_bits = 64;
	resp->max_vq_count = vdev->num_queues;
	if (vdev->ops->gen_count)
		resp->config_size = 0;	/* filled per device type via GET_CONFIG */
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_get_dev_features(struct virtio_msg *msg,
				    struct virtio_msg_dev *vmdev)
{
	struct msg_features *v = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	uint64_t feat = vdev->features;
	uint32_t blocks[2] = { 0 };
	uint32_t index = 0;
	uint32_t num = 0;
	uint32_t i = 0;

	if (msg->msg_size != sizeof(*msg) + sizeof(*v)) {
		virtio_msg_null_resp(msg);
		return;
	}

	/* Report core defaults plus the device-specific feature bits */
	if (vdev->ops->get_features)
		vdev->ops->get_features(vdev, &feat);

	index = v->index;
	num = v->num;
	blocks[0] = feat & 0xffffffff;
	blocks[1] = feat >> 32;

	/*
	 * Features are exchanged in 32-bit blocks. Report @num blocks starting
	 * at @index; out-of-range blocks read as zero (virtio spec 4.4.2.4).
	 */
	if (sizeof(*msg) + sizeof(*v) + num * sizeof(uint32_t) >
	    VIRTIO_MSG_MAX_SIZE) {
		virtio_msg_null_resp(msg);
		return;
	}

	for (i = 0; i < num; i++) {
		uint32_t word = 0;

		if (index + i < ARRAY_SIZE(blocks))
			word = blocks[index + i];
		memcpy(v->features + i * sizeof(word), &word, sizeof(word));
	}

	msg->msg_size = sizeof(*msg) + sizeof(*v) + num * sizeof(uint32_t);
}

static void handle_set_drv_features(struct virtio_msg *msg,
				    struct virtio_msg_dev *vmdev)
{
	struct msg_features *v = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	uint64_t feat = 0;
	uint32_t index = 0;
	uint32_t num = 0;
	uint32_t i = 0;

	if (msg->msg_size < sizeof(*msg) + sizeof(*v)) {
		virtio_msg_null_resp(msg);
		return;
	}

	index = v->index;
	num = v->num;
	if (msg->msg_size != sizeof(*msg) + sizeof(*v) +
			     num * sizeof(uint32_t)) {
		virtio_msg_null_resp(msg);
		return;
	}

	/*
	 * Assemble the proposed 64-bit feature set from @num 32-bit blocks. Any
	 * block outside the 64-bit range is a malformed request.
	 */
	for (i = 0; i < num; i++) {
		uint32_t word = 0;

		if (index + i >= 2) {
			virtio_msg_null_resp(msg);
			return;
		}
		memcpy(&word, v->features + i * sizeof(word), sizeof(word));
		feat |= (uint64_t)word << ((index + i) * 32);
	}

	/*
	 * Validate against the core-supported set and let the device reject
	 * unsupported combinations before latching the negotiated features.
	 * A rejected proposal simply leaves features_neg unchanged; the driver
	 * will fail to reach FEATURES_OK.
	 */
	if (vdevice_check_features(vdev, feat))
		goto out;
	if (vdev->ops->finalize_features &&
	    vdev->ops->finalize_features(vdev, feat))
		goto out;

	vdev->features_neg = feat;
out:
	msg->msg_size = sizeof(*msg);
}

static void handle_get_config(struct virtio_msg *msg,
			      struct virtio_msg_dev *vmdev)
{
	struct msg_get_config *req = (void *)msg->payload;
	struct msg_config_resp *resp = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	uint32_t offset = 0;
	uint32_t size = 0;

	if (msg->msg_size != sizeof(*msg) + sizeof(*req) || !vdev->ops->get) {
		virtio_msg_null_resp(msg);
		return;
	}

	offset = req->offset;
	size = req->size;
	if (size > VIRTIO_MSG_MAX_SIZE - sizeof(*msg) - sizeof(*resp)) {
		virtio_msg_null_resp(msg);
		return;
	}

	memset(resp, 0, sizeof(*resp));
	if (vdev->ops->get(vdev, offset, resp->config, size)) {
		virtio_msg_null_resp(msg);
		return;
	}

	if (vdev->ops->gen_count)
		resp->generation = vdev->ops->gen_count(vdev);
	resp->offset = offset;
	resp->size = size;
	msg->msg_size = sizeof(*msg) + sizeof(*resp) + size;
}

static void handle_set_config(struct virtio_msg *msg,
			      struct virtio_msg_dev *vmdev)
{
	struct msg_set_config *req = (void *)msg->payload;
	struct msg_config_resp *resp = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	uint32_t offset = 0;
	uint32_t size = 0;
	uint32_t gen = 0;

	if (msg->msg_size < sizeof(*msg) + sizeof(*req) || !vdev->ops->set) {
		virtio_msg_null_resp(msg);
		return;
	}

	offset = req->offset;
	size = req->size;
	if (msg->msg_size != sizeof(*msg) + sizeof(*req) + size) {
		virtio_msg_null_resp(msg);
		return;
	}

	if (vdev->ops->set(vdev, offset, req->config, size, &gen)) {
		virtio_msg_null_resp(msg);
		return;
	}

	memset(resp, 0, sizeof(*resp));
	resp->generation = gen;
	resp->offset = offset;
	resp->size = size;
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_get_device_status(struct virtio_msg *msg,
				     struct virtio_msg_dev *vmdev)
{
	struct msg_device_status *resp = (void *)msg->payload;

	resp->status = vdevice_status_read(&vmdev->vdev);
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_set_device_status(struct virtio_msg *msg,
				     struct virtio_msg_dev *vmdev)
{
	struct msg_device_status *v = (void *)msg->payload;

	if (msg->msg_size != sizeof(*msg) + sizeof(*v)) {
		virtio_msg_null_resp(msg);
		return;
	}

	/*
	 * Drive the status state machine. A status of 0 is a clean device
	 * reset, handled without panicking.
	 */
	vdevice_status_write(&vmdev->vdev, v->status);
	v->status = vdevice_status_read(&vmdev->vdev);
	msg->msg_size = sizeof(*msg) + sizeof(*v);
}

static void handle_get_vqueue(struct virtio_msg *msg,
			      struct virtio_msg_dev *vmdev)
{
	struct msg_get_vqueue *req = (void *)msg->payload;
	struct msg_get_vqueue_resp *resp = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	uint32_t index = 0;

	if (msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		virtio_msg_null_resp(msg);
		return;
	}

	index = req->index;
	memset(resp, 0, sizeof(*resp));
	resp->index = index;
	if (index < (uint32_t)vdev->num_queues) {
		struct vdevice_vq *vq = &vdev->vqs[index];

		/* Max descriptors we can follow in one chain */
		resp->max_size = VDEVICE_MAX_DESC_CHAIN;
		resp->size = vq->num;
		resp->descriptor_addr = vq->desc_ba;
		resp->driver_addr = vq->avail_ba;
		resp->device_addr = vq->used_ba;
	}
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_set_vqueue(struct virtio_msg *msg,
			      struct virtio_msg_dev *vmdev)
{
	struct msg_set_vqueue *req = (void *)msg->payload;
	struct virtio_msg_bus *bus = vmdev->bus;
	struct vdevice *vdev = &vmdev->vdev;
	struct vdevice_vq *vq = NULL;
	uint32_t index = 0;
	size_t desc_len = 0;
	size_t avail_len = 0;
	size_t used_len = 0;
	void *desc = NULL;
	void *avail = NULL;
	void *used = NULL;

	if (msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		virtio_msg_null_resp(msg);
		return;
	}

	index = req->index;
	if (index >= (uint32_t)vdev->num_queues || !req->size) {
		virtio_msg_null_resp(msg);
		return;
	}
	vq = &vdev->vqs[index];

	/* Split virtqueue region sizes, see the virtio spec ring layout */
	desc_len = req->size * sizeof(struct virtq_desc);
	avail_len = sizeof(struct virtq_avail) +
		    (req->size + 1) * sizeof(uint16_t);
	used_len = sizeof(struct virtq_used) +
		   req->size * sizeof(struct virtq_used_elem) + sizeof(uint16_t);

	desc = bus->ops->map_area(bus->ops_cookie, req->descriptor_addr,
				  desc_len);
	avail = bus->ops->map_area(bus->ops_cookie, req->driver_addr,
				   avail_len);
	used = bus->ops->map_area(bus->ops_cookie, req->device_addr, used_len);
	if (!desc || !avail || !used)
		goto err_unmap;

	if (vdevice_set_ring(vdev, index, desc, avail, used,
			     req->descriptor_addr, req->driver_addr,
			     req->device_addr, req->size))
		goto err_unmap;

	vdevice_set_ring_ready(vq, true);

	msg->msg_size = sizeof(*msg);
	return;

err_unmap:
	if (used)
		bus->ops->unmap_area(bus->ops_cookie, used, used_len);
	if (avail)
		bus->ops->unmap_area(bus->ops_cookie, avail, avail_len);
	if (desc)
		bus->ops->unmap_area(bus->ops_cookie, desc, desc_len);
	virtio_msg_null_resp(msg);
}

static void handle_reset_vqueue(struct virtio_msg *msg,
				struct virtio_msg_dev *vmdev)
{
	struct msg_reset_vqueue *req = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;

	if (msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		virtio_msg_null_resp(msg);
		return;
	}

	if (req->index < (uint32_t)vdev->num_queues)
		vdevice_reset_queue(vdev, req->index);
	msg->msg_size = sizeof(*msg);
}

static void handle_event_avail(struct virtio_msg *msg,
			       struct virtio_msg_dev *vmdev)
{
	struct msg_event_avail *ev = (void *)msg->payload;
	struct vdevice *vdev = &vmdev->vdev;
	struct vdevice_vq *vq = NULL;

	if (msg->msg_size != sizeof(*msg) + sizeof(*ev)) {
		virtio_msg_null_resp(msg);
		return;
	}

	if (ev->index < (uint32_t)vdev->num_queues) {
		vq = &vdev->vqs[ev->index];
		if (vq->ready) {
			if (vq->notify)
				vq->notify(vdev, vq);
			else
				notif_send_async(NOTIF_VALUE_DO_BOTTOM_HALF, 0);
		}
	}
	msg->msg_size = sizeof(*msg);
}

static void virtio_msg_recv_transport(struct virtio_msg *msg,
				      struct virtio_msg_bus *bus)
{
	struct virtio_msg_dev *vmdev = virtio_msg_bus_device(bus, msg->dev_id);

	if (!vmdev) {
		DMSG("No device %#"PRIx16, msg->dev_id);
		virtio_msg_null_resp(msg);
		return;
	}

	switch (msg->msg_id) {
	case VIRTIO_MSG_DEVICE_INFO:
		handle_device_info(msg, vmdev);
		break;
	case VIRTIO_MSG_GET_DEV_FEATURES:
		handle_get_dev_features(msg, vmdev);
		break;
	case VIRTIO_MSG_SET_DRV_FEATURES:
		handle_set_drv_features(msg, vmdev);
		break;
	case VIRTIO_MSG_GET_CONFIG:
		handle_get_config(msg, vmdev);
		break;
	case VIRTIO_MSG_SET_CONFIG:
		handle_set_config(msg, vmdev);
		break;
	case VIRTIO_MSG_GET_DEVICE_STATUS:
		handle_get_device_status(msg, vmdev);
		break;
	case VIRTIO_MSG_SET_DEVICE_STATUS:
		handle_set_device_status(msg, vmdev);
		break;
	case VIRTIO_MSG_GET_VQUEUE:
		handle_get_vqueue(msg, vmdev);
		break;
	case VIRTIO_MSG_SET_VQUEUE:
		handle_set_vqueue(msg, vmdev);
		break;
	case VIRTIO_MSG_RESET_VQUEUE:
		handle_reset_vqueue(msg, vmdev);
		break;
	case VIRTIO_MSG_EVENT_AVAIL:
		handle_event_avail(msg, vmdev);
		break;
	default:
		DMSG("Unhandled transport msg_id %#"PRIx8, msg->msg_id);
		virtio_msg_null_resp(msg);
		break;
	}
}

void virtio_msg_recv(struct virtio_msg *msg, size_t max_size,
		     struct virtio_msg_bus *bus)
{
	bool is_bus = msg->type & VIRTIO_MSG_TYPE_BUS;

	if (is_bus)
		virtio_msg_recv_bus(msg, max_size, bus);
	else
		virtio_msg_recv_transport(msg, bus);

	/* Turn the request into a response, echoing the token */
	msg->type |= VIRTIO_MSG_TYPE_RESPONSE;
}

void virtio_msg_event_used(struct virtio_msg_dev *vmdev, int qid)
{
	struct virtio_msg_bus *bus = vmdev->bus;
	uint8_t raw_msg[sizeof(struct virtio_msg) + sizeof(struct msg_event_used)];
	struct virtio_msg *msg = (void *)raw_msg;
	struct msg_event_used *event = (void *)msg->payload;

	if (!bus->ops->notify)
		return;

	memset(msg, 0, sizeof(raw_msg));
	msg->type = VIRTIO_MSG_TYPE_REQUEST;
	msg->msg_id = VIRTIO_MSG_EVENT_USED;
	msg->dev_id = vmdev->dev_id;
	msg->msg_size = sizeof(raw_msg);
	event->index = qid;

	bus->ops->notify(bus, vmdev, msg);
}
