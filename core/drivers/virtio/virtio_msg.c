// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * virtio-msg message codec (transport- and role-neutral). Decodes virtio-msg
 * transport and bus messages and fills the response in place. Bus-address
 * translation and virtqueue mapping go through struct virtio_transport_ops,
 * supplied by the transport binding.
 */

#include <drivers/virtio/virtio_dev.h>
#include <drivers/virtio/virtio_msg.h>
#include <drivers/virtio/virtio_transport.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <string.h>
#include <trace.h>

struct virtio_msg_get_devices_req {
	uint16_t offset;
	uint16_t count;
};

struct virtio_msg_get_devices_resp {
	uint16_t offset;
	uint16_t count;
	uint16_t next_offset;
};

struct virtio_msg_ping {
	uint32_t data;
};

struct virtio_msg_get_device_info_resp {
	uint32_t device_id;
	uint32_t vendor_id;
	uint32_t feature_bit_count;
	uint32_t config_size;
	uint32_t max_vq_count;
	uint16_t admin_vq_start_idx;
	uint16_t admin_vq_count;
} __packed __aligned(2);

struct virtio_msg_get_dev_features {
	uint32_t block_idx;
	uint32_t block_count;
} __packed __aligned(2);

struct virtio_msg_set_drv_features {
	uint32_t block_idx;
	uint32_t block_count;
} __packed __aligned(2);

struct virtio_msg_get_config_req {
	uint32_t byte_offset;
	uint32_t byte_count;
} __packed __aligned(2);

struct virtio_msg_get_config_resp {
	uint32_t gen_count;
	uint32_t byte_offset;
	uint32_t byte_count;
} __packed __aligned(2);

struct virtio_msg_device_status {
	uint32_t status;
} __packed __aligned(2);

struct virtio_msg_get_vqueue_req {
	uint32_t vq_idx;
} __packed __aligned(2);

struct virtio_msg_get_vqueue_resp {
	uint32_t vq_idx;
	uint32_t max_size;
	uint32_t vq_size;
	uint32_t reserved;
	uint64_t desc_addr;
	uint64_t driver_addr;
	uint64_t device_addr;
} __packed __aligned(2);

struct virtio_msg_set_vqueue_req {
	uint32_t vq_idx;
	uint32_t reserved1;
	uint32_t vq_size;
	uint32_t reserved2;
	uint64_t desc_addr;
	uint64_t driver_addr;
	uint64_t device_addr;
} __packed __aligned(2);

struct virtio_msg_event_avail {
	uint32_t vq_idx;
	uint32_t next_wrap;
} __packed __aligned(2);

static void dev_get_features(struct virtio_dev *vdev, bitstr_t *bs,
			     size_t count, size_t offset)
{
	if (vdev->ops->get_features)
		vdev->ops->get_features(vdev, bs, count, offset);
	else
		virtio_dev_default_get_features(vdev, bs, count, offset);
}

static void dev_set_features(struct virtio_dev *vdev, bitstr_t *bs,
			     size_t count, size_t offset)
{
	if (vdev->ops->set_features)
		vdev->ops->set_features(vdev, bs, count, offset);
	else
		virtio_dev_default_set_features(vdev, bs, count, offset);
}

static struct virtio_dev_vq *dev_get_vq(struct virtio_dev *vdev, size_t vq_idx)
{
	if (vdev->ops->get_vq)
		return vdev->ops->get_vq(vdev, vq_idx);

	return virtio_dev_get_vq(vdev, vq_idx);
}

static void handle_get_devices(struct virtio_msg_hdr *hdr, size_t max_msg_size)
{
	struct virtio_msg_get_devices_req *req = (void *)(hdr + 1);
	struct virtio_msg_get_devices_resp *resp = (void *)(hdr + 1);
	bitstr_t *bs = (void *)(resp + 1);
	size_t req_count = 0;
	size_t pop_count = 0;
	size_t bs_bytes = 0;

	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr) + sizeof(*req))
		goto err;

	/* We only support offset 0 */
	if (req->offset)
		goto err;

	req_count = req->count;
	bs_bytes = bitstr_size(req_count);
	if (sizeof(*hdr) + sizeof(*resp) + bs_bytes > max_msg_size)
		goto err;

	if (virtio_dev_get_devices_bitstring(bs, req_count, &pop_count))
		goto err;
	resp->offset = 0;
	resp->count = pop_count;
	resp->next_offset = 0;
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp) + bs_bytes;

	return;
err:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
	hdr->dev_num = 0;
	memset(resp, 0, sizeof(*resp));
}

static void handle_ping(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_ping *msg = (void *)(hdr + 1);

	if (hdr->msg_size != sizeof(*hdr) + sizeof(*msg)) {
		hdr->msg_size = sizeof(*hdr) + sizeof(*msg);
		hdr->dev_num = 0;
		memset(msg, 0, sizeof(*msg));
	}
}

static void handle_get_device_info(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_get_device_info_resp *resp = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;

	memset(resp, 0, sizeof(*resp));
	if (hdr->msg_size != sizeof(*hdr))
		goto out;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (!vdev)
		goto out;

	resp->device_id = vdev->ops->dev_id;
	resp->vendor_id = vdev->ops->vendor_id;
	assert(!(vdev->ops->feature_bit_count % 32));
	resp->feature_bit_count = vdev->ops->feature_bit_count;
	resp->config_size = vdev->ops->config_size;
	resp->max_vq_count = vdev->ops->vq_count;
	resp->admin_vq_start_idx = vdev->ops->admin_vq_start_idx;
	resp->admin_vq_count = vdev->ops->admin_vq_count;
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_get_dev_features(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_get_dev_features *v = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;
	size_t block_size = 32;
	size_t count = 0;
	size_t offs = 0;

	if (hdr->msg_size != sizeof(*hdr) + sizeof(*v))
		goto err;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (!vdev)
		goto err;

	/*
	 * Bound the block index and count before scaling to bit counts so the
	 * multiplication below cannot overflow size_t (32-bit on the AArch32
	 * core) and defeat the range check.
	 */
	if (v->block_count > VIRTIO_MAX_FEATURE_BIT_COUNT / block_size ||
	    v->block_idx > VIRTIO_MAX_FEATURE_BIT_COUNT / block_size)
		goto err;

	count = v->block_count * block_size;
	offs = v->block_idx * block_size;
	if (count + offs > VIRTIO_MAX_FEATURE_BIT_COUNT)
		goto err;

	memset(v + 1, 0, v->block_count * sizeof(uint32_t));
	dev_get_features(vdev, (void *)(v + 1), count, offs);
	hdr->msg_size = sizeof(*hdr) + sizeof(*v) +
			v->block_count * sizeof(uint32_t);
	return;
err:
	memset(v, 0, sizeof(*v));
	hdr->msg_size = sizeof(*hdr) + sizeof(*v);
}

static void handle_set_drv_features(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_set_drv_features *v = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;
	size_t block_size = 32;
	size_t count = 0;
	size_t offs = 0;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (!vdev)
		goto out;
	/*
	 * Bound block index/count before scaling so the bit-count
	 * multiplication cannot overflow size_t and bypass the range check.
	 */
	if (v->block_count > VIRTIO_MAX_FEATURE_BIT_COUNT / block_size ||
	    v->block_idx > VIRTIO_MAX_FEATURE_BIT_COUNT / block_size) {
		vdev->features_ok = false;
		goto out;
	}
	if (hdr->msg_size != sizeof(*hdr) + sizeof(*v) +
			     v->block_count * sizeof(uint32_t)) {
		vdev->features_ok = false;
		goto out;
	}

	count = v->block_count * block_size;
	offs = v->block_idx * block_size;
	if (count + offs > VIRTIO_MAX_FEATURE_BIT_COUNT) {
		vdev->features_ok = false;
		goto out;
	}

	dev_set_features(vdev, (void *)(v + 1), count, offs);
out:
	hdr->msg_size = sizeof(*hdr);
}

static void handle_get_config(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_get_config_req *req = (void *)(hdr + 1);
	struct virtio_msg_get_config_resp *resp = (void *)(hdr + 1);
	uint32_t byte_offset = req->byte_offset;
	uint32_t byte_count = req->byte_count;
	struct virtio_dev *vdev = NULL;

	memset(resp, 0, sizeof(*resp));
	if (hdr->msg_size != sizeof(*hdr) + sizeof(*req))
		goto out;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (!vdev || !vdev->ops->get_config)
		goto out;

	if (byte_offset > vdev->ops->config_size ||
	    byte_count > vdev->ops->config_size ||
	    byte_offset + byte_count > vdev->ops->config_size)
		goto out;

	resp->gen_count = vdev->conf_gen_count;
	resp->byte_offset = byte_offset;
	resp->byte_count = byte_count;
	vdev->ops->get_config(vdev, resp + 1, byte_count, byte_offset);
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp) + resp->byte_count;
}

static void handle_set_device_status(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_device_status *v = (void *)(hdr + 1);
	size_t msg_size = sizeof(*hdr) + sizeof(*v);
	struct virtio_dev *vdev = NULL;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (hdr->msg_size != msg_size || !vdev) {
		DMSG("hdr->msg_size %#"PRIx16" vs msg_size %#zx vdev %p",
		     hdr->msg_size, msg_size, (void *)vdev);
		hdr->msg_size = msg_size;
		memset(v, 0, sizeof(*v));
		return;
	}

	DMSG("v->status %#"PRIx32, v->status);
	if (!v->status && vdev->status) {
		virtio_dev_reset(vdev);
		goto out;
	}
	if (v->status & VIRTIO_DEV_STATUS_FAILED)
		goto out;
	if (vdev->status & VIRTIO_DEV_STATUS_DEVICE_NEEDS_RESET)
		goto out;

	if (v->status & VIRTIO_DEV_STATUS_ACKNOWLEDGE) {
		vdev->status |= VIRTIO_DEV_STATUS_ACKNOWLEDGE;
		DMSG("VIRTIO_DEV_STATUS_ACKNOWLEDGE");
	}

	if (v->status & VIRTIO_DEV_STATUS_DRIVER) {
		if (!(vdev->status & VIRTIO_DEV_STATUS_ACKNOWLEDGE))
			goto out;
		vdev->status |= VIRTIO_DEV_STATUS_DRIVER;
		DMSG("VIRTIO_DEV_STATUS_DRIVER");
	}

	if (v->status & VIRTIO_DEV_STATUS_FEATURES_OK) {
		if (!(vdev->status & VIRTIO_DEV_STATUS_DRIVER) ||
		    !vdev->features_ok)
			goto out;
		vdev->status |= VIRTIO_DEV_STATUS_FEATURES_OK;
		DMSG("VIRTIO_DEV_STATUS_FEATURES_OK");
	}

	if (v->status & VIRTIO_DEV_STATUS_DRIVER_OK) {
		if (!(vdev->status & VIRTIO_DEV_STATUS_FEATURES_OK))
			goto out;
		vdev->status |= VIRTIO_DEV_STATUS_DRIVER_OK;
		DMSG("VIRTIO_DEV_STATUS_DRIVER_OK");
	}

out:
	if (vdev->ops->status_changed)
		vdev->ops->status_changed(vdev, vdev->status);
	v->status = vdev->status;
}

static void handle_get_device_status(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_device_status *resp = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (hdr->msg_size != sizeof(*hdr) || !vdev) {
		DMSG("hdr->msg_size %#"PRIx16" vs msg_size %#zx vdev %p",
		     hdr->msg_size, sizeof(*hdr), (void *)vdev);
		memset(resp, 0, sizeof(*resp));
		hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
		hdr->dev_num = 0;
		return;
	}

	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
	resp->status = vdev->status;
}

static void handle_get_vqueue(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_get_vqueue_req *req = (void *)(hdr + 1);
	struct virtio_msg_get_vqueue_resp *resp = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;
	uint32_t vq_idx = req->vq_idx;
	struct virtio_dev_vq *vq = NULL;

	vdev = virtio_dev_lookup(hdr->dev_num);
	memset(resp, 0, sizeof(*resp));
	if (hdr->msg_size != sizeof(*hdr) + sizeof(*req) || !vdev) {
		hdr->dev_num = 0;
		goto out;
	}

	resp->vq_idx = vq_idx;
	vq = dev_get_vq(vdev, vq_idx);
	if (vq) {
		resp->max_size = vdev->ops->max_desc_count /
				 sizeof(struct virtq_desc);
		resp->vq_size = vq->desc_count;
		resp->desc_addr = virt_to_phys(vq->desc);
		resp->driver_addr = virt_to_phys(vq->driver);
		resp->device_addr = virt_to_phys(vq->device);
	}

out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_set_vqueue(struct virtio_msg_hdr *hdr,
			      const struct virtio_transport_ops *ops)
{
	struct virtio_msg_set_vqueue_req *req = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (hdr->msg_size != sizeof(*hdr) + sizeof(*req) || !vdev) {
		hdr->dev_num = 0;
		goto out;
	}

	virtio_dev_setup_vq(vdev, req->vq_idx, ops, req->vq_size,
			    req->desc_addr, req->driver_addr, req->device_addr);
out:
	hdr->msg_size = sizeof(*hdr);
}

static void handle_event_avail(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_event_avail *event = (void *)(hdr + 1);
	struct virtio_dev *vdev = NULL;
	struct virtio_dev_vq *vq = NULL;

	vdev = virtio_dev_lookup(hdr->dev_num);
	if (hdr->msg_size != sizeof(*hdr) + sizeof(*event) || !vdev) {
		hdr->dev_num = 0;
		goto out;
	}
	vq = dev_get_vq(vdev, event->vq_idx);
	if (vq && vq->enabled)
		notif_send_async(NOTIF_VALUE_DO_BOTTOM_HALF, 0);

out:
	hdr->msg_size = sizeof(*hdr);
}

void virtio_msg_handle_bus_req(struct virtio_msg_hdr *hdr,
			       const struct virtio_transport_ops *ops __unused,
			       size_t max_msg_size)
{
	switch (hdr->msg_id) {
	case BUS_MSG_GET_DEVICES:
		handle_get_devices(hdr, max_msg_size);
		break;
	case BUS_MSG_PING:
		handle_ping(hdr);
		break;
	default:
		virtio_msg_set_null_msg(hdr);
		return;
	}
	hdr->type = MSG_TYPE_BUS_RESPONSE;
}

void virtio_msg_handle_transport_req(struct virtio_msg_hdr *hdr,
				     const struct virtio_transport_ops *ops)
{
	switch (hdr->msg_id) {
	case VIRTIO_MSG_DEVICE_INFO:
		handle_get_device_info(hdr);
		break;
	case VIRTIO_MSG_GET_DEV_FEATURES:
		handle_get_dev_features(hdr);
		break;
	case VIRTIO_MSG_SET_DRV_FEATURES:
		handle_set_drv_features(hdr);
		break;
	case VIRTIO_MSG_GET_CONFIG:
		handle_get_config(hdr);
		break;
	case VIRTIO_MSG_GET_DEVICE_STATUS:
		handle_get_device_status(hdr);
		break;
	case VIRTIO_MSG_SET_DEVICE_STATUS:
		handle_set_device_status(hdr);
		break;
	case VIRTIO_MSG_GET_VQUEUE:
		handle_get_vqueue(hdr);
		break;
	case VIRTIO_MSG_SET_VQUEUE:
		handle_set_vqueue(hdr, ops);
		break;
	case VIRTIO_MSG_EVENT_AVAIL:
		handle_event_avail(hdr);
		break;
	default:
		virtio_msg_set_null_msg(hdr);
		return;
	}
	hdr->type = MSG_TYPE_TRANSPORT_RESPONSE;
}

void virtio_msg_recv(struct virtio_msg_hdr *hdr,
		     const struct virtio_transport_ops *ops,
		     size_t max_msg_size)
{
	switch (hdr->type) {
	case MSG_TYPE_TRANSPORT_REQUEST:
		virtio_msg_handle_transport_req(hdr, ops);
		break;
	case MSG_TYPE_BUS_REQUEST:
		virtio_msg_handle_bus_req(hdr, ops, max_msg_size);
		break;
	default:
		virtio_msg_set_null_msg(hdr);
	}
}
