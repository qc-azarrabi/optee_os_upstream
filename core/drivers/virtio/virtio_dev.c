// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * Backend (device-side) virtio core: device registry, host-side split-ring
 * accessors (vringh_*), default feature negotiation and virtqueue setup, and
 * the notification bottom-half that drives virtqueue callbacks. Transport- and
 * architecture-agnostic: all bus-address translation goes through
 * struct virtio_transport_ops stored per virtqueue.
 */

#include <bitstring.h>
#include <drivers/virtio/virtio_dev.h>
#include <initcall.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <string.h>
#include <sys/queue.h>
#include <trace.h>
#include <util.h>

TAILQ_HEAD(virtio_dev_head, virtio_dev);

static uint16_t next_dev_num;
static struct virtio_dev_head virtio_dev_head =
	TAILQ_HEAD_INITIALIZER(virtio_dev_head);

TEE_Result virtio_dev_get_devices_bitstring(bitstr_t *bs, size_t count,
					    size_t *pop_count)
{
	struct virtio_dev *vdev = NULL;
	size_t c = 0;

	memset(bs, 0, bitstr_size(count));
	TAILQ_FOREACH(vdev, &virtio_dev_head, link) {
		if (vdev->dev_num >= count)
			return TEE_ERROR_SHORT_BUFFER;

		bit_set(bs, vdev->dev_num);
		c++;
	}

	*pop_count = c;

	return TEE_SUCCESS;
}

struct virtio_dev *virtio_dev_lookup(uint16_t dev_num)
{
	struct virtio_dev *vdev = NULL;

	TAILQ_FOREACH(vdev, &virtio_dev_head, link)
		if (vdev->dev_num == dev_num)
			return vdev;

	return NULL;
}

TEE_Result virtio_dev_register(struct virtio_dev *vdev)
{
	size_t n = 0;

	if (!vdev->ops || !vdev->ops->vq_count)
		return TEE_ERROR_BAD_PARAMETERS;
	if (next_dev_num == UINT16_MAX)
		return TEE_ERROR_GENERIC;

	vdev->vqs = calloc(vdev->ops->vq_count, sizeof(*vdev->vqs));
	if (!vdev->vqs)
		return TEE_ERROR_OUT_OF_MEMORY;
	for (n = 0; n < vdev->ops->vq_count; n++)
		vdev->vqs[n].vq_id = n;

	vdev->dev_num = next_dev_num;
	next_dev_num++;

	TAILQ_INSERT_TAIL(&virtio_dev_head, vdev, link);

	return TEE_SUCCESS;
}

void virtio_dev_default_get_features(struct virtio_dev *vdev, bitstr_t *bs,
				     size_t count, size_t offset)
{
	const struct virtio_dev_ops *ops = vdev->ops;
	size_t n = 0;

	assert(count + offset <= VIRTIO_MAX_FEATURE_BIT_COUNT);

	for (n = 0; n < ops->feature_count; n++) {
		if (ops->features[n] >= offset &&
		    ops->features[n] - offset < count)
			bit_set(bs, ops->features[n] - offset);
	}
}

static bool feature_offered(const struct virtio_dev_ops *ops, size_t f)
{
	size_t n = 0;

	for (n = 0; n < ops->feature_count; n++)
		if (ops->features[n] == f)
			return true;

	return false;
}

void virtio_dev_default_set_features(struct virtio_dev *vdev, bitstr_t *bs,
				     size_t count, size_t offset)
{
	size_t n = 0;

	assert(count + offset <= VIRTIO_MAX_FEATURE_BIT_COUNT);

	for (n = 0; n < count; n++) {
		if (!bit_test(bs, n))
			continue;
		if (!feature_offered(vdev->ops, n + offset)) {
			EMSG("Unknown feature %zu", n + offset);
			vdev->features_ok = false;
			return;
		}
		bit_set(vdev->features, n + offset);
	}

	vdev->features_ok = true;
}

TEE_Result virtio_dev_get_features(uint16_t dev_num, bitstr_t *bs, size_t count)
{
	struct virtio_dev *vdev = virtio_dev_lookup(dev_num);

	if (!vdev)
		return TEE_ERROR_ITEM_NOT_FOUND;

	if (vdev->ops->get_features)
		vdev->ops->get_features(vdev, bs, count, 0);
	else
		virtio_dev_default_get_features(vdev, bs, count, 0);

	return TEE_SUCCESS;
}

TEE_Result virtio_dev_vq_init(struct virtio_dev_vq *vq,
			      const struct virtio_transport_ops *ops,
			      size_t vq_size, uint64_t desc_ba,
			      uint64_t drv_ba, uint64_t dev_ba)
{
	TEE_Result res = TEE_SUCCESS;
	size_t sz = 0;

	DMSG("vq idx %u size %zu desc %#"PRIx64" drv %#"PRIx64" dev %#"PRIx64,
	     vq->vq_id, vq_size, desc_ba, drv_ba, dev_ba);
	if (MUL_OVERFLOW(vq_size, sizeof(*vq->desc), &sz))
		return TEE_ERROR_BAD_PARAMETERS;

	res = ops->map_area(ops, desc_ba);
	if (res)
		return res;
	vq->desc = ops->bus_addr_to_va(ops, desc_ba, sz);
	if (!vq->desc) {
		res = TEE_ERROR_BAD_PARAMETERS;
		goto err_desc;
	}

	res = ops->map_area(ops, drv_ba);
	if (res)
		goto err_desc;
	vq->driver = ops->bus_addr_to_va(ops, drv_ba, sizeof(*vq->driver));
	if (!vq->driver) {
		res = TEE_ERROR_BAD_PARAMETERS;
		goto err_drv;
	}

	res = ops->map_area(ops, dev_ba);
	if (res)
		goto err_drv;
	vq->device = ops->bus_addr_to_va(ops, dev_ba, sizeof(*vq->device));
	if (!vq->device) {
		res = TEE_ERROR_BAD_PARAMETERS;
		goto err_dev;
	}

	DMSG("vq idx %u PA desc %#"PRIxPA" drv %#"PRIxPA" dev %#"PRIxPA,
	     vq->vq_id, virt_to_phys(vq->desc), virt_to_phys(vq->driver),
	     virt_to_phys(vq->device));

	vq->ops = ops;
	vq->desc_ba = desc_ba;
	vq->driver_ba = drv_ba;
	vq->device_ba = dev_ba;
	vq->desc_count = vq_size;

	return TEE_SUCCESS;

err_dev:
	ops->unmap_area(ops, dev_ba);
err_drv:
	ops->unmap_area(ops, drv_ba);
err_desc:
	ops->unmap_area(ops, desc_ba);

	return res;
}

TEE_Result virtio_dev_vq_enable(struct virtio_dev_vq *vq)
{
	assert(!vq->enabled);
	vq->enabled = true;

	return TEE_SUCCESS;
}

void virtio_dev_vq_disable(struct virtio_dev_vq *vq)
{
	assert(vq->enabled);
	vq->enabled = false;
}

struct virtio_dev_vq *virtio_dev_get_vq(struct virtio_dev *vdev, size_t vq_idx)
{
	if (vq_idx >= vdev->ops->vq_count)
		return NULL;

	return vdev->vqs + vq_idx;
}

TEE_Result virtio_dev_setup_vq(struct virtio_dev *vdev, size_t idx,
			       const struct virtio_transport_ops *ops,
			       size_t size, uint64_t desc_ba,
			       uint64_t drv_ba, uint64_t dev_ba)
{
	const struct virtio_dev_ops *dops = vdev->ops;
	struct virtio_dev_vq *vq = NULL;
	TEE_Result res = TEE_SUCCESS;

	if (dops->get_vq)
		vq = dops->get_vq(vdev, idx);
	else
		vq = virtio_dev_get_vq(vdev, idx);
	if (!vq)
		return TEE_ERROR_BAD_PARAMETERS;

	res = virtio_dev_vq_init(vq, ops, size, desc_ba, drv_ba, dev_ba);
	if (res)
		return res;

	if (dops->setup_vq) {
		dops->setup_vq(vdev, vq);
		return TEE_SUCCESS;
	}

	vq->vdev = vdev;
	if (dops->vqs_info)
		vq->callback = dops->vqs_info[idx].callback;

	res = virtio_dev_vq_enable(vq);
	if (res) {
		vq->vdev = NULL;
		vq->callback = NULL;
	}

	return res;
}

TEE_Result vringh_get_avail(struct virtio_dev_vq *vq, struct vringh_ctx *ctx)
{
	if (vq->avail_idx == vq->driver->idx)
		return TEE_ERROR_ITEM_NOT_FOUND;

	FMSG("vq id %u vq->avail_idx %u vq->driver->idx %u",
	     vq->vq_id, vq->avail_idx, vq->driver->idx);
	*ctx = (struct vringh_ctx){
		.vq = vq,
		.avail_idx = vq->avail_idx,
		.first_desc_idx = vq->driver->ring[vq->avail_idx %
						    vq->desc_count],
	};

	return TEE_SUCCESS;
}

TEE_Result vringh_pull(void *addr, struct vringh_ctx *ctx, size_t offs,
		       size_t len)
{
	size_t desc_idx = ctx->first_desc_idx;
	struct virtio_dev_vq *vq = ctx->vq;
	struct virtq_desc *desc = vq->desc;
	size_t dst_offs = 0;
	void *src = NULL;
	uint64_t bus_addr = 0;
	size_t l = 0;
	size_t o = 0;

	while (true) {
		if (desc_idx >= vq->desc_count)
			return TEE_ERROR_GENERIC;
		if (desc[desc_idx].flags & VIRTQ_DESC_F_WRITE)
			return TEE_ERROR_GENERIC;

		if (desc[desc_idx].len + o > offs) {
			if (o < offs) {
				bus_addr = desc[desc_idx].addr + offs - o;
				l = desc[desc_idx].len + offs - o;
			} else {
				bus_addr = desc[desc_idx].addr;
				l = desc[desc_idx].len;
			}
			l = MIN(l, len - dst_offs);
			src = vq->ops->bus_addr_to_va(vq->ops, bus_addr, l);
			FMSG("bus_addr %#"PRIx64" VA %p PA %#"PRIxPA,
			     bus_addr, src, virt_to_phys(src));
			if (!src)
				return TEE_ERROR_GENERIC;
			if (TRACE_LEVEL >= TRACE_FLOW)
				DHEXDUMP(src, len);
			memcpy((uint8_t *)addr + dst_offs, src, l);

			dst_offs += l;
			assert(dst_offs <= len);
			if (dst_offs == len)
				return TEE_SUCCESS;
		}

		o += desc[desc_idx].len;
		if (desc[desc_idx].flags & VIRTQ_DESC_F_NEXT)
			desc_idx = desc[desc_idx].next;
		else
			return TEE_ERROR_GENERIC;
	}
}

void vringh_complete(struct vringh_ctx *ctx)
{
	struct virtio_dev_vq *vq = ctx->vq;
	struct virtq_used *used = vq->device;
	size_t ring_idx = used->idx % vq->desc_count;

	FMSG("used->ring[%zu] PA %#"PRIxPA" id %"PRIu16,
	     ring_idx, virt_to_phys(used), ctx->first_desc_idx);
	used->ring[ring_idx].id = ctx->first_desc_idx;
	used->ring[ring_idx].len = 0;
	dsb();
	vq->avail_idx = ctx->avail_idx + 1;
	used->idx = vq->avail_idx;

	ctx->vq = NULL;
}

TEE_Result vringh_get_writable(struct virtio_dev_vq *vq, struct vringh_ctx *ctx,
			       size_t max_len __unused)
{
	if (vq->avail_idx == vq->driver->idx)
		return TEE_ERROR_ITEM_NOT_FOUND;

	FMSG("vq id %u vq->avail_idx %u vq->driver->idx %u",
	     vq->vq_id, vq->avail_idx, vq->driver->idx);
	*ctx = (struct vringh_ctx){
		.vq = vq,
		.avail_idx = vq->avail_idx,
		.first_desc_idx = vq->driver->ring[vq->avail_idx %
						    vq->desc_count],
	};

	return TEE_SUCCESS;
}

TEE_Result vringh_push(const void *addr, struct vringh_ctx *ctx, size_t offs,
		       size_t len)
{
	size_t desc_idx = ctx->first_desc_idx;
	struct virtio_dev_vq *vq = ctx->vq;
	struct virtq_desc *desc = vq->desc;
	size_t src_offs = 0;
	void *dst = NULL;
	uint64_t bus_addr = 0;
	size_t l = 0;
	size_t o = 0;

	while (true) {
		if (desc_idx >= vq->desc_count)
			return TEE_ERROR_GENERIC;
		if (!(desc[desc_idx].flags & VIRTQ_DESC_F_WRITE))
			return TEE_ERROR_GENERIC;
		if (desc[desc_idx].len + o > offs) {
			if (o < offs) {
				bus_addr = desc[desc_idx].addr + offs - o;
				l = desc[desc_idx].len + offs - o;
			} else {
				bus_addr = desc[desc_idx].addr;
				l = desc[desc_idx].len;
			}
			l = MIN(l, len - src_offs);
			dst = vq->ops->bus_addr_to_va(vq->ops, bus_addr, l);
			FMSG("bus_addr %#"PRIx64" VA %p PA %#"PRIxPA,
			     bus_addr, dst, virt_to_phys(dst));
			if (!dst)
				return TEE_ERROR_GENERIC;
			memcpy(dst, (const uint8_t *)addr + src_offs, l);

			src_offs += l;
			assert(src_offs <= len);
			if (src_offs == len)
				return TEE_SUCCESS;
		}

		o += desc[desc_idx].len;
		if (desc[desc_idx].flags & VIRTQ_DESC_F_NEXT)
			desc_idx = desc[desc_idx].next;
		else
			return TEE_ERROR_GENERIC;
	}
}

void vringh_complete_len(struct vringh_ctx *ctx, size_t len)
{
	struct virtio_dev_vq *vq = ctx->vq;
	struct virtq_used *used = vq->device;
	size_t ring_idx = used->idx % vq->desc_count;

	FMSG("used->ring[%zu] PA %#"PRIxPA" id %"PRIu16" len %zu",
	     ring_idx, virt_to_phys(used), ctx->first_desc_idx, len);
	used->ring[ring_idx].id = ctx->first_desc_idx;
	used->ring[ring_idx].len = len;
	dsb();
	vq->avail_idx = ctx->avail_idx + 1;
	used->idx = vq->avail_idx;

	ctx->vq = NULL;
}

void virtio_dev_config_changed(struct virtio_dev *vdev)
{
	vdev->conf_gen_count++;
}

void virtio_dev_reset(struct virtio_dev *vdev)
{
	const struct virtio_dev_ops *ops = vdev->ops;
	size_t n = 0;

	for (n = 0; n < ops->vq_count; n++) {
		struct virtio_dev_vq *vq = vdev->vqs + n;

		if (vq->enabled)
			virtio_dev_vq_disable(vq);
		vq->callback = NULL;
		vq->vdev = NULL;
	}

	if (ops->reset)
		ops->reset(vdev);

	memset(vdev->features, 0, sizeof(vdev->features));
	vdev->features_ok = false;
	vdev->status = 0;
}

void virtio_dev_reset_all(void)
{
	struct virtio_dev *vdev = NULL;

	TAILQ_FOREACH(vdev, &virtio_dev_head, link)
		virtio_dev_reset(vdev);
}

static void atomic_virtio_notif(struct notif_driver *ndrv __unused,
				enum notif_event ev __maybe_unused,
				uint16_t vm_id __maybe_unused)
{
	DMSG("Event %d vm_id %#"PRIx16, (int)ev, vm_id);
}

static void yielding_virtio_notif(struct notif_driver *ndrv __unused,
				  enum notif_event ev)
{
	struct virtio_dev *vdev = NULL;
	struct virtio_dev_vq *vq = NULL;
	size_t n = 0;

	DMSG("Event %d", (int)ev);
	if (ev != NOTIF_EVENT_DO_BOTTOM_HALF)
		return;

	/*
	 * Unlocked since we only modify this during boot while we're still
	 * single threaded. This will need to change once we add or remove
	 * devices after boot.
	 */
	TAILQ_FOREACH(vdev, &virtio_dev_head, link) {
		for (n = 0; n < vdev->ops->vq_count; n++) {
			vq = vdev->vqs + n;
			if (vq->enabled && vq->callback)
				vq->callback(vq);
		}
	}
}

static struct notif_driver virtio_notif __nex_data = {
	.atomic_cb = atomic_virtio_notif,
	.yielding_cb = yielding_virtio_notif,
};

static TEE_Result init_virtio_notif(void)
{
	notif_register_driver(&virtio_notif);

	return TEE_SUCCESS;
}
nex_service_init(init_virtio_notif);
