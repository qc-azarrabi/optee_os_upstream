/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * Backend (device-side) virtio core. OP-TEE acts as the virtio *device*; the
 * normal-world driver is the *frontend*. This header is transport- and
 * architecture-agnostic: it never references FF-A. All bus-address translation
 * and mapping is done through struct virtio_transport_ops (see
 * <drivers/virtio/virtio_transport.h>), supplied by the transport binding and
 * stored per virtqueue.
 *
 * Naming follows the virtio spec's *device* role: the backend types use the
 * virtio_dev_* namespace. The virtio_driver / virtio_drv_* names are reserved
 * for a future frontend (spec "driver") role.
 */
#ifndef __DRIVERS_VIRTIO_VIRTIO_DEV_H
#define __DRIVERS_VIRTIO_VIRTIO_DEV_H

#include <bitstring.h>
#include <drivers/virtio/virtio_transport.h>
#include <sys/queue.h>
#include <tee_api_types.h>
#include <types_ext.h>

#define VIRTIO_F_VERSION_1			32
#define VIRTIO_F_ACCESS_PLATFORM		33
#define VIRTIO_MAX_FEATURE_BIT_COUNT		64

/* 2.1 Device Status Field */
#define VIRTIO_DEV_STATUS_ACKNOWLEDGE		BIT(0)
#define VIRTIO_DEV_STATUS_DRIVER		BIT(1)
#define VIRTIO_DEV_STATUS_DRIVER_OK		BIT(2)
#define VIRTIO_DEV_STATUS_FEATURES_OK		BIT(3)
#define VIRTIO_DEV_STATUS_DEVICE_NEEDS_RESET	BIT(6)
#define VIRTIO_DEV_STATUS_FAILED		BIT(7)

/* This marks a buffer as continuing via the next field. */
#define VIRTQ_DESC_F_NEXT		1
/* This marks a buffer as write-only (otherwise read-only). */
#define VIRTQ_DESC_F_WRITE		2
/* This means the buffer contains a list of buffer descriptors. */
#define VIRTQ_DESC_F_INDIRECT		4
/*
 * The device uses this in used->flags to advise the driver: don't kick me
 * when you add a buffer. It's unreliable, so it's simply an optimization.
 */
#define VIRTQ_USED_F_NO_NOTIFY		1
/*
 * The driver uses this in avail->flags to advise the device: don't
 * interrupt me when you consume a buffer. It's unreliable, so it's simply
 * an optimization.
 */
#define VIRTQ_AVAIL_F_NO_INTERRUPT	1
/* Support for indirect descriptors */
#define VIRTIO_F_INDIRECT_DESC		28
/* Support for avail_event and used_event fields */
#define VIRTIO_F_EVENT_IDX		29
/* Arbitrary descriptor layouts. */
#define VIRTIO_F_ANY_LAYOUT		27

struct virtq_desc {
	uint64_t addr;	/* Buffer Address. */
	uint32_t len;	/* Buffer Length. */
	uint16_t flags;	/* The flags depending on descriptor type. */
	uint16_t next;	/* We chain unused descriptors via this, too */
};

struct virtq_used_elem {
	uint32_t id;	/* Index of start of used descriptor chain. */
	uint32_t len;	/* Total length of the descriptor chain written to. */
};

struct virtq_used {
	uint16_t flags;
	uint16_t idx;
	struct virtq_used_elem ring[];
	/* Only if VIRTIO_F_EVENT_IDX: le16 avail_event; */
};

struct virtq_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[];
	/* Only if VIRTIO_F_EVENT_IDX: le16 used_event; */
};

struct virtio_dev;

/*
 * struct virtio_dev_vq - a backend (device-side) virtqueue.
 *
 * @ops:	Transport seam used to translate/map ring bus addresses.
 * @callback:	Bottom-half handler invoked when the driver kicks this queue.
 * @vdev:	Owning device.
 * @desc:	Mapped descriptor table.
 * @driver:	Mapped available ring (driver -> device).
 * @device:	Mapped used ring (device -> driver).
 * @desc_ba:	Bus address of the descriptor table.
 * @driver_ba:	Bus address of the available ring.
 * @device_ba:	Bus address of the used ring.
 * @desc_count:	Number of descriptors (queue size).
 * @avail_idx:	Next available-ring index the device will consume.
 * @vq_id:	Queue index within the device.
 * @enabled:	Whether the queue has been set up and enabled.
 */
struct virtio_dev_vq {
	const struct virtio_transport_ops *ops;
	void (*callback)(struct virtio_dev_vq *vq);
	struct virtio_dev *vdev;
	struct virtq_desc *desc;
	struct virtq_avail *driver;
	struct virtq_used *device;
	uint64_t desc_ba;
	uint64_t driver_ba;
	uint64_t device_ba;
	unsigned int desc_count;
	uint16_t avail_idx;
	unsigned int vq_id;
	bool enabled;
};

/*
 * struct virtio_dev_vq_ctx - device-side ring accessor context.
 *
 * A single avail-ring head flows through the virtio_dev_vq_* accessors and is
 * retired to the used ring exactly once. The read/write helpers operate on
 * *uni-directional* descriptor chains (a chain is either all device-readable
 * or all device-writable); a head is retired by exactly one of
 * virtio_dev_vq_complete() after either the read or the write side.
 */
struct virtio_dev_vq_ctx {
	struct virtio_dev_vq *vq;
	uint16_t avail_idx;
	uint16_t first_desc_idx;
};

/*
 * struct virtio_dev_vq_info - per-virtqueue description for bulk setup.
 *
 * @name:	Human-readable name, or NULL for an unused slot.
 * @callback:	Bottom-half handler wired onto the queue on setup.
 */
struct virtio_dev_vq_info {
	const char *name;
	void (*callback)(struct virtio_dev_vq *vq);
};

/*
 * struct virtio_dev_ops - per-device-class operations and identity.
 *
 * Identity/size fields (@dev_id .. @max_desc_count) are mandatory. The feature
 * table (@features/@feature_count) and per-vq table (@vqs_info) let the
 * framework provide default feature negotiation and virtqueue setup, so a
 * simple device supplies only @get_config (+ optionally @set_config/@reset).
 *
 * @get_features/@set_features/@setup_vq/@get_vq are optional overrides; when
 * NULL the framework defaults (driven by @features/@vqs_info) are used.
 */
struct virtio_dev_ops {
	uint32_t dev_id;
	uint32_t vendor_id;
	uint8_t config_size;
	uint8_t vq_count;
	uint8_t admin_vq_start_idx;
	uint8_t admin_vq_count;
	uint8_t feature_bit_count;
	uint32_t max_desc_count;

	/* Feature table used by the default feature negotiation. */
	const uint32_t *features;
	uint16_t feature_count;

	/* Per-vq table used by the default virtio_dev_setup_vq(). */
	const struct virtio_dev_vq_info *vqs_info;

	/* Optional feature overrides. */
	void (*get_features)(struct virtio_dev *vdev, bitstr_t *bs,
			     size_t count, size_t offset);
	void (*set_features)(struct virtio_dev *vdev, bitstr_t *bs,
			     size_t count, size_t offset);
	TEE_Result (*validate_features)(struct virtio_dev *vdev);

	/* Config space accessors. */
	void (*get_config)(struct virtio_dev *vdev, void *data,
			   size_t count, size_t offset);
	void (*set_config)(struct virtio_dev *vdev, const void *data,
			   size_t count, size_t offset);

	/* Optional virtqueue overrides. */
	void (*setup_vq)(struct virtio_dev *vdev, struct virtio_dev_vq *vq);
	struct virtio_dev_vq *(*get_vq)(struct virtio_dev *vdev, size_t vq_idx);

	/* Optional lifecycle hooks. */
	void (*status_changed)(struct virtio_dev *vdev, uint8_t new_status);
	void (*reset)(struct virtio_dev *vdev);
};

/*
 * struct virtio_dev - a registered backend virtio device instance.
 *
 * @dev_num:	Assigned enumeration number (DEN0153 GET_DEVICES/DEVICE_INFO).
 * @features:	Negotiated feature bits.
 * @features_ok:	Whether the last SET_DRV_FEATURES was acceptable.
 * @status:	Device status field.
 * @ops:	Device-class operations and identity.
 * @conf_gen_count:	Config generation counter.
 * @vqs:	Array of @ops->vq_count virtqueues (allocated on register).
 * @priv:	Device-private pointer (optional; avoids container_of).
 * @link:	Registry linkage.
 */
struct virtio_dev {
	uint16_t dev_num;
	bitstr_t features[bitstr_size(VIRTIO_MAX_FEATURE_BIT_COUNT)];
	bool features_ok;
	uint8_t status;
	const struct virtio_dev_ops *ops;
	uint64_t conf_gen_count;
	struct virtio_dev_vq *vqs;
	void *priv;
	TAILQ_ENTRY(virtio_dev) link;
};

/* Registry. */
TEE_Result virtio_dev_register(struct virtio_dev *vdev);
struct virtio_dev *virtio_dev_lookup(uint16_t dev_num);
TEE_Result virtio_dev_get_devices_bitstring(bitstr_t *bs, size_t count,
					    size_t *pop_count);
TEE_Result virtio_dev_get_features(uint16_t dev_num, bitstr_t *bs,
				   size_t count);

/*
 * Default feature negotiation, usable by device get_features/set_features
 * overrides that want to build on the framework's table handling.
 */
void virtio_dev_default_get_features(struct virtio_dev *vdev, bitstr_t *bs,
				     size_t count, size_t offset);
void virtio_dev_default_set_features(struct virtio_dev *vdev, bitstr_t *bs,
				     size_t count, size_t offset);

/* Virtqueue lifecycle. */
TEE_Result virtio_dev_vq_init(struct virtio_dev_vq *vq,
			      const struct virtio_transport_ops *ops,
			      size_t vq_size, uint64_t desc_ba,
			      uint64_t drv_ba, uint64_t dev_ba);
TEE_Result virtio_dev_vq_enable(struct virtio_dev_vq *vq);
void virtio_dev_vq_disable(struct virtio_dev_vq *vq);

/* Default get_vq (indexes vdev->vqs) and bulk setup helper. */
struct virtio_dev_vq *virtio_dev_get_vq(struct virtio_dev *vdev,
					size_t vq_idx);
TEE_Result virtio_dev_setup_vq(struct virtio_dev *vdev, size_t idx,
			       const struct virtio_transport_ops *ops,
			       size_t size, uint64_t desc_ba,
			       uint64_t drv_ba, uint64_t dev_ba);

/*
 * Device-side split-ring accessors (virtio_dev_vq_*).
 *
 * The device (OP-TEE backend) consumes the driver's avail ring and produces the
 * used ring, i.e. the role Linux's vringh serves for a vhost/vDPA host; OP-TEE
 * runs at S-EL1 rather than as a hypervisor host, so these are named for the
 * virtio *device* side instead.
 *
 * These operate on uni-directional descriptor chains; see
 * struct virtio_dev_vq_ctx.
 * Read side (driver -> device): virtio_dev_vq_get_avail() claims a head,
 * virtio_dev_vq_pull() copies bytes out of the chain, virtio_dev_vq_complete()
 * retires it.
 * Write side (device -> driver): virtio_dev_vq_get_writable() claims a head,
 * virtio_dev_vq_push() copies bytes into the chain,
 * virtio_dev_vq_complete_len() retires it with the written length.
 */
TEE_Result virtio_dev_vq_get_avail(struct virtio_dev_vq *vq,
				   struct virtio_dev_vq_ctx *ctx);
TEE_Result virtio_dev_vq_pull(void *addr, struct virtio_dev_vq_ctx *ctx,
			      size_t offs, size_t len);
void virtio_dev_vq_complete(struct virtio_dev_vq_ctx *ctx);

TEE_Result virtio_dev_vq_get_writable(struct virtio_dev_vq *vq,
				      struct virtio_dev_vq_ctx *ctx,
				      size_t max_len);
TEE_Result virtio_dev_vq_push(const void *addr, struct virtio_dev_vq_ctx *ctx,
			      size_t offs, size_t len);
void virtio_dev_vq_complete_len(struct virtio_dev_vq_ctx *ctx, size_t len);

/* Bump the config generation counter (and, in future, raise EVENT_CONFIG). */
void virtio_dev_config_changed(struct virtio_dev *vdev);

/*
 * Reset a single device: disable and detach all its virtqueues, invoke the
 * optional device reset op, and clear negotiated features and status.
 */
void virtio_dev_reset(struct virtio_dev *vdev);

/* Reset every registered device (used by the transport-level RESET). */
void virtio_dev_reset_all(void);

#endif /*__DRIVERS_VIRTIO_VIRTIO_DEV_H*/
