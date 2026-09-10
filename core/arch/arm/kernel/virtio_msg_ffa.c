// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 */

#include <ffa.h>
#include <initcall.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/thread_spmc.h>
#include <kernel/virtio_msg.h>
#include <kernel/virtio_msg_ffa.h>
#include <malloc.h>
#include <mm/mobj.h>
#include <string.h>
#include <sys/queue.h>
#include <trace.h>
#include <util.h>

/*
 * FF-A bus addresses encode a 16-bit area id in the top bits and a 48-bit
 * offset into that area (DEN0153 "Shared memory areas").
 */
#define AREA_ID_SHIFT		48
#define AREA_OFFSET_MASK	(BIT64(AREA_ID_SHIFT) - 1)

/* FF-A bus message ids (DEN0153 "FF-A bus messages") */
#define FFA_BUS_MSG_VERSION		0x80
#define FFA_BUS_MSG_AREA_SHARE		0x81
#define FFA_BUS_MSG_AREA_UNSHARE	0x82
#define FFA_BUS_MSG_RESET		0x83
#define FFA_BUS_MSG_EVENT_POLL		0x84
#define FFA_BUS_MSG_EVENT_CONFIGURE	0x85
#define FFA_BUS_EVENT_AREA_RELEASE	0xc0

/* FF-A bus feature bits reported in the version response */
#define FFA_BUS_FEATURE_DIRECT_MSG_RX	BIT(0)
#define FFA_BUS_FEATURE_DIRECT_MSG_TX	BIT(1)
#define FFA_BUS_FEATURE_FFA_NOTIF_RX	BIT(4)
#define FFA_BUS_FEATURE_FFA_NOTIF_TX	BIT(5)

/* Version negotiation constants (protocol v1, virtio-msg revision 1) */
#define FFA_PROTO_VERSION		0x00010000
#define VMSG_REVISION			1
#define VMSG_FEATURES			0

/* Area-share and event results */
#define FFA_RESULT_SUCCESS		0
#define FFA_RESULT_ERROR		1
#define FFA_UNSHARE_BUSY		2

/* Event delivery methods (EVENT_CONFIGURE) */
#define FFA_BUS_EVENT_POLLING		0
#define FFA_BUS_EVENT_NOTIF_ASSISTED	1
#define FFA_BUS_EVENT_INDIRECT		2

/* Maximum number of device-type drivers registered on the FF-A bus */
#define VIRTIO_MSG_FFA_MAX_DRV		4

struct msg_version_req {
	uint32_t proto_vers;
	uint32_t vmsg_rev;
	uint32_t vmsg_features;
	uint32_t features;
	uint16_t area_num;
} __packed;

struct msg_version_resp {
	uint32_t proto_vers;
	uint32_t vmsg_rev;
	uint32_t vmsg_features;
	uint32_t features;
	uint16_t area_num;
} __packed;

struct msg_area_share_req {
	uint16_t area_id;
	uint64_t handle;
	uint64_t tag;
	uint32_t page_count;
	uint32_t attrs;
} __packed;

struct msg_area_share_resp {
	uint16_t area_id;
	uint16_t result;
} __packed;

struct msg_area_unshare_req {
	uint16_t area_id;
} __packed;

struct msg_area_unshare_resp {
	uint16_t area_id;
	uint16_t result;
} __packed;

struct msg_reset_resp {
	uint16_t result;
} __packed;

struct msg_event_configure_req {
	uint8_t type;
	uint8_t reserved;
	uint16_t notify_id;
} __packed;

struct msg_event_configure_resp {
	uint16_t result;
} __packed;

/*
 * struct virtio_area - one FF-A shared-memory area retrieved for an endpoint.
 * @mobj:	the memory object backing the area (both SPMC topologies).
 * @handle:	FF-A memory handle used to retrieve and later relinquish it.
 * @id:		area id assigned by the driver.
 * @link:	list linkage in the owning endpoint.
 */
struct virtio_area {
	struct mobj *mobj;
	uint64_t handle;
	uint16_t id;
	TAILQ_ENTRY(virtio_area) link;
};

TAILQ_HEAD(virtio_area_head, virtio_area);

/*
 * struct virtio_msg_ffa_ep - one non-secure FF-A endpoint.
 * @ffa_ep_id:	FF-A id of the non-secure endpoint this serves.
 * @bus:	the virtio-msg bus for this endpoint (its own device table).
 * @areas:	shared-memory areas retrieved for this endpoint.
 * @lock:	protects @areas and the event configuration.
 * @ev_type:	event delivery method negotiated via EVENT_CONFIGURE.
 * @ev_notify_id: FF-A notification id used for EVENT_USED (NOTIF_ASSISTED).
 * @link:	list linkage in ep_head.
 *
 * Per-endpoint state keeps VMs isolated: each has an independent bus, device
 * table and area list (unlike the single global lists of the earlier PoC).
 */
struct virtio_msg_ffa_ep {
	uint16_t ffa_ep_id;
	struct virtio_msg_bus bus;
	struct virtio_area_head areas;
	unsigned int lock;
	uint8_t ev_type;
	uint16_t ev_notify_id;
	TAILQ_ENTRY(virtio_msg_ffa_ep) link;
};

TAILQ_HEAD(virtio_msg_ffa_ep_head, virtio_msg_ffa_ep);
static struct virtio_msg_ffa_ep_head ep_head =
	TAILQ_HEAD_INITIALIZER(ep_head);
/* Serialises endpoint creation and lookup */
static unsigned int ep_head_lock = SPINLOCK_UNLOCK;

static struct virtio_msg_bus_driver *drivers[VIRTIO_MSG_FFA_MAX_DRV];
static unsigned int drivers_count;

/*
 * Area management.
 *
 * mobj_ffa_get_by_cookie() retrieves the shared memory whether the SPMC is
 * external (S-EL2, where it populates the mobj from the RX buffer) or internal
 * (S-EL1). No topology conditional is needed here; the only topology-specific
 * code in this file is the outbound notification in send_event_used().
 */
static struct virtio_area *ep_find_area(struct virtio_msg_ffa_ep *ep,
					uint16_t area_id)
{
	struct virtio_area *a = NULL;

	TAILQ_FOREACH(a, &ep->areas, link)
		if (a->id == area_id)
			return a;

	return NULL;
}

static TEE_Result ep_area_share(struct virtio_msg_ffa_ep *ep, uint16_t area_id,
				uint64_t handle)
{
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	a = calloc(1, sizeof(*a));
	if (!a)
		return TEE_ERROR_OUT_OF_MEMORY;

	a->mobj = mobj_ffa_get_by_cookie(handle, 0);
	if (!a->mobj) {
		free(a);
		return TEE_ERROR_GENERIC;
	}
	a->handle = handle;
	a->id = area_id;

	state = cpu_spin_lock_xsave(&ep->lock);
	TAILQ_INSERT_TAIL(&ep->areas, a, link);
	cpu_spin_unlock_xrestore(&ep->lock, state);

	return TEE_SUCCESS;
}

static TEE_Result ep_area_unshare(struct virtio_msg_ffa_ep *ep,
				  uint16_t area_id)
{
	TEE_Result res = TEE_ERROR_ITEM_NOT_FOUND;
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&ep->lock);
	a = ep_find_area(ep, area_id);
	if (a) {
		res = mobj_ffa_unregister_by_cookie(a->handle);
		if (!res || res == TEE_ERROR_ITEM_NOT_FOUND)
			TAILQ_REMOVE(&ep->areas, a, link);
	}
	cpu_spin_unlock_xrestore(&ep->lock, state);

	if (a && (!res || res == TEE_ERROR_ITEM_NOT_FOUND)) {
		mobj_put(a->mobj);
		free(a);
	}

	return res;
}

/*
 * Bus ops (struct virtio_msg_bus_ops), provided to the transport/bus layer.
 * The cookie is the owning endpoint.
 */
static void *virtio_msg_ffa_map_area(void *cookie, uint64_t bus_addr,
				     size_t len)
{
	struct virtio_msg_ffa_ep *ep = cookie;
	uint16_t area_id = (bus_addr >> AREA_ID_SHIFT) & UINT16_MAX;
	size_t offset = bus_addr & AREA_OFFSET_MASK;
	struct virtio_area *a = NULL;
	uint32_t state = 0;
	void *va = NULL;

	state = cpu_spin_lock_xsave(&ep->lock);
	a = ep_find_area(ep, area_id);
	if (a) {
		mobj_get(a->mobj);
		if (mobj_inc_map(a->mobj)) {
			mobj_put(a->mobj);
			a = NULL;
		}
	}
	cpu_spin_unlock_xrestore(&ep->lock, state);

	if (!a)
		return NULL;

	va = mobj_get_va(a->mobj, offset, len);
	if (!va) {
		mobj_dec_map(a->mobj);
		mobj_put(a->mobj);
	}

	return va;
}

static void virtio_msg_ffa_unmap_area(void *cookie, void *va, size_t len)
{
	struct virtio_msg_ffa_ep *ep = cookie;
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	if (!va)
		return;

	/*
	 * Find the area whose mapping contains @va so the map count is
	 * balanced against the matching virtio_msg_ffa_map_area().
	 */
	state = cpu_spin_lock_xsave(&ep->lock);
	TAILQ_FOREACH(a, &ep->areas, link) {
		void *base = mobj_get_va(a->mobj, 0, 1);
		size_t sz = a->mobj->size;

		if (base && (uint8_t *)va >= (uint8_t *)base &&
		    (uint8_t *)va + len <= (uint8_t *)base + sz)
			break;
	}
	cpu_spin_unlock_xrestore(&ep->lock, state);

	if (a) {
		mobj_dec_map(a->mobj);
		mobj_put(a->mobj);
	}
}

static void virtio_msg_ffa_send_event_used(struct virtio_msg_dev *vmdev,
					   int qid __unused)
{
	struct virtio_msg_ffa_ep *ep = vmdev->bus->ops_cookie;

	if (ep->ev_type != FFA_BUS_EVENT_NOTIF_ASSISTED)
		return;

#if !defined(CFG_CORE_SEL1_SPMC)
	if (spmc_ffa_set_notification(ep->ffa_ep_id, 0,
				      BIT64(ep->ev_notify_id)))
		DMSG("EVENT_USED notification to %#"PRIx16" failed",
		     ep->ffa_ep_id);
#else
	DMSG("EVENT_USED notification unsupported with S-EL1 SPMC");
#endif
}

static const struct virtio_msg_bus_ops virtio_msg_ffa_bus_ops = {
	.map_area = virtio_msg_ffa_map_area,
	.unmap_area = virtio_msg_ffa_unmap_area,
	.send_event_used = virtio_msg_ffa_send_event_used,
};

/* Endpoint management */
static struct virtio_msg_ffa_ep *ep_alloc(uint16_t ffa_ep_id)
{
	struct virtio_msg_ffa_ep *ep = NULL;
	bool any = false;
	size_t i = 0;

	ep = calloc(1, sizeof(*ep));
	if (!ep)
		return NULL;

	ep->ffa_ep_id = ffa_ep_id;
	ep->lock = SPINLOCK_UNLOCK;
	ep->ev_type = FFA_BUS_EVENT_POLLING;
	TAILQ_INIT(&ep->areas);
	virtio_msg_bus_init(&ep->bus, &virtio_msg_ffa_bus_ops, ep);

	/* Let every registered driver attach its device to this bus */
	for (i = 0; i < drivers_count; i++) {
		if (!virtio_msg_bus_attach_driver(&ep->bus, drivers[i]))
			any = true;
	}

	if (!any) {
		free(ep);
		return NULL;
	}

	return ep;
}

static struct virtio_msg_ffa_ep *ep_lookup(uint16_t ffa_ep_id)
{
	struct virtio_msg_ffa_ep *ep = NULL;

	TAILQ_FOREACH(ep, &ep_head, link)
		if (ep->ffa_ep_id == ffa_ep_id)
			return ep;

	return NULL;
}

static struct virtio_msg_ffa_ep *ep_find_or_create(uint16_t ffa_ep_id)
{
	struct virtio_msg_ffa_ep *ep = NULL;
	struct virtio_msg_ffa_ep *raced = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&ep_head_lock);
	ep = ep_lookup(ffa_ep_id);
	cpu_spin_unlock_xrestore(&ep_head_lock, state);
	if (ep)
		return ep;

	/*
	 * Allocation and driver init run outside the list lock (they may
	 * allocate). Re-check under the lock before publishing so a concurrent
	 * caller for the same endpoint does not create a duplicate; if we lost
	 * the race, drop ours and use the published one.
	 */
	ep = ep_alloc(ffa_ep_id);
	if (!ep)
		return NULL;

	state = cpu_spin_lock_xsave(&ep_head_lock);
	raced = ep_lookup(ffa_ep_id);
	if (!raced)
		TAILQ_INSERT_TAIL(&ep_head, ep, link);
	cpu_spin_unlock_xrestore(&ep_head_lock, state);

	if (raced) {
		free(ep);
		return raced;
	}

	return ep;
}

/* FF-A bus message handlers */
static void handle_version(struct virtio_msg *msg)
{
	struct msg_version_req *req = (void *)msg->payload;
	struct msg_version_resp *resp = (void *)msg->payload;

	if (msg->dev_id || msg->msg_size != sizeof(*msg) + sizeof(*req) ||
	    req->proto_vers != FFA_PROTO_VERSION ||
	    req->vmsg_rev != VMSG_REVISION) {
		msg->dev_id = 0;
		memset(resp, 0, sizeof(*resp));
		goto out;
	}

	memset(resp, 0, sizeof(*resp));
	resp->proto_vers = FFA_PROTO_VERSION;
	resp->vmsg_rev = VMSG_REVISION;
	resp->vmsg_features = VMSG_FEATURES;
	resp->features = FFA_BUS_FEATURE_DIRECT_MSG_RX |
			 FFA_BUS_FEATURE_FFA_NOTIF_TX;
	resp->area_num = 0xff;
out:
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_area_share(struct virtio_msg *msg,
			      struct virtio_msg_ffa_ep *ep)
{
	struct msg_area_share_req *req = (void *)msg->payload;
	struct msg_area_share_resp *resp = (void *)msg->payload;
	uint64_t handle = req->handle;
	uint16_t area_id = req->area_id;

	if (msg->dev_id || msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		msg->dev_id = 0;
		memset(resp, 0, sizeof(*resp));
		goto out;
	}

	memset(resp, 0, sizeof(*resp));
	resp->area_id = area_id;
	if (ep_area_share(ep, area_id, handle))
		resp->result = FFA_RESULT_ERROR;
out:
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_area_unshare(struct virtio_msg *msg,
				struct virtio_msg_ffa_ep *ep)
{
	struct msg_area_unshare_req *req = (void *)msg->payload;
	struct msg_area_unshare_resp *resp = (void *)msg->payload;
	uint16_t area_id = req->area_id;
	TEE_Result res = TEE_SUCCESS;

	if (msg->dev_id || msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		msg->dev_id = 0;
		memset(resp, 0, sizeof(*resp));
		goto out;
	}

	memset(resp, 0, sizeof(*resp));
	resp->area_id = area_id;
	res = ep_area_unshare(ep, area_id);
	if (res == TEE_ERROR_BUSY)
		resp->result = FFA_UNSHARE_BUSY;
	else if (res)
		resp->result = FFA_RESULT_ERROR;
out:
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_event_configure(struct virtio_msg *msg,
				   struct virtio_msg_ffa_ep *ep)
{
	struct msg_event_configure_req *req = (void *)msg->payload;
	struct msg_event_configure_resp *resp = (void *)msg->payload;
	uint32_t state = 0;

	if (msg->dev_id || msg->msg_size != sizeof(*msg) + sizeof(*req)) {
		msg->dev_id = 0;
		memset(resp, 0, sizeof(*resp));
		resp->result = FFA_RESULT_ERROR;
		goto out;
	}

	memset(resp, 0, sizeof(*resp));
	switch (req->type) {
	case FFA_BUS_EVENT_POLLING:
		state = cpu_spin_lock_xsave(&ep->lock);
		ep->ev_type = req->type;
		cpu_spin_unlock_xrestore(&ep->lock, state);
		break;
	case FFA_BUS_EVENT_NOTIF_ASSISTED:
		state = cpu_spin_lock_xsave(&ep->lock);
		ep->ev_type = req->type;
		ep->ev_notify_id = req->notify_id;
		cpu_spin_unlock_xrestore(&ep->lock, state);
		break;
	case FFA_BUS_EVENT_INDIRECT:
	default:
		resp->result = FFA_RESULT_ERROR;
		break;
	}
out:
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void handle_reset(struct virtio_msg *msg)
{
	struct msg_reset_resp *resp = (void *)msg->payload;

	/*
	 * A full bus reset would unregister devices, tear down virtqueues and
	 * unshare all areas. That teardown is not implemented yet, so report a
	 * failure rather than pretending success.
	 */
	memset(resp, 0, sizeof(*resp));
	resp->result = FFA_RESULT_ERROR;
	msg->dev_id = 0;
	msg->msg_size = sizeof(*msg) + sizeof(*resp);
}

static void recv_bus_msg(struct virtio_msg *msg, struct virtio_msg_ffa_ep *ep)
{
	switch (msg->msg_id) {
	case FFA_BUS_MSG_VERSION:
		handle_version(msg);
		break;
	case FFA_BUS_MSG_AREA_SHARE:
		handle_area_share(msg, ep);
		break;
	case FFA_BUS_MSG_AREA_UNSHARE:
		handle_area_unshare(msg, ep);
		break;
	case FFA_BUS_MSG_EVENT_CONFIGURE:
		handle_event_configure(msg, ep);
		break;
	case FFA_BUS_MSG_RESET:
		handle_reset(msg);
		break;
	default:
		/* Fall back to the transport/bus core (GET_DEVICES, PING) */
		virtio_msg_recv(msg, VIRTIO_MSG_MAX_SIZE, &ep->bus);
		break;
	}
}

void virtio_msg_ffa_recv(struct thread_smc_1_2_regs *args, uint16_t caller_id)
{
	void *payload = &args->a[4];
	struct virtio_msg *msg = payload;
	struct virtio_msg_ffa_ep *ep = NULL;
	bool is_bus = msg->type & VIRTIO_MSG_TYPE_BUS;
	uint8_t ffa_bus_id = msg->msg_id;

	FMSG("caller %#"PRIx16" type %#x msg_id %#x dev_id %#x msg_size %#x",
	     caller_id, msg->type, msg->msg_id, msg->dev_id, msg->msg_size);

	ep = ep_find_or_create(caller_id);
	if (!ep) {
		msg->dev_id = 0;
		msg->msg_size = sizeof(*msg);
		goto out;
	}

	/*
	 * FF-A bus framing carries both the FF-A-specific bus messages
	 * (VERSION, AREA_SHARE, AREA_UNSHARE, EVENT_CONFIGURE, RESET; all with
	 * the bus type bit set) and the transport-independent virtio-msg
	 * requests. Route the former here and hand everything else to the core.
	 */
	if (is_bus && (ffa_bus_id == FFA_BUS_MSG_VERSION ||
		       ffa_bus_id == FFA_BUS_MSG_AREA_SHARE ||
		       ffa_bus_id == FFA_BUS_MSG_AREA_UNSHARE ||
		       ffa_bus_id == FFA_BUS_MSG_EVENT_CONFIGURE ||
		       ffa_bus_id == FFA_BUS_MSG_RESET))
		recv_bus_msg(msg, ep);
	else
		virtio_msg_recv(msg, VIRTIO_MSG_MAX_SIZE, &ep->bus);

	msg->type |= VIRTIO_MSG_TYPE_RESPONSE;
out:
	/* Zero-pad the tail so no stale data leaks back to the driver */
	if (msg->msg_size < VIRTIO_MSG_MAX_SIZE)
		memset((uint8_t *)msg + msg->msg_size, 0,
		       VIRTIO_MSG_MAX_SIZE - msg->msg_size);
}

TEE_Result virtio_msg_ffa_register_driver(struct virtio_msg_bus_driver *drv)
{
	if (drivers_count == VIRTIO_MSG_FFA_MAX_DRV)
		return TEE_ERROR_OUT_OF_MEMORY;

	drivers[drivers_count++] = drv;

	return TEE_SUCCESS;
}

/*
 * Bottom-half: the driver's EVENT_AVAIL kick raises the async notification;
 * here we sweep every ready queue of every device on every endpoint and run
 * its notify callback.
 *
 * A device's notify callback may sleep (take a mutex, allocate), so it must
 * not run under ep_head_lock. Endpoints are only ever added, never removed,
 * so an endpoint pointer stays valid across dropping the lock: save the next
 * link before releasing, sweep the current endpoint unlocked, then resume.
 */
static void virtio_msg_ffa_yielding_cb(struct notif_driver *ndrv __unused,
				       enum notif_event ev)
{
	struct virtio_msg_ffa_ep *ep = NULL;
	struct virtio_msg_ffa_ep *next = NULL;
	uint32_t state = 0;
	size_t i = 0;
	int q = 0;

	if (ev != NOTIF_EVENT_DO_BOTTOM_HALF)
		return;

	state = cpu_spin_lock_xsave(&ep_head_lock);
	ep = TAILQ_FIRST(&ep_head);
	while (ep) {
		next = TAILQ_NEXT(ep, link);
		cpu_spin_unlock_xrestore(&ep_head_lock, state);

		for (i = 0; i < VIRTIO_MSG_BUS_MAX_DEVS; i++) {
			struct virtio_msg_dev *vmdev = ep->bus.devs[i];
			struct vdevice *vdev = NULL;

			if (!vmdev)
				continue;
			vdev = &vmdev->vdev;
			for (q = 0; q < vdev->num_queues; q++) {
				struct vdevice_vq *vq = &vdev->vqs[q];

				if (vq->ready && vq->notify)
					vq->notify(vdev, vq);
			}
		}

		state = cpu_spin_lock_xsave(&ep_head_lock);
		ep = next;
	}
	cpu_spin_unlock_xrestore(&ep_head_lock, state);
}

static struct notif_driver virtio_msg_ffa_notif __nex_data = {
	.yielding_cb = virtio_msg_ffa_yielding_cb,
};

static TEE_Result virtio_msg_ffa_init(void)
{
	notif_register_driver(&virtio_msg_ffa_notif);

	return TEE_SUCCESS;
}

nex_service_init_late(virtio_msg_ffa_init);
