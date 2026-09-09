// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * FF-A binding for the virtio-msg transport (DEN0153 "Virtio Message Bus over
 * FF-A"). This is the L1 layer: it provides the single
 * struct virtio_transport_ops instance used by the transport-agnostic backend
 * core and codec, manages the shared-memory areas the driver lends, and
 * receives virtio-msg messages carried in FFA_MSG_SEND_DIRECT_REQ2, dispatched
 * to this module by virtio UUID on OP-TEE's core endpoint.
 *
 * Memory sharing is mode-agnostic: mobj_ffa_get_by_cookie() transparently
 * issues FFA_MEM_RETRIEVE_REQ under an external SPMC (SEL2) and pops a
 * pre-built mobj from the inactive list when OP-TEE is the S-EL1 SPMC.
 */

#include <drivers/virtio/virtio_dev.h>
#include <drivers/virtio/virtio_msg.h>
#include <drivers/virtio/virtio_msg_ffa.h>
#include <drivers/virtio/virtio_transport.h>
#include <initcall.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/thread_spmc.h>
#include <mm/mobj.h>
#include <string.h>
#include <sys/queue.h>
#include <trace.h>
#include <util.h>

#define VIRTIO_MSG_FFA_AREA_ID_MASK	0xffff
#define VIRTIO_MSG_FFA_AREA_ID_SHIFT	48

/* FF-A specific bus message opcodes (DEN0153). */
#define FFA_BUS_MSG_VERSION		0x80
#define FFA_BUS_MSG_AREA_SHARE		0x81
#define FFA_BUS_MSG_AREA_UNSHARE	0x82
#define FFA_BUS_MSG_RESET		0x83
#define FFA_BUS_MSG_EVENT_POLL		0x84
#define FFA_BUS_MSG_EVENT_CONFIGURE	0x85
#define FFA_BUS_MSG_AREA_RELEASE	0xc0

/* Transport feature bits advertised in the VERSION response. */
#define FFA_BUS_FEATURE_DIRECT_MSG_RX	BIT(0)
#define FFA_BUS_FEATURE_DIRECT_MSG_TX	BIT(1)
#define FFA_BUS_FEATURE_INDIRECT_MSG_RX	BIT(2)
#define FFA_BUS_FEATURE_INDIRECT_MSG_TX	BIT(3)
#define FFA_BUS_FEATURE_FFA_NOTIF_RX	BIT(4)
#define FFA_BUS_FEATURE_FFA_NOTIF_TX	BIT(5)
#define FFA_BUS_FEATURE_FIFO		BIT(6)

/* FF-A direct messaging carries at most 96 bytes of payload (a4..a17). */
#define FFA_MSG_SIZE			96

#define FFA_PROTO_VERSION		0x00010000
#define VMSG_REVISION			1
#define VMSG_FEATURES			0

/*
 * Virtio device (backend) endpoint UUID c66028b5-2498-4aa1-9de7-77da6122abf0,
 * as little-endian 32-bit words for REQ2 dispatch (args->a2/a3).
 */
#define VIRTIO_MSG_FFA_UUID_WORDS \
	{ 0xb52860c6, 0xa14a9824, 0xda77e79d, 0xf0ab2261 }

struct virtio_msg_version_req {
	uint32_t proto_vers;
	uint32_t vmsg_rev;
};

struct virtio_msg_version_resp {
	uint32_t proto_vers;
	uint32_t vmsg_rev;
	uint32_t vmsg_features;
	uint32_t features;
	uint16_t area_max_count;
} __packed __aligned(2);

struct virtio_msg_ffa_area_share_req {
	uint16_t area_id;
	uint64_t handle;
	uint64_t tag;
	uint32_t page_count;
	uint32_t attrs;
} __packed __aligned(2);

struct virtio_msg_ffa_area_share_resp {
	uint16_t area_id;
	uint16_t result;
};

struct virtio_msg_ffa_area_unshare_req {
	uint16_t area_id;
};

struct virtio_msg_ffa_area_unshare_resp {
	uint16_t area_id;
	uint16_t result;
};

struct virtio_msg_ffa_reset_resp {
	uint16_t result;
};

struct virtio_msg_ffa_event_config_req {
	uint8_t type;
	uint8_t reserved;
	uint16_t notify_id;
};

struct virtio_msg_ffa_event_config_resp {
	uint16_t result;
};

/*
 * struct virtio_area - a shared-memory area lent by the driver.
 *
 * @mobj:	Memory object obtained from the FF-A handle (cookie).
 * @handle:	FF-A memory handle (cookie).
 * @id:		Driver-assigned area id (high 16 bits of a bus address).
 * @link:	Area list linkage.
 */
struct virtio_area {
	struct mobj *mobj;
	uint64_t handle;
	uint16_t id;
	TAILQ_ENTRY(virtio_area) link;
};

TAILQ_HEAD(virtio_area_head, virtio_area);
static struct virtio_area_head areas_head = TAILQ_HEAD_INITIALIZER(areas_head);
static unsigned int areas_lock = SPINLOCK_UNLOCK;

static uint16_t area_id_from_bus_addr(uint64_t bus_addr)
{
	return (bus_addr >> VIRTIO_MSG_FFA_AREA_ID_SHIFT) &
	       VIRTIO_MSG_FFA_AREA_ID_MASK;
}

static struct virtio_area *find_virtio_area(uint16_t area_id)
{
	struct virtio_area *a = NULL;

	TAILQ_FOREACH(a, &areas_head, link)
		if (a->id == area_id)
			return a;

	return NULL;
}

static TEE_Result virtio_area_share(uint16_t area_id, uint64_t handle)
{
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	a = calloc(1, sizeof(*a));
	if (!a)
		return TEE_ERROR_OUT_OF_MEMORY;

	a->mobj = mobj_ffa_get_by_cookie(handle, MOBJ_USE_CASE_NS_SHM);
	if (!a->mobj) {
		free(a);
		return TEE_ERROR_GENERIC;
	}

	a->handle = handle;
	a->id = area_id;

	state = cpu_spin_lock_xsave(&areas_lock);
	TAILQ_INSERT_TAIL(&areas_head, a, link);
	cpu_spin_unlock_xrestore(&areas_lock, state);

	return TEE_SUCCESS;
}

static TEE_Result virtio_area_unshare(uint16_t area_id)
{
	TEE_Result res = TEE_ERROR_ITEM_NOT_FOUND;
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&areas_lock);
	a = find_virtio_area(area_id);
	if (a) {
		res = mobj_ffa_unregister_by_cookie(a->handle);
		if (!res || res == TEE_ERROR_ITEM_NOT_FOUND)
			TAILQ_REMOVE(&areas_head, a, link);
	}
	cpu_spin_unlock_xrestore(&areas_lock, state);

	if (!res)
		free(a);

	return res;
}

static void virtio_area_unshare_all(void)
{
	struct virtio_area *a = NULL;
	struct virtio_area *tmp = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&areas_lock);
	TAILQ_FOREACH_SAFE(a, &areas_head, link, tmp) {
		if (!mobj_ffa_unregister_by_cookie(a->handle) ||
		    mobj_ffa_unregister_by_cookie(a->handle) ==
		    TEE_ERROR_ITEM_NOT_FOUND) {
			TAILQ_REMOVE(&areas_head, a, link);
			free(a);
		}
	}
	cpu_spin_unlock_xrestore(&areas_lock, state);
}

/* struct virtio_transport_ops implementation. */

static void *virtio_ffa_bus_addr_to_va(const struct virtio_transport_ops *ops
				       __unused, uint64_t bus_addr, size_t len)
{
	struct virtio_area *a = NULL;
	uint16_t area_id = 0;
	uint32_t state = 0;
	size_t offs = 0;

	area_id = area_id_from_bus_addr(bus_addr);
	offs = bus_addr & ~SHIFT_U64(VIRTIO_MSG_FFA_AREA_ID_MASK,
				     VIRTIO_MSG_FFA_AREA_ID_SHIFT);

	state = cpu_spin_lock_xsave(&areas_lock);
	a = find_virtio_area(area_id);
	cpu_spin_unlock_xrestore(&areas_lock, state);
	if (!a)
		return NULL;

	return mobj_get_va(a->mobj, offs, len);
}

static TEE_Result virtio_ffa_map_area(const struct virtio_transport_ops *ops
				      __unused, uint64_t bus_addr)
{
	TEE_Result res = TEE_SUCCESS;
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&areas_lock);
	a = find_virtio_area(area_id_from_bus_addr(bus_addr));
	if (a)
		mobj_get(a->mobj);
	cpu_spin_unlock_xrestore(&areas_lock, state);

	if (!a)
		return TEE_ERROR_BAD_PARAMETERS;

	res = mobj_inc_map(a->mobj);
	if (res)
		mobj_put(a->mobj);

	return res;
}

static void virtio_ffa_unmap_area(const struct virtio_transport_ops *ops
				  __unused, uint64_t bus_addr)
{
	struct virtio_area *a = NULL;
	uint32_t state = 0;

	state = cpu_spin_lock_xsave(&areas_lock);
	a = find_virtio_area(area_id_from_bus_addr(bus_addr));
	cpu_spin_unlock_xrestore(&areas_lock, state);

	if (a && !mobj_dec_map(a->mobj))
		mobj_put(a->mobj);
}

static const struct virtio_transport_ops virtio_ffa_ops = {
	.bus_addr_to_va = virtio_ffa_bus_addr_to_va,
	.map_area = virtio_ffa_map_area,
	.unmap_area = virtio_ffa_unmap_area,
	/* Device->driver events use the EVENT_POLL model; no send_event yet. */
	.send_event = NULL,
	.max_msg_payload = FFA_MSG_SIZE,
};

/* FF-A specific bus message handlers. */

static void handle_bus_msg_version(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_version_req *req = (void *)(hdr + 1);
	struct virtio_msg_version_resp *resp = (void *)(hdr + 1);

	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr) + sizeof(*req) ||
	    req->proto_vers != FFA_PROTO_VERSION ||
	    req->vmsg_rev != VMSG_REVISION) {
		hdr->dev_num = 0;
		memset(resp, 0, sizeof(*resp));
		goto out;
	}

	memset(resp, 0, sizeof(*resp));
	resp->proto_vers = FFA_PROTO_VERSION;
	resp->vmsg_rev = VMSG_REVISION;
	resp->vmsg_features = VMSG_FEATURES;
	/* Direct-message receive only for now. */
	resp->features = FFA_BUS_FEATURE_DIRECT_MSG_RX;
	resp->area_max_count = 0xff;
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_bus_msg_area_share(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_ffa_area_share_req *req = (void *)(hdr + 1);
	struct virtio_msg_ffa_area_share_resp *resp = (void *)(hdr + 1);
	uint64_t handle = req->handle;
	uint16_t area_id = req->area_id;

	memset(resp, 0, sizeof(*resp));
	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr) + sizeof(*req)) {
		hdr->dev_num = 0;
		goto out;
	}
	resp->area_id = area_id;

	if (virtio_area_share(area_id, handle))
		resp->result = 1;
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_bus_msg_area_unshare(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_ffa_area_unshare_req *req = (void *)(hdr + 1);
	struct virtio_msg_ffa_area_unshare_resp *resp = (void *)req;
	uint16_t area_id = req->area_id;
	TEE_Result res = TEE_SUCCESS;

	memset(resp, 0, sizeof(*resp));
	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr) + sizeof(*req)) {
		hdr->dev_num = 0;
		goto out;
	}
	resp->area_id = area_id;

	res = virtio_area_unshare(area_id);
	if (res == TEE_ERROR_BUSY)
		resp->result = 2;
	else if (res)
		resp->result = 1;
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_bus_msg_reset(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_ffa_reset_resp *resp = (void *)(hdr + 1);

	memset(resp, 0, sizeof(*resp));
	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr)) {
		hdr->dev_num = 0;
		resp->result = 1;
		goto out;
	}

	/*
	 * Tear down all backend state: reset each device (disabling and
	 * detaching its virtqueues) and unshare every area. Areas are unshared
	 * only after the devices release their ring mappings.
	 */
	virtio_dev_reset_all();
	virtio_area_unshare_all();
out:
	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void handle_bus_msg_event_configure(struct virtio_msg_hdr *hdr)
{
	struct virtio_msg_ffa_event_config_req *req = (void *)(hdr + 1);
	struct virtio_msg_ffa_event_config_resp *resp = (void *)req;

	memset(resp, 0, sizeof(*resp));
	if (hdr->dev_num || hdr->msg_size != sizeof(*hdr) + sizeof(*req))
		hdr->dev_num = 0;

	hdr->msg_size = sizeof(*hdr) + sizeof(*resp);
}

static void recv_bus_req(struct virtio_msg_hdr *hdr)
{
	switch (hdr->msg_id) {
	case FFA_BUS_MSG_VERSION:
		handle_bus_msg_version(hdr);
		break;
	case FFA_BUS_MSG_AREA_SHARE:
		handle_bus_msg_area_share(hdr);
		break;
	case FFA_BUS_MSG_AREA_UNSHARE:
		handle_bus_msg_area_unshare(hdr);
		break;
	case FFA_BUS_MSG_RESET:
		handle_bus_msg_reset(hdr);
		break;
	case FFA_BUS_MSG_EVENT_CONFIGURE:
		handle_bus_msg_event_configure(hdr);
		break;
	default:
		/* Generic bus opcodes (GET_DEVICES, PING) are role-neutral. */
		virtio_msg_handle_bus_req(hdr, &virtio_ffa_ops, FFA_MSG_SIZE);
		return;
	}
	hdr->type = MSG_TYPE_BUS_RESPONSE;
}

void virtio_msg_ffa_recv(struct thread_smc_1_2_regs *args)
{
	struct virtio_msg_hdr *hdr = (struct virtio_msg_hdr *)(args->a + 4);

	FMSG("hdr type %#x msg_id %#x dev_num %#x msg_size %#x",
	     hdr->type, hdr->msg_id, hdr->dev_num, hdr->msg_size);
	switch (hdr->type) {
	case MSG_TYPE_BUS_REQUEST:
		recv_bus_req(hdr);
		break;
	case MSG_TYPE_TRANSPORT_REQUEST:
		virtio_msg_handle_transport_req(hdr, &virtio_ffa_ops);
		break;
	default:
		virtio_msg_set_null_msg(hdr);
	}

	assert(hdr->msg_size <= FFA_MSG_SIZE);
	memset((uint8_t *)hdr + hdr->msg_size, 0, FFA_MSG_SIZE - hdr->msg_size);
}

static struct spmc_uuid_handler virtio_uuid_handler __nex_data = {
	.uuid_words = VIRTIO_MSG_FFA_UUID_WORDS,
	.recv = virtio_msg_ffa_recv,
	.name = "virtio-msg",
};

static TEE_Result virtio_msg_ffa_init(void)
{
	return spmc_register_uuid_handler(&virtio_uuid_handler);
}
nex_service_init_late(virtio_msg_ffa_init);
