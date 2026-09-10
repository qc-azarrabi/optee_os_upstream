// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2025, Linaro Limited
 */

#include <assert.h>
#include <initcall.h>
#include <kernel/delay.h>
#include <kernel/mutex.h>
#include <kernel/virtio_msg.h>
#include <kernel/virtio_msg_ffa.h>
#include <kernel/virtio_vsock.h>
#include <malloc.h>
#include <mm/core_mmu.h>
#include <string.h>
#include <sys/queue.h>
#include <trace.h>
#include <util.h>

/*
 * virtio-vsock device (backend), see the virtio spec "Socket Device".
 *
 * This is a device-type driver on the virtio-msg bus (struct virtio_bus_driver).
 * One instance is attached to every bus (FF-A endpoint). The device-facing side
 * is the vdevice/virtio-msg transport; the TA-facing side is the GP socket API
 * exposed through kernel/virtio_vsock.h.
 *
 * Two virtqueues carry data (the third, EVENT, is unused by this backend):
 *   RX (0): device -> driver. We push packets from the per-device send queue
 *           into the driver's receive buffers (vsock_do_send).
 *   TX (1): driver -> device. We pull packets the driver transmitted and act on
 *           them (vsock_tx_process -> handle_op_*).
 *
 * The RX path follows QTEE's vhost_vsock deferred send-queue model: outbound
 * packets are queued (vsock_send_queue) and drained into whatever RX buffers the
 * driver has posted, splitting a packet across several buffers when needed.
 */

/* Device-specific feature bit (device-independent bits come from the core) */
#define VIRTIO_VSOCK_F_SEQPACKET	1

/* virtio device id and a fixed config-space generation, see the spec */
#define VIRTIO_VSOCK_DEVICE_ID		19
#define VIRTIO_VSOCK_CONFIG_GEN		100

/* Well-known context ids */
#define VIRTIO_VSOCK_CID_HOST		2

/* Packet operations, see the virtio spec "Socket Device Operation" */
#define VIRTIO_VSOCK_OP_INVALID		0
#define VIRTIO_VSOCK_OP_REQUEST		1
#define VIRTIO_VSOCK_OP_RESPONSE	2
#define VIRTIO_VSOCK_OP_RST		3
#define VIRTIO_VSOCK_OP_SHUTDOWN	4
#define VIRTIO_VSOCK_OP_RW		5
#define VIRTIO_VSOCK_OP_CREDIT_UPDATE	6
#define VIRTIO_VSOCK_OP_CREDIT_REQUEST	7

#define VIRTIO_VSOCK_SHUTDOWN_F_RECEIVE	0
#define VIRTIO_VSOCK_SHUTDOWN_F_SEND	1

/* Upper bound on a single received packet payload (bounds the TX allocation) */
#define VIRTIO_VSOCK_MAX_PKT_SIZE	0x10000

enum virtio_vsock_vq_idx {
	VIRTIO_VSOCK_VQ_IDX_RX = 0,
	VIRTIO_VSOCK_VQ_IDX_TX,
	VIRTIO_VSOCK_VQ_IDX_EVENT,
	VIRTIO_VSOCK_VQ_IDX_COUNT,
};

/* On-wire packet header, little-endian (virtio spec "Device Operation") */
struct virtio_vsock_hdr {
	uint64_t src_cid;
	uint64_t dst_cid;
	uint32_t src_port;
	uint32_t dst_port;
	uint32_t len;
	uint16_t type;
	uint16_t op;
	uint32_t flags;
	uint32_t buf_alloc;
	uint32_t fwd_cnt;
} __packed;

#define VIRTIO_VSOCK_HDR_SIZE		sizeof(struct virtio_vsock_hdr)

/*
 * struct virtio_vsock_pkt - one outbound packet awaiting an RX buffer.
 * @hdr:	packet header (mutated in place when a packet is split).
 * @data:	payload buffer (NULL for a header-only packet).
 * @len:	payload length.
 * @offset:	payload bytes already delivered to the driver.
 */
struct virtio_vsock_pkt {
	struct virtio_vsock_hdr hdr;
	void *data;
	size_t len;
	size_t offset;
	TAILQ_ENTRY(virtio_vsock_pkt) link;
};

TAILQ_HEAD(virtio_vsock_pkt_head, virtio_vsock_pkt);

/*
 * struct virtio_vsock_device - the vsock backend on one bus.
 * @vdev:	the device core we serve; owned by the virtio-msg framework.
 * @cid:	this device's context id (exposed as the config space).
 * @vqs:	the three virtqueues (RX, TX, EVENT).
 * @send_queue:	outbound packets pending delivery on the RX queue.
 *
 * Hung off @vdev->priv. This device type only ever sees struct vdevice; the
 * virtio-msg transport wrapper is owned and hidden by the framework.
 */
struct virtio_vsock_device {
	struct vdevice *vdev;
	uint64_t cid;
	struct vdevice_vq vqs[VIRTIO_VSOCK_VQ_IDX_COUNT];
	struct virtio_vsock_pkt_head send_queue;
};

TAILQ_HEAD(virtio_vsock_socket_head, virtio_vsock_socket);

/*
 * A single mutex serialises everything the device touches from thread context:
 * the socket lists, per-socket message queues, the send queue and all
 * virtqueue processing. The transport's notify callbacks and the GP socket API
 * both run in normal thread context (the async-notif bottom half releases its
 * spinlock before calling us), so a mutex is both legal and sufficient.
 */
static struct mutex sock_lock = MUTEX_INITIALIZER;
static size_t sock_msgs_wait_count;
static struct condvar sock_msgs_cv = CONDVAR_INITIALIZER;
static size_t sock_send_wait_count;
static struct condvar sock_send_cv = CONDVAR_INITIALIZER;
static struct virtio_vsock_socket_head sockets_head =
	TAILQ_HEAD_INITIALIZER(sockets_head);

static struct virtio_vsock_device *vdev_to_dev(struct vdevice *vdev)
{
	return vdev->priv;
}

/* Socket lookup helpers (all assume sock_lock held) */

static struct virtio_vsock_socket *find_listening_vsock(uint32_t port,
							uint16_t type)
{
	struct virtio_vsock_socket *v = NULL;

	TAILQ_FOREACH(v, &sockets_head, link)
		if (v->listen && v->type == type && v->src_port == port)
			return v;

	return NULL;
}

static bool cmp_socket(const struct virtio_vsock_socket *a,
		       const struct virtio_vsock_socket *b)
{
	return a->src_cid == b->src_cid && a->dst_cid == b->dst_cid &&
	       a->src_port == b->src_port && a->dst_port == b->dst_port &&
	       a->type == b->type;
}

static struct virtio_vsock_socket *
find_socket(const struct virtio_vsock_socket *ref)
{
	struct virtio_vsock_socket *v = NULL;

	TAILQ_FOREACH(v, &sockets_head, link)
		if (cmp_socket(v, ref))
			return v;

	return NULL;
}

static struct virtio_vsock_socket *find_socket2(uint64_t src_cid,
						uint64_t dst_cid,
						uint32_t src_port,
						uint32_t dst_port,
						uint16_t type)
{
	struct virtio_vsock_socket ref = {
		.src_cid = src_cid,
		.dst_cid = dst_cid,
		.src_port = src_port,
		.dst_port = dst_port,
		.type = type,
	};

	return find_socket(&ref);
}

static void add_msg(struct virtio_vsock_msg_head *msgs,
		    struct virtio_vsock_msg *m)
{
	TAILQ_INSERT_TAIL(msgs, m, link);
	if (sock_msgs_wait_count)
		condvar_broadcast(&sock_msgs_cv);
}

static TEE_Result add_socket(struct virtio_vsock_socket *vvs,
			     struct virtio_vsock_msg_head *msgs)
{
	struct virtio_vsock_msg *msg = NULL;

	if (msgs) {
		msg = calloc(1, sizeof(*msg));
		if (!msg)
			return TEE_ERROR_OUT_OF_MEMORY;
		msg->type = VIRTIO_VSOCKET_MSG_TYPE_REQ;
		msg->vvs_req = vvs;
	}

	if (find_socket(vvs)) {
		free(msg);
		return TEE_ERROR_ACCESS_CONFLICT;
	}

	TAILQ_INSERT_TAIL(&sockets_head, vvs, link);
	if (msg)
		add_msg(msgs, msg);

	return TEE_SUCCESS;
}

/*
 * Outbound RX path: enqueue a packet and drain into the driver's RX buffers.
 */

static struct virtio_vsock_pkt *
vsock_pkt_alloc(const struct virtio_vsock_hdr *hdr, const void *data,
		size_t len)
{
	struct virtio_vsock_pkt *pkt = calloc(1, sizeof(*pkt));

	if (!pkt)
		return NULL;

	pkt->hdr = *hdr;
	if (len) {
		pkt->data = malloc(len);
		if (!pkt->data) {
			free(pkt);
			return NULL;
		}
		memcpy(pkt->data, data, len);
		pkt->len = len;
	}

	return pkt;
}

static void vsock_pkt_free(struct virtio_vsock_device *dev,
			   struct virtio_vsock_pkt *pkt)
{
	TAILQ_REMOVE(&dev->send_queue, pkt, link);
	free(pkt->data);
	free(pkt);
}

/*
 * Drain the send queue into the RX virtqueue. Follows QTEE vhost_vsock: pop RX
 * descriptor chains and copy queued packets into them, splitting a packet that
 * does not fit into one chain and restoring its EOM/EOR record flags on the
 * final fragment. Assumes sock_lock held.
 */
static void vsock_do_send(struct virtio_vsock_device *dev)
{
	struct vdevice *vdev = dev->vdev;
	struct vdevice_vq *vq = &dev->vqs[VIRTIO_VSOCK_VQ_IDX_RX];
	bool added = false;

	if (!vq->ready)
		return;

	/* Suppress driver kicks while we drain */
	vdevice_disable_notify(vq);

	for (;;) {
		struct virtio_vsock_pkt *pkt = NULL;
		struct virtio_vsock_hdr *hdr = NULL;
		struct vdevice_sglist_cursor cur;
		struct vdevice_chain chain;
		size_t len = 0;
		size_t payload_len = 0;
		uint32_t saved_flags = 0;
		int err = 0;

		pkt = TAILQ_FIRST(&dev->send_queue);
		if (!pkt) {
			/* Nothing to send: allow the driver to kick us again */
			vdevice_enable_notify(vq);
			break;
		}

		err = vdevice_get_vq_desc(vq, &chain);
		if (err < 0)
			break;		/* Faulty queue: leave it alone */
		if (err == 1) {
			/* Ring empty: re-enable notify, re-scan on a race */
			if (vdevice_enable_notify(vq)) {
				vdevice_disable_notify(vq);
				continue;
			}
			break;
		}

		/* On the RX queue we expect only device-writable buffers */
		if (chain.out.sg) {
			vsock_pkt_free(dev, pkt);
			break;
		}

		len = vdevice_sglist_len(&chain.in);
		if (len < VIRTIO_VSOCK_HDR_SIZE) {
			vsock_pkt_free(dev, pkt);
			break;
		}

		hdr = &pkt->hdr;
		payload_len = pkt->len - pkt->offset;
		if (payload_len > len - VIRTIO_VSOCK_HDR_SIZE) {
			/* Packet larger than this buffer: split it */
			payload_len = len - VIRTIO_VSOCK_HDR_SIZE;

			/* Carry record markers only on the final fragment */
			saved_flags = hdr->flags & (VIRTIO_VSOCK_SEQ_EOM |
						    VIRTIO_VSOCK_SEQ_EOR);
			hdr->flags &= ~(VIRTIO_VSOCK_SEQ_EOM |
					VIRTIO_VSOCK_SEQ_EOR);
		}

		hdr->len = payload_len;

		vdevice_sglist_cursor_reset(&cur);
		if (vdevice_sglist_copy_to(vq, &chain.in, &cur, hdr,
					   VIRTIO_VSOCK_HDR_SIZE)) {
			vsock_pkt_free(dev, pkt);
			break;
		}
		if (payload_len &&
		    vdevice_sglist_copy_to(vq, &chain.in, &cur,
					   (uint8_t *)pkt->data + pkt->offset,
					   payload_len)) {
			vsock_pkt_free(dev, pkt);
			break;
		}

		vdevice_add_used_one(vq, chain.head,
				     VIRTIO_VSOCK_HDR_SIZE + payload_len);
		added = true;

		pkt->offset += payload_len;
		if (pkt->offset == pkt->len)
			vsock_pkt_free(dev, pkt);
		else
			hdr->flags |= saved_flags;
	}

	if (added)
		vdevice_signal(vdev, vq);
}

static void vsock_queue_and_send(struct virtio_vsock_device *dev,
				 struct virtio_vsock_pkt *pkt)
{
	TAILQ_INSERT_TAIL(&dev->send_queue, pkt, link);
	vsock_do_send(dev);
}

/* Enqueue a header-only packet described by @hdr */
static void vsock_reply(struct virtio_vsock_device *dev,
			const struct virtio_vsock_hdr *hdr)
{
	struct virtio_vsock_pkt *pkt = NULL;

	if (!dev)
		return;

	pkt = vsock_pkt_alloc(hdr, NULL, 0);
	if (pkt)
		vsock_queue_and_send(dev, pkt);
}

/* Enqueue a header-only packet built from a socket's tuple */
static void vsock_reply_op(struct virtio_vsock_socket *vvs, uint16_t op)
{
	struct virtio_vsock_hdr hdr = {
		.src_cid = vvs->src_cid,
		.dst_cid = vvs->dst_cid,
		.src_port = vvs->src_port,
		.dst_port = vvs->dst_port,
		.type = vvs->type,
		.op = op,
	};

	vsock_reply(vvs->dev, &hdr);
}

static void init_resp_hdr(struct virtio_vsock_hdr *resp,
			  const struct virtio_vsock_hdr *req, uint16_t op)
{
	*resp = (struct virtio_vsock_hdr){
		.src_cid = req->dst_cid,
		.dst_cid = req->src_cid,
		.src_port = req->dst_port,
		.dst_port = req->src_port,
		.type = req->type,
		.op = op,
	};
}

/*
 * Inbound TX path: handlers for the packet operations the driver sends. All run
 * under sock_lock (from vsock_tx_process).
 */

static void handle_op_request(struct virtio_vsock_device *dev,
			      struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *lvvs = NULL;
	struct virtio_vsock_socket *vvs = NULL;
	struct virtio_vsock_hdr resp = { };

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_RESPONSE);

	if (req->src_cid != dev->cid || !req->src_port) {
		DMSG("Bad request");
		resp.op = VIRTIO_VSOCK_OP_RST;
		goto out;
	}

	lvvs = find_listening_vsock(req->dst_port, req->type);
	if (!lvvs || lvvs->src_cid != req->dst_cid) {
		DMSG("No listener");
		resp.op = VIRTIO_VSOCK_OP_RST;
		goto out;
	}

	vvs = calloc(1, sizeof(*vvs));
	if (!vvs) {
		resp.op = VIRTIO_VSOCK_OP_RST;
		goto out;
	}
	vvs->src_cid = req->src_cid;
	vvs->dst_cid = req->dst_cid;
	vvs->src_port = req->src_port;
	vvs->dst_port = req->dst_port;
	vvs->type = req->type;
	vvs->owner = lvvs->owner;
	vvs->dev = dev;
	vvs->buf_alloc = SMALL_PAGE_SIZE;
	vvs->peer_buf_alloc = req->buf_alloc;
	vvs->peer_fwd_cnt = req->fwd_cnt;
	TAILQ_INIT(&vvs->msgs);

	if (add_socket(vvs, &lvvs->msgs)) {
		DMSG("Connection already exists");
		free(vvs);
		resp.op = VIRTIO_VSOCK_OP_RST;
		goto out;
	}

	resp.buf_alloc = vvs->buf_alloc;
out:
	vsock_reply(dev, &resp);
}

static void handle_op_rw(struct virtio_vsock_device *dev, struct vdevice_vq *vq,
			 struct vdevice_sglist *sgl,
			 struct vdevice_sglist_cursor *cur,
			 struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	struct virtio_vsock_msg *msg = NULL;
	struct virtio_vsock_hdr resp = { };

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		goto err;
	}

	vvs->peer_buf_alloc = req->buf_alloc;
	vvs->peer_fwd_cnt = req->fwd_cnt;
	if (sock_send_wait_count)
		condvar_broadcast(&sock_send_cv);

	if (req->len > VIRTIO_VSOCK_MAX_PKT_SIZE) {
		DMSG("Payload too large");
		goto err;
	}
	if (vvs->rx_cnt - vvs->fwd_cnt > vvs->buf_alloc) {
		DMSG("Peer is overdrafting from allocated buffer");
		goto err;
	}

	msg = calloc(1, sizeof(*msg));
	if (!msg)
		return;
	msg->type = VIRTIO_VSOCKET_MSG_TYPE_DATA;
	if (req->len) {
		msg->data.buf = malloc(req->len);
		if (!msg->data.buf) {
			free(msg);
			return;
		}
		if (vdevice_sglist_copy_from(vq, sgl, cur, msg->data.buf,
					     req->len)) {
			DMSG("Short RW payload");
			free(msg->data.buf);
			free(msg);
			goto err;
		}
	}
	msg->data.len = req->len;
	msg->data.flags = req->flags & (VIRTIO_VSOCK_SEQ_EOM |
					VIRTIO_VSOCK_SEQ_EOR);

	vvs->fwd_cnt += req->len;
	add_msg(&vvs->msgs, msg);
	return;

err:
	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_RST);
	vsock_reply(dev, &resp);
}

static void handle_op_shutdown(struct virtio_vsock_device *dev,
			       struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	struct virtio_vsock_hdr resp = { };

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_SHUTDOWN);
	resp.flags = BIT(VIRTIO_VSOCK_SHUTDOWN_F_RECEIVE) |
		     BIT(VIRTIO_VSOCK_SHUTDOWN_F_SEND);

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		resp.op = VIRTIO_VSOCK_OP_RST;
		resp.flags = 0;
	} else {
		vvs->dead = true;
	}

	vsock_reply(dev, &resp);
}

static void handle_op_credit_update(struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (vvs) {
		vvs->peer_buf_alloc = req->buf_alloc;
		vvs->peer_fwd_cnt = req->fwd_cnt;
		if (sock_send_wait_count)
			condvar_broadcast(&sock_send_cv);
	}
}

static void handle_op_credit_request(struct virtio_vsock_device *dev,
				     struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	struct virtio_vsock_hdr resp = { };

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_CREDIT_UPDATE);

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		resp.op = VIRTIO_VSOCK_OP_RST;
	} else {
		vvs->peer_buf_alloc = req->buf_alloc;
		vvs->peer_fwd_cnt = req->fwd_cnt;
		resp.buf_alloc = vvs->buf_alloc;
		resp.fwd_cnt = vvs->fwd_cnt;
	}

	vsock_reply(dev, &resp);
}

static void handle_op_rst(struct virtio_vsock_device *dev,
			  struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;

	vvs = find_socket2(req->dst_cid, req->src_cid, req->dst_port,
			   req->src_port, req->type);
	if (vvs) {
		DMSG("Killing socket");
		vvs->dead = true;
	}
}

/*
 * Pull one packet from a TX descriptor chain and dispatch it. Assumes
 * sock_lock held.
 */
static void vsock_tx_one(struct virtio_vsock_device *dev, struct vdevice_vq *vq,
			 struct vdevice_chain *chain)
{
	struct vdevice_sglist_cursor cur;
	struct virtio_vsock_hdr hdr = { };
	size_t len = vdevice_sglist_len(&chain->out);

	if (len < VIRTIO_VSOCK_HDR_SIZE ||
	    len > VIRTIO_VSOCK_HDR_SIZE + VIRTIO_VSOCK_MAX_PKT_SIZE) {
		DMSG("Bad TX packet length %zu", len);
		return;
	}

	vdevice_sglist_cursor_reset(&cur);
	if (vdevice_sglist_copy_from(vq, &chain->out, &cur, &hdr,
				     VIRTIO_VSOCK_HDR_SIZE)) {
		DMSG("Short TX header");
		return;
	}

	FMSG("op %"PRIu16" src_port %"PRIu32" dst_port %"PRIu32" len %"PRIu32,
	     hdr.op, hdr.src_port, hdr.dst_port, hdr.len);

	switch (hdr.op) {
	case VIRTIO_VSOCK_OP_REQUEST:
		handle_op_request(dev, &hdr);
		break;
	case VIRTIO_VSOCK_OP_RW:
		handle_op_rw(dev, vq, &chain->out, &cur, &hdr);
		break;
	case VIRTIO_VSOCK_OP_RST:
		handle_op_rst(dev, &hdr);
		break;
	case VIRTIO_VSOCK_OP_SHUTDOWN:
		handle_op_shutdown(dev, &hdr);
		break;
	case VIRTIO_VSOCK_OP_CREDIT_UPDATE:
		handle_op_credit_update(&hdr);
		break;
	case VIRTIO_VSOCK_OP_CREDIT_REQUEST:
		handle_op_credit_request(dev, &hdr);
		break;
	default:
		DMSG("Unknown op %"PRIu16, hdr.op);
		break;
	}
}

/* Consume packets the driver posted on the TX queue. Assumes sock_lock held. */
static void vsock_tx_process(struct virtio_vsock_device *dev)
{
	struct vdevice *vdev = dev->vdev;
	struct vdevice_vq *vq = &dev->vqs[VIRTIO_VSOCK_VQ_IDX_TX];
	bool added = false;

	if (!vq->ready)
		return;

	vdevice_disable_notify(vq);

	for (;;) {
		struct vdevice_chain chain;
		int err = vdevice_get_vq_desc(vq, &chain);

		if (err < 0)
			break;		/* Faulty queue: leave it alone */
		if (err == 1) {
			if (vdevice_enable_notify(vq)) {
				vdevice_disable_notify(vq);
				continue;
			}
			break;
		}

		/* On the TX queue we expect only device-readable buffers */
		if (chain.in.sg)
			DMSG("Unexpected writable buffer on TX");
		else
			vsock_tx_one(dev, vq, &chain);

		/* Always return the descriptor to the used ring */
		vdevice_add_used_one(vq, chain.head, 0);
		added = true;
	}

	if (added)
		vdevice_signal(vdev, vq);
}

/* Transport notify callbacks; run in thread context without sock_lock held */

static void vsock_rx_notify(struct vdevice *vdev,
			    struct vdevice_vq *vq __unused)
{
	struct virtio_vsock_device *dev = vdev_to_dev(vdev);

	mutex_lock(&sock_lock);
	vsock_do_send(dev);
	mutex_unlock(&sock_lock);
}

static void vsock_tx_notify(struct vdevice *vdev,
			    struct vdevice_vq *vq __unused)
{
	struct virtio_vsock_device *dev = vdev_to_dev(vdev);

	mutex_lock(&sock_lock);
	vsock_tx_process(dev);
	mutex_unlock(&sock_lock);
}

static void vsock_event_notify(struct vdevice *vdev __unused,
			       struct vdevice_vq *vq __unused)
{
}

/*
 * GP socket API (kernel/virtio_vsock.h), consumed by core/tee/socket.c.
 */

TEE_Result virtio_vsock_listen(uint32_t port, uint16_t type, void *owner,
			       struct virtio_vsock_socket **vvs_ret)
{
	struct virtio_vsock_socket *vvs = NULL;
	TEE_Result res = TEE_SUCCESS;

	vvs = calloc(1, sizeof(*vvs));
	if (!vvs)
		return TEE_ERROR_OUT_OF_MEMORY;
	vvs->src_port = port;
	vvs->src_cid = VIRTIO_VSOCK_CID_HOST;
	vvs->type = type;
	vvs->owner = owner;
	vvs->listen = true;
	vvs->buf_alloc = SMALL_PAGE_SIZE;
	TAILQ_INIT(&vvs->msgs);

	mutex_lock(&sock_lock);
	if (find_listening_vsock(port, type)) {
		res = TEE_ERROR_ACCESS_CONFLICT;
		goto out;
	}
	TAILQ_INSERT_TAIL(&sockets_head, vvs, link);
out:
	mutex_unlock(&sock_lock);
	if (res)
		free(vvs);
	else
		*vvs_ret = vvs;

	return res;
}

void virtio_vsock_close(struct virtio_vsock_socket *vvs)
{
	mutex_lock(&sock_lock);

	TAILQ_REMOVE(&sockets_head, vvs, link);
	while (!TAILQ_EMPTY(&vvs->msgs)) {
		struct virtio_vsock_msg *m = TAILQ_FIRST(&vvs->msgs);

		TAILQ_REMOVE(&vvs->msgs, m, link);
		switch (m->type) {
		case VIRTIO_VSOCKET_MSG_TYPE_DATA:
			free(m->data.buf);
			break;
		case VIRTIO_VSOCKET_MSG_TYPE_REQ:
			/* An accepted-but-unclaimed connection: reset it */
			vsock_reply_op(m->vvs_req, VIRTIO_VSOCK_OP_RST);
			free(m->vvs_req);
			break;
		default:
			break;
		}
		free(m);
	}

	/* Tell the peer a connected socket is gone */
	if (!vvs->listen)
		vsock_reply_op(vvs, VIRTIO_VSOCK_OP_RST);

	mutex_unlock(&sock_lock);
	free(vvs);
}

void virtio_vsock_msgq_lock(struct virtio_vsock_socket *vvs __unused)
{
	mutex_lock(&sock_lock);
}

void virtio_vsock_msgq_unlock(struct virtio_vsock_socket *vvs __unused)
{
	mutex_unlock(&sock_lock);
}

static struct virtio_vsock_msg *
find_msg_type(struct virtio_vsock_msg_head *msgs,
	      enum virtio_vsock_msg_type type)
{
	struct virtio_vsock_msg *m = NULL;

	TAILQ_FOREACH(m, msgs, link)
		if (m->type == type)
			return m;

	return NULL;
}

struct virtio_vsock_msg *
virtio_vsock_msgq_peek(struct virtio_vsock_socket *vvs,
		       enum virtio_vsock_msg_type type, uint32_t timeout)
{
	struct virtio_vsock_msg *m = find_msg_type(&vvs->msgs, type);
	uint64_t to = 0;
	int to_us = 0;

	if (timeout)
		to = timeout_init_us((uint64_t)timeout * 1000);

	while (!m && timeout) {
		to_us = timeout_elapsed_us(to);
		if (to_us >= 0)
			break;

		sock_msgs_wait_count++;
		condvar_wait_timeout(&sock_msgs_cv, &sock_lock, -to_us / 1000);
		assert(sock_msgs_wait_count);
		sock_msgs_wait_count--;
		m = find_msg_type(&vvs->msgs, type);
	}

	return m;
}

void virtio_vsock_msgq_dequeue(struct virtio_vsock_socket *vvs,
			       struct virtio_vsock_msg *m)
{
	assert(m);
	if (m->type == VIRTIO_VSOCKET_MSG_TYPE_DATA)
		vvs->rx_cnt += m->data.len;
	TAILQ_REMOVE(&vvs->msgs, m, link);
	free(m);
}

/* Wait until the peer has credit to receive @sz bytes. Assumes sock_lock held. */
static TEE_Result vsock_check_send_buffer(struct virtio_vsock_socket *vvs,
					  size_t *sz, uint32_t timeout)
{
	uint32_t buf_cnt = 0;
	uint64_t to = 0;
	int to_us = 0;

	if (timeout)
		to = timeout_init_us((uint64_t)timeout * 1000);

	for (;;) {
		if (vvs->local_tx_cnt > vvs->peer_fwd_cnt)
			buf_cnt = vvs->local_tx_cnt - vvs->peer_fwd_cnt;
		else
			buf_cnt = 0;

		if (vvs->peer_buf_alloc > buf_cnt) {
			*sz = MIN(*sz, (size_t)vvs->peer_buf_alloc);
			return TEE_SUCCESS;
		}

		if (timeout)
			to_us = timeout_elapsed_us(to);
		if (to_us >= 0)
			return TEE_ERROR_TIMEOUT;

		sock_send_wait_count++;
		condvar_wait_timeout(&sock_send_cv, &sock_lock, -to_us / 1000);
		assert(sock_send_wait_count);
		sock_send_wait_count--;
	}
}

TEE_Result virtio_vsock_send(struct virtio_vsock_socket *vvs, const void *buf,
			     size_t *blen, uint32_t flags, uint32_t timeout)
{
	struct virtio_vsock_pkt *pkt = NULL;
	struct virtio_vsock_hdr hdr = {
		.src_cid = vvs->dst_cid,
		.dst_cid = vvs->src_cid,
		.src_port = vvs->dst_port,
		.dst_port = vvs->src_port,
		.type = vvs->type,
		.op = VIRTIO_VSOCK_OP_RW,
		.buf_alloc = vvs->buf_alloc,
		.fwd_cnt = vvs->fwd_cnt,
	};
	TEE_Result res = TEE_SUCCESS;
	size_t sz = 0;

	if (vvs->type != VIRTIO_VSOCK_TYPE_SEQPACKET && flags)
		return TEE_ERROR_BAD_PARAMETERS;

	mutex_lock(&sock_lock);

	if (vvs->dead) {
		res = TEE_ERROR_COMMUNICATION;
		goto out;
	}

	sz = *blen;
	res = vsock_check_send_buffer(vvs, &sz, timeout);
	if (res)
		goto out;

	/* Only carry record flags when the whole buffer is sent at once */
	if (flags && sz == *blen)
		hdr.flags = flags;

	pkt = vsock_pkt_alloc(&hdr, buf, sz);
	if (!pkt) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto out;
	}

	vvs->local_tx_cnt += sz;
	*blen = sz;
	vsock_queue_and_send(vvs->dev, pkt);
out:
	mutex_unlock(&sock_lock);

	return res;
}

/*
 * vdevice ops and bus-driver registration.
 */

static int vsock_start(struct vdevice *vdev)
{
	int i = 0;

	for (i = 0; i < vdev->num_queues; i++)
		vdevice_set_ring_ready(&vdev->vqs[i], true);

	return 0;
}

static void vsock_stop(struct vdevice *vdev)
{
	int i = 0;

	for (i = 0; i < vdev->num_queues; i++)
		vdevice_set_ring_ready(&vdev->vqs[i], false);
}

static void vsock_reset_vq(struct vdevice *vdev __unused,
			   struct vdevice_vq *vq __unused)
{
}

static uint32_t vsock_gen_count(struct vdevice *vdev __unused)
{
	return VIRTIO_VSOCK_CONFIG_GEN;
}

static int vsock_get(struct vdevice *vdev, size_t offset, void *buf, size_t len)
{
	struct virtio_vsock_device *dev = vdev_to_dev(vdev);

	if (offset + len > sizeof(dev->cid) || offset + len < offset)
		return -1;

	memcpy(buf, (uint8_t *)&dev->cid + offset, len);

	return 0;
}

static int vsock_set(struct vdevice *vdev __unused, size_t offset __unused,
		     const void *buf __unused, size_t len __unused,
		     uint32_t *gen __unused)
{
	/* Config space is read-only */
	return -1;
}

static void vsock_get_features(struct vdevice *vdev __unused, uint64_t *features)
{
	*features |= BIT64(VIRTIO_VSOCK_F_SEQPACKET) |
		     BIT64(VDEVICE_F_ACCESS_PLATFORM);
}

static int vsock_finalize_features(struct vdevice *vdev __unused,
				   uint64_t features __unused)
{
	return 0;
}

static const struct vdevice_ops vsock_ops = {
	.start = vsock_start,
	.stop = vsock_stop,
	.reset_vq = vsock_reset_vq,
	.gen_count = vsock_gen_count,
	.get = vsock_get,
	.set = vsock_set,
	.get_features = vsock_get_features,
	.finalize_features = vsock_finalize_features,
};

static int vsock_bus_init(struct vdevice *vdev)
{
	struct virtio_vsock_device *dev = calloc(1, sizeof(*dev));

	if (!dev)
		return -1;

	dev->vdev = vdev;
	/* CID of the non-secure endpoint; 0 matches the reference driver */
	dev->cid = 0;
	TAILQ_INIT(&dev->send_queue);

	vdevice_init(vdev, dev->vqs, VIRTIO_VSOCK_VQ_IDX_COUNT, &vsock_ops);
	vdev->dev_id = VIRTIO_VSOCK_DEVICE_ID;
	vdev->vendor_id = 0;
	vdev->priv = dev;

	/* Notify callbacks are cleared by vdevice_init(), set them after */
	dev->vqs[VIRTIO_VSOCK_VQ_IDX_RX].notify = vsock_rx_notify;
	dev->vqs[VIRTIO_VSOCK_VQ_IDX_TX].notify = vsock_tx_notify;
	dev->vqs[VIRTIO_VSOCK_VQ_IDX_EVENT].notify = vsock_event_notify;

	return 0;
}

static void vsock_bus_deinit(struct vdevice *vdev)
{
	struct virtio_vsock_device *dev = vdev_to_dev(vdev);

	/* TODO: tear down sockets and drain the send queue */
	free(dev);
}

static struct virtio_bus_driver vsock_driver = {
	.init = vsock_bus_init,
	.deinit = vsock_bus_deinit,
};

static TEE_Result vsock_init(void)
{
	return virtio_msg_ffa_register_driver(&vsock_driver);
}

nex_service_init_late(vsock_init);
