// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * virtio vsock backend (device side). OP-TEE is the vsock device; the
 * normal-world driver is the frontend. Implements the device-class ops
 * (struct virtio_dev_ops) registered with the transport-agnostic backend
 * core, the RX/TX bottom-half callbacks that drive the vsock protocol, and an
 * in-kernel socket API used by the GP socket layer.
 *
 * Naming note: the RX and TX virtqueues are named from the *driver's*
 * perspective, matching the virtio-vsock spec. The device therefore *reads*
 * driver-posted buffers from the TX queue and *writes* buffers the driver
 * receives into the RX queue.
 */

#include <bitstring.h>
#include <confine_array_index.h>
#include <drivers/virtio/virtio_dev.h>
#include <drivers/virtio/virtio_vsock.h>
#include <initcall.h>
#include <kernel/delay.h>
#include <kernel/mutex.h>
#include <mm/core_mmu.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#define VIRTIO_VSOCK_F_STREAM			0
#define VIRTIO_VSOCK_F_SEQPACKET		1
#define VIRTIO_VSOCK_F_NO_IMPLIED_STREAM	2

#define VIRTIO_VSOCK_DEVICE_ID		19

#define VIRTIO_VSOCK_OP_INVALID		0
/* Connection operations. */
#define VIRTIO_VSOCK_OP_REQUEST		1
#define VIRTIO_VSOCK_OP_RESPONSE	2
#define VIRTIO_VSOCK_OP_RST		3
#define VIRTIO_VSOCK_OP_SHUTDOWN	4
/* Payload transfer. */
#define VIRTIO_VSOCK_OP_RW		5
/* Tell the peer our credit info. */
#define VIRTIO_VSOCK_OP_CREDIT_UPDATE	6
/* Request the peer to send its credit info. */
#define VIRTIO_VSOCK_OP_CREDIT_REQUEST	7

#define VIRTIO_VSOCK_SHUTDOWN_F_RECEIVE	0
#define VIRTIO_VSOCK_SHUTDOWN_F_SEND	1

/* The host (device) side of a vsock connection uses this well-known CID. */
#define VIRTIO_VSOCK_CID_HOST		2

enum virtio_vsock_vq_idx {
	VIRTIO_VSOCK_VQ_IDX_RX = 0,
	VIRTIO_VSOCK_VQ_IDX_TX,
	VIRTIO_VSOCK_VQ_IDX_EVENT,
	VIRTIO_VSOCK_VQ_IDX_COUNT,
};

TAILQ_HEAD(virtio_vsock_socket_head, virtio_vsock_socket);

struct virtio_vsock_device {
	struct virtio_dev vdev;
	uint64_t cid;
};

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

static const struct virtio_dev_ops vsock_ops;
static const uint32_t vsock_features[] = {
	VIRTIO_F_VERSION_1,
	VIRTIO_VSOCK_F_STREAM,
	VIRTIO_VSOCK_F_SEQPACKET,
	VIRTIO_VSOCK_F_NO_IMPLIED_STREAM,
	VIRTIO_F_ACCESS_PLATFORM,
};

static struct mutex sock_lock = MUTEX_INITIALIZER;
static size_t sock_vvs_msgs_wait_count;
static struct condvar sock_vvs_msgs_cv = CONDVAR_INITIALIZER;
static size_t sock_vvs_send_wait_count;
static struct condvar sock_vvs_send_cv = CONDVAR_INITIALIZER;
static struct virtio_vsock_socket_head sockets_head =
	TAILQ_HEAD_INITIALIZER(sockets_head);
static struct virtio_vsock_socket_head sockets_backlog_head =
	TAILQ_HEAD_INITIALIZER(sockets_backlog_head);

static struct virtio_vsock_socket *find_listening_vsock(uint32_t port,
							uint16_t type)
{
	struct virtio_vsock_socket *v = NULL;

	TAILQ_FOREACH(v, &sockets_head, link)
		if (v->listen && v->type == type && v->src_port == port)
			return v;

	return NULL;
}

static bool cmp_socket(const struct virtio_vsock_socket *vvs1,
		       const struct virtio_vsock_socket *vvs2)
{
	return vvs1->src_cid == vvs2->src_cid &&
	       vvs1->dst_cid == vvs2->dst_cid &&
	       vvs1->src_port == vvs2->src_port &&
	       vvs1->dst_port == vvs2->dst_port &&
	       vvs1->type == vvs2->type;
}

static struct virtio_vsock_socket *
find_socket(const struct virtio_vsock_socket *vvs)
{
	struct virtio_vsock_socket *v = NULL;

	TAILQ_FOREACH(v, &sockets_head, link)
		if (cmp_socket(v, vvs))
			return v;

	return NULL;
}

static struct virtio_vsock_socket *find_socket2(uint64_t src_cid,
						uint64_t dst_cid,
						uint32_t src_port,
						uint32_t dst_port,
						uint16_t type)
{
	struct virtio_vsock_socket vvs = {
		.src_cid = src_cid,
		.dst_cid = dst_cid,
		.src_port = src_port,
		.dst_port = dst_port,
		.type = type,
	};

	return find_socket(&vvs);
}

static void add_msg(struct virtio_vsock_msg_head *msgs,
		    struct virtio_vsock_msg *m)
{
	TAILQ_INSERT_TAIL(msgs, m, link);
	if (sock_vvs_msgs_wait_count)
		condvar_broadcast(&sock_vvs_msgs_cv);
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

static struct virtio_vsock_device *vdev_to_vsdev(struct virtio_dev *vdev)
{
	assert(vdev->ops == &vsock_ops);

	return container_of(vdev, struct virtio_vsock_device, vdev);
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

static void handle_op_request(struct virtio_dev_vq_ctx *ctx,
			      struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_device *dev = vdev_to_vsdev(ctx->vq->vdev);
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_socket *lvvs = NULL;
	struct virtio_vsock_socket *vvs = NULL;
	struct virtio_vsock_hdr resp = { };
	struct virtio_dev_vq *rxq = dev->vdev.vqs + VIRTIO_VSOCK_VQ_IDX_RX;

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_RESPONSE);
	if (req->src_cid != dev->cid || !req->src_port) {
		DMSG("Bad req");
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
		DMSG("Calloc failed");
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
	res = virtio_dev_vq_get_writable(rxq, &wctx, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		return;
	}

	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		return;
	}

	virtio_dev_vq_complete_len(&wctx, sizeof(resp));
}

static void handle_op_rw(struct virtio_dev_vq_ctx *ctx,
			 struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_msg *msg = NULL;
	struct virtio_vsock_socket *vvs = NULL;
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = { };
	struct virtio_dev_vq *rxq = ctx->vq->vdev->vqs + VIRTIO_VSOCK_VQ_IDX_RX;

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		goto err;
	}

	vvs->peer_buf_alloc = req->buf_alloc;
	vvs->peer_fwd_cnt = req->fwd_cnt;

	/*
	 * rx_cnt counts all bytes received from the peer; fwd_cnt counts the
	 * bytes already delivered to the local owner (advanced in
	 * virtio_vsock_msgq_dequeue()). The peer must not have more than
	 * buf_alloc bytes outstanding (received but not yet forwarded).
	 */
	if (req->len > vvs->buf_alloc ||
	    vvs->rx_cnt - vvs->fwd_cnt > vvs->buf_alloc - req->len) {
		DMSG("Peer is overdrafting from allocated buffer");
		goto err;
	}

	msg = calloc(1, sizeof(*msg));
	if (!msg) {
		DMSG("calloc");
		return;
	}
	msg->type = VIRTIO_VSOCKET_MSG_TYPE_DATA;
	msg->data.buf = calloc(1, req->len);
	if (!msg->data.buf) {
		free(msg);
		DMSG("calloc");
		return;
	}

	msg->data.len = req->len;
	msg->data.flags = req->flags & (VIRTIO_VSOCK_SEQ_EOM |
					VIRTIO_VSOCK_SEQ_EOR);

	res = virtio_dev_vq_pull(msg->data.buf, ctx, sizeof(*req), req->len);
	if (res) {
		DMSG("virtio_dev_vq_pull: res %#"PRIx32, res);
		free(msg->data.buf);
		free(msg);
		goto err;
	}
	if (TRACE_LEVEL >= TRACE_FLOW)
		DHEXDUMP(msg->data.buf, msg->data.len);

	vvs->rx_cnt += req->len;
	virtio_vsock_msgq_lock(vvs);
	add_msg(&vvs->msgs, msg);
	virtio_vsock_msgq_unlock(vvs);
	return;

err:
	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_RST);
	res = virtio_dev_vq_get_writable(rxq, &wctx, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		return;
	}

	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		return;
	}

	virtio_dev_vq_complete_len(&wctx, sizeof(resp));
}

static void handle_op_shutdown(struct virtio_dev_vq_ctx *ctx,
			       struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = { };
	struct virtio_dev_vq *rxq = ctx->vq->vdev->vqs + VIRTIO_VSOCK_VQ_IDX_RX;

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_SHUTDOWN);
	resp.flags = BIT(VIRTIO_VSOCK_SHUTDOWN_F_RECEIVE) |
		     BIT(VIRTIO_VSOCK_SHUTDOWN_F_SEND);
	res = virtio_dev_vq_get_writable(rxq, &wctx, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		return;
	}

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		resp.op = VIRTIO_VSOCK_OP_RST;
		resp.flags = 0;
		goto out;
	}
	vvs->dead = true;

out:
	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		return;
	}

	virtio_dev_vq_complete_len(&wctx, sizeof(resp));
}

static void handle_op_credit_update(struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (vvs) {
		vvs->peer_buf_alloc = req->buf_alloc;
		vvs->peer_fwd_cnt = req->fwd_cnt;
	}
}

static void handle_op_credit_request(struct virtio_dev_vq_ctx *ctx,
				     struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = { };
	struct virtio_dev_vq *rxq = ctx->vq->vdev->vqs + VIRTIO_VSOCK_VQ_IDX_RX;

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_CREDIT_UPDATE);
	res = virtio_dev_vq_get_writable(rxq, &wctx, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		return;
	}

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (!vvs) {
		DMSG("Connection not found");
		resp.op = VIRTIO_VSOCK_OP_RST;
		resp.flags = 0;
		goto out;
	}

	vvs->peer_buf_alloc = req->buf_alloc;
	vvs->peer_fwd_cnt = req->fwd_cnt;
	resp.buf_alloc = vvs->buf_alloc;
	resp.fwd_cnt = vvs->fwd_cnt;

out:
	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		return;
	}

	virtio_dev_vq_complete_len(&wctx, sizeof(resp));
}

static void handle_op_rst(struct virtio_dev_vq_ctx *ctx,
			  struct virtio_vsock_hdr *req)
{
	struct virtio_vsock_socket *vvs = NULL;
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = { };
	struct virtio_dev_vq *rxq = ctx->vq->vdev->vqs + VIRTIO_VSOCK_VQ_IDX_RX;

	init_resp_hdr(&resp, req, VIRTIO_VSOCK_OP_RST);
	res = virtio_dev_vq_get_writable(rxq, &wctx, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		return;
	}

	vvs = find_socket2(req->src_cid, req->dst_cid, req->src_port,
			   req->dst_port, req->type);
	if (vvs) {
		DMSG("Killing socket");
		vvs->dead = true;
	}

	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		return;
	}

	virtio_dev_vq_complete_len(&wctx, sizeof(resp));
}

static void handle_rx_payload(struct virtio_dev_vq_ctx *ctx)
{
	struct virtio_vsock_hdr hdr = { };
	TEE_Result res = TEE_SUCCESS;

	res = virtio_dev_vq_pull(&hdr, ctx, 0, sizeof(hdr));
	if (res) {
		DMSG("virtio_dev_vq_pull: res %#"PRIx32, res);
		return;
	}
	FMSG("src_cid %#"PRIx64" dst_cid %#"PRIx64,
	     hdr.src_cid, hdr.dst_cid);
	FMSG("src_port %#"PRIx32" dst_port %#"PRIx32,
	     hdr.src_port, hdr.dst_port);
	FMSG("len %"PRIu32" type %"PRIu16" op %"PRIu16,
	     hdr.len, hdr.type, hdr.op);
	FMSG("flags %#"PRIx32" buf_alloc %"PRIu32" fwd_cnt %"PRIu32,
	     hdr.flags, hdr.buf_alloc, hdr.fwd_cnt);

	switch (hdr.op) {
	case VIRTIO_VSOCK_OP_REQUEST:
		DMSG("VIRTIO_VSOCK_OP_REQUEST");
		handle_op_request(ctx, &hdr);
		break;
	case VIRTIO_VSOCK_OP_RW:
		DMSG("VIRTIO_VSOCK_OP_RW");
		handle_op_rw(ctx, &hdr);
		break;
	case VIRTIO_VSOCK_OP_RST:
		DMSG("VIRTIO_VSOCK_OP_RST");
		handle_op_rst(ctx, &hdr);
		break;
	case VIRTIO_VSOCK_OP_SHUTDOWN:
		DMSG("VIRTIO_VSOCK_OP_SHUTDOWN");
		handle_op_shutdown(ctx, &hdr);
		break;
	case VIRTIO_VSOCK_OP_CREDIT_UPDATE:
		DMSG("VIRTIO_VSOCK_OP_CREDIT_UPDATE");
		handle_op_credit_update(&hdr);
		break;
	case VIRTIO_VSOCK_OP_CREDIT_REQUEST:
		DMSG("VIRTIO_VSOCK_OP_CREDIT_REQUEST");
		handle_op_credit_request(ctx, &hdr);
		break;
	default:
		DMSG("Unknown op %"PRIu16, hdr.op);
	}
}

/*
 * Driver-posted buffers arrive on the TX queue (driver's perspective); the
 * device reads and processes them here.
 */
static void vsock_vq_tx_callback(struct virtio_dev_vq *vq)
{
	struct virtio_dev_vq_ctx ctx = { };

	DMSG("idx %u", vq->vq_id);

	while (!virtio_dev_vq_get_avail(vq, &ctx)) {
		handle_rx_payload(&ctx);
		virtio_dev_vq_complete(&ctx);
	}
}

static bool reply_op(struct virtio_dev_vq *vq, struct virtio_vsock_socket *vvs,
		     uint16_t op)
{
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = {
		/*
		 * Device->driver messages put the host (device) side as the
		 * source and the normal-world peer as the destination, matching
		 * the orientation used by init_resp_hdr() and
		 * virtio_vsock_send().
		 */
		.src_cid = vvs->dst_cid,
		.dst_cid = vvs->src_cid,
		.src_port = vvs->dst_port,
		.dst_port = vvs->src_port,
		.type = vvs->type,
		.op = op,
		.buf_alloc = vvs->buf_alloc,
		.fwd_cnt = vvs->fwd_cnt,
	};

	if (virtio_dev_vq_get_writable(vq, &wctx, sizeof(resp)))
		return false;
	if (virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp)))
		return false;
	virtio_dev_vq_complete_len(&wctx, sizeof(resp));

	return true;
}

/*
 * Advertise our current credit (buf_alloc/fwd_cnt) to the peer. Best-effort:
 * if no RX buffer is available the update is simply skipped, and the peer will
 * learn the new credit from the next message we send. The socket lock must be
 * held by the caller.
 */
static void send_credit_update(struct virtio_vsock_socket *vvs)
{
	reply_op(vvs->dev->vdev.vqs + VIRTIO_VSOCK_VQ_IDX_RX, vvs,
		 VIRTIO_VSOCK_OP_CREDIT_UPDATE);
}

static bool process_backlog(struct virtio_dev_vq *vq)
{
	struct virtio_vsock_socket *vvs = NULL;

	assert(vq == vq->vdev->vqs + VIRTIO_VSOCK_VQ_IDX_RX);

	while (!TAILQ_EMPTY(&sockets_backlog_head)) {
		vvs = TAILQ_FIRST(&sockets_backlog_head);
		assert((vvs->pending_op == VIRTIO_VSOCK_OP_RST &&
			vvs->dead && !vvs->owner) ||
		       vvs->pending_op == VIRTIO_VSOCK_OP_RESPONSE ||
		       vvs->pending_op == VIRTIO_VSOCK_OP_SHUTDOWN);
		if (!reply_op(vq, vvs, vvs->pending_op))
			break;

		TAILQ_REMOVE(&sockets_backlog_head, vvs, link_backlog);
		/*
		 * If the socket is fully dead, freeing it is all that remains
		 * to do.
		 */
		if (vvs->pending_op == VIRTIO_VSOCK_OP_RST)
			free(vvs);
		else
			vvs->pending_op = VIRTIO_VSOCK_OP_INVALID;
	}

	return TAILQ_EMPTY(&sockets_backlog_head);
}

/*
 * Buffers the driver posted to receive into arrive on the RX queue; the device
 * uses them to drain the deferred-reply backlog.
 */
static void vsock_vq_rx_callback(struct virtio_dev_vq *vq)
{
	DMSG("idx %u", vq->vq_id);
	mutex_lock(&sock_lock);
	process_backlog(vq);
	if (sock_vvs_send_wait_count)
		condvar_broadcast(&sock_vvs_send_cv);
	mutex_unlock(&sock_lock);
}

static void vsock_vq_event_callback(struct virtio_dev_vq *vq __maybe_unused)
{
	DMSG("idx %u", vq->vq_id);
}

static const struct virtio_dev_vq_info vsock_vqs_info[] = {
	[VIRTIO_VSOCK_VQ_IDX_RX] = {
		.name = "rx",
		.callback = vsock_vq_rx_callback,
	},
	[VIRTIO_VSOCK_VQ_IDX_TX] = {
		.name = "tx",
		.callback = vsock_vq_tx_callback,
	},
	[VIRTIO_VSOCK_VQ_IDX_EVENT] = {
		.name = "event",
		.callback = vsock_vq_event_callback,
	},
};

static void vsock_get_config(struct virtio_dev *vdev, void *data,
			     size_t count, size_t offset)
{
	struct virtio_vsock_device *dev = vdev_to_vsdev(vdev);
	uint8_t *vsock_config = (uint8_t *)&dev->cid;

	assert(sizeof(dev->cid) == vsock_ops.config_size);
	assert(count + offset <= sizeof(dev->cid));

	memcpy(data, vsock_config + offset, count);
}

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
		struct virtio_vsock_msg *m = NULL;
		struct virtio_dev_vq *vq = NULL;

		m = TAILQ_FIRST(&vvs->msgs);
		TAILQ_REMOVE(&vvs->msgs, m, link);
		switch (m->type) {
		case VIRTIO_VSOCKET_MSG_TYPE_DATA:
			free(m->data.buf);
			break;
		case VIRTIO_VSOCKET_MSG_TYPE_REQ:
			vq = m->vvs_req->dev->vdev.vqs +
			     VIRTIO_VSOCK_VQ_IDX_RX;
			if (!reply_op(vq, m->vvs_req, VIRTIO_VSOCK_OP_RST)) {
				m->vvs_req->dead = true;
				m->vvs_req->pending_op = VIRTIO_VSOCK_OP_RST;
				TAILQ_INSERT_TAIL(&sockets_backlog_head,
						  m->vvs_req, link_backlog);
				break;
			}
			free(m->vvs_req);
			break;
		case VIRTIO_VSOCKET_MSG_TYPE_SHM:
		default:
			break;
		}
		free(m);
	}

	if (!vvs->listen &&
	    !reply_op(vvs->dev->vdev.vqs + VIRTIO_VSOCK_VQ_IDX_RX, vvs,
		      VIRTIO_VSOCK_OP_RST)) {
		vvs->owner = NULL;
		vvs->dead = true;
		vvs->pending_op = VIRTIO_VSOCK_OP_RST;
		TAILQ_INSERT_TAIL(&sockets_backlog_head, vvs, link_backlog);
	} else {
		free(vvs);
	}

	mutex_unlock(&sock_lock);
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

		sock_vvs_msgs_wait_count++;
		condvar_wait_timeout(&sock_vvs_msgs_cv, &sock_lock,
				     -to_us / 1000);
		assert(sock_vvs_msgs_wait_count);
		sock_vvs_msgs_wait_count--;
		m = find_msg_type(&vvs->msgs, type);
	}

	return m;
}

void virtio_vsock_msgq_dequeue(struct virtio_vsock_socket *vvs,
			       struct virtio_vsock_msg *m)
{
	assert(m);
	/*
	 * Delivering received data to the local owner advances fwd_cnt, which
	 * is what we advertise back to the peer as consumed (freeing credit).
	 * Proactively send a credit update so a one-way peer->host stream does
	 * not stall: the normal-world frontend only refreshes its view of our
	 * credit from messages we send, and never polls with CREDIT_REQUEST.
	 */
	if (m->type == VIRTIO_VSOCKET_MSG_TYPE_DATA) {
		vvs->fwd_cnt += m->data.len;
		TAILQ_REMOVE(&vvs->msgs, m, link);
		free(m);
		if (!vvs->dead)
			send_credit_update(vvs);
		return;
	}
	TAILQ_REMOVE(&vvs->msgs, m, link);
	free(m);
}

static TEE_Result vsock_check_send_buffer(struct virtio_vsock_socket *vvs,
					  size_t *sz, uint32_t timeout)
{
	uint32_t buf_cnt = 0;
	uint64_t to = 0;
	int to_us = 0;

	if (timeout)
		to = timeout_init_us((uint64_t)timeout * 1000);

	while (true) {
		if (process_backlog(vvs->dev->vdev.vqs +
				    VIRTIO_VSOCK_VQ_IDX_RX)) {
			if (vvs->local_tx_cnt > vvs->peer_fwd_cnt)
				buf_cnt = vvs->local_tx_cnt - vvs->peer_fwd_cnt;
			else
				buf_cnt = 0;

			if (vvs->peer_buf_alloc > buf_cnt) {
				*sz = MIN(*sz, vvs->peer_buf_alloc);
				return TEE_SUCCESS;
			}
		}

		if (timeout)
			to_us = timeout_elapsed_us(to);
		if (to_us >= 0)
			return TEE_ERROR_TIMEOUT;

		sock_vvs_send_wait_count++;
		condvar_wait_timeout(&sock_vvs_send_cv, &sock_lock,
				     -to_us / 1000);
		assert(sock_vvs_send_wait_count);
		sock_vvs_send_wait_count--;
	}
}

TEE_Result virtio_vsock_send(struct virtio_vsock_socket *vvs, const void *buf,
			     size_t *blen, uint32_t flags, uint32_t timeout)
{
	TEE_Result res = TEE_SUCCESS;
	struct virtio_dev_vq_ctx wctx = { };
	struct virtio_vsock_hdr resp = {
		.src_cid = vvs->dst_cid,
		.dst_cid = vvs->src_cid,
		.src_port = vvs->dst_port,
		.dst_port = vvs->src_port,
		.type = vvs->type,
		.op = VIRTIO_VSOCK_OP_RW,
		.buf_alloc = vvs->buf_alloc,
		.fwd_cnt = vvs->fwd_cnt,
	};
	size_t sz = 0;

	if (vvs->type != VIRTIO_VSOCK_TYPE_SEQPACKET && flags)
		return TEE_ERROR_BAD_PARAMETERS;

	mutex_lock(&sock_lock);
	sz = *blen;
	res = vsock_check_send_buffer(vvs, &sz, timeout);
	if (res) {
		DMSG("vsock_check_send_buffer: res %#"PRIx32, res);
		goto out;
	}

	resp.len = sz;
	/* Only set the flags if the entire buffer is transmitted. */
	if (flags && sz == *blen)
		resp.flags = flags;

	res = virtio_dev_vq_get_writable(vvs->dev->vdev.vqs +
					 VIRTIO_VSOCK_VQ_IDX_RX,
					 &wctx, sizeof(resp) + sz);
	if (res) {
		DMSG("virtio_dev_vq_get_writable: res %#"PRIx32, res);
		goto out;
	}

	res = virtio_dev_vq_push(&resp, &wctx, 0, sizeof(resp));
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		goto out;
	}

	res = virtio_dev_vq_push(buf, &wctx, sizeof(resp), sz);
	if (res) {
		DMSG("virtio_dev_vq_push: res %#"PRIx32, res);
		goto out;
	}

	vvs->local_tx_cnt += sz;
	*blen = sz;
	virtio_dev_vq_complete_len(&wctx, sizeof(resp) + sz);
out:
	mutex_unlock(&sock_lock);

	return res;
}

static const struct virtio_dev_ops vsock_ops = {
	.dev_id = VIRTIO_VSOCK_DEVICE_ID,
	.vendor_id = 0,
	.config_size = sizeof(((struct virtio_vsock_device *)0)->cid),
	.vq_count = VIRTIO_VSOCK_VQ_IDX_COUNT,
	.admin_vq_start_idx = 0,	/* VIRTIO_F_ADMIN_VQ not used */
	.admin_vq_count = 0,		/* VIRTIO_F_ADMIN_VQ not used */
	.feature_bit_count = VIRTIO_MAX_FEATURE_BIT_COUNT,
	.max_desc_count = 1024,
	.features = vsock_features,
	.feature_count = ARRAY_SIZE(vsock_features),
	.vqs_info = vsock_vqs_info,
	.get_config = vsock_get_config,
};

static TEE_Result vsock_init(void)
{
	struct virtio_vsock_device *dev = calloc(1, sizeof(*dev));
	TEE_Result res = TEE_SUCCESS;

	if (!dev)
		return TEE_ERROR_OUT_OF_MEMORY;

	dev->cid = 0; /* The non-secure physical endpoint. */
	dev->vdev.ops = &vsock_ops;

	res = virtio_dev_register(&dev->vdev);
	if (res)
		free(dev);

	return res;
}
nex_service_init(vsock_init);
