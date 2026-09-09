/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * virtio vsock backend (device side). OP-TEE acts as the vsock device; the
 * normal-world driver is the frontend. Provides an in-kernel socket API
 * (listen/close/send + a per-socket message queue) that the GP socket API in
 * core/tee/socket.c builds on to expose vsock to Trusted Applications.
 */
#ifndef __DRIVERS_VIRTIO_VIRTIO_VSOCK_H
#define __DRIVERS_VIRTIO_VIRTIO_VSOCK_H

#include <sys/queue.h>
#include <tee_api_types.h>
#include <types_ext.h>

#define VIRTIO_VSOCK_TYPE_STREAM	1
#define VIRTIO_VSOCK_TYPE_SEQPACKET	2

#define VIRTIO_VSOCK_SEQ_EOM	BIT(0)
#define VIRTIO_VSOCK_SEQ_EOR	BIT(1)

struct virtio_vsock_device;

TAILQ_HEAD(virtio_vsock_msg_head, virtio_vsock_msg);

/*
 * struct virtio_vsock_socket - a vsock endpoint (listening or connected).
 *
 * @src_cid/@dst_cid/@src_port/@dst_port:	Connection 4-tuple. For the
 *		device side, the source is the host (OP-TEE), the destination
 *		is the normal-world peer.
 * @type:	VIRTIO_VSOCK_TYPE_STREAM or VIRTIO_VSOCK_TYPE_SEQPACKET.
 * @listen:	True for a listening socket.
 * @dead:	True once the peer or the owner has torn the connection down.
 * @pending_op:	Deferred op to send when a used buffer becomes available.
 * @buf_alloc/@fwd_cnt/@rx_cnt:	Local credit accounting.
 * @peer_buf_alloc/@peer_fwd_cnt/@local_tx_cnt:	Peer credit accounting.
 * @owner:	Opaque owner handle (e.g. the GP socket instance).
 * @dev:	Owning vsock device.
 * @link:	Linkage in the global socket list.
 * @link_backlog:	Linkage in the deferred-reply backlog list.
 * @msgs:	Per-socket inbound message queue.
 */
struct virtio_vsock_socket {
	uint64_t src_cid;
	uint64_t dst_cid;
	uint32_t src_port;
	uint32_t dst_port;
	uint16_t type;
	bool listen;
	bool dead;
	uint16_t pending_op;
	uint32_t buf_alloc;
	uint32_t fwd_cnt;
	uint32_t rx_cnt;
	uint32_t peer_buf_alloc;
	uint32_t peer_fwd_cnt;
	uint32_t local_tx_cnt;
	void *owner;
	struct virtio_vsock_device *dev;
	TAILQ_ENTRY(virtio_vsock_socket) link;
	TAILQ_ENTRY(virtio_vsock_socket) link_backlog;
	struct virtio_vsock_msg_head msgs;
};

enum virtio_vsock_msg_type {
	VIRTIO_VSOCKET_MSG_TYPE_DATA,
	VIRTIO_VSOCKET_MSG_TYPE_REQ,
	VIRTIO_VSOCKET_MSG_TYPE_SHM,
};

/*
 * struct virtio_vsock_msg - a queued inbound event for a socket.
 *
 * DATA carries a received payload buffer; REQ carries an incoming connection
 * request (the new socket awaiting accept); SHM is reserved for a future
 * zero-copy path.
 */
struct virtio_vsock_msg {
	enum virtio_vsock_msg_type type;
	union {
		struct {
			uint32_t flags;
			void *buf;
			size_t len;
			size_t offs;
		} data;
		struct {
			uint64_t bus_addr;
			uint64_t len;
		} shm;
		struct virtio_vsock_socket *vvs_req;
	};
	TAILQ_ENTRY(virtio_vsock_msg) link;
};

/* Open a listening socket on @port. On success *@vvs holds the new socket. */
TEE_Result virtio_vsock_listen(uint32_t port, uint16_t type, void *owner,
			       struct virtio_vsock_socket **vvs);

/* Close and free a socket, sending RST to the peer where required. */
void virtio_vsock_close(struct virtio_vsock_socket *vvs);

/* Convenience destructor for handle-based cleanup. */
static inline void virtio_vsock_destructor(void *ptr)
{
	virtio_vsock_close(ptr);
}

void virtio_vsock_msgq_lock(struct virtio_vsock_socket *vvs);
void virtio_vsock_msgq_unlock(struct virtio_vsock_socket *vvs);

/* Peek the queue head of @type, waiting up to @timeout ms. Lock must be held. */
struct virtio_vsock_msg *
virtio_vsock_msgq_peek(struct virtio_vsock_socket *vvs,
		       enum virtio_vsock_msg_type type, uint32_t timeout);

/* Remove and free a queued message. Lock must be held. */
void virtio_vsock_msgq_dequeue(struct virtio_vsock_socket *vvs,
			       struct virtio_vsock_msg *msg);

/* Send @blen bytes from @buf. Updates *@blen with the count actually sent. */
TEE_Result virtio_vsock_send(struct virtio_vsock_socket *vvs, const void *buf,
			     size_t *blen, uint32_t flags, uint32_t timeout);

#endif /*__DRIVERS_VIRTIO_VIRTIO_VSOCK_H*/
