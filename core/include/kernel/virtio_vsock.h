/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2025, Linaro Limited
 */
#ifndef __KERNEL_VIRTIO_VSOCK_H
#define __KERNEL_VIRTIO_VSOCK_H

#include <sys/queue.h>
#include <tee_api_types.h>
#include <types_ext.h>
#include <util.h>

/* Socket types, see the virtio spec "Socket Device" */
#define VIRTIO_VSOCK_TYPE_STREAM	1
#define VIRTIO_VSOCK_TYPE_SEQPACKET	2

/* SEQPACKET record markers carried in the packet header flags */
#define VIRTIO_VSOCK_SEQ_EOM		BIT(0)
#define VIRTIO_VSOCK_SEQ_EOR		BIT(1)

struct virtio_vsock_device;

TAILQ_HEAD(virtio_vsock_msg_head, virtio_vsock_msg);

/*
 * struct virtio_vsock_socket - one vsock connection or listening endpoint.
 *
 * The <src,dst> cid/port tuple and @type identify a connection. A listening
 * socket has @listen set and only @src_cid/@src_port/@type meaningful. Flow
 * control mirrors the virtio spec credit scheme (@buf_alloc/@fwd_cnt locally,
 * @peer_* as last reported by the peer). @owner is an opaque back-pointer for
 * the GP socket layer; @dev is the device the connection lives on.
 */
struct virtio_vsock_socket {
	uint64_t src_cid;
	uint64_t dst_cid;
	uint32_t src_port;
	uint32_t dst_port;
	uint16_t type;			/* VIRTIO_VSOCK_TYPE_* */
	bool listen;
	bool dead;
	uint32_t buf_alloc;
	uint32_t fwd_cnt;
	uint32_t rx_cnt;
	uint32_t peer_buf_alloc;
	uint32_t peer_fwd_cnt;
	uint32_t local_tx_cnt;
	void *owner;
	struct virtio_vsock_device *dev;
	TAILQ_ENTRY(virtio_vsock_socket) link;
	struct virtio_vsock_msg_head msgs;
};

enum virtio_vsock_msg_type {
	VIRTIO_VSOCKET_MSG_TYPE_DATA,
	VIRTIO_VSOCKET_MSG_TYPE_REQ,
};

/*
 * struct virtio_vsock_msg - one queued item on a socket.
 *
 * DATA carries received payload (with its record flags); REQ notifies a
 * listening socket that a new connection request arrived (@vvs_req).
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
		struct virtio_vsock_socket *vvs_req;
	};
	TAILQ_ENTRY(virtio_vsock_msg) link;
};

/*
 * virtio_vsock_listen() - create a listening socket on @port for @type.
 * @owner is stored for the caller and inherited by accepted connections.
 */
TEE_Result virtio_vsock_listen(uint32_t port, uint16_t type, void *owner,
			       struct virtio_vsock_socket **vvs);

/* Close a socket (listening or connected) and free it */
void virtio_vsock_close(struct virtio_vsock_socket *vvs);

/* Convenience destructor for the GP socket layer */
static inline void virtio_vsock_destructor(void *ptr)
{
	virtio_vsock_close(ptr);
}

void virtio_vsock_msgq_lock(struct virtio_vsock_socket *vvs);
void virtio_vsock_msgq_unlock(struct virtio_vsock_socket *vvs);

/* Return the head message of @type, don't remove; assumes the lock held */
struct virtio_vsock_msg *
virtio_vsock_msgq_peek(struct virtio_vsock_socket *vvs,
		       enum virtio_vsock_msg_type type, uint32_t timeout);

/* Remove and free @msg from the socket queue; assumes the lock held */
void virtio_vsock_msgq_dequeue(struct virtio_vsock_socket *vvs,
			       struct virtio_vsock_msg *msg);

/* Send @buf (up to *@blen bytes, updated on return) on a connected socket */
TEE_Result virtio_vsock_send(struct virtio_vsock_socket *vvs, const void *buf,
			     size_t *blen, uint32_t flags, uint32_t timeout);

#endif /*__KERNEL_VIRTIO_VSOCK_H*/
