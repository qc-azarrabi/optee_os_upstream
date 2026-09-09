/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * virtio-msg message codec, transport- and role-neutral. Encodes/decodes the
 * virtio-msg transport and bus messages defined by DEN0153. The transport
 * binding (e.g. virtio_msg_ffa.c) hands a decoded message and a
 * struct virtio_transport_ops to virtio_msg_recv(); the codec fills the
 * response in place.
 */
#ifndef __DRIVERS_VIRTIO_VIRTIO_MSG_H
#define __DRIVERS_VIRTIO_VIRTIO_MSG_H

#include <drivers/virtio/virtio_transport.h>
#include <stdint.h>

/* Message types (virtio_msg_hdr::type). */
#define MSG_TYPE_TRANSPORT_REQUEST	0x0
#define MSG_TYPE_TRANSPORT_RESPONSE	0x1
#define MSG_TYPE_BUS_REQUEST		0x2
#define MSG_TYPE_BUS_RESPONSE		0x3

/* Bus message opcodes. */
#define BUS_MSG_GET_DEVICES		0x02
#define BUS_MSG_PING			0x03
#define BUS_MSG_EVENT_DEVICE		0x40

/* Transport message opcodes. */
#define VIRTIO_MSG_DEVICE_INFO		0x02
#define VIRTIO_MSG_GET_DEV_FEATURES	0x03
#define VIRTIO_MSG_SET_DRV_FEATURES	0x04
#define VIRTIO_MSG_GET_CONFIG		0x05
#define VIRTIO_MSG_SET_CONFIG		0x06
#define VIRTIO_MSG_GET_DEVICE_STATUS	0x07
#define VIRTIO_MSG_SET_DEVICE_STATUS	0x08
#define VIRTIO_MSG_GET_VQUEUE		0x09
#define VIRTIO_MSG_SET_VQUEUE		0x0a
#define VIRTIO_MSG_RESET_VQUEUE		0x0b
#define VIRTIO_MSG_GET_SHM		0x0c
#define VIRTIO_MSG_EVENT_CONFIG		0x40
#define VIRTIO_MSG_EVENT_AVAIL		0x41
#define VIRTIO_MSG_EVENT_USED		0x42

struct virtio_msg_hdr {
	uint8_t type;
	uint8_t msg_id;
	uint16_t dev_num;
	uint16_t msg_uid;
	uint16_t msg_size;
};

static inline void virtio_msg_set_null_msg(struct virtio_msg_hdr *hdr)
{
	*hdr = (struct virtio_msg_hdr){
		.type = MSG_TYPE_BUS_RESPONSE,
		.msg_size = sizeof(struct virtio_msg_hdr),
	};
}

/*
 * virtio_msg_recv() - decode a received virtio-msg message and fill the
 * response in place.
 *
 * @hdr:		Message header followed by its payload.
 * @ops:		Transport seam for bus-address translation.
 * @max_msg_size:	Maximum total message size the transport can carry.
 */
void virtio_msg_recv(struct virtio_msg_hdr *hdr,
		     const struct virtio_transport_ops *ops,
		     size_t max_msg_size);

/* Internal dispatch helpers (also used by the unit tests). */
void virtio_msg_handle_bus_req(struct virtio_msg_hdr *hdr,
			       const struct virtio_transport_ops *ops,
			       size_t max_msg_size);
void virtio_msg_handle_transport_req(struct virtio_msg_hdr *hdr,
				     const struct virtio_transport_ops *ops);

#endif /*__DRIVERS_VIRTIO_VIRTIO_MSG_H*/
