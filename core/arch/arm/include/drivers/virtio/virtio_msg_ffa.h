/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Innovation Center, Inc.
 *
 * FF-A binding for the virtio-msg transport (DEN0153 "Virtio Message Bus over
 * FF-A"). Provides the struct virtio_transport_ops used by the backend core
 * and codec, manages shared memory areas, and receives FFA_MSG_SEND_DIRECT_REQ2
 * messages dispatched by virtio UUID on OP-TEE's core endpoint.
 */
#ifndef __DRIVERS_VIRTIO_VIRTIO_MSG_FFA_H
#define __DRIVERS_VIRTIO_VIRTIO_MSG_FFA_H

#include <kernel/thread.h>

/*
 * Handle a virtio-msg message carried in an FFA_MSG_SEND_DIRECT_REQ2. The
 * message header and payload are in args->a4..a17; the response is written in
 * place. The SPMC dispatcher builds the RESP2 header around it.
 */
void virtio_msg_ffa_recv(struct thread_smc_1_2_regs *args);

#endif /*__DRIVERS_VIRTIO_VIRTIO_MSG_FFA_H*/
