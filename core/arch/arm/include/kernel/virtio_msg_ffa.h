/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 */

#ifndef __KERNEL_VIRTIO_MSG_FFA_H
#define __KERNEL_VIRTIO_MSG_FFA_H

#include <kernel/thread.h>
#include <kernel/virtio_msg.h>

/*
 * virtio-msg over FF-A carrier (Arm DEN0153).
 *
 * This layer binds the transport-independent virtio-msg core (struct
 * virtio_msg_bus) to the FF-A transport. A non-secure endpoint reaches the
 * OP-TEE core endpoint with FFA_MSG_SEND_DIRECT_REQ2 addressed to the
 * virtio-msg bus UUID (c66028b5-2498-4aa1-9de7-77da6122abf0); thread_spmc.c
 * routes such messages to virtio_msg_ffa_recv().
 *
 * One struct virtio_msg_ffa_ep is instantiated per non-secure FF-A endpoint,
 * each owning an independent bus, device table and shared-memory area list,
 * so multiple VMs are isolated from one another.
 */

/*
 * virtio_msg_ffa_recv() - handle one inbound virtio-msg over FF-A.
 * @args:	the DIRECT_REQ2 registers; the message occupies args->a[4..].
 * @caller_id:	FF-A id of the sending (non-secure) endpoint.
 *
 * Decodes the FF-A bus framing, dispatches to the per-endpoint bus and writes
 * the response back in place, zero-padded to the message buffer size.
 */
void virtio_msg_ffa_recv(struct thread_smc_1_2_regs *args, uint16_t caller_id);

/*
 * virtio_msg_ffa_register_driver() - register a device-type driver.
 *
 * The driver's init() is called for every endpoint (existing and future) to
 * attach its device instances to that endpoint's bus. Mirrors QTEE's
 * ffa_bus_dev_driver_add(). Must be called before any endpoint is created
 * (i.e. from an init level no later than the LSP init).
 */
TEE_Result virtio_msg_ffa_register_driver(struct virtio_msg_bus_driver *drv);

#endif /*__KERNEL_VIRTIO_MSG_FFA_H*/
