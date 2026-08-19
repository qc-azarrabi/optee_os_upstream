/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2025 NXP
 */

#ifndef __SBI_MPXY_RPMI_H
#define __SBI_MPXY_RPMI_H

#if defined(CFG_RISCV_SBI_MPXY_RPMI)

#ifndef __ASSEMBLER__

#include <compiler.h>
#include <encoding.h>
#include <rpmi.h>
#include <sbi_mpxy.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <types_ext.h>
#include <util.h>

/** Message Header Size in bytes */
#define RPMI_MSG_HDR_SIZE		(8)
/** Data field size in bytes */
#define RPMI_MSG_DATA_SIZE(__slot_size) ((__slot_size) - RPMI_MSG_HDR_SIZE)
/** Minimum slot size in bytes */
#define RPMI_SLOT_SIZE_MIN		(64)

/* RPMI message protocol specific MPXY attributes */
enum sbi_mpxy_rpmi_attribute_id {
	SBI_MPXY_RPMI_ATTR_SERVICEGROUP_ID = SBI_MPXY_ATTR_MSGPROTO_ATTR_START,
	SBI_MPXY_RPMI_ATTR_SERVICEGROUP_VERSION,
	SBI_MPXY_RPMI_ATTR_IMPLEMENTATION_ID,
	SBI_MPXY_RPMI_ATTR_IMPLEMENTATION_VERSION,
	SBI_MPXY_RPMI_ATTR_MAX_ID
};

/* RPMI message protocol specific MPXY message types */
enum sbi_mpxy_rpmi_message_type {
	SBI_MPXY_RPMI_MSG_TYPE_GET_ATTRIBUTE,
	SBI_MPXY_RPMI_MSG_TYPE_SET_ATTRIBUTE,
	SBI_MPXY_RPMI_MSG_TYPE_SEND_WITH_RESPONSE,
	SBI_MPXY_RPMI_MSG_TYPE_SEND_WITHOUT_RESPONSE,
	SBI_MPXY_RPMI_MSG_TYPE_NOTIFICATION_EVENT,
	SBI_MPXY_RPMI_MSG_MAX_TYPE
};

/* RPMI specific SBI MPXY channel attributes. */
struct sbi_mpxy_rpmi_channel_attrs {
	uint32_t servicegroup_id;
	uint32_t servicegroup_version;
	uint32_t implementation_id;
	uint32_t implementation_version;
};

/* RPMI/MPXY message instance */
struct sbi_mpxy_rpmi_message {
	enum sbi_mpxy_rpmi_message_type type;
	union {
		struct {
			enum sbi_mpxy_rpmi_attribute_id id;
			uint32_t value;
		} attribute;

		struct {
			uint32_t service_id;
			void *request;
			unsigned long request_len;
			void *response;
			unsigned long response_len;
			unsigned long max_response_len;
		} data;

		struct {
			uint16_t event_datalen;
			uint8_t event_id;
			uint8_t *event_data;
		} notification;
	};
	int error;
};

/*
 * struct sbi_mpxy_rpmi_channel - RPMI-capable MPXY channel descriptor
 * @hart_id:        Logical hart ID associated with the channel (if applicable)
 * @channel_id:     MPXY channel ID as returned by SBI
 * @attrs:          Basic MPXY channel attributes (protocol ID, max len, etc.)
 * @rpmi_attrs:     RPMI-specific channel attributes (service group, MSI, etc.)
 * @notif:          Pointer to allocated buffer for receiving notifications
 */
struct sbi_mpxy_rpmi_channel {
	uint32_t hart_id;
	uint32_t channel_id;
	struct sbi_mpxy_channel_attrs attrs;
	struct sbi_mpxy_rpmi_channel_attrs rpmi_attrs;
	struct sbi_mpxy_notification_data *notif;
};

/* An instance of RPMI-over-MPXY channel group */
struct sbi_mpxy_rpmi_context {
	uint32_t channel_count;
	unsigned long mpxy_shmem_size;
	struct sbi_mpxy_rpmi_channel *channels;
};

/* Forward declaration of the RPMI context */
extern struct sbi_mpxy_rpmi_context *sbi_mpxy_rpmi_ctx;

struct thread_abi_args;

/* RPMI Request Forward ServiceGroup Definitions */
struct rpmi_reqfwd_retrieve_current_message_req {
	uint32_t start_index;
};

struct rpmi_reqfwd_retrieve_current_message_resp {
	int32_t status;
	uint32_t remaining;
	uint32_t returned;
	/* remaining space need to be adjusted for the above 3 u32's */
	uint8_t request_message[RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) -
				(sizeof(uint32_t) * 3)];
};

struct rpmi_reqfwd_complete_current_message_req {
	uint8_t response_data[RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN)];
};

struct rpmi_reqfwd_complete_current_message_resp {
	int32_t status;
	uint32_t num_messages;
};

/** RPMI Request Forward ServiceGroup Service IDs */
enum rpmi_reqfwd_service_id {
	RPMI_REQFWD_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_REQFWD_SRV_RETRIEVE_CURRENT_MESSAGE = 0x02,
	RPMI_REQFWD_SRV_COMPLETE_CURRENT_MESSAGE = 0x03,
	RPMI_REQFWD_SRV_MAX_COUNT,
};

/** RPMI/MPXY message helper routines */

static inline void
sbi_mpxy_rpmi_init_get_attribute(struct sbi_mpxy_rpmi_message *message,
				 enum sbi_mpxy_rpmi_attribute_id id)
{
	message->type = SBI_MPXY_RPMI_MSG_TYPE_GET_ATTRIBUTE;
	message->attribute.id = id;
	message->attribute.value = 0;
	message->error = 0;
}

static inline void
sbi_mpxy_rpmi_init_set_attribute(struct sbi_mpxy_rpmi_message *message,
				 enum sbi_mpxy_rpmi_attribute_id id,
				 uint32_t value)
{
	message->type = SBI_MPXY_RPMI_MSG_TYPE_SET_ATTRIBUTE;
	message->attribute.id = id;
	message->attribute.value = value;
	message->error = 0;
}

static inline void
sbi_mpxy_rpmi_init_send_with_response(struct sbi_mpxy_rpmi_message *message,
				      uint32_t service_id, void *request,
				      unsigned long request_len, void *response,
				      unsigned long max_response_len)
{
	message->type = SBI_MPXY_RPMI_MSG_TYPE_SEND_WITH_RESPONSE;
	message->data.service_id = service_id;
	message->data.request = request;
	message->data.request_len = request_len;
	message->data.response = response;
	message->data.response_len = 0;
	message->data.max_response_len = max_response_len;
	message->error = 0;
}

static inline void
sbi_mpxy_rpmi_init_send_without_response(struct sbi_mpxy_rpmi_message *message,
					 uint32_t service_id, void *request,
					 unsigned long request_len)
{
	message->type = SBI_MPXY_RPMI_MSG_TYPE_SEND_WITHOUT_RESPONSE;
	message->data.service_id = service_id;
	message->data.request = request;
	message->data.request_len = request_len;
	message->data.response = NULL;
	message->data.max_response_len = 0;
	message->data.response_len = 0;
	message->error = 0;
}

void sbi_mpxy_rpmi_probe_channels(void);
int sbi_mpxy_rpmi_read_attributes(struct sbi_mpxy_rpmi_channel *channel);
int sbi_mpxy_rpmi_send_data(struct sbi_mpxy_rpmi_channel *channel, void *data);
void thread_return_to_udomain_by_sbi_mpxy(unsigned long arg0,
					  unsigned long arg1,
					  unsigned long arg2,
					  unsigned long arg3,
					  unsigned long arg4,
					  unsigned long arg5 __unused);

/*
 * Return the MPXY channel id of the RPMI TEE service group (0x0010), used by
 * parcel consumption to issue MEM_PARCEL_ACCEPT / RELEASE. Returns 0 on
 * success, -1 if no TEE channel was probed.
 */
int sbi_mpxy_rpmi_get_tee_channel_id(uint32_t *channel_id);

/*
 * Consume a parcel forwarded from the REE: accept it on the TEE channel, map
 * its non-secure pages into OP-TEE core, write the pattern to prove access,
 * then unmap and release. The four 32-bit inputs are packed into two registers:
 *   a1 = parcel_id      | (nonce   << 32)
 *   a2 = creator_access | (pattern << 32)
 * On return a0 = status, a1 = value read back.
 */
void rpmi_tee_parcel_consume(struct thread_abi_args *args);

#endif /*__ASSEMBLER__*/
#endif /*defined(CFG_RISCV_SBI_MPXY_RPMI)*/
#endif /*__SBI_MPXY_RPMI_H*/
