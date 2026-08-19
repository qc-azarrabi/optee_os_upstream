// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2025 NXP
 */

#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/thread_private.h>
#include <riscv.h>
#include <rpmi.h>
#include <sbi.h>
#include <sbi_mpxy.h>
#include <sbi_mpxy_rpmi.h>
#include <stdlib.h>
#include <string.h>
#include <tee/optee_abi.h>
#include <tee/teeabi_opteed.h>
#include <tee/teeabi_opteed_macros.h>

struct sbi_mpxy_rpmi_context *sbi_mpxy_rpmi_ctx;

/**
 * @brief Probes available MPXY channels supporting the RPMI protocol.
 *
 * This function initializes the global RPMI context by identifying available
 * MPXY channels with the RPMI protocol, reading their attributes, and
 * allocating memory for handling notifications. The result is stored in a
 * global context (sbi_mpxy_rpmi_ctx). If probing fails, the context is set to
 * NULL.
 */
void sbi_mpxy_rpmi_probe_channels(void)
{
	struct sbi_mpxy_rpmi_channel *channel = NULL;
	uint32_t *channel_ids = NULL;
	unsigned long shmem_size = 0;
	uint32_t valid_channels = 0;
	uint32_t i = 0;
	int ret = 0;
	uint32_t channel_id = 0;

	if (!sbi_probe_extension(SBI_EXT_MPXY))
		panic("sbi mpxy extension must be supported");

	if (sbi_mpxy_rpmi_ctx) {
		EMSG("RPMI/MPXY context already initialized");
		return;
	}

	sbi_mpxy_rpmi_ctx = calloc(1, sizeof(*sbi_mpxy_rpmi_ctx));
	if (!sbi_mpxy_rpmi_ctx) {
		EMSG("Out of memory for RPMI context");
		goto error;
	}

	ret = sbi_mpxy_get_shmem_size(&shmem_size);
	if (ret) {
		EMSG("Failed to get MPXY shared memory size (ret=%d)", ret);
		goto error;
	}

	sbi_mpxy_rpmi_ctx->mpxy_shmem_size = shmem_size;

	/* Setup MPXY shared memory on current hart */
	ret = sbi_mpxy_set_shmem(shmem_size);
	if (ret) {
		EMSG("Failed to set MPXY shared memory (ret=%d)", ret);
		goto error;
	}

	ret = sbi_mpxy_get_channel_count(&sbi_mpxy_rpmi_ctx->channel_count);
	if (ret || !sbi_mpxy_rpmi_ctx->channel_count) {
		EMSG("Failed to get MPXY channel count (ret=%d)", ret);
		goto error;
	}

	channel_ids =
		calloc(sbi_mpxy_rpmi_ctx->channel_count, sizeof(*channel_ids));
	if (!channel_ids) {
		EMSG("Failed to allocate channel ID list");
		goto error;
	}

	ret = sbi_mpxy_get_channel_ids(sbi_mpxy_rpmi_ctx->channel_count,
				       channel_ids);
	if (ret) {
		EMSG("Failed to fetch channel IDs (ret=%d)", ret);
		goto error;
	}

	sbi_mpxy_rpmi_ctx->channels =
		calloc(sbi_mpxy_rpmi_ctx->channel_count,
		       sizeof(*sbi_mpxy_rpmi_ctx->channels));
	if (!sbi_mpxy_rpmi_ctx->channels) {
		EMSG("Failed to allocate channel table");
		goto error;
	}

	for (i = 0; i < sbi_mpxy_rpmi_ctx->channel_count; i++) {
		channel_id = channel_ids[i];
		channel = &sbi_mpxy_rpmi_ctx->channels[valid_channels];
		channel->channel_id = channel_id;

		ret = sbi_mpxy_read_attributes(channel_id,
					       SBI_MPXY_ATTR_MSG_PROT_ID,
					       sizeof(channel->attrs) /
						       sizeof(uint32_t),
					       &channel->attrs);
		if (ret) {
			EMSG("Failed to read MPXY attributes for channel %u",
			     channel_id);
			continue;
		}

		if (channel->attrs.msg_proto_id != SBI_MPXY_MSGPROTO_RPMI_ID) {
			DMSG("Channel %u is not RPMI (proto_id=%u), skipping",
			     channel_id, channel->attrs.msg_proto_id);
			continue;
		}

		ret = sbi_mpxy_rpmi_read_attributes(channel);
		if (ret) {
			EMSG("Failed to read RPMI attributes for channel %u",
			     channel_id);
			continue;
		}

		channel->notif = malloc(shmem_size);
		if (!channel->notif) {
			EMSG("No memory for channel %u notif buffer",
			     channel_id);
			goto error;
		}

		memset(channel->notif, 0, shmem_size);
		valid_channels++;
	}

	free(channel_ids);
	channel_ids = NULL;

	if (!valid_channels) {
		EMSG("No usable RPMI channels found");
		goto error;
	}

	assert(sbi_mpxy_rpmi_ctx->channel_count == valid_channels);

	return;

error:
	if (channel_ids)
		free(channel_ids);

	if (sbi_mpxy_rpmi_ctx) {
		if (sbi_mpxy_rpmi_ctx->channels) {
			for (i = 0; i < valid_channels; i++)
				free(sbi_mpxy_rpmi_ctx->channels[i].notif);
			free(sbi_mpxy_rpmi_ctx->channels);
		}
		free(sbi_mpxy_rpmi_ctx);
		sbi_mpxy_rpmi_ctx = NULL;
	}
}

/**
 * @brief Reads RPMI-specific attributes from a given MPXY channel.
 *
 * This function queries and fills the RPMI-specific attribute structure
 * (rpmi_attrs) for the specified channel using the SBI MPXY interface.
 *
 * @param channel Pointer to the RPMI channel instance to query.
 *
 * @return 0 on success, or a negative SBI error code on failure.
 */
int sbi_mpxy_rpmi_read_attributes(struct sbi_mpxy_rpmi_channel *channel)
{
	return sbi_mpxy_read_attributes(channel->channel_id,
					SBI_MPXY_ATTR_MSGPROTO_ATTR_START,
					sizeof(channel->rpmi_attrs) /
						sizeof(uint32_t),
					&channel->rpmi_attrs);
}

/**
 * @brief Sends a raw RPMI message over an MPXY channel.
 *
 * This function transmits a message to the associated platform microcontroller
 * (PuC) using the given RPMI-enabled MPXY channel.
 *
 * @param channel Pointer to the RPMI channel used for transmission.
 * @param data Pointer to the message payload to send. It must be a
 *             properly initialized RPMI message structure.
 *
 * @return 0 on success, or a negative RPMI or SBI error code on failure.
 */
int sbi_mpxy_rpmi_send_data(struct sbi_mpxy_rpmi_channel *channel, void *data)
{
	struct sbi_mpxy_rpmi_message *message = data;
	int ret = 0;

	if (channel->attrs.msg_proto_id != SBI_MPXY_MSGPROTO_RPMI_ID)
		return RPMI_ERR_NOTSUPP;

	switch (message->type) {
	case SBI_MPXY_RPMI_MSG_TYPE_GET_ATTRIBUTE:
		switch (message->attribute.id) {
		case SBI_MPXY_RPMI_ATTR_SERVICEGROUP_ID:
			message->attribute.value =
				channel->rpmi_attrs.servicegroup_id;
			break;
		case SBI_MPXY_RPMI_ATTR_SERVICEGROUP_VERSION:
			message->attribute.value =
				channel->rpmi_attrs.servicegroup_version;
			break;
		case SBI_MPXY_RPMI_ATTR_IMPLEMENTATION_ID:
			message->attribute.value =
				channel->rpmi_attrs.implementation_id;
			break;
		case SBI_MPXY_RPMI_ATTR_IMPLEMENTATION_VERSION:
			message->attribute.value =
				channel->rpmi_attrs.implementation_version;
			break;
		default:
			ret = RPMI_ERR_NOTSUPP;
			break;
		}
		break;
	case SBI_MPXY_RPMI_MSG_TYPE_SET_ATTRIBUTE:
		/*
		 * All RPMI Message Protocol Attributes of an SBI MPXY Channel
		 * are RO.
		 */
		ret = RPMI_ERR_NOTSUPP;
		break;
	case SBI_MPXY_RPMI_MSG_TYPE_SEND_WITH_RESPONSE:
		if ((!message->data.request && message->data.request_len) ||
		    (!message->data.response &&
		     message->data.max_response_len)) {
			ret = RPMI_ERR_INVALID_PARAM;
			break;
		}
		if (!(channel->attrs.capability &
		      SBI_MPXY_CHAN_CAP_SEND_WITH_RESP)) {
			ret = RPMI_ERR_IO;
			break;
		}
		ret = sbi_mpxy_send_message_with_response(channel->channel_id
		      , message->data.service_id, message->data.request,
		      message->data.request_len, message->data.response,
		      message->data.max_response_len,
		      &message->data.response_len);
		break;
	case SBI_MPXY_RPMI_MSG_TYPE_SEND_WITHOUT_RESPONSE:
		if (!message->data.request && message->data.request_len) {
			ret = RPMI_ERR_INVALID_PARAM;
			break;
		}
		if (!(channel->attrs.capability &
		      SBI_MPXY_CHAN_CAP_SEND_WITHOUT_RESP)) {
			ret = RPMI_ERR_IO;
			break;
		}
		ret = sbi_mpxy_send_message_without_response(channel->channel_id
		      , message->data.service_id, message->data.request,
		      message->data.request_len);
		break;
	default:
		ret = RPMI_ERR_NOTSUPP;
		break;
	}

	message->error = ret;

	return RPMI_SUCCESS;
}

/**
 * @brief Retrieves the channel ID for the Request Forwarding service group.
 *
 * This function searches for the channel ID associated with the Request
 * Forwarding service group for a given hart ID.
 *
 * @param hartid The hart ID for which to retrieve the channel ID.
 * @param channel_id Pointer to store the retrieved channel ID.
 *
 * @return 0 on success, or -1 if the channel is not found.
 */
static int rpmi_get_reqfwd_channel_id_by_hartid(uint32_t hartid,
						uint32_t *channel_id)
{
	struct sbi_mpxy_rpmi_channel *channel = NULL;
	uint32_t i = 0;

	if (!sbi_mpxy_rpmi_ctx)
		return -1;

	for (i = 0; i < sbi_mpxy_rpmi_ctx->channel_count; i++) {
		channel = &sbi_mpxy_rpmi_ctx->channels[i];
		if (channel->rpmi_attrs.servicegroup_id !=
		    RPMI_SRVGRP_REQUEST_FORWARD) {
			continue;
		}
		if (channel->hart_id == hartid) {
			*channel_id = channel->channel_id;
			return 0;
		}
	}

	return -1;
}

/*
 * Return the MPXY channel id of the RPMI TEE service group (0x0010). Parcel
 * consumption issues MEM_PARCEL_ACCEPT / MEM_PARCEL_RELEASE on this channel.
 * The trusted domain reaches it directly; MPXY channel access is not
 * domain-gated. Returns 0 on success, -1 if no TEE channel was probed.
 */
#define RPMI_SRVGRP_TEE			0x0010

int sbi_mpxy_rpmi_get_tee_channel_id(uint32_t *channel_id)
{
	struct sbi_mpxy_rpmi_channel *channel = NULL;
	uint32_t i = 0;

	if (!sbi_mpxy_rpmi_ctx)
		return -1;

	for (i = 0; i < sbi_mpxy_rpmi_ctx->channel_count; i++) {
		channel = &sbi_mpxy_rpmi_ctx->channels[i];
		if (channel->rpmi_attrs.servicegroup_id == RPMI_SRVGRP_TEE) {
			*channel_id = channel->channel_id;
			return 0;
		}
	}

	return -1;
}

/* Called with all exception being masked */
static void
thread_sbi_mpxy_reqfwd_retrieve_message(struct thread_abi_args *args)
{
	struct rpmi_reqfwd_retrieve_current_message_req req;
	struct rpmi_reqfwd_retrieve_current_message_resp resp;
	uint32_t hartid = 0, channel_id = 0;
	unsigned long ack_len;
	uint8_t *dest = (uint8_t *)args;
	uint32_t total_received = 0;
	const uint32_t expected_size = sizeof(*args);
	int rc;

	hartid = thread_get_hartid();

	rc = rpmi_get_reqfwd_channel_id_by_hartid(hartid, &channel_id);
	if (rc) {
		EMSG("Failed to get reqfwd channel id for hart %d", hartid);
		panic();
	}

	do {
		req.start_index = total_received;

		rc = sbi_mpxy_send_message_with_response(channel_id,
				RPMI_REQFWD_SRV_RETRIEVE_CURRENT_MESSAGE,
				&req, sizeof(req), &resp, sizeof(resp),
				&ack_len);

		if (rc != SBI_SUCCESS || !ack_len) {
			EMSG("SBI ReqFwd retrieve message failed: rc=%d, ack_len=%lu",
			     rc, ack_len);
			panic("SBI ReqFwd retrieve message returns error");
		}

		if (resp.status != RPMI_SUCCESS) {
			EMSG("RPMI ReqFwd retrieve failed: status=%d", resp.status);
			panic("RPMI ReqFwd retrieve status error");
		}

		memcpy(dest + total_received, resp.request_message, resp.returned);
		total_received += resp.returned;
	} while (resp.remaining > 0);

	if (total_received != expected_size) {
		EMSG("REQFWD incomplete message: expected %u bytes, got %u bytes",
		     expected_size, total_received);
		panic("REQFWD message size mismatch");
	}
}

/* Called with all exception being masked */
static void
thread_sbi_mpxy_reqfwd_complete_message(struct thread_abi_args *args)
{
	struct rpmi_reqfwd_complete_current_message_req req = { };
	struct rpmi_reqfwd_complete_current_message_resp resp = { };
	uint32_t hartid = 0, channel_id = 0;
	unsigned long ack_len;
	const uint32_t response_size = sizeof(unsigned long) * 5;
	int rc;

	hartid = thread_get_hartid();

	rc = rpmi_get_reqfwd_channel_id_by_hartid(hartid, &channel_id);
	if (rc) {
		EMSG("Failed to get reqfwd channel id for hart %d", hartid);
		panic();
	}

	memcpy(req.response_data, args, response_size);

	rc = sbi_mpxy_send_message_with_response(channel_id,
			RPMI_REQFWD_SRV_COMPLETE_CURRENT_MESSAGE,
			&req, response_size,
			&resp, sizeof(resp),
			&ack_len);

	if (rc != SBI_SUCCESS || resp.status != RPMI_SUCCESS)
		panic("SBI ReqFwd complete message returns error");
}

#define ABI_ENTRY_TYPE_FAST		1
#define ABI_ENTRY_TYPE_YIELD		0
#define FUNCID_TYPE_SHIFT		31
#define FUNCID_TYPE_MASK		0x1
#define GET_ABI_ENTRY_TYPE(id)		(((id) >> FUNCID_TYPE_SHIFT) & \
					 FUNCID_TYPE_MASK)

static void thread_handle_request(struct thread_abi_args *args)
{
	uint32_t funcid_type;
	int rc;

	// DMSG("Sent from host domain: "
	//      "args->a0=0x%08lX, args->a1=0x%08lX, args->a2=0x%08lX "
	//      "args->a3=0x%08lX, args->a4=0x%08lX, args->a5=0x%08lX",
	//      args->a0, args->a1, args->a2, args->a3, args->a4, args->a5);

	funcid_type = GET_ABI_ENTRY_TYPE(args->a0);
	if (funcid_type == ABI_ENTRY_TYPE_YIELD) {
		rc = thread_handle_std_abi(args->a0, args->a1, args->a2,
					   args->a3, args->a4, args->a5,
					   args->a6, args->a7);

		/*
		 * Normally thread_handle_std_abi() should return via
		 * thread_rpc(), but if thread_handle_std_abi() hasn't switched
		 * stack (error detected) it will do a normal "C" return.
		 */
		/* Restore thread_handle_std_abi() return value */
		args->a1 = rc;
		args->a2 = 0;
		args->a3 = 0;
		args->a4 = 0;
		args->a5 = 0;
		args->a0 = TEEABI_OPTEED_RETURN_CALL_DONE;
	} else if (funcid_type == ABI_ENTRY_TYPE_FAST) {
		thread_handle_fast_abi(args);
		args->a5 = args->a4;
		args->a4 = args->a3;
		args->a3 = args->a2;
		args->a2 = args->a1;
		args->a1 = args->a0;
		args->a0 = TEEABI_OPTEED_RETURN_CALL_DONE;
	}

	// DMSG("Send to host domain: "
	//      "args->a0=0x%08lX, args->a1=0x%08lX, args->a2=0x%08lX "
	//      "args->a3=0x%08lX, args->a4=0x%08lX, args->a5=0x%08lX",
	//      args->a0, args->a1, args->a2, args->a3, args->a4, args->a5);
}

void __noreturn
thread_return_to_udomain_by_sbi_mpxy(unsigned long arg0,
				     unsigned long arg1,
				     unsigned long arg2,
				     unsigned long arg3,
				     unsigned long arg4,
				     unsigned long arg5 __unused)
{
	struct thread_abi_args args = { .a0 = arg0, .a1 = arg1, .a2 = arg2,
					.a3 = arg3, .a4 = arg4 };

	assert((thread_get_exceptions() & THREAD_EXCP_ALL) == THREAD_EXCP_ALL);

	/*
	 * Complete message except the following two cases:
	 *  - a0 = TEEABI_OPTEED_RETURN_ENTRY_DONE
	 *  - a0 = TEEABI_OPTEED_RETURN_ON_DONE
	 * These two cases happen in boot time when OP-TEE finishes boot time
	 * initialization. There are no message to be handled so we don't need
	 * to complete message.
	 */
	if (arg0 == TEEABI_OPTEED_RETURN_ENTRY_DONE ||
	    arg0 == TEEABI_OPTEED_RETURN_ON_DONE)
	    goto msg_loop;

	thread_sbi_mpxy_reqfwd_complete_message(&args);

msg_loop:
	while (1) {
		thread_sbi_mpxy_reqfwd_retrieve_message(&args);
		thread_handle_request(&args);
		thread_sbi_mpxy_reqfwd_complete_message(&args);
	}
}
