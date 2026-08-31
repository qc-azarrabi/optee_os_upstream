// SPDX-License-Identifier: BSD-2-Clause
/*
 * Debug pseudo TA to fire the OP-TEE -> REE async-notification doorbell on
 * demand from normal world. On the RISC-V virt platform there is no secure
 * timer or secure interrupt to self-drive the async-notif producer, so this
 * PTA lets a small client app poke the real producer entry, notif_send_async(),
 * which raises the RPMI TEE SIGNAL_RAISE(target=REE) that OpenSBI turns into a
 * System MSI. No new RPMI service and no non-spec behavior: only the trigger is
 * a test poke.
 */

#include <config.h>
#include <kernel/notif.h>
#include <kernel/pseudo_ta.h>
#include <tee_api_defines.h>
#include <tee_api_types.h>

#define PTA_NAME "tee_signal_poke.pta"

/* f0e5a9b1-3c7d-4e2a-9b6f-1a2b3c4d5e6f */
#define PTA_TEE_SIGNAL_POKE_UUID \
	{ 0xf0e5a9b1, 0x3c7d, 0x4e2a, \
	  { 0x9b, 0x6f, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f } }

/*
 * PTA_TEE_SIGNAL_POKE_CMD_RAISE - raise the async-notif doorbell.
 * [in]  value[0].a  async value to signal (optional; 0 => do-bottom-half).
 *                   Must be <= NOTIF_ASYNC_VALUE_MAX.
 */
#define PTA_TEE_SIGNAL_POKE_CMD_RAISE 0

static TEE_Result cmd_raise(uint32_t param_types,
			    TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t value = NOTIF_VALUE_DO_BOTTOM_HALF;

	if (TEE_PARAM_TYPE_GET(param_types, 0) == TEE_PARAM_TYPE_VALUE_INPUT) {
		value = params[0].value.a;
		if (value > NOTIF_ASYNC_VALUE_MAX)
			return TEE_ERROR_BAD_PARAMETERS;
	} else if (param_types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_NONE,
						  TEE_PARAM_TYPE_NONE,
						  TEE_PARAM_TYPE_NONE,
						  TEE_PARAM_TYPE_NONE)) {
		return TEE_ERROR_BAD_PARAMETERS;
	}

	/* guest_id 0: no NS-virtualization on this platform */
	notif_send_async(value, 0);

	return TEE_SUCCESS;
}

static TEE_Result invoke_command(void *session __unused, uint32_t cmd,
				 uint32_t param_types,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	switch (cmd) {
	case PTA_TEE_SIGNAL_POKE_CMD_RAISE:
		return cmd_raise(param_types, params);
	default:
		return TEE_ERROR_NOT_IMPLEMENTED;
	}
}

pseudo_ta_register(.uuid = PTA_TEE_SIGNAL_POKE_UUID, .name = PTA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = invoke_command);
