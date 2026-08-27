// SPDX-License-Identifier: BSD-2-Clause
/*
 * RISC-V asynchronous-notification backend. This is the RISC-V counterpart of
 * core/kernel/notif_default.c: it keeps the same per-guest pending-value bitmap
 * and the same notif_alloc/free/get_value semantics, but the outbound "wake the
 * normal world" step is NOT a GIC PPI (interrupt_raise_pi() as on ARM). Instead
 * it issues the standard RPMI TEE SIGNAL_RAISE (0x07) service on the REE-facing
 * TEE MPXY channel, exactly as the memory-parcel path issues MEM_PARCEL_ACCEPT /
 * RELEASE (sbi_mpxy_rpmi_get_tee_channel_id() + a framework-answered service).
 *
 * The signal is only a doorbell: signal index 0 tells the REE "async-notif is
 * pending, come drain it". The REE observes the raised signal and then drains
 * the actual 64-bit notification values via the existing GET_ASYNC_NOTIF_VALUE
 * fast-call.
 */

#include <assert.h>
#include <bitstring.h>
#include <config.h>
#include <initcall.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/virtualization.h>
#include <sbi_mpxy.h>
#include <sbi_mpxy_rpmi.h>
#include <string.h>
#include <trace.h>
#include <types_ext.h>

#define RPMI_TEE_SRV_SIGNAL_RAISE	0x07
#define RPMI_TEE_ENDPOINT_REE		0
#define RPMI_TEE_ENDPOINT_OPTEE		1
/* Signal index OP-TEE raises to mean "async-notif pending, go drain". */
#define RPMI_TEE_SIGNAL_ASYNC_NOTIF	0

/* rpmi_tee_signal_raise_req: { u32 target_id; u32 signal_len; u32 signal[]; } */
struct signal_raise_req {
	uint32_t target_id;
	uint32_t signal_len;
	uint32_t signal[1];
};

/* rpmi_tee_signal_raise_resp: { s32 status; } */
struct signal_raise_resp {
	int32_t status;
};

struct notif_vm_bitmap {
	bool alloc_values_inited;
	bitstr_t bit_decl(values, NOTIF_ASYNC_VALUE_MAX + 1);
	bitstr_t bit_decl(alloc_values, NOTIF_ASYNC_VALUE_MAX + 1);
};

static unsigned int notif_lock = SPINLOCK_UNLOCK;
/* Id used to look up the guest specific struct notif_vm_bitmap */
static unsigned int notif_vm_bitmap_id __nex_bss;
/* Notification state when ns-virtualization isn't enabled */
static struct notif_vm_bitmap default_notif_vm_bitmap;

static struct notif_vm_bitmap *get_notif_vm_bitmap(struct guest_partition *prtn)
{
	if (IS_ENABLED(CFG_NS_VIRTUALIZATION)) {
		if (!prtn)
			return NULL;
		return virt_get_guest_spec_data(prtn, notif_vm_bitmap_id);
	}
	return &default_notif_vm_bitmap;
}

TEE_Result notif_alloc_async_value(uint32_t *val)
{
	struct guest_partition *prtn = NULL;
	struct notif_vm_bitmap *nvb = NULL;
	TEE_Result res = TEE_SUCCESS;
	uint32_t old_itr_status = 0;
	int bit = 0;

	prtn = virt_get_current_guest();
	nvb = get_notif_vm_bitmap(prtn);
	if (!nvb) {
		res = TEE_ERROR_BAD_PARAMETERS;
		goto out;
	}

	old_itr_status = cpu_spin_lock_xsave(&notif_lock);

	if (!nvb->alloc_values_inited) {
		bit_set(nvb->alloc_values, NOTIF_VALUE_DO_BOTTOM_HALF);
		nvb->alloc_values_inited = true;
	}

	bit_ffc(nvb->alloc_values, (int)NOTIF_ASYNC_VALUE_MAX + 1, &bit);
	if (bit < 0) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto out_unlock;
	}
	*val = bit;
	bit_set(nvb->alloc_values, bit);

out_unlock:
	cpu_spin_unlock_xrestore(&notif_lock, old_itr_status);
out:
	virt_put_guest(prtn);

	return res;
}

void notif_free_async_value(uint32_t val)
{
	struct guest_partition *prtn = NULL;
	struct notif_vm_bitmap *nvb = NULL;
	uint32_t old_itr_status = 0;

	prtn = virt_get_current_guest();
	nvb = get_notif_vm_bitmap(prtn);
	if (!nvb)
		goto out;

	old_itr_status = cpu_spin_lock_xsave(&notif_lock);

	assert(val < NOTIF_ASYNC_VALUE_MAX);
	assert(bit_test(nvb->alloc_values, val));
	bit_clear(nvb->alloc_values, val);

	cpu_spin_unlock_xrestore(&notif_lock, old_itr_status);
out:
	virt_put_guest(prtn);
}

uint32_t notif_get_value(bool *value_valid, bool *value_pending)
{
	struct guest_partition *prtn = NULL;
	struct notif_vm_bitmap *nvb = NULL;
	uint32_t old_itr_status = 0;
	uint32_t res = 0;
	int bit = -1;

	prtn = virt_get_current_guest();
	nvb = get_notif_vm_bitmap(prtn);
	if (!nvb) {
		*value_valid = false;
		goto out;
	}

	old_itr_status = cpu_spin_lock_xsave(&notif_lock);

	bit_ffs(nvb->values, (int)NOTIF_ASYNC_VALUE_MAX + 1, &bit);
	*value_valid = (bit >= 0);
	if (!*value_valid)
		goto out_unlock;

	res = bit;
	bit_clear(nvb->values, res);
	bit_ffs(nvb->values, (int)NOTIF_ASYNC_VALUE_MAX + 1, &bit);

out_unlock:
	cpu_spin_unlock_xrestore(&notif_lock, old_itr_status);
out:
	virt_put_guest(prtn);
	*value_pending = (bit >= 0);

	return res;
}

/*
 * Emit the outbound doorbell: standard RPMI TEE SIGNAL_RAISE(target=REE,
 * signal 0) on the REE-facing TEE MPXY channel. Framework-answered in OpenSBI
 * (no domain switch, no forwarding), the same pattern as the memory parcels.
 */
static void raise_ree_async_notif_signal(void)
{
	struct signal_raise_req req = { };
	struct signal_raise_resp resp = { };
	unsigned long resp_len = 0;
	uint32_t channel_id = 0;
	int rc;

	if (sbi_mpxy_rpmi_get_tee_channel_id(&channel_id)) {
		EMSG("async-notif: no TEE channel");
		return;
	}

	req.target_id = RPMI_TEE_ENDPOINT_REE;
	req.signal_len = 1;
	req.signal[0] = RPMI_TEE_SIGNAL_ASYNC_NOTIF;

	rc = sbi_mpxy_send_message_with_response(channel_id,
						 RPMI_TEE_SRV_SIGNAL_RAISE,
						 &req, sizeof(req),
						 &resp, sizeof(resp),
						 &resp_len);
	if (rc || resp.status)
		EMSG("async-notif: SIGNAL_RAISE failed rc=%d status=%d",
		     rc, resp.status);
}

void notif_send_async(uint32_t value, uint16_t guest_id)
{
	struct guest_partition *prtn = NULL;
	struct notif_vm_bitmap *nvb = NULL;
	uint32_t old_itr_status = 0;

	assert(value <= NOTIF_ASYNC_VALUE_MAX);

	prtn = virt_get_guest(guest_id);
	nvb = get_notif_vm_bitmap(prtn);
	if (!nvb)
		goto out;

	old_itr_status = cpu_spin_lock_xsave(&notif_lock);
	bit_set(nvb->values, value);
	cpu_spin_unlock_xrestore(&notif_lock, old_itr_status);

	/* Doorbell the REE outside the lock (blocking SBI MPXY ecall). */
	raise_ree_async_notif_signal();
out:
	virt_put_guest(prtn);
}

static TEE_Result notif_init(void)
{
	if (IS_ENABLED(CFG_NS_VIRTUALIZATION) &&
	    virt_add_guest_spec_data(&notif_vm_bitmap_id,
				     sizeof(struct notif_vm_bitmap), NULL))
		panic("virt_add_guest_spec_data");
	return TEE_SUCCESS;
}
nex_service_init(notif_init);
