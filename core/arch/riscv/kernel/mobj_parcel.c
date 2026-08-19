// SPDX-License-Identifier: BSD-2-Clause
/*
 * OP-TEE-side consumption of an RPMI TEE memory parcel.
 *
 * Given a parcel handle forwarded from the REE, this calls MEM_PARCEL_ACCEPT
 * on the TEE MPXY channel to retrieve the parcel's block list, maps the
 * described non-secure pages into OP-TEE core via a registered-shared-memory
 * mobj, writes a caller-supplied pattern so the REE can observe genuine TEE
 * access to the shared memory, then unmaps and issues MEM_PARCEL_RELEASE.
 *
 * This exercises real parcel consumption end to end, as opposed to the
 * framework-side bookkeeping and ownership validation performed in OpenSBI.
 */

#include <kernel/thread_private.h>
#include <mm/core_mmu.h>
#include <mm/mobj.h>
#include <sbi_mpxy.h>
#include <sbi_mpxy_rpmi.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <types_ext.h>
#include <util.h>

/*
 * RPMI TEE wire constants. These mirror the OpenSBI definitions in
 * <sbi_utils/mailbox/rpmi_msgprot.h> and are duplicated here because the RPMI
 * TEE service-group headers are not exported to OP-TEE core.
 */
#define RPMI_TEE_SRV_MEM_PARCEL_ACCEPT	0x0A
#define RPMI_TEE_SRV_MEM_PARCEL_RELEASE	0x0B
#define RPMI_TEE_ENDPOINT_REE		0
#define RPMI_TEE_ENDPOINT_OPTEE		1
#define RPMI_TEE_PARCEL_ACCESS_R	(1U << 29)
#define RPMI_TEE_PARCEL_ACCESS_W	(1U << 30)

/* Bound the block list / page count we are willing to map for one parcel. */
#define PARCEL_MAX_BLOCKS		64
#define PARCEL_MAX_PAGES		64

/* MEM_PARCEL_ACCEPT request (fixed header; no trailing other_id/other_access). */
struct parcel_accept_req {
	uint32_t acceptor_id;
	uint32_t access;
	uint32_t mem_parcel_id;
	uint32_t nonce;
	uint32_t creator_id;
	uint32_t creator_access;
	uint32_t flags;
	uint32_t address_high;
	uint32_t address_low;
	uint32_t max_pages;
	uint32_t other_cnt;
	uint32_t data[];
};

/* MEM_PARCEL_ACCEPT response: fixed header + block_high[bc] block_low[bc]. */
struct parcel_accept_resp {
	uint32_t status;
	uint32_t flags;
	uint32_t page_cnt;
	uint32_t block_cnt;
	uint32_t data[];
};

/* MEM_PARCEL_RELEASE request: header + endpoint_id[endpoint_cnt]. */
struct parcel_release_req {
	uint32_t mem_parcel_id;
	uint32_t flags;
	uint32_t endpoint_cnt;
	uint32_t endpoint_id[];
};

struct parcel_release_resp {
	uint32_t status;
};

/* Consume result codes returned to the REE in a0. */
#define PARCEL_CONSUME_OK		0
#define PARCEL_CONSUME_ERR_NO_CHANNEL	1
#define PARCEL_CONSUME_ERR_ACCEPT	2
#define PARCEL_CONSUME_ERR_BLOCKS	3
#define PARCEL_CONSUME_ERR_MAP		4
#define PARCEL_CONSUME_ERR_RELEASE	5

static int parcel_release(uint32_t channel_id, uint32_t parcel_id)
{
	uint8_t buf[sizeof(struct parcel_release_req) + sizeof(uint32_t)] = { };
	struct parcel_release_req *req = (void *)buf;
	struct parcel_release_resp resp = { };
	unsigned long resp_len = 0;
	int rc;

	req->mem_parcel_id = parcel_id;
	req->flags = 0;
	req->endpoint_cnt = 1;
	req->endpoint_id[0] = RPMI_TEE_ENDPOINT_OPTEE;

	rc = sbi_mpxy_send_message_with_response(channel_id,
						 RPMI_TEE_SRV_MEM_PARCEL_RELEASE,
						 buf, sizeof(buf),
						 &resp, sizeof(resp),
						 &resp_len);
	if (rc || resp.status)
		return -1;

	return 0;
}

/*
 * Accept the parcel, map its pages, write @pattern to the first word so the
 * REE can observe the TEE's access, read it back into *out, unmap, release.
 * Returns one of PARCEL_CONSUME_*.
 */
/*
 * Fast calls run on the small per-hart tmp stack (a few KiB), so the large
 * accept-response and page-list buffers are kept off the stack. The parcel
 * consume path is not concurrently re-entered in this configuration.
 */
static uint8_t respbuf[sizeof(struct parcel_accept_resp) +
		       2 * PARCEL_MAX_BLOCKS * sizeof(uint32_t)] __aligned(8);
static paddr_t pages[PARCEL_MAX_PAGES];

static uint32_t parcel_consume(uint32_t parcel_id, uint32_t nonce,
			       uint32_t creator_access, uint32_t pattern,
			       uint64_t *out)
{
	struct parcel_accept_resp *resp = (void *)respbuf;
	struct parcel_accept_req req = { };
	uint32_t channel_id = 0;
	unsigned long resp_len = 0;
	uint32_t access = RPMI_TEE_PARCEL_ACCESS_R | RPMI_TEE_PARCEL_ACCESS_W;
	struct mobj *mobj = NULL;
	size_t num_pages = 0;
	uint32_t bc, i, j;
	uint64_t *va;
	int rc;

	if (sbi_mpxy_rpmi_get_tee_channel_id(&channel_id))
		return PARCEL_CONSUME_ERR_NO_CHANNEL;

	req.acceptor_id = RPMI_TEE_ENDPOINT_OPTEE;
	req.access = access;
	req.mem_parcel_id = parcel_id;
	req.nonce = nonce;
	req.creator_id = RPMI_TEE_ENDPOINT_REE;
	req.creator_access = creator_access;
	req.flags = 0;
	req.max_pages = 0;
	req.other_cnt = 0;

	rc = sbi_mpxy_send_message_with_response(channel_id,
						 RPMI_TEE_SRV_MEM_PARCEL_ACCEPT,
						 &req, sizeof(req),
						 respbuf, sizeof(respbuf),
						 &resp_len);
	if (rc || resp->status) {
		EMSG("parcel accept failed: rc=%d status=%d", rc, resp->status);
		return PARCEL_CONSUME_ERR_ACCEPT;
	}

	bc = resp->block_cnt;
	if (!bc || bc > PARCEL_MAX_BLOCKS)
		return PARCEL_CONSUME_ERR_BLOCKS;

	/* Expand block list (page-unit encoding) into a flat page array. */
	for (i = 0; i < bc; i++) {
		uint32_t high = resp->data[i];
		uint32_t low = resp->data[bc + i];
		uint64_t page = ((uint64_t)high << 20) | (low >> 12);
		uint32_t count = (low & 0xFFF) + 1;

		for (j = 0; j < count; j++) {
			if (num_pages >= PARCEL_MAX_PAGES)
				return PARCEL_CONSUME_ERR_BLOCKS;
			pages[num_pages++] = (paddr_t)((page + j) << 12);
		}
	}

	/* Map the non-secure REE pages into OP-TEE core and touch them. */
	mobj = mobj_reg_shm_alloc(pages, num_pages, 0, parcel_id);
	if (!mobj) {
		parcel_release(channel_id, parcel_id);
		return PARCEL_CONSUME_ERR_MAP;
	}
	if (mobj_inc_map(mobj)) {
		mobj_put(mobj);
		parcel_release(channel_id, parcel_id);
		return PARCEL_CONSUME_ERR_MAP;
	}

	va = mobj_get_va(mobj, 0, num_pages * SMALL_PAGE_SIZE);
	if (!va) {
		mobj_dec_map(mobj);
		mobj_put(mobj);
		parcel_release(channel_id, parcel_id);
		return PARCEL_CONSUME_ERR_MAP;
	}

	*va = pattern;
	if (out)
		*out = *va;

	mobj_dec_map(mobj);
	mobj_put(mobj);

	if (parcel_release(channel_id, parcel_id))
		return PARCEL_CONSUME_ERR_RELEASE;

	return PARCEL_CONSUME_OK;
}

/*
 * Fast-ABI entry: TEE_CALL forwards the full a0-a7 register block to OP-TEE
 * (the TEE MPXY channel advertises a PAGE_SIZE message buffer, so all eight
 * registers survive the REE -> OpenSBI -> OP-TEE hop). The four 32-bit consume
 * arguments are nonetheless packed into a1 and a2 by choice, to keep the
 * request compact; spreading them across a1-a4 would work equally well:
 *   a1 = parcel_id       | (nonce   << 32)
 *   a2 = creator_access  | (pattern << 32)
 * The parcel's block list is not carried here; OP-TEE retrieves it directly via
 * MEM_PARCEL_ACCEPT on the TEE MPXY channel (PAGE_SIZE response buffer).
 * On return a0 carries a PARCEL_CONSUME_* status and a1 the value read back from
 * the mapped memory.
 */
void rpmi_tee_parcel_consume(struct thread_abi_args *args)
{
	uint32_t parcel_id = (uint32_t)args->a1;
	uint32_t nonce = (uint32_t)(args->a1 >> 32);
	uint32_t creator_access = (uint32_t)args->a2;
	uint32_t pattern = (uint32_t)(args->a2 >> 32);
	uint64_t out = 0;
	uint32_t st;

	st = parcel_consume(parcel_id, nonce, creator_access, pattern, &out);

	args->a0 = st;
	args->a1 = out;
	args->a2 = 0;
	args->a3 = 0;
}
