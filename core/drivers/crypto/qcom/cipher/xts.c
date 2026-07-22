// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 *
 * AES-XTS cipher context.
 */

#include <crypto/crypto_impl.h>
#include <io.h>
#include <malloc.h>
#include <string.h>
#include <string_ext.h>
#include <utee_defines.h>

#include "../ce/ce.h"
#include "algorithms.h"

struct ce_xts_ctx {
	struct crypto_cipher_ctx ctx;
	uint8_t key[32];
	uint8_t xts_key[32];
	size_t key_len;
	bool encrypt;
	uint8_t tweak[CE_AES_BLOCK_SIZE];
};

static struct ce_xts_ctx *to_xts(struct crypto_cipher_ctx *ctx)
{
	return container_of(ctx, struct ce_xts_ctx, ctx);
}

static TEE_Result xts_init(struct crypto_cipher_ctx *ctx,
			   TEE_OperationMode mode,
			   const uint8_t *key1, size_t key1_len,
			   const uint8_t *key2, size_t key2_len,
			   const uint8_t *iv, size_t iv_len)
{
	struct ce_xts_ctx *c = to_xts(ctx);

	if ((key1_len != 16 && key1_len != 32) || key2_len != key1_len)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!key2)
		return TEE_ERROR_BAD_PARAMETERS;

	if (iv && iv_len != CE_AES_BLOCK_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(c->key, key1, key1_len);
	memcpy(c->xts_key, key2, key1_len);
	c->key_len = key1_len;
	c->encrypt = (mode == TEE_MODE_ENCRYPT);
	if (iv)
		memcpy(c->tweak, iv, CE_AES_BLOCK_SIZE);
	else
		memset(c->tweak, 0, CE_AES_BLOCK_SIZE);

	return TEE_SUCCESS;
}

static TEE_Result xts_update(struct crypto_cipher_ctx *ctx,
			     bool last_block __unused,
			     const uint8_t *data, size_t len, uint8_t *dst)
{
	struct ce_xts_ctx *c = to_xts(ctx);
	vaddr_t base = ce_get_base();
	uint32_t encr_cfg = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!len || len % CE_AES_BLOCK_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	if (len >= CE_ENCR_XTS_DU_SIZE_MAX)
		return TEE_ERROR_BAD_PARAMETERS;

	encr_cfg = ce_encr_cfg(CE_ENCR_MODE_XTS, c->key_len, c->encrypt);

	ce_lock();

	io_write32_off(base, CE_CONFIG, CE_CONFIG_DEFAULT);
	ce_load_key(base, c->key, c->key_len);
	ce_load_xts_key(base, c->xts_key, c->key_len);
	io_write32_off(base, CE_ENCR_XTS_DU_SIZE, len);
	ce_load_iv(base, c->tweak);

	res = ce_aes_xfer(base, encr_cfg, data, dst, len, c->tweak);

	ce_unlock();

	return res;
}

static void xts_final(struct crypto_cipher_ctx *ctx)
{
	struct ce_xts_ctx *c = to_xts(ctx);

	memzero_explicit(c->key, sizeof(c->key));
	memzero_explicit(c->xts_key, sizeof(c->xts_key));
	memzero_explicit(c->tweak, sizeof(c->tweak));
}

static void xts_free(struct crypto_cipher_ctx *ctx)
{
	struct ce_xts_ctx *c = to_xts(ctx);

	memzero_explicit(c->key, sizeof(c->key));
	memzero_explicit(c->xts_key, sizeof(c->xts_key));
	memzero_explicit(c->tweak, sizeof(c->tweak));
	free(c);
}

static void xts_copy_state(struct crypto_cipher_ctx *dst_ctx,
			   struct crypto_cipher_ctx *src_ctx)
{
	memcpy(to_xts(dst_ctx), to_xts(src_ctx),
	       sizeof(struct ce_xts_ctx));
}

static struct crypto_cipher_ops xts_ops = {
	.init = xts_init,
	.update = xts_update,
	.final = xts_final,
	.free_ctx = xts_free,
	.copy_state = xts_copy_state,
};

TEE_Result ce_aes_xts_allocate(void **ctx)
{
	struct ce_xts_ctx *c = calloc(1, sizeof(*c));

	if (!c)
		return TEE_ERROR_OUT_OF_MEMORY;

	c->ctx.ops = &xts_ops;
	*ctx = &c->ctx;

	return TEE_SUCCESS;
}
