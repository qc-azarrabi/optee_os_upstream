// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 *
 * AES-GCM AEAD context using the CRYPTO0 CE hardware.
 */

#include <crypto/crypto_impl.h>
#include <io.h>
#include <malloc.h>
#include <string.h>
#include <string_ext.h>
#include <tee_api_types.h>
#include <utee_defines.h>

#include "../ce/ce.h"
#include "algorithms.h"

#define GCM_NONCE_LEN		12U
#define GCM_TAG_MIN		4U
#define GCM_TAG_MAX		16U

struct ce_gcm_ctx {
	struct crypto_authenc_ctx ctx;

	/* Key material. */
	uint8_t  key[32];
	size_t   key_len;

	/* Operation parameters fixed at gcm_init(). */
	bool encrypt;
	size_t tag_len;
	size_t aad_total;
	size_t payload_total;

	/*
	 * GCTR state. j0 is fixed for the lifetime of the operation.
	 * cipher_iv starts at J0+1 and advances after each payload segment.
	 */
	uint8_t j0[CE_AES_BLOCK_SIZE];
	uint8_t cipher_iv[CE_AES_BLOCK_SIZE];

	/*
	 * GHASH state saved in software between lock acquisitions so the
	 * hardware accumulator can be restored before each new segment.
	 */
	uint32_t auth_iv[CE_AUTH_IV_WORDS];
	uint32_t auth_byte_cnt[4];

	/* Progress tracking. */
	size_t aad_done;
	size_t payload_done;
	bool aad_flushed;
	bool payload_started;

	/* Partial AAD buffer; padded and flushed before the first payload. */
	uint8_t aad_pad[CE_AES_BLOCK_SIZE];
	size_t aad_pad_len;
};

static struct ce_gcm_ctx *to_gcm(struct crypto_authenc_ctx *ctx)
{
	return container_of(ctx, struct ce_gcm_ctx, ctx);
}

/* Restore HW context: */
static void gcm_hw_setup(struct ce_gcm_ctx *c, vaddr_t base)
{
	unsigned int i = 0;

	io_write32_off(base, CE_CONFIG, CE_CONFIG_DEFAULT);
	ce_load_key(base, c->key, c->key_len);
	ce_load_auth_key(base, c->key, c->key_len);

	/* Load J0 into ENCR_CCM_INIT_CNTR (hardware uses it to encrypt tag). */
	for (i = 0; i < 4; i++) {
		uint32_t w = 0;

		memcpy(&w, c->j0 + i * 4, 4);
		io_write32_off(base, CE_ENCR_CCM_INIT_CNTR(i), w);
	}

	ce_load_iv_gcm(base, c->cipher_iv);
	ce_load_auth_iv(base, c->auth_iv);
	ce_load_auth_byte_cnt(base, c->auth_byte_cnt);
}

static TEE_Result gcm_init(struct crypto_authenc_ctx *ctx,
			   TEE_OperationMode mode,
			   const uint8_t *key, size_t key_len,
			   const uint8_t *nonce, size_t nonce_len,
			   size_t tag_len, size_t aad_len,
			   size_t payload_len)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);

	if (key_len != 16 && key_len != 32)
		return TEE_ERROR_BAD_PARAMETERS;

	if (nonce_len != GCM_NONCE_LEN)
		return TEE_ERROR_NOT_SUPPORTED;

	if (tag_len < GCM_TAG_MIN || tag_len > GCM_TAG_MAX)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(c->key, key, key_len);
	c->key_len = key_len;
	c->encrypt = (mode == TEE_MODE_ENCRYPT);
	c->tag_len = tag_len;
	c->aad_total = aad_len;
	c->payload_total = payload_len;

	/* Construct J0: nonce || 0x000001 */
	memset(c->j0, 0, sizeof(c->j0));
	memcpy(c->j0, nonce, GCM_NONCE_LEN);
	c->j0[15] = 1;

	/* Reset GHASH state. */
	memset(c->auth_iv, 0, sizeof(c->auth_iv));
	memset(c->auth_byte_cnt, 0, sizeof(c->auth_byte_cnt));

	/* Reset GCTR counter to J0+1; updated after each payload segment. */
	memcpy(c->cipher_iv, c->j0, CE_AES_BLOCK_SIZE);
	c->cipher_iv[15]++;

	/* Progress tracking. */
	c->payload_started = false;
	c->aad_done = 0;
	c->payload_done = 0;
	c->aad_flushed = false;
	c->aad_pad_len = 0;

	return TEE_SUCCESS;
}

/* AAD. */

static TEE_Result gcm_flush_aad_block(struct ce_gcm_ctx *c, vaddr_t base,
				      const uint8_t *block, bool first)
{
	TEE_Result res = TEE_SUCCESS;
	uint32_t auth_cfg = 0;

	auth_cfg = ce_auth_cfg(CE_AUTH_MODE_GCM, c->key_len,
			       c->encrypt, first, false, c->tag_len);

	res = ce_aes_aead_auth(base, auth_cfg, block, CE_AES_BLOCK_SIZE);
	if (!res) {
		ce_read_auth_iv(base, c->auth_iv);
		ce_read_auth_byte_cnt(base, c->auth_byte_cnt);
	}

	return res;
}

/* Flush any remaining partial AAD, zero-padded to 16 bytes. */
static TEE_Result gcm_flush_remaining_aad(struct ce_gcm_ctx *c, vaddr_t base)
{
	TEE_Result res = TEE_SUCCESS;

	if (c->aad_flushed)
		return TEE_SUCCESS;

	if (c->aad_pad_len) {
		/* Zero-pad the partial block. */
		memset(c->aad_pad + c->aad_pad_len, 0,
		       CE_AES_BLOCK_SIZE - c->aad_pad_len);
		res = gcm_flush_aad_block(c, base, c->aad_pad,
					  c->aad_done == 0);
		if (res)
			return res;

		c->aad_done += c->aad_pad_len;
		c->aad_pad_len = 0;
	}

	c->aad_flushed = true;

	return TEE_SUCCESS;
}

static TEE_Result gcm_update_aad(struct crypto_authenc_ctx *ctx,
				 const uint8_t *data, size_t len)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);
	vaddr_t base = ce_get_base();
	TEE_Result res = TEE_SUCCESS;
	size_t off = 0;

	if (!len)
		return TEE_SUCCESS;

	ce_lock();
	gcm_hw_setup(c, base);
	/* Process buffered partial block + new data in 16-byte chunks. */
	while (off < len) {
		size_t copy = MIN(len - off,
				  CE_AES_BLOCK_SIZE - c->aad_pad_len);

		memcpy(c->aad_pad + c->aad_pad_len, data + off, copy);
		c->aad_pad_len += copy;
		off += copy;

		if (c->aad_pad_len == CE_AES_BLOCK_SIZE) {
			bool first = (c->aad_done == 0);

			res = gcm_flush_aad_block(c, base, c->aad_pad, first);
			if (res)
				goto out;

			c->aad_done += CE_AES_BLOCK_SIZE;
			c->aad_pad_len = 0;
		}
	}
out:
	ce_unlock();

	return res;
}

/* Ciphertext. */

static TEE_Result gcm_update_payload(struct crypto_authenc_ctx *ctx,
				     TEE_OperationMode mode __unused,
				     const uint8_t *src, size_t len,
				     uint8_t *dst)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);
	vaddr_t base = ce_get_base();
	uint32_t encr_cfg = 0;
	uint32_t auth_cfg = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!len || len % CE_AES_BLOCK_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	encr_cfg = ce_encr_cfg(CE_ENCR_MODE_GCM, c->key_len, c->encrypt);

	ce_lock();
	gcm_hw_setup(c, base);
	/* Flush remaining partial AAD. */
	res = gcm_flush_remaining_aad(c, base);
	if (res)
		goto out;

	/*
	 * Compute first AFTER flushing AAD - if a partial AAD block was just
	 * sent as the first segment, this payload xfer must have first = false.
	 */
	auth_cfg = ce_auth_cfg(CE_AUTH_MODE_GCM, c->key_len, c->encrypt,
			       !c->aad_done && !c->payload_done, false,
			       c->tag_len);

	res = ce_aes_aead_xfer(base, encr_cfg, auth_cfg, src, dst, len);
	if (!res) {
		ce_read_auth_iv(base, c->auth_iv);
		ce_read_auth_byte_cnt(base, c->auth_byte_cnt);
		ce_read_cipher_iv(base, c->cipher_iv);

		c->payload_started = true;
		c->payload_done += len;
	}
out:
	ce_unlock();

	return res;
}

/* Process the final payload chunk (may be zero bytes). */
static TEE_Result gcm_do_final(struct ce_gcm_ctx *c, const uint8_t *src,
			       size_t len, uint8_t *dst, uint8_t *tag_out)
{
	vaddr_t base = ce_get_base();
	uint32_t encr_cfg = 0;
	uint32_t auth_cfg = 0;
	uint32_t info_nonce[4] = { };
	TEE_Result res = TEE_SUCCESS;
	bool first = false;

	encr_cfg = ce_encr_cfg_last(CE_ENCR_MODE_GCM, c->key_len, c->encrypt);

	ce_lock();
	gcm_hw_setup(c, base);
	/* Flush remaining partial AAD; LAST is carried by the xfer below. */
	res = gcm_flush_remaining_aad(c, base);
	if (res)
		goto out;

	/*
	 * Compute first AFTER flushing AAD so that if a partial AAD block
	 * was just sent as the first segment, first = false for payload xfer.
	 * first = true only when no segment has been sent to hardware at all.
	 */
	first = !c->aad_done && !c->payload_done;
	auth_cfg = ce_auth_cfg(CE_AUTH_MODE_GCM, c->key_len, c->encrypt,
			       first, true, c->tag_len);

	/*
	 * AUTH_INFO_NONCEn = {len(A) || len(C)} in bits, big-endian.
	 * put_be64 writes big-endian; LITTLE_ENDIAN_MODE hardware reversal
	 * produces the correct network-byte-order value.
	 */
	put_be64(&info_nonce[0], (uint64_t)c->aad_done * 8);
	put_be64(&info_nonce[2], (uint64_t)(c->payload_done + len) * 8);
	ce_load_info_nonce(base, info_nonce);

	res = ce_aes_aead_xfer(base, encr_cfg, auth_cfg, src, dst, len);
	if (res)
		goto out;

	/* Tag is the first tag_len bytes of AUTH_IVn. */
	ce_read_auth_iv(base, c->auth_iv);
	memcpy(tag_out, c->auth_iv, c->tag_len);
out:
	ce_unlock();

	return res;
}

static TEE_Result gcm_enc_final(struct crypto_authenc_ctx *ctx,
				const uint8_t *src, size_t len,
				uint8_t *dst, uint8_t *tag,
				size_t *tag_len)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);
	uint8_t computed_tag[GCM_TAG_MAX] = { };
	TEE_Result res = TEE_SUCCESS;

	res = gcm_do_final(c, src, len, dst, computed_tag);
	if (res)
		return res;

	if (*tag_len < c->tag_len)
		return TEE_ERROR_SHORT_BUFFER;

	memcpy(tag, computed_tag, c->tag_len);
	*tag_len = c->tag_len;

	return TEE_SUCCESS;
}

static TEE_Result gcm_dec_final(struct crypto_authenc_ctx *ctx,
				const uint8_t *src, size_t len,
				uint8_t *dst, const uint8_t *tag,
				size_t tag_len)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);
	uint8_t computed_tag[GCM_TAG_MAX] = { };
	TEE_Result res = TEE_SUCCESS;

	res = gcm_do_final(c, src, len, dst, computed_tag);
	if (res)
		return res;

	if (tag_len != c->tag_len)
		return TEE_ERROR_MAC_INVALID;

	if (consttime_memcmp(computed_tag, tag, c->tag_len))
		return TEE_ERROR_MAC_INVALID;

	return TEE_SUCCESS;
}

static void gcm_final(struct crypto_authenc_ctx *ctx)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);

	memzero_explicit(c->key, sizeof(c->key));
	memzero_explicit(c->j0, sizeof(c->j0));
	memzero_explicit(c->cipher_iv, sizeof(c->cipher_iv));
	memzero_explicit(c->auth_iv, sizeof(c->auth_iv));
	memzero_explicit(c->aad_pad, sizeof(c->aad_pad));
}

static void gcm_free_ctx(struct crypto_authenc_ctx *ctx)
{
	struct ce_gcm_ctx *c = to_gcm(ctx);

	memzero_explicit(c->key, sizeof(c->key));
	memzero_explicit(c->j0, sizeof(c->j0));
	memzero_explicit(c->cipher_iv, sizeof(c->cipher_iv));
	memzero_explicit(c->auth_iv, sizeof(c->auth_iv));
	memzero_explicit(c->aad_pad, sizeof(c->aad_pad));
	free(c);
}

static void gcm_copy_state(struct crypto_authenc_ctx *dst_ctx,
			   struct crypto_authenc_ctx *src_ctx)
{
	memcpy(to_gcm(dst_ctx), to_gcm(src_ctx), sizeof(struct ce_gcm_ctx));
}

static const struct crypto_authenc_ops gcm_ops = {
	.init = gcm_init,
	.update_aad = gcm_update_aad,
	.update_payload = gcm_update_payload,
	.enc_final = gcm_enc_final,
	.dec_final = gcm_dec_final,
	.final = gcm_final,
	.free_ctx = gcm_free_ctx,
	.copy_state = gcm_copy_state,
};

TEE_Result ce_aes_gcm_allocate(void **ctx)
{
	struct ce_gcm_ctx *c = calloc(1, sizeof(*c));

	if (!c)
		return TEE_ERROR_OUT_OF_MEMORY;

	c->ctx.ops = &gcm_ops;
	*ctx = &c->ctx;

	return TEE_SUCCESS;
}
