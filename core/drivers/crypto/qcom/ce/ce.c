// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 *
 * Shared CRYPTO0 CE block cipher helpers.
 */

#include <arm.h>
#include <assert.h>
#include <hwkm.h>
#include <io.h>
#include <kernel/mutex.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <string.h>
#include <trace.h>
#include <util.h>

#include "ce.h"

/* Serializes all access to the shared CRYPTO0 CE cipher block. */
static struct mutex ce_mu = MUTEX_INITIALIZER;

void ce_lock(void)
{
	mutex_lock(&ce_mu);
}

void ce_unlock(void)
{
	mutex_unlock(&ce_mu);
}

vaddr_t ce_get_base(void)
{
	struct hwkm_drv_ctx *ctx = hwkm_get_context();

	assert(ctx && ctx->crypto0_base);
	return ctx->crypto0_base + CE_REG_OFFSET;
}

void ce_load_key(vaddr_t base, const uint8_t *key, size_t key_len)
{
	unsigned int i = 0;
	uint32_t w = 0;

	for (i = 0; i < CE_MAX_KEY_WORDS; i++) {
		if (i < key_len / 4)
			memcpy(&w, key + i * 4, 4);
		else
			w = 0;

		io_write32_off(base, CE_ENCR_KEY(i), w);
	}
}

void ce_load_xts_key(vaddr_t base, const uint8_t *key, size_t key_len)
{
	unsigned int i = 0;
	uint32_t w = 0;

	for (i = 0; i < CE_MAX_KEY_WORDS; i++) {
		if (i < key_len / 4)
			memcpy(&w, key + i * 4, 4);
		else
			w = 0;

		io_write32_off(base, CE_ENCR_XTS_KEY(i), w);
	}
}

void ce_load_iv(vaddr_t base, const uint8_t iv[CE_AES_BLOCK_SIZE])
{
	static const uint32_t iv_regs[] = {
		CE_ENCR_IV0, CE_ENCR_IV1,
		CE_ENCR_IV2, CE_ENCR_IV3,
	};
	static const uint32_t mask_regs[] = {
		CE_ENCR_CNTR_MASK0, CE_ENCR_CNTR_MASK1,
		CE_ENCR_CNTR_MASK2, CE_ENCR_CNTR_MASK3,
	};
	uint32_t w = 0;
	unsigned int i = 0;

	for (i = 0; i < 4; i++) {
		memcpy(&w, iv + i * 4, 4);
		io_write32_off(base, iv_regs[i], w);
		io_write32_off(base, mask_regs[i], 0xffffffffU);
	}
}

void ce_load_iv_gcm(vaddr_t base, const uint8_t iv[CE_AES_BLOCK_SIZE])
{
	uint32_t w = 0;
	unsigned int i = 0;

	static const uint32_t iv_regs[] = {
		CE_ENCR_IV0, CE_ENCR_IV1,
		CE_ENCR_IV2, CE_ENCR_IV3,
	};
	static const uint32_t mask_regs[] = {
		CE_ENCR_CNTR_MASK0, CE_ENCR_CNTR_MASK1,
		CE_ENCR_CNTR_MASK2, CE_ENCR_CNTR_MASK3,
	};
	static const uint32_t masks[] = { 0, 0, 0, 0xffffffffU };

	for (i = 0; i < 4; i++) {
		memcpy(&w, iv + i * 4, 4);
		io_write32_off(base, iv_regs[i], w);
		io_write32_off(base, mask_regs[i], masks[i]);
	}
}

static void ce_read_iv(vaddr_t base, uint8_t iv[CE_AES_BLOCK_SIZE])
{
	static const uint32_t iv_regs[] = {
		CE_ENCR_IV0, CE_ENCR_IV1,
		CE_ENCR_IV2, CE_ENCR_IV3,
	};
	uint32_t w = 0;
	unsigned int i = 0;

	for (i = 0; i < 4; i++) {
		w = io_read32_off(base, iv_regs[i]);
		memcpy(iv + i * 4, &w, 4);
	}
}

/* Read current ENCR_CNTR_IVn counter back into a byte buffer. */
void ce_read_cipher_iv(vaddr_t base, uint8_t iv[CE_AES_BLOCK_SIZE])
{
	ce_read_iv(base, iv);
}

/* ce_get_state() - post-completion status check. */
static TEE_Result ce_get_state(vaddr_t base)
{
	uint32_t s = io_read32_off(base, CE_STATUS);
	uint32_t s2 = io_read32_off(base, CE_STATUS2);
	TEE_Result res = TEE_SUCCESS;

	if (s & CE_STATUS_MAC_FAILED) {
		res = TEE_ERROR_MAC_INVALID;
	} else if (s & (CE_STATUS_ERR_INTR | CE_STATUS_SW_ERR)) {
		EMSG("qcom_ce: status error 0x%08" PRIx32, s);
		res = TEE_ERROR_GENERIC;
	} else if (s & CE_STATUS_HSD_ERR) {
		EMSG("qcom_ce: HSD error 0x%08" PRIx32, s);
		res = TEE_ERROR_GENERIC;
	} else if (!(s & CE_STATUS_OPERATION_DONE) ||
		   (s & CE_STATUS_ENCR_BUSY) ||
		   (s & CE_STATUS_AUTH_BUSY)) {
		EMSG("qcom_ce: engine still busy 0x%08" PRIx32, s);
		res = TEE_ERROR_BUSY;
	}

	if (s2 & CE_STATUS2_KEY_ERR) {
		EMSG("qcom_ce: key error");
		res = TEE_ERROR_BAD_STATE;
	}

	return res;
}

uint32_t ce_encr_cfg(uint32_t mode, size_t key_len, bool encrypt)
{
	uint32_t cfg = 0;

	cfg  = (CE_ENCR_ALG_AES << CE_ENCR_SEG_CFG_ALG_SHIFT) &
	       CE_ENCR_SEG_CFG_ALG_MASK;

	cfg |= (mode << CE_ENCR_SEG_CFG_MODE_SHIFT) &
	       CE_ENCR_SEG_CFG_MODE_MASK;

	cfg |= ((key_len == 32 ? CE_KEY_SZ_AES256 : CE_KEY_SZ_AES128)
		<< CE_ENCR_SEG_CFG_KEY_SZ_SHIFT) &
	       CE_ENCR_SEG_CFG_KEY_SZ_MASK;

	if (encrypt)
		cfg |= CE_ENCR_SEG_CFG_ENCODE;

	return cfg;
}

uint32_t ce_encr_cfg_last(uint32_t mode, size_t key_len, bool encrypt)
{
	return ce_encr_cfg(mode, key_len, encrypt) | CE_ENCR_SEG_CFG_LAST;
}

uint32_t ce_auth_cfg(uint32_t mode, size_t key_len, bool encrypt,
		     bool first, bool last, size_t tag_len)
{
	uint32_t cfg = 0;

	cfg = (CE_AUTH_ALG_AES << CE_AUTH_SEG_CFG_ALG_SHIFT) &
	      CE_AUTH_SEG_CFG_ALG_MASK;

	cfg |= (mode << CE_AUTH_SEG_CFG_MODE_SHIFT) &
	       CE_AUTH_SEG_CFG_MODE_MASK;

	cfg |= ((key_len == 32 ? CE_KEY_SZ_AES256 : CE_KEY_SZ_AES128)
		<< CE_AUTH_SEG_CFG_KEY_SZ_SHIFT) &
	       CE_AUTH_SEG_CFG_KEY_SZ_MASK;

	/* AUTH_SIZE encodes tag length in bytes minus 1. */
	if (tag_len)
		cfg |= (((tag_len - 1) << CE_AUTH_SEG_CFG_SIZE_SHIFT) &
			CE_AUTH_SEG_CFG_SIZE_MASK);

	/* GCM: auth after cipher on encrypt, before cipher on decrypt. */
	if (encrypt)
		cfg |= CE_AUTH_SEG_CFG_POS_AFTER;

	if (first)
		cfg |= CE_AUTH_SEG_CFG_FIRST;

	if (last)
		cfg |= CE_AUTH_SEG_CFG_LAST;

	return cfg;
}

void ce_load_auth_key(vaddr_t base, const uint8_t *key, size_t key_len)
{
	unsigned int i = 0;
	uint32_t w = 0;

	for (i = 0; i < 16; i++) {
		if (i < key_len / 4)
			memcpy(&w, key + i * 4, 4);
		else
			w = 0;
		io_write32_off(base, CE_AUTH_KEY(i), w);
	}
}

void ce_load_auth_iv(vaddr_t base, const uint32_t auth_iv[CE_AUTH_IV_WORDS])
{
	unsigned int i = 0;

	for (i = 0; i < CE_AUTH_IV_WORDS; i++)
		io_write32_off(base, CE_AUTH_IV(i), auth_iv[i]);
}

void ce_read_auth_iv(vaddr_t base, uint32_t auth_iv[CE_AUTH_IV_WORDS])
{
	unsigned int i = 0;

	for (i = 0; i < CE_AUTH_IV_WORDS; i++)
		auth_iv[i] = io_read32_off(base, CE_AUTH_IV(i));
}

void ce_load_auth_byte_cnt(vaddr_t base, const uint32_t cnt[4])
{
	io_write32_off(base, CE_AUTH_BYTECNT0, cnt[0]);
	io_write32_off(base, CE_AUTH_BYTECNT1, cnt[1]);
	io_write32_off(base, CE_AUTH_BYTECNT2, cnt[2]);
	io_write32_off(base, CE_AUTH_BYTECNT3, cnt[3]);
}

void ce_read_auth_byte_cnt(vaddr_t base, uint32_t cnt[4])
{
	cnt[0] = io_read32_off(base, CE_AUTH_BYTECNT0);
	cnt[1] = io_read32_off(base, CE_AUTH_BYTECNT1);
	cnt[2] = io_read32_off(base, CE_AUTH_BYTECNT2);
	cnt[3] = io_read32_off(base, CE_AUTH_BYTECNT3);
}

void ce_load_info_nonce(vaddr_t base, const uint32_t nonce[4])
{
	unsigned int i = 0;

	for (i = 0; i < 4; i++)
		io_write32_off(base, CE_AUTH_INFO_NONCE(i), nonce[i]);
}

/* aead_transfer() - internal engine for AEAD segments. */
static TEE_Result aead_transfer(vaddr_t base, uint32_t encr_cfg,
				uint32_t auth_cfg,
				const uint8_t *in, uint8_t *out,
				size_t auth_start, size_t auth_len,
				size_t cipher_start, size_t cipher_len)
{
	size_t seg_size = MAX(auth_start + auth_len,
			      cipher_start + cipher_len);

	unsigned int din_words = (cipher_len ? DIV_ROUND_UP(cipher_len, 4)
					     : DIV_ROUND_UP(auth_len, 4));
	unsigned int dout_words = cipher_len ? DIV_ROUND_UP(cipher_len, 4) : 0;
	unsigned int din_done = 0;
	unsigned int dout_done = 0;
	uint32_t status = 0;
	uint32_t w = 0;

	io_write32_off(base, CE_AUTH_SEG_SIZE, auth_len);
	io_write32_off(base, CE_AUTH_SEG_START, auth_start);
	io_write32_off(base, CE_SEG_SIZE, seg_size);
	io_write32_off(base, CE_AUTH_SEG_CFG, auth_cfg);
	io_write32_off(base, CE_ENCR_SEG_SIZE, cipher_len);
	io_write32_off(base, CE_ENCR_SEG_START, cipher_start);
	io_write32_off(base, CE_SEG_SIZE, seg_size);
	io_write32_off(base, CE_ENCR_SEG_CFG, encr_cfg);

	io_write32_off(base, CE_GOPROC, CE_GOPROC_GO);

	/* ISB + dummy STATUS read. */
	isb();
	(void)io_read32_off(base, CE_STATUS);

	/*
	 * DIN/DOUT loop. AAD-only: dout_words=0, DIN carries auth data.
	 * DOUT may stall - flush when DIN_RDY but no DOUT reads expected.
	 */
	while (din_done < din_words || dout_done < dout_words) {
		uint32_t avail = 0;

		status = io_read32_off(base, CE_STATUS);

		if ((status & CE_STATUS_DIN_RDY) && din_done < din_words) {
			uint32_t s2 = io_read32_off(base, CE_STATUS);

			avail = (s2 & CE_STATUS_DIN_SIZE_AVAIL) >>
				CE_STATUS_DIN_SIZE_AVAIL_SHIFT;
			avail /= 4;
			while (avail && din_done < din_words) {
				memcpy(&w, in + din_done * 4, 4);
				io_write32_off(base,
					       CE_DATA_IN(din_done % 4), w);
				din_done++;
				avail--;
			}
		} else if ((status & CE_STATUS_DIN_RDY) && !dout_words) {
			/* AAD-only: flush stale DOUT data to unblock DIN. */
			if (status & CE_STATUS_DOUT_RDY) {
				uint32_t s2 = io_read32_off(base, CE_STATUS);

				avail = (s2 & CE_STATUS_DOUT_SIZE_AVAIL) >>
					CE_STATUS_DOUT_SIZE_AVAIL_SHIFT;
				avail /= 4;
				while (avail--) {
					(void)io_read32_off(base,
						CE_DATA_OUT(0));
				}
			}
		}

		if ((status & CE_STATUS_DOUT_RDY) && dout_done < dout_words) {
			uint32_t s2 = io_read32_off(base, CE_STATUS);
			uint32_t off = 0;

			avail = (s2 & CE_STATUS_DOUT_SIZE_AVAIL) >>
				CE_STATUS_DOUT_SIZE_AVAIL_SHIFT;
			avail /= 4;
			while (avail && dout_done < dout_words) {
				off = CE_DATA_OUT(dout_done % 4);
				w = io_read32_off(base, off);
				memcpy(out + dout_done * 4, &w, 4);
				dout_done++;
				avail--;
			}
		}
	}

	/*
	 * Auth engine requires OPERATION_DONE poll - unlike cipher-only
	 * where DOUT exhaustion is sufficient.
	 */
	while (!(io_read32_off(base, CE_STATUS) & CE_STATUS_OPERATION_DONE))
		;

	return ce_get_state(base);
}

/*
 * ce_aes_aead_auth() - Feed one AAD block through the auth engine only.
 * Caller must load AUTH_IVn/AUTH_BYTECNTn before and read them back after.
 */
TEE_Result ce_aes_aead_auth(vaddr_t base, uint32_t auth_cfg,
			    const uint8_t *aad, size_t aad_len)
{
	return aead_transfer(base, 0, auth_cfg, aad, NULL,
			     0, aad_len, aad_len, 0);
}

/*
 * ce_aes_aead_xfer() - Encrypt/decrypt @len bytes with simultaneous auth.
 * Caller must load AUTH_IVn/AUTH_BYTECNTn before and read them back after.
 */
TEE_Result ce_aes_aead_xfer(vaddr_t base, uint32_t encr_cfg,
			    uint32_t auth_cfg,
			    const uint8_t *in, uint8_t *out,
			    size_t len)
{
	return aead_transfer(base, encr_cfg, auth_cfg, in, out,
			     0, len, 0, len);
}

/*
 * ce_aes_xfer() - Encrypt or decrypt @len bytes (cipher-only).
 * Caller must load IV/key before and pass @iv_out to read back the updated IV
 * (CBC: last ciphertext block; XTS: final tweak; ECB: pass NULL).
 */
TEE_Result ce_aes_xfer(vaddr_t base, uint32_t encr_cfg,
		       const uint8_t *in, uint8_t *out, size_t len,
		       uint8_t iv_out[CE_AES_BLOCK_SIZE])
{
	uint32_t status = 0;
	uint32_t w = 0;
	unsigned int din_words = DIV_ROUND_UP(len, 4);
	unsigned int dout_words = DIV_ROUND_UP(len, 4);
	unsigned int din_done = 0;
	unsigned int dout_done = 0;
	TEE_Result res = TEE_SUCCESS;

	io_write32_off(base, CE_AUTH_SEG_CFG, 0);
	io_write32_off(base, CE_ENCR_SEG_SIZE, len);
	io_write32_off(base, CE_ENCR_SEG_START, 0);
	io_write32_off(base, CE_SEG_SIZE, len);
	io_write32_off(base, CE_ENCR_SEG_CFG, encr_cfg);

	io_write32_off(base, CE_GOPROC, CE_GOPROC_GO);

	/* ISB + dummy STATUS read. */
	isb();
	(void)io_read32_off(base, CE_STATUS);

	/* Interleaved DIN-write / DOUT-read loop. */
	while (din_done < din_words || dout_done < dout_words) {
		uint32_t din_avail = 0;
		uint32_t dout_avail = 0;

		status = io_read32_off(base, CE_STATUS);

		/* Write as many DIN words as the FIFO has space for. */
		if ((status & CE_STATUS_DIN_RDY) &&
		    din_done < din_words) {
			uint32_t s2 = io_read32_off(base, CE_STATUS);

			din_avail = (s2 & CE_STATUS_DIN_SIZE_AVAIL) >>
				    CE_STATUS_DIN_SIZE_AVAIL_SHIFT;
			din_avail /= 4; /* bytes -> words */
			while (din_avail && din_done < din_words) {
				memcpy(&w, in + din_done * 4, 4);
				io_write32_off(base,
					       CE_DATA_IN(din_done % 4),
					       w);
				din_done++;
				din_avail--;
			}
		}

		/* Read as many DOUT words as the FIFO contains. */
		if ((status & CE_STATUS_DOUT_RDY) &&
		    dout_done < dout_words) {
			uint32_t s2 = io_read32_off(base, CE_STATUS);

			dout_avail = (s2 & CE_STATUS_DOUT_SIZE_AVAIL) >>
				     CE_STATUS_DOUT_SIZE_AVAIL_SHIFT;
			dout_avail /= 4; /* bytes -> words */
			while (dout_avail && dout_done < dout_words) {
				uint32_t off = CE_DATA_OUT(dout_done % 4);

				w = io_read32_off(base, off);
				memcpy(out + dout_done * 4, &w, 4);
				dout_done++;
				dout_avail--;
			}
		}
	}

	/* Post-completion status check. */
	res = ce_get_state(base);
	if (res)
		return res;

	/* Read back the hardware-updated IV/tweak. */
	if (iv_out) {
		ce_read_iv(base, iv_out);
	} else {
		uint8_t discard[CE_AES_BLOCK_SIZE] = { };

		ce_read_iv(base, discard);
	}

	return TEE_SUCCESS;
}
