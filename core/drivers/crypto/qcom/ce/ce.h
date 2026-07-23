/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc.
 *
 * Shared CRYPTO0 CE block cipher and AEAD helpers.
 */

#ifndef CE_H
#define CE_H

#include <tee_api_types.h>
#include <types_ext.h>

#include "ce_regs.h"

/* ce_get_base() - Return the virtual base address of CRYPTO_REG. */
vaddr_t ce_get_base(void);

/*
 * ce_lock() / ce_unlock() - Serialize access to the shared CRYPTO0 CE block.
 * Callers must hold this lock for the full register-program + transfer
 * sequence of each update() call.
 */
void ce_lock(void);
void ce_unlock(void);

/* ce_load_key() - Write AES key into ENCR_KEYn hardware registers. */
void ce_load_key(vaddr_t base, const uint8_t *key, size_t key_len);
/* ce_load_xts_key() - Write XTS tweak key into ENCR_XTS_KEYn registers. */
void ce_load_xts_key(vaddr_t base, const uint8_t *key, size_t key_len);
/* ce_load_auth_key() - Write auth key into AUTH_KEYn registers (GCM/CCM). */
void ce_load_auth_key(vaddr_t base, const uint8_t *key, size_t key_len);
/* ce_load_iv() - Write IV/counter into ENCR_CNTR_IVn + ENCR_CNTR_MASKn. */
void ce_load_iv(vaddr_t base, const uint8_t iv[CE_AES_BLOCK_SIZE]);
/* ce_load_iv_gcm() - Like ce_load_iv() but sets only mask3=0xFFFFFFFF,
 * masks 0–2=0. GCM uses inc32(J0): only the least-significant 32 bits
 * (word 3) increment; the upper 96 bits (nonce) must not change.
 */
void ce_load_iv_gcm(vaddr_t base, const uint8_t iv[CE_AES_BLOCK_SIZE]);
/* ce_read_cipher_iv() - Read current counter from ENCR_CNTR_IVn. */
void ce_read_cipher_iv(vaddr_t base, uint8_t iv[CE_AES_BLOCK_SIZE]);
/* ce_load_auth_iv() - Write GHASH accumulator into AUTH_IVn[0..15]. */
void ce_load_auth_iv(vaddr_t base, const uint32_t auth_iv[CE_AUTH_IV_WORDS]);
/* ce_read_auth_iv() - Read GHASH accumulator from AUTH_IVn[0..15]. */
void ce_read_auth_iv(vaddr_t base, uint32_t auth_iv[CE_AUTH_IV_WORDS]);
/* ce_load_auth_byte_cnt() - Write bit counters into AUTH_BYTECNTn[0..3]. */
void ce_load_auth_byte_cnt(vaddr_t base, const uint32_t cnt[4]);
/* ce_read_auth_byte_cnt() - Read bit counters from AUTH_BYTECNTn[0..3]. */
void ce_read_auth_byte_cnt(vaddr_t base, uint32_t cnt[4]);
/* ce_load_info_nonce() - Write GCM length block into AUTH_INFO_NONCEn[0..3]. */
void ce_load_info_nonce(vaddr_t base, const uint32_t nonce[4]);

/* ce_encr_cfg() - Build ENCR_SEG_CFG for @mode / @key_len / @encrypt. */
uint32_t ce_encr_cfg(uint32_t mode, size_t key_len, bool encrypt);

/* ce_encr_cfg_last() - Like ce_encr_cfg() but sets the LAST bit. */
uint32_t ce_encr_cfg_last(uint32_t mode, size_t key_len, bool encrypt);

/*
 * ce_auth_cfg() - Build AUTH_SEG_CFG for GCM/CCM.
 * @tag_len: authentication tag length in bytes (e.g. 16); sets AUTH_SIZE
 *           field to (tag_len - 1) as required by hardware.
 */
uint32_t ce_auth_cfg(uint32_t mode, size_t key_len, bool encrypt,
		     bool first, bool last, size_t tag_len);

/*
 * ce_aes_xfer() - Encrypt or decrypt @len bytes (cipher-only).
 * Caller must load IV/key before and pass @iv_out to read back the updated IV
 * (CBC: last ciphertext block; XTS: final tweak; ECB: pass NULL).
 */
TEE_Result ce_aes_xfer(vaddr_t base, uint32_t encr_cfg,
		       const uint8_t *in, uint8_t *out, size_t len,
		       uint8_t iv_out[CE_AES_BLOCK_SIZE]);

/*
 * ce_aes_aead_auth() - Feed AAD through the auth engine; cipher engine idle.
 * Caller must load AUTH_IVn/AUTH_BYTECNTn before and read them back after.
 */
TEE_Result ce_aes_aead_auth(vaddr_t base, uint32_t auth_cfg,
			    const uint8_t *aad, size_t aad_len);

/*
 * ce_aes_aead_xfer() - Encrypt/decrypt @len bytes with simultaneous auth.
 * Caller must load AUTH_IVn/AUTH_BYTECNTn before and read them back after.
 */
TEE_Result ce_aes_aead_xfer(vaddr_t base, uint32_t encr_cfg,
			    uint32_t auth_cfg,
			    const uint8_t *in, uint8_t *out,
			    size_t len);

#endif /* CE_H */
