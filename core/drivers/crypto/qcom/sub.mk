# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026, Qualcomm Technologies, Inc.

incdirs-y += include
srcs-y += hwkm.c hwkm_huk.c hwkm_transaction.c

CFG_QCOM_CE_AES_ECB ?= n
CFG_QCOM_CE_AES_CBC ?= n
CFG_QCOM_CE_AES_XTS ?= n
CFG_QCOM_CE_AES_GCM ?= n

CFG_QCOM_CE_CIPHER := $(if $(filter y,$(CFG_QCOM_CE_AES_ECB) \
                                       $(CFG_QCOM_CE_AES_CBC) \
                                       $(CFG_QCOM_CE_AES_XTS)),y,n)
CFG_QCOM_CE_AEAD := $(if $(filter y,$(CFG_QCOM_CE_AES_GCM)),y,n)

subdirs-$(CFG_QCOM_CE_CIPHER) += ce cipher
subdirs-$(CFG_QCOM_CE_AEAD) += ce aead

# Bitmap of fuse regions whose SHA256 digest is folded into the HUK KDF input.
# Each bit corresponds to a fuse region index; set a bit to bind the HUK to
# that region's content. Defaults to 0 (no fuse regions included).
CFG_HWKM_HUK_FUSE_REGION_DIGEST ?= 0x0

# Mix TZ_SKDK_L2 into the HUK KDF via the MKS field. When enabled, an SKDK L3
# key is derived from TZ_SKDK_L2 and passed as the mixing key for the UKDK L3
# and L4 derivation steps, binding the HUK to the SKDK lineage in addition to
# the UKDK. Defaults to y.
CFG_HWKM_HUK_MIX_SKDK ?= y
