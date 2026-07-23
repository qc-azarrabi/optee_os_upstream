# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026, Qualcomm Technologies, Inc.

$(call force,CFG_CRYPTO_DRIVER,y,Mandated by CFG_QCOM_CE_AEAD)
$(call force,CFG_CRYPTO_DRV_AUTHENC,y,Mandated by CFG_QCOM_CE_AEAD)

srcs-y += authenc.c
srcs-$(CFG_QCOM_CE_AES_GCM) += gcm.c
