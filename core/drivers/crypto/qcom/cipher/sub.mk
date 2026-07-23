# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026, Qualcomm Technologies, Inc.

$(call force,CFG_CRYPTO_DRIVER,y,Mandated by CFG_QCOM_CE_CIPHER)
$(call force,CFG_CRYPTO_DRV_CIPHER,y,Mandated by CFG_QCOM_CE_CIPHER)

srcs-y += cipher.c
srcs-$(CFG_QCOM_CE_AES_ECB) += ecb.c
srcs-$(CFG_QCOM_CE_AES_CBC) += cbc.c
srcs-$(CFG_QCOM_CE_AES_XTS) += xts.c
