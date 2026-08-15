/*******************************************************************************
 * Copyright 2026 RFIM Space.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software - OrbSight-2
 *
 */

#ifndef HSS_BOOT_PARAMS_H
#define HSS_BOOT_PARAMS_H

/*!
 * \file Boot Parameters driven boot API
 * \brief Bootloader 0 boot source selection from the Boot Parameters store
 *
 * HSS_BootParamsBootInit() replaces the plain HSS_BootInit() call at the
 * automatic boot points when CONFIG_BOOT_PARAMS is enabled. It walks the
 * boot_sequence[] of the shared Boot Parameters store (BSP_Boot_Params,
 * written by the application over the golden SW QSPI Flash), attempting
 * each boot source in order until one provides a valid boot image:
 *
 *   - BOOT_SRC_PRIMARY   (10)  MSS eMMC
 *   - BOOT_SRC_SECONDARY (20)  Fabric eMMC, via the CoreMMC service
 *   - BOOT_SRC_GOLDEN    (255) golden SW QSPI NOR Flash
 *
 * Attempt tracking is recorded back into the store (current_try before
 * each attempt, last_failed / last_successful after it), and when
 * integrity_check_en is set the image MD5 is verified per the HSS payload
 * MD5 contract (BOOT_verify_md5) before the image is allowed to boot.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* bool comes from hss_types.h (OpenSBI sbi_types bool on CONFIG_OPENSBI
 * builds): do NOT include <stdbool.h> here, its bool macro would make
 * this prototype clash with the boot/registry function-pointer types */

bool HSS_BootParamsBootInit(void);

#ifdef __cplusplus
}
#endif

#endif
