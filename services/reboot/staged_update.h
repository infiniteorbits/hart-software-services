/*******************************************************************************
 * Copyright 2026 RFIM Space.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software - OrbSight-2
 *
 */

#ifndef STAGED_UPDATE_H
#define STAGED_UPDATE_H

/*!
 * \file Staged Auto Update API
 * \brief Boot-time check for an application-staged FPGA update request
 *
 * The application stages a new bitstream in the UPDATE slot of the
 * bitstream SPI Flash, then programs an update request flag into the
 * golden SW QSPI Flash (BSP_bitstream_execute_iap in the application
 * BSP) and resets the MSS. At the next boot, before any payload is
 * started, HSS_StagedUpdateInit() finds the flag, clears it,
 * authenticates the staged image and initiates the System Controller
 * Auto Update service. On success the System Controller reprograms the
 * device (fabric and eNVM) and issues a full device reset; on failure
 * boot continues normally on the current design.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* bool comes from hss_types.h (OpenSBI sbi_types bool on CONFIG_OPENSBI
 * builds): do NOT include <stdbool.h> here, its bool macro would make
 * this prototype clash with the registry's function-pointer types */

bool HSS_StagedUpdateInit(void);

#ifdef __cplusplus
}
#endif

#endif
