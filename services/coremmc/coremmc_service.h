#ifndef HSS_COREMMC_SERVICE_H
#define HSS_COREMMC_SERVICE_H

/*******************************************************************************
 * Copyright 2026 RFIM Space.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software - OrbSight-2
 *
 * Hart Software Services - Secondary eMMC (fabric CoreMMC) Initialization
 *
 */

/*!
 * \file CoreMMC API
 * \brief Secondary eMMC (fabric CoreMMC) Service API
 *
 * The secondary eMMC is not on the hardened MSS MMC controller: it hangs off
 * a CoreMMC instance in the FPGA fabric, reached over the FIC as an AHB slave
 * at CONFIG_SERVICE_COREMMC_BASE_ADDR.
 *
 * Two consequences shape this API, and both differ from services/mmc:
 *
 *   - The CoreMMC driver keeps its state in a caller-owned mmc_instance_t
 *     rather than inside the driver, so exactly one instance lives here and
 *     every user goes through these functions. A second instance would
 *     re-run the whole eMMC identification sequence behind the back of
 *     whoever was using the first one.
 *   - The instance is built with a 512-byte FIFO, so a whole transfer has to
 *     fit in one block: there is no DMA path and no multi-block path. Every
 *     512 bytes costs its own polled transfer, which makes this device
 *     appreciably slower than the primary one for the same byte count.
 *
 * The driver's blocking entry points poll their status registers and never
 * wait on an interrupt, so nothing here needs the PLIC - which is what makes
 * it usable from the USBDMSC poll loop as well as from the boot path.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "hss_types.h"

bool HSS_CoreMMC_Init(void);
bool HSS_CoreMMC_ReadBlock(void *pDest, size_t srcOffset, size_t byteCount);
bool HSS_CoreMMC_WriteBlock(size_t dstOffset, void *pSrc, size_t byteCount);
void HSS_CoreMMC_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize,
    uint32_t *pBlockCount);

#ifdef __cplusplus
}
#endif

#endif
