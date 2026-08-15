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
 * \file CoreMMC Service
 * \brief Secondary eMMC (fabric CoreMMC) Service
 *
 * See coremmc_service.h for why this owns the one instance and why every
 * transfer is single-block.
 *
 * Like services/mmc, CoreMMC does not need a "service" to run every
 * super-loop - it only needs to be initialized before it is used, and the
 * boot path and the USBDMSC path both do that through HSS_CoreMMC_Init().
 */

#include "config.h"
#include "hss_types.h"
#include "hss_debug.h"
#include "hss_clock.h"
#include "hss_perfctr.h"

#include <assert.h>
#include <string.h>

#include "coremmc_service.h"
#include "hss_memcpy_via_pdma.h"

#include "core_mmc.h"

/*
 * The CoreMMC driver moves a block as BLK_SIZE/4 32-bit words straight out
 * of the caller's buffer, and BLK_SIZE is fixed at 512 in the driver.
 */
#define HSS_COREMMC_SECTOR_SIZE (512u)

/*
 * The one CoreMMC instance driving the secondary eMMC. The MMC data width
 * (8-bit) and FIFO depth (512 bytes) are hardware instantiation parameters;
 * the driver reads them back from the CoreMMC version register during
 * MMC_init() rather than taking them from software.
 */
static mmc_instance_t coreMmc_;
static mmc_params_t coreMmcParams_;
static bool coreMmcInitialized_ = false;

/*
 * Reads that are not a whole number of sectors land their tail here, exactly
 * as services/mmc does: the boot path asks for sizeof(struct HSS_BootImage),
 * which is not a multiple of the sector size.
 */
static char runtBuffer_[HSS_COREMMC_SECTOR_SIZE]
    __attribute__((aligned(sizeof(uint32_t))));

static bool coremmc_bring_up_(void)
{
    mmc_transfer_status_t retval;

    MMC_param_config(&coreMmcParams_);

    coreMmcParams_.clk_rate_div      = CONFIG_SERVICE_COREMMC_CLK_RATE_DIV;
    coreMmcParams_.init_clk_rate_div = CONFIG_SERVICE_COREMMC_INIT_CLK_RATE_DIV;
    coreMmcParams_.data_timeout      = CONFIG_SERVICE_COREMMC_DATA_TIMEOUT;

    retval = MMC_init(&coreMmc_,
        (addr_t)CONFIG_SERVICE_COREMMC_BASE_ADDR, &coreMmcParams_);

    if (retval != MMC_INIT_SUCCESS) {
        /*
         * The device may still have been busy after a power cycle or a warm
         * reset; one retry is what the MSS eMMC bring-up does too.
         */
        HSS_SpinDelay_MilliSecs(50u);
        retval = MMC_init(&coreMmc_,
            (addr_t)CONFIG_SERVICE_COREMMC_BASE_ADDR, &coreMmcParams_);
    }

    if (retval != MMC_INIT_SUCCESS) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "MMC_init() returned unexpected %d\n", retval);
    }

    coreMmcInitialized_ = (retval == MMC_INIT_SUCCESS);

    return coreMmcInitialized_;
}

bool HSS_CoreMMC_Init(void)
{
    bool result = true;

    if (!coreMmcInitialized_) {
        int perf_ctr_index = PERF_CTR_UNINITIALIZED;
        HSS_PerfCtr_Allocate(&perf_ctr_index, "CoreMMC Init");

        mHSS_DEBUG_PRINTF(LOG_STATUS,
            "Attempting to select secondary eMMC (fabric CoreMMC) ... ");
        result = coremmc_bring_up_();
        mHSS_DEBUG_PRINTF_EX("%s\n", result ? "Passed" : "Failed");

        HSS_PerfCtr_Lap(perf_ctr_index);
    }

    return result;
}

//
// HSS_CoreMMC_ReadBlock will handle reads of less than a multiple of the
// sector size by doing the last transfer into a sector buffer.
//
// Single-block throughout: the instance has a 512-byte FIFO, so there is no
// multi-block entry point to fall back to.
//
bool HSS_CoreMMC_ReadBlock(void *pDest, size_t srcOffset, size_t byteCount)
{
    char *pCDest = (char *)pDest;

    assert(((size_t)srcOffset & (HSS_COREMMC_SECTOR_SIZE-1)) == 0u);
    assert(((size_t)pCDest & (sizeof(uint32_t)-1)) == 0u);

    if (!coreMmcInitialized_ && !HSS_CoreMMC_Init()) {
        return false;
    }

    uint32_t src_sector_num = (uint32_t)(srcOffset / HSS_COREMMC_SECTOR_SIZE);
    mmc_transfer_status_t result = MMC_TRANSFER_SUCCESS;
    size_t sectorByteCount = byteCount - (byteCount % HSS_COREMMC_SECTOR_SIZE);

    while ((result == MMC_TRANSFER_SUCCESS) && sectorByteCount) {
        result = MMC_single_block_read(&coreMmc_, src_sector_num,
            (uint32_t *)pCDest);

        if (result != MMC_TRANSFER_SUCCESS) {
            mHSS_DEBUG_PRINTF(LOG_ERROR,
                "MMC_single_block_read() unexpectedly returned %d\n", result);
            break;
        }

        src_sector_num++;
        sectorByteCount = sectorByteCount - HSS_COREMMC_SECTOR_SIZE;
        pCDest = pCDest + HSS_COREMMC_SECTOR_SIZE;
    }

    byteCount = byteCount % HSS_COREMMC_SECTOR_SIZE;

    // handle remainder
    if ((result == MMC_TRANSFER_SUCCESS) && byteCount) {
        assert(byteCount < HSS_COREMMC_SECTOR_SIZE);

        result = MMC_single_block_read(&coreMmc_, src_sector_num,
            (uint32_t *)runtBuffer_);

        if (result != MMC_TRANSFER_SUCCESS) {
            mHSS_DEBUG_PRINTF(LOG_ERROR,
                "MMC_single_block_read() unexpectedly returned %d\n", result);
        } else {
            memcpy_via_pdma(pCDest, runtBuffer_, byteCount);
        }
    }

    return (result == MMC_TRANSFER_SUCCESS);
}

//
// HSS_CoreMMC_WriteBlock will handle requested writes of less than a multiple
// of the sector size by rounding up to the next full sector worth.
//
// No erase ahead of a write: eMMC blocks are overwritten in place.
//
bool HSS_CoreMMC_WriteBlock(size_t dstOffset, void *pSrc, size_t byteCount)
{
    char *pCSrc = (char *)pSrc;

    // if byte count is not a multiple of the sector size, round it up...
    if (byteCount & (HSS_COREMMC_SECTOR_SIZE-1)) {
        byteCount = byteCount + HSS_COREMMC_SECTOR_SIZE;
        byteCount &= ~(HSS_COREMMC_SECTOR_SIZE-1);
    }

    // The CoreMMC driver uses uint32_t* as its pointer type
    // To ensure alignment, would rather tramp through void* and
    // assert check here
    assert(((size_t)dstOffset & (HSS_COREMMC_SECTOR_SIZE-1)) == 0u);
    assert(((size_t)pCSrc & (sizeof(uint32_t)-1)) == 0u);
    assert((byteCount & (HSS_COREMMC_SECTOR_SIZE-1)) == 0u);

    if (!coreMmcInitialized_ && !HSS_CoreMMC_Init()) {
        return false;
    }

    uint32_t dst_sector_num = (uint32_t)(dstOffset / HSS_COREMMC_SECTOR_SIZE);
    mmc_transfer_status_t result = MMC_TRANSFER_SUCCESS;

    while ((result == MMC_TRANSFER_SUCCESS) && (byteCount)) {
        result = MMC_single_block_write(&coreMmc_, (uint32_t *)pCSrc,
            dst_sector_num);

        if (result != MMC_TRANSFER_SUCCESS) {
            mHSS_DEBUG_PRINTF(LOG_ERROR,
                "MMC_single_block_write() unexpectedly returned %d\n", result);
        }

        dst_sector_num++;
        byteCount = byteCount - HSS_COREMMC_SECTOR_SIZE;
        pCSrc = pCSrc + HSS_COREMMC_SECTOR_SIZE;
    }

    return (result == MMC_TRANSFER_SUCCESS);
}

void HSS_CoreMMC_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize,
    uint32_t *pBlockCount)
{
    /*
     * hw_sec_count is the device capacity in 512-byte sectors, read from
     * EXT_CSD during MMC_init(). It is only meaningful once the device is
     * up, so bring it up rather than reporting a zero-sized drive - which
     * is what a USBDMSC host would otherwise be told.
     */
    if (!coreMmcInitialized_) {
        (void)HSS_CoreMMC_Init();
    }

    *pBlockSize = (uint32_t)HSS_COREMMC_SECTOR_SIZE;
    *pEraseSize = (uint32_t)HSS_COREMMC_SECTOR_SIZE;
    *pBlockCount = coreMmcInitialized_ ? coreMmc_.hw_sec_count : 0u;
}
