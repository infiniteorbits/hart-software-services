/*******************************************************************************
 * Copyright 2024 Infinite Orbits
 *
 *
 * MPFS HSS Embedded Software
 *
 */

/**
 * \file HSS Slot Selection
 * \brief Slot Selection
 */

#include "config.h"
#include "hss_types.h"

#include "hss_init.h"
#include "hss_boot_service.h"
#include "hss_boot_init.h"
#include "hss_sys_setup.h"
#include "hss_progress.h"
#include "hss_slot_selection.h"

#if IS_ENABLED(CONFIG_SERVICE_SPI)
#  include <mss_sys_services.h>
#  define SPI_FLASH_BOOT_ENABLED (CONFIG_SERVICE_BOOT_SPI_FLASH_OFFSET != 0xFFFFFFFF)
#else
#  define SPI_FLASH_BOOT_ENABLED 0
#endif

#if IS_ENABLED(CONFIG_SERVICE_OPENSBI)
#  include "opensbi_service.h"
#endif

#if IS_ENABLED(CONFIG_SERVICE_QSPI)
#  include "qspi_service.h"
#endif

#if IS_ENABLED(CONFIG_SERVICE_MMC)
#  include "mmc_service.h"
#  include "gpt.h"
#endif

#if (SPI_FLASH_BOOT_ENABLED)
#  include "mss_sys_services.h"
#endif

#include "hss_state_machine.h"
#include "hss_debug.h"
#include "hss_perfctr.h"

#include <string.h>
#include <assert.h>

#if IS_ENABLED(CONFIG_COMPRESSION)
#  include "hss_decompress.h"
#endif

#if IS_ENABLED(CONFIG_CRYPTO_SIGNING)
#  include "hss_boot_secure.h"
#endif

#include "hss_boot_pmp.h"
#include "hss_atomic.h"

#define BLOCK_SIZE_BYTES 512
#define SLOT_SIZE_BYTES (100 * 1024 * 1024)  // 100 MB in bytes

typedef enum {
    REGION_SLOT_0 = 0,
    REGION_SLOT_1 = REGION_SLOT_0 + (SLOT_SIZE_BYTES / BLOCK_SIZE_BYTES),
    REGION_SLOT_2 = REGION_SLOT_1 + (SLOT_SIZE_BYTES / BLOCK_SIZE_BYTES),
    REGION_SLOT_3 = REGION_SLOT_2 + (SLOT_SIZE_BYTES / BLOCK_SIZE_BYTES),
    BOOT_PARAMS_REGION = REGION_SLOT_3 + (SLOT_SIZE_BYTES / BLOCK_SIZE_BYTES)
} SlotsRegion;

typedef struct {
    uint8_t LastFailed;
    uint8_t CurrentTry;
    uint8_t LastSuccessful;
    uint8_t BootSequence[4];
} ParamData;

// Enumeration to define the order of elements
typedef enum {
    LAST_FAILED_INDEX,      // Index for LastFailed
    CURRENT_TRY_INDEX,      // Index for CurrentTry
    LAST_SUCCESSFUL_INDEX,  // Index for LastSuccessful
    BOOT_SEQUENCE_INDEX     // Index for BootSequence
} ParamDataIndex;

ParamData Params;
size_t ParamRegionOffset = BOOT_PARAMS_REGION * BLOCK_SIZE_BYTES;
uint8_t buff[BLOCK_SIZE_BYTES];
bool slotFailedFlag = false;
bool BootGoldenFlag = false;

SlotsRegion determineSlotToRead(ParamData *params);
void copyParamDataToBuffer(const ParamData *params, uint8_t *buffer);
void copyBufferToParamData(const uint8_t *buffer, ParamData *params) ;

// Function to copy a ParamData structure to a buffer
void copyParamDataToBuffer(const ParamData *params, uint8_t *buffer) {
    buffer[LAST_FAILED_INDEX] = params->LastFailed;
    buffer[CURRENT_TRY_INDEX] = params->CurrentTry;
    buffer[LAST_SUCCESSFUL_INDEX] = params->LastSuccessful;
    for (size_t i = 0; i < sizeof(params->BootSequence); ++i) {
        buffer[BOOT_SEQUENCE_INDEX + i] = params->BootSequence[i];
    }
}

// Function to copy from a buffer to a ParamData structure
void copyBufferToParamData(const uint8_t *buffer, ParamData *params) {
    params->LastFailed = buffer[LAST_FAILED_INDEX];
    params->CurrentTry = buffer[CURRENT_TRY_INDEX];
    params->LastSuccessful = buffer[LAST_SUCCESSFUL_INDEX];
    for (size_t i = 0; i < sizeof(params->BootSequence); ++i) {
        params->BootSequence[i] = buffer[BOOT_SEQUENCE_INDEX + i];
    }
}

// Function to determine the region to read based on LastFailed
SlotsRegion determineSlotToRead(ParamData *params) 
{
    if(params->LastFailed < sizeof(Params.BootSequence)) {

        if (params->BootSequence[params->LastFailed] == 0u) {
            mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence[%d] = BOOT REGION_SLOT_0 \n", params->LastFailed);
            return REGION_SLOT_0;
        } else  if (params->BootSequence[params->LastFailed] == 1u) {
            mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence[%d] = BOOT REGION_SLOT_1\n", params->LastFailed);
            return REGION_SLOT_1;
        } else  if (params->BootSequence[params->LastFailed] == 2u) {
            mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence[%d] = BOOT REGION_SLOT_2\n", params->LastFailed);
            return REGION_SLOT_2;
        } else  if (params->BootSequence[params->LastFailed] == 3u) {
            mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence[%d] = BOOT REGION_SLOT_3\n", params->LastFailed);
            return REGION_SLOT_3;
        } else {
            params->BootSequence[params->LastFailed] = params->LastSuccessful;
            BootGoldenFlag = true;
            mHSS_DEBUG_PRINTF(LOG_ERROR,"BOOT SEQUENCE VALUE OF OUT RANGE, BOOT GOLDEN APP \n");
            return REGION_SLOT_0;
        }
    }
    else {

        mHSS_DEBUG_PRINTF(LOG_ERROR,"LAST FAILED VALUE OUT OF RANGE, BOOT GOLDEN APP\n");
        params->LastFailed = 0u;
        Params.CurrentTry = 0u;
        BootGoldenFlag = true;
        HSS_Slot_save_params();
        return REGION_SLOT_0;
    }
}

void HSS_Slot_Failed(void)
{
    if ((!slotFailedFlag) && (!BootGoldenFlag)) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "HSS_Slot_Failed() Failed\n");
        HSS_Slot_save_params();
        slotFailedFlag = true;
    }
}

void HSS_Slot_save_params(void)
{
    copyParamDataToBuffer(&Params, buff);
    HSS_MMC_WriteBlock(ParamRegionOffset, buff, BLOCK_SIZE_BYTES);
}

void HSS_SlotSelection(size_t *pSlotOffset)
{
    // Read the parameters from eMMC
    HSS_MMC_ReadBlock(&buff, ParamRegionOffset, BLOCK_SIZE_BYTES);
    copyBufferToParamData(buff, &Params);

    if (!BootGoldenFlag) {
        Params.LastFailed = Params.CurrentTry;
        Params.CurrentTry++;
    }

    // Determine the region to read based on LastFailed
    *pSlotOffset = (size_t)determineSlotToRead(&Params);

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastFailed: %u\n", Params.LastFailed);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"CurrentTry: %u\n", Params.CurrentTry);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastSuccessful: %u\n", Params.LastSuccessful);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence: %d, %d, %d, %d \n ", Params.BootSequence[0],
            Params.BootSequence[1], Params.BootSequence[2], Params.BootSequence[3]);
}

