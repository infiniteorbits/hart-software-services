/*******************************************************************************
 * Copyright 2026 RFIM Space.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software - OrbSight-2
 *
 */

/*!
 * \file Boot Parameters driven boot
 * \brief Bootloader 0 boot source selection from the Boot Parameters store
 *
 * Implements the Bootloader 0 boot policy on top of the shared BSP
 * modules (boards/mpfs-orbsight-v2/bsp/, copies of the application BSP):
 *
 *   1. Read the Boot Parameters (most recent valid record, or the
 *      documented defaults: Primary, Secondary, Golden, Golden with the
 *      integrity check enabled).
 *   2. Walk boot_sequence[] in order. For each entry: record current_try,
 *      select the matching HSS storage, and run the stock boot-image load
 *      (HSS_BootInit: header read, magic check, copy to DDR, register).
 *   3. When integrity_check_en is set, stream the image from its storage
 *      again and verify its MD5 per the HSS payload MD5 contract
 *      (BOOT_verify_md5 hashes the header signature/md5Sum window as
 *      zeros). A failing image is deregistered and never booted.
 *   4. Record last_successful / last_failed (with last_failed_error, the
 *      boot_error_status_t that describes why that entry was rejected)
 *      and stop at the first entry that yields a bootable image.
 *
 * The tracking fields hold boot_source_t values (10 / 20 / 255); 0 means
 * "none recorded yet". last_failed_error belongs to last_failed and is
 * left in place on a successful boot, so the pair always describes the
 * last rejection seen, not the last attempt. Store write failures are
 * logged but never block
 * the boot itself. If the store cannot be read at all, the stock
 * HSS_BootInit() flow runs unchanged (it tries every registered storage).
 *
 * BOOT_SRC_SECONDARY is the fabric eMMC behind the CoreMMC instance,
 * served by services/coremmc. It is skipped only when that service is
 * not compiled in.
 */

#include "config.h"
#include "hss_types.h"
#include "hss_debug.h"

#include "hss_boot_params.h"
#include "hss_boot_init.h"
#include "hss_boot_service.h"

#include "BSP_Boot_Params.h"
#include "BSP_Boot_Loader0.h"

#if IS_ENABLED(CONFIG_OPENSBI)
/*
 * The BSP headers pull in <stdbool.h>, whose bool macro would make every
 * declaration below _Bool, while hss_types.h (OpenSBI sbi_types) and the
 * prototypes parsed above use the OpenSBI int bool: drop the macro so
 * bool falls back to the sbi_types typedef and the definitions below
 * match their prototypes. stdbool's true/false (1/0) remain valid.
 */
#  undef bool
#endif

/*
 * Guard the cross-repo store contract: the application writes the record
 * this build reads. boot_params_t must stay 33 bytes packed (enums 4
 * bytes - neither tree builds with -fshort-enums; integrity_check_en is
 * a 1-byte C99 bool inside the packed struct in both trees).
 */
_Static_assert(sizeof(boot_params_t) == 33u,
    "boot_params_t layout drifted from the application BSP contract");

/* -------------------------------------------------------------------------
 * Private functions
 * ------------------------------------------------------------------------- */

static const char *boot_source_name_(boot_source_t src)
{
    const char *pResult;

    switch (src) {
    case BOOT_SRC_PRIMARY:
        pResult = "Primary (MSS eMMC)";
        break;

    case BOOT_SRC_SECONDARY:
        pResult = "Secondary (fabric eMMC)";
        break;

    case BOOT_SRC_GOLDEN:
        pResult = "Golden (QSPI)";
        break;

    default:
        pResult = "unknown";
        break;
    }

    return pResult;
}

static const char *boot_error_name_(boot_error_status_t error)
{
    const char *pResult;

    switch (error) {
    case BOOT_OK:
        pResult = "none";
        break;

    case BOOT_ERR_INVALID_PARAM:
        pResult = "invalid parameter";
        break;

    case BOOT_ERR_STORAGE_FAIL:
        pResult = "storage failure / no image";
        break;

    case BOOT_ERR_BOOT_SOURCE:
        pResult = "unsupported boot source";
        break;

    case BOOT_ERR_MD5_VERIFY:
        pResult = "MD5 verification failed";
        break;

    default:
        pResult = "unknown";
        break;
    }

    return pResult;
}

/*!
 * \brief Selects the HSS storage backing a boot source.
 *
 * \return false when the source has no usable HSS storage, i.e. its
 * service is not compiled into this build.
 */
static bool storage_select_(boot_source_t src)
{
    bool result = false;

    switch (src) {
    case BOOT_SRC_PRIMARY:
#if IS_ENABLED(CONFIG_SERVICE_MMC)
        HSS_BootSelectEMMC();
        result = true;
#endif
        break;

    case BOOT_SRC_SECONDARY:
#if IS_ENABLED(CONFIG_SERVICE_COREMMC)
        HSS_BootSelectCoreMMC();
        result = true;
#endif
        break;

    case BOOT_SRC_GOLDEN:
#if IS_ENABLED(CONFIG_SERVICE_QSPI)
        HSS_BootSelectQSPI();
        result = true;
#endif
        break;

    default:
        break;
    }

    return result;
}

/*!
 * \brief Verifies the MD5 of the just-loaded boot image per the HSS
 * payload MD5 contract, streaming it from its storage backend.
 *
 * \return BOOT_OK when the image verifies, otherwise the
 * boot_error_status_t to record in last_failed_error.
 */
static boot_error_status_t integrity_check_(boot_source_t src)
{
    struct HSS_BootImage const * const pBootImage =
        (struct HSS_BootImage *)(uintptr_t)CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR;
    boot_error_status_t status;

    if ((pBootImage->bootImageLength == 0u)
            || (pBootImage->bootImageLength > 0xFFFFFFFFu)) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: invalid boot image length %lu\n",
            pBootImage->bootImageLength);
        return BOOT_ERR_INVALID_PARAM;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL,
        "bootparams: verifying image MD5 (%lu bytes) ...\n",
        pBootImage->bootImageLength);

    status = BOOT_verify_md5(src, (uint32_t)pBootImage->bootImageLength,
        pBootImage->md5Sum);

    if (status != BOOT_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: image MD5 verification failed: %d\n", status);
        return status;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "bootparams: image MD5 verified\n");
    return BOOT_OK;
}

/*!
 * \brief Records a rejected boot entry and persists the parameters.
 *
 * last_failed and last_failed_error are written as a pair, so a host
 * reading the store always sees which entry was rejected alongside the
 * reason it was rejected.
 */
static void params_record_failure_(boot_params_t *pParams, boot_source_t src,
    boot_error_status_t error)
{
    pParams->last_failed       = src;
    pParams->last_failed_error = error;

    mHSS_DEBUG_PRINTF(LOG_ERROR, "bootparams: %s rejected: %s (%d)\n",
        boot_source_name_(src), boot_error_name_(error), error);
}

static void params_store_(boot_params_t *pParams)
{
    if (BOOT_params_write(pParams) != BOOT_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: Boot Parameters write failed\n");
    }
}

/* -------------------------------------------------------------------------
 * Public functions
 * ------------------------------------------------------------------------- */

bool HSS_BootParamsBootInit(void)
{
    boot_params_t params;
    bool          booted = false;

    if (BOOT_params_read(&params) != BOOT_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: Boot Parameters read failed, "
            "falling back to the default boot flow\n");
        return HSS_BootInit();
    }

    for (unsigned int i = 0u; i < (unsigned int)BOOT_SEQ_MAX_ENTRIES; i++) {
        boot_source_t const src = params.boot_sequence[i];
        boot_error_status_t status;

        /* skip immediate repeats (e.g. the default ... Golden, Golden)  */
        if ((i > 0u) && (src == params.boot_sequence[i - 1u])) {
            continue;
        }

        mHSS_DEBUG_PRINTF(LOG_NORMAL, "bootparams: entry %u: %s\n",
            i, boot_source_name_(src));

        if (!storage_select_(src)) {
            /* no HSS storage behind this source: params_record_failure_
             * logs it as BOOT_ERR_BOOT_SOURCE                            */
            params_record_failure_(&params, src, BOOT_ERR_BOOT_SOURCE);
            params_store_(&params);
            continue;
        }

        /* record the attempt before it runs: after a hang and watchdog
         * reset, current_try != last_successful marks the culprit       */
        params.current_try = src;
        params_store_(&params);

        booted = HSS_BootInit();

        /* no image loaded: the storage held nothing usable            */
        status = booted ? BOOT_OK : BOOT_ERR_STORAGE_FAIL;

        if (booted && params.integrity_check_en) {
            status = integrity_check_(src);

            if (status != BOOT_OK) {
                /* never boot an image that failed verification         */
                HSS_Register_Boot_Image(NULL);
                booted = false;
            }
        }

        if (booted) {
            params.last_successful = src;
            params_store_(&params);
            break;
        }

        params_record_failure_(&params, src, status);
        params_store_(&params);
    }

    if (!booted) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: no boot sequence entry provided a bootable image\n");
    }

    return booted;
}
