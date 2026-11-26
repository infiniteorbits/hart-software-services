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
 *      documented defaults: no override, Primary, Secondary, Golden with
 *      the integrity check enabled).
 *   2. Walk boot_sequence[] in order. boot_sequence[0] is a one-shot
 *      override (BOOT_SRC_NONE = no override): when set it is attempted
 *      first and cleared back to BOOT_SRC_NONE in the same store write
 *      that records current_try, i.e. before the attempt runs, so it is
 *      never attempted again whatever the outcome. For each entry: record
 *      current_try, select the matching HSS storage, and run the stock
 *      boot-image load (HSS_BootInit: header read, magic check, copy to
 *      DDR, register).
 *   3. When integrity_check_en is set and MD5 verification is built in
 *      (menuconfig CONFIG_BOOT_PARAMS_MD5_VERIFY, default y), stream the
 *      image from its storage again and verify its MD5 per the HSS
 *      payload MD5 contract (BOOT_verify_md5 hashes the header
 *      signature/md5Sum window as zeros). A failing image is
 *      deregistered and never booted. With the option disabled the
 *      image boots unverified and a warning is logged.
 *   4. Record last_successful / last_failed (with last_failed_error, the
 *      boot_error_status_t that describes why that entry was rejected)
 *      and stop at the first entry that yields a bootable image.
 *   5. When the Flash boot log is enabled (menuconfig
 *      CONFIG_BOOT_PARAMS_LOG), each boot event printed by
 *      mBOOTPARAMS_LOG() within CONFIG_BOOT_PARAMS_LOG_LEVEL is also
 *      appended to the Bootloader log (BOOT_log_write), which the
 *      application reads back with BOOT_log_read.
 *
 * The tracking fields hold boot_source_t values (10 / 20 / 255); 0 means
 * "none recorded yet". last_failed_error belongs to last_failed and is
 * left in place on a successful boot, so the pair always describes the
 * last rejection seen, not the last attempt. Store write failures are
 * logged but never block
 * the boot itself. If the store cannot be read at all, the stock
 * HSS_BootInit() flow runs unchanged (it tries every registered storage).
 *
 * BOOT_SRC_SECONDARY (fabric eMMC via CoreMMC) has no HSS driver yet and
 * is skipped with a log message.
 */

#include "config.h"
#include "hss_types.h"
#if IS_ENABLED(CONFIG_BOOT_PARAMS_LOG)
/* sbi_snprintf; before hss_debug.h, which declares the console functions
 * itself only when this header has not been seen                          */
#  include <sbi/sbi_console.h>
#  include "hss_version.h"
#endif
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
 * Flash boot log (menuconfig CONFIG_BOOT_PARAMS_LOG)
 *
 * mBOOTPARAMS_LOG(level, fmt, ...) prints a boot event on the console
 * exactly like mHSS_DEBUG_PRINTF() and, when its severity is within
 * CONFIG_BOOT_PARAMS_LOG_LEVEL, also appends it to the Bootloader log
 * (BOOT_log_write; on NAND the ring at BOOT_LOG_NAND_BASE_ADDR /
 * BOOT_LOG_NAND_SIZE, BSP_Config.h) as "<E|W|I> <message>". Only
 * LOG_ERROR, LOG_WARN and LOG_NORMAL may be passed. Messages kept off the
 * Flash log (progress, duplicates) use mHSS_DEBUG_PRINTF() directly.
 *
 * Every call runs after DDR training (BOOT_params_read already needs the
 * NAND driver's DDR buffers), so the Flash is usable at each log point.
 * ------------------------------------------------------------------------- */

#if IS_ENABLED(CONFIG_BOOT_PARAMS_LOG)

/* severities, most severe first; CONFIG_BOOT_PARAMS_LOG_LEVEL is the
 * least severe one written (1 Error, 2 Warning, 3 Info)                    */
#  define BOOTLOG_SEVERITY_LOG_ERROR    1u
#  define BOOTLOG_SEVERITY_LOG_WARN     2u
#  define BOOTLOG_SEVERITY_LOG_NORMAL   3u

/* "<tag> " then the message; BOOT_log_write stores at most 256 bytes      */
#  define BOOTLOG_TAG_LEN               2u
#  define BOOTLOG_LINE_LEN              (BOOTLOG_TAG_LEN + 256u + 1u)

static char bootlog_line_[BOOTLOG_LINE_LEN];
static bool bootlog_disabled_ = false;

static bool bootlog_wanted_(unsigned int severity)
{
    return !bootlog_disabled_
        && (severity <= (unsigned int)CONFIG_BOOT_PARAMS_LOG_LEVEL);
}

/*!
 * \brief Tags the message formatted into bootlog_line_ and appends it to
 * the Flash boot log.
 *
 * A failed write disables the Flash log until the next reset, so the boot
 * never waits on a failing Flash more than once. It never fails the boot.
 */
static void bootlog_commit_(unsigned int severity)
{
    static const char tags[] = { '?', 'E', 'W', 'I' };
    unsigned int len = BOOTLOG_TAG_LEN;

    bootlog_line_[0] = tags[severity];
    bootlog_line_[1] = ' ';

    /* drop the console line feed: BOOT_log_write terminates each record */
    while (bootlog_line_[len] != '\0') {
        len++;
    }
    while ((len > BOOTLOG_TAG_LEN) && (bootlog_line_[len - 1u] == '\n')) {
        len--;
        bootlog_line_[len] = '\0';
    }

    if (BOOT_log_write(bootlog_line_) != BOOT_OK) {
        bootlog_disabled_ = true;
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "bootparams: boot log write failed, "
            "Flash boot log disabled until reset\n");
    }
}

#  define mBOOTPARAMS_LOG(logLevel, ...) {                                 \
        mHSS_DEBUG_PRINTF(logLevel, __VA_ARGS__);                           \
        if (bootlog_wanted_(BOOTLOG_SEVERITY_##logLevel)) {                 \
            bootlog_line_[BOOTLOG_TAG_LEN] = '\0';                          \
            (void)sbi_snprintf(&bootlog_line_[BOOTLOG_TAG_LEN],             \
                BOOTLOG_LINE_LEN - BOOTLOG_TAG_LEN, __VA_ARGS__);           \
            bootlog_commit_(BOOTLOG_SEVERITY_##logLevel);                   \
        }                                                                   \
    }

#else

#  define mBOOTPARAMS_LOG(logLevel, ...)                                   \
        mHSS_DEBUG_PRINTF(logLevel, __VA_ARGS__)

#endif

/* -------------------------------------------------------------------------
 * Private functions
 * ------------------------------------------------------------------------- */

static const char *boot_source_name_(boot_source_t src)
{
    const char *pResult;

    switch (src) {
    case BOOT_SRC_NONE:
        pResult = "none";
        break;

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
 * \return false when the source has no usable HSS storage (fabric eMMC,
 * or the matching service is not compiled in).
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

    case BOOT_SRC_GOLDEN:
#if IS_ENABLED(CONFIG_SERVICE_QSPI)
        HSS_BootSelectQSPI();
        result = true;
#endif
        break;

    case BOOT_SRC_SECONDARY:  /* no HSS CoreMMC driver yet               */
    default:
        break;
    }

    return result;
}

/*
 * MD5 verification is the menuconfig option CONFIG_BOOT_PARAMS_MD5_VERIFY.
 * The code tests its hidden inverse, CONFIG_BOOT_PARAMS_MD5_VERIFY_DISABLED,
 * so that a config.h generated before the option existed (no CONFIG_ line
 * for either symbol) keeps the check built in (see services/boot/Kconfig).
 */
#if !IS_ENABLED(CONFIG_BOOT_PARAMS_MD5_VERIFY_DISABLED)
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
        mBOOTPARAMS_LOG(LOG_ERROR,
            "bootparams: invalid boot image length %lu\n",
            pBootImage->bootImageLength);
        return BOOT_ERR_INVALID_PARAM;
    }

    /* console only: progress, and the failure reason below reaches the
     * Flash log through params_record_failure_()                          */
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

    mBOOTPARAMS_LOG(LOG_NORMAL, "bootparams: image MD5 verified\n");
    return BOOT_OK;
}
#endif /* !CONFIG_BOOT_PARAMS_MD5_VERIFY_DISABLED */

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

    mBOOTPARAMS_LOG(LOG_ERROR, "bootparams: %s rejected: %s (%d)\n",
        boot_source_name_(src), boot_error_name_(error), error);
}

static void params_store_(boot_params_t *pParams)
{
    if (BOOT_params_write(pParams) != BOOT_OK) {
        mBOOTPARAMS_LOG(LOG_ERROR,
            "bootparams: Boot Parameters write failed\n");
    }
}

/* -------------------------------------------------------------------------
 * Public functions
 * ------------------------------------------------------------------------- */

bool HSS_BootParamsBootInit(void)
{
    boot_params_t params;
    boot_source_t prev   = BOOT_SRC_NONE;
    bool          booted = false;

    if (BOOT_params_read(&params) != BOOT_OK) {
        mBOOTPARAMS_LOG(LOG_ERROR,
            "bootparams: Boot Parameters read failed, "
            "falling back to the default boot flow\n");
        return HSS_BootInit();
    }

#if IS_ENABLED(CONFIG_BOOT_PARAMS_LOG)
    /* marks the start of each boot in the Flash log                       */
    mBOOTPARAMS_LOG(LOG_NORMAL,
        "bootparams: boot start (HSS %d.%d.%d): override %s, "
        "sequence %s / %s / %s, integrity check %s\n",
        HSS_VERSION_MAJOR, HSS_VERSION_MINOR, HSS_VERSION_PATCH,
        boot_source_name_(params.boot_sequence[0]),
        boot_source_name_(params.boot_sequence[1]),
        boot_source_name_(params.boot_sequence[2]),
        boot_source_name_(params.boot_sequence[3]),
        params.integrity_check_en ? "on" : "off");
#endif

    for (unsigned int i = 0u; i < (unsigned int)BOOT_SEQ_MAX_ENTRIES; i++) {
        boot_source_t const src = params.boot_sequence[i];
        boot_error_status_t status;

        /* BOOT_SRC_NONE: no override in boot_sequence[0]. Also skip
         * immediate repeats (e.g. ... Golden, Golden): prev holds the
         * entry as read, unaffected by the override clear below         */
        if ((src == BOOT_SRC_NONE) || (src == prev)) {
            continue;
        }
        prev = src;

        mBOOTPARAMS_LOG(LOG_NORMAL, "bootparams: entry %u: %s\n",
            i, boot_source_name_(src));

        if (i == 0u) {
            /* boot_sequence[0] is a one-shot override: consume it before
             * it is attempted, so it is tried at most once whatever the
             * outcome - rejected, booted, or hung before the result could
             * be recorded. Persisted by the store write that follows.   */
            params.boot_sequence[0] = BOOT_SRC_NONE;
            mBOOTPARAMS_LOG(LOG_NORMAL,
                "bootparams: one-shot override, cleared for later boots\n");
        }

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
#if !IS_ENABLED(CONFIG_BOOT_PARAMS_MD5_VERIFY_DISABLED)
            status = integrity_check_(src);

            if (status != BOOT_OK) {
                /* never boot an image that failed verification         */
                HSS_Register_Boot_Image(NULL);
                booted = false;
            }
#else
            mBOOTPARAMS_LOG(LOG_WARN,
                "bootparams: integrity check requested, but MD5 "
                "verification is disabled in this build "
                "(CONFIG_BOOT_PARAMS_MD5_VERIFY): %s boots unverified\n",
                boot_source_name_(src));
#endif
        }

        if (booted) {
            params.last_successful = src;
            params_store_(&params);
            mBOOTPARAMS_LOG(LOG_NORMAL, "bootparams: booting %s\n",
                boot_source_name_(src));
            break;
        }

        params_record_failure_(&params, src, status);
        params_store_(&params);
    }

    if (!booted) {
        mBOOTPARAMS_LOG(LOG_ERROR,
            "bootparams: no boot sequence entry provided a bootable image\n");
    }

    return booted;
}
