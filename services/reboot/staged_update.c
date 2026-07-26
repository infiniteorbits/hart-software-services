/*******************************************************************************
 * Copyright 2026 RFIM Space.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software - OrbSight-2
 *
 */

/*!
 * \file Staged Auto Update
 * \brief Boot-time check for an application-staged FPGA update request
 *
 * Reads the 4-byte update request flag at the top of the golden SW QSPI
 * NOR Flash (MSS QSPI @ 0x21000000, infineon_s25fl driver). The flag is
 * programmed by the application (BSP_bitstream_execute_iap in
 * BSP_Bitstream_Update.c) just before it resets the MSS - the address
 * and magic value are mirrored from there. When the flag carries the
 * request magic, it is cleared first by erasing its sector (so a failing
 * image can never cause an update boot loop), the staged UPDATE
 * bitstream (SPI directory entry 1) is authenticated, and the System
 * Controller Auto Update service (46H) is initiated while no payload is
 * running: the E51 executes from L2-LIM, the U54s are still parked, and
 * eNVM is quiet. On success the System Controller reprograms the device
 * and issues a full device reset; this function then never returns. On
 * any failure, boot continues normally.
 */

#include "config.h"
#include "hss_types.h"
#include "hss_debug.h"
#include "hss_clock.h"

#include "staged_update.h"

#include "infineon_s25fl.h"
#include "mss_sys_services.h"
#include "mss_watchdog.h"

/* -------------------------------------------------------------------------
 * Update request flag contract (see BSP_Bitstream_Update.c in the
 * application tree - the address and magic must match there)
 * ------------------------------------------------------------------------- */

/* the 64 KB sector directly below the reserved Bootloader log ring
 * (2 x 64 KB @ 0x01FD0000) and Boot Parameters sector (0x01FF0000); it
 * holds only this 4-byte flag                                             */
#define STGUPD_FLAG_ADDR                0x01FC0000u

/* "UPDT", stored little endian; erased Flash (0xFFFFFFFF) or any other
 * value reads as "no request"                                             */
#define STGUPD_FLAG_MAGIC               0x55504454u

/* Auto Update trigger contract (see reboot_service.c)                      */
#define STGUPD_AUTO_UPDATE_IMAGE_INDEX  1u
#define STGUPD_ERR_SAME_VERSION         24u

/* -------------------------------------------------------------------------
 * Private data
 * ------------------------------------------------------------------------- */

/*
 * Golden SW QSPI NOR Flash on the MSS QSPI controller; mirrors the device
 * instance used by the QSPI service (qspi_api.c). The infineon_s25fl
 * driver initializes the controller lazily and never alters the Flash
 * device state, so owning a second instance of the same device is safe.
 */
static flash_device_t stgupdFlashDevice = {
    .ctrl_base   = 0x21000000u,
    .clk_div     = QSPI_CLK_DIV_16,
    .is_mss_qspi = 1u,
};

static uint8_t flagBuffer[4] __attribute__((aligned(4)));

/* -------------------------------------------------------------------------
 * Private functions
 * ------------------------------------------------------------------------- */

static uint32_t stgupd_read_flag_(void)
{
    Flash_read(&stgupdFlashDevice, flagBuffer, STGUPD_FLAG_ADDR,
        sizeof(flagBuffer));

    return ((uint32_t)flagBuffer[0]) | ((uint32_t)flagBuffer[1] << 8)
        | ((uint32_t)flagBuffer[2] << 16) | ((uint32_t)flagBuffer[3] << 24);
}

/* -------------------------------------------------------------------------
 * Private functions - Auto Update trigger (mirrors reboot_service.c)
 * ------------------------------------------------------------------------- */

static void stgupd_trigger_auto_update_(void)
{
    uint16_t ret;

    /* interrupt mode is broken for failed services; use polling mode      */
    MSS_SYS_select_service_mode(MSS_SYS_SERVICE_POLLING_MODE, NULL);

    /* the image must be authenticated before attempting Auto Update, as
     * invalid images may cause the System Controller to hang             */
    ret = MSS_SYS_authenticate_iap_image(STGUPD_AUTO_UPDATE_IMAGE_INDEX);
    if (ret) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "stgupd: staged image failed authentication: %d\n", ret);
        return;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "stgupd: Auto Update in progress\n");
    HSS_SpinDelay_MilliSecs(100u);

    ret = MSS_SYS_execute_iap(MSS_SYS_IAP_AUTOUPDATE_CMD, 0, 0);
    if (!ret) {
        /* the System Controller is now reprogramming the device and will
         * issue a full device reset on completion (~30 s); keep the E51
         * watchdog refreshed while waiting                               */
        while (1) {
            MSS_WD_reload(MSS_WDOG0_LO);
        }
    }

    switch (ret) {
    case STGUPD_ERR_SAME_VERSION:
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "stgupd: staged image is not an update: %d\n", ret);
        break;

    default:
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "stgupd: no valid Auto Update image found: %d\n", ret);
        break;
    }
}

/* -------------------------------------------------------------------------
 * Public functions
 * ------------------------------------------------------------------------- */

bool HSS_StagedUpdateInit(void)
{
    if (stgupd_read_flag_() != STGUPD_FLAG_MAGIC) {
        return true;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL,
        "stgupd: application-staged update request found\n");

    /* clear the flag before triggering, so a failing image can never
     * cause an update boot loop; if it cannot be cleared, do not trigger
     * either                                                             */
    if (Flash_64KByte_erase(&stgupdFlashDevice, STGUPD_FLAG_ADDR,
            sizeof(flagBuffer)) != 0u) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "stgupd: failed to clear update request, skipping update\n");
        return true;
    }

    if (stgupd_read_flag_() == STGUPD_FLAG_MAGIC) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
            "stgupd: update request still set after erase, "
            "skipping update\n");
        return true;
    }

    /* does not return when the update is accepted                         */
    stgupd_trigger_auto_update_();

    return true;
}
