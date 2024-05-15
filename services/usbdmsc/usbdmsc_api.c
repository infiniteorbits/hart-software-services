/**************************************************************************//**
 * Copyright 2019-2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 *
 * SVN $Revision: 11522 $
 * SVN $Date: 2019-06-26 10:14:17 +0530 (Wed, 26 Jun 2019) $
 */

#include "config.h"
#include "hss_types.h"
#include "hss_debug.h"

#undef ROUNDUP
#undef ROUNDDOWN
#include "mss_hal.h"
#include "mss_assert.h"
#include "flash_drive_app.h"
#include "mss_plic.h"
#include "uart_helper.h"
#include "usbdmsc_service.h"
#include "hss_init.h"

/**************************************************************************//**
 */
void USBDMSC_Init(void)
{
    SYSREG->SOFT_RESET_CR &= ~ (1u << 16u);

    PLIC_init();

    PLIC_SetPriority(PLIC_USB_DMA_INT_OFFSET, 3);
    PLIC_SetPriority(PLIC_USB_MC_INT_OFFSET, 3);

    PLIC_EnableIRQ(PLIC_USB_DMA_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_USB_MC_INT_OFFSET);

    PLIC_SetPriority(MMC_main_PLIC, 2u);
    PLIC_SetPriority(MMC_wakeup_PLIC, 2u);
    PLIC_EnableIRQ(MMC_main_PLIC);
    PLIC_EnableIRQ(MMC_wakeup_PLIC);

    HSS_USBInit();

    MSS_MPU_configure(MSS_MPU_USB, MSS_MPU_PMP_REGION3, 0x08000000u, 0x200000u,
        MPU_MODE_READ_ACCESS|MPU_MODE_WRITE_ACCESS|MPU_MODE_EXEC_ACCESS, MSS_MPU_AM_NAPOT, 0u);

    /* DMA init for eMMC */
    MSS_MPU_configure(MSS_MPU_MMC, MSS_MPU_PMP_REGION3, 0x08000000u, 0x200000u,
        MPU_MODE_READ_ACCESS|MPU_MODE_WRITE_ACCESS|MPU_MODE_EXEC_ACCESS, MSS_MPU_AM_NAPOT, 0u);
}

HSSTicks_t last_poll_time = 0u;

bool USBDMSC_Poll(void)
{
    bool done = false;
#if !defined(CONFIG_SERVICE_USBDMSC_REGISTER) || !defined(CONFIG_SERVICE_TINYCLI_REGISTER)
    uint8_t rx_byte = 0;

    bool retval = uart_getchar(&rx_byte, 0, false);

    if (retval) {
        if ((rx_byte == '\003') || (rx_byte == '\033')) {
        done = true;
        }
    } else {
#else
    {
#endif

        //poll PLIC
        uint32_t source = PLIC_ClaimIRQ();

        switch (source) {
#if defined(CONFIG_SERVICE_MMC)
        case MMC_main_PLIC: // MMC interrupt
            PLIC_mmc_main_IRQHandler(); // interrupt 88
            break;
#endif

        case PLIC_USB_MC_INT_OFFSET: // main USB interrupt
            PLIC_usb_mc_IRQHandler(); // interrupt 87
            break;

        case PLIC_USB_DMA_INT_OFFSET: // DMA USB interrupt
            PLIC_usb_dma_IRQHandler(); // interrupt 86
            break;

        default:
            break;
        }

        if (source != INVALID_IRQn) {
            PLIC_CompleteIRQ(source);
        }
    }

    if (HSS_Timer_IsElapsed(last_poll_time, 5*TICKS_PER_SEC)) {
        FLASH_DRIVE_dump_xfer_status();
        last_poll_time = HSS_GetTime();
    }

    return done;
}

void USBDMSC_Shutdown(void)
{
#ifndef CONFIG_SERVICE_USBDMSC_REGISTER
    PLIC_ClearPendingIRQ();
    USBDMSC_Deactivate();
#endif
}

void USBDMSC_Start(void)
{
    bool done = !FLASH_DRIVE_init();

    if (done) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "FLASH_DRIVE_init() returned false...\n");
    } else {
#if !defined(CONFIG_SERVICE_USBDMSC_REGISTER) || !defined(CONFIG_SERVICE_TINYCLI_REGISTER)
        bool isHostConnected = false;
        mHSS_PUTS("Waiting for USB Host to connect... (CTRL-C to quit)\n");

        do {
            if (!isHostConnected) {
                // if we are not connected, wait until we are
                isHostConnected = FLASH_DRIVE_is_host_connected();
                if (isHostConnected) {
                    mHSS_PUTS("USB Host connected. Waiting for disconnect... (CTRL-C to quit)\n");
                }
            } else {
                // else quit once we've disconnected again...
                isHostConnected = FLASH_DRIVE_is_host_connected();
                if (!isHostConnected) {
                    done = true;
                }
            }

            done = done || USBDMSC_Poll();
        } while (!done);

        void HSS_Storage_FlushWriteBuffer(void);
        HSS_Storage_FlushWriteBuffer();

        mHSS_PUTS("\nUSB Host disconnected...\n");
#else
        USBDMSC_Activate();
#endif
    }
}
