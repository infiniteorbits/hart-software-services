/*******************************************************************************
 * Copyright 2019 Microchip Corporation.
 *
 * SPDX-License-Identifier: MIT
 * 
 * MPFS HSS Embedded Software
 *
 */

/**
 * \file HSS Debug UART Initalization
 * \brief Debug UART Initialization
 */

#include "config.h"
#include "hss_types.h"
#include "hss_init.h"

#include <assert.h>

#include "hss_debug.h"

#include <mss_uart.h>

bool HSS_UARTInit(void)
{
    // initialise debug UART

#if defined(CONFIG_PLATFORM_POLARFIRESOC)
    MSS_UART_init(&g_mss_uart0_lo, MSS_UART_57600_BAUD, 
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    // default all UARTs to 57600 for now 
    // subsequent OS loads can change these if needed...
    MSS_UART_init(&g_mss_uart1_lo, MSS_UART_57600_BAUD, 
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_UART_init(&g_mss_uart2_lo, MSS_UART_57600_BAUD, 
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_UART_init(&g_mss_uart3_lo, MSS_UART_57600_BAUD, 
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_UART_init(&g_mss_uart4_lo, MSS_UART_57600_BAUD, 
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);
#else
#  error Unknown PLATFORM
#endif

    return true;
}