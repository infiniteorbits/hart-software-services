/******************************************************************************
 * Copyright (c) 2024 Infinite Orbits. All rights reserved.
 *
 * @file vmem_spi.c
 * @brief Implementation of vmem spi functionality for CSP.
 *
 * This file contains the implementation of the vmem spi functionality for CSP,
 * which allows transferring information over CAN using the CSP protocol.
 *
 * @authors
 * - A. Tarragó (abel.tarrago@ixrev.com)
 * - Shivaraj (shivaraj@infiniteorbits.com)
 * - Pratishtha (pratishtha@infiniteorbits.com)
 *
 * @version
 * - 1.0: Initial version
 *
 * @date
 * - 2024-09-10: Created
 *****************************************************************************/

/*---------------------------------Includes---------------------------------*/
#include "hss_spi.h"
#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"
#include "hss_debug.h"

/*----------------------------Constant Definitions--------------------------*/
#define BLOCK_SIZE                      (64 * 1024) // 64 KB
#define MSS_SYS_MAILBOX_DATA_OFFSET     0u
#define SPI_COMMAND                     14
#define SPI_SLOT_SIZE         (10 * 1024 * 1024)  // 10 MB

const uint8_t hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                             '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

const uint8_t g_greeting_msg[] =
"\n\r **** SPI Menu **** \n\
\r - 0. Golden Bitstream Authentication \n\
\r - 1. Update Bitstream Authentication \n\
\r - 2. IAP Bitstream Authentication \n\
\r - 3. IAP Authentication \n\
\r - 4. Auto Update \n\
\r - 5. Program IAP\n";


void delay1(volatile uint32_t n);
//void erase_section(uint32_t address);
void execute_designinfo_service(void);
void display_output(uint8_t* in_buffer, uint32_t byte_length);

/**
 * @brief Delays execution for a given number of cycles.
 * @param n Number of delay cycles.
 */
void delay1(volatile uint32_t n)
{
    while (n)
        n--;
}

/**cc
 * @brief Erases a section of flash memory starting from the specified address.
 *        The section is erased in 64k blocks
 * @param address The starting address of the section to be erased.
 */
void erase_section(uint32_t address) {

    for (uint32_t index = 0; index < SPI_SLOT_SIZE; index += BLOCK_SIZE)
    {
        FLASH_erase_64k_block(address + index);
        delay1(500);
    }
}

/**
 * @brief Display content of buffer passed as parameter as hex values.
 */
void display_output
(
    uint8_t* in_buffer,
    uint32_t byte_length
)
{
    uint32_t inc;
    //uint8_t byte = 0;

    for(inc = 0; inc < byte_length; ++inc)
    {
        //byte = in_buffer[inc];
        /*MSS_UART_polled_tx(&g_mss_uart1_lo, &hex_chars[((byte & 0xF0) >> 4) ], 1);
        MSS_UART_polled_tx(&g_mss_uart1_lo, &hex_chars[(byte & 0x0F)], 1);
        MSS_UART_polled_tx(&g_mss_uart1_lo, (const uint8_t*)" ", sizeof(" "));*/
    }
}

/**
 * @brief  Demonstrate design info service
 */
void execute_designinfo_service(void)
{
    uint8_t status;
    uint8_t data_buffer [1024];

   mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r  - Design Information: ");
    status = MSS_SYS_get_design_info(data_buffer, MSS_SYS_MAILBOX_DATA_OFFSET);

    if(MSS_SYS_SUCCESS == status)
    {
       display_output(data_buffer, 10);

      mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r  - Design Version: ");
       display_output((data_buffer + 32), 2);

      mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r  - Design Back-Level: ");
       display_output((data_buffer + 34), 2);
      mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r");
    }
    else
    {
       mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r DesignInfo Service failed.\r\n");
    }
}
/*-----------------------------Global Functions-----------------------------*/
/**
 * @brief Reads data from the SPI flash memory into the provided buffer.
 *        The data is read from the flash address and copied into the output buffer.
 */
#define MAX_TRANSFER_SIZE 0x8000 //32KB

bool spi_read(void *pDest, size_t srcOffset, size_t byteCount)
{
    uint8_t *pDestBytes = (uint8_t *)pDest;
    size_t totalRead = 0;

    while (totalRead < byteCount) {
        size_t bytesToRead = (byteCount - totalRead) < MAX_TRANSFER_SIZE ? (byteCount - totalRead) : MAX_TRANSFER_SIZE;
        FLASH_read(srcOffset + totalRead, pDestBytes + totalRead, bytesToRead);
        totalRead += bytesToRead;
    }
      
    return true;
}


/**
 * @brief Writes data to the SPI flash memory at the specified address.
 *        The data is written starting from the global flash address plus the given address offset.
 *        Prints the address and length of the data being written.
 */
bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
    FLASH_program(dstOffset, pSrc, byteCount);
    return true;
}

void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount)
{
    uint32_t sectorSize = 0x10000; //64KB
    *pEraseSize = *pBlockSize = sectorSize;
}

/**
 * @brief Initializes the spi.
 */
bool spi_init(void)
{
    uint8_t manufacturer_id, device_id;
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);
   mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n\r **** SPI Init **** \n\
    \r  - Device ID: %u \n\
    \r  - Manufacturer ID: %u \n\n", device_id, manufacturer_id);
    //execute_designinfo_service();
    //MSS_UART_polled_tx_string (&g_mss_uart1_lo, g_greeting_msg);

    return true;
}

