/******************************************************************************
 * Copyright (c) 2024 Infinite Orbits. All rights reserved.
 *
 * @file hss_slot_selection.c
 * @brief Implementation of slot selection.
 *
 * This file contains the implementation of the slot selection functionality,
 * which allows booting up a sepcific image in a specific slot region.
 *
 * @authors
 * - A. Tarragó (abel.tarrago@ixrev.com)
 *
 * @version
 * - 1.0: Initial version only emmc1
 * - 1.1: Add spi-flash
 * - 1.2: Add emmc2
 * 
 * @date
 * - 2024-09-10: Created
 *****************************************************************************/

/*---------------------------------Includes---------------------------------*/
#include "config.h"
#include "hss_types.h"
#include "hss_init.h"
#include "hss_slot_selection.h"
#include "mmc_service.h"
#include "mss_mmc.h"
#include "hss_crc32.h"
#include "hss_debug.h"
#include <string.h>
#include <assert.h>
#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"

/*----------------------------Constant Definitions--------------------------*/
#define SPI_SLOT_SIZE           (10 * 1024 * 1024)  // 10 MB
#define BLOCK_SIZE_SPI          (64 * 1024)         // 64 KB
#define MAX_TRANSFER_SIZE       (32 * 1024)         // 32 KB
#define BLOCK_SIZE_EMMC         512
#define BLOCK_SIZE_CRC          512
#define STREAM_GEN_BASE_ADDR    0x4A000000u

typedef enum {
    DDR_DIS = 1,
    EMMPR_EN = 8,   // Bit 8
    EMMSC_EN = 9,   // Bit 9
    SW_EN = 10,     // Bit 10
    SW_DIS = 11,    // Bit 11
    SW_SEL0 = 12,   // Bit 12
    SW_SEL1 = 13,   // Bit 13
    P3V5_PG = 14,   // Bit 14
    P2V0_PG = 15,   // Bit 15
    P1V5_PG = 16,   // Bit 16
    DDR_PG = 17     // Bit 17
} RegisterBits;

typedef struct {
    uint8_t LastFailed;
    uint8_t CurrentTry;
    uint8_t LastSuccessful;
    uint8_t BootSequence[3];
    uint32_t CRC;
} ParamData;

/*------------------------------Local Variables-----------------------------*/
ParamData Params;
uint8_t buff[sizeof(Params)];
uint32_t* stream_gen_base_register = (void*)(STREAM_GEN_BASE_ADDR + 0x10);

/*-----------------------------Local Functions------------------------------*/
const char* getBootDeviceName(uint8_t id);
void delay1(volatile uint32_t n);
void set_register_bit(uint32_t* register_map_outputs, RegisterBits bit);
void clear_register_bit(uint32_t* register_map_outputs, RegisterBits bit);

const char* getBootDeviceName(uint8_t id)
{
    switch (id) {
        case 0x01:
            return "EMMC1";
        case 0x02:
            return "EMMC2";
        case 0x03:
            return "SPI_FLASH";
        default:
            return "None";
    }
}

/**
 * @brief Function to set a specific bit to 1
 */
void set_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    uint32_t reg_value = *register_map_outputs; // Read the current value of the register

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r set bit %d of register_map_outputs = %08x", bit, *register_map_outputs);

    switch (bit)
    {
        case EMMPR_EN:
            reg_value |= (1 << EMMPR_EN);
            break;
        case EMMSC_EN:
            reg_value |= (1 << EMMSC_EN);
            break;
        case SW_EN:
            reg_value |= (1 << SW_EN);
            break;
        case SW_SEL0:
            reg_value |= (1 << SW_SEL0);
            break;
        case SW_SEL1:
            reg_value |= (1 << SW_SEL1);
            break;
        default:
            return; // If the bit is not one of the defined ones, do nothing
    }

    *register_map_outputs = reg_value; // Write the new value to the register*/
}

/**
 * @brief Function to clear a specific bit to 0
 */
void clear_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    uint32_t reg_value = *register_map_outputs; // Read the current value of the register

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r clear bit %d of register_map_outputs = %08x", bit, *register_map_outputs);

    switch (bit)
    {
        case EMMPR_EN:
            reg_value &= ~(1 << EMMPR_EN);
            break;
        case EMMSC_EN:
            reg_value &= ~(1 << EMMSC_EN);
            break;
        case SW_EN:
            reg_value &= ~(1 << SW_EN);
            break;
        case SW_SEL0:
            reg_value &= ~(1 << SW_SEL0);
            break;
        case SW_SEL1:
            reg_value &= ~(1 << SW_SEL1);
            break;
        default:
            return; // If the bit is not one of the defined ones, do nothing
    }

    *register_map_outputs = reg_value; // Write the new value to the register
}

/**
 * @brief Function to enable specific emmc
 */
void enable_emmc(uint8_t emmc_id)
{
    mss_mmc_status_t ret_status;

    switch (emmc_id) {
        case EMMC_PRIMARY:
            clear_register_bit(stream_gen_base_register, SW_SEL0);
            clear_register_bit(stream_gen_base_register, SW_SEL1);
            set_register_bit(stream_gen_base_register, EMMPR_EN);
            clear_register_bit(stream_gen_base_register, EMMSC_EN);
            set_register_bit(stream_gen_base_register, SW_EN);

            ret_status = HSS_MMCInit();
            if (ret_status == 0) {
                 mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r - Primary eMMC enabled");
            } else {
                 mHSS_DEBUG_PRINTF(LOG_ERROR,"\n \r - Primary eMMC init Failed");
            }
            break;

        case EMMC_SECONDARY:
            set_register_bit(stream_gen_base_register, SW_SEL0);
            set_register_bit(stream_gen_base_register, SW_SEL1);
            clear_register_bit(stream_gen_base_register, EMMPR_EN);
            set_register_bit(stream_gen_base_register, EMMSC_EN);
            set_register_bit(stream_gen_base_register, SW_EN);

            ret_status = HSS_MMCInit();
            if (ret_status == 0) {
                 mHSS_DEBUG_PRINTF(LOG_NORMAL,"\n \r - Secondary eMMC enabled");
            } else {
                 mHSS_DEBUG_PRINTF(LOG_ERROR,"\n \r - Secondary eMMC init Failed");
            }
            break;

        default:
             mHSS_DEBUG_PRINTF(LOG_ERROR,"\n\r - Invalid eMMC ID");
            break;
    }
}

void delay1(volatile uint32_t n)
{
    while (n)
        n--;
}

void erase_section(uint32_t address) 
{
    for (uint32_t index = 0; index < SPI_SLOT_SIZE; index += BLOCK_SIZE_SPI)
    {
        FLASH_erase_64k_block(address + index);
        delay1(500);
    }
}


void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount)
{
    uint32_t sectorSize = 0x10000; //64KB
    *pEraseSize = *pBlockSize = sectorSize;
}

bool spi_init(void)
{
    static bool initialized = false; // Static flag to check if initialization has already been done

    if (initialized) {
        // If already initialized, exit without repeating the initialization
        return true;
    }

    uint8_t manufacturer_id = 0, device_id = 0;
    /*FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);*/

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "\n\r **** SPI Init **** \n\
    \r  - Device ID: %u \n\
    \r  - Manufacturer ID: %u \n\n", device_id, manufacturer_id);

    initialized = true;  // Mark as initialized

    return true;
}

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

bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
    FLASH_program(dstOffset, pSrc, byteCount);
    return true;
}


void HSS_slot_update_boot_params(int index)
{
    Params.LastFailed = index;
    Params.CurrentTry = index+1;
    Params.LastSuccessful = Params.LastSuccessful;
    Params.BootSequence[0] = EMMC_PRIMARY;
    Params.BootSequence[1] = EMMC_SECONDARY;
    Params.BootSequence[2] = SPI_FLASH;

    Params.CRC = 0;
    memcpy(buff, &Params, sizeof(Params));

    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;

    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastFailed: %s\n", getBootDeviceName(Params.LastFailed));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"CurrentTry: %s\n", getBootDeviceName(Params.CurrentTry));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"LastSuccessful: %s\n", getBootDeviceName(Params.LastSuccessful));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"BootSequence: %s, %s, %s\n ", 
        getBootDeviceName(Params.BootSequence[0]),
        getBootDeviceName(Params.BootSequence[1]), 
        getBootDeviceName(Params.BootSequence[2]));
    
    /*FLASH_erase_64k_block(BOOT_PARAMS_PADDR);
    delay1(500);
    spi_write(BOOT_PARAMS_PADDR, buff, sizeof(Params));*/

}

void HSS_slot_get_boot_params(void)
{
    //spi_init();
    //spi_read(&buff, BOOT_PARAMS_PADDR, sizeof(Params));
    Params.LastFailed = buff[0];
    Params.CurrentTry = buff[1];
    Params.LastSuccessful = buff[2];
    Params.BootSequence[0] = EMMC_PRIMARY;
    Params.BootSequence[1] = EMMC_SECONDARY;
    Params.BootSequence[2] = SPI_FLASH;
    Params.CRC = (buff[6] << 24) |
                 (buff[7] << 16) |
                 (buff[8] << 8)  |
                  buff[9];
    
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = 0;
    buff[9] = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));

    if(crc != Params.CRC)
    {
        mHSS_DEBUG_PRINTF(LOG_ERROR,"Boot parameters CRC: expected %x | calculated %x\n", Params.CRC, crc);
    }
}

bool validateCrc_custom_emmc(struct HSS_BootImage *pImage)
{
    uint8_t temp_buffer[BLOCK_SIZE_EMMC] = {0};
    uint8_t header_buffer[BLOCK_SIZE_EMMC] = {0};
    uint32_t block_offset = 0;
    uint32_t CRC_read = 0;
    uint32_t CRC_calculated = 0;
    uint32_t bootImageLength = 0;
    int8_t status = 0;
    bool result = false;
    uint32_t start_addr = EMMC0_PADDR / BLOCK_SIZE_EMMC;

    status = MSS_MMC_single_block_read(start_addr, (uint32_t*)header_buffer);
    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        CRC_read =  (header_buffer[19] << 24) |
                    (header_buffer[18] << 16) |
                    (header_buffer[17] << 8)  |
                    header_buffer[16];     

        status = MSS_MMC_single_block_read((start_addr + 2), (uint32_t*)header_buffer);

        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {                
            bootImageLength =   (header_buffer[459] << 24) |
                                (header_buffer[458] << 16) |
                                (header_buffer[457] << 8)  |
                                header_buffer[456];      

            //mHSS_DEBUG_PRINTF(LOG_NORMAL, "bootImageLength: 0x%0X (%d)\n", bootImageLength, bootImageLength);

            for (uint32_t bytes_read = 0; bytes_read < bootImageLength ; bytes_read += BLOCK_SIZE_EMMC )
            {
                status = MSS_MMC_single_block_read(start_addr + block_offset, (uint32_t*)temp_buffer);
                if (status == MSS_MMC_TRANSFER_SUCCESS)
                { 
                    if(block_offset == 0)
                    {
                        temp_buffer[19] = 0;
                        temp_buffer[18] = 0;
                        temp_buffer[17] = 0;
                        temp_buffer[16] = 0;
                    }
                    CRC_calculated = CRC32_calculate_ex(CRC_calculated, (const uint8_t *)&temp_buffer, BLOCK_SIZE_EMMC);
                    block_offset++;
                }else {
                    mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading eMMC blocks\n");
                }
            }

            //mHSS_DEBUG_PRINTF(LOG_NORMAL, "CRC_calculated: 0x%0X\n", CRC_calculated); 

        }else{
            mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading bootImageLength\n");
        }               
    
    } else{
        mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading header\n");
    }    

    if (CRC_read == CRC_calculated) {
        result = true;
    } else {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Checked HSS_BootImage header CRC: calculated %08x vs expected %08x\n",
            CRC_calculated, CRC_read);
    }

    pImage->headerCrc = CRC_calculated;

    return result;
}

bool validateCrc_custom_spi(struct HSS_BootImage *pImage)
{
    uint32_t block_offset = 0;
    uint32_t bootImageLength = 0;
    uint32_t start_addr = SPI0_PADDR;
    uint32_t CRC_read = 0;
    uint32_t CRC_calculated = 0;
    uint8_t header_buffer[BLOCK_SIZE_CRC] = {0};
    uint8_t temp_buffer[BLOCK_SIZE_CRC] = {0};
    bool result;

    // 1. Read the CRC
    FLASH_read(start_addr, header_buffer, (size_t)BLOCK_SIZE_CRC);
    CRC_read =  (header_buffer[19] << 24) |
            (header_buffer[18] << 16) |
            (header_buffer[17] << 8)  |
            header_buffer[16]; 

    // 2. Read the boot image length
    FLASH_read(start_addr, header_buffer, (size_t)BLOCK_SIZE_CRC);
    bootImageLength = (header_buffer[459] << 24) |
                        (header_buffer[458] << 16) |
                        (header_buffer[457] << 8)  |
                        header_buffer[456];
    //mHSS_DEBUG_PRINTF(LOG_NORMAL, "bootImageLength: 0x%0X (%d)\n", bootImageLength, bootImageLength);
 
    // 3. Read and calculate the CRC
    for (uint32_t bytes_read = 0; bytes_read < bootImageLength; bytes_read += BLOCK_SIZE_CRC) {
        if(block_offset == 0)
        {
            temp_buffer[19] = 0;
            temp_buffer[18] = 0;
            temp_buffer[17] = 0;
            temp_buffer[16] = 0;
        }
        FLASH_read(start_addr + block_offset, temp_buffer, (size_t)BLOCK_SIZE_CRC);
        CRC_calculated = CRC32_calculate_ex(CRC_calculated, (const uint8_t *)&temp_buffer, BLOCK_SIZE_CRC);
        block_offset++;
    }
    //mHSS_DEBUG_PRINTF(LOG_NORMAL, "CRC_calculated: 0x%0X\n", CRC_calculated); 

    // 4. Check and return result
    if (CRC_read == CRC_calculated) {
        result = true;
    } else {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Checked HSS_BootImage header CRC: calculated %08x vs expected %08x\n",
            CRC_calculated, CRC_read);
    }

    pImage->headerCrc = CRC_calculated;

    return result;
}