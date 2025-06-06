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
#include "hss_md5.h"
#include <string.h>
#include <assert.h>
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mt25ql01gbbb/micron1gflash.h"

/*----------------------------Constant Definitions--------------------------*/
#define SPI_SLOT_SIZE           (10 * 1024 * 1024)  // 10 MB
#define BLOCK_SIZE_SPI          (64 * 1024)         // 64 KB
#define MAX_TRANSFER_SIZE       (32 * 1024)         // 32 KB
#define BLOCK_SIZE_EMMC         512
#define BLOCK_SIZE_CRC          512
#define STREAM_GEN_BASE_ADDR    0x4A000000u

typedef enum {
    DDR_DIS = (1 << 1),    // Bit 1
    EMMPR_EN = (1 << 8),   // Bit 8
    EMMSC_EN = (1 << 9),   // Bit 9
    SW_EN = (1 << 10),     // Bit 10
    SW_DIS = (1 << 11),    // Bit 11
    SW_SEL0 = (1 << 12),   // Bit 12
    SW_SEL1 = (1 << 13),   // Bit 13
    P3V5_PG = (1 << 14),   // Bit 14
    P2V0_PG = (1 << 15),   // Bit 15
    P1V5_PG = (1 << 16),   // Bit 16
    DDR_PG = (1 << 17),    // Bit 17
} RegisterBits;

typedef struct {
    uint8_t LastFailed;
    uint8_t CurrentTry;
    uint8_t LastSuccessful;
    uint8_t BootSequence[4];
    uint32_t CRC;
    uint8_t ignore_CRC;
    uint8_t BootReport[4];
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

#define BLOCK_SIZE_BYTES 512
size_t ParamRegionOffset = EMMC4_PADDR;

bool get_ignore_crc(void)
{
#if IS_ENABLED(CONFIG_SERVICE_IGNORE_CRC) 
    return true;
#else
    return (Params.ignore_CRC == 0xFF);
#endif
}

uint64_t get_offset(uint8_t slot) 
{
    switch (slot) {
        case 10: case 20:
            return PAYLOAD_1;
        case 11: case 21:
            return PAYLOAD_2;
        default:
            return PAYLOAD_1;
    }
}


uint8_t get_boot_sequence(uint8_t index)
{
    return Params.BootSequence[index];
}

const char* getBootDeviceName(uint8_t id)
{
    switch (id) {
        case 10: case 11: case 12: case 13:
            return "EMMC1";
        case 20: case 21: case 22: case 23:
            return "EMMC2";
        default:
            return "SPI_FLASH";
    }
}

/**
 * @brief Function to set a specific bit to 1
 */
void set_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    //DEBUG_PRINT("\n\r - Setting bit %d in register %08x", bit, *register_map_outputs);
    *register_map_outputs |= bit; // Set the bit
}

/**
 * @brief Function to clear a specific bit (with constraints)
 */
void clear_register_bit(uint32_t* register_map_outputs, RegisterBits bit)
{
    //DEBUG_PRINT("\n\r - Clearing bit %d in register %08x", bit, *register_map_outputs);
    *register_map_outputs &= ~bit; // Clear the bit
}


/**
 * @brief Function to enable specific emmc
 */
void enable_emmc(uint8_t emmc_id)
{
    switch (emmc_id) {
        case EMMC_PRIMARY:
            clear_register_bit(stream_gen_base_register, SW_SEL0);
            clear_register_bit(stream_gen_base_register, SW_SEL1);
            clear_register_bit(stream_gen_base_register, EMMPR_EN);
            set_register_bit(stream_gen_base_register, EMMSC_EN);
            clear_register_bit(stream_gen_base_register, SW_EN);
            //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Primary eMMC enabled\n");
            break;

        case EMMC_SECONDARY:
            set_register_bit(stream_gen_base_register, SW_SEL0);
            set_register_bit(stream_gen_base_register, SW_SEL1);
            set_register_bit(stream_gen_base_register, EMMPR_EN);
            clear_register_bit(stream_gen_base_register, EMMSC_EN);
            clear_register_bit(stream_gen_base_register, SW_EN);
            //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Secondary eMMC enabled \n");
            break;

        default:
             //mHSS_DEBUG_PRINTF(LOG_ERROR,"Invalid eMMC ID \n");
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
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    for (uint32_t index = 0; index < SPI_SLOT_SIZE; index += BLOCK_SIZE_SPI)
    {
        FLASH_erase_64k_block(address + index);
        delay1(500);
    }
#endif
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
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);
#endif

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "SPI Init: Device ID: %u, Manufacturer ID: %u\n", device_id, manufacturer_id);

    initialized = true;  // Mark as initialized

    return true;
}

bool spi_read(void *pDest, size_t srcOffset, size_t byteCount)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    uint8_t *pDestBytes = (uint8_t *)pDest;
    size_t totalRead = 0;

    while (totalRead < byteCount) {
        size_t bytesToRead = (byteCount - totalRead) < MAX_TRANSFER_SIZE ? (byteCount - totalRead) : MAX_TRANSFER_SIZE;
        FLASH_read(srcOffset + totalRead, pDestBytes + totalRead, bytesToRead);
        totalRead += bytesToRead;
    }
#endif
    return true;
}

bool spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_program(dstOffset, pSrc, byteCount);
#endif
    return true;
}

void HSS_slot_restore_boot_sequence(void)
{
    Params.BootSequence[0] = 0;
    Params.ignore_CRC = 0;
    Params.CRC = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;
    memcpy(buff, &Params, sizeof(Params));
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_erase_4k_block(PARAM_PADDR);
    delay1(500);
    spi_write(PARAM_PADDR, buff, sizeof(Params));
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Params restored\n");
#else 
    HSS_MMC_WriteBlock(ParamRegionOffset, buff, BLOCK_SIZE_BYTES);
#endif
}

void HSS_slot_update_boot_params(int index, boot_error_codes code)
{
    Params.LastFailed = index;
    Params.CurrentTry = index+1;
    Params.LastSuccessful = 0;
   // Params.BootSequence[0] = 10;
    Params.BootSequence[1] = EMMC_PRIMARY;
    Params.BootSequence[2] = EMMC_SECONDARY;
    Params.BootSequence[3] = SPI_FLASH;
    //Params.ignore_CRC = 0xFF;
    Params.BootReport[Params.LastFailed] = code;

    Params.CRC = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;
    memcpy(buff, &Params, sizeof(Params));

    /*mHSS_DEBUG_PRINTF(LOG_NORMAL,"update LastFailed: %d\n", Params.LastFailed);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update CurrentTry: %d\n", Params.CurrentTry);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update LastSuccessful: %d\n", Params.LastSuccessful);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update BootReport[%d]: %d\n", Params.LastFailed, code);*/

   // mHSS_DEBUG_PRINTF(LOG_NORMAL,"CRC: 0x%X\n", Params.CRC);
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_erase_4k_block(PARAM_PADDR);
    delay1(500);
    spi_write(PARAM_PADDR, buff, sizeof(Params));
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot parameters update with crc %x\n", crc);
#else 
    HSS_MMC_WriteBlock(ParamRegionOffset, buff, BLOCK_SIZE_BYTES);
#endif
}

void copyBufferToParamData(const uint8_t* buffer, ParamData* params);
void copyBufferToParamData(const uint8_t* buffer, ParamData* params)
{
    params->LastFailed = buffer[0];
    params->CurrentTry = buffer[1];
    params->LastSuccessful = buffer[2];
    params->BootSequence[0] = buffer[3];
    params->BootSequence[1] = EMMC_PRIMARY;  // EMMC1
    params->BootSequence[2] = EMMC_SECONDARY;  // EMMC2
    params->BootSequence[3] = SPI_FLASH;  // SPI_FLASH
    params->CRC = (buffer[11] << 24) |
                 (buffer[10] << 16) |
                 (buffer[9] << 8)  |
                  buffer[8];
    params->ignore_CRC = buffer[12];
    params->BootReport[0] = buffer[13];
    params->BootReport[1] = buffer[14];
    params->BootReport[2] = buffer[15];
    params->BootReport[3] = buffer[16];
}

void HSS_slot_get_boot_params(void)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    spi_init();
    spi_read(&buff, PARAM_PADDR, sizeof(Params));
#else
    HSS_MMCInit();
    HSS_MMC_ReadBlock(&buff, ParamRegionOffset, BLOCK_SIZE_BYTES);
#endif
    copyBufferToParamData(buff, &Params);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Ignore CRC: %d\n",  Params.ignore_CRC);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Sequence[]: %d, %d, %d, %d\n",
           Params.BootSequence[0],
           Params.BootSequence[1],
           Params.BootSequence[2],
           Params.BootSequence[3]);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Report[]:   %d, %d, %d, %d\n",
            Params.BootReport[0],
            Params.BootReport[1],
            Params.BootReport[2],
            Params.BootReport[3]);

    buff[11] = 0;
    buff[10] = 0;
    buff[9] = 0;
    buff[8] = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));

    if(crc != Params.CRC) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,"Boot params failed CRC:  0x%X\n", crc);
    }else{
        mHSS_DEBUG_PRINTF(LOG_STATUS,"Boot params passed CRC:  0x%X\n", crc);
    }
}
void print_md5(const char *label, const uint8_t *hash);
void print_md5(const char *label, const uint8_t *hash) {
    mHSS_DEBUG_PRINTF(LOG_STATUS, "%s: ", label);
    for (int i = 0; i < 16; i++) {
        mHSS_PRINTF("%02x", hash[i]);
    }
    mHSS_PRINTF("\n");
}
bool compare_md5(const uint8_t *a, const uint8_t *b);
bool compare_md5(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 16; i++) {
        if (a[i] != b[i]) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "Error: MD5 mismatch\n");
            return false;
        }
    }
    return true;
}

bool validateCrc_custom_emmc(struct HSS_BootImage *pImage, size_t offset, const char * name)
{
    uint8_t temp_buffer[BLOCK_SIZE_EMMC] = {0};
    uint32_t block_offset = 0;
    int8_t status = MSS_MMC_TRANSFER_SUCCESS;
    bool result = true;
    uint32_t start_addr = offset;
    MD5Context ctx;
    uint16_t digest_absolute_offset = 1488;
    uint16_t md5_index = digest_absolute_offset / BLOCK_SIZE_EMMC;
    uint16_t md5_offset = digest_absolute_offset % BLOCK_SIZE_EMMC;

    md5Init(&ctx);
    mHSS_DEBUG_PRINTF(LOG_STATUS, "%s image length: 0x%0X (%d)\n",name, pImage->bootImageLength, pImage->bootImageLength);
    for (uint32_t bytes_read = 0; bytes_read < pImage->bootImageLength; bytes_read += BLOCK_SIZE_EMMC)
    {
        status = MSS_MMC_single_block_read(start_addr + block_offset, (uint32_t *)temp_buffer);
        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {
            if (block_offset == md5_index) {
                memset(&temp_buffer[md5_offset], 0, 16);
            }

            size_t remaining = pImage->bootImageLength - bytes_read;
            size_t chunk_size = (remaining < BLOCK_SIZE_EMMC) ? remaining : BLOCK_SIZE_EMMC;

            md5Update(&ctx, temp_buffer, chunk_size);
            block_offset++;
        }
        else
        {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "error reading eMMC blocks\n");
        }
    }
    
    md5Finalize(&ctx);
    print_md5("MD5 read", pImage->signature.digest);
    print_md5("MD5 calc", ctx.digest);
    result = compare_md5(ctx.digest, pImage->signature.digest);

    return result;
}

#if IS_ENABLED(CONFIG_SERVICE_SPI)
static uint8_t spi_checkCrc(uint32_t headerCrc_read, uint32_t headerCrc_calculated);
static void spi_readCrc(uint32_t start_addr, uint32_t *headerCrc);
static void spi_CalculateCrc(uint32_t start_addr, uint32_t bootImageLength, uint32_t *headerCrc);
static void spi_readBootImageLength(uint32_t start_addr, uint32_t *bootImageLength);
#endif
bool validateCrc_custom_spi(struct HSS_BootImage *pImage)
{
    uint8_t status = 0;
#if IS_ENABLED(CONFIG_SERVICE_SPI)
   uint32_t headerCrc_read = 0;
   uint32_t headerCrc_calculated = 0;
   uint32_t bootImageLength = 0;
   uint32_t start_addr = SPI0_PADDR;


    spi_readCrc(start_addr, &headerCrc_read);
    spi_readBootImageLength(start_addr, &bootImageLength);
    spi_CalculateCrc(start_addr, bootImageLength, &headerCrc_calculated);
    status = spi_checkCrc(headerCrc_read, headerCrc_calculated);

    pImage->headerCrc = headerCrc_calculated;
#endif
    return status;
}
#if IS_ENABLED(CONFIG_SERVICE_SPI)
static uint8_t spi_checkCrc(uint32_t headerCrc_read, uint32_t headerCrc_calculated) {

    uint8_t status;

    if(headerCrc_read == headerCrc_calculated){
        status = true;
        mHSS_DEBUG_PRINTF(LOG_STATUS, "SPI image passed CRC: 0x%0X\n", headerCrc_calculated);
    }else {
        status = false;
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI image failed CRC: 0x%0X\n", headerCrc_calculated);
    }
    return status;
}

static void spi_readCrc(uint32_t start_addr, uint32_t *headerCrc) {
    uint8_t header_buffer[BLOCK_SIZE_CRC] = {0};
    FLASH_read(start_addr, header_buffer, (size_t)BLOCK_SIZE_CRC);

     *headerCrc = (header_buffer[19] << 24) |         // Save the CRC read from the header
                  (header_buffer[18] << 16) |
                  (header_buffer[17] << 8)  |
                   header_buffer[16];
}

static void spi_readBootImageLength(uint32_t start_addr, uint32_t *bootImageLength) {
    uint8_t header_buffer[BLOCK_SIZE_CRC] = {0};
    FLASH_read(start_addr + (BLOCK_SIZE_CRC * 2), header_buffer, (size_t)BLOCK_SIZE_CRC);
    *bootImageLength = (header_buffer[459] << 24) |
                       (header_buffer[458] << 16) |
                       (header_buffer[457] << 8)  |
                        header_buffer[456];
    mHSS_DEBUG_PRINTF(LOG_STATUS, "SPI image length: 0x%0X (%d)\n", *bootImageLength, *bootImageLength);
}

static void spi_CalculateCrc(uint32_t start_addr, uint32_t bootImageLength, uint32_t *headerCrc) {
    uint8_t temp_buffer[BLOCK_SIZE_CRC] = {0};
    uint32_t block_offset = 0;

    *headerCrc = 0;
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Calculating CRC...\n");
    for (uint32_t bytes_read = 0; bytes_read < bootImageLength; bytes_read += BLOCK_SIZE_CRC) {
        FLASH_read(start_addr + block_offset, temp_buffer, (size_t)BLOCK_SIZE_CRC);
        if(block_offset == 0)
        {
            for (int i = 16; i < 20; i++) {
                temp_buffer[i] = 0;         // Clear the CRC bytes in the header
            }
        }
        *headerCrc = CRC32_calculate_ex(*headerCrc, (const uint8_t *)&temp_buffer, BLOCK_SIZE_CRC);
        block_offset= block_offset + BLOCK_SIZE_CRC;
    }
}
#endif
