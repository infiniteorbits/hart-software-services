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
#include "thirdparty/md5/md5.h"
#include <string.h>
#include <assert.h>
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mt25ql01gbbb/micron1gflash.h"

/*----------------------------Constant Definitions--------------------------*/
#define SPI_SLOT_SIZE           (10 * 1024 * 1024)  // 10 MB
#define BLOCK_SIZE_SPI          (64 * 1024)         // 64 KB
#define MAX_TRANSFER_SIZE       (32 * 1024)         // 32 KB
#define BLOCK_SIZE              512
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
    uint8_t linux_LastFailed;
    uint8_t linux_CurrentTry;
    uint8_t linux_current_sw;
    uint8_t linux_boot_sequence;
    uint8_t linux_verify_payload;
    uint8_t freertos_LastFailed;
    uint8_t freertos_CurrentTry;
    uint8_t freertos_current_sw;
    uint8_t freertos_boot_sequence;
    uint8_t freertos_verify_payload;
} BootSoftwareParams;

/*------------------------------Local Variables-----------------------------*/
static BootSoftwareParams Params;
static uint8_t buff[sizeof(BootSoftwareParams)];
static uint32_t* const stream_gen_base_register = (uint32_t*)(STREAM_GEN_BASE_ADDR + 0x10);

/*-----------------------------Local Functions------------------------------*/
static void HSS_slot_set_register_bit(uint32_t* reg, RegisterBits bit);
static void HSS_slot_clear_register_bit(uint32_t* reg, RegisterBits bit);
static void HSS_slot_print_md5(const char *label, const uint8_t *hash);
static bool HSS_slot_compare_md5(const uint8_t *a, const uint8_t *b);

static void HSS_slot_set_register_bit(uint32_t* reg, RegisterBits bit) {
    *reg |= bit;
}

static void HSS_slot_clear_register_bit(uint32_t* reg, RegisterBits bit) {
    *reg &= ~bit;
}

static void HSS_slot_print_md5(const char *label, const uint8_t *hash) {
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "%s: ", label);
    for (int i = 0; i < 16; i++) {
        mHSS_PRINTF("%02x", hash[i]);
    }
    mHSS_PRINTF("\n");
}

static bool HSS_slot_compare_md5(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 16; i++) {
        if (a[i] != b[i]) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 mismatch\n");
            return false;
        }
    }
    mHSS_DEBUG_PRINTF(LOG_STATUS, "MD5 passed\n");
    return true;
}

uint8_t HSS_slot_get_boot_sequence(uint8_t index) {
    switch(index)
    {
        case 0:
            return Params.freertos_boot_sequence;
        case 1:
            return EMMC_PRIMARY;
        case 2:
            return EMMC_SECONDARY;
        case 3:
            return SPI_FLASH;
        case 4:
            return QSPI;
        default:
            return QSPI;
    }
}

bool HSS_slot_get_verify_payload(void)
{
#if IS_ENABLED(CONFIG_SERVICE_verify_payload) 
    return true;
#else
    return (Params.freertos_verify_payload == 0xFF);
#endif
}

uint64_t HSS_slot_get_offset(uint8_t slot) 
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

void HSS_slot_enable_emmc(uint8_t emmc_id)
{
    switch (emmc_id) {
        case EMMC_PRIMARY:
            HSS_slot_clear_register_bit(stream_gen_base_register, SW_SEL0);
            HSS_slot_clear_register_bit(stream_gen_base_register, SW_SEL1);
            HSS_slot_clear_register_bit(stream_gen_base_register, EMMPR_EN);
            HSS_slot_set_register_bit(stream_gen_base_register, EMMSC_EN);
            HSS_slot_clear_register_bit(stream_gen_base_register, SW_EN);
            //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Primary eMMC enabled\n");
            break;

        case EMMC_SECONDARY:
            HSS_slot_set_register_bit(stream_gen_base_register, SW_SEL0);
            HSS_slot_set_register_bit(stream_gen_base_register, SW_SEL1);
            HSS_slot_set_register_bit(stream_gen_base_register, EMMPR_EN);
            HSS_slot_clear_register_bit(stream_gen_base_register, EMMSC_EN);
            HSS_slot_clear_register_bit(stream_gen_base_register, SW_EN);
            //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Secondary eMMC enabled \n");
            break;

        default:
             //mHSS_DEBUG_PRINTF(LOG_ERROR,"Invalid eMMC ID \n");
            break;
    }
}

void HSS_slot_restore_boot_sequence(void)
{
    Params.freertos_boot_sequence = 0;
    Params.freertos_verify_payload = 0;
    memcpy(buff, &Params, sizeof(Params));
    HSS_MMC_WriteBlock((size_t)(PARAM_REGION), buff, BLOCK_SIZE);
}

void HSS_slot_update_boot_params(int index, boot_error_codes code)
{
    Params.freertos_LastFailed = index;
    Params.freertos_CurrentTry = index+1;
    Params.freertos_current_sw = 0;
    memcpy(buff, &Params, sizeof(Params));
    HSS_MMC_WriteBlock((size_t)(PARAM_REGION), buff, BLOCK_SIZE);
}

void HSS_slot_get_boot_params(void)
{
    HSS_MMCInit();
    HSS_MMC_ReadBlock(&buff, (size_t)(PARAM_REGION), BLOCK_SIZE);
    memcpy(&Params, buff, sizeof(BootSoftwareParams));
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Ignore CRC: %d\n",  Params.freertos_verify_payload);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Sequence[]: %d, 10, 20, 30, 40\n", Params.freertos_boot_sequence);
}

bool HSS_slot_validate_md5(struct HSS_BootImage *pImage, size_t offset, memory_type_t mem_type)
{
    uint8_t temp_buffer[BLOCK_SIZE] = {0};
    uint32_t block_offset = 0;
    bool result = true;
    uint32_t start_addr = offset;
    MD5Context ctx;
    uint16_t digest_absolute_offset = 1488;
    uint16_t md5_index = digest_absolute_offset / BLOCK_SIZE;
    uint16_t md5_offset = digest_absolute_offset % BLOCK_SIZE;

    md5_init(&ctx);
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Bootimage length: 0x%0X (%d)\n", pImage->bootImageLength, pImage->bootImageLength);

    for (uint32_t bytes_read = 0; bytes_read < pImage->bootImageLength; bytes_read += BLOCK_SIZE)
    {
        int8_t status = -1;

        switch (mem_type) {
            case EMMC_PRIMARY:
            case EMMC_SECONDARY:
                status = MSS_MMC_single_block_read(start_addr + block_offset, (uint32_t *)temp_buffer);
                break;
            case SPI_FLASH:
                status = true;
                FLASH_read(start_addr + (block_offset * BLOCK_SIZE), temp_buffer, BLOCK_SIZE);
                break;
            case QSPI:
                status = true;
                HSS_QSPI_ReadBlock(temp_buffer, start_addr + (block_offset * BLOCK_SIZE), BLOCK_SIZE);
                break;
            default:
                mHSS_DEBUG_PRINTF(LOG_ERROR, "Invalid memory type!\n");
                return false;
        }

        if (status) {
            if (block_offset == md5_index) {
                memset(&temp_buffer[md5_offset], 0, 16);
            }

            size_t remaining = pImage->bootImageLength - bytes_read;
            size_t chunk_size = (remaining < BLOCK_SIZE) ? remaining : BLOCK_SIZE;

            md5_update(&ctx, temp_buffer, chunk_size);
            block_offset++;
        } else {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "Error reading block at offset 0x%X\n", start_addr + block_offset);
            return false;
        }
    }

    md5_finalize(&ctx);
    HSS_slot_print_md5("MD5 read", pImage->signature.digest);
    HSS_slot_print_md5("MD5 calc", ctx.digest);
    result = HSS_slot_compare_md5(ctx.digest, pImage->signature.digest);

    return result;
}


void HSS_slot_spi_get_info(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount) {
    uint32_t sectorSize = 0x10000; //64KB
    *pEraseSize = *pBlockSize = sectorSize;
}

bool HSS_slot_spi_init(void)
{
    static bool initialized = false;

    if (initialized) {
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

bool HSS_slot_spi_read(void *pDest, size_t srcOffset, size_t byteCount)
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

bool HSS_slot_spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_program(dstOffset, pSrc, byteCount);
#endif
    return true;
}