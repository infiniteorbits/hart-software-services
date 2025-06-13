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
    uint8_t LastFailed;
    uint8_t CurrentTry;
    uint8_t current_sw;
    uint8_t boot_seq_rtos[5];
    uint32_t CRC;
    uint8_t verify_payload;
    uint8_t payload_ver_status[5];
} ParamData;

/*------------------------------Local Variables-----------------------------*/
static ParamData Params;
static uint8_t buff[sizeof(ParamData)];
static uint32_t* const stream_gen_base_register = (uint32_t*)(STREAM_GEN_BASE_ADDR + 0x10);

/*-----------------------------Local Functions------------------------------*/
static void set_register_bit(uint32_t* reg, RegisterBits bit);
static void clear_register_bit(uint32_t* reg, RegisterBits bit);
static void copyBufferToParamData(const uint8_t* buffer, ParamData* params);
static void print_md5(const char *label, const uint8_t *hash);
static bool compare_md5(const uint8_t *a, const uint8_t *b);

static void set_register_bit(uint32_t* reg, RegisterBits bit) {
    *reg |= bit;
}

static void clear_register_bit(uint32_t* reg, RegisterBits bit) {
    *reg &= ~bit;
}

static void print_md5(const char *label, const uint8_t *hash) {
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "%s: ", label);
    for (int i = 0; i < 16; i++) {
        mHSS_PRINTF("%02x", hash[i]);
    }
    mHSS_PRINTF("\n");
}

static bool compare_md5(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 16; i++) {
        if (a[i] != b[i]) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 mismatch\n");
            return false;
        }
    }
    mHSS_DEBUG_PRINTF(LOG_STATUS, "MD5 passed\n");
    return true;
}

uint8_t get_boot_sequence(uint8_t index) {
    return Params.boot_seq_rtos[index];
}

bool get_verify_payload(void)
{
#if IS_ENABLED(CONFIG_SERVICE_verify_payload) 
    return true;
#else
    return (Params.verify_payload == 0xFF);
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

void HSS_slot_restore_boot_sequence(void)
{
    Params.boot_seq_rtos[0] = 0;
    Params.verify_payload = 0;
    Params.CRC = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;
    memcpy(buff, &Params, sizeof(Params));
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_erase_4k_block(PARAM_PADDR);
    spi_write(PARAM_PADDR, buff, sizeof(Params));
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Params restored\n");
#else 
    HSS_MMC_WriteBlock((size_t)(PARAM_REGION), buff, BLOCK_SIZE);
#endif
}

void HSS_slot_update_boot_params(int index, boot_error_codes code)
{
    Params.LastFailed = index;
    Params.CurrentTry = index+1;
    Params.current_sw = 0;
   // Params.boot_seq_rtos[0] = 10;
    Params.boot_seq_rtos[1] = EMMC_PRIMARY;
    Params.boot_seq_rtos[2] = EMMC_SECONDARY;
    Params.boot_seq_rtos[3] = SPI_FLASH;
    Params.boot_seq_rtos[4] = QSPI;
    //Params.verify_payload = 0xFF;
    Params.payload_ver_status[Params.LastFailed] = code;

    Params.CRC = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));
    Params.CRC = crc;
    memcpy(buff, &Params, sizeof(Params));

    /*mHSS_DEBUG_PRINTF(LOG_NORMAL,"update LastFailed: %d\n", Params.LastFailed);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update CurrentTry: %d\n", Params.CurrentTry);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update current_sw: %d\n", Params.current_sw);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"update payload_ver_status[%d]: %d\n", Params.LastFailed, code);*/

   // mHSS_DEBUG_PRINTF(LOG_NORMAL,"CRC: 0x%X\n", Params.CRC);
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_erase_4k_block(PARAM_PADDR);
    spi_write(PARAM_PADDR, buff, sizeof(Params));
    //mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot parameters update with crc %x\n", crc);
#else 
    HSS_MMC_WriteBlock((size_t)(PARAM_REGION), buff, BLOCK_SIZE);
#endif
}

static void copyBufferToParamData(const uint8_t* buffer, ParamData* params)
{
    params->LastFailed = buffer[0];
    params->CurrentTry = buffer[1];
    params->current_sw = buffer[2];
    params->boot_seq_rtos[0] = buffer[3];
    params->boot_seq_rtos[1] = EMMC_PRIMARY;
    params->boot_seq_rtos[2] = EMMC_SECONDARY;
    params->boot_seq_rtos[3] = SPI_FLASH;
    params->boot_seq_rtos[4] = QSPI;
    params->CRC = (buffer[12] << 24) |
                 (buffer[11] << 16) |
                 (buffer[10] << 8)  |
                  buffer[9];
    params->verify_payload = buffer[13];
    memcpy(params->payload_ver_status, &buffer[14], sizeof(params->payload_ver_status));
}

void HSS_slot_get_boot_params(void)
{
#if IS_ENABLED(CONFIG_SERVICE_SPI)
    spi_init();
    spi_read(&buff, PARAM_PADDR, sizeof(Params));
#else
    HSS_MMCInit();
    HSS_MMC_ReadBlock(&buff, (size_t)(PARAM_REGION), BLOCK_SIZE);
#endif
    copyBufferToParamData(buff, &Params);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Ignore CRC: %d\n",  Params.verify_payload);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Sequence[]: %d, %d, %d, %d, %d\n",
           Params.boot_seq_rtos[0],
           Params.boot_seq_rtos[1],
           Params.boot_seq_rtos[2],
           Params.boot_seq_rtos[3],
           Params.boot_seq_rtos[4]);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot Report[]:   %d, %d, %d, %d, %d\n",
            Params.payload_ver_status[0],
            Params.payload_ver_status[1],
            Params.payload_ver_status[2],
            Params.payload_ver_status[3],
            Params.payload_ver_status[4]);

    buff[12] = 0;
    buff[11] = 0;
    buff[10] = 0;
    buff[9] = 0;
    uint32_t crc = CRC32_calculate((const uint8_t *)&buff, sizeof(Params));

    if(crc != Params.CRC) {
        mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot params failed CRC:  0x%X\n", crc);
    }else{
        mHSS_DEBUG_PRINTF(LOG_NORMAL,"Boot params passed CRC:  0x%X\n", crc);
    }
}

bool validateMd5_custom(struct HSS_BootImage *pImage, size_t offset, memory_type_t mem_type)
{
    uint8_t temp_buffer[BLOCK_SIZE] = {0};
    uint32_t block_offset = 0;
    bool result = true;
    uint32_t start_addr = offset;
    MD5Context ctx;
    uint16_t digest_absolute_offset = 1488;
    uint16_t md5_index = digest_absolute_offset / BLOCK_SIZE;
    uint16_t md5_offset = digest_absolute_offset % BLOCK_SIZE;

    md5Init(&ctx);
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

            md5Update(&ctx, temp_buffer, chunk_size);
            block_offset++;
        } else {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "Error reading block at offset 0x%X\n", start_addr + block_offset);
            return false;
        }
    }

    md5Finalize(&ctx);
    print_md5("MD5 read", pImage->signature.digest);
    print_md5("MD5 calc", ctx.digest);
    result = compare_md5(ctx.digest, pImage->signature.digest);

    return result;
}


void spi_GetInfo(uint32_t *pBlockSize, uint32_t *pEraseSize, uint32_t *pBlockCount) {
    uint32_t sectorSize = 0x10000; //64KB
    *pEraseSize = *pBlockSize = sectorSize;
}

bool spi_init(void)
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