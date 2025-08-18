/******************************************************************************
 * Copyright (c) 2024 Infinite Orbits. All rights reserved.
 *
 * @file    hss_slot_selection.c
 * @brief   Slot selection and image validation (eMMC/SPI/QSPI).
 *
 * This module centralizes slot selection, device muxing and integrity
 * verification of boot images (MD5) across multiple memory backends.
 *
 * Design goals:
 *  - Robust error handling with explicit status codes.
 *  - No magic numbers: semantic #defines for offsets/sizes/bits.
 *  - Static internals in snake_case; public API preserved (HSS_slot_*).
 *  - Defensive checks (null pointers, bounds, backend availability).
 *
 * @authors
 *  - A. Tarragó (abel.tarrago@ixrev.com)
 *
 * @version
 *  - 1.0: Initial version only emmc1
 *  - 1.1: Add spi-flash
 *  - 1.2: Add emmc2
 *  - 1.3: Hardening, error codes, snake_case internals, fixes & docs
 *
 * @date
 *  - 2024-09-10: Created
 *  - 2025-08-14: Hardened & refactor
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
#include <stdint.h>
#include <string.h>

#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"

/*----------------------------Constant Definitions--------------------------*/

/* Common sizes */
#define BLOCK_SIZE_BYTES            (512u)
#define MD5_DIGEST_SIZE             (16u)
#define MD5_SIGNATURE_ABS_OFFSET    (1488u)   /* Absolute offset inside image */
#define STREAM_GEN_BASE_ADDR        (0x4A000000u)
#define STREAM_GEN_REG_OFFSET       (0x10u)

/* SPI/QSPI parameters */
#define SPI_SLOT_SIZE_BYTES         (10u * 1024u * 1024u) /* 10 MiB */
#define SPI_ERASE_BLOCK_BYTES       (64u * 1024u)         /* 64 KiB */
#define SPI_MAX_XFER_BYTES          (32u * 1024u)         /* 32 KiB */

/*-------------------------------Type Definitions---------------------------*/

/** @brief Bitfield for stream generator register control. */
typedef enum {
    REGBIT_DDR_DIS   = (1u << 1),
    REGBIT_EMMPR_EN  = (1u << 8),
    REGBIT_EMMSC_EN  = (1u << 9),
    REGBIT_SW_EN     = (1u << 10),
    REGBIT_SW_DIS    = (1u << 11),
    REGBIT_SW_SEL0   = (1u << 12),
    REGBIT_SW_SEL1   = (1u << 13),
    REGBIT_P3V5_PG   = (1u << 14),
    REGBIT_P2V0_PG   = (1u << 15),
    REGBIT_P1V5_PG   = (1u << 16),
    REGBIT_DDR_PG    = (1u << 17),
} stream_gen_bits_t;

/** @brief Boot software parameter block (persisted in eMMC). */
typedef struct {
    uint8_t linux_last_failed;
    uint8_t linux_current_try;
    uint8_t linux_current_sw;
    uint8_t linux_boot_sequence;
    uint8_t linux_verify_payload;
    uint8_t freertos_last_failed;
    uint8_t freertos_current_try;
    uint8_t freertos_current_sw;
    uint8_t freertos_boot_sequence;
    uint8_t freertos_verify_payload;
} boot_software_params_t;

/** @brief Module status codes. */
typedef enum {
    HSSS_OK = 0,
    HSSS_ERR_INVALID_ARG,
    HSSS_ERR_BACKEND,
    HSSS_ERR_READ,
    HSSS_ERR_WRITE,
    HSSS_ERR_VERIFY,
    HSSS_ERR_UNSUPPORTED,
} hss_status_t;

/*------------------------------Local Variables-----------------------------*/
static boot_software_params_t g_params = {0};
static uint32_t *const g_stream_gen_reg =
    (uint32_t *)(STREAM_GEN_BASE_ADDR + STREAM_GEN_REG_OFFSET);

/*-----------------------------Local Prototypes-----------------------------*/
static inline void set_register_bit(volatile uint32_t *reg, stream_gen_bits_t bit);
static inline void clear_register_bit(volatile uint32_t *reg, stream_gen_bits_t bit);
static void print_md5(const char *label, const uint8_t *hash);
static bool compare_md5(const uint8_t *a, const uint8_t *b);
static hss_status_t md5_compute_over_backend(const struct HSS_BootImage *image,
                                              size_t start_offset,
                                              memory_type_t mem_type,
                                              uint8_t out_digest[MD5_DIGEST_SIZE]);
static hss_status_t spi_backend_init_once(void);
static hss_status_t spi_backend_read(void *dest, size_t src_offset, size_t len);
static hss_status_t spi_backend_write(size_t dst_offset, const void *src, size_t len);

/*-----------------------------Local Functions------------------------------*/

static inline void set_register_bit(volatile uint32_t *reg, stream_gen_bits_t bit)
{
    if (reg) { *reg |= (uint32_t)bit; }
}

static inline void clear_register_bit(volatile uint32_t *reg, stream_gen_bits_t bit)
{
    if (reg) { *reg &= (uint32_t)(~bit); }
}

static void print_md5(const char *label, const uint8_t *hash)
{
    if (!label || !hash) { return; }
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "%s: ", label);
    for (uint32_t i = 0; i < MD5_DIGEST_SIZE; i++) {
        mHSS_PRINTF("%02x", hash[i]);
    }
    mHSS_PRINTF("\n");
}

static bool compare_md5(const uint8_t *a, const uint8_t *b)
{
    if (!a || !b) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 compare: null pointer\n");
        return false;
    }
    for (uint32_t i = 0; i < MD5_DIGEST_SIZE; i++) {
        if (a[i] != b[i]) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 mismatch at byte %u\n", i);
            return false;
        }
    }
    return true;
}

/**
 * @brief Compute MD5 of boot image area across backends, zeroing the embedded digest.
 */
static hss_status_t md5_compute_over_backend(const struct HSS_BootImage *image,
                                              size_t start_offset,
                                              memory_type_t mem_type,
                                              uint8_t out_digest[MD5_DIGEST_SIZE])
{
    if (!image || !out_digest) {
        return HSSS_ERR_INVALID_ARG;
    }

    const uint32_t image_len = image->bootImageLength;
    if (image_len == 0u) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 compute: empty image length\n");
        return HSSS_ERR_INVALID_ARG;
    }

    MD5Context ctx;
    md5_init(&ctx);

    uint8_t block_buf[BLOCK_SIZE_BYTES];
    uint32_t block_index       = 0u;
    const uint32_t md5_index   = (MD5_SIGNATURE_ABS_OFFSET / BLOCK_SIZE_BYTES);
    const uint32_t md5_offset  = (MD5_SIGNATURE_ABS_OFFSET % BLOCK_SIZE_BYTES);

    for (uint32_t processed = 0; processed < image_len; processed += BLOCK_SIZE_BYTES) {
        size_t chunk = (image_len - processed) < BLOCK_SIZE_BYTES ?
                       (image_len - processed) : BLOCK_SIZE_BYTES;

        hss_status_t st = HSSS_OK;

        switch (mem_type) {
        case EMMC_PRIMARY:
        case EMMC_SECONDARY: {
            /*
             * When booting from either eMMC (primary or secondary),
             * the same eMMC driver function is used to read blocks.
             * Therefore, both cases are handled identically in code.
             */
            /* eMMC API reads one 512B block into 32-bit buffer */
            /* eMMC API reads one 512B block into 32-bit buffer */
            int8_t ok = MSS_MMC_single_block_read((uint32_t)(start_offset + block_index),
                                                  (uint32_t *)block_buf);
            st = ok ? HSSS_OK : HSSS_ERR_READ;
            break;
        }
        case SPI_FLASH: {
#if IS_ENABLED(CONFIG_SERVICE_SPI)
            FLASH_read(start_offset + ((size_t)block_index * BLOCK_SIZE_BYTES),
                       block_buf, BLOCK_SIZE_BYTES);
            st = HSSS_OK;
#else
            st = HSSS_ERR_UNSUPPORTED;
#endif
            break;
        }
        case QSPI: {
#if IS_ENABLED(CONFIG_SERVICE_QSPI)
            HSS_QSPI_ReadBlock(block_buf,
                               start_offset + ((size_t)block_index * BLOCK_SIZE_BYTES),
                               BLOCK_SIZE_BYTES);
            st = HSSS_OK;
#else
            st = HSSS_ERR_UNSUPPORTED;
#endif
            break;
        }
        default:
            st = HSSS_ERR_UNSUPPORTED;
            break;
        }

        if (st != HSSS_OK) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "Read failed (backend %d, block %u)\n",
                              (int)mem_type, block_index);
            return st;
        }

        if (block_index == md5_index) {
            /* Zero the in-image digest before hashing */
            if ((md5_offset + MD5_DIGEST_SIZE) <= BLOCK_SIZE_BYTES) {
                memset(&block_buf[md5_offset], 0, MD5_DIGEST_SIZE);
            } else {
                mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 offset spans blocks\n");
                return HSSS_ERR_INVALID_ARG;
            }
        }

        md5_update(&ctx, block_buf, chunk);
        block_index++;
    }

    md5_finalize(&ctx);
    memcpy(out_digest, ctx.digest, MD5_DIGEST_SIZE);
    return HSSS_OK;
}

/*------------------------------SPI helpers----------------------------*/

#if IS_ENABLED(CONFIG_SERVICE_SPI)
/* Maps flash_status_t to hsss_status_t depending on I/O context */
static hss_status_t map_flash_status(flash_status_t st, hss_status_t io_err)
{
    switch (st) {
    case FLASH_OK:               return HSSS_OK;
    case FLASH_ERR_INVALID_ARG:  return HSSS_ERR_INVALID_ARG;
    case FLASH_ERR_UNSUPPORTED:  return HSSS_ERR_UNSUPPORTED;
    case FLASH_ERR_TIMEOUT:      /* fallthrough */
    case FLASH_ERR_SPI:          /* fallthrough */
    default:                     return io_err;
    }
}
#endif

static hss_status_t spi_backend_init_once(void)
{
    static bool initialized = false;

    if (initialized) { return HSSS_OK; }

#if IS_ENABLED(CONFIG_SERVICE_SPI)
    uint8_t manufacturer_id = 0u, device_id = 0u;

    /* Init SPI core/driver */
    flash_status_t fst = FLASH_init();
    if (fst != FLASH_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI init failed (%d)\n", (int)fst);
        return map_flash_status(fst, HSSS_ERR_BACKEND);
    }

    /*
     * Intentional: leave SPI flash globally unprotected after init.
     * See rationale in code comments where this is called.
     */
    fst = FLASH_global_unprotect();
    if (fst != FLASH_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI global unprotect failed (%d)\n", (int)fst);
        return map_flash_status(fst, HSSS_ERR_BACKEND);
    }

    /* Read IDs (not strictly required to proceed, but useful to sanity-check) */
    fst = FLASH_read_device_id(&manufacturer_id, &device_id);
    if (fst != FLASH_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI read device ID failed (%d)\n", (int)fst);
        return map_flash_status(fst, HSSS_ERR_BACKEND);
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL,
                      "SPI Init OK: Device ID=%u, Manufacturer ID=%u\n",
                      device_id, manufacturer_id);

    initialized = true;
    return HSSS_OK;
#else
    return HSSS_ERR_UNSUPPORTED;
#endif
}

static hss_status_t spi_backend_read(void *dest, size_t src_offset, size_t len)
{
    if (!dest || len == 0u) { return HSSS_ERR_INVALID_ARG; }

#if IS_ENABLED(CONFIG_SERVICE_SPI)
    uint8_t *dst = (uint8_t *)dest;
    size_t done = 0u;

    while (done < len) {
        const size_t remaining = (len - done);
        const size_t chunk = (remaining < SPI_MAX_XFER_BYTES) ? remaining : SPI_MAX_XFER_BYTES;

        flash_status_t fst = FLASH_read((uint32_t)(src_offset + done), dst + done, chunk);
        if (fst != FLASH_OK) {
            mHSS_DEBUG_PRINTF(LOG_ERROR,
                              "SPI read failed at +0x%zx (chunk=%zu, st=%d)\n",
                              done, chunk, (int)fst);
            return map_flash_status(fst, HSSS_ERR_READ);
        }
        done += chunk;
    }
    return HSSS_OK;
#else
    (void)dest; (void)src_offset; (void)len;
    return HSSS_ERR_UNSUPPORTED;
#endif
}

static hss_status_t spi_backend_write(size_t dst_offset, const void *src, size_t len)
{
    if (!src || len == 0u) { return HSSS_ERR_INVALID_ARG; }

#if IS_ENABLED(CONFIG_SERVICE_SPI)
    /*
     * Assumes caller has handled any necessary erase/alignment policy.
     * FLASH_program_st() handles page boundaries internally.
     */
    const uint8_t *p = (const uint8_t *)src;
    flash_status_t fst = FLASH_program((uint32_t)dst_offset, p, len);
    if (fst != FLASH_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR,
                          "SPI program failed at 0x%zx (len=%zu, st=%d)\n",
                          dst_offset, len, (int)fst);
        return map_flash_status(fst, HSSS_ERR_WRITE);
    }
    return HSSS_OK;
#else
    (void)dst_offset; (void)src; (void)len;
    return HSSS_ERR_UNSUPPORTED;
#endif
}


/*-------------------------------Public API---------------------------------*/

memory_type_t HSS_slot_get_boot_sequence(uint8_t index)
{
    switch (index) {
    case 0:  return g_params.freertos_boot_sequence;
    case 1:  return EMMC_PRIMARY;
    case 2:  return EMMC_SECONDARY;
    case 3:  return SPI_FLASH;
    case 4:  return QSPI;
    default: return QSPI;
    }
}

bool HSS_slot_get_verify_payload(void)
{
#if IS_ENABLED(CONFIG_SERVICE_verify_payload)
    return true;
#else
    return (g_params.freertos_verify_payload == 0xFFu);
#endif
}

uint64_t HSS_slot_get_offset(uint8_t slot)
{
    switch (slot) {
    case 10: case 20: return PAYLOAD_1;
    case 11: case 21: return PAYLOAD_2;
    default:           return PAYLOAD_1;
    }
}

/**
 * @brief Route the stream generator to the selected eMMC device.
 */
bool HSS_slot_enable_emmc(memory_type_t emmc_id)
{
    bool result = true;
    switch (emmc_id) {
    case EMMC_PRIMARY:
        clear_register_bit(g_stream_gen_reg, REGBIT_SW_SEL0);
        clear_register_bit(g_stream_gen_reg, REGBIT_SW_SEL1);
        clear_register_bit(g_stream_gen_reg, REGBIT_EMMPR_EN);
        set_register_bit  (g_stream_gen_reg, REGBIT_EMMSC_EN);
        clear_register_bit(g_stream_gen_reg, REGBIT_SW_EN);
        break;

    case EMMC_SECONDARY:
        set_register_bit  (g_stream_gen_reg, REGBIT_SW_SEL0);
        set_register_bit  (g_stream_gen_reg, REGBIT_SW_SEL1);
        set_register_bit  (g_stream_gen_reg, REGBIT_EMMPR_EN);
        clear_register_bit(g_stream_gen_reg, REGBIT_EMMSC_EN);
        clear_register_bit(g_stream_gen_reg, REGBIT_SW_EN);
        break;

    default:
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Invalid eMMC ID: %u\n", emmc_id);
        result = false;
        break;
    }
    return result;
}

/**
 * @brief Reset/clear boot sequence and verification flags in parameter region 
 * to break the retry loop after a failure and prevent repeatedly booting the same slot.
 * @return true on success, false on write error.
 */
bool HSS_slot_restore_boot_sequence(void)
{
    g_params.freertos_boot_sequence  = 0u;
    g_params.freertos_verify_payload = 0u;
    const bool ok = HSS_MMC_WriteBlock((size_t)(PARAM_REGION),
                                       (uint8_t*)&g_params, BLOCK_SIZE_BYTES);
    if (!ok) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Failed to write PARAM_REGION on restore\n");
    }
    return ok;
}

/**
 * @brief Update boot params after an attempt.
 * @param index Boot attempt index.
 * @param code  Error code to persist (domain-specific).
 * @return true on success.
 */
bool HSS_slot_update_boot_params(int index)
{
    if (index < 0 || index > 0xFF) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "update_boot_params: invalid index %d\n", index);
        return false;
    }

    g_params.freertos_last_failed   = (uint8_t)index;
    g_params.freertos_current_try   = (uint8_t)(index + 1);
    g_params.freertos_current_sw   = 0u;

    const bool ok = HSS_MMC_WriteBlock((size_t)(PARAM_REGION),
                                       (uint8_t*)&g_params, BLOCK_SIZE_BYTES);
    if (!ok) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Failed to write PARAM_REGION on update\n");
    }
    return ok;
}

/**
 * @brief Load boot params from eMMC parameter region.
 * @return true on success, false on read error.
 */
bool HSS_slot_get_boot_params(void)
{
    HSS_MMCInit();

    const bool ok = HSS_MMC_ReadBlock((uint8_t*)&g_params,
                                      (size_t)(PARAM_REGION),
                                      BLOCK_SIZE_BYTES);
    if (!ok) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "Failed to read PARAM_REGION\n");
        return false;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL,
        "Boot Ignore CRC: %u\n",  g_params.freertos_verify_payload);
    mHSS_DEBUG_PRINTF(LOG_NORMAL,
        "Boot Sequence[]: %u, 10, 20, 30, 40\n", g_params.freertos_boot_sequence);

    return true;
}

/**
 * @brief Validate MD5 digest of an HSS boot image in a given backend.
 *
 * The function recomputes MD5 across the image region while zeroing the
 * embedded digest at MD5_SIGNATURE_ABS_OFFSET, then compares with
 * pImage->signature.digest.
 *
 * @param pImage   Boot image descriptor (length + embedded signature).
 * @param offset   Start offset (backend-specific units: eMMC LBA or byte addr).
 * @param mem_type Backend (EMMC_PRIMARY/EMMC_SECONDARY/SPI_FLASH/QSPI).
 * @return true if MD5 matches, false otherwise.
 */
bool HSS_slot_validate_md5(struct HSS_BootImage *pImage, size_t offset, memory_type_t mem_type)
{
    if (!pImage) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "validate_md5: null image pointer\n");
        return false;
    }

    if (mem_type == SPI_FLASH) {
        const hss_status_t ist = spi_backend_init_once();
        if (ist != HSSS_OK) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI init failed (%d)\n", (int)ist);
            return false;
        }
    }

    uint8_t calc[MD5_DIGEST_SIZE] = {0};
    const hss_status_t st = md5_compute_over_backend(pImage, offset, mem_type, calc);
    if (st != HSSS_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 compute failed (%d)\n", (int)st);
        return false;
    }

    print_md5("MD5 read", pImage->signature.digest);
    print_md5("MD5 calc", calc);
    const bool ok = compare_md5(calc, pImage->signature.digest);

    if (ok) {
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "MD5 passed\n");
    } else {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "MD5 mismatch\n");
    }

    return ok;
}

/*------------------------------ SPI API --------------------------------*/

bool HSS_slot_spi_get_info(uint32_t *p_block_size,
    uint32_t *p_erase_size,
    uint32_t *p_block_count)
{
    if (!p_block_size || !p_erase_size || !p_block_count) {
    return false;
    }

    *p_block_size  = SPI_ERASE_BLOCK_BYTES;
    *p_erase_size  = SPI_ERASE_BLOCK_BYTES;
    *p_block_count = (SPI_SLOT_SIZE_BYTES / SPI_ERASE_BLOCK_BYTES);

    return true;
}


bool HSS_slot_spi_init(void)
{
    const hss_status_t st = spi_backend_init_once();
    if (st != HSSS_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "HSS_slot_spi_init failed (%d)\n", (int)st);
        return false;
    }
    return true;
}

bool HSS_slot_spi_read(void *pDest, size_t srcOffset, size_t byteCount)
{
    const hss_status_t st = spi_backend_read(pDest, srcOffset, byteCount);
    if (st != HSSS_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI read failed (%d)\n", (int)st);
        return false;
    }
    return true;
}

bool HSS_slot_spi_write(size_t dstOffset, void *pSrc, size_t byteCount)
{
    const hss_status_t st = spi_backend_write(dstOffset, pSrc, byteCount);
    if (st != HSSS_OK) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "SPI write failed (%d)\n", (int)st);
        return false;
    }
    return true;
}
