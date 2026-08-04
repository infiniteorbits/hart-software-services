/**-----------------------------------------------------------------------------
 * @file        BSP_Boot_Params.c
 * @brief       Persistent Boot Parameters store for Bootloader 0.
 *
 *              Parameters are kept in a slot-based, append-only record store
 *              so that frequent updates (current_try, last_failed,
 *              last_successful on every boot attempt) do not wear the Flash:
 *              each BOOT_params_write() programs a CRC32-protected record
 *              into the next free slot, BOOT_params_read() returns the most
 *              recent valid record, and the region is erased only once all
 *              slots have been consumed. A record torn by power loss fails
 *              its CRC and the previous record remains in effect.
 *
 *              The storage backend is selected by BOOT_PARAMS_NAND
 *              (BSP_Config.h): NAND Flash via the BSP_Nand_Flash API when
 *              defined, otherwise the reserved top sector of the golden SW
 *              QSPI NOR Flash (MSS QSPI @ 0x21000000, sector 0x01FF0000).
 *
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *              Koksal Kurt    | koksal@rfim.co.uk
 * @date        February 2026 - July 2026
 *
 * @version     1.3.0       /// Implemented the slot-based parameter store
 *                          /// with the NAND/QSPI backend switch.
 *                          /// Fixed the QSPI backend to target the golden
 *                          /// SW Flash device instead of the bitstream
 *                          /// Flash, matching the documented design.
 *                          /// integrity_check_en is serialized in the
 *                          /// stored record as a 32-bit magic word
 *                          /// ("enabled" only on exact match), so a
 *                          /// corrupted record can never read back as
 *                          /// enabled.
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 *
 * @note MISRA C:2012 deviations:
 *  - Rule 15.5: Multiple return statements retained for early-exit error
 *    handling, consistent with BSP coding style.
 * -----------------------------------------------------------------------------
*/

#include "BSP_Boot_Params.h"
#include "BSP_Config.h"

#ifdef BOOT_PARAMS_NAND
    #include "BSP_Nand_Flash.h"
#else
    #include "BSP_Flash.h"
#endif

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
    #include <stddef.h>
    #include <string.h>
#endif

/* -----------------------------------------------------------------------------
 * Private configuration
 * -----------------------------------------------------------------------------
*/

#ifdef BOOT_PARAMS_NAND

/** @brief Base address of the Boot Parameters region: the topmost block
 *  reachable through the 32-bit NAND byte-address API. The NAND device is
 *  256 Gbit (32 GB), so only its first 4 GB window is addressable; this
 *  region ends exactly at the 4 GB boundary (last byte 0xFFFFFFFF), which
 *  requires the NAND driver's range checks to use overflow-safe math
 *  (addr + (len - 1u), never addr + len). The Bootloader log ring sits
 *  directly below (see BSP_Boot_Loader0.c). */
#define BOOT_PARAMS_BASE_ADDR           (0xFFFE0000u)

/** @brief Size of the Boot Parameters region: one NAND erase block;
 *  adjust when the NAND driver geometry is final (placeholder: 128 KB). */
#define BOOT_PARAMS_REGION_SIZE         (0x00020000u)

/** @brief Record slot size. One slot per NAND page (placeholder: 2 KB) so
 *  every record program is a single page-aligned operation and NAND
 *  partial-page programming (NOP) limits are never exceeded. */
#define BOOT_PARAMS_SLOT_SIZE           (2048u)

#else /* QSPI NOR backend */

/** @brief Base address of the Boot Parameters region: the topmost 64 KB
 *  sector of the golden SW Flash, above the Bootloader log ring at
 *  0x01FD0000 (see BSP_Boot_Loader0.c). Golden SW payloads must not
 *  extend into this region. */
#define BOOT_PARAMS_BASE_ADDR           (0x01FF0000u)

/** @brief Size of the Boot Parameters region: one 64 KB erase sector. */
#define BOOT_PARAMS_REGION_SIZE         (0x00010000u)

/** @brief Record slot size: 64 bytes gives 1024 parameter updates per
 *  sector erase. */
#define BOOT_PARAMS_SLOT_SIZE           (64u)

#endif /* BOOT_PARAMS_NAND */

/** @brief Record marker programmed at the start of every valid slot
 *  ("BOOT" in ASCII, stored little endian). Never all-0xFF, so a marker
 *  never aliases erased Flash. */
#define BOOT_PARAMS_MAGIC               (0x424F4F54u)

/** @brief Stored record layout: [magic:4][payload][crc32:4]. The CRC32
 *  covers the payload bytes only. The payload serializes boot_params_t
 *  as: the fields up to and including current_try copied as-is, followed
 *  by a 32-bit little-endian integrity word - integrity_check_en is
 *  stored as BOOT_PARAMS_INTEGRITY_MAGIC when true and reads back as
 *  enabled ONLY on an exact magic match, so a corrupted record can never
 *  read back as enabled. The magic is a storage detail private to this
 *  module; the API keeps the plain bool field. */
#define BOOT_PARAMS_MAGIC_SIZE          (4u)
#define BOOT_PARAMS_CRC_SIZE            (4u)
#define BOOT_PARAMS_TRACKED_SIZE        ((uint32_t)offsetof(boot_params_t,   \
                                            integrity_check_en))
#define BOOT_PARAMS_INTEGRITY_SIZE      (4u)
#define BOOT_PARAMS_PAYLOAD_SIZE        (BOOT_PARAMS_TRACKED_SIZE +          \
                                            BOOT_PARAMS_INTEGRITY_SIZE)
#define BOOT_PARAMS_RECORD_SIZE         (BOOT_PARAMS_MAGIC_SIZE +            \
                                            BOOT_PARAMS_PAYLOAD_SIZE +       \
                                            BOOT_PARAMS_CRC_SIZE)

/** @brief Stored integrity word for "check enabled" ("INTG" in ASCII).
 *  Any other stored value reads back as disabled. */
#define BOOT_PARAMS_INTEGRITY_MAGIC     (0x494E5447u)

/** @brief Stored integrity word for "check disabled". */
#define BOOT_PARAMS_INTEGRITY_OFF       (0x00000000u)

/** @brief Value of an erased Flash byte. */
#define BOOT_PARAMS_ERASED_BYTE         (0xFFu)

/** @brief CRC32 polynomial (reflected, ISO 3309 / ITU-T V.42), matching
 *  the CRC32 used elsewhere in the application. */
#define BOOT_PARAMS_CRC32_POLY          (0xEDB88320u)

_Static_assert(BOOT_PARAMS_RECORD_SIZE <= BOOT_PARAMS_SLOT_SIZE,
        "boot_params_t record must fit in one store slot");

/* -----------------------------------------------------------------------------
 * Private data
 * -----------------------------------------------------------------------------
*/

/** @brief Defaults returned by BOOT_params_read() when the store holds no
 *  valid record yet (virgin or fully corrupted Flash). The three index
 *  fields are 0 ("no attempt recorded yet"); integrity checking defaults
 *  to enabled. */
static const boot_params_t g_boot_params_defaults = {
    .boot_sequence      = { BOOT_SRC_PRIMARY, BOOT_SRC_SECONDARY,
                            BOOT_SRC_GOLDEN,  BOOT_SRC_GOLDEN },
    .last_failed        = (boot_source_t)0,
    .last_successful    = (boot_source_t)0,
    .current_try        = (boot_source_t)0,
    .integrity_check_en = true
};

/** @brief Staging buffers for record scan/program/verify. Make the module
 *  non-reentrant, consistent with the other BSP storage modules. */
static uint8_t g_record_buf[BOOT_PARAMS_RECORD_SIZE]                         \
        __attribute__((aligned(4)));
static uint8_t g_readback_buf[BOOT_PARAMS_RECORD_SIZE]                       \
        __attribute__((aligned(4)));

/* -----------------------------------------------------------------------------
 * Private helpers - storage backend
 *
 * The Boot Parameters are kept on NAND Flash when BOOT_PARAMS_NAND is
 * defined (BSP_Config.h), on the golden SW QSPI NOR Flash otherwise. All
 * wrappers take device-relative byte addresses and return 0u on success,
 * 1u on failure.
 * -----------------------------------------------------------------------------
*/

#ifdef BOOT_PARAMS_NAND

/**
 * @brief Reads from the Boot Parameters storage backend (NAND).
 */
static uint8_t
params_storage_read(uint8_t* dst, uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_read(dst, addr,     \
            len) == BSP_NAND_OK) ? 0u : 1u);
}

/**
 * @brief Programs the Boot Parameters storage backend (NAND).
 *
 * @note BSP_nand_write() is documented to handle erase internally; when
 * called from this module it must program into already-erased space
 * WITHOUT erasing the containing block, or appended records would
 * destroy earlier ones in the same block.
 */
static uint8_t
params_storage_program(const uint8_t* src, uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_write(src, addr,    \
            len) == BSP_NAND_OK) ? 0u : 1u);
}

/**
 * @brief Erases the Boot Parameters storage backend region (NAND).
 */
static uint8_t
params_storage_erase(uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_erase(addr,         \
            len) == BSP_NAND_OK) ? 0u : 1u);
}

#else /* QSPI NOR backend */

/**
 * @brief Reads from the Boot Parameters storage backend (golden SW QSPI
 * NOR).
 */
static uint8_t
params_storage_read(uint8_t* dst, uint32_t addr, uint32_t len)
{
    /* Flash_read cannot report failure                                     */
    Flash_read(FLASH_DEVICE_GOLDEN_SW, dst, addr, len);

    return 0u;
}

/**
 * @brief Programs the Boot Parameters storage backend (golden SW QSPI
 * NOR).
 */
static uint8_t
params_storage_program(const uint8_t* src, uint32_t addr, uint32_t len)
{
    return Flash_program(FLASH_DEVICE_GOLDEN_SW, src, addr, len);
}

/**
 * @brief Erases the Boot Parameters storage backend region (golden SW
 * QSPI NOR).
 */
static uint8_t
params_storage_erase(uint32_t addr, uint32_t len)
{
    return Flash_64KByte_erase(FLASH_DEVICE_GOLDEN_SW, addr, len);
}

#endif /* BOOT_PARAMS_NAND */

/* -----------------------------------------------------------------------------
 * Private helpers - record store
 * -----------------------------------------------------------------------------
*/

/**
 * @brief Computes the CRC32 (reflected, poly 0xEDB88320, init and final
 * XOR 0xFFFFFFFF) of a data buffer.
 *
 * @param data    Input bytes.
 * @param length  Number of input bytes.
 *
 * @return The CRC32 value.
 */
static uint32_t
params_crc32(const uint8_t* data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < length; i++)
    {
        crc ^= (uint32_t)data[i];

        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1u) ^ BOOT_PARAMS_CRC32_POLY;
            }
            else
            {
                crc >>= 1u;
            }
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

/**
 * @brief Stores a 32-bit value little endian into a byte buffer.
 */
static void
params_store_u32(uint8_t* buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFu);
    buf[1] = (uint8_t)((value >> 8u) & 0xFFu);
    buf[2] = (uint8_t)((value >> 16u) & 0xFFu);
    buf[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

/**
 * @brief Loads a 32-bit little-endian value from a byte buffer.
 */
static uint32_t
params_load_u32(const uint8_t* buf)
{
    return ((uint32_t)buf[0])                                                |
           ((uint32_t)buf[1] << 8u)                                          |
           ((uint32_t)buf[2] << 16u)                                         |
           ((uint32_t)buf[3] << 24u);
}

/**
 * @brief Checks whether a record buffer reads as erased Flash.
 *
 * @param record  Buffer of BOOT_PARAMS_RECORD_SIZE bytes.
 *
 * @return 1u if every byte is 0xFF, 0u otherwise.
 */
static uint8_t
params_record_erased(const uint8_t* record)
{
    uint32_t i;

    for (i = 0u; i < BOOT_PARAMS_RECORD_SIZE; i++)
    {
        if (record[i] != BOOT_PARAMS_ERASED_BYTE)
        {
            return 0u;
        }
    }

    return 1u;
}

/**
 * @brief Checks whether a record buffer holds a valid record.
 *
 * @param record  Buffer of BOOT_PARAMS_RECORD_SIZE bytes.
 *
 * @return 1u if the magic marker and payload CRC32 are valid, 0u
 * otherwise (erased, torn or corrupted record).
 */
static uint8_t
params_record_valid(const uint8_t* record)
{
    uint32_t stored_crc;

    if (params_load_u32(record) != BOOT_PARAMS_MAGIC)
    {
        return 0u;
    }

    stored_crc = params_load_u32(                                            \
            &record[BOOT_PARAMS_MAGIC_SIZE + BOOT_PARAMS_PAYLOAD_SIZE]);

    if (params_crc32(&record[BOOT_PARAMS_MAGIC_SIZE],                        \
            BOOT_PARAMS_PAYLOAD_SIZE) != stored_crc)
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief Scans the record store for the most recent valid record and the
 * first free slot.
 *
 * Slots are consumed strictly in order, so the scan stops at the first
 * fully-erased slot. Slots holding torn or corrupted records are skipped
 * (counted as used).
 *
 * @param[out] latest      Populated with the payload of the most recent
 * valid record when *found_valid is 1u.
 * @param[out] found_valid 1u if at least one valid record exists.
 * @param[out] free_off    Region-relative offset of the first free slot
 * when *found_free is 1u.
 * @param[out] found_free  1u if a free slot exists.
 *
 * @return BOOT_OK on success, BOOT_ERR_STORAGE_FAIL if a read fails.
 */
static boot_error_status_t
params_scan(boot_params_t* latest, uint8_t* found_valid,                     \
        uint32_t* free_off, uint8_t* found_free)
{
    uint32_t slot_off;

    *found_valid = 0u;
    *found_free  = 0u;

    for (slot_off = 0u; slot_off < BOOT_PARAMS_REGION_SIZE;                  \
            slot_off += BOOT_PARAMS_SLOT_SIZE)
    {
        if (params_storage_read(g_record_buf,                                \
                BOOT_PARAMS_BASE_ADDR + slot_off,                            \
                BOOT_PARAMS_RECORD_SIZE) != 0u)
        {
            return BOOT_ERR_STORAGE_FAIL;
        }

        if (params_record_erased(g_record_buf) == 1u)
        {
            *free_off   = slot_off;
            *found_free = 1u;
            break;
        }

        if (params_record_valid(g_record_buf) == 1u)
        {
            (void)memcpy(latest, &g_record_buf[BOOT_PARAMS_MAGIC_SIZE],      \
                    BOOT_PARAMS_TRACKED_SIZE);

            /* Enabled only on an exact stored-magic match                  */
            latest->integrity_check_en = (params_load_u32(                   \
                    &g_record_buf[BOOT_PARAMS_MAGIC_SIZE +                   \
                        BOOT_PARAMS_TRACKED_SIZE]) ==                        \
                    BOOT_PARAMS_INTEGRITY_MAGIC) ? true : false;

            *found_valid = 1u;
        }
    }

    return BOOT_OK;
}

/**
 * @brief Validates that a boot source value is one of the defined
 * enumerators.
 *
 * @param src  The value to validate.
 *
 * @return 1u if valid, 0u otherwise.
 */
static uint8_t
params_boot_source_valid(boot_source_t src)
{
    if ((src == BOOT_SRC_PRIMARY) || (src == BOOT_SRC_SECONDARY) ||          \
            (src == BOOT_SRC_GOLDEN))
    {
        return 1u;
    }

    return 0u;
}

/* -----------------------------------------------------------------------------
 * Public functions
 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
* @brief Reads the persistent Boot Parameters from Flash (NAND when
* BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
*
* Returns the most recent valid record in the store. If the store holds no
* valid record (virgin Flash, or every record torn/corrupted), the
* documented defaults are returned instead: boot sequence Primary,
* Secondary, Golden, Golden; index fields 0; integrity check enabled.
*
* @param[inout] params
* Pointer to a boot_params_t structure that will be populated with the
* values currently stored in Flash.
*
* @return boot_error_status_t
* BOOT_OK on success,
* BOOT_ERR_INVALID_PARAM if params is NULL,
* BOOT_ERR_STORAGE_FAIL if the Flash read operation fails.
* ------------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_params_read(boot_params_t* const params)
{
    boot_error_status_t status;
    uint8_t             found_valid;
    uint8_t             found_free;
    uint32_t            free_off;

    if (params == NULL)
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    status = params_scan(params, &found_valid, &free_off, &found_free);

    if (status != BOOT_OK)
    {
        return status;
    }

    if (found_valid == 0u)
    {
        *params = g_boot_params_defaults;
    }

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
* @brief Writes updated Boot Parameters to Flash (NAND when
* BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
*
* The record is appended to the next free store slot; the region is erased
* only when all slots have been consumed, so frequent updates do not wear
* the Flash. The programmed record is read back and verified.
*
* @param[in] params
* Pointer to a boot_params_t structure containing the updated values.
*
* @return boot_error_status_t
* BOOT_OK on success,
* BOOT_ERR_INVALID_PARAM if params is NULL,
* BOOT_ERR_BOOT_SOURCE if a boot_sequence entry is not a valid boot source,
* BOOT_ERR_STORAGE_FAIL if Flash erase/programming/verify fails.
* ------------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_params_write(const boot_params_t* params)
{
    static boot_params_t scan_params;   /* scan scratch, content unused     */

    boot_error_status_t status;
    uint8_t             found_valid;
    uint8_t             found_free;
    uint32_t            free_off;
    uint32_t            crc;
    uint32_t            i;

    if (params == NULL)
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    for (i = 0u; i < (uint32_t)BOOT_SEQ_MAX_ENTRIES; i++)
    {
        if (params_boot_source_valid(params->boot_sequence[i]) == 0u)
        {
            return BOOT_ERR_BOOT_SOURCE;
        }
    }

    status = params_scan(&scan_params, &found_valid, &free_off, &found_free);

    if (status != BOOT_OK)
    {
        return status;
    }

    if (found_free == 0u)
    {
        /* Store full: recycle the region and start over at slot 0         */
        if (params_storage_erase(BOOT_PARAMS_BASE_ADDR,                      \
                BOOT_PARAMS_REGION_SIZE) != 0u)
        {
            return BOOT_ERR_STORAGE_FAIL;
        }

        free_off = 0u;
    }

    /* Serialize the record: [magic][payload][crc32(payload)]; the
     * integrity flag is stored as its 32-bit magic word.                   */
    params_store_u32(&g_record_buf[0], BOOT_PARAMS_MAGIC);
    (void)memcpy(&g_record_buf[BOOT_PARAMS_MAGIC_SIZE], params,              \
            BOOT_PARAMS_TRACKED_SIZE);
    params_store_u32(                                                        \
            &g_record_buf[BOOT_PARAMS_MAGIC_SIZE + BOOT_PARAMS_TRACKED_SIZE],\
            (params->integrity_check_en) ?                                   \
                    BOOT_PARAMS_INTEGRITY_MAGIC : BOOT_PARAMS_INTEGRITY_OFF);

    crc = params_crc32(&g_record_buf[BOOT_PARAMS_MAGIC_SIZE],                \
            BOOT_PARAMS_PAYLOAD_SIZE);
    params_store_u32(                                                        \
            &g_record_buf[BOOT_PARAMS_MAGIC_SIZE + BOOT_PARAMS_PAYLOAD_SIZE],\
            crc);

    if (params_storage_program(g_record_buf,                                 \
            BOOT_PARAMS_BASE_ADDR + free_off,                                \
            BOOT_PARAMS_RECORD_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    /* Read back and verify the programmed record                           */
    if (params_storage_read(g_readback_buf,                                  \
            BOOT_PARAMS_BASE_ADDR + free_off,                                \
            BOOT_PARAMS_RECORD_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    if (memcmp(g_record_buf, g_readback_buf, BOOT_PARAMS_RECORD_SIZE) != 0)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    return BOOT_OK;
}

#ifdef __cplusplus
}
#endif
