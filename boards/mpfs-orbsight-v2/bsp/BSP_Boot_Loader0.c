/**-----------------------------------------------------------------------------
 * @file        BSP_Boot_Loader0.c
 * @brief       Bootloader 0 support services:
 *              - Software MD5 (RFC 1321) digest computation.
 *              - Payload integrity verification for the boot sources
 *                (Primary: MSS eMMC, Golden: golden SW QSPI NOR Flash).
 *              - Sector-rotating Bootloader log. The log storage backend is
 *                selected by BOOT_PARAMS_NAND (BSP_Config.h): NAND Flash
 *                via the BSP_Nand_Flash API when defined, otherwise the
 *                golden SW QSPI NOR Flash (MSS QSPI controller
 *                @ 0x21000000) through the infineon_S25FL driver
 *                (BSP_Flash device FLASH_DEVICE_GOLDEN_SW).
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *              Koksal Kurt    | koksal.kurt@outlook.com
 * @date        February 2026 - July 2026
 *
 * @version     1.3.0       /// Implemented MD5 compute/verify and the
 *                          /// QSPI-NOR-backed Bootloader log
 *                          /// BOOT_PARAMS_NAND switch selecting the log
 *                          /// storage backend (NAND vs QSPI NOR)
 *                          /// BOOT_verify_md5 hashes the HSS header
 *                          /// signature/md5Sum window as zeros per the
 *                          /// payload generator MD5 contract
 *                          /// Fixed the QSPI log backend to target the
 *                          /// golden SW Flash device instead of the
 *                          /// bitstream Flash, matching the documented
 *                          /// design
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 *
 * @note MISRA C:2012 deviations:
 *  - Rule 11.3: Cast from uint8_t* to uint32_t* is required by the MSS MMC
 *    driver API which operates on 32-bit-aligned buffers.
 *  - Rule 15.5: Multiple return statements retained for early-exit error
 *    handling, consistent with BSP coding style.
 * -----------------------------------------------------------------------------
*/

#include "BSP_Boot_Loader0.h"
#include "BSP_Config.h"

#include <drivers/mss/mss_mmc/mss_mmc.h>

#include "BSP_Flash.h"

#ifdef BOOT_PARAMS_NAND
    #include "BSP_Nand_Flash.h"
#endif

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
    #include <stddef.h>
#endif

/* -----------------------------------------------------------------------------
 * Private configuration
 * -----------------------------------------------------------------------------
*/

#ifdef BOOT_PARAMS_NAND

/** @brief NAND device holding the Bootloader log ring. */
#define BOOT_LOG_NAND_DEVICE            (NAND1)

/** @brief Log ring erase-unit size on NAND. Must equal (or be a multiple
 *  of) the NAND erase-block size so that erasing one ring sector never
 *  touches the other; adjust when the NAND driver geometry is final
 *  (placeholder: 128 KB blocks). */
#define BOOT_LOG_SECTOR_SIZE            (0x00020000u)

/** @brief Number of erase units forming the Bootloader log ring. */
#define BOOT_LOG_NUM_SECTORS            (2u)

/** @brief Base address of the Bootloader log region: at the top of the
 *  32-bit-addressable NAND window (the 256 Gbit / 32 GB device exceeds
 *  32-bit byte addressing, so the first 4 GB window is used), directly
 *  below the Boot Parameters block at 0xFFFE0000
 *  (see BSP_Boot_Params.c). */
#define BOOT_LOG_BASE_ADDR              (0xFFFA0000u)

#else /* QSPI NOR backend */

/** @brief Flash sector size used for erase operations (64 KB). */
#define BOOT_LOG_SECTOR_SIZE            (0x00010000u)

/** @brief Number of 64 KB Flash sectors forming the Bootloader log ring. */
#define BOOT_LOG_NUM_SECTORS            (2u)

/** @brief Base address of the Bootloader log region in the golden SW Flash
 *  (MSS QSPI). The region sits at the top of the 32 MB device, below the
 *  topmost sector at 0x01FF0000 which is reserved for the Boot Parameters
 *  (see BSP_Boot_Params.c). Golden SW payloads must not extend into this
 *  region. */
#define BOOT_LOG_BASE_ADDR              (0x01FD0000u)

#endif /* BOOT_PARAMS_NAND */

/** @brief Total size of the Bootloader log region. */
#define BOOT_LOG_SIZE                   (BOOT_LOG_NUM_SECTORS *              \
                                            BOOT_LOG_SECTOR_SIZE)

/** @brief Size of the per-sector sequence-number header. Each log sector
 *  starts with a 32-bit little-endian sequence number; the sector holding
 *  the highest sequence number is the active (append) sector. An erased
 *  sector reads 0xFFFFFFFF, which is never a valid sequence number. */
#define BOOT_LOG_SEQ_HDR_SIZE           (4u)

/** @brief Sequence-number value of an erased (empty) sector. */
#define BOOT_LOG_SEQ_EMPTY              (0xFFFFFFFFu)

/** @brief Maximum stored length of a single log message (bytes, excluding
 *  the '\n' record terminator). Longer messages are truncated. */
#define BOOT_LOG_MAX_MSG_LEN            (256u)

/** @brief Record terminator appended after every log message. */
#define BOOT_LOG_MSG_TERMINATOR         ((uint8_t)'\n')

/** @brief Value of an erased NOR Flash byte; marks unused log space. */
#define BOOT_FLASH_ERASED_BYTE          (0xFFu)

/** @brief Chunk size for streamed Flash/eMMC reads. Matches the eMMC block
 *  size so one chunk holds exactly one eMMC sector. */
#define BOOT_CHUNK_SIZE                 (512u)

/** @brief Base address of the golden SW payload in the golden SW Flash. */
#define BOOT_SPI_BASE_ADDR        (0x00000000u)

/** @brief Maximum golden SW payload size accepted for verification:
 *  everything below the reserved top 192 KB of the golden SW Flash
 *  (matches BSP_GOLDEN_SW_SLOT_SIZE in BSP_Golden_Update.c). The
 *  reservation holds regardless of the BOOT_PARAMS_NAND backend switch. */
#define BOOT_SPI_MAX_PAYLOAD      (0x01FD0000u)

/** @brief MD5 processing block size in bytes (RFC 1321). */
#define BOOT_MD5_BLOCK_SIZE             (64u)

/** @brief MD5 digest size in bytes. */
#define BOOT_MD5_DIGEST_SIZE            (16u)

/** @brief Buffer fill level at which the 8-byte length field is appended
 *  during MD5 finalization (RFC 1321 padding). */
#define BOOT_MD5_PAD_TARGET             (56u)

/** @brief Total size of struct HSS_BootImage at the start of a payload. */
#define BOOT_HSS_HEADER_SIZE            (1632u)

/** @brief Offset of the 64-bit little-endian bootImageLength field. */
#define BOOT_HSS_IMAGE_LEN_OFFSET       (1480u)

/** @brief Offset of the signature/md5Sum union region; also the offset of
 *  the embedded 16-byte MD5 digest (md5Sum aliases the signature start). */
#define BOOT_HSS_SIG_OFFSET             (1488u)

/** @brief Size of the signature/md5Sum union region. The MD5 payload
 *  contract hashes these bytes as zeros. */
#define BOOT_HSS_SIG_SIZE               (144u)

/** @brief Size of the embedded MD5 digest. */
#define BOOT_HSS_MD5_SIZE               (16u)
/* -----------------------------------------------------------------------------
 * Private data
 * -----------------------------------------------------------------------------
*/

/** @brief Shared staging buffer for streamed Flash/eMMC reads. Makes the
 *  module non-reentrant, consistent with the other BSP storage modules. */
static uint8_t g_chunk_buf[BOOT_CHUNK_SIZE] __attribute__((aligned(4)));

/** @brief MD5 streaming context (RFC 1321). */
typedef struct {
    uint32_t state[4];
    uint64_t total_len;
    uint8_t  buffer[BOOT_MD5_BLOCK_SIZE];
    uint32_t buffer_len;
    uint32_t reserved;      /* Explicit tail padding (8-byte alignment)    */
} boot_md5_ctx_t;

/** @brief MD5 per-round additive constants (RFC 1321, K[i] =
 *  floor(abs(sin(i + 1)) * 2^32)). */
static const uint32_t MD5_K[64] = {
    0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu,
    0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
    0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu,
    0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
    0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau,
    0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
    0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu,
    0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
    0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu,
    0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
    0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u,
    0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
    0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u,
    0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
    0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
    0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u
};

/** @brief MD5 per-round left-rotation amounts (RFC 1321). */
static const uint32_t MD5_S[64] = {
    7u, 12u, 17u, 22u, 7u, 12u, 17u, 22u,
    7u, 12u, 17u, 22u, 7u, 12u, 17u, 22u,
    5u,  9u, 14u, 20u, 5u,  9u, 14u, 20u,
    5u,  9u, 14u, 20u, 5u,  9u, 14u, 20u,
    4u, 11u, 16u, 23u, 4u, 11u, 16u, 23u,
    4u, 11u, 16u, 23u, 4u, 11u, 16u, 23u,
    6u, 10u, 15u, 21u, 6u, 10u, 15u, 21u,
    6u, 10u, 15u, 21u, 6u, 10u, 15u, 21u
};

/* -----------------------------------------------------------------------------
 * Private helpers - MD5 (RFC 1321)
 * -----------------------------------------------------------------------------
*/

/**
 * @brief Rotates a 32-bit value left by the given number of bits.
 *
 * @param value  The value to rotate.
 * @param bits   Rotation amount, 1..31.
 *
 * @return The rotated value.
 */
static uint32_t
md5_rotl(uint32_t value, uint32_t bits)
{
    return (value << bits) | (value >> (32u - bits));
}

/**
 * @brief Runs the MD5 compression function over one 64-byte block.
 *
 * @param state  The four 32-bit chaining variables, updated in place.
 * @param block  The 64-byte input block.
 */
static void
md5_transform(uint32_t state[4], const uint8_t block[BOOT_MD5_BLOCK_SIZE])
{
    uint32_t m[16];
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
    uint32_t f;
    uint32_t g;
    uint32_t tmp;
    uint32_t i;

    /* Decode the block into 16 little-endian 32-bit words                  */
    for (i = 0u; i < 16u; i++)
    {
        m[i] = ((uint32_t)block[(i * 4u)])                                   |
               ((uint32_t)block[(i * 4u) + 1u] << 8u)                        |
               ((uint32_t)block[(i * 4u) + 2u] << 16u)                       |
               ((uint32_t)block[(i * 4u) + 3u] << 24u);
    }

    a = state[0];
    b = state[1];
    c = state[2];
    d = state[3];

    for (i = 0u; i < 64u; i++)
    {
        if (i < 16u)
        {
            f = (b & c) | ((~b) & d);
            g = i;
        }
        else if (i < 32u)
        {
            f = (d & b) | ((~d) & c);
            g = ((5u * i) + 1u) & 0x0Fu;
        }
        else if (i < 48u)
        {
            f = b ^ c ^ d;
            g = ((3u * i) + 5u) & 0x0Fu;
        }
        else
        {
            f = c ^ (b | (~d));
            g = (7u * i) & 0x0Fu;
        }

        tmp = d;
        d   = c;
        c   = b;
        b   = b + md5_rotl(a + f + MD5_K[i] + m[g], MD5_S[i]);
        a   = tmp;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
}

/**
 * @brief Initializes an MD5 streaming context.
 *
 * @param[out] ctx  The context to initialize.
 */
static void
md5_init(boot_md5_ctx_t* ctx)
{
    ctx->state[0]   = 0x67452301u;
    ctx->state[1]   = 0xefcdab89u;
    ctx->state[2]   = 0x98badcfeu;
    ctx->state[3]   = 0x10325476u;
    ctx->total_len  = 0u;
    ctx->buffer_len = 0u;
}

/**
 * @brief Feeds data into an MD5 streaming context.
 *
 * @param ctx   The context to update.
 * @param data  Input bytes.
 * @param len   Number of input bytes.
 */
static void
md5_update(boot_md5_ctx_t* ctx, const uint8_t* data, uint32_t len)
{
    uint32_t idx;

    ctx->total_len += (uint64_t)len;

    for (idx = 0u; idx < len; idx++)
    {
        ctx->buffer[ctx->buffer_len] = data[idx];
        ctx->buffer_len++;

        if (ctx->buffer_len == BOOT_MD5_BLOCK_SIZE)
        {
            md5_transform(ctx->state, ctx->buffer);
            ctx->buffer_len = 0u;
        }
    }
}

/**
 * @brief Applies RFC 1321 padding and produces the final digest.
 *
 * @param ctx          The context to finalize.
 * @param[out] digest  Output buffer of 16 bytes.
 */
static void
md5_final(boot_md5_ctx_t* ctx, uint8_t digest[BOOT_MD5_DIGEST_SIZE])
{
    const uint64_t bit_len   = ctx->total_len * 8u;
    const uint8_t  pad_first = 0x80u;
    const uint8_t  pad_zero  = 0x00u;
    uint8_t        len_bytes[8];
    uint32_t       i;

    md5_update(ctx, &pad_first, 1u);

    while (ctx->buffer_len != BOOT_MD5_PAD_TARGET)
    {
        md5_update(ctx, &pad_zero, 1u);
    }

    /* Append the pre-padding message length in bits, little endian         */
    for (i = 0u; i < 8u; i++)
    {
        len_bytes[i] = (uint8_t)((bit_len >> (8u * i)) & 0xFFu);
    }

    md5_update(ctx, len_bytes, 8u);

    for (i = 0u; i < 4u; i++)
    {
        digest[(i * 4u)]      = (uint8_t)(ctx->state[i] & 0xFFu);
        digest[(i * 4u) + 1u] = (uint8_t)((ctx->state[i] >> 8u) & 0xFFu);
        digest[(i * 4u) + 2u] = (uint8_t)((ctx->state[i] >> 16u) & 0xFFu);
        digest[(i * 4u) + 3u] = (uint8_t)((ctx->state[i] >> 24u) & 0xFFu);
    }
}

/**
 * @brief Compares two MD5 digests.
 *
 * @param a  First digest.
 * @param b  Second digest.
 *
 * @return 1u if equal, 0u otherwise.
 */
static uint8_t
md5_digest_equal(const uint8_t a[BOOT_MD5_DIGEST_SIZE],                      \
        const uint8_t b[BOOT_MD5_DIGEST_SIZE])
{
    uint8_t  equal = 1u;
    uint32_t i;

    for (i = 0u; i < BOOT_MD5_DIGEST_SIZE; i++)
    {
        if (a[i] != b[i])
        {
            equal = 0u;
        }
    }

    return equal;
}

/**
 * @brief Feeds one payload chunk into an MD5 context, hashing any bytes
 *        that overlap the HSS header signature/md5Sum window as zeros.
 *
 * The payload generator computes the embedded digest over the image with
 * the [BOOT_HSS_SIG_OFFSET, BOOT_HSS_SIG_OFFSET + BOOT_HSS_SIG_SIZE)
 * region zeroed and then stores the digest inside that region, so the
 * stored bytes there must not enter the hash (see BSP_Boot_Loader0.h).
 *
 * @param ctx          The context to update.
 * @param buf          Chunk bytes read from storage; bytes overlapping
 *                     the window are zeroed in place.
 * @param len          Number of valid bytes in buf.
 * @param payload_off  Payload offset of buf[0].
 */
static void
md5_update_payload(boot_md5_ctx_t* ctx, uint8_t* buf, uint32_t len,          \
        uint32_t payload_off)
{
    uint32_t zero_start = BOOT_HSS_SIG_OFFSET;
    uint32_t zero_end   = BOOT_HSS_SIG_OFFSET + BOOT_HSS_SIG_SIZE;
    uint32_t i;

    if (zero_start < payload_off)
    {
        zero_start = payload_off;
    }

    if (zero_end > (payload_off + len))
    {
        zero_end = payload_off + len;
    }

    for (i = zero_start; i < zero_end; i++)
    {
        buf[i - payload_off] = 0u;
    }

    md5_update(ctx, buf, len);
}

/* -----------------------------------------------------------------------------
 * Private helpers - Bootloader log storage backend
 *
 * The log is kept on NAND Flash when BOOT_PARAMS_NAND is defined
 * (BSP_Config.h), on the golden SW QSPI NOR Flash otherwise. All wrappers
 * take device-relative byte addresses and return 0u on success, 1u on
 * failure.
 * -----------------------------------------------------------------------------
*/

#ifdef BOOT_PARAMS_NAND

/**
 * @brief Reads from the log storage backend (NAND).
 */
static uint8_t
log_storage_read(uint8_t* dst, uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_read(dst, addr, len)    \
            == BSP_NAND_OK) ? 0u : 1u);
}

/**
 * @brief Programs the log storage backend (NAND).
 *
 * @note BSP_nand_write() is documented to handle erase internally; when
 * called from this module it must program into already-erased space
 * WITHOUT erasing the containing block, or appended log records would
 * destroy earlier ones in the same block.
 */
static uint8_t
log_storage_program(const uint8_t* src, uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_write(src, addr, len)   \
            == BSP_NAND_OK) ? 0u : 1u);
}

/**
 * @brief Erases a range of the log storage backend (NAND).
 */
static uint8_t
log_storage_erase(uint32_t addr, uint32_t len)
{
    return (uint8_t)((BSP_nand_erase(addr, len)        \
            == BSP_NAND_OK) ? 0u : 1u);
}

#else /* QSPI NOR backend */

/**
 * @brief Reads from the log storage backend (golden SW QSPI NOR).
 */
static uint8_t
log_storage_read(uint8_t* dst, uint32_t addr, uint32_t len)
{
    /* Flash_read cannot report failure                                     */
    Flash_read(FLASH_DEVICE_GOLDEN_SW, dst, addr, len);

    return 0u;
}

/**
 * @brief Programs the log storage backend (golden SW QSPI NOR).
 */
static uint8_t
log_storage_program(const uint8_t* src, uint32_t addr, uint32_t len)
{
    return Flash_program(FLASH_DEVICE_GOLDEN_SW, src, addr, len);
}

/**
 * @brief Erases a range of the log storage backend (golden SW QSPI NOR).
 */
static uint8_t
log_storage_erase(uint32_t addr, uint32_t len)
{
    return Flash_64KByte_erase(FLASH_DEVICE_GOLDEN_SW, addr, len);
}

#endif /* BOOT_PARAMS_NAND */

/* -----------------------------------------------------------------------------
 * Private helpers - Bootloader log
 * -----------------------------------------------------------------------------
*/

/**
 * @brief Returns the absolute Flash address of a log ring sector.
 *
 * @param sector_idx  Ring sector index, 0..BOOT_LOG_NUM_SECTORS-1.
 *
 * @return Absolute Flash address of the sector.
 */
static uint32_t
log_sector_addr(uint32_t sector_idx)
{
    return BOOT_LOG_BASE_ADDR + (sector_idx * BOOT_LOG_SECTOR_SIZE);
}

/**
 * @brief Reads the 32-bit little-endian sequence number of a log sector.
 *
 * @param sector_idx  Ring sector index.
 * @param[out] seq  The sequence number; BOOT_LOG_SEQ_EMPTY if the sector
 * is erased.
 *
 * @return BOOT_OK on success, BOOT_ERR_STORAGE_FAIL if the read fails.
 */
static boot_error_status_t
log_read_seq(uint32_t sector_idx, uint32_t* seq)
{
    uint8_t seq_buf[BOOT_LOG_SEQ_HDR_SIZE] __attribute__((aligned(4)));

    if (log_storage_read(seq_buf, log_sector_addr(sector_idx),              \
            BOOT_LOG_SEQ_HDR_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    *seq = ((uint32_t)seq_buf[0])                                            |
           ((uint32_t)seq_buf[1] << 8u)                                      |
           ((uint32_t)seq_buf[2] << 16u)                                     |
           ((uint32_t)seq_buf[3] << 24u);

    return BOOT_OK;
}

/**
 * @brief Locates the sector currently open for appending.
 *
 * The active sector is the one carrying the highest valid sequence number.
 *
 * @param[out] found       1u if an active sector exists, 0u if the whole
 * ring is erased.
 * @param[out] active_idx  Ring index of the active sector.
 * @param[out] active_seq  Sequence number of the active sector.
 *
 * @return BOOT_OK on success, BOOT_ERR_STORAGE_FAIL if a read fails.
 */
static boot_error_status_t
log_find_active(uint8_t* found, uint32_t* active_idx, uint32_t* active_seq)
{
    uint32_t seq = 0u;
    uint32_t i;

    *found = 0u;

    for (i = 0u; i < BOOT_LOG_NUM_SECTORS; i++)
    {
        if (log_read_seq(i, &seq) != BOOT_OK)
        {
            return BOOT_ERR_STORAGE_FAIL;
        }

        if (seq != BOOT_LOG_SEQ_EMPTY)
        {
            if ((*found == 0u) || (seq > *active_seq))
            {
                *active_idx = i;
                *active_seq = seq;
                *found      = 1u;
            }
        }
    }

    return BOOT_OK;
}

/**
 * @brief Finds the append offset inside a log sector.
 *
 * Scans the message area for the first erased (0xFF) byte. Stored messages
 * are sanitized so they never contain 0xFF, making the first erased byte
 * the write head.
 *
 * @param sector_addr  Absolute Flash address of the sector.
 * @param[out] append_off  Offset of the first free byte relative to the
 * sector start, or BOOT_LOG_SECTOR_SIZE if the sector is full.
 *
 * @return BOOT_OK on success, BOOT_ERR_STORAGE_FAIL if a read fails.
 */
static boot_error_status_t
log_append_offset(uint32_t sector_addr, uint32_t* append_off)
{
    uint32_t offset = BOOT_LOG_SEQ_HDR_SIZE;
    uint32_t chunk;
    uint32_t i;

    while (offset < BOOT_LOG_SECTOR_SIZE)
    {
        chunk = BOOT_LOG_SECTOR_SIZE - offset;

        if (chunk > BOOT_CHUNK_SIZE)
        {
            chunk = BOOT_CHUNK_SIZE;
        }

        if (log_storage_read(g_chunk_buf, sector_addr + offset,             \
                chunk) != 0u)
        {
            return BOOT_ERR_STORAGE_FAIL;
        }

        for (i = 0u; i < chunk; i++)
        {
            if (g_chunk_buf[i] == BOOT_FLASH_ERASED_BYTE)
            {
                *append_off = offset + i;

                return BOOT_OK;
            }
        }

        offset += chunk;
    }

    *append_off = BOOT_LOG_SECTOR_SIZE;

    return BOOT_OK;
}

/**
 * @brief Erases a log sector and stamps its sequence-number header.
 *
 * @param sector_idx  Ring index of the sector to open.
 * @param seq         Sequence number to assign.
 *
 * @return BOOT_OK on success, BOOT_ERR_STORAGE_FAIL on Flash error.
 */
static boot_error_status_t
log_open_sector(uint32_t sector_idx, uint32_t seq)
{
    uint8_t  seq_buf[BOOT_LOG_SEQ_HDR_SIZE] __attribute__((aligned(4)));
    uint32_t sector_addr = log_sector_addr(sector_idx);

    if (log_storage_erase(sector_addr, BOOT_LOG_SECTOR_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    seq_buf[0] = (uint8_t)(seq & 0xFFu);
    seq_buf[1] = (uint8_t)((seq >> 8u) & 0xFFu);
    seq_buf[2] = (uint8_t)((seq >> 16u) & 0xFFu);
    seq_buf[3] = (uint8_t)((seq >> 24u) & 0xFFu);

    if (log_storage_program(seq_buf, sector_addr,                            \
            BOOT_LOG_SEQ_HDR_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
 * Public functions
 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
 * @brief Compute the MD5 checksum of arbitrary data.
 *
 * @param[in] data Pointer to input buffer.
 * @param[in] len Length in bytes.
 * @param[out] md5 Output buffer of 16 bytes.
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_INVALID_PARAM for NULL pointers,
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_compute_md5(const void* data, uint32_t len, uint8_t md5[16])
{
    boot_md5_ctx_t ctx;

    if ((data == NULL) || (md5 == NULL))
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    md5_init(&ctx);
    md5_update(&ctx, (const uint8_t*)data, len);
    md5_final(&ctx, md5);

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
 * @brief Verify the MD5 checksum of a stored payload.
 *
 * The payload is streamed from the storage backend of the given boot source
 * and hashed in BOOT_CHUNK_SIZE chunks:
 * - BOOT_SRC_PRIMARY: MSS eMMC, starting at sector MSS_EMMC_SECTOR_NUMBER.
 *   The eMMC controller must already be initialized (see mmc_init()).
 * - BOOT_SRC_GOLDEN: golden SW QSPI NOR Flash, starting at address 0x0.
 * - BOOT_SRC_SECONDARY: not supported (no Fabric eMMC controller wired up).
 *
 * Per the HSS payload MD5 contract, bytes overlapping the header's
 * signature/md5Sum window [BOOT_HSS_SIG_OFFSET, BOOT_HSS_SIG_OFFSET +
 * BOOT_HSS_SIG_SIZE) are hashed as zeros (the embedded digest must not
 * enter its own hash). With 512-byte chunks the window spans chunks 2
 * (bytes 464..511) and 3 (bytes 0..95); md5_update_payload() handles the
 * substitution generically.
 *
 * @param[in] boot_src (Primary/Secondary/Golden).
 * @param[in] size Payload size in bytes.
 * @param[in] expected_md5 Pointer to expected 16-byte MD5 digest.
 *
 * @return boot_error_status_t
 * BOOT_OK if digest matches,
 * BOOT_ERR_MD5_VERIFY if mismatch,
 * BOOT_ERR_STORAGE_FAIL if read fails,
 * BOOT_ERR_BOOT_SOURCE for an invalid or unsupported boot source,
 * BOOT_ERR_INVALID_PARAM for NULL pointer, zero or out-of-range size.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_verify_md5(boot_source_t boot_src, uint32_t size,                      \
        const uint8_t expected_md5[16])
{
    boot_md5_ctx_t   ctx;
    uint8_t          digest[BOOT_MD5_DIGEST_SIZE];
    mss_mmc_status_t mmc_status;
    uint32_t         remaining;
    uint32_t         chunk;
    uint32_t         flash_addr;
    uint32_t         sector;
    uint32_t         payload_off;

    if ((expected_md5 == NULL) || (size == 0u))
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    md5_init(&ctx);
    remaining   = size;
    payload_off = 0u;

    switch (boot_src)
    {
        case BOOT_SRC_GOLDEN:
            /* The payload must not extend into the reserved log and boot
             * parameters region at the top of the device.                  */
            if (size > BOOT_SPI_MAX_PAYLOAD)
            {
                return BOOT_ERR_INVALID_PARAM;
            }

            flash_addr = BOOT_SPI_BASE_ADDR;

            while (remaining != 0u)
            {
                chunk = (remaining > BOOT_CHUNK_SIZE) ?                      \
                        BOOT_CHUNK_SIZE : remaining;

                Flash_read(FLASH_DEVICE_GOLDEN_SW, g_chunk_buf,              \
                        flash_addr, chunk);
                md5_update_payload(&ctx, g_chunk_buf, chunk, payload_off);

                flash_addr  += chunk;
                payload_off += chunk;
                remaining   -= chunk;
            }
            break;

        case BOOT_SRC_PRIMARY:
            sector = MSS_EMMC_SECTOR_NUMBER;

            while (remaining != 0u)
            {
                /* MISRA Rule 11.3 deviation: MSS MMC API requires
                 * uint32_t*.                                               */
                mmc_status = MSS_MMC_single_block_read(sector,               \
                        (uint32_t*)g_chunk_buf);

                if (mmc_status != MSS_MMC_TRANSFER_SUCCESS)
                {
                    return BOOT_ERR_STORAGE_FAIL;
                }

                chunk = (remaining > MSS_EMMC_BLOCK_SIZE) ?                  \
                        MSS_EMMC_BLOCK_SIZE : remaining;

                md5_update_payload(&ctx, g_chunk_buf, chunk, payload_off);

                sector++;
                payload_off += chunk;
                remaining   -= chunk;
            }
            break;

        case BOOT_SRC_SECONDARY:
        default:
            return BOOT_ERR_BOOT_SOURCE;
    }

    md5_final(&ctx, digest);

    if (md5_digest_equal(digest, expected_md5) == 0u)
    {
        return BOOT_ERR_MD5_VERIFY;
    }

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
 * @brief Writes a Bootloader log message into the Flash circular buffer
 * (NAND when BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
 *
 * The message is appended to the active log sector followed by a '\n'
 * terminator. When the active sector is full the ring rotates: the next
 * sector is erased, stamped with the next sequence number and becomes the
 * append target (the oldest messages are lost). Messages longer than
 * BOOT_LOG_MAX_MSG_LEN bytes are truncated; 0xFF bytes are replaced by '?'
 * so stored data never aliases erased Flash.
 *
 * @param[in] msg
 * Null-terminated string containing the log message to store.
 *
 * @return boot_error_status_t
 * BOOT_OK on success (an empty message is a no-op),
 * BOOT_ERR_STORAGE_FAIL if a Flash read/program/erase fails,
 * BOOT_ERR_INVALID_PARAM if msg is NULL.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_write(const char *msg)
{
    static uint8_t g_msg_buf[BOOT_LOG_MAX_MSG_LEN + 1u]                      \
            __attribute__((aligned(4)));

    boot_error_status_t status;
    uint32_t            msg_len = 0u;
    uint32_t            record_len;
    uint32_t            sector_addr;
    uint32_t            append_off;
    uint32_t            active_idx = 0u;
    uint32_t            active_seq = 0u;
    uint32_t            next_idx;
    uint32_t            next_seq;
    uint8_t             found;

    if (msg == NULL)
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    /* Stage the message, bounding its length and sanitizing 0xFF bytes     */
    while ((msg_len < BOOT_LOG_MAX_MSG_LEN) && (msg[msg_len] != '\0'))
    {
        g_msg_buf[msg_len] =                                                 \
                ((uint8_t)msg[msg_len] == BOOT_FLASH_ERASED_BYTE) ?          \
                        (uint8_t)'?' : (uint8_t)msg[msg_len];
        msg_len++;
    }

    if (msg_len == 0u)
    {
        return BOOT_OK;
    }

    g_msg_buf[msg_len] = BOOT_LOG_MSG_TERMINATOR;
    record_len         = msg_len + 1u;

    status = log_find_active(&found, &active_idx, &active_seq);

    if (status != BOOT_OK)
    {
        return status;
    }

    if (found == 0u)
    {
        /* Empty ring: open the first sector with sequence number 1        */
        status = log_open_sector(0u, 1u);

        if (status != BOOT_OK)
        {
            return status;
        }

        sector_addr = log_sector_addr(0u);
        append_off  = BOOT_LOG_SEQ_HDR_SIZE;
    }
    else
    {
        sector_addr = log_sector_addr(active_idx);
        status      = log_append_offset(sector_addr, &append_off);

        if (status != BOOT_OK)
        {
            return status;
        }

        if ((append_off + record_len) > BOOT_LOG_SECTOR_SIZE)
        {
            /* Active sector full: rotate to the next ring sector          */
            next_idx = (active_idx + 1u) % BOOT_LOG_NUM_SECTORS;
            next_seq = active_seq + 1u;

            /* Never stamp the reserved erased-sector marker: a torn
             * header program could otherwise leave the ring stuck.        */
            if (next_seq == BOOT_LOG_SEQ_EMPTY)
            {
                next_seq = 1u;
            }

            status = log_open_sector(next_idx, next_seq);

            if (status != BOOT_OK)
            {
                return status;
            }

            sector_addr = log_sector_addr(next_idx);
            append_off  = BOOT_LOG_SEQ_HDR_SIZE;
        }
    }

    if (log_storage_program(g_msg_buf, sector_addr + append_off,             \
            record_len) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
 * @brief Reads the entire Bootloader log buffer from Flash (NAND when
 * BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
 *
 * Log sectors are returned oldest first (ascending sequence number), each
 * message terminated by '\n'. If dst is too small the log is truncated to
 * max_len - 1 bytes. dst is always null terminated.
 *
 * @param[out] dst
 * Destination buffer where the log contents will be copied.
 *
 * @param[in] max_len
 * Maximum number of bytes available in dst. Must be > 0.
 *
 * @param[out] out_len
 * Number of bytes actually returned in dst (excluding the null
 * terminator).
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_INVALID_PARAM if pointers are NULL or arguments invalid,
 * BOOT_ERR_STORAGE_FAIL if a Flash read fails.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_read(char *dst, uint32_t max_len, uint32_t* out_len)
{
    uint32_t seqs[BOOT_LOG_NUM_SECTORS];
    uint8_t  done[BOOT_LOG_NUM_SECTORS];
    uint32_t capacity;
    uint32_t copied = 0u;
    uint32_t sector_addr;
    uint32_t used_end;
    uint32_t offset;
    uint32_t chunk;
    uint32_t pick;
    uint8_t  pick_found;
    uint32_t i;

    if ((dst == NULL) || (out_len == NULL) || (max_len == 0u))
    {
        return BOOT_ERR_INVALID_PARAM;
    }

    capacity = max_len - 1u;
    *out_len = 0u;

    for (i = 0u; i < BOOT_LOG_NUM_SECTORS; i++)
    {
        if (log_read_seq(i, &seqs[i]) != BOOT_OK)
        {
            dst[0] = '\0';

            return BOOT_ERR_STORAGE_FAIL;
        }

        done[i] = (seqs[i] == BOOT_LOG_SEQ_EMPTY) ? 1u : 0u;
    }

    /* Copy sectors oldest first (ascending sequence number)                */
    for (;;)
    {
        pick       = 0u;
        pick_found = 0u;

        for (i = 0u; i < BOOT_LOG_NUM_SECTORS; i++)
        {
            if (done[i] == 0u)
            {
                if ((pick_found == 0u) || (seqs[i] < seqs[pick]))
                {
                    pick       = i;
                    pick_found = 1u;
                }
            }
        }

        if (pick_found == 0u)
        {
            break;
        }

        done[pick]  = 1u;
        sector_addr = log_sector_addr(pick);

        if (log_append_offset(sector_addr, &used_end) != BOOT_OK)
        {
            dst[copied] = '\0';

            return BOOT_ERR_STORAGE_FAIL;
        }

        offset = BOOT_LOG_SEQ_HDR_SIZE;

        while ((offset < used_end) && (copied < capacity))
        {
            chunk = used_end - offset;

            if (chunk > BOOT_CHUNK_SIZE)
            {
                chunk = BOOT_CHUNK_SIZE;
            }

            if (chunk > (capacity - copied))
            {
                chunk = capacity - copied;
            }

            if (log_storage_read(g_chunk_buf, sector_addr + offset,          \
                    chunk) != 0u)
            {
                dst[copied] = '\0';

                return BOOT_ERR_STORAGE_FAIL;
            }

            for (i = 0u; i < chunk; i++)
            {
                dst[copied + i] = (char)g_chunk_buf[i];
            }

            copied += chunk;
            offset += chunk;
        }
    }

    dst[copied] = '\0';
    *out_len    = copied;

    return BOOT_OK;
}

/* -----------------------------------------------------------------------------
 * @brief Erase the entire Bootloader log buffer.
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_STORAGE_FAIL if erase fails.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_clear(void)
{
    if (log_storage_erase(BOOT_LOG_BASE_ADDR, BOOT_LOG_SIZE) != 0u)
    {
        return BOOT_ERR_STORAGE_FAIL;
    }

    return BOOT_OK;
}

#ifdef __cplusplus
}
#endif
