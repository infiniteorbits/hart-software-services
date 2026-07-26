/** ----------------------------------------------------------------------------
 * @file        BSP_Boot_Loader0.h
 * @brief       Bootloader 0 support services: software MD5 (RFC 1321)
 *              computation, boot payload integrity verification and the
 *              Bootloader log. The log storage backend is selected by
 *              BOOT_PARAMS_NAND (BSP_Config.h): NAND Flash when defined,
 *              golden SW QSPI NOR Flash otherwise.
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *              Koksal Kurt    | koksal@rfim.co.uk
 * @date        February 2026 - July 2026
 * @version     1.3.0       /// Implemented MD5 compute/verify and the
 *                          /// QSPI-NOR-backed Bootloader log
 *                          /// BOOT_PARAMS_NAND switch selecting the log
 *                          /// storage backend (NAND vs QSPI NOR)
 *                          /// BOOT_verify_md5 hashes the HSS header
 *                          /// signature/md5Sum window as zeros per the
 *                          /// payload generator MD5 contract; HSS header
 *                          /// layout constants exported
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_BOOT_LOADER0_H_
#define BSP_BOOT_LOADER0_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
#endif

#include "BSP_Boot_Params.h"


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
BOOT_compute_md5(const void* data, uint32_t len, uint8_t md5[16]);
/* -----------------------------------------------------------------------------
 * @brief Verify the MD5 checksum of a stored payload.
 *
 * The payload is streamed from the storage backend of the given boot source
 * (Primary: MSS eMMC, Golden: golden SW QSPI NOR Flash) and its MD5 digest
 * is compared against the expected one. BOOT_SRC_SECONDARY is not supported
 * as no Fabric eMMC controller is wired up.
 *
 * Per the HSS payload MD5 contract, bytes overlapping the header's
 * signature/md5Sum window [BOOT_HSS_SIG_OFFSET, BOOT_HSS_SIG_OFFSET +
 * BOOT_HSS_SIG_SIZE) are hashed as zeros, so the digest embedded at that
 * offset never enters its own hash.
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
        const uint8_t expected_md5[16]);
/* -----------------------------------------------------------------------------
 * @brief Writes a Bootloader log message into the Flash circular buffer
 * (NAND when BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
 *
 * Messages are appended to a ring of Flash sectors; when the active sector
 * is full the ring rotates and the oldest messages are lost. Messages are
 * truncated to 256 bytes.
 *
 * @param[in] msg
 * Null-terminated string containing the log message to store.
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_STORAGE_FAIL if a Flash read/program/erase fails,
 * BOOT_ERR_INVALID_PARAM if msg is NULL.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_write(const char *msg);

/* -----------------------------------------------------------------------------
 * @brief Reads the entire Bootloader log buffer from Flash (NAND when
 * BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
 *
 * Messages are returned oldest first, each terminated by '\n'. If dst is
 * too small the log is truncated; dst is always null terminated.
 *
 * @param[out] dst
 * Destination buffer where the log contents will be copied.
 *
 * @param[in] max_len
 * Maximum number of bytes available in dst. Must be > 0.
 *
 * @param[out] out_len
 * Number of bytes actually returned in dst (excluding the null
 terminator).
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_INVALID_PARAM if pointers are NULL or arguments invalid,
 * BOOT_ERR_STORAGE_FAIL if a Flash read fails.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_read(char *dst, uint32_t max_len, uint32_t* out_len);

/* -----------------------------------------------------------------------------
 * @brief Erase the entire Bootloader log buffer.
 *
 * @return boot_error_status_t
 * BOOT_OK on success,
 * BOOT_ERR_STORAGE_FAIL if erase fails.
 * -----------------------------------------------------------------------------
*/
boot_error_status_t
BOOT_log_clear(void);


#ifdef __cplusplus
    }
#endif

#endif /* BSP_BOOT_LOADER0_H_ */
