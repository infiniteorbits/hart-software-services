/** ----------------------------------------------------------------------------
 * @file        BSP_Boot_Params.h
 * @brief       Persistent Boot Parameters for Bootloader 0. The storage
 *              backend is selected by BOOT_PARAMS_NAND (BSP_Config.h):
 *              NAND Flash when defined, golden SW QSPI NOR Flash otherwise.
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *              Koksal Kurt    | koksal@rfim.co.uk
 * @date        February 2026 - July 2026
 * @version     1.3.0       /// Implemented the slot-based parameter store
 *                          /// with the NAND/QSPI backend switch.
 *                          /// The staged FPGA update request lives in its
 *                          /// own store (BSP_Update_Request), not here.
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_BOOT_PARAMS_H_
#define BSP_BOOT_PARAMS_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint.h>
#else
    #include <stdint.h>
#endif

#include <stdbool.h>

/* -----------------------------------------------------------------------------
 * Maximum number of bootable entries in the boot sequence.
 * -----------------------------------------------------------------------------
*/
#define BOOT_SEQ_MAX_ENTRIES 4

/* -----------------------------------------------------------------------------
 * Result codes for Boot Parameters operations.
 * -----------------------------------------------------------------------------
*/
typedef enum {
    BOOT_OK = 0,                // Operation successful
    BOOT_ERR_INVALID_PARAM,     // Invalid pointer or illegal field
    BOOT_ERR_STORAGE_FAIL,      // SPI Flash read/write error
    BOOT_ERR_BOOT_SOURCE,       // Invalid or unsupported boot source
    BOOT_ERR_MD5_VERIFY,        // MD5 verification failed
} boot_error_status_t;
/* -----------------------------------------------------------------------------
 * Boot source identifiers used to configure the boot_sequence[]
 * -----------------------------------------------------------------------------
*/
typedef enum {
    BOOT_SRC_PRIMARY = 10,      // MSS eMMC primary
    BOOT_SRC_SECONDARY = 20,    // Fabric eMMC secondary
    BOOT_SRC_GOLDEN = 255       // QSPI Flash Golden
} boot_source_t;
/* -----------------------------------------------------------------------------
 * The Boot Parameters modules are shared with the HSS, which builds its
 * own copy of these files (boards/mpfs-orbsight-v2/bsp/ in the HSS
 * tree): any change here must be applied to both copies.
 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
 * Persistent Boot Parameters stored in SPI Flash.
 *
 * - boot_sequence[]: Ordered list of boot entries. Typical configuration
 *                      includes:
 * - Configurable (Primary/Secondary/Golden)
 * - MSS Payload (Primary)
 * - Fabric Payload (Secondary)
 * - QSPI Flash Payload (Golden).
 * - last_failed: Boot source value (boot_source_t) of the last entry that
 * failed to boot; 0 = none recorded yet.
 * - last_successful: Boot source value of the last entry that provided a
 * bootable (verified) image; 0 = none recorded yet.
 * - current_try: Boot source value being attempted by Bootloader 0,
 * written before each attempt: after an unexpected reset,
 * current_try != last_successful identifies the interrupted attempt.
 * - integrity_check_en: Enables (1) or disables (0) MD5 integrity verification
 * for Linux and Fabric images before execution.
 * -----------------------------------------------------------------------------
*/
typedef struct __attribute__((packed)) {
    boot_source_t boot_sequence[BOOT_SEQ_MAX_ENTRIES];
    boot_source_t last_failed;
    boot_source_t last_successful;
    boot_source_t current_try;
    bool integrity_check_en;
} boot_params_t;

/* -----------------------------------------------------------------------------
* @brief Reads the persistent Boot Parameters from Flash (NAND when
* BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
*
* Returns the most recent valid stored record. If the store holds no valid
* record (virgin Flash, or every record torn/corrupted), documented
* defaults are returned: boot sequence Primary, Secondary, Golden, Golden;
* index fields 0; integrity check enabled.
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
BOOT_params_read(boot_params_t* const params);

/* -----------------------------------------------------------------------------
* @brief Writes updated Boot Parameters to Flash (NAND when
* BOOT_PARAMS_NAND is defined, QSPI NOR otherwise).
*
* The record is appended to the next free store slot (wear-levelled); the
* region is erased only when all slots are consumed. The programmed record
* is read back and verified.
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
BOOT_params_write(const boot_params_t* params);


#ifdef __cplusplus
    }
#endif

#endif /* BSP_BOOT_PARAMS_H_ */
