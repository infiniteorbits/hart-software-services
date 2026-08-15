/** ----------------------------------------------------------------------------
 * @file        BSP_CoreMMC.h
 * @brief       HSS-side shim for the secondary eMMC access layer.
 *
 *              The application tree has a real BSP_CoreMMC that owns the
 *              CoreMMC instance (src/application/bsp/BSP_CoreMMC.c). In the
 *              HSS that instance belongs to services/coremmc, so this header
 *              carries only the part of the interface the shared Bootloader 0
 *              modules use, and BSP_CoreMMC.c forwards it.
 *
 *              The point of the shim is that BSP_Boot_Loader0.c stays a
 *              straight copy of the application file: it calls
 *              BSP_coremmc_init() / BSP_coremmc_read_block() in both trees
 *              and never learns which one it is compiled into.
 *
 * @author      Koksal Kurt | koksal@rfim.co.uk
 * @date        August 2026
 * @version     1.3.0       /// Initial version (HSS port)
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_COREMMC_H_
#define BSP_COREMMC_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
#endif

/* --------------------------------------------------------------------------
 * @brief Status codes for the secondary eMMC access layer.
 *
 * Mirrors the application enumeration; only OK is distinguished from "not
 * OK" by the shared modules, but the values are kept identical so a status
 * logged on one side means the same thing on the other.
 * --------------------------------------------------------------------------
*/
typedef enum {
    BSP_COREMMC_OK = 0,             /**< Operation successful               */
    BSP_COREMMC_ERR_NOT_READY,      /**< Controller / device not initialised */
    BSP_COREMMC_ERR_INVALID_PARAM,  /**< NULL or misaligned buffer          */
    BSP_COREMMC_ERR_INIT,           /**< Bring-up did not succeed           */
    BSP_COREMMC_ERR_TRANSFER        /**< Block read / write failed          */
} bsp_coremmc_status_t;

/* --------------------------------------------------------------------------
 * @brief Brings the secondary eMMC up, once. Lazy: a call after a successful
 * bring-up returns BSP_COREMMC_OK without touching the hardware.
 *
 * @return BSP_COREMMC_OK on success, BSP_COREMMC_ERR_INIT otherwise.
 * --------------------------------------------------------------------------
*/
bsp_coremmc_status_t
BSP_coremmc_init(void);

/* --------------------------------------------------------------------------
 * @brief Reads one BSP_COREMMC_BLOCK_SIZE block from the secondary eMMC.
 *
 * @param sector LBA sector to read.
 * @param p_dst  Destination buffer, at least one block and 4-byte aligned.
 *
 * @return BSP_COREMMC_OK, BSP_COREMMC_ERR_INVALID_PARAM or
 * BSP_COREMMC_ERR_TRANSFER.
 * --------------------------------------------------------------------------
*/
bsp_coremmc_status_t
BSP_coremmc_read_block(uint32_t sector, void* p_dst);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COREMMC_H_ */
