/** ----------------------------------------------------------------------------
 * @file        BSP_Nand_Flash.h
 * @brief
 * @author      Koksal Kurt | koksal.kurt@outlook.com
 * @date        July 2026
 * @version     1.3.0
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_NAND_H_
#define BSP_NAND_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
#endif

/* --------------------------------------------------------------------------
 * @brief BSP return codes for NAND operations.
 * -------------------------------------------------------------------------- 
*/
typedef enum {
    BSP_NAND_OK = 0, /**< Operation completed successfully */
    BSP_NAND_ERR_INVALID_ID, /**< NAND0/NAND1 not available or invalid */
    BSP_NAND_ERR_OUT_OF_RANGE, /**< addr + length exceeds device geometry */
    BSP_NAND_ERR_READ_FAILED, /**< Low-level read error (ECC corrected or 
                                  uncorrectable) */
    BSP_NAND_ERR_WRITE_FAILED, /**< Page program failure */
    BSP_NAND_ERR_ERASE_FAILED, /**< Block erase failure */
    BSP_NAND_ERR_ALIGNMENT, /**< Write not aligned to page size */
    BSP_NAND_ERR_BAD_BLOCK, /**< Attempt to access a marked bad block */
    BSP_NAND_ERR_INTERNAL /**< Unexpected internal failure */
} bsp_nand_status_t;

/* --------------------------------------------------------------------------
 * @brief Initializes the NAND Flash controller and device.
 *
 * Performs controller configuration, device reset, ONFI parameter and
 * geometry discovery, and bad-block table construction. Must be called
 * once before any other NAND API call (BSP_nand_read / BSP_nand_write /
 * BSP_nand_erase).
 *
 * Calling this function again re-runs the full initialization sequence.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_INTERNAL if controller or device initialization fails.
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_init(void);

/* --------------------------------------------------------------------------
 * @brief Reads raw data from a NAND Flash device.
 *
 * @param dst Destination buffer where the data will be copied.
 * @param addr Absolute byte address inside the NAND device.
 * @param length Number of bytes to read.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_INVALID_ID / _OUT_OF_RANGE / _READ_FAILED otherwise.
 * -------------------------------------------------------------------------- 
*/
bsp_nand_status_t
BSP_nand_read(void* dst, uint32_t addr, uint32_t length);
/* --------------------------------------------------------------------------
 * @brief Writes data to a NAND Flash device.
 *
 * The BSP shall handle internally:
 * - Flash erase
 * - Chunking / page programming
 *
 * @param src Source buffer containing data to write.
 * @param addr Absolute byte address inside the NAND device.
 * @param length Number of bytes to write.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_WRITE_FAILED / _OUT_OF_RANGE / _ALIGNMENT, etc.
 * -------------------------------------------------------------------------- 
*/
bsp_nand_status_t
BSP_nand_write(const void* src, uint32_t addr, uint32_t length);
/* --------------------------------------------------------------------------
 * @brief Erase all NAND blocks touched by the given address range.
 *
 * Caller specifies a byte range, NOT block indices.
 *
 * The BSP internally:
 * - computes first_block = address / block_size
 * - computes last_block = (address + length - 1) / block_size
 * - erases all blocks in the interval
 * - skips bad blocks automatically
 *
 * @param[in] address Start address in bytes.
 * @param[in] length Number of bytes whose blocks must be erased.
 *
 * @return BSP_NAND_OK on success.
 * -------------------------------------------------------------------------- 
*/
bsp_nand_status_t
BSP_nand_erase(uint32_t address, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_NAND_H_ */
