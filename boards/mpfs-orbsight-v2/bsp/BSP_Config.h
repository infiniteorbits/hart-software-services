/** ----------------------------------------------------------------------------
 * @file        BSP_Config.h
 * @brief       HSS-side subset of the application BSP_Config.h: only the
 *              macros consumed by the shared Bootloader 0 modules
 *              (BSP_Boot_Params, BSP_Boot_Loader0, BSP_Nand_Flash) are
 *              carried here. The full configuration lives in the
 *              application tree (src/application/bsp/BSP_Config.h); the
 *              macros below must be kept in sync with it.
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *              Koksal Kurt    | koksal@rfim.co.uk
 * @date        July 2026
 * @version     1.3.0       /// Initial version (HSS port)
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
*/

#ifndef BSP_CONFIG_H_
#define BSP_CONFIG_H_

/** ----------------------------------------------------------------------------
 * @brief Selects the storage backend for Bootloader 0 persistent data
 * (Bootloader log ring and Boot Parameters).
 *
 * When defined, BSP_Boot_Loader0.c and BSP_Boot_Params.c store their data
 * on NAND Flash through the BSP_Nand_Flash API (NAND0). When undefined,
 * the data is kept in the reserved top region of the golden SW QSPI NOR
 * Flash (MSS QSPI @ 0x21000000: log ring @ 0x01FD0000, Boot Parameters
 * sector @ 0x01FF0000).
 *
 * @note This toggle MUST match the application's BSP_Config.h: the
 * application writes the store the HSS reads. The BSP_Nand_Flash
 * implementation is not available yet; leave this undefined until it is.
 *
 * @note TWEAKABLE macro
 * -----------------------------------------------------------------------------
*/
/// #define BOOT_PARAMS_NAND

/** * @brief Starting sector number for eMMC read/write operations.
 * Defines the logical block address (LBA) where the data transfer begins.
 *
 * For Yocto Linux Image | payload      0u
 * For HSS Payload with Yocto GPT       139264u
 *
 * @note TWAKABLE macro
 */
/// #define MSS_EMMC_SECTOR_NUMBER      (139264u)
#define MSS_EMMC_SECTOR_NUMBER          (0u)

/**
 * @brief Standard hardware block size for the eMMC device.
 * Fixed at 512 bytes per the eMMC specification.
 *
 * @note CONST macro
 */
#define MSS_EMMC_BLOCK_SIZE              (512u)

#endif
