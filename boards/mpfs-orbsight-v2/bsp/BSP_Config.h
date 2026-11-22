/** ----------------------------------------------------------------------------
 * @file        BSP_Config.h
 * @brief       HSS-side subset of the application BSP_Config.h: only the
 *              macros consumed by the shared Bootloader 0 modules
 *              (BSP_Boot_Params, BSP_Boot_Loader0, BSP_Nand_Flash) are
 *              carried here. The full configuration lives in the
 *              application tree (src/application/bsp/BSP_Config.h); the
 *              macros below must be kept in sync with it.
 * @author      Koksal Kurt    | koksal.kurt@outlook.com
 * @date        July 2026 - November 2026
 * @version     1.4.0       /// BSP_Nand_Flash driver (and the NAND
 *                          /// backend of BSP_Boot_Params /
 *                          /// BSP_Boot_Loader0) brought over from the
 *                          /// application: NAND Flash Configuration added
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
 * application writes the store the HSS reads. The NAND backend uses the
 * BSP_Nand_Flash driver (see NAND Flash Configuration below).
 *
 * @note TWEAKABLE macro
 * -----------------------------------------------------------------------------
*/
#define BOOT_PARAMS_NAND

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

/** ----------------------------------------------------------------------------
 * @name NAND Flash Configuration
 * @{
 */

/**
 * @brief DDR buffers of the NAND driver (command stream, page staging and
 * ECC scratch; BSP_Nand_Flash.c). Same region as the application.
 *
 * The fabric DMA engines are not cache coherent and the driver also uses the
 * non-cached alias of the buffers (address + 0x40000000), so the region must
 * lie in the cached DDR aperture below 0x90000000. In the HSS, DDR is
 * trained before the boot service reads the Boot Parameters, and the region
 * is overwritten on every NAND access - including the Boot Parameters
 * write after the payload has been loaded - so it must not overlap the
 * payload image (OrbSight-2 payload at 0x80000000; boot image staging at
 * CONFIG_SERVICE_BOOT_DDR_TARGET_ADDR).
 *
 * @note TWEAKABLE macros. Keep in sync with the application's BSP_Config.h.
 */
#define BSP_NAND_DMA_BUF_ADDR               (0x8E000000u)
#define BSP_NAND_DMA_BUF_SIZE               (0x00040000u)   /// 256 KB

/**
 * @brief Forces the synchronous (NV-DDR) interface on every target.
 *
 * BSP_nand_init() then never falls back to the asynchronous SDR path: not
 * when the ONFI parameter page reports no NV-DDR support, and not when the
 * synchronous READ ID check fails. Undefine it to restore the automatic
 * SDR fallback.
 *
 * @warning A part without NV-DDR (MT29F64G08ADAAA3W on the 64G units), or a
 * PHY whose synchronous reads time out, is not expected to work in this
 * mode: expect BSP_nand_read() to fail. With BOOT_PARAMS_NAND defined the
 * HSS then cannot read the Boot Parameters and falls back to the default
 * boot flow.
 *
 * @note TWEAKABLE macro. Keep in sync with the application's BSP_Config.h.
 */
#define BSP_NAND_FORCE_SYNC

/** @}
 * -----------------------------------------------------------------------------
 */

#endif
