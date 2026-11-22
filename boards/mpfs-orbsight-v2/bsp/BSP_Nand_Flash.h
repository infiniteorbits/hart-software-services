/** ----------------------------------------------------------------------------
 * @file        BSP_Nand_Flash.h
 * @brief       NAND Flash driver for the OrbSight-2 fabric NAND controller
 *              (nand_phy / nand_os2_sdrrd PHY fed by CoreAXI4ProtoConv
 *              MM2S / S2MM streams on FIC-1).
 * @author      Koksal Kurt | koksal.kurt@outlook.com
 * @date        July 2026 - October 2026
 * @version     1.4.0       /// Stub replaced by the NAD-Flash bsp_nand
 *                          /// driver (branch nand-sdr-read-os2 @ 9ee2519),
 *                          /// adapted to this interface: no device id,
 *                          /// 64-bit byte addresses.
 *                          /// Added BSP_nand_program (program without
 *                          /// erase) and the public geometry macros.
 *                          /// BSP_Nand_Flash_Config.h, _Onfi.h and
 *                          /// _Bch8.c/.h merged into BSP_Nand_Flash.c; the
 *                          /// bring-up / diagnostic API is no longer
 *                          /// exported.
 *                          /// All four chip enables: 32 GB
 *                          /// (BSP_NAND_NUM_TARGETS), CE2..CE4 used once
 *                          /// they pass the init check, else
 *                          /// BSP_NAND_ERR_NO_TARGET
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
 * @brief NAND device geometry: data bytes per page, pages per erase block,
 * erase-block size, and the blocks and bytes the driver addresses.
 *
 * The 32 GB package has four targets, one per chip enable CE1..CE4, each of
 * 2 LUNs x 4096 blocks = 8 GB (ONFI parameter page of the MT29F256G08AUCABH3
 * and the MT29F64G08ADAAA3W; block b of LUN 1 is b + 4096). Block b of the
 * device is block b % BSP_NAND_TARGET_BLOCKS of chip enable
 * b / BSP_NAND_TARGET_BLOCKS + 1: CE1 holds the first 8 GB, CE4 the last.
 *
 * CE2..CE4 need an R/B# of their own (FPGA PHY with the per-CE R/B# change,
 * NAND_RB2_N..RB4_N routed); BSP_nand_init() checks each of them, see there.
 * --------------------------------------------------------------------------
*/
#define BSP_NAND_PAGE_SIZE          (8192u)
#define BSP_NAND_PAGES_PER_BLOCK    (128u)
#define BSP_NAND_BLOCK_SIZE         (BSP_NAND_PAGE_SIZE * BSP_NAND_PAGES_PER_BLOCK)
#define BSP_NAND_NUM_TARGETS        (4u)
#define BSP_NAND_TARGET_BLOCKS      (8192u)
#define BSP_NAND_NUM_BLOCKS         (BSP_NAND_TARGET_BLOCKS * BSP_NAND_NUM_TARGETS)
#define BSP_NAND_DEVICE_SIZE        ((uint64_t)BSP_NAND_BLOCK_SIZE * BSP_NAND_NUM_BLOCKS)

/* --------------------------------------------------------------------------
 * @brief BSP return codes for NAND operations.
 * --------------------------------------------------------------------------
*/
typedef enum {
    BSP_NAND_OK = 0, /**< Operation completed successfully */
    BSP_NAND_ERR_OUT_OF_RANGE, /**< addr + length exceeds device geometry */
    BSP_NAND_ERR_READ_FAILED, /**< Read transfer failed or the page is
                                  uncorrectable */
    BSP_NAND_ERR_WRITE_FAILED, /**< Page program failure */
    BSP_NAND_ERR_ERASE_FAILED, /**< Block erase failure */
    BSP_NAND_ERR_ALIGNMENT, /**< Write not aligned to page size */
    BSP_NAND_ERR_BAD_BLOCK, /**< Attempt to access a marked bad block */
    BSP_NAND_ERR_INTERNAL, /**< Unexpected internal failure */
    BSP_NAND_ERR_NO_TARGET /**< The range reaches a chip enable that did not
                               pass the init check (see BSP_nand_init) */
} bsp_nand_status_t;

/* --------------------------------------------------------------------------
 * @brief Total BCH-8 bits corrected by BSP_nand_read() since power-up.
 *
 * Read-only for callers. Sampling it around a read shows whether that read
 * needed correction (BSP_Memory_Scrubbing). Only the SDR path of a v9+ PHY
 * corrects with BCH-8 and moves it; on the NV-DDR path it stays put.
 * --------------------------------------------------------------------------
*/
extern uint32_t g_bch_corrected;

/* --------------------------------------------------------------------------
 * @brief Initializes the NAND Flash controller and device.
 *
 * Optional: BSP_nand_read / BSP_nand_write / BSP_nand_program /
 * BSP_nand_erase initialize on first use. Calling it explicitly lets the
 * application report an initialization failure at boot; calling it again
 * re-runs the full sequence.
 *
 * The PHY variant is read from its signature (debug window 12). On an
 * SDR-capable PHY (nand_os2_sdrrd) the ONFI parameter page is read in the
 * asynchronous interface first; a part without NV-DDR support stays
 * asynchronous, the fast SDR read timing is calibrated and all later access
 * goes through the SDR path (BCH-8 ECC on a v9+ PHY). Otherwise the
 * synchronous path is set up: RESET, SET FEATURES (timing mode) and the
 * synchronous clock; if a synchronous READ ID then fails, the part is reset
 * back to the asynchronous interface and the SDR path is used instead.
 * Every target goes through the same steps and all of them use one
 * interface: NV-DDR only if it works on each.
 *
 * CE1 is always used. CE2..CE4 are used only on an SDR-capable PHY and only
 * if their RESET shows a busy pulse on the R/B# the PHY watches (it is
 * their own only with the per-CE R/B# FPGA change and NAND_RB2_N..RB4_N
 * routed) and their ONFI parameter page reports the driver's geometry. A
 * target that fails the check is left alone: its addresses return
 * BSP_NAND_ERR_NO_TARGET, and the rest of the device works as before.
 *
 * No bad-block table is built.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_INTERNAL if the init command stream could not be sent or
 * CE1 reports another geometry than BSP_NAND_* (the address map would not
 * fit the part).
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_init(void);

/* --------------------------------------------------------------------------
 * @brief Reads data from the NAND Flash device.
 *
 * Any byte range, page by page; an erased page reads as 0xFF. In SDR mode
 * on a v9+ PHY every page is BCH-8 corrected and an uncorrectable page
 * fails the read; corrected bit counts are not returned (see
 * g_bch_corrected). In NV-DDR mode the Hamming decoder corrects one bit per
 * bit lane and 512-word block and reports nothing.
 *
 * @param dst Destination buffer where the data will be copied.
 * @param addr Absolute byte address inside the NAND device.
 * @param length Number of bytes to read.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_OUT_OF_RANGE if length is 0 or the range exceeds the device,
 * BSP_NAND_ERR_READ_FAILED on a transfer or uncorrectable ECC error,
 * BSP_NAND_ERR_NO_TARGET if the range reaches a chip enable not in use,
 * BSP_NAND_ERR_INTERNAL if dst is NULL or initialization fails.
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_read(void* dst, uint64_t addr, uint64_t length);

/* --------------------------------------------------------------------------
 * @brief Writes data to the NAND Flash device.
 *
 * Erases every block the range touches, then programs it page by page.
 * The erase is block-wide: data in the touched blocks outside
 * [addr, addr + length) is lost.
 *
 * @param src Source buffer containing data to write.
 * @param addr Absolute byte address inside the NAND device; a multiple of
 * the page size (8192 bytes).
 * @param length Number of bytes to write; a multiple of the page size.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_OUT_OF_RANGE if length is 0 or the range exceeds the device,
 * BSP_NAND_ERR_ALIGNMENT if addr or length is not page aligned,
 * BSP_NAND_ERR_ERASE_FAILED / BSP_NAND_ERR_WRITE_FAILED on a device error,
 * BSP_NAND_ERR_NO_TARGET if the range reaches a chip enable not in use,
 * BSP_NAND_ERR_INTERNAL if src is NULL or initialization fails.
 *
 * @note On the synchronous (NV-DDR) path the program and erase status is
 * not read back; only the SDR path checks the status FAIL bit.
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_write(const void* src, uint64_t addr, uint64_t length);

/* --------------------------------------------------------------------------
 * @brief Programs data into erased NAND pages without erasing.
 *
 * For appending to a block that already holds data (log records, parameter
 * slots): the rest of the block is left untouched. The caller owns the
 * erase. A partial last page is padded with the erased value 0xFF. Program
 * every page only once between two erases of its block: NAND pages cannot
 * be partially reprogrammed.
 *
 * @param src Source buffer containing data to program.
 * @param addr Absolute byte address inside the NAND device; a multiple of
 * the page size (BSP_NAND_PAGE_SIZE).
 * @param length Number of bytes to program; any non-zero length.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_OUT_OF_RANGE if length is 0 or the range exceeds the device,
 * BSP_NAND_ERR_ALIGNMENT if addr is not page aligned,
 * BSP_NAND_ERR_WRITE_FAILED on a device error,
 * BSP_NAND_ERR_NO_TARGET if the range reaches a chip enable not in use,
 * BSP_NAND_ERR_INTERNAL if src is NULL or initialization fails.
 *
 * @note On the synchronous (NV-DDR) path the program status is not read
 * back; only the SDR path checks the status FAIL bit.
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_program(const void* src, uint64_t addr, uint64_t length);

/* --------------------------------------------------------------------------
 * @brief Erase all NAND blocks touched by the given address range.
 *
 * Caller specifies a byte range, NOT block indices.
 *
 * The BSP internally:
 * - computes first_block = address / block_size
 * - computes last_block = (address + length - 1) / block_size
 * - erases all blocks in the interval
 *
 * Bad blocks are NOT skipped.
 *
 * @param[in] address Start address in bytes.
 * @param[in] length Number of bytes whose blocks must be erased.
 *
 * @return BSP_NAND_OK on success,
 * BSP_NAND_ERR_OUT_OF_RANGE if length is 0 or the range exceeds the device,
 * BSP_NAND_ERR_ERASE_FAILED on a device error,
 * BSP_NAND_ERR_NO_TARGET if the range reaches a chip enable not in use,
 * BSP_NAND_ERR_INTERNAL if initialization fails.
 * --------------------------------------------------------------------------
*/
bsp_nand_status_t
BSP_nand_erase(uint64_t address, uint64_t length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_NAND_H_ */
