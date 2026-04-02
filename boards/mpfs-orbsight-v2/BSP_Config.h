/** ----------------------------------------------------------------------------
 * @file        BSP_Config.h
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk
 * @date        January 2026 - February 2026
 *
 * @version     1.2.0       /// Got rid of ethernet link setup macros
 *                          /// Managed now by config functions as per updated
 *                          /// ICD
 *                          /// Added BSP_MAX_NUM_OBSERVERS
 * @version     1.0.5       /// Moved the VERSION macros into separate file
 *                          /// Added link speed macro
 * @version     1.0.4       /// Increased patch version
 *                          /// Moved out UNIT_TEST macros
 * @version     1.0.3       /// Added CONST and TWEAKABLE macro types
 * @version     1.0.2       /// Increased the patch version
 *                          /// Added macro for setting the link speed as
 *                          /// 1Gps w jumbo frames on
 * @version     1.0.1       /// Increased the patch version
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
*/

#ifndef BSP_CONFIG_H_
#define BSP_CONFIG_H_

/** ----------------------------------------------------------------------------
 * @NOTE: Macros are split into two types:
 *  CONST       The user must not change at any circumstance, they are given
 *              as they are and should remain intact
 *  TWEAKABLE   The user is free to modify as per needs
 */

/** ----------------------------------------------------------------------------
 * @brief Global toggle for UART diagnostic messaging.
 *
 * When this macro is defined, the system enables a suite of verbose debug
 * strings directed to the UART console. It allows for real-time monitoring
 * of application flow, hardware initialization status, and error states.
 *
 * @note Undefining this macro in production builds reduces the binary size
 * and eliminates UART transmission overhead.
 *
 * @note TWEAKABLE macro
 * -----------------------------------------------------------------------------
*/
#define DEBUG_PRINT

/** ----------------------------------------------------------------------------
 * @brief Enables intentional ECC error injection for memory testing.
 *
 * When defined, this macro activates diagnostic code paths that deliberately
 * corrupt data or parity bits during memory writes. This is used to verify
 * that the hardware's Error Correction Code (ECC) logic correctly detects
 * (and, if applicable, corrects) memory faults.
 *
 * @warning This should never be enabled in production builds as it
 * intentionally compromises data integrity.
 *
 * @note TWEAKABLE macro
 * -----------------------------------------------------------------------------
*/
/// #define TEST_INJECT_ECC_ERRORS

/** ----------------------------------------------------------------------------
 * @name MSS eMMC Software Update  Configuration
 * @{
 */

/** ----------------------------------------------------------------------------
 * @brief Memory address for storing images uploaded via Ethernet.
 * This constant defines the starting destination in DDR where the incoming
 * software image or data packet is buffered during the Ethernet
 * transfer process.
 *
 * @note TWEAKABLE macro
 * -----------------------------------------------------------------------------
*/
#define SW_IMAGE_ADDR                   (0x80B00000u)


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
 * @brief Total size of the data buffer in bytes.
 * @note This must be a multiple of the block size (512 bytes).
 * For this configuration, it represents 8 blocks (4096 / 512).
 *
 * @note CONST macro
 */
#define MSS_EMMC_BUFFER_SIZE             (4096u)

/**
 * @brief SDMA (Single Operation Direct Memory Access) Enable flag.
 * Set to 1u to enable, 0u to disable. Currently disabled in favor of ADMA2.
 *
 * @note TWEAKABLE macro mutually exclusive with MSS_EMMC_USE_ADMA2
 */
#define MSS_EMMC_USE_SDMA                (0u)

/**
 * @brief ADMA2 (Advanced DMA 2) Enable flag.
 * Set to 1u to utilize the high-performance scatter-gather DMA engine
 * supported by the eMMC host controller.
 *
 * @note TWEAKABLE macro mutually exclusive with MSS_EMMC_USE_SDMA
 */
#define MSS_EMMC_USE_ADMA2               (1u)

/**
 * @brief Standard hardware block size for the eMMC device.
 * Fixed at 512 bytes per the eMMC specification.
 *
 * @note CONST macro
 */
#define MSS_EMMC_BLOCK_SIZE              (512u)

/**
 * @brief Macro to make use of the BSP DDR interface which uses fast
 * pdma transfer
 *
 * @note CONST macro
 */
#define BSP_DDR_READ_WRITE_SW_IMAGE


/**
 * @brief Camera acquisition destination address
 * @note  As of this version only low non-cached memory regions are supported
 *
 * @note TWEAKABLE  macro
 */
#define BSP_CAMERA_ACQ_DEST_ADDR        ((uint64_t)(0xCF000000u + 0x1000u))



/**
 * @brief Address of a camera image chunk to be sent over ethernet
 * @note  As of this version only low non-cached memory regions are supported
 *
 * @note TWEAKABLE  macro
 */
#define BSP_CAMERA_CHUNK_DEST_ADDR      ((uint64_t)(0x88000000u))

/**
 * @brief Camera acquisition DMA timeout -- in ticks
 *
 * @note TWEAKABLE macro
 */
#define BSP_CAMERA_ACQ_DMA_TIMEOUT      (100000000u)

/**
 * @brief Camera acquisition stream descriptor address
 * @note  As of this version only low non-cached memory regions are supported
 *
 * @note TWEAKABLE macro
 */
#define BSP_CAMERA_ACQ_STREAM_DESC_ADDR (0xC0000000u)

/**
 * @brief Ethernet transmit packet size
 *
 * @note TWEAKABLE macro
 */
#define BSP_ETH_PACKET_LENGTH           (1108u)

/**
 * @brief Max number of observers per subject for the HK Event driven machine
 *          based on the Observer design pattern
 *
 * @note TWEAKABLE macro
 */
#define BSP_MAX_NUM_OBSERVERS           (8u)

/**
 * @brief Voltages monitored values for HK -- in milliVolt (mV)
 *
 * @note CONST macros
 * @note TBD: Provide the correct values
 */
#define BSP_TVS_MONITORED_VOLTAGE_V1        (640)
#define BSP_TVS_MONITORED_VOLTAGE_V18       (1010)
#define BSP_TVS_MONITORED_VOLTAGE_V25       (1830)

/**
 * @brief Macro for building with HSS
 *
 * @note CONST macro
 */
#define BSP_HSS_BUILD



/** @}
 * -----------------------------------------------------------------------------
 */

#endif
