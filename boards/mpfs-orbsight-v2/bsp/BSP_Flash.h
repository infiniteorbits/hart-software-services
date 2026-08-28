/** ----------------------------------------------------------------------------
 * @file        BSP_Flash.h
 * @brief       QSPI NOR Flash device definitions of the OrbSight-2 board for
 *              the generic infineon_S25FL driver.
 * @author      Koksal Kurt | koksal.kurt@outlook.com
 * @date        July 2026
 * @version     1.3.0       /// Initial version
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_FLASH_H_
#define BSP_FLASH_H_

/* HSS port: the generic S25FL driver lives at baremetal/drivers/
 * infineon_s25fl in the HSS tree (same driver as the application's
 * drivers/off_chip/infineon_S25FL); this include is the only line that
 * differs from the application copy of this file. */
#include "infineon_s25fl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * @brief The two identical S25FL256-class QSPI NOR Flash devices of the
 * OrbSight-2 board:
 *
 * - Bitstream Flash: System Controller QSPI @ 0x37020100 (SCB bus). Holds
 *   the SPI directory and the golden / UPDATE / IAP bitstream slots used by
 *   the System Controller for IAP / Auto Update.
 * - Golden SW Flash: MSS QSPI @ 0x21000000. Holds the golden SW image, the
 *   bootloader log ring and the boot parameters store.
 * --------------------------------------------------------------------------
 */
extern flash_device_t g_flash_device_bitstream;
extern flash_device_t g_flash_device_golden_sw;

/* --------------------------------------------------------------------------
 * @brief Device selectors passed to the Flash_*() driver API.
 * --------------------------------------------------------------------------
 */
#define FLASH_DEVICE_BITSTREAM      (&g_flash_device_bitstream)
#define FLASH_DEVICE_GOLDEN_SW      (&g_flash_device_golden_sw)

#ifdef __cplusplus
}
#endif

#endif /* BSP_FLASH_H_ */
