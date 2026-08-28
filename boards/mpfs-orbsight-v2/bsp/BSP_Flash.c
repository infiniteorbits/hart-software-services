/** ----------------------------------------------------------------------------
 * @file        BSP_Flash.c
 * @brief       QSPI NOR Flash device definitions of the OrbSight-2 board for
 *              the generic infineon_S25FL driver.
 * @author      Koksal Kurt | koksal.kurt@outlook.com
 * @date        July 2026
 * @version     1.3.0       /// Initial version
 *
 * @copyright   RFIM Space 2026
 * -----------------------------------------------------------------------------
 */

#include "BSP_Flash.h"

/* Bitstream Flash: the SC QSPI runs from the 80 MHz SCB clock, /8 = 10 MHz. */
flash_device_t g_flash_device_bitstream = {
    .ctrl_base   = 0x37020100u,
    .clk_div     = QSPI_CLK_DIV_8,
    .is_mss_qspi = 0u,
};

/* Golden SW Flash: the MSS QSPI runs from the 150 MHz AHB clock,
 * /16 = 9.4 MHz. */
flash_device_t g_flash_device_golden_sw = {
    .ctrl_base   = 0x21000000u,
    .clk_div     = QSPI_CLK_DIV_16,
    .is_mss_qspi = 1u,
};
