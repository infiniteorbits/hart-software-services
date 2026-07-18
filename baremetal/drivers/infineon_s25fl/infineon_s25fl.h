/***************************************************************************//**
 * Copyright 2019 - 2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Generic driver for Infineon S25FL-class QSPI NOR Flash memories sitting
 * behind register-compatible QSPI controllers (CoreQSPI RTL v2 layout),
 * driven through the CoreQSPI bare metal driver.
 *
 * The driver holds no board knowledge: each physical Flash device is
 * described by a caller-owned flash_device_t instance carrying the QSPI
 * controller base address and SPI clock configuration. Board support code
 * defines one instance per on-board device and passes it to every API call.
 *
 * Command sequences follow the RFIM YMODEM Autoprogram reference
 * implementation: write enable (0x06) before every program and erase, WIP
 * polling on RDSR1 (0x05), error detection on RDSR2 (0x07, E_ERR bit 6 /
 * P_ERR bit 5), error clearing with CLSR (0x30). The dedicated stateless
 * 4-byte-address opcodes are used (0x13 read, 0x12 page program, 0xDC 64 KB
 * erase), so the device addressing mode is never changed and boot agents
 * (e.g. the PolarFire SoC System Controller) always find the Flash in its
 * default state.
 *
 * All functions initialize the controller of the given device lazily on
 * first use, so calling Flash_init() explicitly is optional.
 *
 * Adapted by RFIM Space, 2026 (Koksal Kurt | koksal@rfim.co.uk).
 */

#ifndef INFINEON_S25FL_H_
#define INFINEON_S25FL_H_

#include <stdint.h>
#include "core_qspi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------------------------------------*//**
  Describes one S25FL QSPI NOR Flash device. The caller owns the instance:
  set ctrl_base, clk_div and is_mss_qspi, and zero-initialize the remaining
  fields. The same instance must be passed to every API call targeting the
  device.
*/
typedef struct flash_device {
    uint32_t        ctrl_base;    /**< QSPI controller register base address */
    qspi_clk_div    clk_div;      /**< SPI clock divider from the controller
                                       input clock                           */
    uint8_t         is_mss_qspi;  /**< Non-zero when the controller is the MSS
                                       QSPI, whose subblock clock must be
                                       enabled before register access        */
    uint8_t         initialized;  /**< Set by Flash_init(); zero-initialize  */
    qspi_instance_t controller;   /**< CoreQSPI driver instance; owned by the
                                       driver, zero-initialize               */
} flash_device_t;

/*-------------------------------------------------------------------------*//**
  The Flash_init() function initializes the QSPI controller of the given
  Flash device for normal (1-bit) SPI operations. Only the controller is
  configured; no Flash device state (addressing mode, configuration
  registers) is modified.

  @param device
  Target Flash device.

  @return
    This function does not return a value.
*/
void
Flash_init
(
    flash_device_t* device
);

/*-------------------------------------------------------------------------*//**
  The Flash_readid() function reads the first 3 bytes of the device JEDEC ID.

  @param device
  Target Flash device.

  @param buf
  Buffer of at least 3 bytes receiving the JEDEC ID.

  @return
    This function does not return a value.
*/
void
Flash_readid
(
    flash_device_t* device,
    uint8_t* buf
);

/*-------------------------------------------------------------------------*//**
  The Flash_read() function reads data from the given Flash device.

  @param device
  Target Flash device.

  @param buf
  Destination buffer.

  @param addr
  Flash byte address to read from.

  @param len
  Number of bytes to read.

  @return
    This function does not return a value.
*/
void
Flash_read
(
    flash_device_t* device,
    uint8_t* buf,
    uint32_t addr,
    uint32_t len
);

/*-------------------------------------------------------------------------*//**
  The Flash_program() function writes data into the given Flash device.
  The target range must have been erased beforehand. The buffer is split
  into 256-byte page programs internally.

  @param device
  Target Flash device.

  @param buf
  Source buffer.

  @param addr
  Flash byte address to program.

  @param len
  Number of bytes to program.

  @return
    0 on success, non-zero on program error or timeout.
*/
uint8_t
Flash_program
(
    flash_device_t* device,
    const uint8_t* buf,
    uint32_t addr,
    uint32_t len
);

/*-------------------------------------------------------------------------*//**
  The Flash_64KByte_erase() function erases the 64 KB sectors of the given
  Flash device covering [addr, addr + len).

  @param device
  Target Flash device.

  @param addr
  Flash byte address (aligned down to a 64 KB boundary internally).

  @param len
  Number of bytes to erase (rounded up to full sectors).

  @return
    0 on success, non-zero on erase error or timeout.
*/
uint8_t
Flash_64KByte_erase
(
    flash_device_t* device,
    uint32_t addr,
    uint32_t len
);

#ifdef __cplusplus
}
#endif

#endif /* INFINEON_S25FL_H_*/
