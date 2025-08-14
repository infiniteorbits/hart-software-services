#ifndef MICRON1GFLASH_H
#define MICRON1GFLASH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DIE_ERASE_0_256MB 0
#define DIE_ERASE_256MB_512MB 1
#define DIE_ERASE_512MB_768MB 2
#define DIE_ERASE_768MB_1GB 3

#define ERASE_4K_BLOCK  0
#define ERASE_64K_BLOCK  1

/* ----------------------- Status codes ----------------------- */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_INVALID_ARG,
    FLASH_ERR_SPI,
    FLASH_ERR_TIMEOUT,
    FLASH_ERR_ALIGN,
    FLASH_ERR_UNSUPPORTED
} flash_status_t;

/* ----------------------- Public API ------------------------- */
/**
 * @brief Initialize the SPI flash driver and the SPI core.
 */
flash_status_t FLASH_init(void);

/**
 * @brief Read JEDEC Manufacturer and Device ID.
 * @param[out] manufacturer_id Pointer to store the JEDEC manufacturer ID.
 * @param[out] device_id       Pointer to store the device ID (first byte).
 */
flash_status_t FLASH_read_device_id(uint8_t *manufacturer_id, uint8_t *device_id);

/**
 * @brief Globally unprotect (clear BP bits in Status Register 1).
 *        Intentional: no matching "protect again" helper in this driver.
 */
flash_status_t FLASH_global_unprotect(void);

/**
 * @brief Read bytes from the flash memory.
 * @param address       Start byte address.
 * @param rx_buffer     Destination buffer.
 * @param size_in_bytes Number of bytes to read.
 */
flash_status_t FLASH_read(uint32_t address, uint8_t *rx_buffer, size_t size_in_bytes);

/**
 * @brief Program bytes into the flash (handles 256B page boundaries).
 * @param address       Start byte address.
 * @param write_buffer  Source buffer.
 * @param size_in_bytes Number of bytes to program.
 */
flash_status_t FLASH_program(uint32_t address, const uint8_t *write_buffer, size_t size_in_bytes);

/**
 * @brief Erase a 4 KiB block (address must be 4K-aligned).
 */
flash_status_t FLASH_erase_4k_block(uint32_t address);

/**
 * @brief Erase a 64 KiB block (address must be 64K-aligned).
 */
flash_status_t FLASH_erase_64k_block(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* MICRON1GFLASH_H */

