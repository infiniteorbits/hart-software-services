/******************************************************************************
 * Micron 1G SPI Flash driver (hardened)
 * - Status codes on all public functions
 * - Timeouts for long-running operations
 * - Parameter validation and alignment checks
 * - 4-byte address mode helper
 * - Clear English comments and small cleanups
 ******************************************************************************/

 #include "micron1gflash.h"
 #include "../CoreSPI/core_spi.h"
 #include "../../../mpfs_hal/mss_hal.h"
 
 #include <string.h>
 
 /* ------------------------ Opcodes & constants ------------------------ */
 #define READ_ARRAY_OPCODE                 0x03
 #define DEVICE_ID_READ                    0x9F
 #define WRITE_ENABLE_CMD                  0x06
 #define WRITE_DISABLE_CMD                 0x04
 #define PROGRAM_PAGE_CMD                  0x02
 #define WRITE_STATUS1_OPCODE              0x01
 #define ERASE_4K_BLOCK_OPCODE             0x20
 #define ERASE_64K_BLOCK_OPCODE            0xD8
 #define READ_STATUS_OPCODE                0x05
 #define READ_FLAG_STATUS_REGISTER         0x70
 #define ADDRESS_MODE_4BYTE                0xB7
 
 #define READY_BIT_MASK                    0x80u   /* Flag Status Register bit7 = Ready */
 
 #define NB_BYTES_PER_PAGE                 256u
 #define ERASE_4K_ALIGNMENT                (4u * 1024u)
 #define ERASE_64K_ALIGNMENT               (64u * 1024u)
 
 /* Driver behavior config */
 #ifndef FLASH_OP_TIMEOUT_LOOPS
 #define FLASH_OP_TIMEOUT_LOOPS            (10000000u) /* Tune for your clock/latency */
 #endif
 
 /* If your device always requires 4-byte addressing, leave as 1. */
 #ifndef FLASH_USE_4BYTE_MODE
 #define FLASH_USE_4BYTE_MODE              1
 #endif
 
 /* ---------------------- Platform / Core SPI setup ---------------------- */
 spi_instance_t g_flash_core_spi;
 #define SPI_INSTANCE            (&g_flash_core_spi)
 #define SPI_SLAVE               0
 
 /* CORESPI base address mapping: adjust to your design if needed. */
 #ifndef CORESPI_BASE_ADDR
 #define CORESPI_BASE_ADDR       0x4F000000UL
 #endif
 
 /* ---------------------- Local helpers (static) ------------------------- */
 static flash_status_t wait_ready_with_timeout(void);
 static void write_enable(void);
 static void write_disable(void);
 static void enter_4byte_address_mode(void);
 static void write_cmd_data(spi_instance_t *this_spi,
                            const uint8_t *cmd_buffer, uint16_t cmd_byte_size,
                            const uint8_t *data_buffer, uint16_t data_byte_size);
 
 /* ---------------------------------------------------------------------- */
 /* Public API                                                             */
 /* ---------------------------------------------------------------------- */
 
 flash_status_t FLASH_init(void)
 {
     /* Configure the SPI core (32-bit frame here is a core setting, not SPI word size). */
     SPI_init(SPI_INSTANCE, CORESPI_BASE_ADDR, 32);
     SPI_configure_master_mode(SPI_INSTANCE);
     return FLASH_OK;
 }
 
 flash_status_t FLASH_read_device_id(uint8_t *manufacturer_id, uint8_t *device_id)
 {
     if (!manufacturer_id || !device_id) {
         return FLASH_ERR_INVALID_ARG;
     }
 
     uint8_t cmd = DEVICE_ID_READ;
     uint8_t read_buffer[3] = {0};
 
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, &cmd, 1, read_buffer, sizeof(read_buffer));
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
     *manufacturer_id = read_buffer[0];
     *device_id       = read_buffer[1]; /* Note: the third byte may be capacity; extend if needed. */
     return FLASH_OK;
 }
 
 flash_status_t FLASH_global_unprotect(void)
 {
     /* Intentional: leave the SPI flash globally unprotected after init.
      *
      * Rationale:
      *  - The boot flow only needs read access after this point; this driver
      *    does not provide a "protect again" helper.
      *  - We unprotect once here to allow any prior write/maintenance step
      *    before handing over to the boot process.
      *  - The device will remain unprotected until reset/power-cycle.
      *
      * Security note:
      *  - If a later stage requires write protection, that stage must
      *    re-establish protection (e.g., set BP bits) or we should extend
      *    this driver with a matching "protect" helper.
      */
     uint8_t cmd[2];
 
     write_enable();
 
     cmd[0] = WRITE_STATUS1_OPCODE;
     cmd[1] = 0x00u; /* BP bits = 0 -> unprotected */
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, cmd, 2, 0, 0);
     flash_status_t st = wait_ready_with_timeout();
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
     return st;
 }
 
 flash_status_t FLASH_read(uint32_t address, uint8_t *rx_buffer, size_t size_in_bytes)
 {
     if (!rx_buffer || size_in_bytes == 0u) {
         return FLASH_ERR_INVALID_ARG;
     }
 
 #if FLASH_USE_4BYTE_MODE
     enter_4byte_address_mode();
 #endif
 
     uint8_t cmd[5] = {
         READ_ARRAY_OPCODE,
         (uint8_t)(address >> 24),
         (uint8_t)(address >> 16),
         (uint8_t)(address >> 8),
         (uint8_t)(address)
     };
 
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, cmd, sizeof(cmd), rx_buffer, size_in_bytes);
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
     return FLASH_OK;
 }
 
 flash_status_t FLASH_program(uint32_t address, const uint8_t *write_buffer, size_t size_in_bytes)
 {
     if (!write_buffer || size_in_bytes == 0u) {
         return FLASH_ERR_INVALID_ARG;
     }
 
 #if FLASH_USE_4BYTE_MODE
     enter_4byte_address_mode();
 #endif
 
     size_t in_buffer_idx = 0;
 
     while (in_buffer_idx < size_in_bytes) {
         /* Respect 256B page boundaries. */
         size_t page_remaining = NB_BYTES_PER_PAGE - ((address + in_buffer_idx) & (NB_BYTES_PER_PAGE - 1u));
         size_t chunk = (size_in_bytes - in_buffer_idx < page_remaining) ?
                         (size_in_bytes - in_buffer_idx) : page_remaining;
 
         write_enable();
 
         uint8_t cmd[5] = {
             PROGRAM_PAGE_CMD,
             (uint8_t)((address + in_buffer_idx) >> 24),
             (uint8_t)((address + in_buffer_idx) >> 16),
             (uint8_t)((address + in_buffer_idx) >> 8),
             (uint8_t)((address + in_buffer_idx))
         };
 
         SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
         write_cmd_data(SPI_INSTANCE, cmd, sizeof(cmd),
                        &write_buffer[in_buffer_idx], (uint16_t)chunk);
         flash_status_t st = wait_ready_with_timeout();
         SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
         if (st != FLASH_OK) {
             return st;
         }
 
         in_buffer_idx += chunk;
     }
 
     write_disable(); /* optional */
     return FLASH_OK;
 }
 
 flash_status_t FLASH_erase_64k_block(uint32_t address)
 {
     if ((address & (ERASE_64K_ALIGNMENT - 1u)) != 0u) {
         return FLASH_ERR_ALIGN;
     }
 
 #if FLASH_USE_4BYTE_MODE
     enter_4byte_address_mode();
 #endif
 
     write_enable();
 
     uint8_t cmd[5] = {
         ERASE_64K_BLOCK_OPCODE,
         (uint8_t)(address >> 24),
         (uint8_t)(address >> 16),
         (uint8_t)(address >> 8),
         (uint8_t)(address)
     };
 
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, cmd, sizeof(cmd), 0, 0);
     flash_status_t st = wait_ready_with_timeout();
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
     return st;
 }
 
 flash_status_t FLASH_erase_4k_block(uint32_t address)
 {
     if ((address & (ERASE_4K_ALIGNMENT - 1u)) != 0u) {
         return FLASH_ERR_ALIGN;
     }
 
 #if FLASH_USE_4BYTE_MODE
     enter_4byte_address_mode();
 #endif
 
     write_enable();
 
     uint8_t cmd[5] = {
         ERASE_4K_BLOCK_OPCODE,
         (uint8_t)(address >> 24),
         (uint8_t)(address >> 16),
         (uint8_t)(address >> 8),
         (uint8_t)(address)
     };
 
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, cmd, sizeof(cmd), 0, 0);
     flash_status_t st = wait_ready_with_timeout();
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 
     return st;
 }
 
 /* ---------------------------------------------------------------------- */
 /* Local helpers                                                          */
 /* ---------------------------------------------------------------------- */
 
 static flash_status_t wait_ready_with_timeout(void)
 {
     /* Poll Flag Status Register (0x70), bit7=1 => Ready */
     uint8_t ready = 0u;
     uint8_t command = READ_FLAG_STATUS_REGISTER;
     uint32_t loops = 0u;
 
     do {
         SPI_transfer_block(SPI_INSTANCE, &command, 1, &ready, sizeof(ready));
         if (++loops > FLASH_OP_TIMEOUT_LOOPS) {
             return FLASH_ERR_TIMEOUT;
         }
     } while ((ready & READY_BIT_MASK) == 0u);
 
     return FLASH_OK;
 }
 
 static void write_enable(void)
 {
     uint8_t cmd = WRITE_ENABLE_CMD;
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, &cmd, 1, 0, 0);
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 }
 
 static void write_disable(void)
 {
     uint8_t cmd = WRITE_DISABLE_CMD;
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, &cmd, 1, 0, 0);
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 }
 
 static void enter_4byte_address_mode(void)
 {
 #if FLASH_USE_4BYTE_MODE
     uint8_t cmd = ADDRESS_MODE_4BYTE;
     SPI_set_slave_select(SPI_INSTANCE, SPI_SLAVE);
     SPI_transfer_block(SPI_INSTANCE, &cmd, 1, 0, 0);
     SPI_clear_slave_select(SPI_INSTANCE, SPI_SLAVE);
 #endif
 }
 
 static void write_cmd_data(spi_instance_t *this_spi,
                            const uint8_t *cmd_buffer, uint16_t cmd_byte_size,
                            const uint8_t *data_buffer, uint16_t data_byte_size)
 {
     (void)this_spi;
     /* Simple concatenation into a small stack buffer; if your platform needs
        DMA/PDMA or larger transfers, switch to DMA or split the transfer. */
     uint16_t transfer_size = (uint16_t)(cmd_byte_size + data_byte_size);
 
     /* Local buffer (cmd + up to 256B of data) */
     uint8_t tx_buffer[5 + NB_BYTES_PER_PAGE];
 
     /* Guard: do not overflow the local buffer */
     if (transfer_size > sizeof(tx_buffer)) {
         transfer_size = sizeof(tx_buffer);
     }
 
     /* Copy command bytes */
     for (uint16_t i = 0; i < cmd_byte_size && i < sizeof(tx_buffer); i++) {
         tx_buffer[i] = cmd_buffer[i];
     }
 
     /* Copy data bytes immediately after the command */
     for (uint16_t i = 0; i < data_byte_size && (i + cmd_byte_size) < sizeof(tx_buffer); i++) {
         tx_buffer[cmd_byte_size + i] = data_buffer[i];
     }
 
     SPI_transfer_block(SPI_INSTANCE, tx_buffer, transfer_size, 0, 0);
 }
 