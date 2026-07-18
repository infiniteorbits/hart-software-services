/***************************************************************************//**
 * Copyright 2019 - 2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Generic driver for Infineon S25FL-class QSPI NOR Flash memories, driven
 * through the CoreQSPI bare metal driver. See infineon_s25fl.h for the API
 * description. Board support code owns the flash_device_t instances; the
 * driver holds no board knowledge.
 *
 * Adapted by RFIM Space, 2026 (Koksal Kurt | koksal@rfim.co.uk):
 * reworked from the original single-device MSS QSPI implementation into a
 * generic instance-based layer supporting any number of devices behind
 * CoreQSPI-compatible controllers.
 */

#include "infineon_s25fl.h"
#include "core_qspi.h"

#include "mss_peripherals.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Private configuration
 */

/* Flash page program granularity in bytes. */
#define FLASH_PAGE_LENGTH                       (256u)

/* Bounded wait for WIP to clear (status register poll iterations). Each
 * iteration is a full status-read transaction, so this covers the worst-case
 * 64 KB sector erase time with ample margin. */
#define FLASH_WIP_TIMEOUT                       (3000000u)

/* Flash sector size for the 64 KB erase command. */
#define FLASH_SECTOR_SIZE                       (0x00010000u)

/* Flash command opcodes */
#define FLASH_READ_ID_OPCODE                    (0x9Fu)
#define FLASH_WRITE_ENABLE                      (0x06u)   /* WREN  */
#define FLASH_READ_STATUS_REG                   (0x05u)   /* RDSR1 */
#define FLASH_READ_STATUS_REG2                  (0x07u)   /* RDSR2 */
#define FLASH_CLR_STATUS_REG                    (0x30u)   /* CLSR  */
#define FLASH_4BYTE_READ                        (0x13u)
#define FLASH_4BYTE_PAGE_PROG                   (0x12u)
#define FLASH_4BYTE_SECTOR_ERASE                (0xDCu)

/* Status register 1 WIP (write in progress) bit. */
#define FLASH_STATUS_WIP_MASK                   (0x01u)

/* Status register 2 erase error bit (E_ERR). */
#define FLASH_STATUS2_EFAIL_MASK                (0x40u)

/* Status register 2 program error bit (P_ERR). */
#define FLASH_STATUS2_PFAIL_MASK                (0x20u)

/*******************************************************************************
 * Private data
 */

/* Command buffer for page program: opcode + 4 address bytes + page. */
static uint8_t g_flash_cmd_buf[5u + FLASH_PAGE_LENGTH]                       \
        __attribute__ ((aligned (4))) = {0};

/*******************************************************************************
 * Local functions
 */

/*------------------------------------------------------------------------------
 * Validates a device instance, initializing its controller on first use.
 * Returns NULL for an invalid instance.
 */
static flash_device_t*
flash_get_ctx(flash_device_t* device)
{
    if (device == (flash_device_t*)0)
    {
        return (flash_device_t*)0;
    }

    if (device->initialized == 0u)
    {
        Flash_init(device);
    }

    return device;
}

/*------------------------------------------------------------------------------
 * Issues the WRITE ENABLE command. Required before every program or erase
 * operation.
 */
static void
flash_write_enable(flash_device_t* ctx)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) =             \
            {FLASH_WRITE_ENABLE};

    QSPI_polled_transfer_block(&ctx->controller, 0u, command_buf, 0u,        \
            (uint8_t*)0, 0u, 0u);
}

/*------------------------------------------------------------------------------
 * Polls status register 1 until WIP clears or the bounded timeout expires.
 * Returns 0 when WIP cleared, non-zero on timeout.
 */
static uint8_t
flash_wait_wip_clear(flash_device_t* ctx)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) =             \
            {FLASH_READ_STATUS_REG};
    uint8_t  status_buf[4] __attribute__ ((aligned (4))) =                   \
            {FLASH_STATUS_WIP_MASK};
    uint32_t timeout = FLASH_WIP_TIMEOUT;

    do
    {
        QSPI_polled_transfer_block(&ctx->controller, 0u, command_buf, 0u,    \
                status_buf, 1u, 0u);

        if ((status_buf[0] & FLASH_STATUS_WIP_MASK) == 0u)
        {
            return 0u;
        }
        timeout--;
    } while (timeout > 0u);

    return 1u;
}

/*------------------------------------------------------------------------------
 * Reads status register 2 and tests the given error bit, then clears the
 * error flags. Returns 0 when no error, non-zero when the error bit was set.
 */
static uint8_t
flash_error(flash_device_t* ctx, uint8_t error_mask)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) =             \
            {FLASH_READ_STATUS_REG2};
    const uint8_t command_buf_clsr[1] __attribute__ ((aligned (4))) =        \
            {FLASH_CLR_STATUS_REG};
    uint8_t status_buf[4] __attribute__ ((aligned (4))) = {0};
    uint8_t error = 0u;

    QSPI_polled_transfer_block(&ctx->controller, 0u, command_buf, 0u,        \
            status_buf, 1u, 0u);

    if ((status_buf[0] & error_mask) != 0u)
    {
        error = 1u;
    }

    QSPI_polled_transfer_block(&ctx->controller, 0u, command_buf_clsr, 0u,   \
            (uint8_t*)0, 0u, 0u);

    return error;
}

/*------------------------------------------------------------------------------
 * Programs up to one 256-byte page. The address and length must not cross a
 * page boundary. Returns 0 on success, non-zero on program error or timeout.
 */
static uint8_t
flash_program_page(flash_device_t* ctx, const uint8_t* buf,                  \
        uint32_t addr, uint32_t len)
{
    uint32_t idx;

    flash_write_enable(ctx);

    g_flash_cmd_buf[0] = FLASH_4BYTE_PAGE_PROG;
    g_flash_cmd_buf[1] = (uint8_t)((addr >> 24u) & 0xFFu);
    g_flash_cmd_buf[2] = (uint8_t)((addr >> 16u) & 0xFFu);
    g_flash_cmd_buf[3] = (uint8_t)((addr >> 8u) & 0xFFu);
    g_flash_cmd_buf[4] = (uint8_t)(addr & 0xFFu);

    for (idx = 0u; idx < len; idx++)
    {
        g_flash_cmd_buf[5u + idx] = buf[idx];
    }

    QSPI_polled_transfer_block(&ctx->controller, 4u, g_flash_cmd_buf, len,   \
            (uint8_t*)0, 0u, 0u);

    if (flash_wait_wip_clear(ctx) != 0u)
    {
        return 1u;
    }

    return flash_error(ctx, FLASH_STATUS2_PFAIL_MASK);
}

/*******************************************************************************
 * Public API
 */

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_init
(
    flash_device_t* device
)
{
    qspi_config_t qspi_cfg = {0};

    if (device == (flash_device_t*)0)
    {
        return;
    }

    /* The MSS QSPI subblock clock must be enabled before its registers are
     * touched; other controllers (e.g. the SC QSPI) are clocked already.
     * The HSS accesses the QSPI Flash from the E51, so the clock is
     * requested for hart 0. */
    if (device->is_mss_qspi != 0u)
    {
        (void)mss_config_clk_rst(MSS_PERIPH_QSPIXIP, (uint8_t)0u,            \
                PERIPHERAL_ON);
    }

    QSPI_init(&device->controller, (addr_t)device->ctrl_base);

    /* Normal (1-bit) SPI format, mode 3. Only the controller is configured;
     * no Flash device state (addressing mode, configuration registers) is
     * modified. */
    qspi_cfg.clk_div   = device->clk_div;
    qspi_cfg.sample    = QSPI_SAMPLE_POSAGE_SPICLK;
    qspi_cfg.spi_mode  = QSPI_MODE3;
    qspi_cfg.xip       = QSPI_DISABLE;
    qspi_cfg.io_format = QSPI_NORMAL;
    QSPI_configure(&device->controller, &qspi_cfg);

    device->initialized = 1u;
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_readid
(
    flash_device_t* device,
    uint8_t* buf
)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) =             \
            {FLASH_READ_ID_OPCODE};
    flash_device_t* ctx = flash_get_ctx(device);

    if (ctx == (flash_device_t*)0)
    {
        return;
    }

    QSPI_polled_transfer_block(&ctx->controller, 0u, command_buf, 0u,        \
            buf, 3u, 0u);
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_read
(
    flash_device_t* device,
    uint8_t* buf,
    uint32_t addr,
    uint32_t len
)
{
    uint8_t command_buf[5] __attribute__ ((aligned (4))) = {0};
    flash_device_t* ctx = flash_get_ctx(device);

    if (ctx == (flash_device_t*)0)
    {
        return;
    }

    command_buf[0] = FLASH_4BYTE_READ;
    command_buf[1] = (uint8_t)((addr >> 24u) & 0xFFu);
    command_buf[2] = (uint8_t)((addr >> 16u) & 0xFFu);
    command_buf[3] = (uint8_t)((addr >> 8u) & 0xFFu);
    command_buf[4] = (uint8_t)(addr & 0xFFu);

    QSPI_polled_transfer_block(&ctx->controller, 4u, command_buf, 0u,        \
            buf, len, 0u);
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
uint8_t
Flash_program
(
    flash_device_t* device,
    const uint8_t* buf,
    uint32_t addr,
    uint32_t len
)
{
    uint32_t       remaining   = len;
    uint32_t       target_addr = addr;
    const uint8_t* source      = buf;
    flash_device_t* ctx        = flash_get_ctx(device);

    if (ctx == (flash_device_t*)0)
    {
        return 1u;
    }

    while (remaining > 0u)
    {
        /* Never cross a 256-byte page boundary within one program command. */
        uint32_t page_offset = target_addr % FLASH_PAGE_LENGTH;
        uint32_t chunk       = FLASH_PAGE_LENGTH - page_offset;

        if (chunk > remaining)
        {
            chunk = remaining;
        }

        if (flash_program_page(ctx, source, target_addr, chunk) != 0u)
        {
            return 1u;
        }

        remaining   -= chunk;
        target_addr += chunk;
        source      += chunk;
    }

    return 0u;
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
uint8_t
Flash_64KByte_erase
(
    flash_device_t* device,
    uint32_t addr,
    uint32_t len
)
{
    uint8_t  command_buf[5] __attribute__ ((aligned (4))) = {0};
    uint32_t target_addr;
    uint32_t erase_end;
    flash_device_t* ctx = flash_get_ctx(device);

    if (ctx == (flash_device_t*)0)
    {
        return 1u;
    }

    if (len == 0u)
    {
        return 0u;
    }

    target_addr = addr & ~(FLASH_SECTOR_SIZE - 1u);
    erase_end   = addr + len;

    do
    {
        flash_write_enable(ctx);

        command_buf[0] = FLASH_4BYTE_SECTOR_ERASE;
        command_buf[1] = (uint8_t)((target_addr >> 24u) & 0xFFu);
        command_buf[2] = (uint8_t)((target_addr >> 16u) & 0xFFu);
        command_buf[3] = (uint8_t)((target_addr >> 8u) & 0xFFu);
        command_buf[4] = (uint8_t)(target_addr & 0xFFu);

        QSPI_polled_transfer_block(&ctx->controller, 4u, command_buf, 0u,    \
                (uint8_t*)0, 0u, 0u);

        if (flash_wait_wip_clear(ctx) != 0u)
        {
            return 1u;
        }

        if (flash_error(ctx, FLASH_STATUS2_EFAIL_MASK) != 0u)
        {
            return 1u;
        }

        target_addr += FLASH_SECTOR_SIZE;
    } while (target_addr < erase_end);

    return 0u;
}

#ifdef __cplusplus
}
#endif
