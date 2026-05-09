/***************************************************************************//** * *
 * Driver for infineon_s25fl QSPI flash memory.
 * This driver uses the MPFS MSS QSPI driver interface.
 *
 */
#include "drivers/infineon_s25fl/infineon_s25fl.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"

/*Following constant must be defined if you want to use the interrupt mode
  transfers provided by the MSS QSPI driver. Comment this out to use the polling
  mode transfers.*/
//#define USE_QSPI_INTERRUPT      1u

#ifdef __cplusplus
extern "C" {
#endif

#ifdef  USE_QSPI_INTERRUPT
#define QSPI_TRANSFER_BLOCK(num_addr_bytes, tx_buffer, tx_byte_size, rd_buffer, rd_byte_size, num_idle_cycles) \
        {\
            MSS_QSPI_irq_transfer_block((num_addr_bytes), (tx_buffer), (tx_byte_size), (rd_buffer), (rd_byte_size), (num_idle_cycles));\
            if((tx_buffer && !rd_buffer)) \
            {wait_for_tx_complete();} \
            if((rd_buffer && !tx_buffer)) \
            {wait_for_tx_complete();} \
            if((tx_buffer && rd_byte_size && rd_buffer && tx_byte_size)) \
              {wait_for_rx_complete();\
              wait_for_tx_complete(); \
              } \
        }
#else
#define QSPI_TRANSFER_BLOCK(num_addr_bytes, tx_buffer, tx_byte_size, rd_buffer, rd_byte_size, num_idle_cycles) \
    MSS_QSPI_polled_transfer_block((num_addr_bytes), (tx_buffer), (tx_byte_size), (rd_buffer), (rd_byte_size), (num_idle_cycles));
#endif

#define PAGE_LENGTH                             256u

#define INFINEON_RESET_ENABLE                     0x66
#define INFINEON_RESET_MEMORY                     0x99

#define INFINEON_READ_ID_OPCODE                   0x9F
#define INFINEON_MIO_READ_ID_OPCODE               0xAF

#define INFINEON_READ                             0x03
#define INFINEON_FAST_READ                        0x0B

#define INFINEON_4BYTE_READ                       0x13
#define INFINEON_4BYTE_FAST_READ                  0x0C
#define INFINEON_4BYTE_DUALO_FAST_READ            0x3C
#define INFINEON_4BYTE_DUALIO_FAST_READ           0xBC
#define INFINEON_4BYTE_QUADO_FAST_READ            0x6C
#define INFINEON_4BYTE_QUADIO_FAST_READ           0xEC

#define INFINEON_WRITE_ENABLE                     0x06
#define INFINEON_WRITE_DISABLE                    0x04

#define INFINEON_READ_STATUS_REG                  0x05
#define INFINEON_READ_FLAG_STATUS_REG             0x07
#define INFINEON_READ_ENH_V_CONFIG_REG            0x65

#define INFINEON_WR_STATUS_REG                    0x01
#define INFINEON_WR_NV_CONFIG_REG                 0xB1
#define INFINEON_WR_V_CONFIG_REG                  0x81
#define INFINEON_WR_ENH_V_CONFIG_REG              0x61
#define INFINEON_WR_EXT_ADDR_REG                  0xC5
#define INFINEON_CLR_FLAG_STATUS_REG              0x30

#define INFINEON_PAGE_PROGRAM                     0x02
#define INFINEON_DUAL_INPUT_FAST_PROG             0xA2
#define INFINEON_EXT_DUAL_INPUT_FAST_PROG         0xD2
#define INFINEON_QUAD_INPUT_FAST_PROG             0x32
#define INFINEON_EXT_QUAD_INPUT_FAST_PROG         0x38

#define INFINEON_4BYTE_PAGE_PROG                  0x12
#define INFINEON_4BYTE_QUAD_INPUT_FAST_PROG       0x34
#define INFINEON_4BYTE_QUAD_INPUT_EXT_FAST_PROG   0x3E

#define INFINEON_SECTOR_ERASE                     0xDC
#define INFINEON_BULK_ERASE                       0xC7

#define INFINEON_ENTER_4BYTE_ADDR_MODE            0xB7
#define INFINEON_EXIT_4BYTE_ADDR_MODE             0x29

#define INFINEON_JEDEC_ID                         0x01u

mss_qspi_config_t beforexip_qspi_config={0};
mss_qspi_config_t g_qspi_config = {0};
mss_qspi_config_t qspi_config_read={0};
#ifdef NVDEBUG
extern uint8_t g_ui_buf[500];
extern mss_uart_instance_t *g_uart;
#endif
volatile uint8_t g_rx_complete = 0u;
volatile uint8_t g_tx_complete = 0u;
static volatile uint8_t g_enh_v_val __attribute__ ((aligned (4))) = 0x0u;

/*******************************************************************************
 * Local functions
 */
static void read_statusreg(uint8_t* rd_buf);

static void read_enh_v_cfgreg(uint8_t* rd_buf);
static void read_flagstatusreg(uint8_t* rd_buf);
static void enable_4byte_addressing(void);
static void disable_4byte_addressing(void);
static void device_reset(void);
static void write_enh_v_confreg(uint8_t* enh_v_val);
static mss_qspi_io_format probe_io_format(void);
static mss_qspi_io_format update_io_format(mss_qspi_io_format t_io_format);
static uint8_t program_page(uint8_t* buf,uint32_t page,uint32_t len);

#ifdef USE_QSPI_INTERRUPT
void transfer_status_handler(uint32_t status)
{
    if (STTS_RDONE_MASK == (STTS_RDONE_MASK & status))
    {
        g_rx_complete = 1;
    }
    else if (STTS_TDONE_MASK == (STTS_TDONE_MASK & status))
    {
        g_tx_complete = 1;
    }
}

static void wait_for_tx_complete(void)
{
    //while (0u == g_tx_complete);
    g_tx_complete = 0u;
}

static void wait_for_rx_complete(void)
{
    //while (0u == g_rx_complete);
    g_rx_complete = 0u;
}
#endif

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_init
(
    mss_qspi_io_format io_format
)
{
    volatile mss_qspi_io_format t_io_format = MSS_QSPI_NORMAL;

    MSS_QSPI_init();

#ifdef USE_QSPI_INTERRUPT
    MSS_QSPI_set_status_handler(transfer_status_handler);
#endif

    g_qspi_config.clk_div =  MSS_QSPI_CLK_DIV_30;     //Tested OK INFINEON_FAST_READ command at MSS_QSPI_CLK_DIV_12
    g_qspi_config.sample = MSS_QSPI_SAMPLE_POSAGE_SPICLK;
    g_qspi_config.spi_mode = MSS_QSPI_MODE3;
    g_qspi_config.xip = MSS_QSPI_DISABLE;
    g_qspi_config.io_format = MSS_QSPI_NORMAL;
    MSS_QSPI_configure(&g_qspi_config);

    device_reset();

    /* Find out the current mode of the flash memory device
     * and configure qspi controller to that mode.*/
    t_io_format = probe_io_format();

    g_qspi_config.io_format = t_io_format;
    MSS_QSPI_configure(&g_qspi_config);

    /* If the desired IO format is same as the currently configured IO Format
     * Then we are done. Otherwise configure the Flash and QSPI controller
     * to the IO format provided by the user.
     */
    if (io_format != t_io_format)
    {
        g_qspi_config.io_format = t_io_format;
        MSS_QSPI_configure(&g_qspi_config);

        read_enh_v_cfgreg((uint8_t*)&g_enh_v_val);

        if (io_format == MSS_QSPI_QUAD_FULL)
        {
            g_enh_v_val |= 0x40u; /* set the dual mode bit*/
            g_enh_v_val &= ~0x80u; /*clear the quad mode bit*/
        }
        else if (io_format == MSS_QSPI_DUAL_FULL)
        {
            g_enh_v_val |= 0x80u;  /*set the quad mode bit*/
            g_enh_v_val &= ~0x40u; /*clear the dual mode but*/
        }
        else
        {
            g_enh_v_val |= 0xC0u; /*normal*/
        }
        write_enh_v_confreg((uint8_t*)&g_enh_v_val);
        read_enh_v_cfgreg((uint8_t*)&g_enh_v_val);
    }

    g_qspi_config.io_format = io_format;
    MSS_QSPI_configure(&g_qspi_config);
    enable_4byte_addressing();

    MSS_QSPI_enable();
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_readid
(
    uint8_t* buf
)
{
    uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_READ_ID_OPCODE};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = g_qspi_config.io_format;
    if ((t_io_format != MSS_QSPI_QUAD_FULL) && (t_io_format != MSS_QSPI_DUAL_FULL)) {
        t_io_format = update_io_format(MSS_QSPI_NORMAL);
        QSPI_TRANSFER_BLOCK(0, command_buf, 0, buf, 3, 0);
        update_io_format(t_io_format);
    } else {
        command_buf[0] = INFINEON_MIO_READ_ID_OPCODE;
        QSPI_TRANSFER_BLOCK(0, command_buf, 0, buf, 3, 0);
    }
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_read
(
    uint8_t* buf,
    uint32_t addr,
    uint32_t len
)
{
    uint8_t dummy_cycles = 0u;
    uint8_t command_buf[10] __attribute__ ((aligned (4))) = {0u};

    command_buf[1] = (addr >> 24u) & 0xFFu;
    command_buf[2] = (addr >> 16u) & 0xFFu;
    command_buf[3] = (addr >> 8u) & 0xFFu;
    command_buf[4] = addr & 0xFFu;

    switch(g_qspi_config.io_format)
    {
       case MSS_QSPI_NORMAL:
           command_buf[0] = INFINEON_4BYTE_FAST_READ;
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_DUAL_EX_RO:
           command_buf[0] = INFINEON_4BYTE_DUALO_FAST_READ;   /* 1-1-2 */
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_QUAD_EX_RO:
           command_buf[0] = INFINEON_4BYTE_QUADO_FAST_READ;   /* 1-1-4 */
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_DUAL_EX_RW:
           command_buf[0] = INFINEON_4BYTE_DUALIO_FAST_READ;   /* 1-2-2 */
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_QUAD_EX_RW:
           command_buf[0] = INFINEON_4BYTE_QUADIO_FAST_READ;   /* 1-4-4 */
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_DUAL_FULL:
           command_buf[0] = INFINEON_4BYTE_FAST_READ;
           dummy_cycles = 8u;
           break;
       case MSS_QSPI_QUAD_FULL:
           command_buf[0] = INFINEON_4BYTE_FAST_READ;
           dummy_cycles = 8u;                              /* For Quad mode */
           break;
       default:
           ASSERT(0);
           break;
    }

    QSPI_TRANSFER_BLOCK(4, command_buf, 0, buf, len, dummy_cycles);
}

/***************************************************************************//**
 * See infineon_s25fl for details of how to use this function.
 */
uint8_t
Flash_program
(
    uint8_t* buf,
    uint32_t addr,
    uint32_t len
)
{
    int32_t remaining_length = (int32_t)len;
    uint32_t target_offset = addr;
    uint8_t status = 0xFF;

    while(remaining_length > 0)
    {
        uint32_t page_length;

        if(remaining_length >= PAGE_LENGTH)
        {
            page_length = PAGE_LENGTH;
        }
        else
        {
            page_length = remaining_length;
        }

        status = program_page(buf, target_offset, page_length);

        remaining_length -= page_length;
        target_offset += page_length;
        buf += page_length;
    }

    return (status);
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
uint8_t
Flash_erase(void)
{
    uint8_t status = 0xFFu;
    uint8_t command_buf[5] __attribute__ ((aligned (4))) = {INFINEON_WRITE_ENABLE};
    /*Both Write enable and Die erase can work in all modes*/

    /* Write enable command must be executed before erase
     * WRITE ENABLE 06h 1-0-0 2-0-0 4-0-0 0 no dummy cycles.
     * */
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);

    /* Erase the die. This will write 1 to all bits
     * DIE ERASE C4h 1-1-0 2-2-0 4-4-0 no dummy cycles
     * */
    command_buf[0] = INFINEON_BULK_ERASE;
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);
    update_io_format(t_io_format);

    while (1){
        read_statusreg(&status);
        if (!(status & 0x1))        // wait until Bit [0] WIP = 0
            break;
    }

    read_flagstatusreg(&status);

    return(status & (0x1 << 6));    /// E_ERR

}

uint8_t
Flash_sector_erase(uint32_t addr)
{
    mHSS_DEBUG_PRINTF(LOG_ERROR, "Sector Erase Called for addr: %d\n", addr);

    uint8_t status = 0u;
    uint8_t command_buf[5] __attribute__ ((aligned (4))) = {0};
    /*execute Enter 4-byte mode command */
    command_buf[0] = INFINEON_ENTER_4BYTE_ADDR_MODE;
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);

    /*execute Write enable command again for writing the data*/
//    command_buf[0] = INFINEON_WRITE_ENABLE;
//    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);

        /*execute Write enable command again for writing the data*/
        command_buf[0] = INFINEON_WRITE_ENABLE;
        QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);

        command_buf[1] = (addr >> 24) & 0xFFu;
        command_buf[2] = (addr >> 16) & 0xFFu;
        command_buf[3] = (addr >> 8) & 0xFFu;
        command_buf[4] = addr & 0xFFu;
        command_buf[0] = INFINEON_SECTOR_ERASE;   // erase one 64KByte Block
        QSPI_TRANSFER_BLOCK(4, command_buf, 0, (uint8_t*)0u, 0u, 0u);

        while (1){
            read_statusreg(&status);
            if (!(status & 0x1))        // wait until Bit [0] WIP = 0
                break;
        }

        read_flagstatusreg(&status);
        if (status & (0x1 << 6))    /// E_ERR
            status = 1u;

    return 0u;

}



/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_enter_xip
(
    void
)
{
    disable_4byte_addressing();

    uint8_t command_buf[5] __attribute__ ((aligned (4))) = {INFINEON_WRITE_ENABLE};
    uint32_t temp;

    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);

    command_buf[0] = INFINEON_WR_V_CONFIG_REG;
    command_buf[1] = 0xF3u;     /*Enable XIP*/

    /*Enable XIP by writing to volatile configuration register*/
    QSPI_TRANSFER_BLOCK(0, command_buf, 1, (uint8_t*)0, 0, 0);
    /*Drive XIP confirmation using FAST read and keeping DQ0 to 0 during idle cycle*/
    command_buf[0] = INFINEON_FAST_READ;
    command_buf[1] = 0x00u;
    command_buf[2] = 0x00u;
    command_buf[3] = 0x00u;

    /*Following command must be sent in polling method only.
      Using interrupt method is not possible here because, after sending this
      command flash memory immediately goes into the XIP mode and reading the
      status register in the IRQ returns the flash memory value instead of
      register value and this will not allow interrupt to be processed properly.*/
    if ((MSS_QSPI_QUAD_FULL == g_qspi_config.io_format) ||
            (MSS_QSPI_QUAD_EX_RW == g_qspi_config.io_format) ||
            (MSS_QSPI_QUAD_EX_RO == g_qspi_config.io_format))
    {
        QSPI_TRANSFER_BLOCK(3, command_buf, 1, (uint8_t*)&temp, 4, 10);
    }
    else
    {
        QSPI_TRANSFER_BLOCK(3, command_buf, 1, (uint8_t*)&temp, 4, 8);
    }

    MSS_QSPI_get_config(&beforexip_qspi_config);

    /*Force the XIP to work correctly, we must use QSPI_SAMPLE_NEGAGE_SPICLK per spec*/
    beforexip_qspi_config.sample = MSS_QSPI_SAMPLE_NEGAGE_SPICLK;
    beforexip_qspi_config.xip = MSS_QSPI_ENABLE;

    MSS_QSPI_configure(&beforexip_qspi_config);
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_read_status_regs
(
    uint8_t* buf
)
{
    read_statusreg(&buf[0]);
    /// read_nv_cfgreg(&buf[1]); /* 2bytes */
    /// read_v_cfgreg(&buf[3]);
    /// read_enh_v_cfgreg(&buf[4]);
    /// read_flagstatusreg(&buf[5]);
}

/***************************************************************************//**
 * See infineon_s25fl.h for details of how to use this function.
 */
void
Flash_exit_xip
(
    void
)
{
    uint8_t command_buf[5] __attribute__ ((aligned (4))) = {INFINEON_FAST_READ};
    uint32_t temp = 0u;

    beforexip_qspi_config.sample = MSS_QSPI_SAMPLE_POSAGE_SPICLK;
    beforexip_qspi_config.xip = MSS_QSPI_DISABLE;
    MSS_QSPI_configure(&beforexip_qspi_config);

    /* Drive XIP confirmation bit using FAST read and keeping DQ0 to 1 during
     * idle cycle this will exit the XIP*/

    command_buf[0] = INFINEON_FAST_READ;
    command_buf[1] = 0x00u;
    command_buf[2] = 0x00u;
    command_buf[3] = 0xFFu;

    QSPI_TRANSFER_BLOCK(3, command_buf, 0, (uint8_t*)&temp, 1, 8);

    enable_4byte_addressing();
}

void
Flash_clr_flagstatusreg
(
    void
)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_CLR_FLAG_STATUS_REG};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    QSPI_TRANSFER_BLOCK(0, command_buf, 0, 0, 0,0);
    update_io_format(t_io_format);
}


/*******************************************************************************
 * Local functions
 */
static void
read_statusreg
(
    uint8_t* rd_buf
)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_READ_STATUS_REG};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    QSPI_TRANSFER_BLOCK(0, command_buf, 0, rd_buf, 1,0);
    update_io_format(t_io_format);
}


static void
enable_4byte_addressing
(
    void
)
{
    uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_WRITE_ENABLE};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    /* Write enable command must be executed before writing extended addr reg. */
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);

    command_buf[0] =  INFINEON_ENTER_4BYTE_ADDR_MODE;

    /* This command works for all modes. No Dummy cycles */
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);
    update_io_format(t_io_format);
}

static void
disable_4byte_addressing
(
    void
)
{
    uint8_t command_buf[2] __attribute__ ((aligned (4))) = {INFINEON_WRITE_ENABLE};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    /* Write enable command must be executed before writing extended addr reg. */
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);

    command_buf[0] =  INFINEON_EXIT_4BYTE_ADDR_MODE;

    /* This command works for all modes. No Dummy cycles */
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);
    update_io_format(t_io_format);

}

static void
device_reset(void)
{
    uint8_t command_buf[2] __attribute__ ((aligned (4))) = {INFINEON_RESET_ENABLE};

    QSPI_TRANSFER_BLOCK(0, (void *)command_buf, 0, 0, 0,0);
    for(volatile uint32_t idx =0u; idx < 1000u ; idx++);    /* delay */

    command_buf[1] = INFINEON_RESET_MEMORY;
    QSPI_TRANSFER_BLOCK(0, (void *)command_buf, 0, 0, 0,0);
    for(volatile uint32_t idx =0u; idx < 1000u ; idx++);    /* delay */
}

static void
read_enh_v_cfgreg
(
    uint8_t* rd_buf
)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_READ_ENH_V_CONFIG_REG};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    /*This command works for all modes. No Dummy cycles*/
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, rd_buf, 1,0);

    update_io_format(t_io_format);
}

static void
write_enh_v_confreg
(
    uint8_t* enh_v_val
)
{
    uint8_t command_buf[2] __attribute__ ((aligned (4))) = {INFINEON_WRITE_ENABLE};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);
    /*execute Write enable command again for writing the data*/
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0, 0);
    command_buf[0] =  INFINEON_WR_ENH_V_CONFIG_REG;
    command_buf[1] =  *enh_v_val;

    /*This command works for all modes. No Dummy cycles*/
    QSPI_TRANSFER_BLOCK(0, command_buf, 1, (uint8_t*)0, 0, 0);
    update_io_format(t_io_format);

}

void
Flash_read_flag_status_v_regs
(
    uint8_t* rd_buf
)
{
    read_flagstatusreg(rd_buf);
}


static void
read_flagstatusreg
(
    uint8_t* rd_buf
)
{
    const uint8_t command_buf[1] __attribute__ ((aligned (4))) = {INFINEON_READ_FLAG_STATUS_REG};
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);

    /*This command works for all modes. No Dummy cycles*/
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, rd_buf, 1,0);
    update_io_format(t_io_format);
}

static
mss_qspi_io_format
update_io_format
(
        mss_qspi_io_format io_format
)
{
    volatile mss_qspi_io_format t_io_format;
    t_io_format = g_qspi_config.io_format;

    if ((t_io_format != MSS_QSPI_QUAD_FULL) && (t_io_format != MSS_QSPI_DUAL_FULL)) {
        if (t_io_format != io_format) {
            g_qspi_config.io_format = io_format;
            MSS_QSPI_configure(&g_qspi_config);
        }
    }

    return t_io_format;
}

static
mss_qspi_io_format
probe_io_format
(
    void
)
{
    volatile uint8_t device_id __attribute__ ((aligned (4))) = 0x0u;
    mss_qspi_io_format io_format = MSS_QSPI_NORMAL;

    for(uint8_t idx = 0u; idx < 8u; idx++)
    {
        g_qspi_config.io_format = (MSS_QSPI_QUAD_FULL - idx);
        MSS_QSPI_configure(&g_qspi_config);

        Flash_readid((uint8_t*)&device_id);

        if (INFINEON_JEDEC_ID == device_id)
        {
            io_format = (MSS_QSPI_QUAD_FULL - idx);
            break;
        }
    }

    return(io_format);
}

/* Any address within the page is valid.
 * If len -> PAGE_LENGTH :
 *       Bytes more than PAGE_LENGTH are ignored
 * If addr -> page start addr and len > more than remaining bytes in the page :
 *       Bytes overflowing the page boundary are ignored
 * if len < pag_size :
 *       Only len bytes are modified, rest remain unchanged.
 */
static uint8_t
program_page
(
    uint8_t* buf,
    uint32_t addr,
    uint32_t len
)
{
    uint8_t status = 0u;
    uint8_t command_buf[300] __attribute__ ((aligned (4))) = {0};
    uint32_t length = len;
    uint32_t offset = addr % PAGE_LENGTH;
    if(addr == 0)
    {

    }
    if (len > PAGE_LENGTH)
    {
        length = PAGE_LENGTH;
    }

    if (offset && ((length + offset) > PAGE_LENGTH))
    {
        length = PAGE_LENGTH - offset;
    }
    volatile mss_qspi_io_format t_io_format;

    t_io_format = update_io_format(MSS_QSPI_NORMAL);


    /*execute Write enable command again for writing the data*/
    command_buf[0] = INFINEON_WRITE_ENABLE;
    QSPI_TRANSFER_BLOCK(0, command_buf, 0, (uint8_t*)0, 0,0);
    update_io_format(t_io_format);

    command_buf[1] = (addr >> 24) & 0xFFu;
    command_buf[2] = (addr >> 16) & 0xFFu;
    command_buf[3] = (addr >> 8) & 0xFFu;
    command_buf[4] = addr & 0xFFu;

    for (uint16_t idx=0; idx< length;idx++)
    {
        command_buf[5 + idx] = *(uint8_t*)(buf+idx);
    }
    /* Dummy cycles for all program commands are 0.
     * DUAL EX_RO and DUAL EX_RW commands dont have 4 BYTE in their name but
     * they take 4 byte address when Flash is configured in 4byte mode.
     * Refer command set in the flash memory datasheet*/
    switch(g_qspi_config.io_format)
    {
       case MSS_QSPI_NORMAL     :
           command_buf[0] = INFINEON_4BYTE_PAGE_PROG;
           break;
       case MSS_QSPI_DUAL_EX_RO :
           command_buf[0] = INFINEON_DUAL_INPUT_FAST_PROG;              /* 1-1-2 */
           break;
       case MSS_QSPI_QUAD_EX_RO :
           Flash_init(MSS_QSPI_QUAD_EX_RW);
           command_buf[0] = 0x38;        /* 1-1-4 */
           break;
       case MSS_QSPI_DUAL_EX_RW :
           command_buf[0] = INFINEON_EXT_DUAL_INPUT_FAST_PROG;          /* 1-2-2 */
           break;
       case MSS_QSPI_QUAD_EX_RW :
           command_buf[0] = INFINEON_4BYTE_QUAD_INPUT_EXT_FAST_PROG;    /* 1-4-4 */
           break;
       case MSS_QSPI_DUAL_FULL  :
           command_buf[0] = INFINEON_4BYTE_PAGE_PROG;
           break;
       case MSS_QSPI_QUAD_FULL  :
           command_buf[0] = INFINEON_4BYTE_PAGE_PROG;
           break;
       default:
           ASSERT(0);
           break;
    }

    QSPI_TRANSFER_BLOCK(4, command_buf, length, (uint8_t*)0u, 0u, 0u);

    while (1){
        read_statusreg(&status);
        if (!(status & 0x1))        // wait until Bit [0] WIP = 0
            break;
    }

    read_flagstatusreg(&status);

    return(status & (0x1 << 5));    /// P_ERR
}

#ifdef __cplusplus
}
#endif
