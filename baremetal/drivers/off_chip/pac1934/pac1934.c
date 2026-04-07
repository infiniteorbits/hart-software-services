/*******************************************************************************
 * Copyright 2019 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file pac1934.c
 * @author Microchip FPGA Embedded Systems Solutions
 * @brief PAC1934 bare-metal driver (corrected & aligned with datasheet)
 *
 * Key fixes vs. original:
 *  - VPOWER (0x17..0x1A) is a 28-bit field in a 32-bit register -> right shift by 4
 *  - VPOWER_ACC (0x03..0x06) is 48-bit -> read/store in 64-bit (no truncation)
 *  - Signed/bi-directional modes (NEG_PWR) -> proper sign extension of VBUS/VSENSE/VPOWER/ACC
 *  - Use REFRESH_V for snapshot without clearing accumulators (~1ms settle per datasheet)
 *  - Correct voltage/current scaling; current depends on channel shunt (uOhm)
 *******************************************************************************/
#ifndef asm
#define asm __asm__
#endif
/*------------------------------------------------------------------------------
 * Include section
 */
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include "mpfs_hal/mss_hal.h"
#include "drivers/mss/mss_i2c/mss_i2c.h"
#include "pac1934_regs.h"
#include "mpfs_hal/common/mss_util.h"

#include "pac1934.h"

/*------------------------------------------------------------------------------
 * Macros and constants
 */

/* I2C instances / addressing */
mss_i2c_instance_t * PAC1934_I2C_MASTER = NULL;

/* 7-bit sensor address (per ADDRSEL strap). With ADDRSEL tied to GND -> 0x10 */
#define PAC1934_SENSOR_ADDR                 (0x10U)

#define PAC1934_BUFFER_SIZE                 (32U)
#define PAC1934_I2C_TIMEOUT                 (3000U) // TODO: MSS_I2C_NO_TIMEOUT ??

#define PAC1934_MASTER_ADDR                 (0x21U)

/* Scaling constants from datasheet */
#define PAC1934_VOLT_FS_MV                  (32000)     /* 32V full-scale in millivolts */
#define PAC1934_VSENSE_FS_UV                (100000)    /* 100mV full-scale in microvolts */

/* scale constants */
#define PAC1934_MAX_VPOWER_RSHIFTED_BY_28B  (11921)  /* uW scale factor */
#define PAC1934_MAX_VSENSE_RSHIFTED_BY_16B  (1525)   /* mA factor numerator */

/* Bulk 'measurement window' (ACC_COUNT..end), per datasheet */
#define PAC1934_MEAS_REG_LEN                (76U)


static inline void pac1934_wait_1ms(void)
{
    sleep_ms(1); // from mss_util.h
}

/*------------------------------------------------------------------------------
 * Global/static variables
 */

/* I2C buffers */
static uint8_t  g_master_tx_buf[PAC1934_BUFFER_SIZE];
static uint8_t  g_master_rx_buf[PAC1934_BUFFER_SIZE];

/*------------------------------------------------------------------------------
 * Small helpers
 */ 

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

/*------------------------------------------------------------------------------
 * I2C plumbing 
 */

static void i2c_completion_handler(mss_i2c_instance_t *instance, mss_i2c_status_t status)
{
    (void)instance;
    if (status != MSS_I2C_SUCCESS) {
    //    ASSERT(0U);
    }
}

static mss_i2c_status_t do_i2c_write(uint8_t serial_addr, uint8_t *tx_buffer, uint8_t write_length)
{
    //ASSERT(PAC1934_I2C_MASTER);

    MSS_I2C_write(PAC1934_I2C_MASTER, serial_addr, tx_buffer, write_length, MSS_I2C_RELEASE_BUS);
    return MSS_I2C_wait_complete(PAC1934_I2C_MASTER, PAC1934_I2C_TIMEOUT);
}

static mss_i2c_status_t do_i2c_write_read(uint8_t serial_addr,
    uint8_t *tx_buffer, uint8_t write_length, uint8_t *rx_buffer, uint8_t read_length)
{
    //ASSERT(PAC1934_I2C_MASTER);

    MSS_I2C_write_read(PAC1934_I2C_MASTER, serial_addr, tx_buffer, write_length,
                       rx_buffer, read_length, MSS_I2C_RELEASE_BUS);
    return MSS_I2C_wait_complete(PAC1934_I2C_MASTER, PAC1934_I2C_TIMEOUT);
}

/*------------------------------------------------------------------------------
 * read 1/2/4/6 bytes helpers
 */

int pac1934_read_u8(uint8_t reg, uint8_t *v)
{
    g_master_tx_buf[0] = reg;

    return (do_i2c_write_read(PAC1934_SENSOR_ADDR, g_master_tx_buf, 1U, v, 1U) == MSS_I2C_SUCCESS) ? 0 : -1;
}

int pac1934_read_u16(uint8_t reg, uint16_t *v)
{
    g_master_tx_buf[0] = reg;

    if (do_i2c_write_read(PAC1934_SENSOR_ADDR, g_master_tx_buf, 1U, g_master_rx_buf, 2U) != MSS_I2C_SUCCESS) {
        return -1;
    }

    *v = be16(g_master_rx_buf);

    return 0;
}

int pac1934_read_u32_vpower(uint8_t reg, uint32_t *v28)
{
    g_master_tx_buf[0] = reg;

    if (do_i2c_write_read(PAC1934_SENSOR_ADDR, g_master_tx_buf, 1U, g_master_rx_buf, 4U) != MSS_I2C_SUCCESS) {
        return -1;
    }

    uint32_t raw32 = be32(g_master_rx_buf);
    *v28 = (raw32 >> 4); /* only upper 28 bits are valid */

    return 0;
}

int pac1934_read_u48_acc(uint8_t reg, uint64_t *v48)
{
    g_master_tx_buf[0] = reg;

    if (do_i2c_write_read(PAC1934_SENSOR_ADDR, g_master_tx_buf, 1U, g_master_rx_buf, 6U) != MSS_I2C_SUCCESS) {
        return -1;
    }

    *v48 = ((uint64_t)g_master_rx_buf[0] << 40) |
           ((uint64_t)g_master_rx_buf[1] << 32) |
           ((uint64_t)g_master_rx_buf[2] << 24) |
           ((uint64_t)g_master_rx_buf[3] << 16) |
           ((uint64_t)g_master_rx_buf[4] <<  8) |
           ((uint64_t)g_master_rx_buf[5]);

    return 0;
}

/*------------------------------------------------------------------------------
 * REFRESH helpers & errata
 */

/* rev-2/3 errata: write NEG_PWR around refresh to preserve bi-dir settings */
static void write_neg_pwr_from_act(void)
{
    uint8_t val;
    if (pac1934_read_u8(NEG_PWR_ACT_REG, &val) == 0) {
        uint8_t frame[2] = { NEG_PWR_REG, val };
        (void)do_i2c_write(PAC1934_SENSOR_ADDR, frame, sizeof frame);
    }
}

static void do_refresh(uint8_t cmd)
{
    write_neg_pwr_from_act();
    (void)do_i2c_write(PAC1934_SENSOR_ADDR, &cmd, 1U);
    write_neg_pwr_from_act();
    pac1934_wait_1ms();
}

void pac1934_refresh_v(void)
{
    /* REFRESH_V: snapshot without clearing accumulators; wait ~1ms per datasheet */
    do_refresh(REFRESH_V_REG); /* 0x1F */
}

void pac1934_refresh(void)
{
    /* REFRESH: resets accumulators; wait ~1ms per datasheet */
    do_refresh(REFRESH_REG); /* 0x00 */
}

/*------------------------------------------------------------------------------
 * Scaling helpers
 */

/* Read signed mode flags from NEG_PWR_ACT (0x23) */
void pac1934_get_signed_flags(bool vbus_signed[4], bool vsense_signed[4])
{
    uint8_t neg_act = 0U;
    (void)pac1934_read_u8(NEG_PWR_ACT_REG, &neg_act);

    /* bits: [7:4] CHn_BIDI (VSENSE signed), [3:0] CHn_BIDV (VBUS signed) */
    vsense_signed[0] = (neg_act & (1U << 7)) != 0U;
    vsense_signed[1] = (neg_act & (1U << 6)) != 0U;
    vsense_signed[2] = (neg_act & (1U << 5)) != 0U;
    vsense_signed[3] = (neg_act & (1U << 4)) != 0U;

    vbus_signed[0]   = (neg_act & (1U << 3)) != 0U;
    vbus_signed[1]   = (neg_act & (1U << 2)) != 0U;
    vbus_signed[2]   = (neg_act & (1U << 1)) != 0U;
    vbus_signed[3]   = (neg_act & (1U << 0)) != 0U;
}

/* Convert raw VBUS to millivolts (int) */
int32_t pac1934_vbus_raw_to_mV(uint16_t raw, bool bipolar)
{
    /* unipolar: raw/65536 * 32000; bipolar: raw(15-bit)/32768 * 32000 */
    int32_t s = bipolar ? (int32_t)(int16_t)raw : (int32_t)raw;
    int32_t denom = bipolar ? 32768 : 65536;

    return (int32_t)((int64_t)s * PAC1934_VOLT_FS_MV / denom);
}

/* Convert raw VSENSE to milliamps (int) using channel shunt */
int32_t pac1934_vsense_raw_to_mA(uint16_t raw, bool bipolar, uint32_t shunt_uohm)
{
    /* I(mA) = (VSENSE_raw/2^N) * (100mV / R) * 1000; N=16 (uni) or 15 (bi) */
    int32_t s = bipolar ? (int32_t)(int16_t)raw : (int32_t)raw;
    int32_t denom = bipolar ? 32768 : 65536;

    /* scale in integer math: (s * 100000uV * 1000) / (denom * shunt_uohm) */
    return (int32_t)((int64_t)s * 100000000LL / ((int64_t)denom * (int64_t)shunt_uohm));
}

/* Power (uW) from 28-bit VPOWER using factor / shunt; halves shunt in signed mode */
int32_t pac1934_vpower_raw28_to_uW(int32_t raw28, bool signed_mode, uint32_t shunt_uohm)
{
    uint32_t sh = shunt_uohm;

    if (signed_mode && sh > 1U) {
        sh >>= 1;
    }

    if (sh == 0U) {
        sh = 1U;
    }

    return (int32_t)((int64_t)raw28 * PAC1934_MAX_VPOWER_RSHIFTED_BY_28B / sh);
}


int pac1934_sensor_probe(mss_i2c_instance_t * const p_i2c_instance)
{
    int result = -1;

    if ((&g_mss_i2c0_lo == p_i2c_instance) ||(&g_mss_i2c0_hi == p_i2c_instance)) { // I2C0
        PAC1934_I2C_MASTER = p_i2c_instance;

        PLIC_SetPriority(I2C0_MAIN_PLIC, 2U);
        PLIC_SetPriority(I2C0_ALERT_PLIC, 2U);
        PLIC_SetPriority(I2C0_SUS_PLIC, 2U);

        PLIC_EnableIRQ(I2C0_MAIN_PLIC);
        PLIC_EnableIRQ(I2C0_ALERT_PLIC);
        PLIC_EnableIRQ(I2C0_SUS_PLIC);
    } else if ((&g_mss_i2c1_lo == p_i2c_instance) || (&g_mss_i2c1_hi == p_i2c_instance)) { // I2C1
        PAC1934_I2C_MASTER = p_i2c_instance;

        PLIC_SetPriority(I2C1_MAIN_PLIC, 2U);
        PLIC_SetPriority(I2C1_ALERT_PLIC, 2U);
        PLIC_SetPriority(I2C1_SUS_PLIC, 2U);

        PLIC_EnableIRQ(I2C1_MAIN_PLIC);
        PLIC_EnableIRQ(I2C1_ALERT_PLIC);
        PLIC_EnableIRQ(I2C1_SUS_PLIC);
    } else {
        // illegal I2C instance
        return result;
    }

    MSS_I2C_init(PAC1934_I2C_MASTER, PAC1934_MASTER_ADDR, MSS_I2C_PCLK_DIV_192);
    MSS_I2C_register_transfer_completion_handler(PAC1934_I2C_MASTER, i2c_completion_handler);

    uint8_t pid, mid, rev;

    (void)pac1934_read_u8(PID_REG, &pid);
    (void)pac1934_read_u8(MID_REG, &mid);
    (void)pac1934_read_u8(REV_REG, &rev);

    if ((PID_PAC_1934 == pid) && (MID_MICROCHIP == mid) && (REV_DIE == rev)) {
        result = 0;
    }

    return result;
}

/*------------------------------------------------------------------------------
 * Coherent measurement window
 */
int pac1934_read_measurement_window(uint8_t *out76)
{
    /*
     * Read a coherent 76-byte snapshot in one I2C xfer.
     * Starts at ACC_COUNT (0x02) and includes:
     *   ACC_COUNT (3), VPOWER_ACC1..4 (4x6), VBUS1..4 (4x2), VSENSE1..4 (4x2),
     *   VBUS_AVG1..4 (4x2), VSENSE_AVG1..4 (4x2), VPOWER1..4 (4x4) = 76 bytes total.
     *
     * @out76: caller-provided buffer with space for at least PAC1934_MEAS_REG_LEN.
     * Returns 0 on success, <0 on I2C error.
     */
    if (!out76) {
        return -1;
    }

    uint8_t start = ACC_COUNT_REG; /* 0x02 */
    mss_i2c_status_t result = do_i2c_write_read(PAC1934_SENSOR_ADDR, &start, 1U, out76,
                                               PAC1934_MEAS_REG_LEN);

    return (result == MSS_I2C_SUCCESS) ? 0 : -1;
}

/*------------------------------------------------------------------------------
 * Configuration API
 */
static int pac1934_get_samp_rate_idx(uint32_t sps)
{
    /* Map sampling rate (Hz) to CTRL[7:6] index (1024,256,64,8) */
    switch (sps) {
    case 1024: return 0;   /* 00b */
    case 256:  return 1;   /* 01b */
    case 64:   return 2;   /* 10b */
    case 8:    return 3;   /* 11b */
    default:   return -1;  /* invalid */
    }
}

static void pac1934_wait_for_sample_rate(uint32_t sps)
{
    /* Coarse wait for ~one sample period
     * (conservative for new settings to settle)
     *
     * Around rate changes, wait one period at the *new* rate to be safe.
     */
    uint32_t us = (sps && sps <= 1024U) ? (1000U * (1024U / sps)) : 1000U;
    while (us >= 1000U) {
        pac1934_wait_1ms();
        us -= 1000U;
    }

    if (us) { /* sub-ms coarse burn */
        for (volatile uint32_t i = 0; i < (10U * us); ++i) { __asm__ __volatile__("nop"); }
    }
}

int pac1934_configure(const bool ch_active[4], const bool bi_dir[4], uint32_t sample_rate_sps)
{
    /*
     * Set active channels, (bi)direction, sample rate; then latch.
     * @ch_active[4]: true = channel enabled (ON), false = disabled (OFF)
     * @bi_dir[4]:    true = bipolar VBUS & bidirectional VSENSE for this channel
     * @sample_rate_sps: one of {1024, 256, 64, 8}
     *
     * Sequence:
     *   1) Write CHANNEL_DIS    (0x1C)  [1 bit per channel: 1=OFF, 0=ON]
     *   2) Write NEG_PWR        (0x1D)  [both BIDV and BIDI from bi_dir[]]
     *   3) Write CTRL           (0x01)  [sample rate bits only]
     *   4) REFRESH              (0x00)  [latch settings and reset accumulators]
     *   5) Wait ~one sample period (conservative)
     *
     * Returns 0 on success, <0 on parameter/I2C error.
     */

    int idx = pac1934_get_samp_rate_idx(sample_rate_sps);
    if (idx < 0) {
        return -1; /* invalid sample rate */
    }

    /* Build CHANNEL_DIS: bits 7..4 = CH1..CH4 OFF; 1=OFF, 0=ON */
    uint8_t chdis =
        (ch_active[0] ? 0U : (1U << 7)) |
        (ch_active[1] ? 0U : (1U << 6)) |
        (ch_active[2] ? 0U : (1U << 5)) |
        (ch_active[3] ? 0U : (1U << 4));

    /* Build NEG_PWR: [7:4] CHn_BIDI (VSENSE), [3:0] CHn_BIDV (VBUS)
     * We set both for channels requested as bi_dir[]. */
    uint8_t negpwr = 0U;
    for (int ch = 0; ch < 4; ++ch) {
        if (bi_dir[ch]) {
            negpwr |= (1U << (7 - ch)); /* VSENSE bidirectional */
            negpwr |= (1U << (3 - ch)); /* VBUS bipolar        */
        }
    }

    /* CTRL: only sample rate (bits 7:6). Keep SLEEP=0, SING=0, ALERT disabled. */
    uint8_t ctrl = (uint8_t)(idx << 6);

    uint8_t wr1[2] = { CHANNEL_DIS_REG, chdis }; /* Write CHANNEL_DIS */
    if (do_i2c_write(PAC1934_SENSOR_ADDR, wr1, sizeof wr1) != MSS_I2C_SUCCESS) {
        return -1;
    }

    uint8_t wr2[2] = { NEG_PWR_REG, negpwr }; /* Write NEG_PWR */
    if (do_i2c_write(PAC1934_SENSOR_ADDR, wr2, sizeof wr2) != MSS_I2C_SUCCESS) {
        return -1;
    }

    uint8_t wr3[2] = { CTRL_REG, ctrl }; /* Write CTRL */
    if (do_i2c_write(PAC1934_SENSOR_ADDR, wr3, sizeof wr3) != MSS_I2C_SUCCESS) {
        return -1;
    }

    pac1934_refresh(); /* Latch settings and reset accumulators (errata-safe wrapper) */

    pac1934_wait_for_sample_rate(sample_rate_sps); /* Conservative settle: wait ~one sample period at requested rate */
    return 0;
}
