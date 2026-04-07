/*******************************************************************************
 * Copyright 2019 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file pac1934.h
 * @author Microchip FPGA Embedded Systems Solutions
 * @brief The PAC1934 bare metal software driver implementation.
 *
 */

#ifndef PAC1934_H
#define PAC1934_H

/*
 * PAC1934 Driver – Usage Overview
 *
 * This driver provides a small set of functions to bring up the PAC1934,
 * configure its operating mode, trigger measurement snapshots, and read
 * raw or scaled values.  A typical application follows this sequence:
 *
 *   1) Initialise I2C in the platform HAL.
 *   2) Call pac1934_sensor_probe() to confirm the device is present.
 *   3) Configure channels, polarity and sample rate using pac1934_configure().
 *   4) Periodically call pac1934_refresh_v() to latch a new measurement
 *      snapshot without clearing accumulators.
 *   5) Read the values you need either via:
 *      - individual register reads (pac1934_read_u16, _u32_vpower, _u48_acc)
 *      - or one coherent 76‑byte window (pac1934_read_measurement_window).
 *   6) Convert raw values to engineering units using the scaling helpers:
 *      - pac1934_vbus_raw_to_mV()
 *      - pac1934_vsense_raw_to_mA()
 *      - pac1934_vpower_raw28_to_uW()
 *
 * Example (simple polling):
 *
 *      // setup I2C if needed, e.g.
 *      PLIC_init();
 *      PLIC_SetPriority(PLIC_I2C1_MAIN_INT_OFFSET, 3);
 *      PLIC_SetPriority(PLIC_I2C1_ALERT_INT_OFFSET, 3);
 *      PLIC_SetPriority(PLIC_I2C1_SUS_INT_OFFSET, 3);
 *
 *      PLIC_EnableIRQ(PLIC_I2C1_MAIN_INT_OFFSET);
 *      PLIC_EnableIRQ(PLIC_I2C1_ALERT_INT_OFFSET);
 *      PLIC_EnableIRQ(PLIC_I2C1_SUS_INT_OFFSET);
 *
 *      MSS_I2C_init(&g_mss_i2c1_lo, 0x00, MSS_I2C_PCLK_DIV_192);
 *
 *      // Now probe and poll PAC1934 sensor...
 *      if (pac1934_sensor_probe(&m_mss_i2c1_lo) == 0) {
 *          bool active[4] = {true, true, true, true};
 *          bool bipolar[4] = {false, false, false, false};
 *          pac1934_configure(active, bipolar, 1024);   // all channels, unipolar, 1024 sps
 *
 *          for (;;) {
 *              pac1934_refresh_v();                    // latch the latest sample
 *
 *              uint16_t vbus_raw;
 *              pac1934_read_u16(VBUS1_AVG_REG, &vbus_raw);
 *              int32_t mV = pac1934_vbus_raw_to_mV(vbus_raw, false);
 *
 *              uint16_t vsense_raw;
 *              pac1934_read_u16(VSENSE1_AVG_REG, &vsense_raw);
 *              int32_t mA = pac1934_vsense_raw_to_mA(vsense_raw, false, shunt_uohm);
 *
 *              // use mV and mA in your application...
 *          }
 *      }
 *
 * Example (coherent bulk read):
 *
 *      uint8_t buf[76];
 *      pac1934_refresh_v();
 *      pac1934_read_measurement_window(buf);
 *      // buf now contains ACC_COUNT, VPOWER_ACC1..4, VBUS*, VSENSE*, AVG*, etc.,
 *      // all captured in a single I2C transaction.
 *
 * The API is designed so higher‑level code can decide whether it wants
 * simple reads, bulk reads, or accumulation‑based energy calculations,
 * without exposing caller code to register formats or errata handling.
 */

#include "pac1934_regs.h"

/*------------------------------------------------------------------------------
 * Shunt configuration -- **Set these for your board**
 *
 * Per-channel shunt values in uOhms.
 * Defaults to 10 mOhm (10000 uOhm) as a safe placeholder.
 */
#ifndef PAC1934_SHUNT_UOHM_CH1
#  define PAC1934_SHUNT_UOHM_CH1     (10000UL)
#endif
#ifndef PAC1934_SHUNT_UOHM_CH2
#  define PAC1934_SHUNT_UOHM_CH2     (10000UL)
#endif
#ifndef PAC1934_SHUNT_UOHM_CH3
#  define PAC1934_SHUNT_UOHM_CH3     (10000UL)
#endif
#ifndef PAC1934_SHUNT_UOHM_CH4
#  define PAC1934_SHUNT_UOHM_CH4     (10000UL)
#endif

/*------------------------------------------------------------------------------
 * Driver API
 */

/* Device Lifecycle */
int pac1934_sensor_probe(mss_i2c_instance_t * const p_i2c_instance);

int pac1934_configure(const bool ch_active[4], const bool bi_dir[4], uint32_t sample_rate_sps);

/* Snapshot Control */
void pac1934_refresh(void);
void pac1934_refresh_v(void);
int pac1934_read_measurement_window(uint8_t *out76);

/* Raw register reads */
int pac1934_read_u8(uint8_t reg, uint8_t *v);
int pac1934_read_u16(uint8_t reg, uint16_t *v);
int pac1934_read_u32_vpower(uint8_t reg, uint32_t *v28);
int pac1934_read_u48_acc(uint8_t reg, uint64_t *v48);

/* Utility/scaling functions */
void pac1934_get_signed_flags(bool vbus_signed[4], bool vsense_signed[4]);
int32_t pac1934_vbus_raw_to_mV(uint16_t raw, bool bipolar);
int32_t pac1934_vsense_raw_to_mA(uint16_t raw, bool bipolar, uint32_t shunt_uohm);
int32_t pac1934_vpower_raw28_to_uW(int32_t raw28, bool signed_mode, uint32_t shunt_uohm);

#endif
