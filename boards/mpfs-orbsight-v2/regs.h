/**-----------------------------------------------------------------------------
 * @file        BSP_Regs.h
 * @brief       Register map definitions for the RFIM Camera System BSP
 *              4-byte aligned register map with field offsets and masks.
 *
 * Architecture:
 * - _OFFSET / _REG_OFFSET: Absolute address from BAR0.
 * - [BITFIELD]_OFFSET: Points to the parent register's absolute address.
 * - [BITFIELD]_SHIFT: Bit position within the register.
 * - [BITFIELD]_MASK: Pre-shifted bitmask.
 *
 * @author      Trajce Nikolov | nick@rfim.co.uk
 * @date        March  2026
 *
 * @version     1.1.1           Added descriptions and default values in
 *                              comments
 * @version     1.1.0           Complete file restructure
 *                              4 byte alignment register address
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
*/

#ifndef BSP_REGS_H_
#define BSP_REGS_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint>
#else
    #include <stdint.h>
#endif


/// --- 2.1. System Identification & Control (0x0000 - 0x003C) ----------------

#define REGMAP_VERSION_OFFSET                   0x0000
#define REGMAP_VERSION_REG_OFFSET               0x0000
    #define REG_MAP_VER_OFFSET                  0x0000  /// Register map versioning (Reset: 0x01000000)
    #define REG_MAP_VER_SHIFT                   0
    #define REG_MAP_VER_MASK                    0xFFFFFFFF

#define HARDWARE_ID_OFFSET                      0x0004
#define HARDWARE_ID_REG_OFFSET                  0x0004
    #define HW_ID_VAL_OFFSET                    0x0004  /// Unique Camera Hardware ID (Reset: 0x4F524232)
    #define HW_ID_VAL_SHIFT                     0
    #define HW_ID_VAL_MASK                      0xFFFFFFFF

#define FW_VERSION_OFFSET                       0x0008
#define FW_VERSION_REG_OFFSET                   0x0008
    #define FW_VER_VAL_OFFSET                   0x0008  /// Firmware/Gateware Version (Reset: 0x01010400)
    #define FW_VER_VAL_SHIFT                    0
    #define FW_VER_VAL_MASK                     0xFFFFFFFF

#define BUILD_ID_OFFSET                         0x000C
#define BUILD_ID_REG_OFFSET                     0x000C
    #define BUILD_CRC_OFFSET                    0x000C  /// Build Hash/CRC for traceability (Reset: 0x00000000)
    #define BUILD_CRC_SHIFT                     0
    #define BUILD_CRC_MASK                      0xFFFFFFFF

#define SYSTEM_CTRL_OFFSET                      0x0010
#define SYSTEM_CTRL_REG_OFFSET                  0x0010
    #define SYS_ENABLE_OFFSET                   0x0010  /// 1: Enable system logic, 0: Standby (Reset: 0)
    #define SYS_ENABLE_SHIFT                    0
    #define SYS_ENABLE_MASK                     (0x1 << 0)
    #define SYS_SW_RESET_OFFSET                 0x0010  /// 1: Trigger global soft reset (Reset: 0)
    #define SYS_SW_RESET_SHIFT                  1
    #define SYS_SW_RESET_MASK                   (0x1 << 1)
    #define SYS_AXI_EN_OFFSET                   0x0010  /// 1: Enable AXI data master (Reset: 0)
    #define SYS_AXI_EN_SHIFT                    2
    #define SYS_AXI_EN_MASK                     (0x1 << 2)
    #define SYS_TPG_EN_OFFSET                   0x0010  /// 1: Enable Test Pattern Generator (Reset: 0)
    #define SYS_TPG_EN_SHIFT                    3
    #define SYS_TPG_EN_MASK                     (0x1 << 3)
    #define SYS_MODE_SEL_OFFSET                 0x0010  /// 0:Idle, 1:Acq, 2:Playback, 3:Calib (Reset: 0)
    #define SYS_MODE_SEL_SHIFT                  4
    #define SYS_MODE_SEL_MASK                   (0xF << 4)
    #define SYS_CLR_ERR_OFFSET                  0x0010  /// Write 1 to clear latched system errors (Reset: 0)
    #define SYS_CLR_ERR_SHIFT                   8
    #define SYS_CLR_ERR_MASK                    (0x1 << 8)
    #define SYS_WATCHDOG_EN_OFFSET              0x0010  /// 1: Enable internal HW Watchdog (Reset: 1)
    #define SYS_WATCHDOG_EN_SHIFT               12
    #define SYS_WATCHDOG_EN_MASK                (0x1 << 12)

#define SYSTEM_STATUS_OFFSET                    0x0014
#define SYSTEM_STATUS_REG_OFFSET                0x0014
    #define SYS_STAT_OK_OFFSET                  0x0014  /// 1: System healthy/ready (Reset: 1)
    #define SYS_STAT_OK_SHIFT                   0
    #define SYS_STAT_OK_MASK                    (0x1 << 0)
    #define SYS_STAT_DEGRADED_OFFSET            0x0014  /// 1: Degraded mode (temp/volt) (Reset: 0)
    #define SYS_STAT_DEGRADED_SHIFT              1
    #define SYS_STAT_DEGRADED_MASK               (0x1 << 1)
    #define SYS_STAT_FAULT_OFFSET               0x0014  /// 1: Critical HW failure (Reset: 0)
    #define SYS_STAT_FAULT_SHIFT                2
    #define SYS_STAT_FAULT_MASK                 (0x1 << 2)
    #define SYS_LAST_RST_CAUSE_OFFSET           0x0014  /// 0:POR, 1:WDOG, 2:SW, 3:EXT (Reset: 0)
    #define SYS_LAST_RST_CAUSE_SHIFT            4
    #define SYS_LAST_RST_CAUSE_MASK             (0xF << 4)
    #define SYS_PLL_LOCK_OFFSET                 0x0014  /// 1: Internal Clock PLLs Locked (Reset: 0)
    #define SYS_PLL_LOCK_SHIFT                  8
    #define SYS_PLL_LOCK_MASK                   (0x1 << 8)

#define SYSTEM_SCRATCH_OFFSET                   0x003C
#define SYSTEM_SCRATCH_REG_OFFSET               0x003C
    #define SYS_SCRATCH_VAL_OFFSET              0x003C  /// R/W test register for bus validation (Reset: 0xDEADBEEF)
    #define SYS_SCRATCH_VAL_SHIFT               0
    #define SYS_SCRATCH_VAL_MASK                0xFFFFFFFF

/// --- 2.2. Power & Low-Level Control (0x0040 - 0x0044) ----------------------

#define PWREN_CTRL_OFFSET                       0x0040
#define PWREN_CTRL_REG_OFFSET                   0x0040
    #define PWR_3V5_EN_OFFSET                   0x0040  /// Enable 3.5V Analog rail (Reset: 0)
    #define PWR_3V5_EN_SHIFT                    0
    #define PWR_3V5_EN_MASK                     (0x1 << 0)
    #define PWR_2V0_EN_OFFSET                   0x0040  /// Enable 2.0V Analog rail (Reset: 0)
    #define PWR_2V0_EN_SHIFT                    1
    #define PWR_2V0_EN_MASK                     (0x1 << 1)
    #define PWR_3V8_EN_OFFSET                   0x0040  /// Enable 3.8V Digital rail (Reset: 0)
    #define PWR_3V8_EN_SHIFT                    2
    #define PWR_3V8_EN_MASK                     (0x1 << 2)
    #define PWR_0V8_EN_OFFSET                   0x0040  /// Enable 0.8V Core rail (Reset: 0)
    #define PWR_0V8_EN_SHIFT                    3
    #define PWR_0V8_EN_MASK                     (0x1 << 3)
    #define PWR_CAN_PRI_EN_OFFSET               0x0040  /// Enable Primary CAN Transceiver (Reset: 0)
    #define PWR_CAN_PRI_EN_SHIFT                4
    #define PWR_CAN_PRI_EN_MASK                 (0x1 << 4)
    #define PWR_CAN_SEC_EN_OFFSET               0x0040  /// Enable Secondary CAN Transceiver (Reset: 0)
    #define PWR_CAN_SEC_EN_SHIFT                5
    #define PWR_CAN_SEC_EN_MASK                 (0x1 << 5)
    #define PWR_NAND_EN_OFFSET                  0x0040  /// Power to NAND Flash chips (Reset: 0)
    #define PWR_NAND_EN_SHIFT                   6
    #define PWR_NAND_EN_MASK                    (0x1 << 6)
    #define PWR_EMMC_EN_OFFSET                  0x0040  /// Power to eMMC module (Reset: 0)
    #define PWR_EMMC_EN_SHIFT                   7
    #define PWR_EMMC_EN_MASK                    (0x1 << 7)
    #define PWR_VCC_IO_EN_OFFSET                0x0040  /// Enable I/O Bank Voltage (Reset: 0)
    #define PWR_VCC_IO_EN_SHIFT                 8
    #define PWR_VCC_IO_EN_MASK                  (0x1 << 8)
    #define PWR_LVDS_EN_OFFSET                  0x0040  /// Enable LVDS Interface Power (Reset: 0)
    #define PWR_LVDS_EN_SHIFT                   9
    #define PWR_LVDS_EN_MASK                    (0x1 << 9)
    #define PWR_RS422_EN_OFFSET                 0x0040  /// Enable RS422 Transceiver (Reset: 0)
    #define PWR_RS422_EN_SHIFT                  10
    #define PWR_RS422_EN_MASK                   (0x1 << 10)
    #define PWR_ETH_RST_N_OFFSET                0x0040  /// Ethernet PHY Hardware Reset (Active Low) (Reset: 1)
    #define PWR_ETH_RST_N_SHIFT                 11
    #define PWR_ETH_RST_N_MASK                  (0x1 << 11)
    #define PWR_MON_EN_OFFSET                   0x0040  /// Enable Housekeeping ADC/Monitoring (Reset: 1)
    #define PWR_MON_EN_SHIFT                    13
    #define PWR_MON_EN_MASK                     (0x1 << 13)

#define ETH_SOFT_CTRL_OFFSET                    0x0044
#define ETH_SOFT_CTRL_REG_OFFSET                0x0044
    #define ETH_EN_OFFSET                       0x0044  /// Enable RGMII Logic (Reset: 0)
    #define ETH_EN_SHIFT                        0
    #define ETH_EN_MASK                         (0x1 << 0)
    #define ETH_SOFT_RST_OFFSET                 0x0044  /// Software Reset for Ethernet Core (Reset: 0)
    #define ETH_SOFT_RST_SHIFT                  2
    #define ETH_SOFT_RST_MASK                   (0x1 << 2)
    #define ETH_SPEED_OFFSET                    0x0044  /// 0:10M, 1:100M, 2:1G (Reset: 2)
    #define ETH_SPEED_SHIFT                     4
    #define ETH_SPEED_MASK                      (0x3 << 4)
    #define ETH_DUPLEX_OFFSET                   0x0044  /// 0:Full, 1:Half (Reset: 0)
    #define ETH_DUPLEX_SHIFT                    8
    #define ETH_DUPLEX_MASK                     (0x3 << 8)
    #define ETH_LED_EN_OFFSET                   0x0044  /// Enable Link/Act Activity LEDs (Reset: 1)
    #define ETH_LED_EN_SHIFT                    12
    #define ETH_LED_EN_MASK                     (0x1 << 12)

/// --- 2.3. Image Acquisition Parameters (0x0080 - 0x009C) ------------------

#define ACQ_EXPOSURE_OFFSET                     0x0080
#define ACQ_EXPOSURE_REG_OFFSET                 0x0080
    #define ACQ_EXP_VAL_OFFSET                  0x0080  /// Exposure time (1 LSB = 14us) (Reset: 30)
    #define ACQ_EXP_VAL_SHIFT                   0
    #define ACQ_EXP_VAL_MASK                    0xFFFFFFFF

#define ACQ_FRAME_OFFSET                        0x0084
#define ACQ_FRAME_REG_OFFSET                    0x0084
    #define ACQ_FPS_OFFSET                      0x0084  /// Frame rate select 1-15 FPS (Reset: 5)
    #define ACQ_FPS_SHIFT                       0
    #define ACQ_FPS_MASK                        (0xFF << 0)
    #define ACQ_TEST_PAT_EN_OFFSET              0x0084  /// 1: Enable internal test pattern (Reset: 0)
    #define ACQ_TEST_PAT_EN_SHIFT               8
    #define ACQ_TEST_PAT_EN_MASK                (0x1 << 8)
    #define ACQ_COMPR_EN_OFFSET                 0x0084  /// 1: Enable hardware compression (Reset: 0)
    #define ACQ_COMPR_EN_SHIFT                  9
    #define ACQ_COMPR_EN_MASK                   (0x1 << 9)
    #define ACQ_RES_MODE_OFFSET                 0x0084  /// 0:2k, 1:1k, 2:512 (Reset: 0)
    #define ACQ_RES_MODE_SHIFT                  10
    #define ACQ_RES_MODE_MASK                   (0x3 << 10)
    #define ACQ_BURST_LEN_OFFSET                0x0084  /// Number of frames per trigger (Reset: 1)
    #define ACQ_BURST_LEN_SHIFT                 16
    #define ACQ_BURST_LEN_MASK                  (0x7FFF << 16)

#define ACQ_HDR_COEFF_OFFSET                    0x0088
#define ACQ_HDR_COEFF_REG_OFFSET                0x0088
    #define HDR_COEFF_A_OFFSET                  0x0088  /// Linear Coefficient A (Reset: 224)
    #define HDR_COEFF_A_SHIFT                   0
    #define HDR_COEFF_A_MASK                    (0xFFFF << 0)
    #define HDR_COEFF_B_OFFSET                  0x0088  /// Linear Coefficient B (Reset: 400)
    #define HDR_COEFF_B_SHIFT                   16
    #define HDR_COEFF_B_MASK                    (0x7FFF << 16)

#define ACQ_CFG_OFFSET                          0x008C
#define ACQ_CFG_REG_OFFSET                      0x008C
    #define ACQ_HDR_MUX_OFFSET                  0x008C  /// 0:Bot, 1:Top, 2:HDR, 3:CMS (Reset: 2)
    #define ACQ_HDR_MUX_SHIFT                   0
    #define ACQ_HDR_MUX_MASK                    (0x7 << 0)
    #define ACQ_HDR_THRESH_OFFSET               0x008C  /// High-Gain Threshold (Reset: 3003)
    #define ACQ_HDR_THRESH_SHIFT                16
    #define ACQ_HDR_THRESH_MASK                 (0x7FFF << 16)

#define SENSOR_ADC_OFFSET_OFFSET                0x0090
#define SENSOR_ADC_OFFSET_REG_OFFSET            0x0090
    #define ADC_OFF_TOP_OFFSET                  0x0090  /// Sensor Top ADC Offset (Reset: 62271)
    #define ADC_OFF_TOP_SHIFT                   0
    #define ADC_OFF_TOP_MASK                    (0xFFFF << 0)
    #define ADC_OFF_BOT_OFFSET                  0x0090  /// Sensor Bot ADC Offset (Reset: 62271)
    #define ADC_OFF_BOT_SHIFT                   16
    #define ADC_OFF_BOT_MASK                    (0x7FFF << 16)

#define ACQ_SENSOR_CFG_OFFSET                   0x0094
#define ACQ_SENSOR_CFG_REG_OFFSET               0x0094
    #define PGA_GAIN_TOP_OFFSET                 0x0094  /// Top PGA Gain (Reset: 2)
    #define PGA_GAIN_TOP_SHIFT                  0
    #define PGA_GAIN_TOP_MASK                   (0x3F << 0)
    #define PGA_GAIN_BOT_OFFSET                 0x0094  /// Bot PGA Gain (Reset: 38)
    #define PGA_GAIN_BOT_SHIFT                  6
    #define PGA_GAIN_BOT_MASK                   (0x3F << 6)
    #define ACQ_SCAN_DIR_OFFSET                 0x0094  /// 0:Normal, 1:Reverse (Reset: 0)
    #define ACQ_SCAN_DIR_SHIFT                  12
    #define ACQ_SCAN_DIR_MASK                   (0x1 << 12)
    #define ACQ_TRAIN_EN_OFFSET                 0x0094  /// Enable Training Mode (Reset: 0)
    #define ACQ_TRAIN_EN_SHIFT                  14
    #define ACQ_TRAIN_EN_MASK                   (0x1 << 14)
    #define ACQ_TRAIN_VAL_OFFSET                0x0094  /// Custom Training Pattern (Reset: 0x98E)
    #define ACQ_TRAIN_VAL_SHIFT                 16
    #define ACQ_TRAIN_VAL_MASK                  (0xFFF << 16)

#define ACQ_MODE_OFFSET                         0x0098
#define ACQ_MODE_REG_OFFSET                     0x0098
    #define COMPR_MODE_SEL_OFFSET               0x0098  /// 0:None, 1:JPEG, 2:RAW10, 3:RAW12 (Reset: 0)
    #define COMPR_MODE_SEL_SHIFT                0
    #define COMPR_MODE_SEL_MASK                 (0xF << 0)
    #define PAT_MODE_SEL_OFFSET                 0x0098  /// 0:Gradient, 1:Stars, 2:Checker (Reset: 0)
    #define PAT_MODE_SEL_SHIFT                  4
    #define PAT_MODE_SEL_MASK                   (0xF << 4)
    #define PROC_MODE_SEL_OFFSET                0x0098  /// Pipeline Select (Reset: 0)
    #define PROC_MODE_SEL_SHIFT                 8
    #define PROC_MODE_SEL_MASK                  (0xFF << 8)

#define ACQ_PARAM_STATUS_OFFSET                 0x009C
#define ACQ_PARAM_STATUS_REG_OFFSET               0x009C
    #define PAR_VALID_OFFSET                    0x009C  /// 1: Current params accepted (Reset: 0)
    #define PAR_VALID_SHIFT                     0
    #define PAR_VALID_MASK                      (0x1 << 0)
    #define PAR_REJECT_OFFSET                   0x009C  /// 1: Last set rejected (Reset: 0)
    #define PAR_REJECT_SHIFT                    2
    #define PAR_REJECT_MASK                     (0x1 << 2)

/// --- 2.4. Acquisition Command & Result (0x00A0 - 0x00DC) --------------------

#define ACQ_DDR_ADDR_LO_OFFSET                  0x00A0
#define ACQ_DDR_ADDR_LO_REG_OFFSET              0x00A0
    #define DDR_ADDR_LSW_OFFSET                 0x00A0  /// Lower 32-bits of target DDR destination (Reset: 0)
    #define DDR_ADDR_LSW_SHIFT                  0
    #define DDR_ADDR_LSW_MASK                   0xFFFFFFFF

#define ACQ_DDR_ADDR_HI_OFFSET                  0x00A4
#define ACQ_DDR_ADDR_HI_REG_OFFSET              0x00A4
    #define DDR_ADDR_MSW_OFFSET                 0x00A4  /// Upper 32-bits of target DDR destination (Reset: 0)
    #define DDR_ADDR_MSW_SHIFT                  0
    #define DDR_ADDR_MSW_MASK                   0xFFFFFFFF

#define ACQ_CMD_STATUS_OFFSET                   0x00A8
#define ACQ_CMD_STATUS_REG_OFFSET               0x00A8
    #define CMD_START_OFFSET                    0x00A8  /// Write 1: Trigger Acq Sequence (Self-clearing) (Reset: 0)
    #define CMD_START_SHIFT                     0
    #define CMD_START_MASK                      (0x1 << 0)
    #define CMD_ABORT_OFFSET                    0x00A8  /// Write 1: Immediate Engine Stop (Reset: 0)
    #define CMD_ABORT_SHIFT                     1
    #define CMD_ABORT_MASK                      (0x1 << 1)
    #define CMD_BUSY_OFFSET                     0x00A8  /// 1: Acquisition engine is currently running (Reset: 0)
    #define CMD_BUSY_SHIFT                      3
    #define CMD_BUSY_MASK                       (0x1 << 3)
    #define CMD_DONE_OFFSET                     0x00A8  /// 1: Sequence finished (latched until next CMD) (Reset: 0)
    #define CMD_DONE_SHIFT                      4
    #define CMD_DONE_MASK                       (0x1 << 4)
    #define CMD_ERR_CODE_OFFSET                 0x00A8  /// [15:11] 0:Success, 1:Timeout, 2:DDR_Full, 3:Sens_Err (Reset: 0)
    #define CMD_ERR_CODE_SHIFT                  11
    #define CMD_ERR_CODE_MASK                   (0x1F << 11)

#define ACQ_IMAGE_COUNT_OFFSET                  0x00CC
#define ACQ_IMAGE_COUNT_REG_OFFSET              0x00CC
    #define IMG_COUNT_VAL_OFFSET                0x00CC  /// Number of frames successfully stored in memory (Reset: 0)
    #define IMG_COUNT_VAL_SHIFT                 0
    #define IMG_COUNT_VAL_MASK                  0xFFFFFFFF

#define ACQ_REFCLK_LO_OFFSET                    0x00D8
#define ACQ_REFCLK_LO_REG_OFFSET                0x00D8
    #define REF_CLK_LSW_OFFSET                  0x00D8  /// 64-bit Timestamp LSW at end of acquisition (Reset: 0)
    #define REF_CLK_LSW_SHIFT                   0
    #define REF_CLK_LSW_MASK                    0xFFFFFFFF

#define ACQ_REFCLK_HI_OFFSET                    0x00DC
#define ACQ_REFCLK_HI_REG_OFFSET                0x00DC
    #define REF_CLK_MSW_OFFSET                  0x00DC  /// 64-bit Timestamp MSW at end of acquisition (Reset: 0)
    #define REF_CLK_MSW_SHIFT                   0
    #define REF_CLK_MSW_MASK                    0xFFFFFFFF

/// --- 2.5. Housekeeping & Monitoring (0x0100 - 0x0180) ----------------------

#define HK_ACQ_STAT_OFFSET                      0x0100
#define HK_ACQ_STAT_REG_OFFSET                  0x0100
    #define HK_SUCCESS_COUNT_OFFSET             0x0100  /// Lifetime successful acquisitions (Reset: 0)
    #define HK_SUCCESS_COUNT_SHIFT              0
    #define HK_SUCCESS_COUNT_MASK               0xFFFF
    #define HK_FAIL_COUNT_OFFSET                0x0100  /// Lifetime acquisition failures (Reset: 0)
    #define HK_FAIL_COUNT_SHIFT                 16
    #define HK_FAIL_COUNT_MASK                  0xFFFF

#define HK_FRAME_ERR_COUNT_OFFSET               0x0104
#define HK_FRAME_ERR_COUNT_REG_OFFSET           0x0104
    #define HK_CRC_ERR_COUNT_OFFSET             0x0104  /// Count of LVDS CRC errors (Reset: 0)
    #define HK_CRC_ERR_COUNT_SHIFT              0
    #define HK_CRC_ERR_COUNT_MASK               0xFFFF
    #define HK_LINE_ERR_COUNT_OFFSET            0x0104  /// Count of SOF/EOF mismatches (Reset: 0)
    #define HK_LINE_ERR_COUNT_SHIFT             16
    #define HK_LINE_ERR_COUNT_MASK              0xFFFF

#define HK_UPTIME_LO_OFFSET                     0x0114
#define HK_UPTIME_LO_REG_OFFSET                 0x0114
    #define HK_UPTIME_LSW_OFFSET                0x0114  /// Seconds since POR [31:0] (Reset: 0)
    #define HK_UPTIME_LSW_SHIFT                 0
    #define HK_UPTIME_LSW_MASK                  0xFFFFFFFF

#define HK_UPTIME_HI_OFFSET                     0x0118
#define HK_UPTIME_HI_REG_OFFSET                 0x0118
    #define HK_UPTIME_MSW_OFFSET                0x0118  /// Seconds since POR [63:32] (Reset: 0)
    #define HK_UPTIME_MSW_SHIFT                 0
    #define HK_UPTIME_MSW_MASK                  0xFFFFFFFF

#define HK_PWREN_STATUS_OFFSET                  0x011C
#define HK_PWREN_STATUS_REG_OFFSET              0x011C
    #define HK_P3V5_PG_OFFSET                   0x011C  /// 1: 3.5V Analog Rail Power Good (Reset: 0)
    #define HK_P3V5_PG_SHIFT                    0
    #define HK_P3V5_PG_MASK                     (0x1 << 0)
    #define HK_P2V0_PG_OFFSET                   0x011C  /// 1: 2.0V Analog Rail Power Good (Reset: 0)
    #define HK_P2V0_PG_SHIFT                    1
    #define HK_P2V0_PG_MASK                     (0x1 << 1)
    #define HK_PWR_FAULT_CODE_OFFSET            0x011C  /// [2:4] Latched Fault Code (Reset: 0)
    #define HK_PWR_FAULT_CODE_SHIFT             2
    #define HK_PWR_FAULT_CODE_MASK              (0x7 << 2)

#define HK_CURR_MON_3V5_OFFSET                  0x0120
#define HK_CURR_MON_3V5_REG_OFFSET              0x0120
    #define HK_3V5_CURR_MA_OFFSET               0x0120  /// 3.5V Rail Current in mA (Reset: 0)
    #define HK_3V5_CURR_MA_SHIFT                0
    #define HK_3V5_CURR_MA_MASK                 0xFFFFFFFF

#define HK_CURR_MON_2V0_OFFSET                  0x0124
#define HK_CURR_MON_2V0_REG_OFFSET              0x0124
    #define HK_2V0_CURR_MA_OFFSET               0x0124  /// 2.0V Rail Current in mA (Reset: 0)
    #define HK_2V0_CURR_MA_SHIFT                0
    #define HK_2V0_CURR_MA_MASK                 0xFFFFFFFF

#define HK_EMMC_STATUS_OFFSET                   0x014C
#define HK_EMMC_STATUS_REG_OFFSET               0x014C
    #define HK_EMMC_PRI_STAT_OFFSET             0x014C  /// Primary eMMC Interface Status (Reset: 0)
    #define HK_EMMC_PRI_STAT_SHIFT              0
    #define HK_EMMC_PRI_STAT_MASK               0xFFFF
    #define HK_EMMC_SEC_STAT_OFFSET             0x014C  /// Secondary eMMC Interface Status (Reset: 0)
    #define HK_EMMC_SEC_STAT_SHIFT              16
    #define HK_EMMC_SEC_STAT_MASK               0xFFFF

#define HK_NAND_STATUS_OFFSET                   0x0154
#define HK_NAND_STATUS_REG_OFFSET               0x0154
    #define HK_NAND_BUSY_OFFSET                 0x0154  /// 1: Flash controller busy (Reset: 0)
    #define HK_NAND_BUSY_SHIFT                  0
    #define HK_NAND_BUSY_MASK                   (0x1 << 0)
    #define HK_NAND_WP_N_OFFSET                 0x0154  /// 1: Write Protect Inactive (Reset: 1)
    #define HK_NAND_WP_N_SHIFT                  4
    #define HK_NAND_WP_N_MASK                   (0x1 << 4)
    #define HK_NAND_ECC_ERR_OFFSET              0x0154  /// 1: Uncorrectable ECC error (Reset: 0)
    #define HK_NAND_ECC_ERR_SHIFT               8
    #define HK_NAND_ECC_ERR_MASK                (0x1 << 8)

#define HK_SENSOR_TEMP_OFFSET                   0x0178
#define HK_SENSOR_TEMP_REG_OFFSET               0x0178
    #define HK_TEMP_IMG_OFFSET                  0x0178  /// Image Sensor Temp (Units: 0.01 C) (Reset: 0)
    #define HK_TEMP_IMG_SHIFT                   0
    #define HK_TEMP_IMG_MASK                    0xFFFF
    #define HK_TEMP_SoC_OFFSET                  0x0178  /// SoC Junction Temp (Units: 0.01 C) (Reset: 0)
    #define HK_TEMP_SoC_SHIFT                   16
    #define HK_TEMP_SoC_MASK                    0xFFFF

#define HK_BOARD_TEMP_OFFSET                    0x0180
#define HK_BOARD_TEMP_REG_OFFSET                0x0180
    #define HK_TEMP_EXT_OFFSET                  0x0180  /// External Board NTC Temp (Units: 0.01 C) (Reset: 0)
    #define HK_TEMP_EXT_SHIFT                   0
    #define HK_TEMP_EXT_MASK                    0xFFFF

/// --- 2.6. Storage Device Status (0x0240 - 0x0254) ---------------------------

#define DDR_ECC_CTRL_OFFSET                     0x0240
#define DDR_ECC_CTRL_REG_OFFSET                 0x0240
    #define ECC_ENABLE_BIT_OFFSET               0x0240  /// 1: Enable Inline ECC logic; 0: Bypass (Reset: 0)
    #define ECC_ENABLE_BIT_SHIFT                0
    #define ECC_ENABLE_BIT_MASK                 (0x1 << 0)
    #define ECC_SCRUB_EN_OFFSET                 0x0240  /// 1: Enable background memory scrubbing (Reset: 0)
    #define ECC_SCRUB_EN_SHIFT                  1
    #define ECC_SCRUB_EN_MASK                   (0x1 << 1)
    #define ECC_INJECT_ERR_OFFSET               0x0240  /// 1: Inject single-bit error for testing (Reset: 0)
    #define ECC_INJECT_ERR_SHIFT                8
    #define ECC_INJECT_ERR_MASK                 (0x1 << 8)

#define DDR_ECC_ERR_COUNT_OFFSET                0x0244
#define DDR_ECC_ERR_COUNT_REG_OFFSET            0x0244
    #define ECC_SINGLE_ERR_CNT_OFFSET           0x0244  /// Correctable error counter (Reset: 0)
    #define ECC_SINGLE_ERR_CNT_SHIFT            0
    #define ECC_SINGLE_ERR_CNT_MASK             0xFFFF
    #define ECC_DOUBLE_ERR_CNT_OFFSET           0x0244  /// Uncorrectable error counter (Reset: 0)
    #define ECC_DOUBLE_ERR_CNT_SHIFT            16
    #define ECC_DOUBLE_ERR_CNT_MASK             0xFFFF

#define DMA_CTRL_STATUS_OFFSET                  0x0250
#define DMA_CTRL_STATUS_REG_OFFSET               0x0250
    #define DMA_SOFT_RESET_OFFSET               0x0250  /// 1: Reset DMA engine logic (Reset: 0)
    #define DMA_SOFT_RESET_SHIFT                0
    #define DMA_SOFT_RESET_MASK                 (0x1 << 0)
    #define DMA_START_BIT_OFFSET                0x0250  /// 1: Trigger manual DMA transfer (Reset: 0)
    #define DMA_START_BIT_SHIFT                 2
    #define DMA_START_BIT_MASK                  (0x1 << 2)
    #define DMA_PRIORITY_OFFSET                 0x0250  /// 0: Low, 1: High AXI priority (Reset: 0)
    #define DMA_PRIORITY_SHIFT                  4
    #define DMA_PRIORITY_MASK                   (0x3 << 4)
    #define DMA_BUSY_BIT_OFFSET                 0x0250  /// 1: DMA transfer in progress (Reset: 0)
    #define DMA_BUSY_BIT_SHIFT                  8
    #define DMA_BUSY_BIT_MASK                   (0x1 << 8)
    #define DMA_ERR_BIT_OFFSET                  0x0250  /// 1: AXI Bus error or timeout (Reset: 0)
    #define DMA_ERR_BIT_SHIFT                   10
    #define DMA_ERR_BIT_MASK                    (0x1 << 10)

#define ACQ_BUFFER_STATE_OFFSET                 0x0254
#define ACQ_BUFFER_STATE_REG_OFFSET             0x0254
    #define BUF_VALID_BIT_OFFSET                0x0254  /// 1: Buffer contains unread image data (Reset: 0)
    #define BUF_VALID_BIT_SHIFT                 0
    #define BUF_VALID_BIT_MASK                  (0x1 << 0)
    #define BUF_FULL_BIT_OFFSET                 0x0254  /// 1: Maximum buffer capacity reached (Reset: 0)
    #define BUF_FULL_BIT_SHIFT                  1
    #define BUF_FULL_BIT_MASK                   (0x1 << 1)
    #define BUF_LOCK_BIT_OFFSET                 0x0254  /// 1: Buffer locked for host access (Reset: 0)
    #define BUF_LOCK_BIT_SHIFT                  2
    #define BUF_LOCK_BIT_MASK                   (0x1 << 2)
    #define BUF_PAGE_IDX_OFFSET                 0x0254  /// [15:8] Current active buffer page index (Reset: 0)
    #define BUF_PAGE_IDX_SHIFT                  8
    #define BUF_PAGE_IDX_MASK                   (0xFF << 8)

/// --- 2.7. Sensor Control (0x0280 - 0x0284) ---------------------------------

#define SENSOR_CTRL_OFFSET                      0x0280
#define SENSOR_CTRL_REG_OFFSET                  0x0280
    #define SNS_ENABLE_OFFSET                   0x0280  /// 1: Power up and enable sensor clocks (Reset: 0)
    #define SNS_ENABLE_SHIFT                    0
    #define SNS_ENABLE_MASK                     (0x1 << 0)
    #define SNS_SOFT_RESET_OFFSET               0x0280  /// 1: Trigger sensor-specific soft reset (Reset: 0)
    #define SNS_SOFT_RESET_SHIFT                1
    #define SNS_SOFT_RESET_MASK                 (0x1 << 1)
    #define SNS_STREAM_EN_OFFSET                0x0280  /// 1: Enable LVDS data streaming from sensor (Reset: 0)
    #define SNS_STREAM_EN_SHIFT                 2
    #define SNS_STREAM_EN_MASK                  (0x1 << 2)
    #define SNS_OP_MODE_OFFSET                  0x0280  /// [7:4] Mode: 0:Normal, 1:Subsampling, 2:ROI (Reset: 0)
    #define SNS_OP_MODE_SHIFT                   4
    #define SNS_OP_MODE_MASK                    (0xF << 4)
    #define SNS_I2C_BYPASS_OFFSET               0x0280  /// 1: Direct FPGA control of sensor I2C (Reset: 0)
    #define SNS_I2C_BYPASS_SHIFT                8
    #define SNS_I2C_BYPASS_MASK                 (0x1 << 8)
    #define SNS_TRIG_MODE_OFFSET                0x0280  /// 0: Internal, 1: External HW Trigger (Reset: 0)
    #define SNS_TRIG_MODE_SHIFT                 12
    #define SNS_TRIG_MODE_MASK                  (0x1 << 12)

#define SENSOR_STATUS_OFFSET                    0x0284
#define SENSOR_STATUS_REG_OFFSET                0x0284
    #define SNS_PRESENT_OFFSET                  0x0284  /// 1: Sensor hardware detected on bus (Reset: 0)
    #define SNS_PRESENT_SHIFT                   0
    #define SNS_PRESENT_MASK                    (0x1 << 0)
    #define SNS_INIT_DONE_OFFSET                0x0284  /// 1: Internal calibration/init sequence finished (Reset: 0)
    #define SNS_INIT_DONE_SHIFT                 1
    #define SNS_INIT_DONE_MASK                  (0x1 << 1)
    #define SNS_PLL_LOCKED_OFFSET               0x0284  /// 1: Sensor internal PLLs are stable (Reset: 0)
    #define SNS_PLL_LOCKED_SHIFT                2
    #define SNS_PLL_LOCKED_MASK                 (0x1 << 2)
    #define SNS_FIFO_EMPTY_OFFSET               0x0284  /// 1: Sensor interface FIFO is empty (Reset: 1)
    #define SNS_FIFO_EMPTY_SHIFT                4
    #define SNS_FIFO_EMPTY_MASK                 (0x1 << 4)
    #define SNS_FIFO_FULL_OFFSET                0x0284  /// 1: Sensor interface FIFO overflow (Reset: 0)
    #define SNS_FIFO_FULL_SHIFT                 5
    #define SNS_FIFO_FULL_MASK                  (0x1 << 5)
    #define SNS_ERR_BITFIELD_OFFSET             0x0284  /// [15:8] Latched sensor communication errors (Reset: 0)
    #define SNS_ERR_BITFIELD_SHIFT              8
    #define SNS_ERR_BITFIELD_MASK               (0xFF << 8)

/// --- 2.8. Interrupts & Events (0x02C0 - 0x02C8) ----------------------------

#define IRQ_ENABLE_OFFSET                       0x02C0
#define IRQ_ENABLE_REG_OFFSET                   0x02C0
    #define ACQ_DONE_IE_OFFSET                  0x02C0  /// 1: Enable IRQ on acquisition completion (Reset: 0)
    #define ACQ_DONE_IE_SHIFT                   0
    #define ACQ_DONE_IE_MASK                    (0x1 << 0)
    #define DMA_DONE_IE_OFFSET                  0x02C0  /// 1: Enable IRQ on DMA transfer completion (Reset: 0)
    #define DMA_DONE_IE_SHIFT                   1
    #define DMA_DONE_IE_MASK                    (0x1 << 1)
    #define ECC_UNCORR_IE_OFFSET                0x02C0  /// 1: Enable IRQ on uncorrectable ECC error (Reset: 0)
    #define ECC_UNCORR_IE_SHIFT                 2
    #define ECC_UNCORR_IE_MASK                  (0x1 << 2)
    #define ECC_CORR_IE_OFFSET                  0x02C0  /// 1: Enable IRQ on correctable ECC error (Reset: 0)
    #define ECC_CORR_IE_SHIFT                   3
    #define ECC_CORR_IE_MASK                    (0x1 << 3)
    #define FIFO_OVFL_IE_OFFSET                 0x02C0  /// 1: Enable IRQ on sensor FIFO overflow (Reset: 0)
    #define FIFO_OVFL_IE_SHIFT                  4
    #define FIFO_OVFL_IE_MASK                   (0x1 << 4)
    #define SYS_FAULT_IE_OFFSET                 0x02C0  /// 1: Enable IRQ on critical system fault (Reset: 0)
    #define SYS_FAULT_IE_SHIFT                  5
    #define SYS_FAULT_IE_MASK                   (0x1 << 5)

#define IRQ_STATUS_OFFSET                       0x02C4
#define IRQ_STATUS_REG_OFFSET                   0x02C4
    #define IRQ_STAT_FLAGS_OFFSET               0x02C4  /// Pending IRQ flags. Write 1 to clear. (Reset: 0)
    #define IRQ_STAT_FLAGS_SHIFT                0
    #define IRQ_STAT_FLAGS_MASK                 (0x3F << 0)

#define IRQ_LATENCY_CTRL_OFFSET                 0x02C8
#define IRQ_LATENCY_CTRL_REG_OFFSET             0x02C8
    #define IRQ_COALESCE_VAL_OFFSET             0x02C8  /// Interrupt coalescing threshold count (Reset: 1)
    #define IRQ_COALESCE_VAL_SHIFT              0
    #define IRQ_COALESCE_VAL_MASK               0xFFFFFFFF




#define BASE32_ADDR_MSS_BSPREG  0x40000000


#ifdef __cplusplus
}
#endif

#endif /* BSP_REGS_H_ */


