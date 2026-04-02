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
 * @version     1.2.0           Remapped SW/FW registers to 8bit address boundary
 * @version     1.1.2           Aligned all register addresses, fields, masks
 *                              and reset values to ICD register table
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


/// --- 2.1. System Identification & Control (0x0000 - 0x001C) ----------------

#define REGMAP_VERSION_OFFSET                   0x0000          // ICD-REG-000
#define REGMAP_VERSION_REG_OFFSET               0x0000
    #define REG_MAP_VER_OFFSET                  0x0000          /// Register map version [31:0] (Reset: -)
    #define REG_MAP_VER_SHIFT                   0
    #define REG_MAP_VER_MASK                    0xFFFFFFFF

#define HARDWARE_ID_OFFSET                      0x0004          // ICD-REG-001
#define HARDWARE_ID_REG_OFFSET                  0x0004
    #define HW_ID_VAL_OFFSET                    0x0004          /// Unique Camera Hardware Identifier [31:0] (Reset: -)
    #define HW_ID_VAL_SHIFT                     0
    #define HW_ID_VAL_MASK                      0xFFFFFFFF

#define FW_VERSION_OFFSET                       0x0008          // ICD-REG-003
#define FW_VERSION_REG_OFFSET                   0x0008
    #define FW_VER_VAL_OFFSET                   0x0008          /// Firmware Version [31:0] (Reset: -)
    #define FW_VER_VAL_SHIFT                    0
    #define FW_VER_VAL_MASK                     0xFFFFFFFF

#define BUILD_ID_OFFSET                         0x000C          // ICD-REG-004
#define BUILD_ID_REG_OFFSET                     0x000C
    #define BUILD_CRC_OFFSET                    0x000C          /// Build Hash/CRC for traceability [31:0] (Reset: -)
    #define BUILD_CRC_SHIFT                     0
    #define BUILD_CRC_MASK                      0xFFFFFFFF

#define SYSTEM_CTRL_OFFSET                      0x0010          // ICD-REG-005
#define SYSTEM_CTRL_REG_OFFSET                  0x0010
    #define SYS_ENABLE_OFFSET                   0x0010          /// 1: Enable camera system, 0: Disable (Reset: 0)
    #define SYS_ENABLE_SHIFT                    0
    #define SYS_ENABLE_MASK                     (0x1 << 0)
    #define SYS_SW_RESET_OFFSET                 0x0010          /// 1: Trigger software-initiated reset of hardware (Reset: 0)
    #define SYS_SW_RESET_SHIFT                  1
    #define SYS_SW_RESET_MASK                   (0x1 << 1)
    #define SYS_AXI_EN_OFFSET                   0x0010          /// 1: Enable AXI bus for DMA transfer (Reset: 0)
    #define SYS_AXI_EN_SHIFT                    2
    #define SYS_AXI_EN_MASK                     (0x1 << 2)
    #define SYS_TPG_EN_OFFSET                   0x0010          /// 1: Enable Test Pattern Generator (Reset: 0)
    #define SYS_TPG_EN_SHIFT                    3
    #define SYS_TPG_EN_MASK                     (0x1 << 3)
    #define SYS_MODE_SEL_OFFSET                 0x0010          /// Camera operating mode select [7:4] (Reset: 0b0000)
    #define SYS_MODE_SEL_SHIFT                  4
    #define SYS_MODE_SEL_MASK                   (0xF << 4)
    #define SYS_CLR_ERR_OFFSET                  0x0010          /// Write 1 to clear latched system error flags (Reset: 0)
    #define SYS_CLR_ERR_SHIFT                   8
    #define SYS_CLR_ERR_MASK                    (0x1 << 8)
    /* Bits [31:9] Reserved */

#define SYSTEM_STATUS_OFFSET                    0x0014          // ICD-REG-006
#define SYSTEM_STATUS_REG_OFFSET                0x0014
    #define SYS_STAT_OK_OFFSET                  0x0014          /// 1: Normal system operation (Reset: 0)
    #define SYS_STAT_OK_SHIFT                   0
    #define SYS_STAT_OK_MASK                    (0x1 << 0)
    #define SYS_STAT_DEGRADED_OFFSET            0x0014          /// 1: Degraded operating state (Reset: 0)
    #define SYS_STAT_DEGRADED_SHIFT             1
    #define SYS_STAT_DEGRADED_MASK              (0x1 << 1)
    #define SYS_STAT_FAULT_OFFSET               0x0014          /// 1: System fault condition (Reset: 0)
    #define SYS_STAT_FAULT_SHIFT                2
    #define SYS_STAT_FAULT_MASK                 (0x1 << 2)
    /* Bit [3] Reserved */
    #define SYS_LAST_RST_CAUSE_OFFSET           0x0014          /// Cause of last reset [7:4] (Reset: 0)
    #define SYS_LAST_RST_CAUSE_SHIFT            4
    #define SYS_LAST_RST_CAUSE_MASK             (0xF << 4)
    /* Bits [31:8] Reserved */

/* 0x0018 - 0x001C: Reserved (ICD-REG-007) */


/// --- 2.2. Image Acquisition Parameters (0x0020 - 0x003C) -------------------

#define ACQ_EXPOSURE_OFFSET                     0x0020          // ICD-REG-0012
#define ACQ_EXPOSURE_REG_OFFSET                 0x0020
    #define ACQ_EXP_VAL_OFFSET                  0x0020          /// Exposure time; 1 LSB = 14 us [31:0] (Reset: 30)
    #define ACQ_EXP_VAL_SHIFT                   0
    #define ACQ_EXP_VAL_MASK                    0xFFFFFFFF

#define ACQ_FRAME_OFFSET                        0x0024          // ICD-REG-0013
#define ACQ_FRAME_REG_OFFSET                    0x0024
    #define ACQ_FPS_OFFSET                      0x0024          /// Frame rate select 1-15 FPS [7:0] (Reset: 5)
    #define ACQ_FPS_SHIFT                       0
    #define ACQ_FPS_MASK                        (0xFF << 0)
    #define ACQ_TEST_PAT_EN_OFFSET              0x0024          /// 1: Enable test pattern specified in PATTERN_MODE (Reset: 0)
    #define ACQ_TEST_PAT_EN_SHIFT               8
    #define ACQ_TEST_PAT_EN_MASK                (0x1 << 8)
    #define ACQ_COMPR_EN_OFFSET                 0x0024          /// 1: Enable compression selected in COMPRESSION_MODE (Reset: 0)
    #define ACQ_COMPR_EN_SHIFT                  9
    #define ACQ_COMPR_EN_MASK                   (0x1 << 9)
    #define ACQ_RES_MODE_OFFSET                 0x0024          /// Resolution mode: 0=2048x2048, 1=1024x1024, 2=512x512 [11:10] (Reset: 0)
    #define ACQ_RES_MODE_SHIFT                  10
    #define ACQ_RES_MODE_MASK                   (0x3 << 10)
    /* Bits [15:12] Reserved */
    #define ACQ_NUM_FRAMES_OFFSET               0x0024          /// Number of frames in acquisition sequence [31:16] (Reset: -)
    #define ACQ_NUM_FRAMES_SHIFT                16
    #define ACQ_NUM_FRAMES_MASK                 (0x7FFF << 16)

#define ACQ_HDR_COEFF_OFFSET                    0x0028          // ICD-REG-014
#define ACQ_HDR_COEFF_REG_OFFSET                0x0028
    #define HDR_COEFF_A_OFFSET                  0x0028          /// HDR A coefficient [15:0] (Reset: 224)
    #define HDR_COEFF_A_SHIFT                   0
    #define HDR_COEFF_A_MASK                    (0xFFFF << 0)
    #define HDR_COEFF_B_OFFSET                  0x0028          /// HDR B coefficient [31:16] (Reset: 400)
    #define HDR_COEFF_B_SHIFT                   16
    #define HDR_COEFF_B_MASK                    (0x7FFF << 16)

#define ACQ_CFG_OFFSET                          0x002C          // ICD-REG-015 / ICD-REG-016
#define ACQ_CFG_REG_OFFSET                      0x002C
    #define ACQ_HDR_MUX_OFFSET                  0x002C          /// HDR/CMS mux: 0=Bottom, 1=Top, 2=HDR, 3=CMS [2:0] (Reset: 0)
    #define ACQ_HDR_MUX_SHIFT                   0
    #define ACQ_HDR_MUX_MASK                    (0x7 << 0)
    /* Bits [15:3] Reserved */
    #define ACQ_HDR_THRESH_OFFSET               0x002C          /// HDR threshold MSB [31:16] (Reset: 3003)
    #define ACQ_HDR_THRESH_SHIFT                16
    #define ACQ_HDR_THRESH_MASK                 (0x7FFF << 16)

#define SENSOR_ADC_OFFSET_OFFSET                0x0030          // ICD-REG-017 / ICD-REG-019
#define SENSOR_ADC_OFFSET_REG_OFFSET            0x0030
    #define ADC_OFF_TOP_OFFSET                  0x0030          /// Top ADC offset value [15:0] (Reset: 62271)
    #define ADC_OFF_TOP_SHIFT                   0
    #define ADC_OFF_TOP_MASK                    (0xFFFF << 0)
    #define ADC_OFF_BOT_OFFSET                  0x0030          /// Bottom ADC offset value [31:16] (Reset: 62271)
    #define ADC_OFF_BOT_SHIFT                   16
    #define ADC_OFF_BOT_MASK                    (0x7FFF << 16)

#define ACQ_SENSOR_CFG_OFFSET                   0x0034          // ICD-REG-020
#define ACQ_SENSOR_CFG_REG_OFFSET               0x0034
    #define PGA_GAIN_TOP_OFFSET                 0x0034          /// Top PGA gain, 0.5-step adjust [5:0] (Reset: 2)
    #define PGA_GAIN_TOP_SHIFT                  0
    #define PGA_GAIN_TOP_MASK                   (0x3F << 0)
    #define PGA_GAIN_BOT_OFFSET                 0x0034          /// Bottom PGA gain, 0.5-step adjust [11:6] (Reset: 38)
    #define PGA_GAIN_BOT_SHIFT                  6
    #define PGA_GAIN_BOT_MASK                   (0x3F << 6)
    #define ACQ_SCAN_DIR_OFFSET                 0x0034          /// Row scan direction: 0=Normal, 1=Reverse [12] (Reset: 0)
    #define ACQ_SCAN_DIR_SHIFT                  12
    #define ACQ_SCAN_DIR_MASK                   (0x1 << 12)
    #define ACQ_DYN_LDC_EN_OFFSET               0x0034          /// 1: Enable dynamic low dark-current control [13] (Reset: 1)
    #define ACQ_DYN_LDC_EN_SHIFT                13
    #define ACQ_DYN_LDC_EN_MASK                 (0x1 << 13)
    #define ACQ_TRAIN_EN_OFFSET                 0x0034          /// 1: Enable sensor output training pattern [14] (Reset: 0)
    #define ACQ_TRAIN_EN_SHIFT                  14
    #define ACQ_TRAIN_EN_MASK                   (0x1 << 14)
    #define ACQ_CMS_EN_OFFSET                   0x0034          /// 1: Enable correlated multiple sampling [15] (Reset: 0)
    #define ACQ_CMS_EN_SHIFT                    15
    #define ACQ_CMS_EN_MASK                     (0x1 << 15)
    #define ACQ_TRAIN_VAL_OFFSET                0x0034          /// Training pattern value [27:16] (Reset: 0x98E)
    #define ACQ_TRAIN_VAL_SHIFT                 16
    #define ACQ_TRAIN_VAL_MASK                  (0xFFF << 16)
    /* Bits [31:28] Reserved */

#define ACQ_MODE_OFFSET                         0x0038          // ICD-REG-021
#define ACQ_MODE_REG_OFFSET                     0x0038
    #define COMPR_MODE_SEL_OFFSET               0x0038          /// Compression mode: 0=MODE0, 1=MODE1, 2=MODE2 [3:0] (Reset: 0)
    #define COMPR_MODE_SEL_SHIFT                0
    #define COMPR_MODE_SEL_MASK                 (0xF << 0)
    #define PAT_MODE_SEL_OFFSET                 0x0038          /// Test pattern: 0=Gradient, 1=Stars [7:4] (Reset: 0)
    #define PAT_MODE_SEL_SHIFT                  4
    #define PAT_MODE_SEL_MASK                   (0xF << 4)
    #define PROC_MODE_SEL_OFFSET                0x0038          /// Downstream processing mode select [15:8] (Reset: -)
    #define PROC_MODE_SEL_SHIFT                 8
    #define PROC_MODE_SEL_MASK                  (0xFF << 8)
    /* Bits [31:16] Reserved */

#define ACQ_PARAM_STATUS_OFFSET                 0x003C          // ICD-REG-022
#define ACQ_PARAM_STATUS_REG_OFFSET             0x003C
    #define PAR_VALID_OFFSET                    0x003C          /// 1: Last parameter set accepted as valid (Reset: -)
    #define PAR_VALID_SHIFT                     0
    #define PAR_VALID_MASK                      (0x1 << 0)
    #define PAR_CLAMPED_OFFSET                  0x003C          /// 1: Parameters clamped to allowed ranges (Reset: -)
    #define PAR_CLAMPED_SHIFT                   1
    #define PAR_CLAMPED_MASK                    (0x1 << 1)
    #define PAR_REJECT_OFFSET                   0x003C          /// 1: Parameters rejected as invalid (Reset: -)
    #define PAR_REJECT_SHIFT                    2
    #define PAR_REJECT_MASK                     (0x1 << 2)
    /* Bits [31:3] Reserved */


/// --- 2.3. Sensor Control (0x0040 - 0x0048) ---------------------------------

#define SENSOR_CTRL_OFFSET                      0x0040          // ICD-REG-070
#define SENSOR_CTRL_REG_OFFSET                  0x0040
    #define SNS_ENABLE_OFFSET                   0x0040          /// 1: Enable image sensor (Reset: -)
    #define SNS_ENABLE_SHIFT                    0
    #define SNS_ENABLE_MASK                     (0x1 << 0)
    #define SNS_SOFT_RESET_OFFSET               0x0040          /// 1: Reset image sensor logic (Reset: -)
    #define SNS_SOFT_RESET_SHIFT                1
    #define SNS_SOFT_RESET_MASK                 (0x1 << 1)
    #define SNS_STANDBY_OFFSET                  0x0040          /// 1: Place sensor in standby mode (Reset: -)
    #define SNS_STANDBY_SHIFT                   2
    #define SNS_STANDBY_MASK                    (0x1 << 2)
    #define SNS_OP_MODE_OFFSET                  0x0040          /// Sensor operating mode [7:4] (Reset: -)
    #define SNS_OP_MODE_SHIFT                   4
    #define SNS_OP_MODE_MASK                    (0xF << 4)
    /* Bits [31:8] Reserved */

#define SENSOR_STATUS_OFFSET                    0x0044          // ICD-REG-071
#define SENSOR_STATUS_REG_OFFSET                0x0044
    #define SNS_PRESENT_OFFSET                  0x0044          /// 1: Sensor presence detected (Reset: -)
    #define SNS_PRESENT_SHIFT                   0
    #define SNS_PRESENT_MASK                    (0x1 << 0)
    #define SNS_INIT_DONE_OFFSET                0x0044          /// 1: Sensor initialization completed (Reset: -)
    #define SNS_INIT_DONE_SHIFT                 1
    #define SNS_INIT_DONE_MASK                  (0x1 << 1)
    #define SNS_ERROR_OFFSET                    0x0044          /// 1: Sensor error detected (Reset: -)
    #define SNS_ERROR_SHIFT                     2
    #define SNS_ERROR_MASK                      (0x1 << 2)
    /* Bits [31:3] Reserved */

#define SENSOR_TEMP_OFFSET                      0x0048          // ICD-REG-061
#define SENSOR_TEMP_REG_OFFSET                  0x0048
    #define HK_TEMP_IMG_OFFSET                  0x0048          /// Image sensor temperature (0.01 °C) [15:0] (Reset: -)
    #define HK_TEMP_IMG_SHIFT                   0
    #define HK_TEMP_IMG_MASK                    0xFFFF
    #define HK_TEMP_FPGA_OFFSET                 0x0048          /// FPGA temperature (0.01 °C) [31:16] (Reset: -)
    #define HK_TEMP_FPGA_SHIFT                  16
    #define HK_TEMP_FPGA_MASK                   (0x7FFF << 16)

#define TVS_STATUS_OFFSET                       0x004C          // ICD-REG-062
#define TVS_STATUS_REG_OFFSET                   0x004C
    #define TVS_TEMP_HIGH_OFFSET                0x004C          /// 1: FPGA temperature high threshold alert (Reset: -)
    #define TVS_TEMP_HIGH_SHIFT                 0
    #define TVS_TEMP_HIGH_MASK                  (0x1 << 0)
    #define TVS_TEMP_LOW_OFFSET                 0x004C          /// 1: FPGA temperature low threshold alert (Reset: -)
    #define TVS_TEMP_LOW_SHIFT                  1
    #define TVS_TEMP_LOW_MASK                   (0x1 << 1)
    /* Bits [15:2] Reserved */
    #define TVS_V18_OFFSET                      0x004C          /// 1V8 FPGA core voltage readout [31:16] (Reset: -)
    #define TVS_V18_SHIFT                       16
    #define TVS_V18_MASK                        (0x7FFF << 16)

#define TVS_V1_RAW_OFFSET                       0x0050          // ICD-REG-063
#define TVS_V1_RAW_REG_OFFSET                   0x0050
    #define TVS_V1_OFFSET                       0x0050          /// 1V FPGA core voltage readout [15:0] (Reset: -)
    #define TVS_V1_SHIFT                        0
    #define TVS_V1_MASK                         0xFFFF
    #define TVS_V25_OFFSET                      0x0050          /// 2.5V FPGA core voltage readout [31:16] (Reset: -)
    #define TVS_V25_SHIFT                       16
    #define TVS_V25_MASK                        (0x7FFF << 16)


/// --- 2.4. Acquisition Command & Result (0x0200 - 0x021C) -------------------

#define ACQ_DDR_ADDR_LO_OFFSET                  0x0200          // ICD-REG-023
#define ACQ_DDR_ADDR_LO_REG_OFFSET              0x0200
    #define DDR_ADDR_LSW_OFFSET                 0x0200          /// Lower 32 bits of DDR target address (Reset: -)
    #define DDR_ADDR_LSW_SHIFT                  0
    #define DDR_ADDR_LSW_MASK                   0xFFFFFFFF

#define ACQ_DDR_ADDR_HI_OFFSET                  0x0204          // ICD-REG-024
#define ACQ_DDR_ADDR_HI_REG_OFFSET              0x0204
    #define DDR_ADDR_MSW_OFFSET                 0x0204          /// Upper 32 bits of DDR target address (Reset: -)
    #define DDR_ADDR_MSW_SHIFT                  0
    #define DDR_ADDR_MSW_MASK                   0xFFFFFFFF

#define ACQ_CMD_STATUS_OFFSET                   0x0208          // ICD-REG-025
#define ACQ_CMD_STATUS_REG_OFFSET               0x0208
    #define CMD_START_OFFSET                    0x0208          /// Write 1: Start acquisition sequence (RW1C) (Reset: -)
    #define CMD_START_SHIFT                     0
    #define CMD_START_MASK                      (0x1 << 0)
    #define CMD_ABORT_OFFSET                    0x0208          /// Write 1: Abort ongoing acquisition (RW1C) (Reset: -)
    #define CMD_ABORT_SHIFT                     1
    #define CMD_ABORT_MASK                      (0x1 << 1)
    #define CMD_CLEAR_STATUS_OFFSET             0x0208          /// Write 1: Clear DONE and ERROR status bits (RW1C) (Reset: -)
    #define CMD_CLEAR_STATUS_SHIFT              2
    #define CMD_CLEAR_STATUS_MASK               (0x1 << 2)
    #define CMD_BUSY_OFFSET                     0x0208          /// 1: Acquisition engine currently busy (Reset: -)
    #define CMD_BUSY_SHIFT                      3
    #define CMD_BUSY_MASK                       (0x1 << 3)
    #define CMD_DONE_OFFSET                     0x0208          /// 1: Acquisition sequence completed (Reset: -)
    #define CMD_DONE_SHIFT                      4
    #define CMD_DONE_MASK                       (0x1 << 4)
    #define CMD_ERROR_OFFSET                    0x0208          /// 1: Acquisition or transfer error occurred (Reset: -)
    #define CMD_ERROR_SHIFT                     5
    #define CMD_ERROR_MASK                      (0x1 << 5)
    #define CMD_ERR_CODE_OFFSET                 0x0208          /// Encoded reason for last acquisition error [15:11] (Reset: -)
    #define CMD_ERR_CODE_SHIFT                  11
    #define CMD_ERR_CODE_MASK                   (0x1F << 11)
    /* Bits [31:16] Reserved */

#define ACQ_IMAGE_COUNT_OFFSET                  0x020C          // ICD-REG-026
#define ACQ_IMAGE_COUNT_REG_OFFSET              0x020C
    #define IMG_COUNT_VAL_OFFSET                0x020C          /// Number of images produced in last acquisition [31:0] (Reset: -)
    #define IMG_COUNT_VAL_SHIFT                 0
    #define IMG_COUNT_VAL_MASK                  0xFFFFFFFF

#define ACQ_BYTES_LO_OFFSET                     0x0210          // ICD-REG-027
#define ACQ_BYTES_LO_REG_OFFSET                 0x0210
    #define ACQ_BYTES_LSW_OFFSET                0x0210          /// Lower 32 bits of bytes transferred (Reset: -)
    #define ACQ_BYTES_LSW_SHIFT                 0
    #define ACQ_BYTES_LSW_MASK                  0xFFFFFFFF

#define ACQ_BYTES_HI_OFFSET                     0x0214          // ICD-REG-028
#define ACQ_BYTES_HI_REG_OFFSET                 0x0214
    #define ACQ_BYTES_MSW_OFFSET                0x0214          /// Upper 32 bits of bytes transferred (Reset: -)
    #define ACQ_BYTES_MSW_SHIFT                 0
    #define ACQ_BYTES_MSW_MASK                  0xFFFFFFFF

#define ACQ_REFCLK_LO_OFFSET                    0x0218          // ICD-REG-029
#define ACQ_REFCLK_LO_REG_OFFSET                0x0218
    #define REF_CLK_LSW_OFFSET                  0x0218          /// Lower 32 bits of completion reference clock (Reset: -)
    #define REF_CLK_LSW_SHIFT                   0
    #define REF_CLK_LSW_MASK                    0xFFFFFFFF

#define ACQ_REFCLK_HI_OFFSET                    0x021C          // ICD-REG-030
#define ACQ_REFCLK_HI_REG_OFFSET                0x021C
    #define REF_CLK_MSW_OFFSET                  0x021C          /// Upper 32 bits of completion reference clock (Reset: -)
    #define REF_CLK_MSW_SHIFT                   0
    #define REF_CLK_MSW_MASK                    0xFFFFFFFF


/// --- 2.5. Power & Low-Level Control (0x0230 - 0x0238) ----------------------

#define PWREN_CTRL_OFFSET                       0x0230          // ICD-REG-008
#define PWREN_CTRL_REG_OFFSET                   0x0230
    #define PWR_3V5_EN_OFFSET                   0x0230          /// Enable 3.5V power rail (Reset: 0)
    #define PWR_3V5_EN_SHIFT                    0
    #define PWR_3V5_EN_MASK                     (0x1 << 0)
    #define PWR_2V0_EN_OFFSET                   0x0230          /// Enable 2.0V power rail (Reset: 0)
    #define PWR_2V0_EN_SHIFT                    1
    #define PWR_2V0_EN_MASK                     (0x1 << 1)
    #define PWR_3V8_EN_OFFSET                   0x0230          /// Enable 3.8V power rail (Reset: 0)
    #define PWR_3V8_EN_SHIFT                    2
    #define PWR_3V8_EN_MASK                     (0x1 << 2)
    #define PWR_0V8_EN_OFFSET                   0x0230          /// Enable 0.8V power rail (Reset: 0)
    #define PWR_0V8_EN_SHIFT                    3
    #define PWR_0V8_EN_MASK                     (0x1 << 3)
    #define PWR_CAN_PRI_EN_OFFSET               0x0230          /// Enable primary CAN interface (Reset: 0)
    #define PWR_CAN_PRI_EN_SHIFT                4
    #define PWR_CAN_PRI_EN_MASK                 (0x1 << 4)
    #define PWR_CAN_SEC_EN_OFFSET               0x0230          /// Enable secondary CAN interface (Reset: 0)
    #define PWR_CAN_SEC_EN_SHIFT                5
    #define PWR_CAN_SEC_EN_MASK                 (0x1 << 5)
    #define PWR_NAND_EN_OFFSET                  0x0230          /// NAND Flash enable / high-impedance (Reset: 1)
    #define PWR_NAND_EN_SHIFT                   6
    #define PWR_NAND_EN_MASK                    (0x1 << 6)
    #define PWR_EMMC_PR_EN_OFFSET               0x0230          /// Enable primary eMMC power (Reset: 0)
    #define PWR_EMMC_PR_EN_SHIFT                7
    #define PWR_EMMC_PR_EN_MASK                 (0x1 << 7)
    #define PWR_EMMC_SC_EN_OFFSET               0x0230          /// Enable secondary eMMC power (Reset: 1)
    #define PWR_EMMC_SC_EN_SHIFT                8
    #define PWR_EMMC_SC_EN_MASK                 (0x1 << 8)
    #define PWR_ETH_PR_COMA_EN_OFFSET           0x0230          /// Enable Ethernet primary COMA mode (Reset: 0)
    #define PWR_ETH_PR_COMA_EN_SHIFT            9
    #define PWR_ETH_PR_COMA_EN_MASK             (0x1 << 9)
    #define PWR_ETH_SC_COMA_EN_OFFSET           0x0230          /// Enable Ethernet secondary COMA mode (Reset: 0)
    #define PWR_ETH_SC_COMA_EN_SHIFT            10
    #define PWR_ETH_SC_COMA_EN_MASK             (0x1 << 10)
    #define PWR_ETH_PR_RESET_OFFSET             0x0230          /// Reset primary Ethernet interface (Reset: 1)
    #define PWR_ETH_PR_RESET_SHIFT              11
    #define PWR_ETH_PR_RESET_MASK               (0x1 << 11)
    #define PWR_ETH_SC_RESET_OFFSET             0x0230          /// Reset secondary Ethernet interface (Reset: 1)
    #define PWR_ETH_SC_RESET_SHIFT              12
    #define PWR_ETH_SC_RESET_MASK               (0x1 << 12)
    #define PWR_MON_EN_OFFSET                   0x0230          /// Enable board power monitoring (Reset: 1)
    #define PWR_MON_EN_SHIFT                    13
    #define PWR_MON_EN_MASK                     (0x1 << 13)
    /* Bits [31:14] Reserved */

#define ETH_SOFT_CTRL_OFFSET                    0x0234          // ICD-REG-009 / ICD-REG-0010
#define ETH_SOFT_CTRL_REG_OFFSET                0x0234
    #define ETH0_SOFT_EN_OFFSET                 0x0234          /// Enable Ethernet port 0 (Reset: 0)
    #define ETH0_SOFT_EN_SHIFT                  0
    #define ETH0_SOFT_EN_MASK                   (0x1 << 0)
    #define ETH1_SOFT_EN_OFFSET                 0x0234          /// Enable Ethernet port 1 (Reset: 0)
    #define ETH1_SOFT_EN_SHIFT                  1
    #define ETH1_SOFT_EN_MASK                   (0x1 << 1)
    #define ETH0_SOFT_RST_OFFSET                0x0234          /// Software reset Ethernet port 0 (Reset: 0)
    #define ETH0_SOFT_RST_SHIFT                 2
    #define ETH0_SOFT_RST_MASK                  (0x1 << 2)
    #define ETH1_SOFT_RST_OFFSET                0x0234          /// Software reset Ethernet port 1 (Reset: 0)
    #define ETH1_SOFT_RST_SHIFT                 3
    #define ETH1_SOFT_RST_MASK                  (0x1 << 3)
    #define ETH0_LINK_SPEED_OFFSET              0x0234          /// ETH0 speed: 0=10M, 1=100M, 2=1000M [5:4] (Reset: 1)
    #define ETH0_LINK_SPEED_SHIFT               4
    #define ETH0_LINK_SPEED_MASK                (0x3 << 4)
    #define ETH1_LINK_SPEED_OFFSET              0x0234          /// ETH1 speed: 0=10M, 1=100M, 2=1000M [7:6] (Reset: 1)
    #define ETH1_LINK_SPEED_SHIFT               6
    #define ETH1_LINK_SPEED_MASK                (0x3 << 6)
    #define ETH0_MODE_OFFSET                    0x0234          /// ETH0 duplex: 0=Full, 1=Half [9:8] (Reset: 0)
    #define ETH0_MODE_SHIFT                     8
    #define ETH0_MODE_MASK                      (0x3 << 8)

#define ETH_SOFT_CTRL2_OFFSET                   0x0238          // ICD-REG-0011
#define ETH_SOFT_CTRL2_REG_OFFSET               0x0238
    #define ETH1_MODE_OFFSET                    0x0238          /// ETH1 duplex: 0=Full, 1=Half [11:10] (Reset: 0)
    #define ETH1_MODE_SHIFT                     10
    #define ETH1_MODE_MASK                      (0x3 << 10)
    #define ETH0_LED_EN_OFFSET                  0x0238          /// Enable Ethernet primary LED (Reset: 0)
    #define ETH0_LED_EN_SHIFT                   12
    #define ETH0_LED_EN_MASK                    (0x1 << 12)
    #define ETH1_LED_EN_OFFSET                  0x0238          /// Enable Ethernet secondary LED (Reset: 0)
    #define ETH1_LED_EN_SHIFT                   13
    #define ETH1_LED_EN_MASK                    (0x1 << 13)
    /* Bits [31:14] Reserved */


/// --- 2.6. Housekeeping & Monitoring (0x0100 - 0x0180) ----------------------

#define HK_ACQ_COUNT_OFFSET                     0x0100          // ICD-REG-031
#define HK_ACQ_COUNT_REG_OFFSET                 0x0100
    #define HK_SUCCESS_COUNT_OFFSET             0x0100          /// Number of successful acquisitions [15:0] (Reset: -)
    #define HK_SUCCESS_COUNT_SHIFT              0
    #define HK_SUCCESS_COUNT_MASK               0xFFFF
    #define HK_FAIL_COUNT_OFFSET                0x0100          /// Number of failed acquisitions [31:16] (Reset: -)
    #define HK_FAIL_COUNT_SHIFT                 16
    #define HK_FAIL_COUNT_MASK                  (0x7FFF << 16)

#define HK_LAST_ERROR_OFFSET                    0x0104          // ICD-REG-032
#define HK_LAST_ERROR_REG_OFFSET                0x0104
    #define HK_LAST_ERROR_CODE_OFFSET           0x0104          /// Code of last acquisition error [15:0] (Reset: -)
    #define HK_LAST_ERROR_CODE_SHIFT            0
    #define HK_LAST_ERROR_CODE_MASK             0xFFFF
    #define HK_LAST_ERROR_CLASS_OFFSET          0x0104          /// Class/category of last error [31:16] (Reset: -)
    #define HK_LAST_ERROR_CLASS_SHIFT           16
    #define HK_LAST_ERROR_CLASS_MASK            (0x7FFF << 16)

#define HK_LAST_DDR_ADDR_LO_OFFSET              0x0108          // ICD-REG-033
#define HK_LAST_DDR_ADDR_LO_REG_OFFSET          0x0108
    #define HK_LAST_DDR_ADDR_LSW_OFFSET         0x0108          /// Lower 32 bits of last DDR address (Reset: -)
    #define HK_LAST_DDR_ADDR_LSW_SHIFT          0
    #define HK_LAST_DDR_ADDR_LSW_MASK           0xFFFFFFFF

#define HK_LAST_DDR_ADDR_HI_OFFSET              0x010C          // ICD-REG-034
#define HK_LAST_DDR_ADDR_HI_REG_OFFSET          0x010C
    #define HK_LAST_DDR_ADDR_MSW_OFFSET         0x010C          /// Upper 32 bits of last DDR address (Reset: -)
    #define HK_LAST_DDR_ADDR_MSW_SHIFT          0
    #define HK_LAST_DDR_ADDR_MSW_MASK           0xFFFFFFFF

/* 0x0110: Reserved */

#define HK_UPTIME_LO_OFFSET                     0x0114          // ICD-REG-035
#define HK_UPTIME_LO_REG_OFFSET                 0x0114
    #define HK_UPTIME_LSW_OFFSET                0x0114          /// Lower 32 bits of system uptime (Reset: -)
    #define HK_UPTIME_LSW_SHIFT                 0
    #define HK_UPTIME_LSW_MASK                  0xFFFFFFFF

#define HK_UPTIME_HI_OFFSET                     0x0118          // ICD-REG-037
#define HK_UPTIME_HI_REG_OFFSET                 0x0118
    #define HK_UPTIME_MSW_OFFSET                0x0118          /// Upper 32 bits of system uptime (Reset: -)
    #define HK_UPTIME_MSW_SHIFT                 0
    #define HK_UPTIME_MSW_MASK                  0xFFFFFFFF

#define HK_PWREN_STATUS_OFFSET                  0x011C          // ICD-REG-038
#define HK_PWREN_STATUS_REG_OFFSET              0x011C
    #define HK_P3V5_PG_OFFSET                   0x011C          /// 3.5V power rail power-good (Reset: -)
    #define HK_P3V5_PG_SHIFT                    0
    #define HK_P3V5_PG_MASK                     (0x1 << 0)
    #define HK_P2V0_PG_OFFSET                   0x011C          /// 2.0V power rail power-good (Reset: -)
    #define HK_P2V0_PG_SHIFT                    1
    #define HK_P2V0_PG_MASK                     (0x1 << 1)
    #define HK_PWR_FAULT_CODE_OFFSET            0x011C          /// Encoded power fault: 0=None,1=PowerUp,2=3V5,3=2V0,4=ALL [4:2] (Reset: -)
    #define HK_PWR_FAULT_CODE_SHIFT             2
    #define HK_PWR_FAULT_CODE_MASK              (0x7 << 2)

#define HK_ACQ_FRAME_LO_OFFSET                  0x0120          // ICD-REG-039
#define HK_ACQ_FRAME_LO_REG_OFFSET              0x0120
    #define HK_ACQ_FRAME_LSW_OFFSET             0x0120          /// Lower 32 bits of acquisition parameters mirror (Reset: -)
    #define HK_ACQ_FRAME_LSW_SHIFT              0
    #define HK_ACQ_FRAME_LSW_MASK               0xFFFFFFFF

#define HK_ACQ_FRAME_HI_OFFSET                  0x0124          // ICD-REG-040
#define HK_ACQ_FRAME_HI_REG_OFFSET              0x0124
    #define HK_ACQ_FRAME_MSW_OFFSET             0x0124          /// Upper 32 bits of acquisition parameters mirror (Reset: -)
    #define HK_ACQ_FRAME_MSW_SHIFT              0
    #define HK_ACQ_FRAME_MSW_MASK               0xFFFFFFFF

#define HK_ACQ_EXPOSURE_MS_OFFSET               0x0128          // ICD-REG-041
#define HK_ACQ_EXPOSURE_MS_REG_OFFSET           0x0128
    #define HK_EXPOSURE_VAL_OFFSET              0x0128          /// Exposure time of last acquisition [31:0] (Reset: -)
    #define HK_EXPOSURE_VAL_SHIFT               0
    #define HK_EXPOSURE_VAL_MASK                0xFFFFFFFF

#define HK_ACQ_RES_MODE_OFFSET                  0x012C          // ICD-REG-042
#define HK_ACQ_RES_MODE_REG_OFFSET              0x012C
    #define HK_RES_MODE_VAL_OFFSET              0x012C          /// Resolution mode of last acquisition [31:0] (Reset: -)
    #define HK_RES_MODE_VAL_SHIFT               0
    #define HK_RES_MODE_VAL_MASK                0xFFFFFFFF

#define HK_ACQ_MODE_OFFSET                      0x0130          // ICD-REG-043
#define HK_ACQ_MODE_REG_OFFSET                  0x0130
    #define HK_ACQ_MODE_VAL_OFFSET              0x0130          /// Compression, pattern and processing modes used [31:0] (Reset: -)
    #define HK_ACQ_MODE_VAL_SHIFT               0
    #define HK_ACQ_MODE_VAL_MASK                0xFFFFFFFF

#define HK_ACQ_SENSOR_CFG_OFFSET                0x0134          // ICD-REG-044
#define HK_ACQ_SENSOR_CFG_REG_OFFSET            0x0134
    #define HK_PGA_GAINS_HIGH_OFFSET            0x0134          /// PGA gains readout high gain [7:0] (Reset: -)
    #define HK_PGA_GAINS_HIGH_SHIFT             0
    #define HK_PGA_GAINS_HIGH_MASK              0xFF
    #define HK_PGA_GAINS_LOW_OFFSET             0x0134          /// PGA gains readout low gain [15:8] (Reset: -)
    #define HK_PGA_GAINS_LOW_SHIFT              8
    #define HK_PGA_GAINS_LOW_MASK               (0xFF << 8)
    /* Bits [31:16] Reserved */

#define HK_ACQ_ADC_OFFSETS_OFFSET               0x0138          // ICD-REG-045
#define HK_ACQ_ADC_OFFSETS_REG_OFFSET           0x0138
    #define HK_ADC_OFFSET_TOP_OFFSET            0x0138          /// ADC offsets used during acquisition (low gain) [15:0] (Reset: -)
    #define HK_ADC_OFFSET_TOP_SHIFT             0
    #define HK_ADC_OFFSET_TOP_MASK              0xFFFF
    #define HK_ADC_OFFSET_BOT_OFFSET            0x0138          /// ADC offsets used during acquisition (high gain) [31:16] (Reset: -)
    #define HK_ADC_OFFSET_BOT_SHIFT             16
    #define HK_ADC_OFFSET_BOT_MASK              (0x7FFF << 16)

#define HK_EMMCPR_1V8_VI_OFFSET                 0x013C          // ICD-REG-046
#define HK_EMMCPR_1V8_VI_REG_OFFSET             0x013C
    #define HK_EMMCPR_1V8_VOLTAGE_OFFSET        0x013C          /// Primary eMMC 1.8V rail voltage [15:0] (Reset: -)
    #define HK_EMMCPR_1V8_VOLTAGE_SHIFT         0
    #define HK_EMMCPR_1V8_VOLTAGE_MASK          0xFFFF
    #define HK_EMMCPR_1V8_CURRENT_OFFSET        0x013C          /// Primary eMMC 1.8V rail current [31:16] (Reset: -)
    #define HK_EMMCPR_1V8_CURRENT_SHIFT         16
    #define HK_EMMCPR_1V8_CURRENT_MASK          (0x7FFF << 16)

#define HK_EMMCPR_3V3_VI_OFFSET                 0x0140          // ICD-REG-047
#define HK_EMMCPR_3V3_VI_REG_OFFSET             0x0140
    #define HK_EMMCPR_3V3_VOLTAGE_OFFSET        0x0140          /// Primary eMMC 3.3V rail voltage [15:0] (Reset: -)
    #define HK_EMMCPR_3V3_VOLTAGE_SHIFT         0
    #define HK_EMMCPR_3V3_VOLTAGE_MASK          0xFFFF
    #define HK_EMMCPR_3V3_CURRENT_OFFSET        0x0140          /// Primary eMMC 3.3V rail current [31:16] (Reset: -)
    #define HK_EMMCPR_3V3_CURRENT_SHIFT         16
    #define HK_EMMCPR_3V3_CURRENT_MASK          (0x7FFF << 16)

#define HK_EMMCSC_1V8_VI_OFFSET                 0x0144          // ICD-REG-048
#define HK_EMMCSC_1V8_VI_REG_OFFSET             0x0144
    #define HK_EMMCSC_1V8_VOLTAGE_OFFSET        0x0144          /// Secondary eMMC 1.8V rail voltage [15:0] (Reset: -)
    #define HK_EMMCSC_1V8_VOLTAGE_SHIFT         0
    #define HK_EMMCSC_1V8_VOLTAGE_MASK          0xFFFF
    #define HK_EMMCSC_1V8_CURRENT_OFFSET        0x0144          /// Secondary eMMC 1.8V rail current [31:16] (Reset: -)
    #define HK_EMMCSC_1V8_CURRENT_SHIFT         16
    #define HK_EMMCSC_1V8_CURRENT_MASK          (0x7FFF << 16)

#define HK_EMMCSC_3V3_VI_OFFSET                 0x0148          // ICD-REG-049
#define HK_EMMCSC_3V3_VI_REG_OFFSET             0x0148
    #define HK_EMMCSC_3V3_VOLTAGE_OFFSET        0x0148          /// Secondary eMMC 3.3V rail voltage [15:0] (Reset: -)
    #define HK_EMMCSC_3V3_VOLTAGE_SHIFT         0
    #define HK_EMMCSC_3V3_VOLTAGE_MASK          0xFFFF
    #define HK_EMMCSC_3V3_CURRENT_OFFSET        0x0148          /// Secondary eMMC 3.3V rail current [31:16] (Reset: -)
    #define HK_EMMCSC_3V3_CURRENT_SHIFT         16
    #define HK_EMMCSC_3V3_CURRENT_MASK          (0x7FFF << 16)

#define HK_EMMC_STATUS_OFFSET                   0x014C          // ICD-REG-050
#define HK_EMMC_STATUS_REG_OFFSET               0x014C
    #define HK_EMMCPR_STATUS_OFFSET             0x014C          /// Primary eMMC power/fault: 0=OFF OK, 1=ON OK, 2=PowerUp fault, 3=Event fault [15:0] (Reset: -)
    #define HK_EMMCPR_STATUS_SHIFT              0
    #define HK_EMMCPR_STATUS_MASK               0xFFFF
    #define HK_EMMCSC_STATUS_OFFSET             0x014C          /// Secondary eMMC power/fault: 0=OFF OK, 1=ON OK, 2=PowerUp fault, 3=Event fault [31:16] (Reset: -)
    #define HK_EMMCSC_STATUS_SHIFT              16
    #define HK_EMMCSC_STATUS_MASK               (0x7FFF << 16)

#define HK_QSPI_STATUS_OFFSET                   0x0150          // ICD-REG-051
#define HK_QSPI_STATUS_REG_OFFSET               0x0150
    #define HK_QSPI_BUSY_OFFSET                 0x0150          /// QSPI controller busy [0] (Reset: -)
    #define HK_QSPI_BUSY_SHIFT                  0
    #define HK_QSPI_BUSY_MASK                   (0x1 << 0)
    #define HK_QSPI_FLASH_READY_OFFSET          0x0150          /// QSPI flash ready [1] (Reset: -)
    #define HK_QSPI_FLASH_READY_SHIFT           1
    #define HK_QSPI_FLASH_READY_MASK            (0x1 << 1)
    #define HK_QSPI_WIP_OFFSET                  0x0150          /// QSPI write in progress [2] (Reset: -)
    #define HK_QSPI_WIP_SHIFT                   2
    #define HK_QSPI_WIP_MASK                    (0x1 << 2)
    #define HK_QSPI_PROG_SUSPEND_OFFSET         0x0150          /// QSPI program suspended [3] (Reset: -)
    #define HK_QSPI_PROG_SUSPEND_SHIFT          3
    #define HK_QSPI_PROG_SUSPEND_MASK           (0x1 << 3)
    #define HK_QSPI_ERASE_SUSPEND_OFFSET        0x0150          /// QSPI erase suspended [4] (Reset: -)
    #define HK_QSPI_ERASE_SUSPEND_SHIFT         4
    #define HK_QSPI_ERASE_SUSPEND_MASK          (0x1 << 4)
    #define HK_QSPI_TIMEOUT_ERR_OFFSET          0x0150          /// QSPI timeout error [5] (Reset: -)
    #define HK_QSPI_TIMEOUT_ERR_SHIFT           5
    #define HK_QSPI_TIMEOUT_ERR_MASK            (0x1 << 5)
    #define HK_QSPI_PROTOCOL_ERR_OFFSET         0x0150          /// QSPI protocol error [6] (Reset: -)
    #define HK_QSPI_PROTOCOL_ERR_SHIFT          6
    #define HK_QSPI_PROTOCOL_ERR_MASK           (0x1 << 6)
    #define HK_QSPI_ILLEGAL_CMD_OFFSET          0x0150          /// QSPI illegal command [7] (Reset: -)
    #define HK_QSPI_ILLEGAL_CMD_SHIFT           7
    #define HK_QSPI_ILLEGAL_CMD_MASK            (0x1 << 7)
    #define HK_QSPI_ADDR_OOR_OFFSET             0x0150          /// QSPI address out of range [8] (Reset: -)
    #define HK_QSPI_ADDR_OOR_SHIFT              8
    #define HK_QSPI_ADDR_OOR_MASK               (0x1 << 8)
    /* Bits [31:9] Reserved */

#define HK_NAND_STATUS_OFFSET                   0x0154          // ICD-REG-052
#define HK_NAND_STATUS_REG_OFFSET               0x0154
    #define HK_NAND_BUSY_OFFSET                 0x0154          /// NAND controller busy [0] (Reset: -)
    #define HK_NAND_BUSY_SHIFT                  0
    #define HK_NAND_BUSY_MASK                   (0x1 << 0)
    #define HK_NAND_READY_OFFSET                0x0154          /// NAND controller ready [1] (Reset: -)
    #define HK_NAND_READY_SHIFT                 1
    #define HK_NAND_READY_MASK                  (0x1 << 1)
    #define HK_NAND_PROG_IN_PROGRESS_OFFSET     0x0154          /// NAND program operation in progress [2] (Reset: -)
    #define HK_NAND_PROG_IN_PROGRESS_SHIFT      2
    #define HK_NAND_PROG_IN_PROGRESS_MASK       (0x1 << 2)
    #define HK_NAND_ERASE_IN_PROGRESS_OFFSET    0x0154          /// NAND erase operation in progress [3] (Reset: -)
    #define HK_NAND_ERASE_IN_PROGRESS_SHIFT     3
    #define HK_NAND_ERASE_IN_PROGRESS_MASK      (0x1 << 3)
    #define HK_NAND_READ_IN_PROGRESS_OFFSET     0x0154          /// NAND read operation in progress [4] (Reset: -)
    #define HK_NAND_READ_IN_PROGRESS_SHIFT      4
    #define HK_NAND_READ_IN_PROGRESS_MASK       (0x1 << 4)
    #define HK_NAND_LAST_OP_SUCCESS_OFFSET      0x0154          /// Last NAND operation successful [5] (Reset: -)
    #define HK_NAND_LAST_OP_SUCCESS_SHIFT       5
    #define HK_NAND_LAST_OP_SUCCESS_MASK        (0x1 << 5)
    #define HK_NAND_LAST_OP_FAIL_OFFSET         0x0154          /// Last NAND operation failed [6] (Reset: -)
    #define HK_NAND_LAST_OP_FAIL_SHIFT          6
    #define HK_NAND_LAST_OP_FAIL_MASK           (0x1 << 6)
    #define HK_NAND_LAST_OP_ECC_CORR_OFFSET     0x0154          /// Last NAND ECC corrected error [7] (Reset: -)
    #define HK_NAND_LAST_OP_ECC_CORR_SHIFT      7
    #define HK_NAND_LAST_OP_ECC_CORR_MASK       (0x1 << 7)
    #define HK_NAND_ECC_ERR_OFFSET              0x0154          /// NAND ECC error detected [8] (Reset: -)
    #define HK_NAND_ECC_ERR_SHIFT               8
    #define HK_NAND_ECC_ERR_MASK                (0x1 << 8)
    #define HK_NAND_ECC_UNCORR_OFFSET           0x0154          /// NAND uncorrectable ECC error [9] (Reset: -)
    #define HK_NAND_ECC_UNCORR_SHIFT            9
    #define HK_NAND_ECC_UNCORR_MASK             (0x1 << 9)
    #define HK_NAND_BAD_BLOCK_DET_OFFSET        0x0154          /// Bad block detected [10] (Reset: -)
    #define HK_NAND_BAD_BLOCK_DET_SHIFT         10
    #define HK_NAND_BAD_BLOCK_DET_MASK          (0x1 << 10)
    #define HK_NAND_BAD_BLOCK_MARKED_OFFSET     0x0154          /// Bad block marked [11] (Reset: -)
    #define HK_NAND_BAD_BLOCK_MARKED_SHIFT      11
    #define HK_NAND_BAD_BLOCK_MARKED_MASK       (0x1 << 11)
    #define HK_NAND_WP_ACTIVE_OFFSET            0x0154          /// NAND write protection active [12] (Reset: -)
    #define HK_NAND_WP_ACTIVE_SHIFT             12
    #define HK_NAND_WP_ACTIVE_MASK              (0x1 << 12)

#define HK_NAND_ERROR_OFFSET                    0x0158          // ICD-REG-053
#define HK_NAND_ERROR_REG_OFFSET                0x0158
    #define HK_NAND_PROG_FAIL_OFFSET            0x0158          /// NAND program failure [0] (Reset: -)
    #define HK_NAND_PROG_FAIL_SHIFT             0
    #define HK_NAND_PROG_FAIL_MASK              (0x1 << 0)
    #define HK_NAND_ERASE_FAIL_OFFSET           0x0158          /// NAND erase failure [1] (Reset: -)
    #define HK_NAND_ERASE_FAIL_SHIFT            1
    #define HK_NAND_ERASE_FAIL_MASK             (0x1 << 1)
    #define HK_NAND_READ_FAIL_OFFSET            0x0158          /// NAND read failure [2] (Reset: -)
    #define HK_NAND_READ_FAIL_SHIFT             2
    #define HK_NAND_READ_FAIL_MASK              (0x1 << 2)
    #define HK_NAND_TIMEOUT_ERR_OFFSET          0x0158          /// NAND operation timeout [3] (Reset: -)
    #define HK_NAND_TIMEOUT_ERR_SHIFT           3
    #define HK_NAND_TIMEOUT_ERR_MASK            (0x1 << 3)
    #define HK_NAND_ECC_UNCORR_ERR_OFFSET       0x0158          /// NAND uncorrectable ECC error [4] (Reset: -)
    #define HK_NAND_ECC_UNCORR_ERR_SHIFT        4
    #define HK_NAND_ECC_UNCORR_ERR_MASK         (0x1 << 4)
    #define HK_NAND_ECC_THRESH_OFFSET           0x0158          /// NAND ECC correction threshold exceeded [5] (Reset: -)
    #define HK_NAND_ECC_THRESH_SHIFT            5
    #define HK_NAND_ECC_THRESH_MASK             (0x1 << 5)
    #define HK_NAND_BAD_BLOCK_ACCESS_OFFSET     0x0158          /// Access to bad NAND block [6] (Reset: -)
    #define HK_NAND_BAD_BLOCK_ACCESS_SHIFT      6
    #define HK_NAND_BAD_BLOCK_ACCESS_MASK       (0x1 << 6)
    #define HK_NAND_ADDR_OOR_OFFSET             0x0158          /// NAND address out of range [7] (Reset: -)
    #define HK_NAND_ADDR_OOR_SHIFT              7
    #define HK_NAND_ADDR_OOR_MASK               (0x1 << 7)

#define HK_EMMC_PR_STATUS_OFFSET                0x015C          // ICD-REG-054
#define HK_EMMC_PR_STATUS_REG_OFFSET            0x015C
    #define HK_EMMC_PR_BUSY_OFFSET              0x015C          /// Primary eMMC controller busy [0] (Reset: -)
    #define HK_EMMC_PR_BUSY_SHIFT               0
    #define HK_EMMC_PR_BUSY_MASK                (0x1 << 0)
    #define HK_EMMC_PR_READY_OFFSET             0x015C          /// Primary eMMC controller ready [1] (Reset: -)
    #define HK_EMMC_PR_READY_SHIFT              1
    #define HK_EMMC_PR_READY_MASK               (0x1 << 1)
    #define HK_EMMC_PR_IN_PROGRESS_OFFSET       0x015C          /// Primary eMMC command in progress [2] (Reset: -)
    #define HK_EMMC_PR_IN_PROGRESS_SHIFT        2
    #define HK_EMMC_PR_IN_PROGRESS_MASK         (0x1 << 2)
    #define HK_EMMC_PR_DATA_IN_PROGRESS_OFFSET  0x015C          /// Primary eMMC data transfer in progress [3] (Reset: -)
    #define HK_EMMC_PR_DATA_IN_PROGRESS_SHIFT   3
    #define HK_EMMC_PR_DATA_IN_PROGRESS_MASK    (0x1 << 3)
    #define HK_EMMC_PR_TUNING_IN_PROGRESS_OFFSET 0x015C         /// Primary eMMC tuning in progress [4] (Reset: -)
    #define HK_EMMC_PR_TUNING_IN_PROGRESS_SHIFT 4
    #define HK_EMMC_PR_TUNING_IN_PROGRESS_MASK  (0x1 << 4)
    #define HK_EMMC_PR_LAST_SUCCESS_OFFSET      0x015C          /// Primary last eMMC command successful [5] (Reset: -)
    #define HK_EMMC_PR_LAST_SUCCESS_SHIFT       5
    #define HK_EMMC_PR_LAST_SUCCESS_MASK        (0x1 << 5)
    #define HK_EMMC_PR_LAST_FAIL_OFFSET         0x015C          /// Primary last eMMC command failed [6] (Reset: -)
    #define HK_EMMC_PR_LAST_FAIL_SHIFT          6
    #define HK_EMMC_PR_LAST_FAIL_MASK           (0x1 << 6)
    #define HK_EMMC_PR_LAST_DATA_FAIL_OFFSET    0x015C          /// Primary last eMMC data transfer failed [7] (Reset: -)
    #define HK_EMMC_PR_LAST_DATA_FAIL_SHIFT     7
    #define HK_EMMC_PR_LAST_DATA_FAIL_MASK      (0x1 << 7)
    #define HK_EMMC_PR_CARD_INIT_OFFSET         0x015C          /// Primary eMMC card initialized [8] (Reset: -)
    #define HK_EMMC_PR_CARD_INIT_SHIFT          8
    #define HK_EMMC_PR_CARD_INIT_MASK           (0x1 << 8)
    #define HK_EMMC_PR_CARD_IDENT_OFFSET        0x015C          /// Primary eMMC card identified [9] (Reset: -)
    #define HK_EMMC_PR_CARD_IDENT_SHIFT         9
    #define HK_EMMC_PR_CARD_IDENT_MASK          (0x1 << 9)
    #define HK_EMMC_PR_XFER_STATE_OFFSET        0x015C          /// Primary eMMC card in transfer state [10] (Reset: -)
    #define HK_EMMC_PR_XFER_STATE_SHIFT         10
    #define HK_EMMC_PR_XFER_STATE_MASK          (0x1 << 10)
    #define HK_EMMC_PR_SLEEP_STATE_OFFSET       0x015C          /// Primary eMMC card in sleep state [11] (Reset: -)
    #define HK_EMMC_PR_SLEEP_STATE_SHIFT        11
    #define HK_EMMC_PR_SLEEP_STATE_MASK         (0x1 << 11)
    #define HK_EMMC_PR_BOOT_PART_OFFSET         0x015C          /// Primary eMMC boot partition active [12] (Reset: -)
    #define HK_EMMC_PR_BOOT_PART_SHIFT          12
    #define HK_EMMC_PR_BOOT_PART_MASK           (0x1 << 12)
    #define HK_EMMC_PR_USER_PART_OFFSET         0x015C          /// Primary eMMC user partition active [13] (Reset: -)
    #define HK_EMMC_PR_USER_PART_SHIFT          13
    #define HK_EMMC_PR_USER_PART_MASK           (0x1 << 13)
    #define HK_EMMC_PR_RELIABLE_WR_OFFSET       0x015C          /// Primary eMMC reliable write enabled [14] (Reset: -)
    #define HK_EMMC_PR_RELIABLE_WR_SHIFT        14
    #define HK_EMMC_PR_RELIABLE_WR_MASK         (0x1 << 14)
    #define HK_EMMC_PR_CACHE_EN_OFFSET          0x015C          /// Primary eMMC cache enabled [15] (Reset: -)
    #define HK_EMMC_PR_CACHE_EN_SHIFT           15
    #define HK_EMMC_PR_CACHE_EN_MASK            (0x1 << 15)

#define HK_EMMC_PR_ERROR_OFFSET                 0x0160          // ICD-REG-055
#define HK_EMMC_PR_ERROR_REG_OFFSET             0x0160
    #define HK_EMMC_PR_TIMEOUT_ERR_OFFSET       0x0160          /// Primary eMMC command timeout [0] (Reset: -)
    #define HK_EMMC_PR_TIMEOUT_ERR_SHIFT        0
    #define HK_EMMC_PR_TIMEOUT_ERR_MASK         (0x1 << 0)
    #define HK_EMMC_PR_CRC_ERR_OFFSET           0x0160          /// Primary eMMC command CRC error [1] (Reset: -)
    #define HK_EMMC_PR_CRC_ERR_SHIFT            1
    #define HK_EMMC_PR_CRC_ERR_MASK             (0x1 << 1)
    #define HK_EMMC_PR_ILLEGAL_OFFSET           0x0160          /// Primary illegal eMMC command [2] (Reset: -)
    #define HK_EMMC_PR_ILLEGAL_SHIFT            2
    #define HK_EMMC_PR_ILLEGAL_MASK             (0x1 << 2)
    #define HK_EMMC_PR_DATA_TIMEOUT_OFFSET      0x0160          /// Primary eMMC data timeout [3] (Reset: -)
    #define HK_EMMC_PR_DATA_TIMEOUT_SHIFT       3
    #define HK_EMMC_PR_DATA_TIMEOUT_MASK        (0x1 << 3)
    #define HK_EMMC_PR_DATA_CRC_ERR_OFFSET      0x0160          /// Primary eMMC data CRC error [4] (Reset: -)
    #define HK_EMMC_PR_DATA_CRC_ERR_SHIFT       4
    #define HK_EMMC_PR_DATA_CRC_ERR_MASK        (0x1 << 4)
    #define HK_EMMC_PR_DATA_ENDBIT_ERR_OFFSET   0x0160          /// Primary eMMC data end-bit error [5] (Reset: -)
    #define HK_EMMC_PR_DATA_ENDBIT_ERR_SHIFT    5
    #define HK_EMMC_PR_DATA_ENDBIT_ERR_MASK     (0x1 << 5)
    #define HK_EMMC_PR_DATA_STROBE_ERR_OFFSET   0x0160          /// Primary eMMC data strobe error [6] (Reset: -)
    #define HK_EMMC_PR_DATA_STROBE_ERR_SHIFT    6
    #define HK_EMMC_PR_DATA_STROBE_ERR_MASK     (0x1 << 6)
    #define HK_EMMC_PR_ADDR_OOR_OFFSET          0x0160          /// Primary eMMC address out of range [7] (Reset: -)
    #define HK_EMMC_PR_ADDR_OOR_SHIFT           7
    #define HK_EMMC_PR_ADDR_OOR_MASK            (0x1 << 7)
    #define HK_EMMC_PR_WP_VIOLATION_OFFSET      0x0160          /// Primary eMMC write protection violation [8] (Reset: -)
    #define HK_EMMC_PR_WP_VIOLATION_SHIFT       8
    #define HK_EMMC_PR_WP_VIOLATION_MASK        (0x1 << 8)
    #define HK_EMMC_PR_ERASE_SEQ_ERR_OFFSET     0x0160          /// Primary eMMC erase sequence error [9] (Reset: -)
    #define HK_EMMC_PR_ERASE_SEQ_ERR_SHIFT      9
    #define HK_EMMC_PR_ERASE_SEQ_ERR_MASK       (0x1 << 9)
    #define HK_EMMC_PR_SWITCH_ERR_OFFSET        0x0160          /// Primary eMMC EXT_CSD/CMD6 switch error [10] (Reset: -)
    #define HK_EMMC_PR_SWITCH_ERR_SHIFT         10
    #define HK_EMMC_PR_SWITCH_ERR_MASK          (0x1 << 10)
    #define HK_EMMC_PR_TUNING_FAILED_OFFSET     0x0160          /// Primary eMMC tuning failed [11] (Reset: -)
    #define HK_EMMC_PR_TUNING_FAILED_SHIFT      11
    #define HK_EMMC_PR_TUNING_FAILED_MASK       (0x1 << 11)

#define HK_EMMC_SC_STATUS_OFFSET                0x0164          // ICD-REG-056
#define HK_EMMC_SC_STATUS_REG_OFFSET            0x0164
    #define HK_EMMC_SC_BUSY_OFFSET              0x0164          /// Secondary eMMC controller busy [0] (Reset: -)
    #define HK_EMMC_SC_BUSY_SHIFT               0
    #define HK_EMMC_SC_BUSY_MASK                (0x1 << 0)
    #define HK_EMMC_SC_READY_OFFSET             0x0164          /// Secondary eMMC controller ready [1] (Reset: -)
    #define HK_EMMC_SC_READY_SHIFT              1
    #define HK_EMMC_SC_READY_MASK               (0x1 << 1)
    #define HK_EMMC_SC_IN_PROGRESS_OFFSET       0x0164          /// Secondary eMMC command in progress [2] (Reset: -)
    #define HK_EMMC_SC_IN_PROGRESS_SHIFT        2
    #define HK_EMMC_SC_IN_PROGRESS_MASK         (0x1 << 2)
    #define HK_EMMC_SC_DATA_IN_PROGRESS_OFFSET  0x0164          /// Secondary eMMC data transfer in progress [3] (Reset: -)
    #define HK_EMMC_SC_DATA_IN_PROGRESS_SHIFT   3
    #define HK_EMMC_SC_DATA_IN_PROGRESS_MASK    (0x1 << 3)
    #define HK_EMMC_SC_TUNING_IN_PROGRESS_OFFSET 0x0164         /// Secondary eMMC tuning in progress [4] (Reset: -)
    #define HK_EMMC_SC_TUNING_IN_PROGRESS_SHIFT 4
    #define HK_EMMC_SC_TUNING_IN_PROGRESS_MASK  (0x1 << 4)
    #define HK_EMMC_SC_LAST_SUCCESS_OFFSET      0x0164          /// Secondary last eMMC command successful [5] (Reset: -)
    #define HK_EMMC_SC_LAST_SUCCESS_SHIFT       5
    #define HK_EMMC_SC_LAST_SUCCESS_MASK        (0x1 << 5)
    #define HK_EMMC_SC_LAST_FAIL_OFFSET         0x0164          /// Secondary last eMMC command failed [6] (Reset: -)
    #define HK_EMMC_SC_LAST_FAIL_SHIFT          6
    #define HK_EMMC_SC_LAST_FAIL_MASK           (0x1 << 6)
    #define HK_EMMC_SC_LAST_DATA_FAIL_OFFSET    0x0164          /// Secondary last eMMC data transfer failed [7] (Reset: -)
    #define HK_EMMC_SC_LAST_DATA_FAIL_SHIFT     7
    #define HK_EMMC_SC_LAST_DATA_FAIL_MASK      (0x1 << 7)
    #define HK_EMMC_SC_CARD_INIT_OFFSET         0x0164          /// Secondary eMMC card initialized [8] (Reset: -)
    #define HK_EMMC_SC_CARD_INIT_SHIFT          8
    #define HK_EMMC_SC_CARD_INIT_MASK           (0x1 << 8)
    #define HK_EMMC_SC_CARD_IDENT_OFFSET        0x0164          /// Secondary eMMC card identified [9] (Reset: -)
    #define HK_EMMC_SC_CARD_IDENT_SHIFT         9
    #define HK_EMMC_SC_CARD_IDENT_MASK          (0x1 << 9)
    #define HK_EMMC_SC_XFER_STATE_OFFSET        0x0164          /// Secondary eMMC card in transfer state [10] (Reset: -)
    #define HK_EMMC_SC_XFER_STATE_SHIFT         10
    #define HK_EMMC_SC_XFER_STATE_MASK          (0x1 << 10)
    #define HK_EMMC_SC_SLEEP_STATE_OFFSET       0x0164          /// Secondary eMMC card in sleep state [11] (Reset: -)
    #define HK_EMMC_SC_SLEEP_STATE_SHIFT        11
    #define HK_EMMC_SC_SLEEP_STATE_MASK         (0x1 << 11)
    #define HK_EMMC_SC_BOOT_PART_OFFSET         0x0164          /// Secondary eMMC boot partition active [12] (Reset: -)
    #define HK_EMMC_SC_BOOT_PART_SHIFT          12
    #define HK_EMMC_SC_BOOT_PART_MASK           (0x1 << 12)
    #define HK_EMMC_SC_USER_PART_OFFSET         0x0164          /// Secondary eMMC user partition active [13] (Reset: -)
    #define HK_EMMC_SC_USER_PART_SHIFT          13
    #define HK_EMMC_SC_USER_PART_MASK           (0x1 << 13)
    #define HK_EMMC_SC_RELIABLE_WR_OFFSET       0x0164          /// Secondary eMMC reliable write enabled [14] (Reset: -)
    #define HK_EMMC_SC_RELIABLE_WR_SHIFT        14
    #define HK_EMMC_SC_RELIABLE_WR_MASK         (0x1 << 14)
    #define HK_EMMC_SC_CACHE_EN_OFFSET          0x0164          /// Secondary eMMC cache enabled [15] (Reset: -)
    #define HK_EMMC_SC_CACHE_EN_SHIFT           15
    #define HK_EMMC_SC_CACHE_EN_MASK            (0x1 << 15)

#define HK_EMMC_SC_ERROR_OFFSET                 0x0168          // ICD-REG-057
#define HK_EMMC_SC_ERROR_REG_OFFSET             0x0168
    #define HK_EMMC_SC_TIMEOUT_ERR_OFFSET       0x0168          /// Secondary eMMC command timeout [0] (Reset: -)
    #define HK_EMMC_SC_TIMEOUT_ERR_SHIFT        0
    #define HK_EMMC_SC_TIMEOUT_ERR_MASK         (0x1 << 0)
    #define HK_EMMC_SC_CRC_ERR_OFFSET           0x0168          /// Secondary eMMC command CRC error [1] (Reset: -)
    #define HK_EMMC_SC_CRC_ERR_SHIFT            1
    #define HK_EMMC_SC_CRC_ERR_MASK             (0x1 << 1)
    #define HK_EMMC_SC_ILLEGAL_OFFSET           0x0168          /// Illegal secondary eMMC command [2] (Reset: -)
    #define HK_EMMC_SC_ILLEGAL_SHIFT            2
    #define HK_EMMC_SC_ILLEGAL_MASK             (0x1 << 2)
    #define HK_EMMC_SC_DATA_TIMEOUT_OFFSET      0x0168          /// Secondary eMMC data timeout [3] (Reset: -)
    #define HK_EMMC_SC_DATA_TIMEOUT_SHIFT       3
    #define HK_EMMC_SC_DATA_TIMEOUT_MASK        (0x1 << 3)
    #define HK_EMMC_SC_DATA_CRC_ERR_OFFSET      0x0168          /// Secondary eMMC data CRC error [4] (Reset: -)
    #define HK_EMMC_SC_DATA_CRC_ERR_SHIFT       4
    #define HK_EMMC_SC_DATA_CRC_ERR_MASK        (0x1 << 4)
    #define HK_EMMC_SC_DATA_ENDBIT_ERR_OFFSET   0x0168          /// Secondary eMMC data end-bit error [5] (Reset: -)
    #define HK_EMMC_SC_DATA_ENDBIT_ERR_SHIFT    5
    #define HK_EMMC_SC_DATA_ENDBIT_ERR_MASK     (0x1 << 5)
    #define HK_EMMC_SC_DATA_STROBE_ERR_OFFSET   0x0168          /// Secondary eMMC data strobe error [6] (Reset: -)
    #define HK_EMMC_SC_DATA_STROBE_ERR_SHIFT    6
    #define HK_EMMC_SC_DATA_STROBE_ERR_MASK     (0x1 << 6)
    #define HK_EMMC_SC_ADDR_OOR_OFFSET          0x0168          /// Secondary eMMC address out of range [7] (Reset: -)
    #define HK_EMMC_SC_ADDR_OOR_SHIFT           7
    #define HK_EMMC_SC_ADDR_OOR_MASK            (0x1 << 7)
    #define HK_EMMC_SC_WP_VIOLATION_OFFSET      0x0168          /// Secondary eMMC write protection violation [8] (Reset: -)
    #define HK_EMMC_SC_WP_VIOLATION_SHIFT       8
    #define HK_EMMC_SC_WP_VIOLATION_MASK        (0x1 << 8)
    #define HK_EMMC_SC_ERASE_SEQ_ERR_OFFSET     0x0168          /// Secondary eMMC erase sequence error [9] (Reset: -)
    #define HK_EMMC_SC_ERASE_SEQ_ERR_SHIFT      9
    #define HK_EMMC_SC_ERASE_SEQ_ERR_MASK       (0x1 << 9)
    #define HK_EMMC_SC_SWITCH_ERR_OFFSET        0x0168          /// Secondary eMMC EXT_CSD/CMD6 switch error [10] (Reset: -)
    #define HK_EMMC_SC_SWITCH_ERR_SHIFT         10
    #define HK_EMMC_SC_SWITCH_ERR_MASK          (0x1 << 10)
    #define HK_EMMC_SC_TUNING_FAILED_OFFSET     0x0168          /// Secondary eMMC tuning failed [11] (Reset: -)
    #define HK_EMMC_SC_TUNING_FAILED_SHIFT      11
    #define HK_EMMC_SC_TUNING_FAILED_MASK       (0x1 << 11)

#define ETH_STATUS_OFFSET                       0x016C          // ICD-REG-058
#define ETH_STATUS_REG_OFFSET                   0x016C
    #define ETH0_LINK_UP_OFFSET                 0x016C          /// Ethernet port 0 link up [0] (Reset: -)
    #define ETH0_LINK_UP_SHIFT                  0
    #define ETH0_LINK_UP_MASK                   (0x1 << 0)
    #define ETH1_LINK_UP_OFFSET                 0x016C          /// Ethernet port 1 link up [1] (Reset: -)
    #define ETH1_LINK_UP_SHIFT                  1
    #define ETH1_LINK_UP_MASK                   (0x1 << 1)
    #define ETH0_SPEED_STATUS_OFFSET            0x016C          /// Ethernet port 0 negotiated speed [2] (Reset: -)
    #define ETH0_SPEED_STATUS_SHIFT             2
    #define ETH0_SPEED_STATUS_MASK              (0x1 << 2)
    #define ETH1_SPEED_STATUS_OFFSET            0x016C          /// Ethernet port 1 negotiated speed [3] (Reset: -)
    #define ETH1_SPEED_STATUS_SHIFT             3
    #define ETH1_SPEED_STATUS_MASK              (0x1 << 3)
    #define ETH0_LED_STATUS_OFFSET              0x016C          /// Ethernet port 0 link/speed LED status [4] (Reset: -)
    #define ETH0_LED_STATUS_SHIFT               4
    #define ETH0_LED_STATUS_MASK                (0x1 << 4)
    #define ETH1_LED_STATUS_OFFSET              0x016C          /// Ethernet port 1 link/speed LED status [5] (Reset: -)
    #define ETH1_LED_STATUS_SHIFT               5
    #define ETH1_LED_STATUS_MASK                (0x1 << 5)
    /* Bits [31:6] Reserved */

#define HK_LAST_DATE_TIME_LO_OFFSET             0x0170          // ICD-REG-059
#define HK_LAST_DATE_TIME_LO_REG_OFFSET         0x0170
    #define HK_LAST_ACQ_DATE_LSW_OFFSET         0x0170          /// Last acquisition date/time LSW [31:0] (Reset: -)
    #define HK_LAST_ACQ_DATE_LSW_SHIFT          0
    #define HK_LAST_ACQ_DATE_LSW_MASK           0xFFFFFFFF

#define HK_LAST_DATE_TIME_HI_OFFSET             0x0174          // ICD-REG-060
#define HK_LAST_DATE_TIME_HI_REG_OFFSET         0x0174
    #define HK_LAST_ACQ_DATE_MSW_OFFSET         0x0174          /// Last acquisition date/time MSW [31:0] (Reset: -)
    #define HK_LAST_ACQ_DATE_MSW_SHIFT          0
    #define HK_LAST_ACQ_DATE_MSW_MASK           0xFFFFFFFF

#define HK_SENSOR_TEMP_OFFSET                   0x0178          // ICD-REG-061
#define HK_SENSOR_TEMP_REG_OFFSET               0x0178
    #define HK_TEMP_IMG_HK_OFFSET               0x0178          /// Image sensor temperature (0.01 °C) [15:0] (Reset: -)
    #define HK_TEMP_IMG_HK_SHIFT                0
    #define HK_TEMP_IMG_HK_MASK                 0xFFFF
    #define HK_TEMP_FPGA_HK_OFFSET              0x0178          /// FPGA temperature (0.01 °C) [31:16] (Reset: -)
    #define HK_TEMP_FPGA_HK_SHIFT               16
    #define HK_TEMP_FPGA_HK_MASK                (0x7FFF << 16)

#define HK_TVS_STATUS_OFFSET                    0x017C          // ICD-REG-062
#define HK_TVS_STATUS_REG_OFFSET                0x017C
    #define HK_TVS_TEMP_HIGH_OFFSET             0x017C          /// FPGA temperature high threshold alert [0] (Reset: -)
    #define HK_TVS_TEMP_HIGH_SHIFT              0
    #define HK_TVS_TEMP_HIGH_MASK               (0x1 << 0)
    #define HK_TVS_TEMP_LOW_OFFSET              0x017C          /// FPGA temperature low threshold alert [1] (Reset: -)
    #define HK_TVS_TEMP_LOW_SHIFT               1
    #define HK_TVS_TEMP_LOW_MASK                (0x1 << 1)
    /* Bits [15:2] Reserved */
    #define HK_TVS_V18_OFFSET                   0x017C          /// 1V8 FPGA core voltage readout [31:16] (Reset: -)
    #define HK_TVS_V18_SHIFT                    16
    #define HK_TVS_V18_MASK                     (0x7FFF << 16)

#define HK_TVS_V1_RAW_OFFSET                    0x0180          // ICD-REG-063
#define HK_TVS_V1_RAW_REG_OFFSET                0x0180
    #define HK_TVS_V1_OFFSET                    0x0180          /// 1V FPGA core voltage readout [15:0] (Reset: -)
    #define HK_TVS_V1_SHIFT                     0
    #define HK_TVS_V1_MASK                      0xFFFF
    #define HK_TVS_V25_OFFSET                   0x0180          /// 2.5V FPGA core voltage readout [31:16] (Reset: -)
    #define HK_TVS_V25_SHIFT                    16
    #define HK_TVS_V25_MASK                     (0x7FFF << 16)


/// --- 2.7. Storage Device Status: DDR, QSPI, NAND & eMMC (0x0240 - 0x0254) -

#define DDR_ECC_CTRL_OFFSET                     0x0240          // ICD-REG-064
#define DDR_ECC_CTRL_REG_OFFSET                 0x0240
    #define ECC_ENABLE_BIT_OFFSET               0x0240          /// 1: Enable DDR ECC protection (Reset: -)
    #define ECC_ENABLE_BIT_SHIFT                0
    #define ECC_ENABLE_BIT_MASK                 (0x1 << 0)
    #define ECC_IRQ_ON_UNCORR_OFFSET            0x0240          /// 1: Enable interrupt on uncorrectable ECC error (Reset: -)
    #define ECC_IRQ_ON_UNCORR_SHIFT             1
    #define ECC_IRQ_ON_UNCORR_MASK              (0x1 << 1)
    /* Bits [31:2] Reserved */

#define DDR_ECC_STATUS_OFFSET                   0x0244          // ICD-REG-065
#define DDR_ECC_STATUS_REG_OFFSET               0x0244
    #define ECC_ENABLED_OFFSET                  0x0244          /// 1: DDR ECC currently enabled (Reset: -)
    #define ECC_ENABLED_SHIFT                   0
    #define ECC_ENABLED_MASK                    (0x1 << 0)
    #define ECC_CORR_ERROR_OFFSET               0x0244          /// 1: Correctable ECC error detected (Reset: -)
    #define ECC_CORR_ERROR_SHIFT                1
    #define ECC_CORR_ERROR_MASK                 (0x1 << 1)
    #define ECC_UNCORR_ERROR_OFFSET             0x0244          /// 1: Uncorrectable ECC error detected (Reset: -)
    #define ECC_UNCORR_ERROR_SHIFT              2
    #define ECC_UNCORR_ERROR_MASK               (0x1 << 2)
    /* Bits [31:3] Reserved */

#define DDR_ECC_CORR_CNT_OFFSET                 0x0248          // ICD-REG-066
#define DDR_ECC_CORR_CNT_REG_OFFSET             0x0248
    #define ECC_CORR_COUNT_OFFSET               0x0248          /// Number of corrected DDR ECC errors [31:0] (Reset: -)
    #define ECC_CORR_COUNT_SHIFT                0
    #define ECC_CORR_COUNT_MASK                 0xFFFFFFFF

#define DDR_ECC_UNCORR_CNT_OFFSET               0x024C          // ICD-REG-067
#define DDR_ECC_UNCORR_CNT_REG_OFFSET           0x024C
    #define ECC_UNCORR_COUNT_OFFSET             0x024C          /// Number of uncorrected DDR ECC errors [31:0] (Reset: -)
    #define ECC_UNCORR_COUNT_SHIFT              0
    #define ECC_UNCORR_COUNT_MASK               0xFFFFFFFF

#define DMA_CTRL_STATUS_OFFSET                  0x0250          // ICD-REG-068
#define DMA_CTRL_STATUS_REG_OFFSET              0x0250
    #define DMA_INIT_OFFSET                     0x0250          /// 1: Initialize DMA controller (RW1C) (Reset: -)
    #define DMA_INIT_SHIFT                      0
    #define DMA_INIT_MASK                       (0x1 << 0)
    #define DMA_RESET_OFFSET                    0x0250          /// 1: Reset DMA controller (RW1C) (Reset: -)
    #define DMA_RESET_SHIFT                     1
    #define DMA_RESET_MASK                      (0x1 << 1)
    #define DMA_START_BIT_OFFSET                0x0250          /// 1: Start DMA transfer (RW1C) (Reset: -)
    #define DMA_START_BIT_SHIFT                 2
    #define DMA_START_BIT_MASK                  (0x1 << 2)
    #define DMA_ABORT_OFFSET                    0x0250          /// 1: Abort ongoing DMA transfer (RW1C) (Reset: -)
    #define DMA_ABORT_SHIFT                     3
    #define DMA_ABORT_MASK                      (0x1 << 3)
    #define DMA_BUSY_BIT_OFFSET                 0x0250          /// 1: DMA transfer currently in progress (Reset: -)
    #define DMA_BUSY_BIT_SHIFT                  8
    #define DMA_BUSY_BIT_MASK                   (0x1 << 8)
    #define DMA_DONE_BIT_OFFSET                 0x0250          /// 1: DMA transfer completed successfully (Reset: -)
    #define DMA_DONE_BIT_SHIFT                  9
    #define DMA_DONE_BIT_MASK                   (0x1 << 9)
    #define DMA_ERR_BIT_OFFSET                  0x0250          /// 1: DMA transfer error detected (Reset: -)
    #define DMA_ERR_BIT_SHIFT                   10
    #define DMA_ERR_BIT_MASK                    (0x1 << 10)

#define ACQ_BUFFER_STATE_OFFSET                 0x0254          // ICD-REG-069
#define ACQ_BUFFER_STATE_REG_OFFSET             0x0254
    #define BUF_VALID_BIT_OFFSET                0x0254          /// RO 1: Acquisition buffer contains valid data (Reset: -)
    #define BUF_VALID_BIT_SHIFT                 0
    #define BUF_VALID_BIT_MASK                  (0x1 << 0)
    #define BUF_LOCKED_BIT_OFFSET               0x0254          /// RO 1: Acquisition buffer currently locked (Reset: -)
    #define BUF_LOCKED_BIT_SHIFT                1
    #define BUF_LOCKED_BIT_MASK                 (0x1 << 1)
    #define BUF_LOCK_BIT_OFFSET                 0x0254          /// RW 1: Lock acquisition buffer (Reset: -)
    #define BUF_LOCK_BIT_SHIFT                  2
    #define BUF_LOCK_BIT_MASK                   (0x1 << 2)
    #define BUF_RELEASE_BIT_OFFSET              0x0254          /// RW 1: Release acquisition buffer lock (Reset: -)
    #define BUF_RELEASE_BIT_SHIFT               3
    #define BUF_RELEASE_BIT_MASK                (0x1 << 3)
    #define STORE_START_BIT_OFFSET              0x0254          /// RW1C 1: Start storing buffer to NVM (Reset: -)
    #define STORE_START_BIT_SHIFT               4
    #define STORE_START_BIT_MASK                (0x1 << 4)
    #define STORE_ABORT_BIT_OFFSET              0x0254          /// RW1C 1: Abort buffer store operation (Reset: -)
    #define STORE_ABORT_BIT_SHIFT               5
    #define STORE_ABORT_BIT_MASK                (0x1 << 5)
    #define STORE_BUSY_BIT_OFFSET               0x0254          /// RW1C 1: Buffer store operation in progress (Reset: -)
    #define STORE_BUSY_BIT_SHIFT                6
    #define STORE_BUSY_BIT_MASK                 (0x1 << 6)
    #define STORE_DONE_BIT_OFFSET               0x0254          /// RW1C 1: Buffer store operation completed (Reset: -)
    #define STORE_DONE_BIT_SHIFT                7
    #define STORE_DONE_BIT_MASK                 (0x1 << 7)
    #define STORE_ERROR_BIT_OFFSET              0x0254          /// RW1C 1: Buffer store operation error (Reset: -)
    #define STORE_ERROR_BIT_SHIFT               8
    #define STORE_ERROR_BIT_MASK                (0x1 << 8)


/// --- 2.8. Interrupts & Events (0x02C0 - 0x02C8) ----------------------------

#define IRQ_ENABLE_OFFSET                       0x02C0          // ICD-REG-072
#define IRQ_ENABLE_REG_OFFSET                   0x02C0
    #define ACQ_DONE_IE_OFFSET                  0x02C0          /// 1: Enable IRQ on acquisition completion (Reset: -)
    #define ACQ_DONE_IE_SHIFT                   0
    #define ACQ_DONE_IE_MASK                    (0x1 << 0)
    #define ACQ_ERROR_IE_OFFSET                 0x02C0          /// 1: Enable IRQ on acquisition error (Reset: -)
    #define ACQ_ERROR_IE_SHIFT                  1
    #define ACQ_ERROR_IE_MASK                   (0x1 << 1)
    #define ECC_UNCORR_IE_OFFSET                0x02C0          /// 1: Enable IRQ on uncorrectable ECC error (Reset: -)
    #define ECC_UNCORR_IE_SHIFT                 2
    #define ECC_UNCORR_IE_MASK                  (0x1 << 2)
    /* Bits [31:3] Reserved */

#define IRQ_STATUS_OFFSET                       0x02C4          // ICD-REG-073
#define IRQ_STATUS_REG_OFFSET                   0x02C4
    #define IRQ_STAT_FLAGS_OFFSET               0x02C4          /// Latched interrupt flags (same bit pos as IRQ_ENABLE). Write 1 to clear. (Reset: -)
    #define IRQ_STAT_FLAGS_SHIFT                0
    #define IRQ_STAT_FLAGS_MASK                 (0x7 << 0)

#define IRQ_ROUTE_OFFSET                        0x02C8          // ICD-REG-074
#define IRQ_ROUTE_REG_OFFSET                    0x02C8
    #define IRQ_ROUTE_SEL_OFFSET                0x02C8          /// Select interrupt routing: MSS/Fabric [3:0] (Reset: -)
    #define IRQ_ROUTE_SEL_SHIFT                 0
    #define IRQ_ROUTE_SEL_MASK                  (0xF << 0)


#define BASE32_ADDR_MSS_BSPREG  0x40000000


#ifdef __cplusplus
}
#endif

#endif /* BSP_REGS_H_ */
