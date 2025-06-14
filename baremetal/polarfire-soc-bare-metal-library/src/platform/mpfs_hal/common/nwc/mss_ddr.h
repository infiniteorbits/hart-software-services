/*******************************************************************************
 * Copyright 2019 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file mss_ddr.h
 * @author Microchip FPGA Embedded Systems Solutions
 * @brief DDR related defines
 *
 */

/*=========================================================================*//**
  @page DDR setup and monitoring
  ==============================================================================
  @section intro_sec Introduction
  ==============================================================================
  The MPFS microcontroller subsystem (MSS) includes a number of hard core
  components physically located in the north west corner of the MSS on the die.
  This includes the DDR Phy.

  ==============================================================================
  @section Items located in the north west corner
  ==============================================================================
  - MSS PLL
  - SGMII
  - DDR phy
  - MSSIO

  ==============================================================================
  @section Overview of DDR related hardware
  ==============================================================================

  Simplified IP diagram


                                                  +--+
                  +--++ +-----+        +---v--+   |o |
                  |H0 | |H1-H4+--------> AXI  |   |t |
                  ++--+ +-+---+        |switch<---+h |
                   |      |            |      |   |e |
                   |      |            +--+---+   |r |
                  +v------v-------+       |       |  |
                  |L2 cache       |     non-cache |m |
                  +------------+--+       |       |a |
                           +---v----+ +---v---+   |s |
                           |seg 0   | | seg 1 |   |t |
                           +----^---+ +---^---+   |e |
                                |         |       |r |
        +-----------+------+----v---------v---+   |s |
        |Training IP|MTC   |DDR controller    |   +--+
        +-----------+------+--------+---------+
                                    |DFI
                                    |
                          +---------v--------+
                          | DDR Phy          |
                          +------------------+
                          | Bank 6 I/O       |
                          +-------+----------+
                                  |
                       +----------v---------------+
                       | +--+ +--+ +--+ +--+ +--+ |
                       | |D | |D | |R | |  | |  | |
                       | +--+ +--+ +--+ +--+ +--+ |
                       +--------------------------+


  -----------
  Hart0 E51
  -----------
  In most systems, the E51 will will setup and monitor the DDR

  -----------
  L2 Cache
  -----------
  Specific address range is used to access DDR via cache

  -----------
  AXI switch
  -----------
  DDR access via AXI switch for non-cached read/write

  -----------
  SEG regs
  -----------
  Used to map internal DDR address range to external fixed mapping.
  Note: DDR address ranges are at 32 bit and 64 bits

  -----------
  DDR controller
  -----------
  Manages DDR, refresh rates etc

  -----------
  DDR Training IP
  -----------
  Used to carry out training using IP state machines
   - BCLKSCLK_TIP_TRAINING .
   - addcmd_TIP_TRAINING
   - wrlvl_TIP_TRAINING
   - rdgate_TIP_TRAINING
   - dq_dqs_opt_TIP_TRAINING

  -----------
  DDR MTC - Memory test controller
  -----------
  Sends/receives test patterns to DDR. More efficient than using software.
  Used during write calibration and in DDR test routines.

  -----------
  DFI
  -----------
  Industry standard interface between phy, DDRC

  -----------
  DDR phy
  -----------
  PolarFire-SoC DDR phy manges data paath between pins and DFI

  ==============================================================================
  @section Overview of DDR embedded software
  ==============================================================================

  -----------
  Setup
  -----------
      - phy and IO
      - DDRC

  -----------
  Use Training IP
  -----------
      - kick-off RTL training IP state machine
      - Verify all training complete
            - BCLKSCLK_TIP_TRAINING .
            - addcmd_TIP_TRAINING
              This is a coarse training that moves the DDRCLK with PLL phase
              rotations in relation to the Address/Command bits to achieve the
              desired offset on the FPGA side.
            - wrlvl_TIP_TRAINING
            - rdgate_TIP_TRAINING
            - dq_dqs_opt_TIP_TRAINING

  Test this reg to determine training status:
  DDRCFG->DFI.STAT_DFI_TRAINING_COMPLETE.STAT_DFI_TRAINING_COMPLETE;

  -----------
  Write Calibration
  -----------
  The Memory Test Core plugged in to the front end of the DDR controller is used
  to perform lane-based writes and read backs and increment write calibration
  offset for each lane until data match occurs. The settings are recorded by the
  driver and available to be read using by an API function call.

  -----------
  VREF Calibration (optional)
  -----------
  VREF (DDR4 + LPDDR4 only) Set Remote VREF via mode register writes (MRW).
  In DDR4 and LPDDR4, VREF training may be done by writing to Mode Register 6 as
  defined by the JEDEC spec and, for example, Micron's datasheet for its 4Gb
  DDR4 RAM's:

  MR6 register definition from DDR4 datasheet

  | Mode Reg      | Description                                         |
  | ------------- |:---------------------------------------------------:|
  | 13,9,8        | DQ RX EQ must be 000                                |
  | 7             | Vref calib enable = => disables, 1 ==> enabled      |
  | 6             | Vref Calibration range 0 = Range 0, 1 - Range 2     |
  | 5:0           | Vref Calibration value                              |

  This step is not implemented in the current driver. It can be implemented in
  the same way as write Calibration and will be added during board verification.

  -----------
  FPGA VREF (Local VREF training) (optional)
  -----------
  In addition to memory VREFDQ training, or remote training, it is possible to
  train the VREFDQ on the FPGA device. WE refer to this training as local VREF
  training.
  Train FPGA VREF using the vrgen_h and vrgen_v registers
  To manipulate the FPGA VREF value, firmware must write to the DPC_BITS
  register, located at physical address 0x2000 7184.
        CFG_DDR_SGMII_PHY->DPC_BITS.bitfield.dpc_vrgen_h;
        CFG_DDR_SGMII_PHY->DPC_BITS.bitfield.dpc_vrgen_v;
  Like memory VREFDQ training, FPGA VREFDQ training seeks to find an average/
  optimal VREF value by sweeping across the full range and finding a left edge
  and a right edge.
  This step is not implemented in the current driver. It can be implemented in
  the same way as write Calibration and will be added during board verification.

  -----------
  DQ Write Offset
  -----------
  (LPDDR4 only) ), there must be an offset at the input to the LPDDR4 memory
  device between DQ and DQS. Unlike other flavors of DDR, which match DQ and DQS
  at the SDRAM, for LPDDR4 this relationship must be trained, because it will
  vary between 200ps and 600ps, which, depending on the data rate, could be as
  much as one whole bit period.
  This training is integrated with write calibration, because it, too, is done
  on a per-lane basis. That is, each lane is trained separately by sweeping the
  DQ output delay to find a valid range and of DQ output delays and center it.
  DQ output delays are swept using the expert_dlycnt_move_reg0 register located
  in the MSS DDR TIP.


  -----------
  Overview Flow diagram of Embedded software setup
  -----------

               +--------------------------------------------+
               |      Some  Preconditions                   |
               |   DCE, CORE_UP, FLASH VALID, MSS_IO_EN     |
               |   MSS PLL setup, Clks to MSS setup         |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |   Check if in off mode, ret if so          |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  set ddr mode and VS bits                  |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  soft reset I/O decoders                   |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Set RPC registers that need manual setup  |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Soft reset IP- to load RPC ->SCB regs     |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Calibrate I/O - as they are now setup     |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Configure the DDR PLL - Using SCB writes  |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Setup the SEG regs - NB May move this down|
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Set-up the DDRC - Using Libero values     |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Reset training IP                         |
               +--------------------+-----------------------+
                                    |
               +----------------- --v-----------------------+
               |  Rotate BCLK by programmed amount (degrees)|
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Set training parameters                   |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Assert traing reset                       |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Wait until traing complete                |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Write calibrate                           |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  If LPDDR4, calibrate DQ                   |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Sanity check training                     |
               +--------------------+-----------------------+
                                    |
               +--------------------v-----------------------+
               |  Return 0 if all went OK                   |
               +--------------------------------------------+

 *//*=========================================================================*/


#ifndef __MSS_DDRC_H_
#define __MSS_DDRC_H_ 1

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**

 */
typedef enum DDR_TYPE_
{

    DDR3                                = 0x00,         /*!< 0 DDR3          */
    DDR3L                               = 0x01,         /*!< 1 DDR3L         */
    DDR4                                = 0x02,         /*!< 2 DDR4          */
    LPDDR3                              = 0x03,         /*!< 3 LPDDR3        */
    LPDDR4                              = 0x04,         /*!< 4 LPDDR4        */
    DDR_OFF_MODE                        = 0x07          /*!< 4 LPDDR4        */
} DDR_TYPE;

typedef enum DDR_MEMORY_ACCESS_
{
    DDR_NC_256MB,
    DDR_NC_WCB_256MB,
    DDR_NC_2GB,
    DDR_NC_WCB_2GB,
} DDR_MEMORY_ACCESS;

/* DDR clk frequency - should come from  */
#if !defined (LIBERO_SETTING_DDR_CLK)
#define LIBERO_SETTING_DDR_CLK                  DDR_1600_MHZ
#endif

/* this is a fixed value, currently only 5 supported in the TIP  */
#define MAX_POSSIBLE_TIP_TRAININGS    0x05U

/* LIBERO_SETTING_TIP_CFG_PARAMS
 *     ADDCMD_OFFSET                     [0:3]   RW value */
#define ADDRESS_CMD_OFFSETT_MASK        (0x7U<<0U)

#define BCLK_SCLK_OFFSET_SHIFT                (3U)
#define BCLK_SCLK_OFFSET_MASK           (0x7U<<3U)

#define BCLK_DPC_VRGEN_V_SHIFT                (12U)
#define BCLK_DPC_VRGEN_V_MASK          (0x3FU<<12U)

#define BCLK_DPC_VRGEN_H_SHIFT                (4U)
#define BCLK_DPC_VRGEN_H_MASK          (0x3FU<<4U)

#define BCLK_DPC_VRGEN_VS_SHIFT                (0U)
#define BCLK_DPC_VRGEN_VS_MASK           (0xFU<<0U)

/* offsets and masks LIBERO_SETTING_DPC_BITS */
#define DDR_DPC_VS_SHIFT                      0U
#define DDR_DPC_VRGEN_H_SHIFT                 4U
#define DDR_DPC_VRGEN_EN_H_SHIFT              10U
#define DDR_DPC_MOVE_EN_H_SHIFT               11U
#define DDR_DPC_VRGEN_V_SHIFT                 12U
#define DDR_DPC_VRGEN_EN_V_SHIFT              18U
#define DDR_DPC_MOVE_EN_V_SHIFT               19U
#define DDR_DPC_VS_MASK                       (0xFU << DDR_DPC_VS_SHIFT)
#define DDR_DPC_VRGEN_H_MASK                  (0x3FU << DDR_DPC_VRGEN_H_SHIFT)
#define DDR_DPC_VRGEN_EN_H_MASK               (0x1U << DDR_DPC_VRGEN_EN_H_SHIFT)
#define DDR_DPC_MOVE_EN_H_MASK                (0x1U << DDR_DPC_MOVE_EN_H_SHIFT)
#define DDR_DPC_VRGEN_V_MASK                  (0xFU << DDR_DPC_VRGEN_V_SHIFT)
#define DDR_DPC_VRGEN_EN_V_MASK               (0x1U << DDR_DPC_VRGEN_EN_V_SHIFT)
#define DDR_DPC_MOVE_EN_V_MASK                (0x1U << DDR_DPC_MOVE_EN_V_SHIFT)

#define DDR_DPC_VRGEN_V       ((LIBERO_SETTING_DPC_BITS & DDR_DPC_VRGEN_V_MASK)\
                               >>DDR_DPC_VRGEN_V_SHIFT)
#define DDR_VREF_CA                     DDR_DPC_VRGEN_V
#define DDR_DPC_VRGEN_H       ((LIBERO_SETTING_DPC_BITS & DDR_DPC_VRGEN_H_MASK)\
                               >>DDR_DPC_VRGEN_H_SHIFT)
#define DDR_VREF_DATA                   DDR_DPC_VRGEN_H


/* masks and associated values used with  DDRPHY_MODE register */
#define DDRPHY_MODE_MASK                0x7U
/* ECC */
#define DDRPHY_MODE_ECC_MASK            (0x1U<<3U)
#define DDRPHY_MODE_ECC_ON              (0x1U<<3U)
/* Bus width */
#define DDRPHY_MODE_BUS_WIDTH_4_LANE    (0x1U<<5U)
#define DDRPHY_MODE_BUS_WIDTH_MASK      (0x7U<<5U)
/* Number of ranks, 1 or 2 supported */
#define DDRPHY_MODE_RANK_MASK           (0x1U<<26U)
#define DDRPHY_MODE_ONE_RANK            (0x0U<<26U)
#define DDRPHY_MODE_TWO_RANKS           (0x1U<<26U)

#define DMI_DBI_MASK                    (~(0x1U<<8U))

#define DDR_MODE (LIBERO_SETTING_DDRPHY_MODE & DDRPHY_MODE_MASK)
#define DDR_DATA_WIDTH ((LIBERO_SETTING_DDRPHY_MODE & (0x7U<<5U))>>5U)
#define DDR_ECC ((LIBERO_SETTING_DDRPHY_MODE & (0x1U<<3U))>>3U)
#define DDR_CRC ((LIBERO_SETTING_DDRPHY_MODE & (0x1U<<4U))>>4U)
#define DDR_DM ((LIBERO_SETTING_DDRPHY_MODE & (0x1U<<8U))>>8U)
#define DDR_RANKS ((LIBERO_SETTING_DDRPHY_MODE & (0x1U<<26U))>>26U)

#define DDR_FPGA_DQ_DRIVE ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<9U))>>9U)
#define DDR_FPGA_DQS_DRIVE ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<11U))>>11U)
#define DDR_FPGA_ADD_CMD_DRIVE ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<13U))>>13U)
#define DDR_FPGA_CLOCK_OUT_DRIVE ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<15U))>>15U)

#define DDR_FPGA_DQ_TERMINATION ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<17U))>>17U)
#define DDR_FPGA_DQS_TERMINATION ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<19U))>>19U)
#define DDR_FPGA_ADD_CMD_TERMINATION ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<21U))>>21U)
#define DDR_FPGA_PRESET_ODT_CLK ((LIBERO_SETTING_DDRPHY_MODE & (0x3U<<23U))>>23U)

/* Write latency min/max settings If write calibration fails
 * For Libero setting, we iterate through these values looking for a
 * Calibration pass */
#define DDR_CAL_MIN_LATENCY             0UL
#define DDR_CAL_MAX_LATENCY             3UL

#define MTC_TIMEOUT_ERROR               0x02U

#define DDR_MODE_REG_VREF               0xCU

#define DDR_CALIBRATION_PASSED          0xFF
#define DDR_CALIBRATION_FAILED          0xFE
#define DDR_CALIBRATION_SUCCESS         0xFC

#define PARSE_LANE(q) (((LIBERO_SETTING_DATA_LANES_USED==3U)&&\
                                                          (q==2U)) ? (4U) : (q))
#define PARSE_LANE_SHIFT(q,r) (((LIBERO_SETTING_DATA_LANES_USED==3U)\
                                                       &&(q==2U)) ? (16U) : (r))


/*
 * Some settings that are only used during testing in new DDR setup
 */
/* #define LANE_ALIGNMENT_RESET_REQUIRED leave commented, not required */
#define ABNORMAL_RETRAIN_CA_DECREASE_COUNT          2U
#define ABNORMAL_RETRAIN_CA_DLY_DECREASE_COUNT      2U
#define DQ_DQS_NUM_TAPS                             5U
#if !defined (LIBERO_SETTING_MIN_RPC_156_VALUE)
#define LIBERO_SETTING_MIN_RPC_156_VALUE            1U
#endif
#if !defined (LIBERO_SETTING_MAX_RPC_156_VALUE)
#define LIBERO_SETTING_MAX_RPC_156_VALUE            9U
#endif
#if !defined (LIBERO_SETTING_RPC_156_VALUE)

/* DDR clk frequency - should come from  */
#if (LIBERO_SETTING_DDR_CLK == DDR_1600_MHZ)
#define LIBERO_SETTING_RPC_156_VALUE                6U
#else
#define LIBERO_SETTING_RPC_156_VALUE                1U
#endif
#endif
/* #define SW_CONFIG_LPDDR_WR_CALIB_FN */

#if !defined (LIBERO_SETTING_MAX_MANUAL_REF_CLK_PHASE_OFFSET)
#define LIBERO_SETTING_MAX_MANUAL_REF_CLK_PHASE_OFFSET 4U
#endif
#if !defined (LIBERO_SETTING_MIN_MANUAL_REF_CLK_PHASE_OFFSET)
#define LIBERO_SETTING_MIN_MANUAL_REF_CLK_PHASE_OFFSET 2U
#endif
#if !defined (LIBERO_SETTING_MANUAL_REF_CLK_PHASE_OFFSET)
/* If skipping add/cmd training, this value is used */
/* The value used may be trained. The value here should be determined */
/* for the board design by performing a manual sweep. */
#define LIBERO_SETTING_MANUAL_REF_CLK_PHASE_OFFSET    0x00000006UL
    /* CA_BUS_RX_OFF_POST_TRAINING       [0:1]   RW value= 0x1 */
#endif

#ifndef NUM_WRITES_DDR_MODE_REG
#define NUM_WRITES_DDR_MODE_REG 10U
#endif

#ifndef TRANSITION_A5_THRESHOLD
#define TRANSITION_A5_THRESHOLD                     18U
#endif

#ifndef ADD_CMD_INC_FREQ_DDR3
#define ADD_CMD_INC_FREQ_DDR3       3U
#endif
#ifndef  ADD_CMD_INC_FREQ_DDR3L
#define ADD_CMD_INC_FREQ_DDR3L      3U
#endif
#ifndef ADD_CMD_INC_FREQ_DDR4
#define ADD_CMD_INC_FREQ_DDR4       3U
#endif
#ifndef ADD_CMD_INC_FREQ_LPDDR3
#define ADD_CMD_INC_FREQ_LPDDR3     3U
#endif
#ifndef ADD_CMD_INC_FREQ_LPDDR4
#define ADD_CMD_INC_FREQ_LPDDR4     6U
#endif

#ifndef ADD_CMD_TRANS_A5_THRES_DDR3
#define ADD_CMD_TRANS_A5_THRES_DDR3       18U
#endif
#ifndef  ADD_CMD_TRANS_A5_THRES_DDR3L
#define ADD_CMD_TRANS_A5_THRES_DDR3L      18U
#endif
#ifndef ADD_CMD_TRANS_A5_THRES_DDR4
#define ADD_CMD_TRANS_A5_THRES_DDR4       18U
#endif
#ifndef ADD_CMD_TRANS_A5_THRES_LPDDR3
#define ADD_CMD_TRANS_A5_THRES_LPDDR3     18U
#endif
#ifndef ADD_CMD_TRANS_A5_THRES_LPDDR4
#define ADD_CMD_TRANS_A5_THRES_LPDDR4     18U
#endif

/* Value used during write leveling */
#ifndef DPC_VRGEN_H_LPDDR4_WR_LVL_VAL
#define DPC_VRGEN_H_LPDDR4_WR_LVL_VAL   0x5U
#endif
/* Value used during write leveling */
#ifndef DPC_VRGEN_H_DDR3_WR_LVL_VAL
#define DPC_VRGEN_H_DDR3_WR_LVL_VAL     0x2U
#endif

#if !defined (DDR_FULL_32BIT_NC_CHECK_EN)
#define DDR_FULL_32BIT_NC_CHECK_EN  1
#endif

#if !defined (DDR_FULL_32BIT_CACHED_CHECK_EN)
#define DDR_FULL_32BIT_CACHED_CHECK_EN  0
#endif

#if !defined (PATTERN_TEST_NUM_PATTERN_IN_CACHE_READS)
#define PATTERN_TEST_NUM_PATTERN_IN_CACHE_READS  1
#endif

#if !defined (PATTERN_TEST_SIZE)
#define PATTERN_TEST_SIZE 0x02000000UL
#endif

#if !defined (PATTERN_TEST_START_OFFSET)
#define PATTERN_TEST_START_OFFSET      12U
#endif

#if !defined (PATTERN_TEST_NUM_OFFSET_INCS)
#define PATTERN_TEST_NUM_OFFSET_INCS    1U
#endif

#define PATTERN_TEST_MIN_OFFSET         1U
#define PATTERN_TEST_MAX_OFFSET         16U

#if !defined (LIBERO_SETTING_USE_CK_PUSH_DDR4_LPDDR3)
#define LIBERO_SETTING_USE_CK_PUSH_DDR4_LPDDR3    1U
#endif

#if !defined (DEFAULT_RPC_166_VALUE)
#define DEFAULT_RPC_166_VALUE  2UL
#endif

/* set to 0 if you want to turn off tuning */
#if !defined (TUNE_RPC_166_VALUE)
#define TUNE_RPC_166_VALUE 1
#endif

#if !defined (MIN_RPC_166_VALUE)
#define MIN_RPC_166_VALUE  2UL
#endif

#if !defined (MAX_RPC_166_VALUE)
#define MAX_RPC_166_VALUE 4UL
#endif

#define NUM_RPC_166_VALUES (MAX_RPC_166_VALUE - MIN_RPC_166_VALUE)

/* Offsets for each mem type- fixed values */
#if !defined (LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_LPDDR4)
#define LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_LPDDR4            0x00000005UL
#endif
#if !defined (LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_LPDDR3)
#define LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_LPDDR3            0x00000007UL
#endif
#if !defined (LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR4)
#define LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR4              0x00000006UL
#endif
#if !defined (LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR3)
#define LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR3              0x00000006UL
#endif
#if !defined (LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR3L)
#define LIBERO_SETTING_SW_TRAING_BCLK_SCLK_OFFSET_DDR3L             0x00000006UL
#endif
/*
 * 0x6DU => setting vref_ca to 40%
 * This (0x6DU) is the default setting.
 * Currently not being used, here for possible future use.
 * */
#if !defined (DDR_MODE_REG_VREF_VALUE)
#define DDR_MODE_REG_VREF_VALUE       0x6DU
#endif

/* number of test writes to perform */
#if !defined (SW_CFG_NUM_READS_WRITES)
#define SW_CFG_NUM_READS_WRITES                 0x20000U
#endif
#if !defined (SW_CFG_NUM_READS_WRITES_FAST_START)
#define SW_CFG_NUM_READS_WRITES_FAST_START       0x2000U
#endif
/*
 * what test patterns to write/read on start-up
 * */
#if !defined (SW_CONFIG_PATTERN)
#define SW_CONFIG_PATTERN (PATTERN_INCREMENTAL|\
                                        PATTERN_WALKING_ONE|\
                                        PATTERN_WALKING_ZERO|\
                                        PATTERN_RANDOM|\
                                        PATTERN_0xCCCCCCCC|\
                                        PATTERN_0x55555555)
#endif
#if !defined (SW_CONFIG_PATTERN_FAST_START)
#define SW_CONFIG_PATTERN_FAST_START (PATTERN_INCREMENTAL|\
                                        PATTERN_WALKING_ZERO)
#endif

#if !defined (LIBERO_FAST_START)
#define LIBERO_FAST_START     0U
#endif

/*
 * Default order of ADDCMD clk pushes
 * These will be overwritten when supported in Libero
 * Currently (2022.03) if change of order required, define in mss_sw_config.h
 * Define in mss_sw_config.h will take precedence
 */
#if (LIBERO_SETTING_DDR_CLK == DDR_1600_MHZ)
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3                   0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3L                  0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR4                   0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR3                 2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR4                 1U
#endif

#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3                  1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3L                 1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR4                  1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR3                0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR4                2U
#endif

#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3                  2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3L                 2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR4                  2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR3                1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR4                0U
#endif
#else
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3                   0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR3L                  0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_DDR4                   0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR3                 2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_0_DEG_LPDDR4                 0U
#endif

#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3                  1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR3L                 1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_DDR4                  1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR3                0U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_45_DEG_LPDDR4                1U
#endif

#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3                  2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3L
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR3L                 2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_DDR4                  2U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR3
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR3                1U
#endif
#ifndef LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR4
#define LIBERO_SETTING_ADD_CMD_CLK_MOVE_ORDER_90_DEG_LPDDR4                2U
#endif
#endif

/*
 * Sweep offsets
 * They currently are not coming from MSS Configurator (v12.7 and earlier)
 * They may at some point
 *
 * Determined ( 5th Feb 2021 )
 * DDR3@1066   = 3,2,1
 * DDR4@1600   = 7,0,1
 * LPDDR3@1066 = 7,0,1
 * LPDDR4@1600 = 5,4,6,3
 *
 * DDR3@1333   = 0,1      //1,7,0,2
 * DDR4@1333   = 0,7,1
 * LPDDR3@1333 = 0,1      //7,0,6
 * LPDDR4@1333 = 1,2,3
 *
 */
#if !defined (VREF_TRAINING_MIN)
#define VREF_TRAINING_MIN                               5U
#endif
#if !defined (VREF_TRAINING_MAX)
#define VREF_TRAINING_MAX                              30U
#endif
#if !defined (CA_SWEEP_START)
#define CA_SWEEP_START                                  0U
#endif
#if !defined (CA_SWEEP_END)
#define CA_SWEEP_END                                   30U
#endif
#if !defined (CA_SWEEP_INCREMENT)
#define CA_SWEEP_INCREMENT                              5U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR3_1333_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR3_1333_NUM_OFFSETS     3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1333_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR3L_1333_NUM_OFFSETS    3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1600_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR4_1600_NUM_OFFSETS     4U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1600_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_LPDDR3_1600_NUM_OFFSETS   3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1600_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_LPDDR4_1600_NUM_OFFSETS   4U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1067_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR3_1067_NUM_OFFSETS     1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1067_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR3L_1067_NUM_OFFSETS    1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1333_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_DDR4_1333_NUM_OFFSETS     4U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1333_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_LPDDR3_1333_NUM_OFFSETS   2U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1333_NUM_OFFSETS)
#define LIBERO_SETTING_REFCLK_LPDDR4_1333_NUM_OFFSETS   3U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_0        0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_1        1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_2        7U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR3_1333_OFFSET_3        0U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_0       0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_1       1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_2       0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR3L_1333_OFFSET_3       0U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_0        7U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_1        6U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_2        5U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR4_1600_OFFSET_3        0U
#endif

#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_0)
#define LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_0      7U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_1)
#define LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_1      0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_2)
#define LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_2      1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_3)
#define LIBERO_SETTING_REFCLK_LPDDR3_1600_OFFSET_3      2U
#endif
//LPDDR4@1600 = 5,4,6,3 changed to 5,4,6,2 16th Feb Alister
//AUG Offsets changed to 4324
//LPDDR4@1600 = 4,3,2,4 changed to 3,4,2,5 Feb 2022
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_0)
#define LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_0      3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_1)
#define LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_1      4U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_2)
#define LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_2      2U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_3)
#define LIBERO_SETTING_REFCLK_LPDDR4_1600_OFFSET_3      5U
#endif

/*
 * 1333 offset
 */
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_0        1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_1        2U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_2        3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR3_1067_OFFSET_3        2U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_0       1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_1       2U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_2       3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR3L_1067_OFFSET_3       2U
#endif

#if !defined (LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_0)
#define LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_0        0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_1)
#define LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_1        1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_2)
#define LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_2        6U
#endif
#if !defined (LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_3)
#define LIBERO_SETTING_REFCLK_DDR4_1333_OFFSET_3        7U
#endif

#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_0)
#define LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_0      0U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_1)
#define LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_1      1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_2)
#define LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_2      6U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_3)
#define LIBERO_SETTING_REFCLK_LPDDR3_1333_OFFSET_3      0U
#endif

#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_0)
#define LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_0      2U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_1)
#define LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_1      1U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_2)
#define LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_2      3U
#endif
#if !defined (LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_3)
#define LIBERO_SETTING_REFCLK_LPDDR4_1333_OFFSET_3      0U
#endif

#ifndef NOT_A_FULL_RETRAIN
#define NOT_A_FULL_RETRAIN
#endif

#if !defined (RPC_OVERRIDE_166_LANE_FIFO)
#define RPC_OVERRIDE_166_LANE_FIFO 0
#endif
 /*
  * 2 power size is used by the MTC. If using these in your code, the number of
  * lanes used must be subtracted from this number. x16, subtract 1, x32
  * subtract 2.
  */
#define ONE_GB_MTC      30U
#define HALF_GB_MTC     29U
#define FOUR_MB_MTC     22U
#define TWO_MB_MTC      21U
#define ONE_MB_MTC      20U


/*Cached access at 0x00_8000_0000 (-0x80+0x00) */
#define INIT_SETTING_SEG0_0    0x00007F80UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x7F80 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Cached access at 0x10_0000_000 */
#define INIT_SETTING_SEG0_1    0x00007000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x7000 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_2    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_3    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_4    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_5    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:6]  RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_6    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG0_7    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG1_0    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG1_1    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Non-Cached access at 0x00_c000_0000 */
#define INIT_SETTING_SEG1_2    0x00007F40UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x7F40 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Non-Cached access at 0x14_0000_0000 */
#define INIT_SETTING_SEG1_3    0x00006C00UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x6C00 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Non-Cached WCB access at 0x00_d000_0000 */
#define INIT_SETTING_SEG1_4    0x00007F30UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x7F30 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Non-Cached WCB 0x18_0000_0000 */
#define INIT_SETTING_SEG1_5    0x00006800UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x6800 */
    /* RESERVED                          [15:6]  RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*Trace - Trace not in use here so can be left as 0 */
#define INIT_SETTING_SEG1_6    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */
/*not used */
#define INIT_SETTING_SEG1_7    0x00000000UL
    /* ADDRESS_OFFSET                    [0:15]  RW value= 0x0 */
    /* RESERVED                          [15:16] RW value= 0x0 */
    /* LOCKED                            [31:1]  RW value= 0x0 */

#ifndef TWO_MBYTES
#define TWO_MBYTES  0x200000
#endif

/***************************************************************************//**

 */
typedef enum MTC_PATTERN_
{
    MTC_COUNTING_PATTERN            = 0x00,              /*!<  */
    MTC_WALKING_ONE                 = 0x01,              /*!<  */
    MTC_PSEUDO_RANDOM               = 0x02,              /*!<  */
    MTC_NO_REPEATING_PSEUDO_RANDOM  = 0x03,              /*!<  */
    MTC_ALT_ONES_ZEROS              = 0x04,              /*!<  */
    MTC_ALT_5_A                     = 0x05,              /*!<  */
    MTC_USER                        = 0x06,              /*!<  */
    MTC_PSEUDO_RANDOM_16BIT         = 0x07,              /*!<  */
    MTC_PSEUDO_RANDOM_8BIT          = 0x08,              /*!<  */
} MTC_PATTERN;



typedef enum MTC_ADD_PATTERN_
{
    MTC_ADD_SEQUENTIAL              = 0x00,              /*!<  */
    MTC_ADD_RANDOM                 = 0x01,               /*!<  */
} MTC_ADD_PATTERN;

/***************************************************************************//**

 */
typedef enum DDR_SM_STATES_
{
    DDR_STATE_INIT         = 0x00,     /*!< 0 DDR_STATE_INIT*/
    DDR_STATE_MONITOR      = 0x01,     /*!< 1 DDR_STATE_MONITOR */
    DDR_STATE_TRAINING     = 0x02,     /*!< 2 DDR_STATE_TRAINING */
    DDR_STATE_VERIFY       = 0x03,     /*!< 3 DDR_STATE_VERIFY */
} DDR_SM_STATES;

/***************************************************************************//**

 */
typedef enum DDR_SS_COMMAND_
{
    DDR_SS__INIT        = 0x00,     /*!< 0 DDR_SS__INIT */
    DDR_SS_MONITOR      = 0x01,     /*!< 1 DDR_SS_MONITOR */
} DDR_SS_COMMAND;


/***************************************************************************//**

 */
typedef enum DDR_SS_STATUS_
{
    DDR_SETUP_DONE      = 0x01,      /*!< 0 DDR_SETUP_DONE */
    DDR_SETUP_FAIL      = 0x02,      /*!< 1 DDR_SETUP_FAIL */
    DDR_SETUP_SUCCESS   = 0x04,      /*!< 2 DDR_SETUP_SUCCESS */
    DDR_SETUP_OFF_MODE  = 0x08,      /*!< 4 DDR_SETUP_OFF_MODE */
} DDR_SS_STATUS;


/***************************************************************************//**

 */
typedef enum DDR_TRAINING_SM_
{
    DDR_TRAINING_INIT,              /*!< DDR_TRAINING_INIT */
    DDR_TRAINING_FAIL,
    DDR_TRAINING_CHECK_FOR_OFFMODE, /*!< DDR_TRAINING_OFFMODE */
    DDR_TRAINING_SET_MODE_VS_BITS,
    DDR_TRAINING_FLASH_REGS,
    DDR_TRAINING_CORRECT_RPC,
    DDR_TRAINING_SOFT_RESET,
    DDR_TRAINING_CALIBRATE_IO,
    DDR_TRAINING_CONFIG_PLL,
    DDR_TRAINING_SETUP_SEGS,
    DDR_TRAINING_VERIFY_PLL_LOCK,
    DDR_TRAINING_SETUP_DDRC,
    DDR_TRAINING_RESET,
    DDR_TRAINING_ROTATE_CLK,
    DDR_TRAINING_SET_TRAINING_PARAMETERS,
    DDR_TRAINING_IP_SM_BCLKSCLK_SW,
    DDR_TRAINING_MANUAL_ADDCMD_TRAINING_SW,
    DDR_TRAINING_IP_SM_START,
    DDR_TRAINING_IP_SM_START_CHECK,
    DDR_TRAINING_IP_SM_BCLKSCLK,
    DDR_TRAINING_IP_SM_ADDCMD,
    DDR_TRAINING_IP_SM_WRLVL,
    DDR_TRAINING_IP_SM_RDGATE,
    DDR_TRAINING_IP_SM_DQ_DQS,
    DDR_TRAINING_IP_SM_VERIFY,
    DDR_TRAINING_SET_FINAL_MODE,
    DDR_TRAINING_WRITE_CALIBRATION,
    DDR_TRAINING_WRITE_CALIBRATION_RETRY, /*!< Retry on calibration fail */
    DDR_SANITY_CHECKS,
    DDR_FULL_MTC_CHECK,
    DDR_FULL_32BIT_NC_CHECK,
    DDR_FULL_32BIT_CACHE_CHECK,
    DDR_LOAD_PATTERN_TO_CACHE_SETUP,
    DDR_LOAD_PATTERN_TO_CACHE,
    DDR_VERIFY_PATTERN_IN_CACHE,
    DDR_FULL_32BIT_WRC_CHECK,
    DDR_FULL_64BIT_NC_CHECK,
    DDR_FULL_64BIT_CACHE_CHECK,
    DDR_FULL_64BIT_WRC_CHECK,
    DDR_TRAINING_VREFDQ_CALIB,
    DDR_TRAINING_FPGA_VREFDQ_CALIB,
    DDR_TRAINING_FINISH_CHECK,
    DDR_TRAINING_INIT_ALL_MEMORY,
    DDR_TRAINING_FINISHED,
    DDR_TRAINING_FAIL_SM2_VERIFY,
    DDR_TRAINING_FAIL_SM_VERIFY,
    DDR_TRAINING_FAIL_SM_DQ_DQS,
    DDR_TRAINING_FAIL_SM_RDGATE,
    DDR_TRAINING_FAIL_SM_WRLVL,
    DDR_TRAINING_FAIL_SM_ADDCMD,
    DDR_TRAINING_FAIL_SM_BCLKSCLK,
    DDR_TRAINING_FAIL_BCLKSCLK_SW,
    DDR_TRAINING_FAIL_FULL_32BIT_NC_CHECK,
    DDR_TRAINING_FAIL_32BIT_CACHE_CHECK,
    DDR_TRAINING_FAIL_MIN_LATENCY,
    DDR_TRAINING_FAIL_START_CHECK,
    DDR_TRAINING_FAIL_PLL_LOCK,
    DDR_TRAINING_FAIL_DDR_SANITY_CHECKS,
} DDR_TRAINING_SM;


/***************************************************************************//**

 */
typedef enum {
    USR_CMD_GET_DDR_STATUS      = 0x00,    //!< USR_CMD_GET_DDR_STATUS
    USR_CMD_GET_MODE_SETTING    = 0x01,    //!< USR_CMD_GET_MODE_SETTING
    USR_CMD_GET_W_CALIBRATION   = 0x02,    //!< USR_CMD_GET_W_CALIBRATION
    USR_CMD_GET_GREEN_ZONE      = 0x03,    //!< USR_CMD_GET_GREEN_ZONE
    USR_CMD_GET_REG             = 0x04     //!< USR_CMD_GET_REG
} DDR_USER_GET_COMMANDS_t;

/***************************************************************************//**

 */
typedef enum {
    USR_CMD_SET_GREEN_ZONE_DQ        = 0x80,   //!< USR_CMD_SET_GREEN_ZONE_DQ
    USR_CMD_SET_GREEN_ZONE_DQS       = 0x81,   //!< USR_CMD_SET_GREEN_ZONE_DQS
    USR_CMD_SET_GREEN_ZONE_VREF_MAX  = 0x82,   //!< USR_CMD_SET_GREEN_ZONE_VREF
    USR_CMD_SET_GREEN_ZONE_VREF_MIN  = 0x83,   //!< USR_CMD_SET_GREEN_ZONE_VREF
    USR_CMD_SET_RETRAIN              = 0x84,   //!< USR_CMD_SET_RETRAIN
    USR_CMD_SET_REG                  = 0x85    //!< USR_CMD_SET_REG
} DDR_USER_SET_COMMANDS_t;

/***************************************************************************//**

 */
typedef enum SWEEP_STATES_{
    INIT_SWEEP,                     //!< start the sweep
    ADDR_CMD_OFFSET_SWEEP,          //!< sweep address command
    BCLK_SCLK_OFFSET_SWEEP,         //!< sweep bclk sclk
    DPC_VRGEN_V_SWEEP,              //!< sweep vgen_v
    DPC_VRGEN_H_SWEEP,              //!< sweep vgen_h
    DPC_VRGEN_VS_SWEEP,             //!< VS sweep
    FINISHED_SWEEP,                 //!< finished sweep
} SWEEP_STATES;

/***************************************************************************//**

 */
typedef enum {
  USR_OPTION_tip_register_dump    = 0x00     //!< USR_OPTION_tip_register_dump
} USR_STATUS_OPTION_t;


#define MAX_LANES  5

/***************************************************************************//**

 */
typedef enum SEG_SETUP_{
    DEFAULT_SEG_SETUP    = 0x00,
    LIBERO_SEG_SETUP
} SEG_SETUP;

/***************************************************************************//**

 */
typedef enum DDR_LP_OPTION_{
    DDR_LOW_POWER       = 0x00U,
    DDR_NORMAL_POWER    = 0x01U
} DDR_LP_OPTION;


/***************************************************************************//**

 */
typedef struct mss_ddr_diags_{
    uint64_t    train_time;
    uint32_t    num_retrains;
    uint32_t    padding;
} mss_ddr_diag;

/***************************************************************************//**

 */
typedef struct mss_ddr_fpga_vref_{
    uint32_t    status_lower;
    uint32_t    status_upper;
    uint32_t    lower;
    uint32_t    upper;
    uint32_t    vref_result;
} mss_ddr_vref;

/**
 * \brief dll sgmii SCB regs
 */
typedef struct IOSCB_BANKCONT_DDR_ {
                                    /* bit0 - This when asserted resets all the non-volatile register bits e.g. RW-P bits, the bit self clears i.e. is similar to a W1P bit */
                                    /* bit1 - This when asserted resets all the register bits apart from the non-volatile registers, the bit self clears. i.e. is similar to a W1P bit */
    __IO uint32_t soft_reset;       /* bit8 - This asserts the functional reset of the block. It is asserted at power up. When written is stays asserted until written to 0.       */

    __IO uint32_t dpc_bits;         /* bit 3:0:  dpc_vs   bank voltage select for pvt calibration             */
                                    /*  :  dpc_vrgen_h                  */
                                    /*  :  dpc_vrgen_en_h               */
                                    /*  :  dpc_move_en_h                */
                                    /*  :  dpc_vrgen_v                  */
                                    /*  :  dpc_vrgen_en_v               */
                                    /*  :  dpc_move_en_v                */
    __IO uint32_t bank_status;      /* bit 0: Bank power on complete (active low for polling)                */
                                    /* bit 1: Bank calibration complete (active low for polling)             */
} IOSCB_BANKCONT_DDR_STRUCT;


#define IOSCB_BANKCONT_DDR_BASE  0x3E020000UL
#define IOSCB_BANKCONT_DDR  ((volatile IOSCB_BANKCONT_DDR_STRUCT *) IOSCB_BANKCONT_DDR_BASE)

/***************************************************************************//**

 */
typedef struct mss_ddr_write_calibration_{
    uint32_t    status_lower;
    uint32_t    lower[MAX_LANES];
    uint32_t    lane_calib_result;
} mss_ddr_write_calibration;

/***************************************************************************//**

 */
typedef struct mss_lpddr4_dq_calibration_{
    uint32_t    lower[MAX_LANES];
    uint32_t    upper[MAX_LANES];
    uint32_t    calibration_found[MAX_LANES];
} mss_lpddr4_dq_calibration;


/***************************************************************************//**
  Calibration settings derived during write training
 */
typedef struct mss_ddr_calibration_{
  /* CMSIS related defines identifying the UART hardware. */
    mss_ddr_write_calibration write_cal;
    mss_lpddr4_dq_calibration dq_cal;
    mss_ddr_vref fpga_vref;
    mss_ddr_vref mem_vref;
} mss_ddr_calibration;

/***************************************************************************//**
  sweep index's
 */
typedef struct sweep_index_{
    uint8_t cmd_index;
    uint8_t bclk_sclk_index;
    uint8_t dpc_vgen_index;
    uint8_t dpc_vgen_h_index;
    uint8_t dpc_vgen_vs_index;
} sweep_index;

/***************************************************************************//**

 */
uint8_t MSS_DDR_init_simulation(void);

/***************************************************************************//**

 */
uint8_t MSS_DDR_training(uint8_t ddr_type);


/***************************************************************************//**
  The ddr_state_machine() function runs a state machine which initializes and
  monitors the DDR

  @return
    This function returns status, see DDR_SS_STATUS enum

  Example:
  @code

        uint32_t  ddr_status;
        ddr_status = ddr_state_machine(DDR_SS__INIT);

        while((ddr_status & DDR_SETUP_DONE) != DDR_SETUP_DONE)
        {
            ddr_status = ddr_state_machine(DDR_SS_MONITOR);
        }
        if ((ddr_status & DDR_SETUP_FAIL) != DDR_SETUP_FAIL)
        {
            error |= (0x1U << 2U);
        }

  @endcode

 */
uint32_t ddr_state_machine(DDR_SS_COMMAND command);

/***************************************************************************//**
  The setup_ddr_segments() sets up seg regs

  @return
    none

  Example:
  @code

      setup_ddr_segments(DEFAULT_SEG_SETUP);

  @endcode

 */
void setup_ddr_segments(SEG_SETUP option);

/***************************************************************************//**
  The clear_bootup_cache_ways() sets up seg regs

  @return
    none

  Example:
  @code

      clear_bootup_cache_ways(DEFAULT_SEG_SETUP);

  @endcode

 */
void clear_bootup_cache_ways(void);

/***************************************************************************//**
  The fill_cache_new_seg_address()

  @return
    none

  Example:
  @code

      fill_cache_new_seg_address(DEFAULT_SEG_SETUP);

  @endcode

 */
char * fill_cache_new_seg_address(void *dest, void *dest_end);

/***************************************************************************//**
  The mpfs_hal_turn_ddr_selfrefresh_on(void) flushes cache and turns on self
  refresh. When DDR is in self refresh mode, less power is consumed by the
  memory. Data is retained. You can not write or read from the DDR when
  self-refresh is on.


  @return
    none

  Example:
  @code

      mpfs_hal_turn_ddr_selfrefresh_on(DEFAULT_SEG_SETUP);

  @endcode

 */
void mpfs_hal_turn_ddr_selfrefresh_on(void);

/***************************************************************************//**
  The mpfs_hal_turn_ddr_selfrefresh_off()

  @return
    none

  Example:
  @code

      mpfs_hal_turn_ddr_selfrefresh_off();

  @endcode

 */
void mpfs_hal_turn_ddr_selfrefresh_off(void);

/***************************************************************************//**
  The mpfs_hal_ddr_selfrefresh_status()

  @return
    none

  Example:
  @code

      status = mpfs_hal_ddr_selfrefresh_status();

      if(status != 0U)
      {
          printf("self refresh is on\n");
      }

  @endcode

 */
uint32_t mpfs_hal_ddr_selfrefresh_status(void);

/***************************************************************************//**
  The mpfs_hal_ddr_logic_power_state(uint32_t lp_state, uint32_t lp_options)

  @param lp_state 0 => low power, 1 => normal

  @param lp_options bit 0 => pll_outputs
                    bit 1 => addcmd_pins
                    bit 2 => clk_pin
                    bit 3 => dq_pins
                    bit 4 => dqs_pins
                    bit 5 => dqs_pins
                    bit 6 => odt

  @return
    none

  Example:
  @code
      lp_state = 0U;        // low power state
      lp_options = 0x7FU;   // All options chosen
      mpfs_hal_sw_ddr_power_state(lp_state, lp_options);

      if(status != 0U)
      {
          printf("self refresh is on\n");
      }

  @endcode

 */
void mpfs_hal_ddr_logic_power_state(uint32_t lp_state, uint32_t lp_options);


#ifdef __cplusplus
}
#endif

#endif /* __MSS_DDRC_H_ */


