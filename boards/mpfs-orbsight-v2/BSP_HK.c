/** ----------------------------------------------------------------------------
 * @file        BSP_HK.c
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *
 * @date        February - March 2026
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */

/** Remap asm to __asm__ for strict ISO C compliance                          */
#ifndef asm
#define asm __asm__
#endif

#ifdef __cplusplus
extern "C" {
    #include <cstdint.h>
#else
    #include <stdint.h>
#endif

#include "BSP_Config.h"

typedef enum {
    BSP_VOLTAGE_OK                  = 0x000u,
    BSP_VOLTAGE_V1_BELLOW_LIMIT     = 0x001u,
    BSP_VOLTAGE_V1_ABOVE_LIMIT      = 0x002u,
    BSP_VOLTAGE_V18_BELLOW_LIMIT    = 0x004u,
    BSP_VOLTAGE_V18_ABOVE_LIMIT     = 0x008u,
    BSP_VOLTAGE_V25_BELLOW_LIMIT    = 0x010u,
    BSP_VOLTAGE_V25_ABOVE_LIMIT     = 0x020u,
    BSP_VOLTAGE_MASK_MAX_SHIFT      = 5u     /// Keep this one consistent and
                                             /// accurate -- used only in debug
                                             /// prints, not in flight code
} bsp_hk_voltage_state_mask_t;

typedef enum {
    BSP_TEMPERATURE_OK                      = 0x000u,
    BSP_TEMPERATURE_IMAGE_BELLOW_LIMIT      = 0x001u,
    BSP_TEMPERATURE_IMAGE_ABOVE_LIMIT       = 0x002u,
    BSP_TEMPERATURE_FPGA_BELLOW_LIMIT       = 0x004u,
    BSP_TEMPERATURE_FPGA_ABOVE_LIMIT        = 0x008u,
    BSP_TEMPERATURE_MASK_MAX_SHIFT          = 3u    /// Keep this one consistent
                                                    /// and accurate -- used
                                                    /// only in debug prints,
                                                    /// not in flight code
} bsp_hk_temperature_state_mask_t;

typedef enum {
    BSP_CURRENT_AND_VOLTAGE_OK              = 0x000u,
    BSP_CURRENT_BELLOW_LIMIT                = 0x001u,
    BSP_CURRENT_ABOVE_LIMIT                 = 0x002u,
    BSP_VOLTAGE_BELLOW_LIMIT                = 0x004u,
    BSP_VOLTAGE_ABOVE_LIMIT                 = 0x008u,
    BSP_UNKNOWN                             = 0x010u,
    BSP_EMMC_MASK_MAX_SHIFT                 = 4u    /// Keep this one consistent
                                                    /// and accurate -- used
                                                    /// only in debug prints,
                                                    /// not in flight code
} bsp_hk_emmc_state_mask_t;

typedef enum {
    BSP_OP_LESS       = -1,
    BSP_OP_EQUAL      = 0,
    BSP_OP_GREATER    = 1
} bsp_hk_comp_op_t;

#ifdef BSP_HSS_BUILD
    #include "hal/hal.h"
    #include "mpfs_hal/mss_hal.h"
    #include "mpfs_hal/mpfs_hal_version.h"
    #include "mpfs_hal/common/nwc/mss_nwc_init.h"

    #include "drivers/mss/mss_gpio/mss_gpio.h"
    #include "drivers/mss/mss_i2c/mss_i2c.h"
    #include "drivers/off_chip/pac1934/pac1934.h"

    #include "BSP_HK.h"
    #include "BSP_Regs.h"
    #include "BSP_HK_Observer.h"

#else /// BSP_HSS_BUILD
    #include <hal/hal.h>
    #include <mpfs_hal/mss_hal.h>
    #include <mpfs_hal/mpfs_hal_version.h>
    #include <mpfs_hal/common/nwc/mss_nwc_init.h>

    #include <drivers/mss/mss_gpio/mss_gpio.h>

    #include "bsp/BSP_HK.h"
    #include "bsp/BSP_Regs.h"
    #include "bsp/BSP_HK_Observer.h"

    #include <stdbool.h>

    #include <drivers/mss/mss_i2c/mss_i2c.h>
    #include <drivers/off_chip/pac1934/pac1934.h>
#endif

#ifdef DEBUG_PRINT

    #include <stdio.h>
    #include <inttypes.h>

    #ifndef BSP_HSS_BUILD
        #include "drivers/mss/mss_mmuart/mss_uart.h"
        extern mss_uart_instance_t*         g_uart;

        static uint8_t                      g_ui_buf[256+1]                 \
                __attribute__ ((aligned (4))) = {0};
    #else
        #include "hss_debug.h"
    #endif

    static const char* hk_state_codes[] = {
        "BSP_HK_CAMERA_POWER_ON",
        "BSP_HK_VDD3V5_POWER_GOOD",
        "BSP_HK_VDD2V0_POWER_GOOD",
        "BSP_HK_VDD3V8_POWER_GOOD",
        "BSP_HK_VDD0V8_POWER_GOOD",
        "BSP_HK_VDD3V5_VDD2V0_POWER_GOOD",
        "BSP_HK_ALL_POWER_GOOD",
        "BSP_HK_ERROR",
        "BSP_HK_POWER_GOOD",
        "BSP_HK_UNKNOWN"
        "BSP_HK_VOLTAGE_FAILURE",
        "BSP_HK_TEMPERATURE_FAILURE",
        "BSP_HK_EMMC_PR_CURRENT_FAILURE",
        "BSP_HK_EMMC_PR_VOLTAGE_FAILURE",
        "BSP_HK_EMMC_SC_CURRENT_FAILURE",
        "BSP_HK_EMMC_SC_VOLTAGE_FAILURE",
        "BSP_HK_OK"
    };

    static const char* hk_event_codes[] = {
        "BSP_EVT_NONE",
        "BSP_EVT_TVS_TEMP_LOW",
        "BSP_EVT_TVS_TEMP_HIGH",
        "BSP_EVT_TVS_VOLT_LOW",
        "BSP_EVT_TVS_VOLT_HIGH",
        "BSP_EVT_EMMC_PR_CURRENT_LOW",
        "BSP_EVT_EMMC_PR_CURRENT_HIGH",
        "BSP_EVT_EMMC_PR_VOLT_LOW",
        "BSP_EVT_EMMC_PR_VOLT_HIGH",
        "BSP_EVT_EMMC_SC_CURRENT_LOW",
        "BSP_EVT_EMMC_SC_CURRENT_HIGH",
        "BSP_EVT_EMMC_SC_VOLT_LOW",
        "BSP_EVT_EMMC_SC_VOLT_HIGH",
    };

    static void print(uint8_t* msg);
    static void print(uint8_t* msg)
    {
        #ifdef BSP_HSS_BUILD
            /// sbi_puts((const char*)msg);
            sbi_printf(" %s", (const char*)msg);
        #else
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)msg);
        #endif
    }

    static const char* hk_voltage_state_mask_codes[] = {
        "BSP_VOLTAGE_OK",
        "BSP_VOLTAGE_V1_BELLOW_LIMIT",
        "BSP_VOLTAGE_V1_ABOVE_LIMIT",
        "BSP_VOLTAGE_V18_BELLOW_LIMIT",
        "BSP_VOLTAGE_V18_ABOVE_LIMIT",
        "BSP_VOLTAGE_V25_BELLOW_LIMIT",
        "BSP_VOLTAGE_V25_ABOVE_LIMIT"
    };

    static const char* hk_temperature_state_mask_codes[] = {
        "BSP_TEMPERATURE_OK",
        "BSP_TEMPERATURE_IMAGE_BELLOW_LIMIT",
        "BSP_TEMPERATURE_IMAGE_ABOVE_LIMIT",
        "BSP_TEMPERATURE_FPGA_BELLOW_LIMIT",
        "BSP_TEMPERATURE_FPGA_ABOVE_LIMIT"
    };

    static const char* hk_emmc_state_mask_codes[] = {
        "BSP_CURRENT_AND_VOLTAGE_OK",
        "BSP_CURRENT_BELLOW_LIMIT",
        "BSP_CURRENT_ABOVE_LIMIT",
        "BSP_VOLTAGE_BELLOW_LIMIT",
        "BSP_VOLTAGE_ABOVE_LIMIT",
        "BSP_UNKNOWN"
    };

    #ifndef BSP_HSS_BUILD
        static void print_message(const char* message)
        {
            #ifdef BSP_HSS_BUILD
                sbi_printf(" %s", message);
            #else
                MSS_UART_polled_tx_string(g_uart, (uint8_t*)message);
            #endif
        }
    #endif

    static void print_reference_value(const char* label, int32_t value)
    {
        #ifdef BSP_HSS_BUILD
            mHSS_FANCY_PRINTF(LOG_NORMAL, " %s %d", label, value);
        #else
            uint8_t msg[64];
            sprintf((char*)msg, " %s %d\n\r", label, value);
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)msg);
        #endif
    }

    static void print_voltage_status(uint32_t status)
    {
        #ifdef BSP_HSS_BUILD
            mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                            \
                    (const char*)"\n\r Voltage status: ");
        #else
            MSS_UART_polled_tx_string(g_uart,
                    (uint8_t*)"\n\r Voltage status: ");
        #endif
        if (status == 0u)
        {
            #ifdef BSP_HSS_BUILD
                mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                        \
                        (const char*)"\n\r    BSP_VOLTAGE_OK");
            #else
                MSS_UART_polled_tx_string(g_uart,                           \
                        (uint8_t*)"\n\r    BSP_VOLTAGE_OK");
            #endif
        }
        else
        {
            size_t num_masks = BSP_VOLTAGE_MASK_MAX_SHIFT;

            for (uint16_t i = 0; i <= num_masks; ++i)
            {
                if ((status & (1u << i)) == (1u << i))
                {
                    #ifdef BSP_HSS_BUILD
                    mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                \
                                (const char*)hk_voltage_state_mask_codes[i+1]);
                    #else
                        MSS_UART_polled_tx_string(g_uart, (uint8_t*)"\n\r    ");
                        MSS_UART_polled_tx_string(g_uart,                   \
                                (uint8_t*)hk_voltage_state_mask_codes[i+1]);
                    #endif
                }
            }
        }
        #ifndef BSP_HSS_BUILD
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)" \n\r");
        #else
            sbi_printf(" %s", "\n\r ");
        #endif
    }

    static void print_temperature_status(uint32_t status)
    {
        #ifdef BSP_HSS_BUILD
        mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                            \
                    (const char*)"\n\r Temperature status: ");
        #else
            MSS_UART_polled_tx_string(g_uart,
                    (uint8_t*)"\n\r Temperature status: ");
        #endif
        if (status == 0u)
        {
            #ifdef BSP_HSS_BUILD
            mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                        \
                        (const char*)"\n\r    BSP_TEMPERATURE_OK");
            #else
                MSS_UART_polled_tx_string(g_uart,                           \
                        (uint8_t*)"\n\r    BSP_TEMPERATURE_OK");
            #endif
        }
        else
        {
            size_t num_masks = BSP_TEMPERATURE_MASK_MAX_SHIFT;

            for (uint16_t i = 0; i <= num_masks; ++i)
            {
                if ((status & (1u << i)) == (1u << i))
                {
                    #ifdef BSP_HSS_BUILD
                    mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                \
                             (const char*)hk_temperature_state_mask_codes[i+1]);
                    #else
                        MSS_UART_polled_tx_string(g_uart, (uint8_t*)"\n\r    ");
                        MSS_UART_polled_tx_string(g_uart,                   \
                               (uint8_t*)hk_temperature_state_mask_codes[i+1]);
                    #endif
                }
            }
        }
        #ifndef BSP_HSS_BUILD
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)" \n\r");
        #else
            sbi_printf(" %s", "\n\r ");
        #endif
    }

    static void print_emmc_status_labeled(const char* label, uint32_t status)
    {
        #ifdef BSP_HSS_BUILD
        mHSS_FANCY_PRINTF(LOG_NORMAL, "\n\r %s",                            \
                    (const char*)label);
        #else
            sprintf((char*)g_ui_buf, "\n\r %s", label);
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)g_ui_buf);
        #endif
        if (status == 0u)
        {
            #ifdef BSP_HSS_BUILD
            mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                        \
                        (const char*)"\n\r    BSP_CURRENT_AND_VOLTAGE_OK");
            #else
                MSS_UART_polled_tx_string(g_uart,                           \
                        (uint8_t*)"\n\r    BSP_CURRENT_AND_VOLTAGE_OK");
            #endif
        }
        else
        {
            size_t num_masks = BSP_EMMC_MASK_MAX_SHIFT;

            for (uint16_t i = 0; i <= num_masks; ++i)
            {
                if ((status & (1u << i)) == (1u << i))
                {
                    #ifdef BSP_HSS_BUILD
                    mHSS_FANCY_PRINTF(LOG_NORMAL, " %s",                \
                             (const char*)hk_emmc_state_mask_codes[i+1]);
                    #else
                        MSS_UART_polled_tx_string(g_uart, (uint8_t*)"\n\r    ");
                        MSS_UART_polled_tx_string(g_uart,                   \
                               (uint8_t*)hk_emmc_state_mask_codes[i+1]);
                    #endif
                }
            }
        }
        #ifndef BSP_HSS_BUILD
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)" \n\r");
        #else
            sbi_printf(" %s", "\n\r ");
        #endif
    }

    static void print_emmc_status(                                          \
                    uint32_t emmc_pr_status, uint32_t emmc_sc_status)
    {
        print_emmc_status_labeled("eMMC Primary status: ", emmc_pr_status);
        print_emmc_status_labeled("eMMC Secondary status: ", emmc_sc_status);
    }

    static void print_f1(const char* format, int32_t value)
    {
    #ifdef BSP_HSS_BUILD

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
        sbi_printf(" %s %d\n\r", format, value);
    #pragma GCC diagnostic pop
    #else
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
        sprintf((char*)g_ui_buf, format, value);
    #pragma GCC diagnostic pop
        MSS_UART_polled_tx_string(g_uart, g_ui_buf);
        MSS_UART_polled_tx_string(g_uart, (uint8_t*)"\r\n");
    #endif
    }

#endif

/** ----------------------------------------------------------------------------
 *  Util functions
 *  ----------------------------------------------------------------------------
*/

/**
 * TVS Temperature Channel Decoder (no floating point)
 *
 * Raw 16-bit format:
 *   Bit 15:    Reserved
 *   Bits[14:4]: Integer part of temperature in Kelvin
 *   Bits[3:0]:  Fractional part of temperature in Kelvin (1/16 resolution)
 *
 * Returns temperature in milli-Celsius (divide by 1000 for Celsius).
 * Example: 0x133B -> 307.5625 K -> 34562 m°C (34.562 °C)
 */
static int32_t
tvs_decode_temp_milli_celsius(uint16_t raw, const char* label);

/**
 * TVS Voltage Channel Decoder (no floating point)
 *
 * Raw 16-bit format (Table 1-3):
 *   Bit 15:     Sign bit (1 = negative voltage)
 *   Bits[14:3]: Integer part of voltage in mV
 *   Bits[2:0]:  Fractional part of voltage in mV (1/8 resolution)
 *
 * Returns voltage in micro-volts (uV) as int32_t.
 * Divide by 1000 for milli-volts, by 1000000 for volts.
 *
 * Example: 0x385E -> 1803.75 mV -> 1803750 uV
 */
static int32_t
tvs_decode_voltage_micro_volts(uint16_t raw, const char* label);

static uint32_t
read_voltages(bool *voltage_low, bool *voltage_high);

static uint32_t
read_temperatures(bool *temperature_low, bool *temperature_high);

struct i2c_status_t
{
    bool current_pr_low;
    bool current_pr_high;
    bool current_sc_low;
    bool current_sc_high;

    bool voltage_pr_low;
    bool voltage_pr_high;
    bool voltage_sc_low;
    bool voltage_sc_high;
};
static void
read_i2c(struct i2c_status_t* i2c_status,                                   \
        uint32_t* emmc_pr_status, uint32_t* emmc_sc_status);

static bool
value_compare_by_percentage(bsp_hk_comp_op_t op, int32_t value,                \
        uint16_t percentage, int32_t ref);

static bool
value_compare_by_limit(bsp_hk_comp_op_t op, int32_t value, int32_t limit);
/** ----------------------------------------------------------------------------
/// Instance of the Camera Subject -- Part of the Event engine --
/// see BSP_HK_Observer.h for details -- Initialized in the HK PG init, when the
/// camera is turned on, to be observed for events by the Event engine
*/
bsp_subject_t g_camera_subject;

/** ----------------------------------------------------------------------------
 * @brief   Observer callbacks - Events
 * -----------------------------------------------------------------------------
*/
/// Observer callback for Voltage and Temperature failure --
/// it turns the camera off
///
void
on_voltage_temperature_failure(const bsp_event_t *event, void *ctx);

/** -----------------------------------------------------------------------------
 * @brief   HK Action callbacks - Events
 * -----------------------------------------------------------------------------
*/
bsp_hk_state_t
g_event_pass_thru(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_null(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_camera_power_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_camera_power_off(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V5_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V5_off(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD2V0_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD2V0_off(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V8_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V8_off(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD0V8_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD0V8_off(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V5_VDD2V0_on(void *user_data, bsp_hk_state_t state);

bsp_hk_state_t
g_event_is_VDD3V5_VDD2V0_off(void *user_data, bsp_hk_state_t state);



/** ----------------------------------------------------------------------------
 * @brief   Actions
 * -----------------------------------------------------------------------------
*/
bsp_hk_state_t
g_action_camera_power_on(void *user_data, bsp_hk_state_t state,             \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_VDD3V5_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_VDD2V0_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_VDD3V8_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_VDD0V8_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_ALL_power_good(void *user_data, bsp_hk_state_t state,              \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_VDD3V5_VDD2V0_power_good(void *user_data, bsp_hk_state_t state,    \
        event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_action_error(void *user_data, bsp_hk_state_t state,                       \
            event_cb true_event, event_cb false_event);

bsp_hk_state_t
g_event_pass_thru(void *user_data, bsp_hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
      #ifndef BSP_HSS_BUILD
        sprintf((char*)g_ui_buf," PG state check: %s\n\r",                  \
                hk_state_codes[state]);
        print( (uint8_t*)g_ui_buf);
      #else
        /// sbi_printf(" PG state check: %s\n\r", hk_state_codes[state]);
        sbi_printf("  PG state check: %s\n\r",             \
                hk_state_codes[state]);
      #endif
    #endif

    return state;
}

bsp_hk_state_t
g_event_null(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_camera_power_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_camera_power_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V5_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V5_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD2V0_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD2V0_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V8_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V8_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD0V8_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD0V8_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V5_VDD2V0_on(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}

bsp_hk_state_t
g_event_is_VDD3V5_VDD2V0_off(void *user_data, bsp_hk_state_t state)
{
    return g_event_pass_thru(user_data, state);
}


/** ----------------------------------------------------------------------------
 * @brief   Actions
 * -----------------------------------------------------------------------------
*/
bsp_hk_state_t
g_action_camera_power_on(void *user_data, bsp_hk_state_t state,             \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;
    (void)false_event;

    mss_enable_fabric();

    MSS_GPIO_init(GPIO2_LO);

    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_8, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_9, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_10, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_11, MSS_GPIO_INPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_12, MSS_GPIO_INPUT_MODE);

    #ifdef DEBUG_PRINT
        print(                                  \
                (uint8_t*)" PG action: Camera Power On\n\r");
    #endif

    /// Init the Camera s subject to be observed for events by the
    /// Event engine
    (void)BSP_subject_init(&g_camera_subject);

    /// Subscribe to these events
    ///
    (void)BSP_subject_subscribe(&g_camera_subject,                          \
            on_voltage_temperature_failure,                                 \
            user_data);

    /// Launch the powering sequence chain
    ///
    return true_event(user_data, BSP_HK_VDD3V5_POWER_GOOD);
}

bsp_hk_state_t
g_action_VDD3V5_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: VDD3V5 PG MSS_GPIO_10\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_10, 1 );
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V5_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_12_MASK) >> 12u);
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P3V5_PG, ~pin);

    return pin == 1u ?
        true_event(user_data, BSP_HK_VDD2V0_POWER_GOOD) :
        false_event(user_data, BSP_HK_ERROR);
}

bsp_hk_state_t
g_action_VDD2V0_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: VDD2V0 PG MSS_GPIO_8\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_8, 1 );
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_2V0_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P2V0_PG, ~pin);

    return pin == 1u ?
        true_event(user_data, BSP_HK_VDD3V8_POWER_GOOD) :
        false_event(user_data, BSP_HK_ERROR);
}

bsp_hk_state_t
g_action_VDD3V8_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;
    (void)false_event;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: VDD3V8 PG MSS_GPIO_9\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_9, 1 );
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V8_EN, 1u);

    return true_event(user_data, BSP_HK_VDD0V8_POWER_GOOD);
}



bsp_hk_state_t
g_action_VDD0V8_power_good(void *user_data, bsp_hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;
    (void)false_event;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: VDD0V8 PG MSS_GPIO_7\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_7, 1 );
    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_0V8_EN, 1u);

    return true_event(user_data, BSP_HK_VDD3V5_VDD2V0_POWER_GOOD);
}

bsp_hk_state_t
g_action_VDD3V5_VDD2V0_power_good(void *user_data, bsp_hk_state_t state,    \
        event_cb true_event, event_cb false_event)
{
    (void)state;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: VDD3V5 and VDD2V0 PG Check\n\r");
    #endif

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin11 = (uint32_t)((gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
    uint32_t pin12 = (uint32_t)((gpio_inputs & MSS_GPIO_12_MASK) >> 12u);

    return pin11 == 1u && pin12 == 1u ?
            true_event(user_data, BSP_HK_ALL_POWER_GOOD) :
            false_event(user_data, BSP_HK_ERROR);
}

bsp_hk_state_t
g_action_ALL_power_good(void *user_data, bsp_hk_state_t state,              \
        event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    (void)true_event;
    (void)false_event;

    #ifdef DEBUG_PRINT
        print(                                  \
                (uint8_t*)" PG action: SNS_ENABLE\n\r");
    #endif

    #ifdef DEBUG_PRINT
        uint32_t gpio_inputs;
        gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);
      #ifndef BSP_HSS_BUILD
        sprintf((char*)g_ui_buf, " PG pin 11: %d\r\n",                       \
                (uint32_t)(gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
        print( (uint8_t*)g_ui_buf);
        sprintf((char*)g_ui_buf, " PG pin 12: %d\r\n",                       \
                (uint32_t)(gpio_inputs & MSS_GPIO_12_MASK) >> 12u);;
        print( (uint8_t*)g_ui_buf);
      #else
        sbi_printf(" PG pin 11: %d\n\r",                 \
                (uint32_t)(gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
        sbi_printf(" PG pin 12: %d\n\r",                 \
                (uint32_t)(gpio_inputs & MSS_GPIO_12_MASK) >> 12u);
      #endif
    #else
        (void)gpio_inputs;
    #endif

    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, SNS_ENABLE, 1u);
    ///HAL_set_32bit_reg(BASE32_ADDR_MSS_BSPREG, SENSOR_CTRL, 1U);

    delay(DELAY_CYCLES_100MS * 40u);

    return BSP_HK_POWER_GOOD;
}

bsp_hk_state_t
g_action_error(void *user_data, bsp_hk_state_t state,                       \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    (void)true_event;
    (void)false_event;

    #ifdef DEBUG_PRINT
        print(                                  \
                    (uint8_t*)" PG action: Error, Shutting down\n\r");
    #endif

    /// Graceful shutdown
    ///
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_7, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_9, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_8, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_10, 0 );

    ///HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, SNS_ENABLE, 0);

    return BSP_HK_UNKNOWN;
}

/** ----------------------------------------------------------------------------
 * @note    Handy structs as building blocks for the state machine
 * -----------------------------------------------------------------------------
*/
struct inner_t
{
    event_cb true_event_cb;
    event_cb false_event_cb;
};

struct outer_t
{
    action_cb       condition_action_cb;
    struct inner_t  inner;
};

/** ----------------------------------------------------------------------------
 * @note    This is the only place to extend the state machine by simply and
 *          only defining/adding new actions and events
 *
 * @note    The order of the defined/added actions is important! It has to
 *          correspond to bsp_hk_state_t enums
 * -----------------------------------------------------------------------------
*/

struct outer_t state_machine[] =
{
        {g_action_camera_power_on,
                {g_event_is_camera_power_on,
                 g_event_is_camera_power_off}
        },
        {g_action_VDD3V5_power_good,
                {g_event_is_VDD3V5_on,
                 g_event_is_VDD3V5_on}
        },
        {g_action_VDD2V0_power_good,
                 {g_event_is_VDD2V0_on,
                  g_event_is_VDD2V0_off}
        },
        {g_action_VDD3V8_power_good,
                {g_event_is_VDD3V8_on,
                 g_event_is_VDD3V8_off}
        },
        {g_action_VDD0V8_power_good,
                {g_event_is_VDD0V8_on,
                 g_event_is_VDD0V8_off}
        },
        {g_action_VDD3V5_VDD2V0_power_good,
                {g_event_is_VDD3V5_VDD2V0_on,
                 g_event_is_VDD3V5_VDD2V0_off}
        },
        {g_action_ALL_power_good,
                {g_event_null,
                 g_event_null}
        },
        {g_action_error,
                {g_event_null,
                 g_event_null}
        }
};

/** ----------------------------------------------------------------------------
 * @brief                   Start of the Camera HK state machine
 *
 * @param[in] user_data     Any user data to be passed onto the state machine
 *
 * @return                  State of the state machine after applying all the
 *                          actions
 *
 * -----------------------------------------------------------------------------
*/
bsp_hk_state_t
BSP_HK_SM_Run(void* user_data)
{
    bsp_hk_state_t state = BSP_HK_CAMERA_POWER_ON;

    while (state != BSP_HK_POWER_GOOD && state != BSP_HK_UNKNOWN)
    {
        /// Sanity check
        ///
        if (state < BSP_HK_CAMERA_POWER_ON && state > BSP_HK_UNKNOWN)
            return BSP_HK_UNKNOWN;

        struct outer_t outer = state_machine[state];
        struct inner_t inner = outer.inner;

        action_cb condition_action = outer.condition_action_cb;
        state = condition_action(user_data, state,                          \
                inner.true_event_cb, inner.false_event_cb);

    }
    return state;
}

/// Observer callback for Voltage and Temperature failure --
/// it turns the camera off
///
void
on_voltage_temperature_failure(const bsp_event_t *event, void *ctx)
{
    if (BSP_EVT_TVS_TEMP_LOW != event->type &&
        BSP_EVT_TVS_TEMP_HIGH != event->type &&
        BSP_EVT_TVS_VOLT_LOW != event->type &&
        BSP_EVT_TVS_VOLT_HIGH != event->type)
    {
        return;
    }

    switch (event->type)
    {
    case BSP_EVT_TVS_VOLT_LOW:
    case BSP_EVT_TVS_VOLT_HIGH:
        #ifdef DEBUG_PRINT
            print((uint8_t*)" HK EVT action: Voltage failure, "             \
                    "Shutting down\n\r");
        #endif

        (void)g_action_error(ctx, BSP_HK_VOLTAGE_FAILURE, 0, 0);
        break;
    case BSP_EVT_TVS_TEMP_LOW:
    case BSP_EVT_TVS_TEMP_HIGH:
        #ifdef DEBUG_PRINT
            print((uint8_t*)" HK EVT action: Temperature failure, "         \
                    "Shutting down\n\r");
        #endif

        (void)g_action_error(ctx, BSP_HK_TEMPERATURE_FAILURE, 0, 0);
        break;
    default:
        break;
    }
}

static uint32_t
read_voltages(bool *volt_low, bool *volt_high)
{
    *volt_low = false;
    *volt_high = false;

    /// Read voltages
    ///
    uint32_t tvs_v18 = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V18);
    uint32_t tvs_v1  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V1);
    uint32_t tvs_v25 = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V25);

    /// Convert to milli_voltages
    ///
    int32_t tvs_v18_mV = tvs_decode_voltage_micro_volts(tvs_v18 & 0xFFFF,   \
            " TVS_V18: ") / 1000;
    int32_t tvs_v1_mV = tvs_decode_voltage_micro_volts(tvs_v1 & 0xFFFF,     \
            " TVS_V1:  ") / 1000;
    int32_t tvs_v25_mV = tvs_decode_voltage_micro_volts(tvs_v25 & 0xFFFF,   \
            " TVS_V25: ") / 1000;

    uint32_t voltage_status = BSP_VOLTAGE_OK;

    /// 5% threshold from the monitored values
    ///
    if (value_compare_by_percentage(BSP_OP_LESS, tvs_v1_mV,                 \
            95, BSP_TVS_MONITORED_VOLTAGE_V1))
    {
        *volt_low = true;
        voltage_status |= BSP_VOLTAGE_V1_BELLOW_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V1 Lower limit: ",                   \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V1 * 95) / 100));
        #endif
    }
    ///
    if (value_compare_by_percentage(BSP_OP_LESS, tvs_v18_mV,                \
            95, BSP_TVS_MONITORED_VOLTAGE_V18))
    {
        *volt_low = true;
        voltage_status |= BSP_VOLTAGE_V18_BELLOW_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V1 Lower limit: ",                   \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V18 * 95) / 100));
        #endif

    }
    if (value_compare_by_percentage(BSP_OP_LESS, tvs_v25_mV,                \
                95, BSP_TVS_MONITORED_VOLTAGE_V25))
    {
        *volt_low = true;
        voltage_status |= BSP_VOLTAGE_V25_BELLOW_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V1 Lower limit: ",                   \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V25 * 95) / 100));
        #endif

    }
    if (value_compare_by_percentage(BSP_OP_GREATER, tvs_v1_mV,              \
                    105, BSP_TVS_MONITORED_VOLTAGE_V1))
    {
        *volt_high = true;
        voltage_status |= BSP_VOLTAGE_V1_ABOVE_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V1 Upper limit: ",                   \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V1 * 105) / 100));
        #endif

    }
    if (value_compare_by_percentage(BSP_OP_GREATER, tvs_v18_mV,             \
                        105, BSP_TVS_MONITORED_VOLTAGE_V18))
    {
        *volt_high = true;
        voltage_status |= BSP_VOLTAGE_V18_ABOVE_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V18 Upper limit: ",                  \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V18 * 105) / 100));
        #endif
    }
    if (value_compare_by_percentage(BSP_OP_GREATER, tvs_v25_mV,             \
                            105, BSP_TVS_MONITORED_VOLTAGE_V25))
    {
        *volt_high = true;
        voltage_status |= BSP_VOLTAGE_V25_ABOVE_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_V25 Upper limit: ",                  \
                    (int32_t)((BSP_TVS_MONITORED_VOLTAGE_V25 * 105) / 100));
        #endif
    }

    return voltage_status;
}

static uint32_t
read_temperatures(bool *temp_low, bool *temp_high)
{
    *temp_low = false;
    *temp_high = false;

    /// Read temperatures
    ///
    uint32_t hk_temp_img   = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG,\
            HK_TEMP_IMG);
    uint32_t hk_temp_fpga  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG,\
            HK_TEMP_FPGA);
    uint32_t tvs_temp_high = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG,\
            TVS_TEMP_HIGH);
    uint32_t tvs_temp_low  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG,\
            TVS_TEMP_LOW);

    /// convert to celsius degrees
    ///
    int32_t hk_temp_img_C  = tvs_decode_temp_milli_celsius(hk_temp_img & 0xFFFF
            , " TEMP_IMAGE") / 1000;
    int32_t hk_temp_fpga_C = tvs_decode_temp_milli_celsius(hk_temp_fpga & 0xFFFF
            , " TEMP_FPGA") / 1000;
    int32_t tvs_temp_low_C = tvs_decode_temp_milli_celsius(tvs_temp_low & 0xFFFF
            , " TEMP_LOW") / 1000;
    int32_t tvs_temp_high_C=tvs_decode_temp_milli_celsius(tvs_temp_high & 0xFFFF
            , " TEMP_HIGH") / 1000;

    /// Perform the check
    ///
    uint32_t temperature_status = BSP_TEMPERATURE_OK;;

    if (value_compare_by_limit(BSP_OP_LESS, hk_temp_img_C, tvs_temp_low_C))
    {
        *temp_low = true;
        temperature_status |= BSP_TEMPERATURE_IMAGE_BELLOW_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_TEMP_IMG Lower limit: ",            \
                    tvs_temp_low_C);
        #endif
    }
    if (value_compare_by_limit(BSP_OP_LESS, hk_temp_fpga_C, tvs_temp_low_C))
    {
        *temp_low = true;
        temperature_status |= BSP_TEMPERATURE_FPGA_BELLOW_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_TEMP_FPGA Lower limit: ",           \
                    tvs_temp_low_C);
        #endif
    }
    if (value_compare_by_limit(BSP_OP_GREATER,hk_temp_img_C,tvs_temp_high_C))
    {
        *temp_high = true;
        temperature_status |= BSP_TEMPERATURE_IMAGE_ABOVE_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_TEMP_IMG Upper limit: ",            \
                    tvs_temp_high_C);
        #endif
    }
    if (value_compare_by_limit(BSP_OP_GREATER,hk_temp_fpga_C,tvs_temp_high_C))
    {
        *temp_high = true;
        temperature_status |= BSP_TEMPERATURE_FPGA_ABOVE_LIMIT;

        #ifdef DEBUG_PRINT
            print_reference_value("TVS_TEMP_FPGA Upper limit: ",           \
                    tvs_temp_high_C);
        #endif
    }

    return temperature_status;
}

static void
read_i2c(struct i2c_status_t* i2c_status,                                   \
        uint32_t* emmc_pr_status, uint32_t* emmc_sc_status)
{
    i2c_status->current_pr_low      = false;
    i2c_status->current_pr_high     = false;
    i2c_status->current_sc_low      = false;
    i2c_status->current_sc_high     = false;
    i2c_status->voltage_pr_low      = false;
    i2c_status->voltage_pr_high     = false;
    i2c_status->voltage_sc_low      = false;
    i2c_status->voltage_sc_high     = false;

    *emmc_pr_status = BSP_UNKNOWN;
    *emmc_sc_status = BSP_UNKNOWN;

    (void)mss_config_clk_rst(MSS_PERIPH_I2C0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_I2C1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);

    __enable_irq();
    PLIC_init();

    MSS_GPIO_init(GPIO1_LO);
    MSS_GPIO_config(GPIO1_LO, MSS_GPIO_16, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_set_output(GPIO1_LO, MSS_GPIO_16, 1u);

    if (pac1934_sensor_probe(&g_mss_i2c1_lo) == 0)
    {
        bool active[4] = {true, true, true, true};
        bool bipolar[4] = {false, false, false, false};
        pac1934_configure(active, bipolar, 1024);    /// all channels, unipolar,
                                                     /// 1024 sps
        pac1934_refresh_v();

        uint16_t vsense_raw1;
        uint16_t vsense_raw2;
        uint16_t vsense_raw3;
        uint16_t vsense_raw4;

        uint16_t vbus_raw1;
        uint16_t vbus_raw2;
        uint16_t vbus_raw3;
        uint16_t vbus_raw4;

        pac1934_read_u16(VSENSE1_REG, &vsense_raw1);
        int32_t emmc_sc_1V8_mA =                                            \
                pac1934_vsense_raw_to_mA(vsense_raw1, false,                \
                        100);

        pac1934_read_u16(VSENSE2_REG, &vsense_raw2);
        int32_t emmc_sc_3V3_mA =                                            \
                pac1934_vsense_raw_to_mA(vsense_raw2, false,                \
                        100);

        pac1934_read_u16(VSENSE3_REG, &vsense_raw3);
        int32_t emmc_pr_1V8_mA =                                            \
                pac1934_vsense_raw_to_mA(vsense_raw3, false,                \
                        100);

        pac1934_read_u16(VSENSE4_REG, &vsense_raw4);
        int32_t emmc_pr_3V3_mA =                                            \
                pac1934_vsense_raw_to_mA(vsense_raw4, false,                \
                        100);


        pac1934_read_u16(VBUS1_REG, &vbus_raw1);
        int32_t emmc_sc_1V8_mV = pac1934_vbus_raw_to_mV(vbus_raw1, false);

        pac1934_read_u16(VBUS2_REG, &vbus_raw2);
        int32_t emmc_sc_3V3_mV = pac1934_vbus_raw_to_mV(vbus_raw2, false);

        pac1934_read_u16(VBUS3_REG, &vbus_raw3);
        int32_t emmc_pr_1V8_mV = pac1934_vbus_raw_to_mV(vbus_raw3, false);

        pac1934_read_u16(VBUS4_REG, &vbus_raw4);
        int32_t emmc_pr_3V3_mV = pac1934_vbus_raw_to_mV(vbus_raw4, false);

        #ifdef DEBUG_PRINT
            print_f1("\n\r eMMC Primary Current VCCQ: %dmA",emmc_pr_1V8_mA);
            print_f1(" eMMC Primary Current VCC: %dmA",emmc_pr_3V3_mA);
            print_f1(" eMMC Secondary Current VCCQ: %dmA",emmc_sc_1V8_mA);
            print_f1(" eMMC Secondary Current VCC: %dmA",emmc_sc_3V3_mA);
            print_f1(" eMMC Primary Voltage VCCQ: %dmV",emmc_pr_1V8_mV);
            print_f1(" eMMC Primary Voltage VCC: %dmV",emmc_pr_3V3_mV);
            print_f1(" eMMC Secondary Voltage VCCQ: %dmV",emmc_sc_1V8_mV);
            print_f1(" eMMC Secondary Voltage VCC: %dmV",emmc_sc_3V3_mV);
#if 0
            print_f1(" eMMC Primary Current VCCQ: 0x%08X RAW",vsense_raw1);
            print_f1(" eMMC Primary Current VCC: 0x%08X RAW",vsense_raw2);
            print_f1(" eMMC Secondary Current VCCQ: 0x%08X RAW",vsense_raw3);
            print_f1(" eMMC Secondary Current VCC: 0x%08X RAW",vsense_raw4);
            print_f1(" eMMC Primary Voltage VCCQ: 0x%08X RAW",vbus_raw1);
            print_f1(" eMMC Primary Voltage VCC: 0x%08X RAW",vbus_raw2);
            print_f1(" eMMC Secondary Voltage VCCQ: 0x%08X RAW",vbus_raw3);
            print_f1(" eMMC Secondary Voltage VCC: 0x%08X RAW",vbus_raw4);
#endif
        #endif
    }

}

static bool
value_compare_by_percentage(bsp_hk_comp_op_t op, int32_t value,                \
        uint16_t percentage, int32_t ref)
{
    switch (op)
    {
    case BSP_OP_LESS:
        return value < (int32_t)((ref * percentage) / 100) ? true : false;
    case BSP_OP_GREATER:
        return value > (int32_t)((ref * percentage) / 100) ? true : false;
    case BSP_OP_EQUAL:
        return value == (int32_t)((ref * percentage) / 100) ? true : false;
    default:
        break;
    }
    return false;
}

static bool
value_compare_by_limit(bsp_hk_comp_op_t op, int32_t value, int32_t limit)
{
    switch (op)
    {
    case BSP_OP_LESS:
        return value < limit ? true : false;
    case BSP_OP_GREATER:
        return value > limit ? true : false;
    case BSP_OP_EQUAL:
        return value == limit ? true : false;
    default:
        break;
    }
    return false;
}
/** ----------------------------------------------------------------------------
 * @brief                   Runs a step -- suppose to be in a loop -- of the
 *                          Event engine
 *
 * @param[in] user_data     Any user data to be passed onto the Event engine
 *
 * @return                  State of the Event engine
 *
 * -----------------------------------------------------------------------------
*/
bsp_hk_state_t
BSP_HK_EVE_Step(void* user_data)
{
    bool volt_low = false;
    bool volt_high = false;

    bool temp_low = false;
    bool temp_high = false;

    uint32_t voltage_status     = read_voltages(&volt_low, &volt_high);
    uint32_t temperature_status = read_temperatures(&temp_low, &temp_high);

    #ifdef DEBUG_PRINT
        print_voltage_status(voltage_status);
        print_temperature_status(temperature_status);
    #endif

    /// Notify and fire events
    ///
    if (0)//volt_low == true || volt_high == true)
    {
        bsp_event_t event = {
            .type      = volt_low == true ? BSP_EVT_TVS_VOLT_LOW :          \
                    BSP_EVT_TVS_VOLT_HIGH,
            .user_data = user_data,
            .data_len  = 0u
        };
        #ifdef DEBUG_PRINT
            #ifdef BSP_HSS_BUILD
                sbi_printf(                              \
                        " Launching event: %s", hk_event_codes[event.type]);
            #else
                uint8_t msg[64];
                sprintf((char*)msg, " Launching event: %s\n\r",             \
                        hk_event_codes[event.type]);
                print_message((const char*)msg);
            #endif
        #endif
        BSP_subject_notify(&g_camera_subject, &event);

        /// Return fail state
        ///
        return BSP_HK_VOLTAGE_FAILURE;
    }

    if (0u) /// (temp_low == true || temp_high == true) /// Temporary
    {
        bsp_event_t event = {
            .type      = temp_low == true ? BSP_EVT_TVS_TEMP_LOW :          \
                    BSP_EVT_TVS_TEMP_HIGH,
            .user_data = user_data,
            .data_len  = 0u
        };
        #ifdef DEBUG_PRINT
            #ifdef BSP_HSS_BUILD
                sbi_printf(                              \
                        " Launching event: %s", hk_event_codes[event.type]);
            #else
                uint8_t msg[64];
                sprintf((char*)msg, " Launching event: %s\n\r",             \
                        hk_event_codes[event.type]);
                print_message((const char*)msg);
            #endif
        #endif
        BSP_subject_notify(&g_camera_subject, &event);

        /// Return fail state
        ///
        return BSP_HK_TEMPERATURE_FAILURE;
    }

    struct i2c_status_t i2c_status;

    uint32_t emmc_pr_status = BSP_CURRENT_AND_VOLTAGE_OK;
    uint32_t emmc_sc_status = BSP_CURRENT_AND_VOLTAGE_OK;

    read_i2c(&i2c_status, &emmc_pr_status, &emmc_sc_status);

    #ifdef DEBUG_PRINT
        print_emmc_status(emmc_pr_status,emmc_sc_status);
    #endif

    return BSP_HK_OK;
}


/**
 * TVS Voltage Channel Decoder (no floating point)
 *
 * Raw 16-bit format (Table 1-3):
 *   Bit 15:     Sign bit (1 = negative voltage)
 *   Bits[14:3]: Integer part of voltage in mV
 *   Bits[2:0]:  Fractional part of voltage in mV (1/8 resolution)
 *
 * Returns voltage in micro-volts (uV) as int32_t.
 * Divide by 1000 for milli-volts, by 1000000 for volts.
 *
 * Example: 0x385E -> 1803.75 mV -> 1803750 uV
 */
int32_t
tvs_decode_voltage_micro_volts(uint16_t raw, const char* label)
{
    /** --- Extract fields --------------------------------------------------*/
    uint8_t  sign_bit    = (raw >> 15u) & 0x01u;    /** bit 15               */
    uint16_t volt_int    = (raw >>  3u) & 0x0FFFu;  /** bits[14:3], 12 bits  */
    uint8_t  volt_frac   =  raw         & 0x07u;    /** bits[2:0],  3 bits   */

    /**
     * Fractional part represents n/8 mV.
     * Work in micro-volts (uV) to stay integer throughout:
     *
     *   uV = (volt_int * 8 + volt_frac) * (1000 / 8)
     *      = (volt_int * 8 + volt_frac) * 125
     */
    uint32_t magnitude_uv = (uint32_t)(volt_int * 8u + volt_frac) * 125u;

    /** --- Apply sign ------------------------------------------------------*/
    int32_t result_uv = (sign_bit) ? -(int32_t)magnitude_uv
                                   :  (int32_t)magnitude_uv;

    #ifdef DEBUG_PRINT
        if (label != 0)
        {
            /** Display in mV without floating point */
            int32_t mv_whole = result_uv / 1000;
            int32_t mv_frac  = result_uv % 1000;
            if (mv_frac < 0) mv_frac = -mv_frac;

        #ifndef BSP_HSS_BUILD
            sprintf((char*)g_ui_buf, " %s  %d.%03d mV\r\n",                  \
                    label, mv_whole, mv_frac);
            print( (uint8_t*)g_ui_buf);
        #else
            sbi_printf(" %s  %d.%03d mV\n\r", label, mv_whole, mv_frac);
        #endif
        }
    #endif


    return result_uv;
}

/**
 * TVS Temperature Channel Decoder (no floating point)
 *
 * Raw 16-bit format:
 *   Bit 15:    Reserved
 *   Bits[14:4]: Integer part of temperature in Kelvin
 *   Bits[3:0]:  Fractional part of temperature in Kelvin (1/16 resolution)
 *
 * Returns temperature in milli-Celsius (divide by 1000 for Celsius).
 * Example: 0x133B -> 307.5625 K -> 34562 m°C (34.562 °C)
 */
int32_t
tvs_decode_temp_milli_celsius(uint16_t raw, const char* label)
{
    /** --- Extract fields --------------------------------------------------*/
    uint16_t kelvin_int  = (raw >> 4) & 0x07FFu;   /** bits[14:4], 11 bits   */
    uint16_t kelvin_frac = raw & 0x0Fu;            /** bits[3:0],  4 bits    */

    /**
     * Fractional part represents n/16 Kelvin.
     * Work in units of milli-Kelvin to stay integer throughout:
     *
     *   milli_K = (kelvin_int * 16 + kelvin_frac) * (1000 / 16)
     *           = (kelvin_int * 16 + kelvin_frac) * 125  / 2
     *
     * Use 125/2 to avoid any division loss:
     *   multiply by 125 first, then shift right by 1 (divide by 2).
     */
    uint32_t milli_kelvin =                                                 \
            ((uint32_t)(kelvin_int * 16u + kelvin_frac) * 125u) >> 1;

    /** --- Convert milli-Kelvin -> milli-Celsius ---------------------------*/
    /** 0 °C = 273.15 K  =>  273150 milli-Kelvin offset                      */
    int32_t milli_celsius = (int32_t)milli_kelvin - 273150;

    #ifdef DEBUG_PRINT
        if (label != 0)
        {
            /** Print without floating point */
            int32_t whole = milli_celsius / 1000;
            int32_t frac  = milli_celsius % 1000;
            if (frac < 0) frac = -frac;  /** keep fractional part positive   */
            #ifndef BSP_HSS_BUILD
                sprintf((char*)g_ui_buf, " %s  %d.%03d deg C\r\n",           \
                        label, whole, frac);
                print( (uint8_t*)g_ui_buf);
            #else
                sbi_printf(" %s  %d.%03d deg C\n\r", label, whole, frac);
            #endif
       }
    #endif


    return milli_celsius;
}


#ifdef __cplusplus
    }
#endif

