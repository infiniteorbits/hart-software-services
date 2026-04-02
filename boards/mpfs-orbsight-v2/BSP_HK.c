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

#ifdef BSP_HSS_BUILD
    #include "hal/hal.h"
    #include "mpfs_hal/mss_hal.h"
    #include "mpfs_hal/mpfs_hal_version.h"
    #include "mpfs_hal/common/nwc/mss_nwc_init.h"

    #include "drivers/mss/mss_gpio/mss_gpio.h"

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
        "BSP_HK_OK"
    };

    static void print(uint8_t* msg);
    static void print(uint8_t* msg)
    {
        #ifdef BSP_HSS_BUILD
            /// sbi_puts((const char*)msg);
            mHSS_FANCY_PRINTF(LOG_NORMAL, " %s", (const char*)msg);
        #else
            MSS_UART_polled_tx_string(g_uart, (uint8_t*)msg);
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
tvs_decode_temp_milli_celsius(uint16_t raw, char* label);

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
tvs_decode_voltage_micro_volts(uint16_t raw, char* label);


/// Instance of the Camera Subject -- Part of the Event engine --
/// see BSP_HK_Observer.h for details -- Initialized in the HK PG init, when the
/// camera is turned on, to be observed for events by the Event engine
///
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
        mHSS_FANCY_PRINTF(LOG_NORMAL, "  PG state check: %s\n\r",             \
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
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V5_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_12_MASK) >> 12u);
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P3V5_PG, ~pin);

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
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_2V0_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P2V0_PG, ~pin);

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
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V8_EN, 1u);

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
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_0V8_EN, 1u);

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
        mHSS_FANCY_PRINTF(LOG_NORMAL, " PG pin 11: %d\n\r",                 \
                (uint32_t)(gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
        mHSS_FANCY_PRINTF(LOG_NORMAL, " PG pin 12: %d\n\r",                 \
                (uint32_t)(gpio_inputs & MSS_GPIO_12_MASK) >> 12u);
      #endif
    #else
        (void)gpio_inputs;
    #endif

    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, SNS_ENABLE, 1u);
    HAL_set_32bit_reg(BASE32_ADDR_MSS_BSPREG, SENSOR_CTRL, 1U);

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

    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, SNS_ENABLE, 0);

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

    #ifdef DEBUG_PRINT
        print(                                  \
                (uint8_t*)" HK EVT action: Voltage failure, Shutting down\n\r");
    #endif

    (void)g_action_error(ctx, BSP_HK_VOLTAGE_FAILURE, 0, 0);
}


/** -----------------------------------------------------------------------------
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
BSP_HK_EVM_Step(void* user_data)
{
    /// Read voltages
    ///
    uint32_t tvs_v18 = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V18);
    uint32_t tvs_v1  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V1);
    uint32_t tvs_v25 = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_V25);

    /// Convert to milli_voltages
    ///
    int32_t tvs_v18_mV = tvs_decode_voltage_micro_volts(tvs_v18 & 0xFFFF, 0)/
            1000;
    int32_t tvs_v1_mV = tvs_decode_voltage_micro_volts(tvs_v1 & 0xFFFF, 0) /
            1000;
    int32_t tvs_v25_mV = tvs_decode_voltage_micro_volts(tvs_v25 & 0xFFFF, 0)/
            1000;

    bool volt_low = false;
    bool volt_high = false;

    /// 5% threshold from the monitored values
    ///
    if (tvs_v1_mV < ((BSP_TVS_MONITORED_VOLTAGE_V1 * 95) / 100))
        volt_low = true;
    if (tvs_v18_mV < ((BSP_TVS_MONITORED_VOLTAGE_V18 * 95) / 100))
        volt_low = true;
    if (tvs_v25_mV < ((BSP_TVS_MONITORED_VOLTAGE_V25 * 95) / 100))
        volt_low = true;
    if (tvs_v1_mV > ((BSP_TVS_MONITORED_VOLTAGE_V1 * 105) / 100))
        volt_high = true;
    if (tvs_v18_mV > ((BSP_TVS_MONITORED_VOLTAGE_V18 * 105) / 100))
        volt_high = true;
    if (tvs_v25_mV > ((BSP_TVS_MONITORED_VOLTAGE_V25 * 105) / 100))
        volt_high = true;

    /// Notify and fire events
    ///
    if (volt_low || volt_high)
    {
        bsp_event_t event = {
            .type      = volt_low == true ? BSP_EVT_TVS_VOLT_LOW : BSP_EVT_TVS_VOLT_HIGH,
            .user_data = user_data,
            .data_len  = 0u
        };
        BSP_subject_notify(&g_camera_subject, &event);

        /// Return fail state
        ///
        return BSP_HK_VOLTAGE_FAILURE;
    }


    /// Read temperatures
    ///
    uint32_t hk_temp_img   = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_TEMP_IMG);
    uint32_t hk_temp_fpga  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_TEMP_FPGA);

    uint32_t tvs_temp_high = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_TEMP_HIGH);
    uint32_t tvs_temp_low  = HAL_get_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, TVS_TEMP_LOW);

    /// convert to celsius degrees
    ///
    int32_t hk_temp_img_C   = tvs_decode_temp_milli_celsius(hk_temp_img & 0xFFFF, 0) / 1000;
    int32_t hk_temp_fpga_C  = tvs_decode_temp_milli_celsius(hk_temp_fpga & 0xFFFF, 0) / 1000;

    int32_t tvs_temp_high_C = tvs_decode_temp_milli_celsius(tvs_temp_high & 0xFFFF, 0) / 1000;
    int32_t tvs_temp_low_C  = tvs_decode_temp_milli_celsius(tvs_temp_low & 0xFFFF, 0) / 1000;

    /// Perform the check
    ///
    bool temp_low = false;
    bool temp_high = false;

    if (hk_temp_img_C < tvs_temp_low_C) temp_low = true;
    if (hk_temp_fpga_C < tvs_temp_low_C) temp_low = true;

    if (hk_temp_img_C > tvs_temp_high_C) temp_high = true;
    if (hk_temp_fpga_C > tvs_temp_high_C) temp_high = true;

    if (temp_low || temp_high)
    {
        bsp_event_t event = {
            .type      = temp_low == true ? BSP_EVT_TVS_TEMP_LOW : BSP_EVT_TVS_TEMP_HIGH,
            .user_data = user_data,
            .data_len  = 0u
        };
        BSP_subject_notify(&g_camera_subject, &event);

        /// Return fail state
        ///
        return BSP_HK_TEMPERATURE_FAILURE;
    }

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
tvs_decode_voltage_micro_volts(uint16_t raw, char* label)
{
    /** --- Extract fields -------------------------------------------------- */
    uint8_t  sign_bit    = (raw >> 15u) & 0x01u;        /** bit 15           */
    uint16_t volt_int    = (raw >>  3u) & 0x0FFFu;      /** bits[14:3], 12 bits */
    uint8_t  volt_frac   =  raw         & 0x07u;        /** bits[2:0],  3 bits  */

    /**
     * Fractional part represents n/8 mV.
     * Work in micro-volts (uV) to stay integer throughout:
     *
     *   uV = (volt_int * 8 + volt_frac) * (1000 / 8)
     *      = (volt_int * 8 + volt_frac) * 125
     */
    uint32_t magnitude_uv = (uint32_t)(volt_int * 8u + volt_frac) * 125u;

    /** --- Apply sign ------------------------------------------------------ */
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
            sprintf((char*)g_ui_buf, "%s  %d.%03d mV\r\n", label, mv_whole, mv_frac);
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
tvs_decode_temp_milli_celsius(uint16_t raw, char* label)
{
    /** --- Extract fields -------------------------------------------------- */
    uint16_t kelvin_int  = (raw >> 4) & 0x07FFu;   /** bits[14:4], 11 bits  */
    uint16_t kelvin_frac = raw & 0x0Fu;             /** bits[3:0],  4 bits   */

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
    uint32_t milli_kelvin = ((uint32_t)(kelvin_int * 16u + kelvin_frac) * 125u) >> 1;

    /** --- Convert milli-Kelvin -> milli-Celsius --------------------------- */
    /** 0 °C = 273.15 K  =>  273150 milli-Kelvin offset                      */
    int32_t milli_celsius = (int32_t)milli_kelvin - 273150;

    #ifdef DEBUG_PRINT
        if (label != 0)
        {
            /** Print without floating point */
            int32_t whole = milli_celsius / 1000;
            int32_t frac  = milli_celsius % 1000;
            if (frac < 0) frac = -frac;          /** keep fractional part positive   */
            #ifndef BSP_HSS_BUILD
                sprintf((char*)g_ui_buf, "%s  %d.%03d deg C\r\n", label, whole, frac);
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

