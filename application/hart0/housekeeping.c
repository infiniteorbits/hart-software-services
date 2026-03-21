/** ----------------------------------------------------------------------------
 * @file        Housekeeping.c
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk
 *
 * @date        February - March 2026
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */
#include "housekeeping.h"

#include "hss_types.h"
#include "hss_debug.h"

#include <hal/hal.h>
#include <drivers/mss/mss_gpio/mss_gpio.h>
#include <mpfs_hal/common/mss_peripherals.h>

#include "regs.h"

#define DEBUG_PRINT
#ifdef DEBUG_PRINT
    static const char* hk_state_codes[] = {
        "HK_CAMERA_POWER_ON",
        "HK_VDD3V5_POWER_GOOD",
        "HK_VDD2V0_POWER_GOOD",
        "HK_VDD3V8_AND_VDD0V8_ON",
        "HK_ERROR",
        "HK_VDD3V8_OR_VDD0V8_ON",
        "HK_POWER_GOOD",
        "HK_UNKNOWN"
    };
#endif

/** -----------------------------------------------------------------------------
 * @brief   Events
 * -----------------------------------------------------------------------------
*/
hk_state_t
g_event_null(void *user_data, hk_state_t state);

hk_state_t
g_event_is_camera_power_on(void *user_data, hk_state_t state);

hk_state_t
g_event_is_camera_power_off(void *user_data, hk_state_t state);

hk_state_t
g_event_is_VDD3V5_on(void *user_data, hk_state_t state);

hk_state_t
g_event_is_VDD3V5_off(void *user_data, hk_state_t state);

hk_state_t
g_event_is_VDD2V0_on(void *user_data, hk_state_t state);

hk_state_t
g_event_is_VDD2V0_off(void *user_data, hk_state_t state);

hk_state_t
g_event_camera_power_good(void *user_data, hk_state_t state);

hk_state_t
g_event_VDD3V5_VDD2V0_VDD3V8_VDD0V8_off(void *user_data,                    \
        hk_state_t state);


/** -----------------------------------------------------------------------------
 * @brief   Actions
 * -----------------------------------------------------------------------------
*/
hk_state_t
g_action_camera_power_on(void *user_data, hk_state_t state,             \
            event_cb true_event, event_cb false_event);

hk_state_t
g_action_VDD3V5_power_good(void *user_data, hk_state_t state,           \
            event_cb true_event, event_cb false_event);

hk_state_t
g_action_VDD2V0_power_good(void *user_data, hk_state_t state,           \
            event_cb true_event, event_cb false_event);

hk_state_t
g_action_VDD3V8_VDD0V8_power_good(void *user_data, hk_state_t state,    \
            event_cb true_event, event_cb false_event);

hk_state_t
g_action_error(void *user_data, hk_state_t state,                       \
            event_cb true_event, event_cb false_event);

/** -----------------------------------------------------------------------------
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

/** -----------------------------------------------------------------------------
 * @note    This is the only place to extend the state machine by simply and
 *          only defining/adding new actions and events
 *
 * @note    The order of the defined/added actions is important! It has to
 *          correspond to hk_state_t enums
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
                 g_event_is_VDD3V5_off}
        },
        {g_action_VDD2V0_power_good,
                {g_event_is_VDD2V0_on,
                 g_event_is_VDD2V0_off}
        },
        {g_action_VDD3V8_VDD0V8_power_good,
                {g_event_camera_power_good,
                 g_event_VDD3V5_VDD2V0_VDD3V8_VDD0V8_off}
        },
        {g_action_error,
                {g_event_null,
                 g_event_null}
        }
};

/** -----------------------------------------------------------------------------
 * @brief                   Start of the Camera HK state machine
 *
 * @param[in] user_data     Any user data to be passed onto the state machine
 *
 * @return                  State of the state machine after applying all the
 *                          actions
 *
 * -----------------------------------------------------------------------------
*/
hk_state_t
HK_SM_Run(void* user_data)
{
    #ifdef DEBUG_PRINT
        sbi_puts(
          " \n\r");
    #endif

    hk_state_t state = HK_CAMERA_POWER_ON;

    while (state != HK_POWER_GOOD && state != HK_UNKNOWN)
    {
        /// Sanity check
        ///
        if (state < HK_CAMERA_POWER_ON && state > HK_UNKNOWN)
            return HK_UNKNOWN;

        struct outer_t outer = state_machine[state];
        struct inner_t inner = outer.inner;

        action_cb condition_action = outer.condition_action_cb;
        state = condition_action(user_data, state,                          \
                inner.true_event_cb, inner.false_event_cb);

    }

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state: %s\n\r",  \
          hk_state_codes[state]);
        sbi_puts(
          " \n\r");
    #endif

    return state;
}


hk_state_t
g_event_null(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[HK_UNKNOWN]);

    #endif

    return state;
}

hk_state_t
g_event_is_camera_power_on(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[state]);

    #endif

    return state; /// It suppose to be the next state in the state machine
}

hk_state_t
g_event_is_camera_power_off(void *user_data, hk_state_t state)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
       sbi_printf(
         " PG state check: %s\n\r",  \
         hk_state_codes[HK_UNKNOWN]);

   #endif

    return HK_UNKNOWN;
}

hk_state_t
g_event_is_VDD3V5_on(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[state]);

    #endif

    return state; /// It suppose to be the next state in the state machine
}

hk_state_t
g_event_is_VDD3V5_off(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[state]);

    #endif

    return state; /// It suppose to be the next state in the state machine
}

hk_state_t
g_event_is_VDD2V0_on(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[state]);

    #endif

    return state; /// It suppose to be the next state in the state machine
}

hk_state_t
g_event_is_VDD2V0_off(void *user_data, hk_state_t state)
{
    (void)user_data;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[state]);

    #endif

    return state; /// It suppose to be the next state in the state machine
}


hk_state_t
g_event_camera_power_good(void *user_data, hk_state_t state)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[HK_POWER_GOOD]);

    #endif

    return HK_POWER_GOOD;
}

hk_state_t
g_event_VDD3V5_VDD2V0_VDD3V8_VDD0V8_off(void *user_data,                    \
        hk_state_t state)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
        sbi_printf(
          " PG state check: %s\n\r",  \
          hk_state_codes[HK_ERROR]);

    #endif

    return HK_ERROR;
}


/** -----------------------------------------------------------------------------
 * @brief   Actions
 * -----------------------------------------------------------------------------
*/
hk_state_t
g_action_camera_power_on(void *user_data, hk_state_t state,             \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;
    (void)false_event;

    (void)mss_config_clk_rst(MSS_PERIPH_GPIO2,                              \
            (uint8_t) 0, PERIPHERAL_ON);

    MSS_GPIO_init(GPIO2_LO);

    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_7, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_8, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_9, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_10, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_11, MSS_GPIO_INPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_12, MSS_GPIO_INPUT_MODE);

    #ifdef DEBUG_PRINT
       sbi_puts(" PG action: Camera Power On\n\r");
    #endif

    return true_event(user_data, HK_VDD3V5_POWER_GOOD);
}

hk_state_t
g_action_VDD3V5_power_good(void *user_data, hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
       sbi_puts(" PG action: VDD3V5 PG\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_10, 1 );
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V5_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_12_MASK) >> 12u);
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P3V5_PG, ~pin);

    return pin == 1u ?
        true_event(user_data, HK_VDD2V0_POWER_GOOD) :
        false_event(user_data, HK_ERROR);
}

hk_state_t
g_action_VDD2V0_power_good(void *user_data, hk_state_t state,           \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    #ifdef DEBUG_PRINT
       sbi_puts(" PG action: VDD2V0 PG\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_8, 1 );
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_2V0_EN, 1u);

    uint32_t gpio_inputs;
    gpio_inputs = MSS_GPIO_get_inputs(GPIO2_LO);

    uint32_t pin = (uint32_t)((gpio_inputs & MSS_GPIO_11_MASK) >> 11u);
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, HK_P2V0_PG, ~pin);

    return pin == 1u ?
        true_event(user_data, HK_VDD3V8_AND_VDD0V8_ON) :
        false_event(user_data, HK_ERROR);
}

hk_state_t
g_action_VDD3V8_VDD0V8_power_good(void *user_data, hk_state_t state,    \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;
    (void)false_event;

    #ifdef DEBUG_PRINT
       sbi_puts(" PG action: VDD3V8 and VDD0V8 PG\n\r");
    #endif

    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_9, 1 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_7, 1 );
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_3V8_EN, 1u);
    HAL_set_32bit_reg_field(BASE32_ADDR_MSS_BSPREG, PWR_0V8_EN, 1u);

    return true_event(user_data, HK_POWER_GOOD);
}

hk_state_t
g_action_error(void *user_data, hk_state_t state,                       \
            event_cb true_event, event_cb false_event)
{
    (void)user_data;
    (void)state;

    (void)true_event;
    (void)false_event;

    #ifdef DEBUG_PRINT
       sbi_puts(" PG action: Error, Shutting down\n\r");
    #endif

    /// Graceful shutdown
    ///
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_7, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_9, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_8, 0 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_10, 0 );

    return HK_UNKNOWN;
}

