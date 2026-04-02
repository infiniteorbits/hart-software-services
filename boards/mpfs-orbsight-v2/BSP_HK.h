/** ----------------------------------------------------------------------------
 * @file        BSP_HK.h
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk
 * @date        January - March 2026
 *
 * @version     1.2.0       /// Added BSP_HK_ERROR state
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */

#ifndef BSP_HK_H_
#define BSP_HK_H_

#ifdef __cplusplus
extern "C" {
    #include <cstdint.h>
#else
    #include <stdint.h>
#endif

#include <stddef.h>


/** ----------------------------------------------------------------------------
* @brief BSP Camera housekeeping state machine states .
* ------------------------------------------------------------------------------
*/
typedef enum {
    BSP_HK_CAMERA_POWER_ON,
    BSP_HK_VDD3V5_POWER_GOOD,
    BSP_HK_VDD2V0_POWER_GOOD,
    BSP_HK_VDD3V8_POWER_GOOD,
    BSP_HK_VDD0V8_POWER_GOOD,
    BSP_HK_VDD3V5_VDD2V0_POWER_GOOD,
    BSP_HK_ALL_POWER_GOOD,
    BSP_HK_ERROR,
    BSP_HK_POWER_GOOD,
    BSP_HK_UNKNOWN,
    BSP_HK_VOLTAGE_FAILURE,
    BSP_HK_TEMPERATURE_FAILURE,
    BSP_HK_OK
} bsp_hk_state_t;

/** -----------------------------------------------------------------------------
 * @note    The initial action callbacks when the camera is turned on
 *          It is called like this
 *
 *          if (BSP_HK_POWER_GOOD == g_action_camera_power_on((void*)0,
 *                  BSP_HK_CAMERA_POWER_ON,
 *                  g_event_is_camera_power_on,
 *                  g_event_is_camera_power_off
 *             )
 *          {
 *              ... report success;
 *          }
 *
 * @brief   An event callback producing state. Obviously these are attached
 *          to an action, for example and event for true and an event for
 *          false outcome of an action. This is how the state machine is built
 *
 * @param[in] user_data Pointer to user data passed when registering the
 *                      callback
 * @param[in] state:    The current state of the state machine
 * @return              New state
 *
 * -----------------------------------------------------------------------------
*/
typedef bsp_hk_state_t(*event_cb)(void *user_data, bsp_hk_state_t state);

/** -----------------------------------------------------------------------------
 * @brief   An action callback producing true or false for a decision in the
 *          state machine
 *
 * @param[in] user_data:    Pointer to user data passed when registering the
 *                          callback
 * @param[in] state:        The current state of the state machine
 * @param[in] true_event:   Event to happen in case the outcome of the action
 *                          is true
 * @param[in] false_event:  Event to happen in case the outcome of the action
 *                          is false

 *
 * @return                  State of the state machine after applying the action
 *
 * -----------------------------------------------------------------------------
*/
typedef bsp_hk_state_t(*action_cb)(void *user_data, bsp_hk_state_t state,  \
            event_cb true_event, event_cb false_event);


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
bsp_hk_state_t
BSP_HK_SM_Run(void* user_data);

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
BSP_HK_EVM_Step(void* user_data);


#ifdef __cplusplus
    }
#endif

#endif /// BSP_HK_H_
