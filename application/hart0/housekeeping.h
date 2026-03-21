/** ----------------------------------------------------------------------------
 * @file        Housekeeping.h
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk
 * @date        January - March 2026
 *
 * @version     1.2.0       /// Added HK_ERROR state
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */

#ifndef HOUSEKEEPING_H_
#define HOUSEKEEPING_H_

#include <stdint.h>


#include <stdbool.h>
#include <stddef.h>

/** ----------------------------------------------------------------------------
* @brief BSP Camera housekeeping state machine states .
* ------------------------------------------------------------------------------
*/
typedef enum {
    HK_CAMERA_POWER_ON,
    HK_VDD3V5_POWER_GOOD,
    HK_VDD2V0_POWER_GOOD,
    HK_VDD3V8_AND_VDD0V8_ON,
    HK_ERROR,
    HK_VDD3V8_OR_VDD0V8_ON,
    HK_POWER_GOOD,
    HK_UNKNOWN
} hk_state_t;

/** -----------------------------------------------------------------------------
 * @note    The initial action callbacks when the camera is turned on
 *          It is called like this
 *
 *          if (HK_POWER_GOOD == g_action_camera_power_on((void*)0,
 *                  HK_CAMERA_POWER_ON,
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
typedef hk_state_t(*event_cb)(void *user_data, hk_state_t state);

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
typedef hk_state_t(*action_cb)(void *user_data, hk_state_t state,  \
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
hk_state_t HK_SM_Run(void* user_data);

#endif /// HOUSEKEEPING_H_
