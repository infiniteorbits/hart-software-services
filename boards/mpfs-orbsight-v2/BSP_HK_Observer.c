/** ----------------------------------------------------------------------------
 * @file        BSP_HK_Observer.c
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk | claude ai
 *  * @date        March 2026
 *
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
    #include <cstdint.h>
#else
    #include <stdint.h>
#endif

#include "BSP_HK_Observer.h"

#include <stddef.h>
#include <string.h>

bsp_subject_status_t
BSP_subject_init(bsp_subject_t *subject)
{
    if (NULL == subject)
    {
        return BSP_SUBJECT_ERR_NULL_SUBJECT;
    }

    memset(subject, 0, sizeof(bsp_subject_t));
    return BSP_SUBJECT_OK;
}

bsp_subject_status_t
BSP_subject_subscribe(bsp_subject_t *subject, bsp_observer_fn_t cb, void *ctx)
{
    if (NULL == subject) { return BSP_SUBJECT_ERR_NULL_SUBJECT;  }
    if (NULL == cb)      { return BSP_SUBJECT_ERR_NULL_CALLBACK; }

    for (uint8_t i = 0u; i < subject->count; i++)
    {
        if (subject->observers[i].callback == cb)
        {
            return BSP_SUBJECT_ERR_ALREADY_REG;
        }
    }

    if (subject->count >= BSP_MAX_NUM_OBSERVERS)
    {
        return BSP_SUBJECT_ERR_LIST_FULL;
    }

    subject->observers[subject->count].callback = cb;
    subject->observers[subject->count].ctx      = ctx;
    subject->count++;

    return BSP_SUBJECT_OK;
}

bsp_subject_status_t
BSP_subject_unsubscribe(bsp_subject_t *subject, bsp_observer_fn_t cb)
{
    if (NULL == subject) { return BSP_SUBJECT_ERR_NULL_SUBJECT;  }
    if (NULL == cb)      { return BSP_SUBJECT_ERR_NULL_CALLBACK; }

    for (uint8_t i = 0u; i < subject->count; i++)
    {
        if (subject->observers[i].callback == cb)
        {
            for (uint8_t j = i; j < (subject->count - 1u); j++)
            {
                subject->observers[j] = subject->observers[j + 1u];
            }

            memset(&subject->observers[subject->count - 1u], 0,             \
                   sizeof(bsp_observer_t));
            subject->count--;
            return BSP_SUBJECT_OK;
        }
    }

    return BSP_SUBJECT_ERR_NOT_FOUND;
}

bsp_subject_status_t
BSP_subject_notify(bsp_subject_t *subject, const bsp_event_t *event)
{
    if (NULL == subject) { return BSP_SUBJECT_ERR_NULL_SUBJECT; }
    if (NULL == event)   { return BSP_SUBJECT_ERR_NULL_EVENT;   }

    for (uint8_t i = 0u; i < subject->count; i++)
    {
        if (NULL != subject->observers[i].callback)
        {
            subject->observers[i].callback(event, subject->observers[i].ctx);
        }
    }

    return BSP_SUBJECT_OK;
}

#ifdef __cplusplus
    }
#endif

