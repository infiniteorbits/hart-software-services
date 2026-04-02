/** ----------------------------------------------------------------------------
 * @file        BSP_HK_Observer.h
 * @brief
 * @author      Trajce Nikolov | nick@rfim.co.uk | claude ai
 * @date        March 2026
 *
 * @version     1.0.0
 *
 * @copyright   RFIM Space 2025-2026
 * -----------------------------------------------------------------------------
 */

/**
 * @brief   BSP Subject/Observer — Quick Start Usage Guide
 *
 * @details
 *
 * Define a subject (owned by the module that produces events)
 * @code
 *
 *   bsp_subject_t g_mmc_subject;
 *
 * @endcode
 *
 * Initialise the subject once at startup
 * @code
 *
 *   subject_init(&g_mmc_subject);
 *
 * @endcode
 *
 * Write an observer callback
 * @code
 *
 *   void on_mmc_done(const bsp_event_t *event, void *ctx)
 *   {
 *       if (BSP_EVT_MMC_TRANSFER_DONE != event->type) { return; }
 *       printf("Transfer complete\n\r");
 *   }
 *
 * @endcode
 *
 * Subscribe the observer
 * @code
 *
 *   subject_subscribe(&g_mmc_subject, on_mmc_done, NULL);
 *
 * @endcode
 *
 * Fire an event from the producing module
 * @code
 *
 *   bsp_event_t event = {
 *       .type      = BSP_EVT_MMC_TRANSFER_DONE,
 *       .user_data = NULL,
 *       .data_len  = 0u
 *   };
 *   subject_notify(&g_mmc_subject, &event);
 *   // on_mmc_done() is called automatically
 *
 * @endcode
 *
 * Unsubscribe when no longer needed
 * @code
 *
 *   subject_unsubscribe(&g_mmc_subject, on_mmc_done);
 *
 * @endcode
 *
 * -------------------------------------------------------------------------
 *
 * @brief Passing a payload
 *
 * @code
 *
 *   // Define a payload type
 *   typedef struct {
 *       uint32_t bytes_transferred;
 *       uint32_t block_address;
 *   } mmc_done_payload_t;
 *
 *   // Build and fire the event
 *   mmc_done_payload_t payload = { 512u, 0x1000u };
 *   bsp_event_t event = {
 *       .type      = BSP_EVT_MMC_TRANSFER_DONE,
 *       .user_data = &payload,
 *       .data_len  = sizeof(payload)
 *   };
 *   subject_notify(&g_mmc_subject, &event);
 *
 *   // In the observer — cast user_data to the expected type
 *   void on_mmc_done(const bsp_event_t *event, void *ctx)
 *   {
 *       const mmc_done_payload_t *p = (const mmc_done_payload_t *)event->user_data;
 *       printf("Transferred %u bytes @ block 0x%08X\n\r",
 *              p->bytes_transferred, p->block_address);
 *   }
 *
 * @endcode
 *
 * -------------------------------------------------------------------------
 *
 * @brief Passing context to an observer
 *
 * @code
 *
 *   // Define a context type
 *   typedef struct {
 *       uint32_t transfer_count;
 *   } my_ctx_t;
 *
 *   // Observer uses the context
 *   void on_mmc_done(const bsp_event_t *event, void *ctx)
 *   {
 *       my_ctx_t *c = (my_ctx_t *)ctx;
 *       c->transfer_count++;
 *       printf("Total transfers: %u\n\r", c->transfer_count);
 *   }
 *
 *   // Subscribe with context
 *   static my_ctx_t g_ctx = { 0u };
 *   subject_subscribe(&g_mmc_subject, on_mmc_done, &g_ctx);
 *
 * @endcode
 *
 * -------------------------------------------------------------------------
 *
 * @brief Error handling
 *
 * @code
 *
 *   subject_status_t status = subject_subscribe(&g_mmc_subject,
 *                                               on_mmc_done,
 *                                               NULL);
 *   if (BSP_SUBJECT_OK != status)
 *   {
 *       // BSP_SUBJECT_ERR_LIST_FULL    — MAX_OBSERVERS reached
 *       // BSP_SUBJECT_ERR_ALREADY_REG — callback already subscribed
 *       // BSP_SUBJECT_ERR_NULL_BSP_SUBJECT — subject pointer is NULL
 *       // BSP_SUBJECT_ERR_NULL_CALLBACK— cb pointer is NULL
 *       printf("Subscribe failed: %d\n\r", status);
 *   }
 *
 * @endcode
 *
 * -------------------------------------------------------------------------
 *
 * @brief Important rules
 *
 * @code
 *
 *   // Payload lifetime — payload must remain valid for the
 *   //    duration of subject_notify(). Stack allocation is safe.
 *   //    Do NOT store event->user_data pointer after callback returns.
 *
 *   // No re-entrant subscription — do NOT call subject_subscribe()
 *   //    or subject_unsubscribe() from within an observer callback.
 *
 *   // Not thread-safe — if subject_notify() and subject_subscribe()
 *   //    are called from different contexts (ISR + task), protect
 *   //    with a critical section.
 *
 * @endcode
 */

#ifndef BSP_HK_OBSERVER_
#define BSP_HK_OBSERVER_

#ifdef __cplusplus
extern "C" {
    #include <cstdint.h>
#else
    #include <stdint.h>
#endif

#include "BSP_Config.h"

/**
 * @brief Return status codes for subject/observer API functions.
 *
 * Returned by subject_subscribe() and subject_unsubscribe() to indicate
 * the outcome of the requested operation.
 */
typedef enum
{
    BSP_SUBJECT_OK                  = 0,    /**< Operation completed successfully.                      */
    BSP_SUBJECT_ERR_NULL_SUBJECT    = 1,    /**< Subject pointer is NULL.                               */
    BSP_SUBJECT_ERR_NULL_CALLBACK   = 2,    /**< Observer callback pointer is NULL.                     */
    BSP_SUBJECT_ERR_LIST_FULL       = 3,    /**< Observer list has reached BSP_MAX_NUM_OBSERVERS capacity.      */
    BSP_SUBJECT_ERR_NOT_FOUND       = 4,    /**< Callback not found in the observer list.               */
    BSP_SUBJECT_ERR_ALREADY_REG     = 5,    /**< Callback is already registered with this subject.      */
    BSP_SUBJECT_ERR_NULL_EVENT      = 6,    /**< Event pointer passed to subject_notify() is NULL.      */
    BSP_SUBJECT_ERR_INVALID         = 7,    /**< Unspecified invalid argument or internal error.        */
} bsp_subject_status_t;

/** Event types — extend as needed for the system                          */
typedef enum
{
    BSP_EVT_TVS_TEMP_LOW        = 1u,
    BSP_EVT_TVS_TEMP_HIGH       = 2u,
    BSP_EVT_TVS_VOLT_LOW        = 3u,
    BSP_EVT_TVS_VOLT_HIGH       = 4u,
    BSP_EVT_COUNT
} bsp_event_type_t;

/**
 * @brief Event data passed to observer callbacks.
 *
 * Members are ordered by descending alignment requirement to eliminate
 * implicit padding on both 32-bit and 64-bit RISC-V targets.
 *
 * Layout (RV64):
 *   user_data  — 8 bytes (pointer, largest alignment)
 *   data_len   — 4 bytes
 *   type       — 4 bytes (enum, sizeof uint32_t)
 * Total        — 16 bytes, no padding
 */
typedef struct
{
    void               *user_data;  /**< Optional payload. Cast to concrete type at call site.  */
    uint32_t            data_len;   /**< Size in bytes of the data at user_data. Zero if unused. */
    bsp_event_type_t    type;       /**< Event type identifier. See bsp_event_type_t.           */
} bsp_event_t;

/**
 * @brief Observer callback function type.
 *
 * Defines the signature of a callback function that can be registered
 * with a subject via subject_subscribe(). The callback is invoked
 * synchronously by subject_notify() for every event broadcast on the
 * subject to which it is subscribed.
 *
 * @param[in] event  Pointer to the event broadcasted by the subject.
 *                   The pointer is only guaranteed valid for the duration
 *                   of the callback invocation. The observer must not
 *                   store or dereference this pointer after returning.
 * @param[in] ctx    Caller-defined context pointer supplied at registration
 *                   time via subject_subscribe(). May be NULL if no context
 *                   was provided. The observer is responsible for casting
 *                   this to the appropriate concrete type.
 *
 * @note The callback must not call subject_subscribe() or
 *       subject_unsubscribe() on the same subject from which it was
 *       invoked, as this modifies the observer list during iteration.
 *
 * @note If subject_notify() is called from an ISR context, this callback
 *       executes in that same ISR context. The implementation must be
 *       ISR-safe and must not block, sleep, or perform any operation
 *       that may cause priority inversion or re-entrant subject access.
 */
typedef void (*bsp_observer_fn_t)(const bsp_event_t *event, void *ctx);


/**
 * @brief Observer instance associating a callback with its context.
 *
 * Holds a single registered observer entry within a subject's observer
 * list. Each observer is identified uniquely by its callback function
 * pointer. The context pointer allows the same callback function to be
 * reused across multiple subjects or instances while maintaining
 * independent state per registration.
 *
 * @note Both members are pointer-sized on all supported targets (RV32
 *       and RV64). No implicit padding is inserted by the compiler and
 *       no explicit padding is required.
 *
 * Memory layout (RV64):
 * @code
 * ┌──────────────────────┬──────────────────────┐
 * │ callback (8B)        │ ctx (8B)             │
 * └──────────────────────┴──────────────────────┘
 * Total: 16 bytes — no padding
 * @endcode
 *
 * Memory layout (RV32):
 * @code
 * ┌──────────────┬──────────────┐
 * │ callback (4B)│ ctx (4B)     │
 * └──────────────┴──────────────┘
 * Total: 8 bytes — no padding
 * @endcode
 */
typedef struct
{
    bsp_observer_fn_t   callback;   /**< Pointer to the observer callback function.
                                     *   Set to NULL for an empty/unused observer slot.
                                     *   Used as the unique identifier for subscribe
                                     *   and unsubscribe operations.                    */

    void               *ctx;        /**< Caller-defined context pointer passed back
                                     *   to the callback on each notification.
                                     *   May be NULL if the observer requires no
                                     *   external state. Ownership and lifetime of
                                     *   the pointed-to data remain with the caller.   */
} bsp_observer_t;

/**
 * @brief Subject instance holding the registered observer list.
 *
 * Members are ordered and padded explicitly to satisfy the alignment
 * requirement imposed by bsp_observer_t, which contains pointer-sized
 * members (8 bytes on RV64, 4 bytes on RV32).
 *
 * The _pad[] array is sized to round the total struct size up to the
 * next multiple of sizeof(void*), eliminating tail padding and
 * suppressing -Wpadded on both RV32 and RV64 targets.
 *
 * Memory layout (RV64, BSP_MAX_NUM_OBSERVERS = 8):
 * @code
 * ┌─────────────────────────────────────┬───────┬───────────┐
 * │ observers[8] — 128 bytes            │ count │ _pad[7]   │
 * │ (8 × 16B)                           │ (1B)  │ (7B)      │
 * └─────────────────────────────────────┴───────┴───────────┘
 * Total: 136 bytes — no implicit padding
 * @endcode
 *
 * Memory layout (RV32, BSP_MAX_NUM_OBSERVERS = 8):
 * @code
 * ┌─────────────────────────────────────┬───────┬──────────┐
 * │ observers[8] — 64 bytes             │ count │ _pad[3]  │
 * │ (8 × 8B)                            │ (1B)  │ (3B)     │
 * └─────────────────────────────────────┴───────┴──────────┘
 * Total: 68 bytes — no implicit padding
 * @endcode
 */
typedef struct
{
    bsp_observer_t  observers[BSP_MAX_NUM_OBSERVERS];   /**< Registered observer list.
                                                          *   Slots with callback == NULL
                                                          *   are treated as empty.           */

    uint8_t         count;                              /**< Number of currently active
                                                           *   observers in the list.
                                                           *   Range: 0..BSP_MAX_NUM_OBSERVERS.        */

    uint8_t         _pad[sizeof(void *) - 1u];          /**< Explicit tail padding to align
                                                           *   struct size to sizeof(void*)
                                                           *   boundary. Suppresses -Wpadded
                                                           *   on RV32 (3 bytes) and RV64
                                                           *   (7 bytes). Reserved, do not use.*/
} bsp_subject_t;

/**
 * @brief Initialise a subject instance.
 *
 * Clears the observer list and resets the observer count to zero.
 * Must be called once before any calls to subject_subscribe(),
 * subject_unsubscribe(), or subject_notify().
 *
 * @param[in,out] subject  Pointer to the subject instance to initialise.
 *                         Must not be NULL.
 *
 * @retval BSP_SUBJECT_OK               Initialisation successful.
 * @retval BSP_SUBJECT_ERR_NULL_BSP_SUBJECT subject pointer is NULL.
 */
bsp_subject_status_t
BSP_subject_init(bsp_subject_t *subject);


/**
 * @brief Subscribe an observer callback to a subject.
 *
 * Registers the provided callback function and its associated context pointer
 * in the subject's observer list. The callback will be invoked on every
 * subsequent call to subject_notify() until it is unsubscribed.
 *
 * If the callback is already registered with the subject, this function
 * returns BSP_SUBJECT_ERR_ALREADY_REG without adding a duplicate entry.
 *
 * @param[in,out] subject  Pointer to the subject instance. Must not be NULL.
 * @param[in]     cb       Observer callback function to register.
 *                         Must not be NULL.
 * @param[in]     ctx      Caller-defined context pointer passed back to the
 *                         callback on each notification. May be NULL if no
 *                         context is required.
 *
 * @retval BSP_SUBJECT_OK               Observer successfully registered.
 * @retval BSP_SUBJECT_ERR_NULL_BSP_SUBJECT subject pointer is NULL.
 * @retval BSP_SUBJECT_ERR_NULL_CALLBACK cb pointer is NULL.
 * @retval BSP_SUBJECT_ERR_LIST_FULL    Observer list has reached BSP_MAX_NUM_OBSERVERS.
 * @retval BSP_SUBJECT_ERR_ALREADY_REG  Callback is already registered.
 */
bsp_subject_status_t
BSP_subject_subscribe(bsp_subject_t *subject, bsp_observer_fn_t cb, void *ctx);


/**
 * @brief Unsubscribe an observer callback from a subject.
 *
 * Removes the first occurrence of the provided callback function from the
 * subject's observer list. The remaining observers are compacted to fill
 * the vacated slot, preserving their original registration order.
 *
 * After a successful unsubscribe, the callback will no longer be invoked
 * by subject_notify().
 *
 * @param[in,out] subject  Pointer to the subject instance. Must not be NULL.
 * @param[in]     cb       Observer callback function to remove.
 *                         Must not be NULL.
 *
 * @retval BSP_SUBJECT_OK               Observer successfully removed.
 * @retval BSP_SUBJECT_ERR_NULL_BSP_SUBJECT subject pointer is NULL.
 * @retval BSP_SUBJECT_ERR_NULL_CALLBACK cb pointer is NULL.
 * @retval BSP_SUBJECT_ERR_NOT_FOUND    Callback not found in observer list.
 */
bsp_subject_status_t
BSP_subject_unsubscribe(bsp_subject_t *subject, bsp_observer_fn_t cb);


/**
 * @brief Notify all registered observers of an event.
 *
 * Iterates the subject's observer list and invokes each registered callback
 * in the order of registration, passing the event and the observer's context
 * pointer. Notification is synchronous — all callbacks are invoked before
 * this function returns.
 *
 * Observers must not call subject_subscribe() or subject_unsubscribe() on
 * the same subject from within their callback, as this will modify the
 * observer list during iteration.
 *
 * @param[in] subject  Pointer to the subject instance. Must not be NULL.
 * @param[in] event    Pointer to the event to broadcast to all observers.
 *                     Must not be NULL. The event pointer and its data payload
 *                     are only guaranteed valid for the duration of the call.
 *
 * @retval BSP_SUBJECT_OK               All observers notified successfully.
 * @retval BSP_SUBJECT_ERR_NULL_BSP_SUBJECT subject pointer is NULL.
 * @retval BSP_SUBJECT_ERR_NULL_EVENT   event pointer is NULL.
 *
 * @note This function is not thread-safe. If called from multiple execution
 *       contexts (e.g. ISR and task), the caller is responsible for providing
 *       appropriate mutual exclusion around subject_notify() and
 *       subject_subscribe() / subject_unsubscribe().
 */
bsp_subject_status_t
BSP_subject_notify(bsp_subject_t *subject, const bsp_event_t *event);

#ifdef __cplusplus
    }
#endif

#endif /// BSP_HK_OBSERVER_
