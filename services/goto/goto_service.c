/*******************************************************************************
 * Copyright 2019-2025 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software
 *
 */

/*!
 * \file Goto State Machine
 * \brief U54 goto
 */

#include "config.h"
#include "hss_types.h"
#include "hss_state_machine.h"
#include "hss_debug.h"

#include "ssmb_ipi.h"

#include <assert.h>

#include "csr_helper.h"

#include "ssmb_ipi.h"

#include "goto_service.h"

#if IS_ENABLED(CONFIG_SERVICE_BOOT)
#  include "hss_boot_pmp.h"
#endif

#include "mpfs_reg_map.h"
#include "csr_helper.h"

#include "hss_atomic.h"
#include "u54_state.h"

static void goto_init_handler(struct StateMachine * const pMyMachine);
static void goto_idle_handler(struct StateMachine * const pMyMachine);

/*!
 * \brief GOTO Driver States
 */
enum GotoStatesEnum {
    GOTO_INITIALIZATION,
    GOTO_IDLE,
    GOTO_NUM_STATES = GOTO_IDLE+1
};

/*!
 * \brief GOTO Driver State Descriptors
 */
static const struct StateDesc goto_state_descs[] = {
    { (const stateType_t)GOTO_INITIALIZATION, (const char *)"init",         NULL, NULL, &goto_init_handler },
    { (const stateType_t)GOTO_IDLE,           (const char *)"idle",         NULL, NULL, &goto_idle_handler },
};

/*!
 * \brief GOTO Driver State Machine
 */
struct StateMachine goto_service = {
    .state             = (stateType_t)GOTO_INITIALIZATION,
    .prevState         = (stateType_t)SM_INVALID_STATE,
    .numStates         = (const uint32_t)GOTO_NUM_STATES,
    .pMachineName      = (const char *)"goto_service",
    .startTime         = 0u,
    .lastExecutionTime = 0u,
    .executionCount    = 0u,
    .pStateDescs       = goto_state_descs,
    .debugFlag         = false,
    .priority          = 0u,
    .pInstanceData     = NULL,
};


// --------------------------------------------------------------------------------------------------
// Handlers for each state in the state machine
//
static void goto_init_handler(struct StateMachine * const pMyMachine)
{
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "called\n");
    pMyMachine->state++;
}


#include "hss_types.h"
#include "ssmb_ipi.h"
#include "goto_service.h"

void GOTO_ReleaseHarts(uintptr_t entry_point,TxId_t transaction_id);


// This function is called by the E51 to release the U54 harts
void GOTO_ReleaseHarts(uintptr_t entry_point,TxId_t transaction_id)
{
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "GOTO: Releasing U54_1 to 0x%p\n", entry_point);
    enum IPIStatusCode status = IPI_Send(HSS_HART_U54_1, IPI_MSG_OPENSBI_INIT, 0, PRV_M, (void*)0x80000000u, 0);


    if (status != IPI_SUCCESS) {
        mHSS_DEBUG_PRINTF(LOG_ERROR, "E51: IPI_Send FAILED with code %d\n", status);
    } else {
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "E51: IPI_Send SUCCESSFUL\n");
    }
}

////////////////
static void goto_idle_handler(struct StateMachine * const pMyMachine)
{
    (void)pMyMachine; // unused
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "called\n");

}
#if 0

/////////////////
static __attribute__((naked))
void hss_final_transition(unsigned long hartid, void* dtb, uintptr_t entry, unsigned long status) ;
static __attribute__((naked))
void hss_final_transition(unsigned long hartid, void* dtb, uintptr_t entry, unsigned long status) {
    __asm__ volatile (
        "csrw mepc, a2\n\t"
        "csrw mstatus, a3\n\t"
        "mret\n\t"
        : : "r"(hartid), "r"(dtb), "r"(entry), "r"(status) : "memory"
    );
}

#endif
#if 1
void examine_payload(uintptr_t addr, size_t bytes);
void examine_payload(uintptr_t addr, size_t bytes) {
    uint32_t *p = (uint32_t *)addr;
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "--- Payload Hex Dump at 0x%p ---\n", addr);
    for (int i = 0; i < (bytes / 4); i++) {
        mHSS_DEBUG_PRINTF(LOG_NORMAL, "%08x ", p[i]);
        if ((i + 1) % 4 == 0) mHSS_DEBUG_PRINTF(LOG_NORMAL, "\n");
    }
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "----------------------------------\n");
}

// Define the UART0 Line Status Register and the Empty bit
#define UART0_LSR (*((volatile uint32_t *)0x20000014u))
#define LSR_TEMT  0x40u // Transmitter Empty bit
#endif

enum IPIStatusCode HSS_GOTO_IPIHandler(TxId_t transaction_id, enum HSSHartId source,
    uint32_t immediate_arg, void *p_extended_buffer, void *p_ancilliary_buffer_in_ddr)
{
    const enum HSSHartId my_hartid = current_hartid();
    p_extended_buffer = (void*)(uint64_t)0x80000000u;

    // 1. ACKNOWLEDGE IMMEDIATELY
    // We must tell the E51 we got the message before we jump,
    // because after the jump, the HSS code on this hart is GONE.
    IPI_Send(source, IPI_MSG_ACK_COMPLETE, transaction_id, IPI_SUCCESS, NULL, NULL);

    if (source != HSS_HART_E51) {
        return IPI_FAIL;
    }

    mHSS_DEBUG_PRINTF(LOG_NORMAL, "u54_%d: Processing GOTO...\n", my_hartid);
#if 1
    if (p_extended_buffer != NULL) {
        /// Set state so E51 knows this Hart is now busy/running
        HSS_U54_SetState(HSS_State_Running);

        // Inside HSS_GOTO_IPIHandler
        // Use PMP Entry 0 as a "Global Pass"
        // 0x1FFFFFFFFFFFFFull is the encoding for "All Addresses" in NAPOT
        csr_write(pmpaddr0, 0x1FFFFFFFFFFFFFull);
        csr_write(pmpcfg0,  0x1Fu); // NAPOT | R | W | X

        // Clear any existing BEU errors before jumping
        // The BEU (Bus Error Unit) registers are at 0x01700000
        *((volatile uint64_t*)0x01700000) = 0ULL;

        // --- CLEANUP ---
        // --- HARDWARE DE-COUPLE ---
        //csr_write(mie, 0u);    // Disable all interrupts
        //csr_write(mip, 0u);    // Clear any pending interrupts

        // Clear the MSIP (Machine Software Interrupt) specifically
        // to tell the hardware this IPI is finished.
        /// CSR_ClearMSIP();

        // --- MSTATUS ---
        ///uint64_t mstatus_val = 0;
        ///mstatus_val = INSERT_FIELD(mstatus_val, MSTATUS_MPP, PRV_M);
        ///mstatus_val = INSERT_FIELD(mstatus_val, MSTATUS_MPIE, 1);
        ///mstatus_val = INSERT_FIELD(mstatus_val, MSTATUS_FS, 1);

        // --- UART FLUSH ---
        //while ((UART0_LSR & LSR_TEMT) == 0);

        examine_payload((uintptr_t) p_extended_buffer, 256);

        // Add this just before hss_final_transition
        uint64_t current_mstatus = csr_read(mstatus);
        uint32_t mpp_val = (uint32_t)((current_mstatus >> 11) & 0x3u);

        mHSS_DEBUG_PRINTF(LOG_NORMAL, "u54_%d: Transition Check - mstatus: 0x%lx, MPP: %u\n",
                          my_hartid, current_mstatus, mpp_val);

        if (mpp_val != 3) {
            mHSS_DEBUG_PRINTF(LOG_ERROR, "FATAL: Hart %d not configured for Machine Mode jump!\n", my_hartid);
        }

        // Force a complete pipeline and cache clear
        __asm__ volatile ("fence.i" ::: "memory");
        __asm__ volatile ("fence rw, rw" ::: "memory");
        // --- FINAL JUMP ---

        mHSS_DEBUG_PRINTF(LOG_NORMAL, "Jumping to payload now. Goodbye from HSS!\n");

        ///GOTO_ReleaseHarts((uintptr_t)p_extended_buffer, transaction_id);
        ///((void (*)(uintptr_t, uintptr_t))p_extended_buffer)(HSS_HART_U54_1, 0);
        ///
        // Use the HSS-native way to ensure registers are clean:
        extern void hss_final_transition(uintptr_t entry, uintptr_t stack, unsigned int hartid);

        // We jump to 0x80000000.
        // We let the payload assembly set its own stack, so we pass a dummy or 0.
        hss_final_transition(0x80000000u, 0u, 1);
    }
#endif
    return IPI_SUCCESS;
}
