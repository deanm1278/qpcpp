/**
* @file
* @brief QXK/C++ port to blackfin, GNU-ARM toolset
* @cond
******************************************************************************
* Last Updated for Version: 6.3.2
* Date of the Last Update:  2018-06-22
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2005-2018 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Contact information:
* https://www.state-machine.com
* mailto:info@state-machine.com
******************************************************************************
* @endcond
*/
#include "qf_port.h"

extern "C" {

/* prototypes --------------------------------------------------------------*/
void QXK_stackInit_(void *act, QP::QActionHandler thread,
                    void *stkSto, uint_fast16_t stkSize);
void PendSV_Handler(void);
void NMI_Handler(void);
void Thread_ret(void);

/*
* Initialize the exception priorities and IRQ priorities to safe values.
*
* Description:
* On Cortex-M3/M4/M7, this QXK port disables interrupts by means of the
* BASEPRI register. However, this method cannot disable interrupt
* priority zero, which is the default for all interrupts out of reset.
* The following code changes the SysTick priority and all IRQ priorities
* to the safe value QF_BASEPRI, wich the QF critical section can disable.
* This avoids breaching of the QF critical sections in case the
* application programmer forgets to explicitly set priorities of all
* "kernel aware" interrupts.
*
* The interrupt priorities established in QXK_init() can be later
* changed by the application-level code.
*/
void QXK_init(void) {
    //set REG_ICU_EVT14 to call PendSV_Handler
    __asm volatile( 
    "  P0 = 0x1FC02038 ;        \n" // P0 = ICU_EVT14
    "  R0.H = _PendSV_Handler ; \n"
    "  R0.L = _PendSV_Handler ; \n"
    "  [P0] = R0 ;              \n"
    ::: "P0", "R0" );
}

/*****************************************************************************
* Initialize the private stack of an extended QXK thread.
*
* NOTE: the function aligns the stack to the 8-byte boundary for compatibility
* with the AAPCS. Additionally, the function pre-fills the stack with the
* known bit pattern (0xDEADBEEF).
*
* NOTE: QXK_stackInit_() must be called before the QXK kernel is made aware
* of this thread. In that case the kernel cannot use the thread yet, so no
* critical section is needed.
*****************************************************************************/
void QXK_stackInit_(void *act, QP::QActionHandler thread,
                    void *stkSto, uint_fast16_t stkSize)
{
    extern void QXK_threadRet_(void); /* extended thread return */

    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp =
        (uint32_t *)((((uint32_t)stkSto + stkSize) >> 3) << 3);
    uint32_t *sp_limit;

    /* synthesize the ARM Cortex-M exception stack frame...*/
    *(--sp) = (1U << 24);    /* xPSR  (just the THUMB bit) */
    *(--sp) = (uint32_t)thread;          /* PC (the thread routine) */
    *(--sp) = (uint32_t)&QXK_threadRet_; /* LR (return from thread) */
    *(--sp) = 0x0000000CU;   /* R12 */
    *(--sp) = 0x00000003U;   /* R3  */
    *(--sp) = 0x00000002U;   /* R2  */
    *(--sp) = 0x00000001U;   /* R1  */
    *(--sp) = (uint32_t)act; /* R0 (argument to the thread routine */
    *(--sp) = 0x0000000BU;   /* R11 */
    *(--sp) = 0x0000000AU;   /* R10 */
    *(--sp) = 0x00000009U;   /* R9  */
    *(--sp) = 0x00000008U;   /* R8  */
    *(--sp) = 0x00000007U;   /* R7  */
    *(--sp) = 0x00000006U;   /* R6  */
    *(--sp) = 0x00000005U;   /* R5  */
    *(--sp) = 0x00000004U;   /* R4  */

    /* save the top of the stack in the thread's attibute */
    static_cast<QP::QActive *>(act)->m_osObject = sp;

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    sp_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) >> 3) + 1U) << 3);
    for (; sp >= sp_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }
}

volatile unsigned int __imask;

/*****************************************************************************
* Thread_ret is a helper function executed when the QXK activator returns.
*
* NOTE: Thread_ret does not execute in the PendSV context!
* NOTE: Thread_ret executes entirely with interrupts DISABLED.
*****************************************************************************/
//__attribute__ ((naked, optimize("-fno-stack-protector")))
void Thread_ret(void) {
__asm volatile (
    ""
#if 0
    /* After the QXK activator returns, we need to resume the preempted
    * thread. However, this must be accomplished by a return-from-exception,
    * while we are still in the thread context. The switch to the exception
    * contex is accomplished by triggering the NMI exception.
    * NOTE: The NMI exception is triggered with nterrupts DISABLED,
    * because QXK activator disables interrutps before return.
    */

    /* trigger NMI to return to preempted task...
    * NOTE: The NMI exception is triggered with nterrupts DISABLED
    */
    "  LDR     r0,=0xE000ED04   \n" /* Interrupt Control and State Register */
    "  MOV     r1,#1            \n"
    "  LSL     r1,r1,#31        \n" /* r1 := (1 << 31) (NMI bit) */
    "  STR     r1,[r0]          \n" /* ICSR[31] := 1 (pend NMI) */
    "  B       .                \n" /* wait for preemption by NMI */
#endif
    );
}

/*****************************************************************************
* The NMI_Handler exception handler is used for returning back to the
* interrupted task. The NMI exception simply removes its own interrupt
* stack frame from the stack and returns to the preempted task using the
* interrupt stack frame that must be at the top of the stack.
*
* NOTE: The NMI exception is entered with interrupts DISABLED, so it needs
* to re-enable interrupts before it returns to the preempted task.
*****************************************************************************/
//__attribute__ ((naked, optimize("-fno-stack-protector")))
void NMI_Handler(void) {
__asm volatile (
    ""
#if 0
    "  ADD     sp,sp,#(8*4)     \n" /* remove one 8-register exception frame */

    "  CPSIE   i                \n" /* enable interrupts (clear PRIMASK) */
    "  BX      lr               \n" /* return to the preempted task */
#endif
    );
}

/*****************************************************************************
* hand-optimized quick LOG2 in assembly (bfin has no CLZ instruction)
*****************************************************************************/

//__attribute__ ((naked, optimize("-fno-stack-protector")))
uint_fast8_t QF_qlog2(uint32_t x) {
__asm volatile (
    ""
#if 0
    "  MOV     r1,#0            \n"
    "  LSR     r2,r0,#16        \n"
    "  BEQ     QF_qlog2_1       \n"
    "  MOV     r1,#16           \n"
    "  MOV     r0,r2            \n"
    "QF_qlog2_1:                \n"
    "  LSR     r2,r0,#8         \n"
    "  BEQ     QF_qlog2_2       \n"
    "  ADD     r1, r1,#8        \n"
    "  MOV     r0, r2           \n"
    "QF_qlog2_2:                \n"
    "  LSR     r2,r0,#4         \n"
    "  BEQ     QF_qlog2_3       \n"
    "  ADD     r1,r1,#4         \n"
    "  MOV     r0,r2            \n"
    "QF_qlog2_3:                \n"
    "  LDR     r2,=QF_qlog2_LUT \n"
    "  LDRB    r0,[r2,r0]       \n"
    "  ADD     r0,r1, r0        \n"
    "  BX      lr               \n"
    "QF_qlog2_LUT:              \n"
    "  .byte 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4"
#endif
    );
}

} // extern "C"

