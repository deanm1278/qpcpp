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

/* NOTE: keep in synch with the QXK_Attr struct in "qxk.h" !!! */
#define QXK_CURR       0
#define QXK_NEXT       4
#define QXK_ACT_PRIO   8
#define QXK_IDLE_THR   12

/* NOTE: keep in synch with the QXK_Attr struct in "qxk.h" !!! */
/*Q_ASSERT_COMPILE(QXK_CURR == offsetof(QXK_Attr, curr));*/
/*Q_ASSERT_COMPILE(QXK_NEXT == offsetof(QXK_Attr, next));*/
/*Q_ASSERT_COMPILE(QXK_ACT_PRIO == offsetof(QXK_Attr, actPrio));*/

/* NOTE: keep in synch with the QMActive struct in "qf.h/qxk.h" !!! */
#define QMACTIVE_OSOBJ 28
#define QMACTIVE_PRIO  36

/* NOTE: keep in synch with the QActive struct in "qf.h/qxk.h" !!! */
/*Q_ASSERT_COMPILE(QMACTIVE_OSOBJ == offsetof(QActive, osObject));*/
/*Q_ASSERT_COMPILE(QMACTIVE_PRIO == offsetof(QActive, prio));*/

/* helper macros to "stringify" values */
#define VAL(x) #x
#define STRINGIFY(x) VAL(x)

/*****************************************************************************
* The PendSV_Handler exception handler is used for handling context switch
* and asynchronous preemption in QXK. The use of the PendSV exception is
* the recommended and most efficient method for performing context switches
* with ARM Cortex-M.
*
* The PendSV exception should have the lowest priority in the whole system
* (0xFF, see QXK_init). All other exceptions and interrupts should have higher
* priority. For example, for NVIC with 2 priority bits all interrupts and
* exceptions must have numerical value of priority lower than 0xC0. In this
* case the interrupt priority levels available to your applications are (in
* the order from the lowest urgency to the highest urgency): 0x80, 0x40, 0x00.
*
* Also, *all* "kernel aware" ISRs in the QXK application must call the
* QXK_ISR_EXIT() macro, which triggers PendSV when it detects a need for
* a context switch or asynchronous preemption.
*
* Due to tail-chaining and its lowest priority, the PendSV exception will be
* entered immediately after the exit from the *last* nested interrupt (or
* exception). In QXK, this is exactly the time when the QXK activator needs to
* handle the asynchronous preemption.
*
*****************************************************************************/

volatile unsigned int __imask;

//__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler(void) {
__asm volatile (
    "EMUEXCPT; " //TODO: ok now we need to do context switch
#if 0
    /* Prepare some constants before entering a critical section... */
    "  LDR     r3,=QXK_attr_    \n"
    "  LDR     r2,=" STRINGIFY(NVIC_ICSR) "\n" /* Interrupt Control and State */
    "  MOV     r1,#1            \n"
    "  LSL     r1,r1,#27        \n" /* r0 := (1 << 27) (UNPENDSVSET bit) */

    /*<<<<<<<<<<<<<<<<<<<<<<< CRITICAL SECTION BEGIN <<<<<<<<<<<<<<<<<<<<<<<<*/
    "  CPSID   i                \n" /* disable interrupts (set PRIMASK) */

    /* The PendSV exception handler can be preempted by an interrupt,
    * which might pend PendSV exception again. The following write to
    * ICSR[27] un-pends any such spurious instance of PendSV.
    */
    "  STR     r1,[r2]          \n" /* ICSR[27] := 1 (unpend PendSV) */

    /* Check QXK_attr_.next, which contains the pointer to the next thread
    * to run, which is set in QXK_ISR_EXIT(). This pointer must not be NULL.
    */
    "  LDR     r0,[r3,#" STRINGIFY(QXK_NEXT) "]\n" /* r1 := QXK_attr_.next */
    "  CMP     r0,#0            \n" /* is (QXK_attr_.next == 0)? */
    "  BEQ     PendSV_return    \n" /* branch if (QXK_attr_.next == 0) */

    /* Load pointers into registers... */
    "  MOV     r12,r0           \n" /* save QXK_attr_.next in r12 */
    "  LDR     r2,[r0,#" STRINGIFY(QMACTIVE_OSOBJ) "]\n" /* r2 := QXK_attr_.next->osObject */
    "  LDR     r1,[r3,#" STRINGIFY(QXK_CURR) "]\n" /* r1 := QXK_attr_.curr */

    "  CMP     r1,#0            \n" /* (QXK_attr_.curr != 0)? */
    "  BNE     PendSV_save_ex   \n" /* branch if (current thread is extended) */

    "  CMP     r2,#0            \n" /* (QXK_attr_.next->osObject != 0)? */
    "  BNE     PendSV_save_ao   \n" /* branch if (next tread is extended) */

    "PendSV_activate:           \n"

    /* The QXK activator must be called in a thread context, while this code
    * executes in the handler contex of the PendSV exception. The switch
    * to the Thread mode is accomplished by returning from PendSV using
    * a fabricated exception stack frame, where the return address is
    * QXK_activate_().
    *
    * NOTE: the QXK activator is called with interrupts DISABLED and also
    * it returns with interrupts DISABLED.
    */
    "  MOV     r3,#1            \n"
    "  LSL     r3,r3,#24        \n" /* r3 := (1 << 24), set the T bit  (new xpsr) */
    "  LDR     r2,=QXK_activate_\n" /* address of QXK_activate_ */
    "  SUB     r2,r2,#1         \n" /* align Thumb-address at halfword (new pc) */
    "  LDR     r1,=Thread_ret   \n" /* return address after the call   (new lr) */

    "  SUB     sp,sp,#(8*4)     \n" /* reserve space for exception stack frame */
    "  ADD     r0,sp,#(5*4)     \n" /* r0 := 5 registers below the top of stack */
    "  STM     r0!,{r1-r3}      \n" /* save xpsr,pc,lr */

    "  MOV     r0,#6            \n"
    "  MVN     r0,r0            \n" /* r0 := ~6 == 0xFFFFFFF9 */
    "  BX      r0               \n" /* exception-return to the QXK activator */

    /*=========================================================================
    * Saving AO-thread before crossing to eXtended-thread
    * expected register contents:
    * r0  -> QXK_attr_.next
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    "PendSV_save_ao:            \n"
    "  SUB     sp,sp,#(8*4)     \n" /* make room for 8 registers r4-r11 */
    "  MOV     r0,sp            \n" /* r0 := temporary stack pointer */
    "  STMIA   r0!,{r4-r7}      \n" /* save the low registers */
    "  MOV     r4,r8            \n" /* move the high registers to low registers... */
    "  MOV     r5,r9            \n"
    "  MOV     r6,r10           \n"
    "  MOV     r7,r11           \n"
    "  STMIA   r0!,{r4-r7}      \n" /* save the high registers */
    "  MOV     r0,r12           \n" /* restore QXK_attr_.next in r0 */

    "  CMP     r2,#0            \n"
    "  BNE     PendSV_restore_ex\n" /* branch if (QXK_attr_.next->osObject != 0) */
    /* otherwise continue to restoring next AO-thread... */

    /*-------------------------------------------------------------------------
    * Restoring AO-thread after crossing from eXtended-thread
    * expected register contents:
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    "PendSV_restore_ao:         \n"
    "  MOV     r0,#0            \n"
    "  STR     r0,[r3,#" STRINGIFY(QXK_CURR) "]\n" /* QXK_attr_.curr := 0 */
    /* don't clear QXK_attr_.next, as it might be needed for AO activation */

    "  MOV     r0,sp            \n" /* r0 := top of stack */
    "  MOV     r2,r0            \n"
    "  ADD     r2,r2,#(4*4)     \n" /* point r2 to the 4 high registers r7-r11 */
    "  LDMIA   r2!,{r4-r7}      \n" /* pop the 4 high registers into low registers */
    "  MOV     r8,r4            \n" /* move low registers into high registers */
    "  MOV     r9,r5            \n"
    "  MOV     r10,r6           \n"
    "  MOV     r11,r7           \n"
    "  LDMIA   r0!,{r4-r7}      \n" /* pop the low registers */
    "  ADD     sp,sp,#(8*4)     \n" /* remove 8 registers from the stack */

    "  MOV     r2,#6            \n"
    "  MVN     r2,r2            \n" /* r2 := ~6 == 0xFFFFFFF9 */
    "  MOV     lr,r2            \n" /* make sure MSP is used */

    "  MOV     r0,r12           \n" /* r0 := QXK_attr_.next */
    "  MOV     r2,#" STRINGIFY(QMACTIVE_PRIO) "\n" /* r2 := offset of .next into QActive */
    "  LDRB    r0,[r0,r2]       \n" /* r0 := QXK_attr_.next->prio */
    "  LDRB    r2,[r3,#" STRINGIFY(QXK_ACT_PRIO) "]\n" /* r2 := QXK_attr_.actPrio */
    "  CMP     r2,r0            \n"
    "  BCC     PendSV_activate  \n" /* if (next->prio > topPrio) activate the next AO */

    /* otherwise no activation needed... */
    "  MOV     r0,#0            \n"
    "  STR     r0,[r3,#" STRINGIFY(QXK_NEXT) "]\n" /* QXK_attr_.next := 0 (clear the next) */

#ifdef QXK_ON_CONTEXT_SW
    "  MOV     r0,r1            \n" /* r0 := QXK_attr_.curr */
    "  MOV     r1,r12           \n" /* r1 := QXK_attr_.next */
    "  LDR     r2,[r3,#" STRINGIFY(QXK_IDLE_THR) "]\n" /* r2 := idle thr ptr */
    "  CMP     r1,r2            \n"
    "  BNE     PendSV_onContextSw1 \n" /* if (next != idle) call onContextSw */
    "  MOVS    r1,#0            \n" /* otherwise, next := NULL */
    "PendSV_onContextSw1:        \n"
    "  LDR     r2,=QXK_onContextSw \n"
    "  PUSH    {r1,lr}          \n" /* save the aligner + exception lr */
    "  BLX     r2               \n" /* call QXK_onContextSw() */
    "  POP     {r1,r2}          \n" /* restore the aligner + lr into r2 */
    "  MOV     lr,r2            \n" /* restore the exception lr */
#endif /* QXK_ON_CONTEXT_SW */

    /* re-enable interrupts and return from PendSV */
    "PendSV_return:             \n"
    "  CPSIE   i                \n" /* enable interrupts (clear PRIMASK) */

    /*>>>>>>>>>>>>>>>>>>>>>>>> CRITICAL SECTION END >>>>>>>>>>>>>>>>>>>>>>>>>*/
    "  BX      lr               \n" /* return to the preempted AO-thread */

    /*-------------------------------------------------------------------------
    * Saving extended-thread before crossing to AO-thread
    * expected register contents:
    * r0  -> QXK_attr_.next
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    "PendSV_save_ex:            \n"
    "  MRS     r0,PSP           \n" /* r0 := Process Stack Pointer */
    "  SUB     r0,r0,#(8*4)     \n" /* make room for 8 registers r4-r11 */
    "  MOV     r1,r0            \n" /* r1 := temporary PSP (do not clobber r0!) */
    "  STMIA   r1!,{r4-r7}      \n" /* save the low registers */
    "  MOV     r4,r8            \n" /* move the high registers to low registers... */
    "  MOV     r5,r9            \n"
    "  MOV     r6,r10           \n"
    "  MOV     r7,r11           \n"
    "  STMIA   r1!,{r4-r7}      \n" /* save the high registers */
    /* NOTE: at this point r0 holds the top of stack */

    "  LDR     r1,[r3,#" STRINGIFY(QXK_CURR) "]\n" /* r1 := QXK_attr_.curr (restore value) */


    /* store the SP of the current extended-thread */
    "  STR     r0,[r1,#" STRINGIFY(QMACTIVE_OSOBJ) "]\n" /* QXK_attr_.curr->osObject := r0 */
    "  MOV     r0,r12           \n" /* QXK_attr_.next (restore value) */

    "  CMP     r2,#0            \n"
    "  BEQ     PendSV_restore_ao\n" /* branch if (QXK_attr_.next->osObject == 0) */
    /* otherwise continue to restoring next extended-thread... */

    /*-------------------------------------------------------------------------
    * Restoring extended-thread after crossing from AO-thread
    * expected register contents:
    * r0  -> QXK_attr_.next
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    "PendSV_restore_ex:         \n"
#ifdef QXK_ON_CONTEXT_SW
    "  MOV     r0,r1            \n" /* r0 := QXK_attr_.curr */
    "  MOV     r1,r12           \n" /* r1 := QXK_attr_.next */
    "  LDR     r2,[r3,#" STRINGIFY(QXK_IDLE_THR) "]\n" /* r2 := idle thr ptr */
    "  CMP     r0,r2            \n"
    "  BNE     PendSV_onContextSw2 \n" /* if (curr != idle) call onContextSw */
    "  MOV     r0,#0            \n" /* otherwise, curr := NULL */
    "PendSV_onContextSw2:        \n"
    "  LDR     r3,=QXK_onContextSw \n"
    "  BLX     r3               \n" /* call QXK_onContextSw() */

    /* restore the AAPCS-clobbered registers after a functin call...  */
    "  LDR     r3,=QXK_attr_    \n"
    "  LDR     r0,[r3,#" STRINGIFY(QXK_NEXT) "]\n" /* r0 := QXK_attr_.next */
    "  LDR     r2,[r0,#" STRINGIFY(QMACTIVE_OSOBJ) "]\n" /* r2 := QXK_attr_.curr->osObject */
#endif /* QXK_ON_CONTEXT_SW */

    "  STR     r0,[r3,#" STRINGIFY(QXK_CURR) "]\n" /* QXK_attr_.curr := r0 (QXK_attr_.next) */
    "  MOV     r0,#0            \n"
    "  STR     r0,[r3,#" STRINGIFY(QXK_NEXT) "]\n" /* QXK_attr_.next := 0 */

    /* exit the critical section */
    "  CPSIE   i                \n" /* enable interrupts (clear PRIMASK) */

    "  MOV     r0,r2            \n" /* r2 := top of stack */
    "  ADD     r0,r0,#(4*4)     \n" /* point r0 to the 4 high registers r7-r11 */
    "  LDMIA   r0!,{r4-r7}      \n" /* pop the 4 high registers into low registers */
    "  MOV     r8,r4            \n" /* move low registers into high registers */
    "  MOV     r9,r5            \n"
    "  MOV     r10,r6           \n"
    "  MOV     r11,r7           \n"
    "  LDMIA   r2!,{r4-r7}      \n" /* pop the low registers */
    "  MOV     r2,r0            \n" /* r2 := holds the new top of stack */

    "  MOV     r1,#2            \n"
    "  MVN     r1,r1            \n" /* r1 := ~2 == 0xFFFFFFFD */
    "  MOV     lr,r1            \n" /* make sure PSP is used */

    /* set the PSP to the next thread's SP */
    "  MSR     PSP,r2           \n" /* Process Stack Pointer := r2 */

    "  BX      lr               \n" /* return to the next extended-thread */
#endif //0
    );
}

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

