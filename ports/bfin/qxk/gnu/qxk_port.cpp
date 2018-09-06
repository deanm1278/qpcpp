/**
* @file
* @brief QXK/C++ port to blackfin, GNU-BFIN toolset
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

    //TODO:
#if 0
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
#endif
}

volatile unsigned int __imask;

/*****************************************************************************
* hand-optimized quick LOG2 in assembly (bfin has no real CLZ instruction)
*****************************************************************************/

//__attribute__ ((naked, optimize("-fno-stack-protector")))
uint_fast8_t QF_qlog2(uint32_t x) {
	int c = 31;
__asm volatile (
	"CC = BITTST(R0, 31);	\n"
	"IF !CC JUMP log2_clz;	\n"
	"R0 >>= 1;				\n"
"log2_clz:					\n"
    "R0.L = SIGNBITS %0;	\n"
	"R0 = R0.L (Z);			\n"
	"R0 = %1 - R0;			\n"
    :: "d"(x), "d"(c));
}

} // extern "C"

