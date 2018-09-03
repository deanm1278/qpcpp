///***************************************************************************
// Product: DPP example, STM32 NUCLEO-L152RE board, preemptive QK kernel
// Last updated for version 5.6.5
// Last updated on  2016-07-05
//
//                    Q u a n t u m     L e a P s
//                    ---------------------------
//                    innovating embedded systems
//
// Copyright (C) Quantum Leaps, LLC. All rights reserved.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
//
// Contact information:
// http://www.state-machine.com
// mailto:info@state-machine.com
//****************************************************************************

#include <bf706_device.h>
#include "qpcpp.h"
#include "bsp.h"
#include <stdlib.h>

void *operator new(size_t size) throw() {
    return malloc(size);
}
//............................................................................
void operator delete(void *p) throw() {
    free(p);
}
//............................................................................
void operator delete(void *p, unsigned long) throw() {
    free(p);
}

//Q_DEFINE_THIS_FILE

#define ENABLE_BSP_PRINT

volatile uint32_t _systemMs = 0;

void BspInit() {
      //SEC_Global->SEC0_GCTL.bit.RESET = 1;
    //SCI->SEC0_CCTL0.bit.RESET = 1;

    //systick timer enable
    TMR->TCNTL.bit.PWR = 1;
    TMR->TCNTL.bit.AUTORLD = 1;
    TMR->TSCALE.reg = 1; //decrement every 2 clock cycles
    TMR->TPERIOD.reg = (400000000UL)/2000; //1 per ms
    TMR->TCNTL.bit.EN = 1;

    //enable interrupts
    SEC_Global->SEC0_GCTL.bit.EN = 1;
    SCI->SEC0_CCTL0.bit.EN = 1;
}

void BspWrite(char const *buf, uint32_t len) {
	//TODO:
}

uint32_t GetSystemMs() {
    return _systemMs;
}

extern "C" {
	void systick_handler(void)
    {
        QXK_ISR_ENTRY();
		QP::QF::tickX_(0);
        _systemMs++;
        QXK_ISR_EXIT();
    }
};

// namespace QP **************************************************************
namespace QP {

// QF callbacks ==============================================================
void QF::onStartup(void) {
	
    // assigning all priority bits for preemption-prio. and none to sub-prio.
    
    // set priorities of ALL ISRs used in the system, see NOTE00
    //
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    // DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    //NVIC_SetPriority(EXTI0_1_IRQn,   DPP::EXTI0_1_PRIO);
    // ...
}

//............................................................................
void QF::onCleanup(void) {
}
//............................................................................
void QXK::onIdle(void) {
    // toggle the User LED on and then off (not enough LEDs, see NOTE01)
    //QF_INT_DISABLE();

    //QF_INT_ENABLE();
    __asm__ volatile ("IDLE;");
}

//............................................................................
extern "C" void Q_onAssert(char const * const module, int loc) {
	//
    // NOTE: add here your application-specific error handling
    //

    QF_INT_DISABLE();
#ifdef ENABLE_LOGGING
    char __ms[50];
    sprintf(__ms, "[%li] ***QASSERT**** ", GetSystemMs());
    writeDataUART(CONFIG_LOG_SERCOM, __ms);
    writeDataUART(CONFIG_LOG_SERCOM, module);
    sprintf(__ms, " at %i", loc);
    writeDataUART(CONFIG_LOG_SERCOM, __ms);
#endif
    __asm__ volatile ("EMUEXCPT;");
	while(1);
}

extern "C" void assert_failed(char const *module, int loc) {
	__asm__ volatile ("EMUEXCPT;");
	while(1);
}

} // namespace QP
