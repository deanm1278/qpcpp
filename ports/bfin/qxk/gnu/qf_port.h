/// @file
/// @brief QF/C++ port to Blackfin+, dual-mode QXK kernel, GNU toolset
/// @cond
///***************************************************************************
/// Last Updated for Version: 6.1.1
/// Date of the Last Update:  2018-03-05
///
///                    Q u a n t u m     L e a P s
///                    ---------------------------
///                    innovating embedded systems
///
/// Copyright (C) 2005-2018 Quantum Leaps, LLC. All rights reserved.
///
/// This program is open source software: you can redistribute it and/or
/// modify it under the terms of the GNU General Public License as published
/// by the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// Alternatively, this program may be distributed and modified under the
/// terms of Quantum Leaps commercial licenses, which expressly supersede
/// the GNU General Public License and are specifically designed for
/// licensees interested in retaining the proprietary status of their code.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program. If not, see <http://www.gnu.org/licenses/>.
///
/// Contact information:
/// https://www.state-machine.com
/// mailto:info@state-machine.com
///***************************************************************************
/// @endcond

#ifndef qf_port_h
#define qf_port_h

// The maximum number of active objects in the application, see NOTE1
#define QF_MAX_ACTIVE           32

// The maximum number of system clock tick rates
#define QF_MAX_TICK_RATE        2

// QF interrupt disable/enable and log2()...
#define QF_LOG2(n_) QF_qlog2((n_))
extern "C" {
    extern volatile unsigned int __imask;
};

// interrupt disabling policy
#define QF_INT_DISABLE() __asm volatile (\
    "cli R7; %0 = R7;" : "=r"(__imask) : : "R7")
#define QF_INT_ENABLE()  __asm volatile (\
    "R7 = %0; sti R7;" : : "r"(__imask) : "R7")

// QF critical section entry/exit (unconditional interrupt disabling)
//#define QF_CRIT_STAT_TYPE not defined
#define QF_CRIT_ENTRY(dummy) QF_INT_DISABLE()
#define QF_CRIT_EXIT(dummy)  QF_INT_ENABLE()

//TODO: these
// BASEPRI threshold for "QF-aware" interrupts
#define QF_BASEPRI           0

// CMSIS threshold for "QF-aware" interrupts
#define QF_AWARE_ISR_CMSIS_PRI 0

//TODO: should we do this?
#define QF_CRIT_EXIT_NOP()      __asm volatile ("CSYNC;")

#include "qep_port.h" // QEP port

// hand-optimized quick LOG2 in assembly
extern "C" uint_fast8_t QF_qlog2(uint32_t x);

#include "qxk_port.h" // QXK dual-mode kernel port
#include "qf.h"       // QF platform-independent public interface
#include "qxthread.h" // QXK extended thread

//****************************************************************************
// NOTE1:
// The maximum number of active objects QF_MAX_ACTIVE can be increased
// up to 64, if necessary. Here it is set to a lower level to save some RAM.
//

#endif // qf_port_h

