
#ifndef __ASSEMBLY__
#define __ASSEMBLY__
#endif

#include <bf706_device.h>

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

//mask everything but IVG13
#define THREAD_RET_MASK 0x201F

#define SAVE_CONTEXT \
	[--sp] = ( R7:1, P5:2 ); \
	R1 = RETI; \
	RETI = R1; \
	[--sp] = R1; \
	[--sp] = fp; \
\
	[--sp] = i0; \
	[--sp] = i1; \
	[--sp] = i2; \
	[--sp] = i3; \
\
	[--sp] = m0; \
	[--sp] = m1; \
	[--sp] = m2; \
	[--sp] = m3; \
\
	[--sp] = l0; \
	[--sp] = l1; \
	[--sp] = l2; \
	[--sp] = l3; \
\
	[--sp] = b0; \
	[--sp] = b1; \
	[--sp] = b2; \
	[--sp] = b3; \
	[--sp] = a0.x; \
	[--sp] = a0.w; \
	[--sp] = a1.x; \
	[--sp] = a1.w; \
\
	[--sp] = LC0; \
	[--sp] = LC1; \
	[--sp] = LT0; \
	[--sp] = LT1; \
	[--sp] = LB0; \
	[--sp] = LB1; \
\
	[--sp] = RETS; \
	[--sp] = SEQSTAT; \
	[--sp] = SYSCFG

#define RESTORE_CONTEXT \
	SYSCFG = [sp++]; \
	SEQSTAT = [sp++]; \
	RETS = [sp++]; \
\
	LB1 = [sp++]; \
	LB0 = [sp++]; \
	LT1 = [sp++]; \
	LT0 = [sp++]; \
	LC1 = [sp++]; \
	LC0 = [sp++]; \
\
	a1.w = [sp++]; \
	a1.x = [sp++]; \
	a0.w = [sp++]; \
	a0.x = [sp++]; \
	b3 = [sp++]; \
	b2 = [sp++]; \
	b1 = [sp++]; \
	b0 = [sp++]; \
\
	l3 = [sp++]; \
	l2 = [sp++]; \
	l1 = [sp++]; \
	l0 = [sp++]; \
\
	m3 = [sp++]; \
	m2 = [sp++]; \
	m1 = [sp++]; \
	m0 = [sp++]; \
\
	i3 = [sp++]; \
	i2 = [sp++]; \
	i1 = [sp++]; \
	i0 = [sp++]; \
\
	fp = [sp++]; \
	R0 = [sp++]; \
	RETI = R0; \
\
	( R7 : 1, P5 : 2) = [ SP ++ ]; \
	ASTAT = [SP++]; \
	P0 = [SP++]

/*
* Set up the IVG routing.
*/

.align 2
.global _QXK_init;
.type _QXK_init, STT_FUNC;

_QXK_init:
    LINK 0;
	[--SP] = P0;
	[--SP] = R1;

	//Set handler for IVG13
	P0.H = HI(REG_ICU_EVT13);
	P0.L = LO(REG_ICU_EVT13);
	R1.L = _NMI_Handler
	R1.H = _NMI_Handler
	[P0] = R1;

	//Set handler for IVG14
	P0.H = HI(REG_ICU_EVT14);
	P0.L = LO(REG_ICU_EVT14);
	R1.L = _PendSV_Handler
	R1.H = _PendSV_Handler
	[P0] = R1;

	R1 = [SP++];
	P0 = [SP++];
	UNLINK;
	RTS;

/*****************************************************************************
* Thread_ret is a helper function executed when the QXK activator returns.
*
* NOTE: Thread_ret does not execute in the PendSV context!
* NOTE: Thread_ret executes entirely with interrupts DISABLED.
*****************************************************************************/

.align 2
.global _Thread_ret;
.type _Thread_ret, STT_FUNC;
_Thread_ret:
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
	RAISE 13;
	R7 = THREAD_RET_MASK (Z); /* mask off everything but IVG13 */
	STI R7;
	CSYNC;
	IDLE;

/*****************************************************************************
* The NMI_Handler exception handler is used for returning back to the
* interrupted task. The NMI exception restores the context of the preemted
* task.
*
* NOTE: The NMI exception is entered with interrupts DISABLED, so it needs
* to re-enable interrupts before it returns to the preempted task.
*****************************************************************************/

.align 2
.global _NMI_Handler;
.type _NMI_Handler, STT_FUNC;
_NMI_Handler:
	RESTORE_CONTEXT;
	P1.H = ___imask; //re-enable interrupts
	P1.L = ___imask;
	R0 = [P1];
	STI R0;
	P1 = [SP++];
	R0 = [SP++];
	RTI;

/*****************************************************************************
* The PendSV_Handler exception handler is used for handling context switch
* and asynchronous preemption in QXK. The use of the PendSV exception is
* the recommended and most efficient method for performing context switches
* with ARM Cortex-M.
*
* The PendSV exception should have the lowest priority in the whole system
* (see QXK_init). All other exceptions and interrupts should have higher
* priority.
*
* Also, *all* "kernel aware" ISRs in the QXK application must call the
* QXK_ISR_EXIT() macro, which triggers PendSV when it detects a need for
* a context switch or asynchronous preemption.
*
*****************************************************************************/

.align 2
.global _PendSV_Handler;
.type _PendSV_Handler, STT_FUNC;

.extern _QXK_attr_;
.type _QXK_attr_,STT_OBJECT;

.extern ___imask;
.type ___imask,STT_OBJECT;

_PendSV_Handler:
	[--SP] = R0;
	[--SP] = P1;
	[--SP] = P0;
	[--SP] = ASTAT;

    /* disable interrupts */
    P1.H = ___imask;
    P1.L = ___imask;
    CLI R0;
    [P1] = R0;

	P1.H = _QXK_attr_;
	P1.L = _QXK_attr_;

	P0 = [P1+QXK_NEXT]; 			/*P0 = QXK_attr_.next */

	/* Check QXK_attr_.next, which contains the pointer to the next thread
    * to run, which is set in QXK_ISR_EXIT(). This pointer must not be NULL.
    */
	CC = P0 == 0;					/* is (QXK_attr_.next == 0)? */
    IF CC JUMP PendSV_return;		/* branch if (QXK_attr_.next == 0) */

    SAVE_CONTEXT;

    /* Load pointers into registers... */
    R7 = [P0+QMACTIVE_OSOBJ];		/* r7 := QXK_attr_.next->osObject */
    P2 = [P1+QXK_CURR]				/* P2 := QXK_attr_.curr */

    CC = P2 == 0;					/* (QXK_attr_.curr == 0)? */
	IF !CC JUMP PendSV_save_ex;      /* branch if (current thread is extended) */

    CC = R7 == 0;            		/* (QXK_attr_.next->osObject == 0)? */
    IF !CC JUMP PendSV_save_ao    	/* branch if (next tread is extended) */

PendSV_activate:
	/* The QXK activator must be called in a thread context, while this code
    * executes in the handler contex of the PendSV exception. The switch
    * to the Thread mode is accomplished by returning from PendSV using
    * a fabricated exception stack frame, where the return address is
    * QXK_activate_().
    *
    * NOTE: the QXK activator is called with interrupts DISABLED and also
    * it returns with interrupts DISABLED.
    */
    R0.H = _QXK_activate_;
    R0.L = _QXK_activate_;
    RETI = R0;					/* return to QXK_activate_ from this interrupt */

    R0.H = _Thread_ret;
    R0.L = _Thread_ret;
    RETS = R0;					 /* return address after the call */

    RTI;						/* return */

	 /*=========================================================================
    * Saving AO-thread before crossing to eXtended-thread
    * expected register contents:
    * P4  -> QXK_attr_.next
    * P2  -> QXK_attr_.curr
    * r7  -> QXK_attr_.next->osObject (SP)
    * P5  -> &QXK_attr_
    */
PendSV_save_ao:
	//TODO:
	/* otherwise continue to restoring next AO-thread... */

	/*-------------------------------------------------------------------------
	* Restoring AO-thread after crossing from eXtended-thread
	* expected register contents:
	* r1  -> QXK_attr_.curr
	* r2  -> QXK_attr_.next->osObject (SP)
	* r3  -> &QXK_attr_
	* r12 -> QXK_attr_.next
	*/
PendSV_restore_ao:
	//TODO:

PendSV_return:
	ASTAT = [SP++];
	P0 = [SP++];
	P1 = [SP++];
	R0 = [SP++];

PendSV_save_ex:

PendSV_restore_ex:

