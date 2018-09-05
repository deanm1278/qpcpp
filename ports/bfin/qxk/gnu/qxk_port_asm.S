
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

.align 2
.global _PendSV_Handler;
.type _PendSV_Handler, STT_FUNC;

.extern _QXK_attr_;
.type _QXK_attr_,STT_OBJECT;

_PendSV_Handler:
	[--SP] = R7;
	[--SP] = R6;
	[--SP] = P2;
	[--SP] = P5;
	[--SP] = P4;
	[--SP] = P3;
	[--SP] = ASTAT;

	P5.H = _QXK_attr_;
	P5.L = _QXK_attr_;

	//TODO: disable interrupts here if we enable nesting

	P4 = [P5+QXK_NEXT]; 			/*P4 = QXK_attr_.next */

	/* Check QXK_attr_.next, which contains the pointer to the next thread
    * to run, which is set in QXK_ISR_EXIT(). This pointer must not be NULL.
    */
	CC = P4 == 0;					/* is (QXK_attr_.next == 0)? */
    IF CC JUMP PendSV_return;		/* branch if (QXK_attr_.next == 0) */

    /* Load pointers into registers... */
    R7 = [P4+QMACTIVE_OSOBJ];		/* r7 := QXK_attr_.next->osObject */
    P2 = [P5+QXK_CURR]				/* P2 := QXK_attr_.curr */

    CC = P2 == 0;					/* (QXK_attr_.curr == 0)? */
	IF !CC JUMP PendSV_save_ex;      /* branch if (current thread is extended) */

    CC = R7 == 0;            		/* (QXK_attr_.next->osObject == 0)? */
    IF !CC JUMP PendSV_save_ao    	/* branch if (next tread is extended) */

PendSV_activate:
	//TODO: leaving off here

	/* The QXK activator must be called in a thread context, while this code
    * executes in the handler contex of the PendSV exception. The switch
    * to the Thread mode is accomplished by returning from PendSV using
    * a fabricated exception stack frame, where the return address is
    * QXK_activate_().
    *
    * NOTE: the QXK activator is called with interrupts DISABLED and also
    * it returns with interrupts DISABLED.
    */
      MOV     r3,#1
      LSL     r3,r3,#24         /* r3 := (1 << 24), set the T bit  (new xpsr) */
      LDR     r2,=QXK_activate_ /* address of QXK_activate_ */
      SUB     r2,r2,#1          /* align Thumb-address at halfword (new pc) */
      LDR     r1,=Thread_ret    /* return address after the call   (new lr) */

      SUB     sp,sp,#(8*4)      /* reserve space for exception stack frame */
      ADD     r0,sp,#(5*4)      /* r0 := 5 registers below the top of stack */
      STM     r0!,{r1-r3}       /* save xpsr,pc,lr */

      MOV     r0,#6
      MVN     r0,r0             /* r0 := ~6 == 0xFFFFFFF9 */
      BX      r0                /* exception-return to the QXK activator */

	 /*=========================================================================
    * Saving AO-thread before crossing to eXtended-thread
    * expected register contents:
    * P4  -> QXK_attr_.next
    * P2  -> QXK_attr_.curr
    * r7  -> QXK_attr_.next->osObject (SP)
    * P5  -> &QXK_attr_
    */
PendSV_save_ao:

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

PendSV_return:
	ASTAT = [SP++];
	P3 = [SP++];
	P4 = [SP++];
	P5 = [SP++];
	P2 = [SP++];
	R7 = [SP++];

PendSV_save_ex:

PendSV_restore_ex:

#if 0
    /* Prepare some constants before entering a critical section... */
      LDR     r3,=QXK_attr_    
      LDR     r2,= STRINGIFY(NVIC_ICSR)  /* Interrupt Control and State */
      MOV     r1,#1            
      LSL     r1,r1,#27         /* r0 := (1 << 27) (UNPENDSVSET bit) */

    /*<<<<<<<<<<<<<<<<<<<<<<< CRITICAL SECTION BEGIN <<<<<<<<<<<<<<<<<<<<<<<<*/
      CPSID   i                 /* disable interrupts (set PRIMASK) */

    /* The PendSV exception handler can be preempted by an interrupt,
    * which might pend PendSV exception again. The following write to
    * ICSR[27] un-pends any such spurious instance of PendSV.
    */
      STR     r1,[r2]           /* ICSR[27] := 1 (unpend PendSV) */

    /* Check QXK_attr_.next, which contains the pointer to the next thread
    * to run, which is set in QXK_ISR_EXIT(). This pointer must not be NULL.
    */
      LDR     r0,[r3,# STRINGIFY(QXK_NEXT) ] /* r1 := QXK_attr_.next */
      CMP     r0,#0             /* is (QXK_attr_.next == 0)? */
      BEQ     PendSV_return     /* branch if (QXK_attr_.next == 0) */

    /* Load pointers into registers... */
      MOV     r12,r0            /* save QXK_attr_.next in r12 */
      LDR     r2,[r0,# STRINGIFY(QMACTIVE_OSOBJ) ] /* r2 := QXK_attr_.next->osObject */
      LDR     r1,[r3,# STRINGIFY(QXK_CURR) ] /* r1 := QXK_attr_.curr */

      CMP     r1,#0             /* (QXK_attr_.curr != 0)? */
      BNE     PendSV_save_ex    /* branch if (current thread is extended) */

      CMP     r2,#0             /* (QXK_attr_.next->osObject != 0)? */
      BNE     PendSV_save_ao    /* branch if (next tread is extended) */

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
      MOV     r3,#1            
      LSL     r3,r3,#24         /* r3 := (1 << 24), set the T bit  (new xpsr) */
      LDR     r2,=QXK_activate_ /* address of QXK_activate_ */
      SUB     r2,r2,#1          /* align Thumb-address at halfword (new pc) */
      LDR     r1,=Thread_ret    /* return address after the call   (new lr) */

      SUB     sp,sp,#(8*4)      /* reserve space for exception stack frame */
      ADD     r0,sp,#(5*4)      /* r0 := 5 registers below the top of stack */
      STM     r0!,{r1-r3}       /* save xpsr,pc,lr */

      MOV     r0,#6            
      MVN     r0,r0             /* r0 := ~6 == 0xFFFFFFF9 */
      BX      r0                /* exception-return to the QXK activator */

    /*=========================================================================
    * Saving AO-thread before crossing to eXtended-thread
    * expected register contents:
    * r0  -> QXK_attr_.next
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    PendSV_save_ao:            
      SUB     sp,sp,#(8*4)      /* make room for 8 registers r4-r11 */
      MOV     r0,sp             /* r0 := temporary stack pointer */
      STMIA   r0!,{r4-r7}       /* save the low registers */
      MOV     r4,r8             /* move the high registers to low registers... */
      MOV     r5,r9            
      MOV     r6,r10           
      MOV     r7,r11           
      STMIA   r0!,{r4-r7}       /* save the high registers */
      MOV     r0,r12            /* restore QXK_attr_.next in r0 */

      CMP     r2,#0            
      BNE     PendSV_restore_ex /* branch if (QXK_attr_.next->osObject != 0) */
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
      MOV     r0,#0            
      STR     r0,[r3,# STRINGIFY(QXK_CURR) ] /* QXK_attr_.curr := 0 */
    /* don't clear QXK_attr_.next, as it might be needed for AO activation */

      MOV     r0,sp             /* r0 := top of stack */
      MOV     r2,r0            
      ADD     r2,r2,#(4*4)      /* point r2 to the 4 high registers r7-r11 */
      LDMIA   r2!,{r4-r7}       /* pop the 4 high registers into low registers */
      MOV     r8,r4             /* move low registers into high registers */
      MOV     r9,r5            
      MOV     r10,r6           
      MOV     r11,r7           
      LDMIA   r0!,{r4-r7}       /* pop the low registers */
      ADD     sp,sp,#(8*4)      /* remove 8 registers from the stack */

      MOV     r2,#6            
      MVN     r2,r2             /* r2 := ~6 == 0xFFFFFFF9 */
      MOV     lr,r2             /* make sure MSP is used */

      MOV     r0,r12            /* r0 := QXK_attr_.next */
      MOV     r2,# STRINGIFY(QMACTIVE_PRIO)  /* r2 := offset of .next into QActive */
      LDRB    r0,[r0,r2]        /* r0 := QXK_attr_.next->prio */
      LDRB    r2,[r3,# STRINGIFY(QXK_ACT_PRIO) ] /* r2 := QXK_attr_.actPrio */
      CMP     r2,r0            
      BCC     PendSV_activate   /* if (next->prio > topPrio) activate the next AO */

    /* otherwise no activation needed... */
      MOV     r0,#0            
      STR     r0,[r3,# STRINGIFY(QXK_NEXT) ] /* QXK_attr_.next := 0 (clear the next) */

#ifdef QXK_ON_CONTEXT_SW
      MOV     r0,r1             /* r0 := QXK_attr_.curr */
      MOV     r1,r12            /* r1 := QXK_attr_.next */
      LDR     r2,[r3,# STRINGIFY(QXK_IDLE_THR) ] /* r2 := idle thr ptr */
      CMP     r1,r2            
      BNE     PendSV_onContextSw1  /* if (next != idle) call onContextSw */
      MOVS    r1,#0             /* otherwise, next := NULL */
    PendSV_onContextSw1:        
      LDR     r2,=QXK_onContextSw 
      PUSH    {r1,lr}           /* save the aligner + exception lr */
      BLX     r2                /* call QXK_onContextSw() */
      POP     {r1,r2}           /* restore the aligner + lr into r2 */
      MOV     lr,r2             /* restore the exception lr */
#endif /* QXK_ON_CONTEXT_SW */

    /* re-enable interrupts and return from PendSV */
    PendSV_return:             
      CPSIE   i                 /* enable interrupts (clear PRIMASK) */

    /*>>>>>>>>>>>>>>>>>>>>>>>> CRITICAL SECTION END >>>>>>>>>>>>>>>>>>>>>>>>>*/
      BX      lr                /* return to the preempted AO-thread */

    /*-------------------------------------------------------------------------
    * Saving extended-thread before crossing to AO-thread
    * expected register contents:
    * r0  -> QXK_attr_.next
    * r1  -> QXK_attr_.curr
    * r2  -> QXK_attr_.next->osObject (SP)
    * r3  -> &QXK_attr_
    * r12 -> QXK_attr_.next
    */
    PendSV_save_ex:            
      MRS     r0,PSP            /* r0 := Process Stack Pointer */
      SUB     r0,r0,#(8*4)      /* make room for 8 registers r4-r11 */
      MOV     r1,r0             /* r1 := temporary PSP (do not clobber r0!) */
      STMIA   r1!,{r4-r7}       /* save the low registers */
      MOV     r4,r8             /* move the high registers to low registers... */
      MOV     r5,r9            
      MOV     r6,r10           
      MOV     r7,r11           
      STMIA   r1!,{r4-r7}       /* save the high registers */
    /* NOTE: at this point r0 holds the top of stack */

      LDR     r1,[r3,# STRINGIFY(QXK_CURR) ] /* r1 := QXK_attr_.curr (restore value) */


    /* store the SP of the current extended-thread */
      STR     r0,[r1,# STRINGIFY(QMACTIVE_OSOBJ) ] /* QXK_attr_.curr->osObject := r0 */
      MOV     r0,r12            /* QXK_attr_.next (restore value) */

      CMP     r2,#0            
      BEQ     PendSV_restore_ao /* branch if (QXK_attr_.next->osObject == 0) */
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
    PendSV_restore_ex:         
#ifdef QXK_ON_CONTEXT_SW
      MOV     r0,r1             /* r0 := QXK_attr_.curr */
      MOV     r1,r12            /* r1 := QXK_attr_.next */
      LDR     r2,[r3,# STRINGIFY(QXK_IDLE_THR) ] /* r2 := idle thr ptr */
      CMP     r0,r2            
      BNE     PendSV_onContextSw2  /* if (curr != idle) call onContextSw */
      MOV     r0,#0             /* otherwise, curr := NULL */
    PendSV_onContextSw2:        
      LDR     r3,=QXK_onContextSw 
      BLX     r3                /* call QXK_onContextSw() */

    /* restore the AAPCS-clobbered registers after a functin call...  */
      LDR     r3,=QXK_attr_    
      LDR     r0,[r3,# STRINGIFY(QXK_NEXT) ] /* r0 := QXK_attr_.next */
      LDR     r2,[r0,# STRINGIFY(QMACTIVE_OSOBJ) ] /* r2 := QXK_attr_.curr->osObject */
#endif /* QXK_ON_CONTEXT_SW */

      STR     r0,[r3,# STRINGIFY(QXK_CURR) ] /* QXK_attr_.curr := r0 (QXK_attr_.next) */
      MOV     r0,#0            
      STR     r0,[r3,# STRINGIFY(QXK_NEXT) ] /* QXK_attr_.next := 0 */

    /* exit the critical section */
      CPSIE   i                 /* enable interrupts (clear PRIMASK) */

      MOV     r0,r2             /* r2 := top of stack */
      ADD     r0,r0,#(4*4)      /* point r0 to the 4 high registers r7-r11 */
      LDMIA   r0!,{r4-r7}       /* pop the 4 high registers into low registers */
      MOV     r8,r4             /* move low registers into high registers */
      MOV     r9,r5            
      MOV     r10,r6           
      MOV     r11,r7           
      LDMIA   r2!,{r4-r7}       /* pop the low registers */
      MOV     r2,r0             /* r2 := holds the new top of stack */

      MOV     r1,#2            
      MVN     r1,r1             /* r1 := ~2 == 0xFFFFFFFD */
      MOV     lr,r1             /* make sure PSP is used */

    /* set the PSP to the next thread's SP */
      MSR     PSP,r2            /* Process Stack Pointer := r2 */

      BX      lr                /* return to the next extended-thread */
  #endif
