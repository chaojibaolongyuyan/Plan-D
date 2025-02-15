/*
*********************************************************************************************************
*                                               uC/OS-II
*                                         The Real-Time Kernel
*
*                         (c) Copyright 2002, Jean J. Labrosse, Weston, FL
*                                          All Rights Reserved
*
*                                       PAGED S12XE Specific code
*                                           (Codewarrior V4.5)
*                                                 
*
* File         : OS_CPU_C.C
* By           : Eric Shufro
* Port Version : V2.81 and higher)
*********************************************************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "uCOS_II.H"
#include "derivative.h"

/*
*********************************************************************************************************
*                                           LOCALS
*********************************************************************************************************
*/

#if (OS_TMR_EN > 0)
static  INT16U  OSTmrCtr;
#endif

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                            (BEGINNING)
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSInitHookBegin (void)
{
#if OS_TMR_EN > 0
    OSTmrCtr = 0;
#endif
}
#endif

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                               (END)
*
* Description: This function is called by OSInit() at the end of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSInitHookEnd (void)
{
#if OS_CPU_INT_DIS_MEAS_EN > 0
    OS_CPU_IntDisMeasInit();
#endif
}
#endif

/*$PAGE*/
/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0)
void  OSTaskCreateHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskCreateHook(ptcb);
#else
    (void)ptcb;                                                 /* Prevent compiler warning                             */
#endif
}
#endif


/*
*********************************************************************************************************
*                                           TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0)
void  OSTaskDelHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskDelHook(ptcb);
#else
    (void)ptcb;                                                 /* Prevent compiler warning                             */
#endif
}
#endif

/*
*********************************************************************************************************
*                                             IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do  
*              such things as STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are enabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION >= 251)
void  OSTaskIdleHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskIdleHook();
#endif
}
#endif

/*
*********************************************************************************************************
*                                           STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-II's statistics task.  This allows your 
*              application to add functionality to the statistics task.
*
* Arguments  : none
*********************************************************************************************************
*/

#if (OS_CPU_HOOKS_EN > 0)
void  OSTaskStatHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskStatHook();
#endif
}
#endif

/*$PAGE*/
/*
*********************************************************************************************************
*                                        INITIALIZE A TASK'S STACK
*
* Description: This function is called by either OSTaskCreate() or OSTaskCreateExt() to initialize the
*              stack frame of the task being created.  This function is highly processor specific.
*
* Arguments  : task          is a pointer to the task code
*
*              p_arg         is a pointer to a user supplied data area that will be passed to the task
*                            when the task first executes.
*
*              ptos          is a pointer to the top of stack.  It is assumed that 'ptos' points to
*                            a 'free' entry on the task stack.  If OS_STK_GROWTH is set to 1 then 
*                            'ptos' will contain the HIGHEST valid address of the stack.  Similarly, if
*                            OS_STK_GROWTH is set to 0, the 'ptos' will contains the LOWEST valid address
*                            of the stack.
*
*              opt           specifies options that can be used to alter the behavior of OSTaskStkInit().
*                            (see uCOS_II.H for OS_TASK_OPT_???).
*
* Returns    : Always returns the location of the new top-of-stack' once the processor registers have
*              been placed on the stack in the proper order.
*
* Note(s)    : 1) XIRQ interrupts are disabled when your task starts executing. You can change this by 
*                 clearing BIT6 in the CCR.
*              2) The STOP instruction is disabled when your task starts executing.  You can change this
*                 by clearing BIT7 in the CCR.
*              3) The other interrupts (i.e. maskable interrupts) are enabled when your task starts
*                 executing.  You can change this by setting BIT4 in the CCR.
*              4) You can change pass the above options in the 'opt' argument.  You MUST only use the
*                 upper 8 bits of 'opt' because the lower bits are reserved by uC/OS-II.  If you make
*                 changes to the code below, you will need to ensure that it doesn't affect the behaviour
*                 of OSTaskIdle() and OSTaskStat().
*              5) Registers are initialized to make them easy to differentiate with a debugger.
*              6) Take the current values of GPAGE, EPAGE and RPAGE
*********************************************************************************************************
*/

OS_STK *OSTaskStkInit (void (*task)(void *pd), void *p_arg, OS_STK *ptos, INT16U opt)
{
    //INT16U *wstk;
    //INT8U  *bstk;

    //opt     =  opt;                                             /* 'opt' is not used, prevent warning                   */
    //wstk    =  (INT16U *)ptos;                                  /* Load stack pointer                                   */
    //*--wstk =  (INT16U)p_arg;                                   /* Simulate call to function with argument              */
    //*--wstk =  (INT16U)(((INT32U)task) >> 8);                   /* Return address of simulated call. Format: PCH:PCL    */
    //*--wstk =  (INT16U)(((INT32U)task) >> 8);                   /* Put task return address on top of stack              */
    //*--wstk =  (INT16U)0x2222;                                  /* Y Register                                           */
    //*--wstk =  (INT16U)0x1111;                                  /* X Register                                           */
    //*--wstk =  (INT16U)0xBBAA;                                  /* D Register                                           */
    //*--wstk =  (INT16U)0x0080;                                  /* CCR: Disable STOP, Int. Level = 0. User Mode         */
    //bstk    =  (INT8U *)wstk;                                   /* Convert WORD ptr to BYTE ptr to set CCR              */
    //*--bstk = *(INT8U *)0x10;                                   /* Save the GPAGE register (see note 6)                 */
    //*--bstk = *(INT8U *)0x17;                                   /* Save the EPAGE register (see note 6)                 */
    //*--bstk = *(INT8U *)0x16;                                   /* Save the RPAGE register (see note 6)                 */
    //*--bstk =  (INT8U  )task;                                   /* Save the task's PPAGE register value                 */
    //return ((OS_STK *)bstk);                                    /* Return pointer to new top-of-stack                   */

    INT16U *stk;
    
    stk=(INT16U *)ptos;
    *--stk=opt;
    *--stk=(INT16U)(task);
    *--stk=(INT16U)(task);
    *--stk=(INT16U)(0x1122);
    *--stk=(INT16U)(0x3344);
    ((INT8U *)stk)--;
    *(INT8U *)stk=(INT8U)(((INT16U)p_arg)>>8);
    ((INT8U *)stk)--;
    *(INT8U *)stk=(INT8U)(p_arg); 
    ((INT8U *)stk)--;
    *(INT8U *)stk=(INT8U)(0x00);
    ((INT8U *)stk)--;
    *(INT8U *)stk=(INT8U)(0x00);
    //((INT8U *)stk)--;
    //*(INT8U *)stk=*(INT8U*)p_arg;
    
    return((OS_STK *)stk);    
}

/*$PAGE*/
/*
*********************************************************************************************************
*                                           TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other
*              operations during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                 will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the 
*                 task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
void  OSTaskSwHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskSwHook();
#endif
}
#endif

/*
*********************************************************************************************************
*                                           OSTCBInit() HOOK
*
* Description: This function is called by OS_TCBInit() after setting up most of the TCB.
*
* Arguments  : ptcb    is a pointer to the TCB of the task being created.
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSTCBInitHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TCBInitHook(ptcb);
#else
    (void)ptcb;                                                 /* Prevent compiler warning                             */
#endif
}
#endif


/*
*********************************************************************************************************
*                                           I/O PORT ADDRESSES
*********************************************************************************************************
*/
//#define PPAGE 0x0015         // Addres of PPAGE register (assuming MC9S12XEP100 part)
//#define PPAGE 0x0030
//#define RPAGE 0x0016         // Addres of RPAGE register (assuming MC9S12XEP100 part)
//#define EPAGE 0x0017         // Addres of EPAGE register (assuming MC9S12XEP100 part)
//#define GPAGE 0x0010         // Addres of GPAGE register (assuming MC9S12XEP100 part)


/*
*********************************************************************************************************
*                               START HIGHEST PRIORITY TASK READY-TO-RUN
*
* Description : This function is called by OSStart() to start the highest priority task that was created
*               by your application before calling OSStart().
*
* Arguments   : none
*
* Note(s)     : 1) The stack frame is assumed to look as follows:
*   
*                  OSTCBHighRdy->OSTCBStkPtr +  0  -->  gPAGE
*                                            +  1       ePAGE
*                                            +  2       rPAGE
*                                            +  3       pPAGE
*                                            +  1       CCRW
*                                            +  2       B
*                                            +  3       A
*                                            +  4       X (H)
*                                            +  5       X (L)
*                                            +  6       Y (H)
*                                            +  7       Y (L)
*                                            +  8       PC(H)
*                                            +  9       PC(L)
*
*               2) OSStartHighRdy() MUST:
*                      a) Call OSTaskSwHook() then,
*                      b) Set OSRunning to TRUE,
*                      c) Switch to the highest priority task by loading the stack pointer of the
*                         highest priority task into the SP register and execute an RTI instruction.
*********************************************************************************************************
*/
void OSStartHighRdy(void)
{
  OSTaskSwHook();                      // Invoke user defined context switch hook            

  asm
  {
    ldab   #$01                        // Indicate that we are multitasking
    stab   OSRunning
           
    ldx    OSTCBHighRdy                // Point to TCB of highest priority task ready to run 
    lds    0,x                         // Load SP into 68HC12
    
    /*pula                               // Get value of PPAGE register
    staa   PPAGE                       // Store into CPU's PPAGE register                                

    pula                               // Get value of RPAGE register
    staa   RPAGE                       // Store into CPU's RPAGE register                                

    pula                               // Get value of EPAGE register
    staa   EPAGE                       // Store into CPU's EPAGE register                                

    pula                               // Get value of GPAGE register
    staa   GPAGE                       // Store into CPU's GPAGE register*/   
    
    rti                                // Run task   
  }
}


/*
*********************************************************************************************************
*                                       TASK LEVEL CONTEXT SWITCH
*
* Description : This function is called when a task makes a higher priority task ready-to-run.
*
* Arguments   : none
*
* Note(s)     : 1) Upon entry, 
*                  OSTCBCur     points to the OS_TCB of the task to suspend
*                  OSTCBHighRdy points to the OS_TCB of the task to resume
*
*               2) The stack frame of the task to suspend looks as follows:
*
*                  SP            CCR
*                     +  2       B
*                     +  3       A
*                     +  4       X (H)
*                     +  5       X (L)
*                     +  6       Y (H)
*                     +  7       Y (L)
*                     +  8       PC(H)
*                     +  9       PC(L)
*
*               3) The stack frame of the task to resume looks as follows:
* 
*                  OSTCBHighRdy->OSTCBStkPtr +  0  -->  gPAGE
*                                            +  1       ePAGE
*                                            +  2       rPAGE
*                                            +  3       pPAGE
*                                            +  4       CCR
*                                            +  6       B
*                                            +  7       A
*                                            +  8       X (H)
*                                            +  9       X (L)
*                                            + 10       Y (H)
*                                            + 11       Y (L)
*                                            + 12       PC(H)
*                                            + 13       PC(L)
*********************************************************************************************************
*/
void OSCtxSw(void)
{
  asm
  {
    /*ldaa   GPAGE                       // Get current value of GPAGE register                                
    psha                               // Push GPAGE register onto current task's stack

    ldaa   EPAGE                       // Get current value of EPAGE register                                
    psha                               // Push EPAGE register onto current task's stack

    ldaa   RPAGE                       // Get current value of RPAGE register                                
    psha                               // Push RPAGE register onto current task's stack

    ldaa   PPAGE                       // Get current value of PPAGE register                                
    psha                               // Push PPAGE register onto current task's stack*/
    
    ldy    OSTCBCur                    // OSTCBCur->OSTCBStkPtr = Stack Pointer     
    sts    0,y
  }
  OSTaskSwHook();                      // Call user task switch hook                       
  OSTCBCur=OSTCBHighRdy;
  OSPrioCur=OSPrioHighRdy;  
  asm
  {
    ldx    OSTCBCur
    
    lds    0,x                         // Load SP into 68HC12                              
    
    /*pula                               // Get value of PPAGE register
    staa   PPAGE                       // Store into CPU's PPAGE register                                
        
    pula                               // Get value of RPAGE register
    staa   RPAGE                       // Store into CPU's RPAGE register                                

    pula                               // Get value of EPAGE register
    staa   EPAGE                       // Store into CPU's EPAGE register                                

    pula                               // Get value of GPAGE register
    staa   GPAGE                       // Store into CPU's GPAGE register*/ 
    
    rti                                // Run task                                         
  }
}


/*
*********************************************************************************************************
*                                               TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TIME_TICK_HOOK_EN > 0) 
void  OSTimeTickHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TimeTickHook();
#endif

#if OS_TMR_EN > 0
    OSTmrCtr++;
    if (OSTmrCtr >= (OS_TICKS_PER_SEC / OS_TMR_CFG_TICKS_PER_SEC)) {
        OSTmrCtr = 0;
        OSTmrSignal();
    }
#endif
}
#endif


/*
*********************************************************************************************************
*                                    INTERRUPT LEVEL CONTEXT SWITCH
*
* Description : This function is called by OSIntExit() to perform a context switch to a task that has
*               been made ready-to-run by an ISR. The GPAGE, EPAGE, RPAGE and PPAGE CPU registers of the 
*               preempted task have already been stacked during the start of the ISR that is currently 
*               running.
*
* Arguments   : none
*********************************************************************************************************
*/
void OSIntCtxSw(void)
{
  OSTaskSwHook();                      // Call user task switch hook                
  OSTCBCur=OSTCBHighRdy;
  OSPrioCur=OSPrioHighRdy;   
  asm
  {
    ldx    OSTCBCur
    
    lds    0,x                         // Load the SP of the next task
                      
    /*pula                               // Get value of PPAGE register
    staa   PPAGE                       // Store into CPU's PPAGE register                                
                         
    pula                               // Get value of RPAGE register
    staa   RPAGE                       // Store into CPU's RPAGE register                                
                            
    pula                               // Get value of EPAGE register
    staa   EPAGE                       // Store into CPU's EPAGE register                                
                               
    pula                               // Get value of GPAGE register
    staa   GPAGE                       // Store into CPU's GPAGE register*/     
                                  
    rti                                // Run task  
  }
}


/*
*********************************************************************************************************
*                                           SYSTEM TICK ISR
*
* Description : This function is the ISR used to notify uC/OS-II that a system tick has occurred.  You 
*               must setup the S12XE's interrupt vector table so that a Real-Time Interrupt vectors to 
*               this function.
*
* Arguments   : none
*
* Notes       :  1) The 'tick ISR' assumes the we are using the Output Compare specified by OS_TICK_OC
*                   (see APP_CFG.H and this file) to generate a tick that occurs every OS_TICK_OC_CNTS 
*                   (see APP_CFG.H) which corresponds to the number of FRT (Free Running Timer) 
*                   counts to the next interrupt.
*
*                2) All USER interrupts should be modeled EXACTLY like this where the only
*                   line to be modified is the call to your ISR_Handler and perhaps the call to
*                   the label name OSTickISR1.
*********************************************************************************************************
*/
void OSTickISR(void)
{ 
  /*asm
  {
    ldaa   GPAGE                       // Get current value of GPAGE register                                
    psha                               // Push GPAGE register onto current task's stack

    ldaa   EPAGE                       // Get current value of EPAGE register                                
    psha                               // Push EPAGE register onto current task's stack

    ldaa   RPAGE                       // Get current value of RPAGE register                                
    psha                               // Push RPAGE register onto current task's stack

    ldaa   PPAGE                       // Get current value of PPAGE register                                
    psha                               // Push PPAGE register onto current task's stack
  } */
  OSIntNesting++;
  if(OSIntNesting == 1)
  {
    asm
    {
      ldy    OSTCBCur                  // OSTCBCur->OSTCBStkPtr = Stack Pointer     
      sts    0,y                                                                 
    }
  }
  //_FEED_COP();                         /* feeds the dog */
  CRGFLG_RTIF=1;                       //清除实时中断标志位   
  OSTimeTick();                        /* Notify uC/OS-II that a tick has occurred                */
  OSIntExit();
  asm
  {
    /*pula                               // Get value of PPAGE register
    staa   PPAGE                       // Store into CPU's PPAGE register                                
    
    pula                               // Get value of RPAGE register
    staa   RPAGE                       // Store into CPU's RPAGE register                                

    pula                               // Get value of EPAGE register
    staa   EPAGE                       // Store into CPU's EPAGE register                                

    pula                               // Get value of GPAGE register
    staa   GPAGE                       // Store into CPU's GPAGE register*/                                
            
    rti                                // Return from interrupt, no higher priority tasks ready.
  }
}

