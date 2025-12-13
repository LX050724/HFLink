.syntax unified
.cpu cortex-m1
.fpu softvfp
.thumb

.global  g_pfnVectors
.global  Default_Handler

    .section  .text.Reset_Handler
  .weak Reset_Handler
  .type  Reset_Handler, %function

Reset_Handler:
  ldr r0, =_estack
  mov sp, r0      /* set stack pointer */
  /* Call the clock system initialization function.*/
  bl  SystemInit
  /* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl  main
  bx  lr
.size  Reset_Handler, .-Reset_Handler

  .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler



  .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler

  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  /* external interrupts handler */
  .word    UART0_Handler                      /*   0 Interrupt UART0 */
  .word    UART1_Handler                      /*   1 Interrupt UART1 */
  .word    TIMER0_Handler                     /*   2 Interrupt Timer0 */
  .word    TIMER1_Handler                     /*   3 Interrupt Timer1 */
  .word    GPIO0_Handler                      /*   4 Interrupt GPIO0 */
  .word    UARTOVF_Handler                    /*   5 Interrupt UARTOVF */
  .word    RTC_Handler                        /*   6 Interrupt RTC */
  .word    I2C_Handler                        /*   7 Interrupt I2C */
  .word    CAN_Handler                        /*   8 Interrupt CAN */
  .word    ENT_Handler                        /*   9 Interrupt Ethernet */
  .word    EXTINT_0_Handler                   /*   10 Interrupt External 0 */
  .word    DTimer_Handler                     /*   11 Interrupt DualTimer */
  .word    TRNG_Handler                       /*   12 Interrupt TRNG */
  .word    EXTINT_1_Handler                   /*   13 Interrupt External 1 */
  .word    EXTINT_2_Handler                   /*   14 Interrupt External 2 */
  .word    EXTINT_3_Handler                   /*   15 Interrupt External 3 */
  .word    GPIO0_0_Handler                    /*   16 Interrupt GPIO0_0 */
  .word    GPIO0_1_Handler                    /*   17 Interrupt GPIO0_1 */
  .word    GPIO0_2_Handler                    /*   18 Interrupt GPIO0_2 */
  .word    GPIO0_3_Handler                    /*   19 Interrupt GPIO0_3 */
  .word    GPIO0_4_Handler                    /*   20 Interrupt GPIO0_4 */
  .word    GPIO0_5_Handler                    /*   21 Interrupt GPIO0_5 */
  .word    GPIO0_6_Handler                    /*   22 Interrupt GPIO0_6 */
  .word    GPIO0_7_Handler                    /*   23 Interrupt GPIO0_7 */
  .word    GPIO0_8_Handler                    /*   24 Interrupt GPIO0_8 */
  .word    GPIO0_9_Handler                    /*   25 Interrupt GPIO0_9 */
  .word    GPIO0_10_Handler                   /*   26 Interrupt GPIO0_10 */
  .word    GPIO0_11_Handler                   /*   27 Interrupt GPIO0_11 */
  .word    GPIO0_12_Handler                   /*   28 Interrupt GPIO0_12 */
  .word    GPIO0_13_Handler                   /*   29 Interrupt GPIO0_13 */
  .word    GPIO0_14_Handler                   /*   30 Interrupt GPIO0_14 */
  .word    GPIO0_15_Handler                   /*   31 Interrupt GPIO0_15 */
  .size  g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
  .weak WWDGT_IRQHandler
  .thumb_set WWDGT_IRQHandler, Default_Handler
*******************************************************************************/
  .weak UART0_Handler
  .thumb_set UART0_Handler, Default_Handler

  .weak UART1_Handler
  .thumb_set UART1_Handler, Default_Handler

  .weak TIMER0_Handler
  .thumb_set TIMER0_Handler, Default_Handler

  .weak TIMER1_Handler
  .thumb_set TIMER1_Handler, Default_Handler

  .weak GPIO0_Handler
  .thumb_set GPIO0_Handler, Default_Handler

  .weak UARTOVF_Handler
  .thumb_set UARTOVF_Handler, Default_Handler

  .weak RTC_Handler
  .thumb_set RTC_Handler, Default_Handler

  .weak I2C_Handler
  .thumb_set I2C_Handler, Default_Handler

  .weak CAN_Handler
  .thumb_set CAN_Handler, Default_Handler

  .weak ENT_Handler
  .thumb_set ENT_Handler, Default_Handler

  .weak EXTINT_0_Handler
  .thumb_set EXTINT_0_Handler, Default_Handler

  .weak DTimer_Handler
  .thumb_set DTimer_Handler, Default_Handler

  .weak TRNG_Handler
  .thumb_set TRNG_Handler, Default_Handler

  .weak EXTINT_1_Handler
  .thumb_set EXTINT_1_Handler, Default_Handler

  .weak EXTINT_2_Handler
  .thumb_set EXTINT_2_Handler, Default_Handler

  .weak EXTINT_3_Handler
  .thumb_set EXTINT_3_Handler, Default_Handler

  .weak GPIO0_0_Handler
  .thumb_set GPIO0_0_Handler, Default_Handler

  .weak GPIO0_1_Handler
  .thumb_set GPIO0_1_Handler, Default_Handler

  .weak GPIO0_2_Handler
  .thumb_set GPIO0_2_Handler, Default_Handler

  .weak GPIO0_3_Handler
  .thumb_set GPIO0_3_Handler, Default_Handler

  .weak GPIO0_4_Handler
  .thumb_set GPIO0_4_Handler, Default_Handler

  .weak GPIO0_5_Handler
  .thumb_set GPIO0_5_Handler, Default_Handler

  .weak GPIO0_6_Handler
  .thumb_set GPIO0_6_Handler, Default_Handler

  .weak GPIO0_7_Handler
  .thumb_set GPIO0_7_Handler, Default_Handler

  .weak GPIO0_8_Handler
  .thumb_set GPIO0_8_Handler, Default_Handler

  .weak GPIO0_9_Handler
  .thumb_set GPIO0_9_Handler, Default_Handler

  .weak GPIO0_10_Handler
  .thumb_set GPIO0_10_Handler, Default_Handler

  .weak GPIO0_11_Handler
  .thumb_set GPIO0_11_Handler, Default_Handler

  .weak GPIO0_12_Handler
  .thumb_set GPIO0_12_Handler, Default_Handler

  .weak GPIO0_13_Handler
  .thumb_set GPIO0_13_Handler, Default_Handler

  .weak GPIO0_14_Handler
  .thumb_set GPIO0_14_Handler, Default_Handler

  .weak GPIO0_15_Handler
  .thumb_set GPIO0_15_Handler, Default_Handler
