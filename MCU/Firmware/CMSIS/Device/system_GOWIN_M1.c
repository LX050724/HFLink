/*
 *******************************************************************************************
 * @file      system_GOWIN_M1.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     CMSIS Cortex-M1 Device Peripheral Access Layer System Source File.
 *            This file contains the system clock configuration for Cortex-M1 device.
 *
 *            This file provides two functions and one global variable to be called from
 *            user application:
 *              - SystemInit(): Setups the system clock.
 *                              This function is called at startup just after reset and
 *                              before branch to main program. This call is mad inside
 *                              the "startup_GOWIN_M1.s" file.
 *              - SystemCoreClock variable: Contains the core clock, it can be used
 *                              by the user application to setup the SysTick
 *                              timer or configure other parameters.
 *              - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
 *                              be called whenever the core clock is changed
 *                              during program execution.
 *******************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#define SYSTEM_CLOCK (60000000UL) /* 60MHz                */

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
unsigned int SystemCoreClock = SYSTEM_CLOCK; /* System Core Clock Frequency */

/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/
/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 * @brief  Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = SYSTEM_CLOCK;
}

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
/**
 * @param  none
 * @return none
 * @brief  Setup the Cortex-M1 system.
 *         Initialize the System.
 */
__WEAK void SystemInit(void)
{
    uint32_t i, size;
    SystemCoreClock = SYSTEM_CLOCK;

    extern uint8_t __bss_start__[], __bss_end__[];
    extern uint8_t __data_start__[], __data_end__[];
    extern uint8_t __data_load_addr__[];
    extern uint8_t __itcm_start__[], __itcm_end__[];
    extern uint8_t __itcm_loadaddr__[];
    // extern uint8_t __isr_vector_start__[], __isr_vector_end__[];

    size = __data_end__ - __data_start__;
    for (i = 0; i < size; i += sizeof(uint32_t))
    {
        *(uint32_t *)(__data_start__ + i) = *(uint32_t *)(__data_load_addr__ + i);
    }

    size = __itcm_end__ - __itcm_start__;
    for (i = 0; i < size; i += sizeof(uint32_t))
    {
        *(uint32_t *)(__itcm_end__ + i) = *(uint32_t *)(__itcm_loadaddr__ + i);
    }

    size = __bss_end__ - __bss_start__;
    for (i = 0; i < size; i += sizeof(uint32_t))
    {
        *(uint32_t *)(__bss_start__ + i) = 0;
    }

    // __enable_irq();
}
