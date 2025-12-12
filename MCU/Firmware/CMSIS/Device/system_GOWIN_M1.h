/*
 ******************************************************************************************
 * @file      system_GOWIN_M1.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     CMSIS Cortex-M1 Device Peripheral Access Layer Header File.
 ******************************************************************************************
 */

#ifndef SYSTEM_GOWIN_M1_H
#define SYSTEM_GOWIN_M1_H

#ifdef __cplusplus
extern "C" {
#endif

extern unsigned int SystemCoreClock;     /*!< System Clock Frequency (Core Clock) */


/**
  \brief Setup the microcontroller system.

   Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);


/**
  \brief  Update SystemCoreClock variable.

   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_GOWIN_M1_H */
