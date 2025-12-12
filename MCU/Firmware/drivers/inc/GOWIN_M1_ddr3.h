/*
 ********************************************************************************************
 * @file      GOWIN_M1_ddr3.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the DDR3 firmware library.
 ********************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_DDR3_H
#define GOWIN_M1_DDR3_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "GOWIN_M1.h"

/* Functions ------------------------------------------------------------------*/
/**
  * @brief DDR3 Initialization.
  */
extern uint8_t DDR3_Init(void);
	 
/**
  * @brief Read data from DDR3.
  */
extern void DDR3_Read(uint32_t addr, uint32_t *rd_buff);
	 
/**
  * @brief Write data into DDR3
  */
extern void DDR3_Write(uint32_t addr, uint32_t *wr_buff);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DDR3_H */
