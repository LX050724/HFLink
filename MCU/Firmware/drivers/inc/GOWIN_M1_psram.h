/*
 *********************************************************************************************
 * @file      GOWIN_M1_psram.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the PSRAM firmware library.
 *********************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_PSRAM_H
#define GOWIN_M1_PSRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "GOWIN_M1.h"

/* Macros --------------------------------------------------------------------*/
#define WRITE_MODE 1    /* Write mode      */
#define READ_MODE  0    /* Read mode       */
#define CMD_ENABLE 1    /* Enable command  */
#define CMD_UNABLE 0    /* Disable command */

/* Functions ------------------------------------------------------------------*/

/**
  * @brief Check PSRAM hardware initialization status.
  */
extern uint32_t PSRAM_Check_Init_Status(void);

/**
  * @brief Set PSRAM read/write mode.
  */
extern uint32_t PSRAM_Mode_Set(uint32_t mode);

/**
  * @brief Set PSRAM address, save data into this address.
  */
extern uint32_t PSRAM_Address_Set(uint32_t address);

/**
  * @brief Read data from PSRAM buffer.
  */
extern uint32_t PSRAM_Read_Data_Buff(uint32_t *Buff,uint32_t address);

/**
  * @brief Enable PSRAM command.
  */
extern void PSRAM_Cmd_Enable(void);

/**
  * @brief The flag of read PSRAM done.
  */
extern uint32_t PSRAM_Read_Done_Flag(void);

/**
  * @brief Clear the flag of read PSRAM done.
  */
extern void PSRAM_Clear_Read_Done_Flag(void);

/**
  * @brief Write data into PSRAM buffer, 4 word is a cycle.
  */
extern uint32_t PSRAM_Write_Data_Buff(uint32_t *Buff,uint32_t address);

/**
  * @brief Disable PSRAM command.
  */
extern void PSRAM_Cmd_Unable(void);

/**
  * @brief Write a package data into PSRAM.
  */
extern uint32_t PSRAM_Write_Data_Package(uint32_t *Buff,uint32_t address,uint32_t num);

/**
  * @brief Read a package data from PSRAM.
  */
extern uint32_t PSRAM_Read_Data_Package(uint32_t *Buff,uint32_t address,uint32_t num);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_PSRAM_H */
