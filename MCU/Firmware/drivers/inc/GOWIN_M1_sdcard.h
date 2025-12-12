/*
 ************************************************************************************************
 * @file      GOWIN_M1_sdcard.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the SD-Card firmware library.
 ************************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_SDCARD_H
#define GOWIN_M1_SDCARD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <ctype.h> /* for toupper() */
#include "GOWIN_M1.h"

/* Macros --------------------------------------------------------------------*/
#define TRANS_TYPE_MASK    				         ((uint32_t) 0x000000FF)  /* TRANS_TYPE_MASK 			*/
#define TRANS_TYPE_DIRECT_ACCESS           ((uint32_t) 0x00000000)  /* DIRECT_ACCESS 				*/
#define TRANS_TYPE_INIT_SD            		 ((uint32_t) 0x00000001)  /* INIT_SD 							*/
#define TRANS_TYPE_RW_READ_SD_BLOCK        ((uint32_t) 0x00000002)  /* RW_READ_SD_BLOCK 		*/
#define TRANS_TYPE_RW_WRITE_SD_BLOCK       ((uint32_t) 0x00000003)  /* RW_WRITE_SD_BLOCK 		*/

#define TRANS_ERROR_MASK    				       ((uint32_t) 0x000000FF)  /* TRANS_ERROR_MASK 		*/
#define TRANS_ERROR_INIT_NO           		 ((uint32_t) 0x00000000)  /* INIT_NO_ERROR 				*/
#define TRANS_ERROR_INIT_CMD0            	 ((uint32_t) 0x00000001)  /* INIT_CMD0_ERROR 			*/
#define TRANS_ERROR_INIT_CMD1              ((uint32_t) 0x00000002)  /* INIT_CMD1_ERROR 			*/
#define TRANS_ERROR_READ_NO       				 ((uint32_t) 0x00000000)  /* READ_NO_ERROR 				*/
#define TRANS_ERROR_READ_CMD       				 ((uint32_t) 0x00000004)  /* READ_CMD_ERROR 			*/
#define TRANS_ERROR_READ_TOKEN             ((uint32_t) 0x00000008)  /* READ_TOKEN_ERROR 		*/
#define TRANS_ERROR_WRITE_NO       				 ((uint32_t) 0x00000000)  /* WRITE_NO_ERROR 			*/
#define TRANS_ERROR_WRITE_CMD       			 ((uint32_t) 0x00000010)  /* WRITE_CMD_ERROR 			*/
#define TRANS_ERROR_WRITE_DATA     				 ((uint32_t) 0x00000020)  /* WRITE_DATA_ERROR 		*/
#define TRANS_ERROR_WRITE_BUSY		 			   ((uint32_t) 0x00000030)	/* WRITE_BUSY_ERROR			*/

#define TRANS_ERROR_INIT_MASK              ((uint32_t) 0x00000003)  /* INIT_NO_ERROR 				*/
#define TRANS_ERROR_READ_MASK       		   ((uint32_t) 0x0000000C)  /* READ_NO_ERROR 				*/
#define TRANS_ERROR_WRITE_MASK       	     ((uint32_t) 0x00000030)  /* WRITE_NO_ERROR 			*/

#define TRANS_BUSY								 				 ((uint32_t) 0x00000001)  /* TRANS_BUSY 					*/
#define TRANS_START								 				 ((uint32_t) 0x00000001)  /* TRANS_BUSY 					*/

#define SDCARD_BLOCK_SIZE 512																				/* SD Block Size				*/

/* Functions --------------------------------------------------------------------*/
/**
  * @brief SD hardware initialization.
  */
extern int SD_Init(SDCard_TypeDef* sd);

/**
  * @brief SD block write data.
  */
extern int SD_BlockWrite(SDCard_TypeDef* sd , uint8_t* writedata , uint32_t address);

/**
  * @brief SD block read data
  */
extern int SD_BlockRead(SDCard_TypeDef* sd , uint8_t* readdata , uint32_t address);

#ifdef __cplusplus
}
#endif

#endif  /* GOWIN_M1_SDCARD_H */
