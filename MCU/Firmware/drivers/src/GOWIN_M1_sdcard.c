/*
 ****************************************************************************************************
 * @file      GOWIN_M1_sdcard.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the SD-Card firmware library.
 ****************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_sdcard.h"


/* Definitions ---------------------------------------------------------------*/
/**
  * @param SDCard_TypeDef pointer.
  * @return low level.
  * @brief SDCard hardware initialization.
  */
int SD_Init(SDCard_TypeDef* sd)
{
	uint32_t tmp_data;
	
	sd->TRANS_TYPE &= 0;
	sd->TRANS_CTRL = 0;
	sd->TRANS_TYPE |= TRANS_TYPE_INIT_SD;
	sd->TRANS_CTRL = TRANS_START;
	
	tmp_data = sd->TRANS_STS;
	
	while(TRANS_BUSY == tmp_data)
	{
		tmp_data = sd->TRANS_STS;
		tmp_data &= TRANS_BUSY;
	}
	
	while((sd->TRANS_ERROR&TRANS_ERROR_INIT_MASK)!=TRANS_ERROR_INIT_NO);
	
	return 0;
}

/**
  * @param SDCard_TypeDef pointer.
	* @param Write data.
  * @param Address to SD_ADDR_7_0...SD_ADDR_31_24.
  * @return 0/1.
  * @brief SDCard block write data. 
  */
int SD_BlockWrite(SDCard_TypeDef* sd , uint8_t* writedata , uint32_t address)
{
	uint32_t i;
	
	sd->TX_FIFO_CONTROL = 0x01;
	
	for(i = 0;i < SDCARD_BLOCK_SIZE;i++)
	{
		sd->TX_FIFO_DATA = writedata[i];
	}
	
	sd->SD_ADDR_7_0 	= (uint8_t) address;
	sd->SD_ADDR_15_8	= (uint8_t) (address>>8);
	sd->SD_ADDR_23_16	= (uint8_t) (address>>16);
	sd->SD_ADDR_31_24	= (uint8_t) (address>>24);
	
	sd->TRANS_TYPE  	&=0;
	sd->TRANS_TYPE		|= TRANS_TYPE_RW_WRITE_SD_BLOCK;
	sd->TRANS_CTRL = TRANS_START;
	
	while(TRANS_BUSY == (TRANS_BUSY&sd->TRANS_STS));
	
	if((sd->TRANS_ERROR&TRANS_ERROR_WRITE_MASK)==TRANS_ERROR_WRITE_NO)
	{
		sd->TX_FIFO_CONTROL = 0x01;
		
		return 0;
	}
	else
	{
		sd->TX_FIFO_CONTROL = 0x01;
		
		return 1;
	}
}

/**
  * @param SDCard_TypeDef pointer.
	* @param Read data.
  * @param Address to SD_ADDR_7_0...SD_ADDR_31_24.
  * @return 0/1.
  * @brief SDCard block read data. 
  */
int SD_BlockRead(SDCard_TypeDef* sd , uint8_t* readdata , uint32_t address)
{
	uint32_t i;
	
	sd->RX_FIFO_CONTROL = 0x01;
	
	sd->SD_ADDR_7_0 	= (uint8_t) address;
	sd->SD_ADDR_15_8	= (uint8_t) (address>>8);
	sd->SD_ADDR_23_16	= (uint8_t) (address>>16);
	sd->SD_ADDR_31_24	= (uint8_t) (address>>24);
	
	sd->TRANS_TYPE  	&=0;
	sd->TRANS_TYPE		|= TRANS_TYPE_RW_READ_SD_BLOCK;
	sd->TRANS_CTRL = TRANS_START;
	
	while(TRANS_BUSY == (TRANS_BUSY&sd->TRANS_STS));

	if((sd->TRANS_ERROR&TRANS_ERROR_READ_MASK)==TRANS_ERROR_READ_NO)
	{
		*readdata =sd->RX_FIFO_DATA;
		
		for(i=0;i<SDCARD_BLOCK_SIZE;i++)
		{
			*(readdata+i) = sd->RX_FIFO_DATA;
		}
		
		sd->RX_FIFO_CONTROL = 0x01;
		
		return 0;
	}
	else
	{
		return 1;
	}
}
