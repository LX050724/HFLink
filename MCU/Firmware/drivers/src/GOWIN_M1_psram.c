/*
 ********************************************************************************************
 * @file      GOWIN_M1_psram.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the PSRAM firmware library.
 ********************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_psram.h"


/* Definitions ---------------------------------------------------------------*/
/**
  * @brief Check PSRAM hardware initialization status.
  */
uint32_t PSRAM_Check_Init_Status(void)
{
	//1 is done 0 is unready
	return PSRAM->INIT_DONE;
}

/**
  * @brief Set PSRAM read/write mode.
  */
uint32_t PSRAM_Mode_Set(uint32_t mode)
{
	if(mode == 0 || mode == 1)
	{
		PSRAM->CMD = mode;
		return 0;
	}
	else
	{
		return 1; //error mode
	}
}

/**
  * @brief Set PSRAM address, save data into this address.
  */
uint32_t PSRAM_Address_Set(uint32_t address)
{
	if(address < 0x20000)
	{
		PSRAM->ADDRESS = address;
		return 0;
	}
	else
	{
		//out of the range of the address
		return 1;
	}
}

/**
  * @brief Write data into PSRAM buffer, 4 word is a cycle.
  */
uint32_t PSRAM_Write_Data_Buff(uint32_t *Buff,uint32_t address)
{
	PSRAM_Cmd_Unable();
	
	PSRAM->WR_DATA0 = Buff[0];
	PSRAM->WR_DATA1 = Buff[1];
	PSRAM->WR_DATA2 = Buff[2];
	PSRAM->WR_DATA3 = Buff[3];
	
	PSRAM_Address_Set(address);
	
	PSRAM_Mode_Set(WRITE_MODE);
	
	PSRAM_Cmd_Enable();
	
	return 1;
}

/**
  * @brief Read data from PSRAM buffer.
  */
uint32_t PSRAM_Read_Data_Buff(uint32_t *Buff,uint32_t address)
{
	PSRAM_Cmd_Unable();
	
	PSRAM_Address_Set(address);
	
	PSRAM_Mode_Set(READ_MODE);
	
	PSRAM_Cmd_Enable();
	
	//wait response ready
	while((PSRAM_Read_Done_Flag() & 0x01) == 0);
	
	Buff[0] = PSRAM->RD_DATA0 ;
	Buff[1] = PSRAM->RD_DATA1 ;  
	Buff[2] = PSRAM->RD_DATA2 ;
	Buff[3] = PSRAM->RD_DATA3 ;
	
	//clear response
	PSRAM_Clear_Read_Done_Flag();

	return 1;
}

/**
  * @brief Enable PSRAM command.
  */
void PSRAM_Cmd_Enable(void)
{
	PSRAM->CMD_EN = CMD_ENABLE;
}

/**
  * @brief Disable PSRAM command.
  */
void PSRAM_Cmd_Unable(void)
{
	PSRAM->CMD_EN = CMD_UNABLE;
}

/**
  * @brief The flag of read PSRAM done.
  */
uint32_t PSRAM_Read_Done_Flag(void)
{
	return PSRAM->READ_DONE; 
}

/**
  * @brief Clear the flag of read PSRAM done.
  */
void PSRAM_Clear_Read_Done_Flag(void)
{
	PSRAM->READ_DONE = 0x00;
}

/**
  * @brief Write a package data into PSRAM.
  *        The valid address bit is [20-0].
  */
uint32_t PSRAM_Write_Data_Package(uint32_t *Buff,uint32_t address,uint32_t num)
{
	uint32_t i;
	
	uint32_t temp_arry[4] = {0x00000000,0x00000000,0x00000000,0x00000000};
	uint32_t remainder = num & 0x03;//The psram data is align 4 word

	num = num & (~0x03);//bit invert
	
	//The address must align 8
	address = (address >> 3)<<3;

	//first send the 4 align word data
	//and the address is align 8
	for(i =0 ; i< num ; i= i+4)
	{
		PSRAM_Write_Data_Buff(&Buff[i],address + i*2);
	}
	
	//second deal with the remainder word
	if(remainder)
	{
		for(uint32_t j =0 ; j<remainder ; j++)
		{
			temp_arry[j] = *(&Buff[i] + j);
		}
	}
	
	PSRAM_Write_Data_Buff(temp_arry,address + i*2);
	
	return 1;
}

/**
  * @brief Read a package data from PSRAM.
  *        In order to prevent the the arry overflow.
  */
uint32_t PSRAM_Read_Data_Package(uint32_t *Buff,uint32_t address,uint32_t num)
{
	uint32_t i;
	
	uint32_t temp_arry[4] = {0x00000000,0x00000000,0x00000000,0x00000000};
	uint32_t remainder = num & 0x03;//The psram data is align 4 word
	
	num = num & (~0x03);
	
	//The address must align 8
	address = (address >> 3)<<3;
	for(i =0 ; i< num ; i= i+4)
	{
		PSRAM_Read_Data_Buff(&Buff[i],address + i*2);
	}
	
	if(remainder)
	{
		PSRAM_Read_Data_Buff(temp_arry,address + i*2);
		
		for(uint32_t j =0 ; j<remainder ; j++)
		{
			//set remainder data
			*(&Buff[i] + j) = temp_arry[j];
		}
	}
	
	return 1;
}
