/*
 **************************************************************************************************
 * @file      GOWIN_M1_spi_flash.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the SPI-Flash firmware library.
 **************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_spi_flash.h"


/* Declarations ------------------------------------------------------------------*/
static uint8_t spi_get_fifo_depth(uint8_t fifo_depth_config);
static void spi_flash_read_status(void);
static void spi_flash_write_cmd(uint32_t cmd);


/* Functions ------------------------------------------------------------------*/
/**
  * @brief Initializes SPI-Flash
  */
void spi_flash_init(void)
{
	uint32_t buff;

	SPI_FLASH->CTRL |= 0x01;         //reset spi
	while((SPI_FLASH->CTRL & 0x01)); //wait until spi reset complete

	buff = SPI_FLASH->CONFIG;
	uint8_t tx_fifo_depth_config = (buff & 0x3f) >> 4;
	uint8_t tx_fifo_depth = spi_get_fifo_depth(tx_fifo_depth_config);
	uint8_t rx_fifo_depth_config = (buff & 0x03);
	uint8_t rx_fifo_depth = spi_get_fifo_depth(rx_fifo_depth_config);

	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI2 active finish
	
	buff = (2 << 16) +(7 << 8) + (0 << 7);
	//AddrLen = 3; DataLen = 8 DataMerge = 0
	//other fields reset to 0 while spi_default_modes connected to 0
	SPI_FLASH->TRANSFMT = buff;

	SPI_FLASH->CTRL |= (0x1 << 2);    //reset tx fifo
	while((SPI_FLASH->CTRL & 0x2)) ;  //wait tx fifo reset complete
	SPI_FLASH->CTRL  |= ((tx_fifo_depth/2) << 16);  //set tx threshold as half of txfifo depth

	SPI_FLASH->CTRL |= (0x1 << 1);    //reset rx fifo
	while((SPI_FLASH->CTRL & 0x1)) ;  //wait rx fifo reset complete
	SPI_FLASH->CTRL  |= ((rx_fifo_depth/2) << 8);  //set rx threshold as half of rxfifo depth

	//SPI_FLASH_Reg->INTREN |= (0x1 << 4);  //Do not support Interupt signal
	SPI_FLASH->INTREN &= 0x00000000;        //close all the interrupt 
	SPI_FLASH->TIMING &= (~(0xff));
	SPI_FLASH->TIMING |= (0x4);        //Set the SPI Interface Timing Register (0x40)
	                                   //to set SCLK_DIV to 4, which divide the spi_clk into
	                                   //((4+1)*2) = 10 as SCLK
}

/**
  * @brief Read data from SPI-Flash
  */
void spi_flash_read(uint16_t rd_len, uint32_t cmd, uint32_t address,uint8_t *read_buffer)
{
	if(NULL == read_buffer)
	{
		return;
	}
	
	SPI_FLASH->TRANSCTRL = (1 << 30) +        //enable cmd
						             (1 << 29) +        //Open the address 
			                   (2 << 24) +        //trans mode = 2 (read only)
						             (0 << 22) +        //dual mode 0 regular mode
						            ((rd_len-1) << 0);  //set read trans byte count
	SPI_FLASH->ADDR = address;
	SPI_FLASH->CMD  = cmd;
	
	for(uint16_t i = 0; i < rd_len; i++)
	{
		while(SPI_FLASH->STATUS & (0x1 <<14));	//wait if RX FIFO is empty
		
		*read_buffer = SPI_FLASH->DATA;
		read_buffer++;
	}
	
	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI_Ctroller active finish
}

/**
  * @brief Write data into SPI-Flash
  */
void spi_flash_write(uint16_t wr_len, uint32_t cmd, uint32_t address,uint8_t *write_buffer)
{
	if(NULL == write_buffer)
	{
		return;
	}
	
	spi_flash_write_cmd(WRITE_ENABLE_CMD);
	spi_flash_read_status();
	
	SPI_FLASH->TRANSCTRL = (1 << 30) +        	//enable cmd 
												 (1 << 29) + 					//enable address 
												 (1 << 24) +        	//trans mode = 1 (write only)
												 (0 << 22) +         	//dual mode
												 ((wr_len-1) << 12); 	//set read trans byte count

	SPI_FLASH->ADDR = address;
	SPI_FLASH->CMD  = cmd;
	
	for(uint16_t i = 0; i < wr_len; i++)
	{
		while(SPI_FLASH->STATUS & (0x1 <<23));		//check the status of txfifo
		
		SPI_FLASH->DATA = (uint32_t)(*write_buffer);
		write_buffer++;
	}

	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI_ctroller active finish
	spi_flash_write_cmd(WRITE_DISABLE_CMD);
	spi_flash_read_status();
}

/**
  * @brief Switch SPI-Flash mode between download and read/write/erase memory
  */
void change_mode_spi_flash()
{
	uint32_t i;
	//read spi memory access register 0x50
	i = SPI_FLASH->MEMCTRL;
	
	//write back to spi flash reg
	SPI_FLASH->MEMCTRL = i;
	
	//wait the memCtrlChg become 0
	while(SPI_FLASH->MEMCTRL & (0x01 <<8 ));
}

/**
  * @brief Write data into a page of SPI-Flash
  */
void spi_flash_page_program(uint16_t wr_len,uint32_t address,uint8_t *write_buffer)
{
	if(NULL == write_buffer)
	{
		return;
	}
	
	spi_flash_write_cmd(WRITE_ENABLE_CMD);
	spi_flash_read_status();
	
	SPI_FLASH->TRANSCTRL = (1 << 30) +           //enable cmd 
						             (1 << 29) +           //enable address 
			                   (1 << 24) +           //trans mode = 1 (write only)
						             (0 << 22) +           //dual mode
						             ((wr_len-1) << 12);   //set read trans byte count

	SPI_FLASH->ADDR = address;
	SPI_FLASH->CMD  = PROGRAM_CMD;//page program cmd
	
	for(uint16_t i = 0; i < wr_len; i++)
	{
		while(SPI_FLASH->STATUS & (0x1 <<23));		 //check the status of txfifo
		
		SPI_FLASH->DATA = (uint32_t)(*write_buffer);
		write_buffer++;
	}

	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI_ctroller active finish
	
	spi_flash_write_cmd(WRITE_DISABLE_CMD);
	spi_flash_read_status();    //wait spi flash free
}

/**
  * @brief Erase a 4K sector of SPI-Flash
  */
void spi_flash_4ksector_erase(uint32_t address)
{
	spi_flash_write_cmd(WRITE_ENABLE_CMD);
	spi_flash_read_status();
	
	//litter endian and the range of address
	uint8_t temp[3];
	SPI_FLASH->TRANSCTRL = (1 << 30) +        //enable cmd 
			                   (1 << 24) +        //trans mode = 1 (write only)
						             (0 << 22) +        //dual mode
						             (2 << 12);         //set address - 3bytes
	
	temp[0] = (address >> 16 &0xFF);//The address High byte
	temp[1] = (address >> 8  &0xFF);//The address middle byte
	temp[2] = (address       &0xFF);//The address low   byte
	
	SPI_FLASH->CMD  = ERASE_4K_CMD; //4K sector erase cmd
	
	for(uint8_t i = 0; i < 3; i++)
	{
		while(SPI_FLASH->STATUS & (0x1 <<23));	//check the status of txfifo
		
		SPI_FLASH->DATA = temp[i];
	}
	
	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI_ctroller active finish
	
	spi_flash_write_cmd(WRITE_DISABLE_CMD);
	spi_flash_read_status();    //wait spi flash free
}

/**
  * @brief Erase a 64K sector of SPI-Flash
  */
void spi_flash_64ksector_erase(uint32_t address)
{
	spi_flash_write_cmd(WRITE_ENABLE_CMD);
	spi_flash_read_status();
	
	//litter endian and the range of address
	uint8_t temp[3];
	SPI_FLASH->TRANSCTRL = (1 << 30) +        //enable cmd 
			                   (1 << 24) +        //trans mode = 1 (write only)
						             (0 << 22) +        //dual mode
						             (2 << 12);         //set address - 3bytes
	
	temp[0] = (address >> 16 &0xFF);//The address High byte
	temp[1] = (address >> 8  &0xFF);//The address middle byte
	temp[2] = (address       &0xFF);//The address low   byte
	
	SPI_FLASH->CMD  = ERASE_64K_CMD; //64K sector erase cmd
	
	for(uint8_t i = 0; i < 3; i++)
	{
		while(SPI_FLASH->STATUS & (0x1 <<23));	//check the status of txfifo
		
		SPI_FLASH->DATA = temp[i];
	}
	
	while((SPI_FLASH->STATUS & 0x01)); //wait until SPI_ctroller active finish
	
	spi_flash_write_cmd(WRITE_DISABLE_CMD);
	spi_flash_read_status();    //wait spi flash free
}

/**
  * @brief Get SPI FIFO depth
  */
static uint8_t spi_get_fifo_depth(uint8_t fifo_depth_config)
{
	uint8_t fifo_depth = 1;
	
	for(int i = 0; i < fifo_depth_config+1; i++)
	{
		fifo_depth *= 2;
	}
	
	return fifo_depth;
}

/**
  * @brief Write command into SPI-Flash
  */
static void spi_flash_write_cmd(uint32_t cmd)
{
	SPI_FLASH->TRANSCTRL = (0 << 30) +        //Disenable cmd 
			                   (1 << 24) +        //trans mode = 1 (write only)
						             (0 << 22) +        //regular mode
						             (0 << 12);         //set trans byte count is 1
	SPI_FLASH->CMD  = WRITE_CMD;//Start to transfer
	
	if(cmd == ERASE_CHIP_CMD)
	{
		spi_flash_write_cmd(WRITE_ENABLE_CMD);
		spi_flash_read_status();
	}
	
	//Check the status of tx_FIFO
	while(SPI_FLASH->STATUS & (0x1 <<23));
	
	SPI_FLASH->DATA = cmd;
	
	//Check the status of Transfer
	while((SPI_FLASH->STATUS & 0x01));
}

/**
  * @brief Read SPI-Flash status
  */
static void spi_flash_read_status()
{
	uint8_t temp_first;
	SPI_FLASH->TRANSCTRL = (1 << 30) +        //enable cmd
			                   (2 << 24) +        //trans mode = 2 (read only)
						             (0 << 22) +        //dual mode 0 regular mode
						             (0 << 0);          //set read 1 byte
						   
   do
   {
	   	SPI_FLASH->CMD  = READ_STATUS_CMD;//read status
		  temp_first  = SPI_FLASH->DATA;
   }while(temp_first & 0x01 );//Select the low bit and wait become 0
   
   while((SPI_FLASH->STATUS & 0x03)); //wait until SPI_Ctroller active finish
}
