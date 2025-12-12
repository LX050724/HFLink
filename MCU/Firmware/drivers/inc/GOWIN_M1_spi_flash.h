/*
 **************************************************************************************************
 * @file      GOWIN_M1_spi_flash.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the SPI-Flash firmware library.
 **************************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_SPI_FLASH_H
#define GOWIN_M1_SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "GOWIN_M1.h"

/** @addtogroup GOWIN_M1_StdPeriph_Driver
  * @{
  */
/** @addtogroup SPI_FLASH
  * @{
  */

/* Macros ------------------------------------------------------------------*/
#define SPI_CMDEN                     (1UL << 30)
#define SPI_ADDREN                    (1UL << 29)

//The winbone spi-flash command
#define WRITE_CMD													0x01	//write
#define	PROGRAM_CMD												0x02	//page program
#define READ_CMD													0x03	//read
#define WRITE_DISABLE_CMD                 0x04	//disable write
#define READ_STATUS_CMD										0x05	//read status
#define WRITE_ENABLE_CMD                  0x06	//enable write
#define	ERASE_4K_CMD											0x20	//erase 4k
#define ERASE_64K_CMD											0xD8	//erase 64k
#define ERASE_CHIP_CMD                    0x60  //erase full chip


/* Functions ------------------------------------------------------------------*/
/**
  * @brief Initializes SPI-Flash
  */
extern void spi_flash_init(void);

/**
  * @brief Switch SPI-Flash mode between download and read/write/erase memory
  */
extern void change_mode_spi_flash(void);

/**
  * @brief Read data from SPI-Flash
  */
extern void spi_flash_read(uint16_t rd_len, uint32_t cmd, uint32_t address,uint8_t *read_buffer);

/**
  * @brief Write data into SPI-Flash
  */
extern void spi_flash_write(uint16_t wr_len, uint32_t cmd, uint32_t address,uint8_t *write_buffer);

/**
  * @brief Write data into a page of SPI-Flash
  */
extern void spi_flash_page_program(uint16_t wr_len,uint32_t address,uint8_t *write_buffer);

/**
  * @brief Erase a 4K sector of SPI-Flash
  */
extern void spi_flash_4ksector_erase(uint32_t address);

/**
  * @brief Erase a 64K sector of SPI-Flash
  */
extern void spi_flash_64ksector_erase(uint32_t address);


#ifdef __cplusplus
}
#endif

#endif	/* GOWIN_M1_SPI_FLASH_H */

/**
  * @}
  */ 

/**
  * @}
  */
