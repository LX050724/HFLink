/*
 **************************************************************************************************
 * @file      GOWIN_M1_qspi_flash.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the QSPI-Flash firmware library.
 **************************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_QSPI_FLASH_H
#define GOWIN_M1_QSPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "GOWIN_M1.h"

/** @addtogroup GOWIN_M1_StdPeriph_Driver
  * @{
  */
/** @addtogroup QSPI_FLASH
  * @{
  */

/* Macros ------------------------------------------------------------------*/
#define SPI_CMDEN                     (1UL << 30)
#define SPI_ADDREN                    (1UL << 29)

//The winbone qspi-flash command
#define WRITE_CMD													0x01	//write
#define	PROGRAM_CMD												0x02	//page program
#define READ_CMD													0x03	//read
#define WRITE_DISABLE_CMD                 0x04	//disable write
#define READ_STATUS_CMD										0x05	//read status
#define WRITE_ENABLE_CMD                  0x06	//enable write
#define	ERASE_4K_CMD											0x20	//erase 4k
#define ERASE_64K_CMD											0xD8	//erase 64k
#define ERASE_CHIP_CMD                    0x60  //erase full chip

#define DUAL_FAST_READ_CMD                0x3B  //fast read dual output

#define QUAD_PROGRAM_CMD                  0x32  //quad page program
#define QUAD_FAST_READ_CMD                0x6B  //fast read quad output
#define QUAD_IO_FAST_READ_CMD             0xEB  //I/O fast read quad output

#define ENTER_QSPI_MODE_CMD      					0x38	//enable qspi mode
#define EXIT_QSPI_MODE_CMD      					0xFF	//disable qspi mode

#define READ_STATUS_REG1_CMD							0x05 
#define READ_STATUS_REG2_CMD							0x35 
#define READ_STATUS_REG3_CMD							0x15 
#define WRITE_STATUS_REG1_CMD    					0x01 
#define WRITE_STATUS_REG2_CMD    					0x31 
#define WRITE_STATUS_REG3_CMD    					0x11 


/* Functions ------------------------------------------------------------------*/
/**
  * @brief Initializes QSPI-Flash
  */
extern void qspi_flash_init(void);

/**
  * @brief Switch QSPI-Flash mode between download and read/write/erase memory
  */
extern void change_mode_qspi_flash(void);

/**
  * @brief I/O fast read data from QSPI-Flash
  */
extern void qspi_flash_io_fast_read(uint16_t rd_len, uint32_t cmd, uint32_t address,uint8_t *read_buffer);

/**
  * @brief Fast read data from QSPI-Flash
  */
extern void qspi_flash_fast_read(uint16_t rd_len, uint32_t cmd, uint32_t address,uint8_t *read_buffer);
/**
  * @brief Write data into QSPI-Flash
  */
extern void qspi_flash_write(uint16_t wr_len, uint32_t cmd, uint32_t address,uint8_t *write_buffer);

/**
  * @brief Write data into a page of QSPI-Flash
  */
extern void qspi_flash_page_program(uint16_t wr_len,uint32_t address,uint8_t *write_buffer);

/**
  * @brief Erase a 4K sector of QSPI-Flash
  */
extern void qspi_flash_4ksector_erase(uint32_t address);

/**
  * @brief Erase a 64K sector of QSPI-Flash
  */
extern void qspi_flash_64ksector_erase(uint32_t address);

/**
  * @brief Erase a full chip of QSPI-Flash
  */
extern void qspi_flash_chip_erase(void);

/**
  * @brief Write status register of QSPI-Flash
  */
extern void qspi_flash_write_sr(uint8_t reg, uint8_t byte);

/**
  * @brief Read status register of QSPI-Flash
  */
extern uint8_t qspi_flash_read_sr(uint8_t reg);

/**
  * @brief Enable QSPI-Flash
  */
extern void qspi_flash_Enable(void);

void qspi_flash_fast_read_quad(uint32_t address, uint8_t *read_buffer, uint16_t rd_len);

#ifdef __cplusplus
}
#endif

#endif	/* GOWIN_M1_QSPI_FLASH_H */

/**
  * @}
  */ 

/**
  * @}
  */
