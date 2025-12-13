/*
 **************************************************************************************************
 * @file      GOWIN_M1_qspi_flash.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the QSPI-Flash firmware library.
 **************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_qspi_flash.h"

/* Declarations ------------------------------------------------------------------*/
static uint8_t qspi_get_fifo_depth(uint8_t fifo_depth_config);
// static uint8_t qspi_flash_read_status(void);
static void qspi_flash_write_cmd(uint32_t cmd);

/* Functions ------------------------------------------------------------------*/
/**
 * @brief Initializes QSPI-Flash
 */
void qspi_flash_init(void)
{
    uint32_t buff;

    SPI_FLASH->CTRL |= 0x01; // reset qspi
    while ((SPI_FLASH->CTRL & 0x01))
        ; // wait until qspi reset complete

    buff = SPI_FLASH->CONFIG;
    uint8_t tx_fifo_depth_config = (buff & 0x3f) >> 4;
    uint8_t tx_fifo_depth = qspi_get_fifo_depth(tx_fifo_depth_config);
    uint8_t rx_fifo_depth_config = (buff & 0x03);
    uint8_t rx_fifo_depth = qspi_get_fifo_depth(rx_fifo_depth_config);

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI active finish

    buff = QSPI_ADDRESS_24_BITS | QSPI_DATA_8_BITS | SPIFLASH_TRANSFMT_DATAMERGE;
    // other fields reset to 0 while qspi_default_modes connected to 0
    SPI_FLASH->TRANSFMT = buff;

    // SPI_FLASH->CMD = 0xEB;
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_ADDRESS_SAME_DATA | QSPI_DATA_4_LINES | QSPI_TRMODE_DUMMY_READ |
                           (2 << SPIFLASH_TRANSCTRL_DUMMYCNT_Pos); // set read trans byte count

    SPI_FLASH->CTRL |= (0x1 << 2); // reset tx fifo
    while ((SPI_FLASH->CTRL & 0x2))
        ;                                           // wait tx fifo reset complete
    SPI_FLASH->CTRL |= ((tx_fifo_depth / 2) << 16); // set tx threshold as half of txfifo depth

    SPI_FLASH->CTRL |= (0x1 << 1); // reset rx fifo
    while ((SPI_FLASH->CTRL & 0x1))
        ;                                          // wait rx fifo reset complete
    SPI_FLASH->CTRL |= ((rx_fifo_depth / 2) << 8); // set rx threshold as half of rxfifo depth

    SPI_FLASH->INTREN &= 0x00000000; // close all the interrupt
    SPI_FLASH->TIMING &= (~(0xff));
    SPI_FLASH->TIMING |= (7) | (5 << 12) | (5 << 8);
}

/**
 * @brief I/O fast read data from QSPI-Flash
 */
void qspi_flash_io_fast_read(uint16_t rd_len, uint32_t cmd, uint32_t address, uint8_t *read_buffer)
{
    if (NULL == read_buffer)
    {
        return;
    }

    SPI_FLASH->TRANSCTRL = (1 << 30) +          // enable cmd
                           (1 << 29) +          // open the address
                           (1 << 28) +          // address format
                           (5 << 24) +          // trans mode = 5 (Write, Dummy, Read)
                           (2 << 22) +          // dual mode
                           (1 << 9) +           // dummy data count
                           ((rd_len - 1) << 0); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = cmd;

    SPI_FLASH->DATA = 0x10;

    for (uint16_t i = 0; i < rd_len; i++)
    {
        // check the status of txfifo
        do
        {
            *read_buffer = SPI_FLASH->DATA;
            read_buffer++;
        } while (SPI_FLASH->STATUS & (0x1 << 15));
    }

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_Ctroller active finish
}

/**
 * @brief Fast read data from QSPI-Flash
 */
void qspi_flash_fast_read(uint16_t rd_len, uint32_t cmd, uint32_t address, uint8_t *read_buffer)
{
    if (NULL == read_buffer)
    {
        return;
    }

    SPI_FLASH->TRANSCTRL = (1 << 30) |          // enable cmd
                           (1 << 29) |          // open the address
                           (1 << 28) |          // open the address
                           (9 << 24) |          // trans mode = 9 (Dummy, Read)
                           (2 << 22) |          // qual mode
                           (3 << 9) |           // dummy data count
                           ((rd_len - 1) << 0); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = cmd;

    for (uint16_t i = 0; i < rd_len; i++)
    {
        // check the status of txfifo
        do
        {
            *read_buffer = SPI_FLASH->DATA;
            read_buffer++;
        } while (SPI_FLASH->STATUS & (0x1 << 15));
    }

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_Ctroller active finish
}

void qspi_flash_fast_read_quad(uint32_t address, uint8_t *read_buffer, uint16_t rd_len)
{
    if (NULL == read_buffer)
    {
        return;
    }

    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_ADDRESS_SAME_DATA | QSPI_DATA_4_LINES | QSPI_TRMODE_DUMMY_READ |
                           (2 << SPIFLASH_TRANSCTRL_DUMMYCNT_Pos) |
                           ((rd_len - 1) << SPIFLASH_TRANSCTRL_RDDT_CNT_Pos); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = 0xEB;

    for (uint16_t i = 0; i < rd_len; i++)
    {
        // check the status of txfifo
        // do
        // {
        while (SPI_FLASH->STATUS & SPIFLASH_SR_RXFE)
            ;
        *read_buffer = *(uint8_t *)&SPI_FLASH->DATA;
        read_buffer++;
    }

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_Ctroller active finish
}

/**
 * @brief Write data into QSPI-Flash
 */
void qspi_flash_write(uint16_t wr_len, uint32_t cmd, uint32_t address, uint8_t *write_buffer)
{
    if (NULL == write_buffer)
    {
        return;
    }

    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    SPI_FLASH->TRANSCTRL = (1 << 30) +           // enable cmd
                           (1 << 29) +           // enable address
                           (1 << 24) +           // trans mode = 1 (write only)
                           (2 << 22) +           // dual mode
                           ((wr_len - 1) << 12); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = cmd;

    for (uint16_t i = 0; i < wr_len; i++)
    {
        // check the status of txfifo
        while (SPI_FLASH->STATUS & (0x1 << 23))
            ;

        SPI_FLASH->DATA = *write_buffer;
        write_buffer++;
    }

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish
    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status();
}

/**
 * @brief Switch QSPI-Flash mode between download and read/write/erase memory
 */
void change_mode_qspi_flash(void)
{
    uint32_t i;

    // read qspi memory access register 0x50
    i = SPI_FLASH->MEMCTRL;

    // write back to qspi flash reg
    SPI_FLASH->MEMCTRL = i;

    // wait the memCtrlChg become 0
    while (SPI_FLASH->MEMCTRL & (0x01 << 8))
        ;
}

/**
 * @brief Write data into a page of QSPI-Flash
 */
void qspi_flash_page_program(uint16_t wr_len, uint32_t address, uint8_t *write_buffer)
{
    if (NULL == write_buffer)
    {
        return;
    }

    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    SPI_FLASH->TRANSCTRL = (1 << 30) +           // enable cmd
                           (1 << 29) +           // enable address
                           (1 << 24) +           // trans mode = 1 (write only)
                           (2 << 22) +           // dual mode
                           ((wr_len - 1) << 12); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = PROGRAM_CMD; // page program cmd

    for (uint16_t i = 0; i < wr_len; i++)
    {
        // check the status of txfifo
        while (SPI_FLASH->STATUS & (0x1 << 23))
            ;

        SPI_FLASH->DATA = (uint32_t)(*write_buffer);
        write_buffer++;
    }

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish

    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status(); // wait qspi flash free
}

/**
 * @brief Erase a 4K sector of QSPI-Flash
 */
void qspi_flash_4ksector_erase(uint32_t address)
{
    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    // litter endian and the range of address
    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (1 << 29) + // enable address
                           (7 << 24) + // trans mode = 1 (write only)
                           (2 << 22) + // dual mode
                           (0 << 12);  // set address - 3bytes

    SPI_FLASH->ADDR = address;

    SPI_FLASH->CMD = ERASE_4K_CMD; // 4K sector erase cmd

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish

    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status(); // wait qspi flash free
}

/**
 * @brief Erase a 64K sector of QSPI-Flash
 */
void qspi_flash_64ksector_erase(uint32_t address)
{
    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    // litter endian and the range of address
    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (1 << 29) + // enable address
                           (7 << 24) + // trans mode = 1 (write only)
                           (2 << 22) + // dual mode
                           (0 << 12);  // set address - 3bytes

    SPI_FLASH->ADDR = address;

    SPI_FLASH->CMD = ERASE_64K_CMD; // 64K sector erase cmd

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish

    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status(); // wait qspi flash free
}

void qspi_flash_32ksector_erase(uint32_t address)
{
    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    // litter endian and the range of address
    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (1 << 29) + // enable address
                           (7 << 24) + // trans mode = 1 (write only)
                           (2 << 22) + // dual mode
                           (0 << 12);  // set address - 3bytes

    SPI_FLASH->ADDR = address;

    SPI_FLASH->CMD = ERASE_32K_CMD; // 64K sector erase cmd

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish

    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status(); // wait qspi flash free
}

/**
 * @brief Erase full chip of QSPI-Flash
 */
void qspi_flash_chip_erase(void)
{
    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
    qspi_flash_read_status();

    // litter endian and the range of address
    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (0 << 29) + // enable address
                           (7 << 24) + // trans mode = 1 (write only)
                           (2 << 22) + // dual mode
                           (0 << 12);  // set address - 3bytes

    SPI_FLASH->CMD = ERASE_CHIP_CMD; // full chip erase cmd

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_ctroller active finish

    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    qspi_flash_read_status(); // wait qspi flash free
}

/**
 * @brief Get QSPI FIFO depth
 */
static uint8_t qspi_get_fifo_depth(uint8_t fifo_depth_config)
{
    uint8_t fifo_depth = 1;

    for (int i = 0; i < fifo_depth_config + 1; i++)
    {
        fifo_depth *= 2;
    }

    return fifo_depth;
}

/**
 * @brief Write command into QSPI-Flash
 */
static void qspi_flash_write_cmd(uint32_t cmd)
{
    SPI_FLASH->TRANSCTRL = (1 << 30) + // disable cmd
                           (0 << 29) + // enable addr
                           (7 << 24) + // trans mode = 1 (write only)
                           (0 << 22) + // regular mode
                           (0 << 12);  // set trans byte count is 1

    SPI_FLASH->CMD = cmd; // Start to transfer

    if (cmd == ERASE_CHIP_CMD)
    {
        qspi_flash_write_cmd(WRITE_ENABLE_CMD);
        qspi_flash_read_status();
    }

    // Check the status of tx_FIFO
    while (SPI_FLASH->STATUS & (0x1 << 23))
        ;

    // Check the status of Transfer
    while ((SPI_FLASH->STATUS & 0x01))
        ;
}

/**
 * @brief Read QSPI-Flash status
 */
uint8_t qspi_flash_read_status(void)
{
    uint8_t temp_first;

    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (0 << 29) + // enable addr
                           (2 << 24) + // trans mode = 2 (read only)
                           (0 << 22) + // dual mode 0 regular mode
                           (0 << 0);   // set read 1 byte

    do
    {
        SPI_FLASH->CMD = READ_STATUS_CMD; // read status
        temp_first = SPI_FLASH->DATA;
    } while (temp_first & 0x01); // Select the low bit and wait become 0

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_Ctroller active finish

    return temp_first;
}

/**
 * @brief Enable QSPI-Flash
 */
void qspi_flash_Enable(void)
{
    uint8_t stareg2;

    stareg2 = qspi_flash_read_sr(2);

    if ((stareg2 & 0X02) == 0) // QE bit is disable
    {
        qspi_flash_write_cmd(WRITE_ENABLE_CMD);
        stareg2 |= 1 << 1;               // enable QE bit
        qspi_flash_write_sr(2, stareg2); // write register 2
    }

    while (SPI_FLASH->STATUS & (0x1 << 23))
        ;

    while ((SPI_FLASH->STATUS & 0x01))
        ;
}

/**
 * @brief Write QSPI-Flash status register
 */
void qspi_flash_write_sr(uint8_t reg, uint8_t byte)
{
    uint8_t com;

    switch (reg)
    {
    case 1:
        com = WRITE_STATUS_REG1_CMD; // write status register 1
        break;
    case 2:
        com = WRITE_STATUS_REG2_CMD; // write status register 2
        break;
    case 3:
        com = WRITE_STATUS_REG3_CMD; // write status register 3
        break;
    default:
        com = WRITE_STATUS_REG1_CMD;
        break;
    }

    SPI_FLASH->TRANSCTRL = (1 << 30) + // disable cmd
                           (0 << 29) + // enable addr
                           (1 << 24) + // trans mode = 1 (write only)
                           (0 << 22) + // regular mode
                           (0 << 12);  // set trans byte count is 1

    SPI_FLASH->CMD = com; // Start to transfer

    SPI_FLASH->DATA = byte;

    while (SPI_FLASH->STATUS & (0x1 << 23))
        ;

    while ((SPI_FLASH->STATUS & 0x01))
        ;
}

/**
 * @brief Read QSPI-Flash status register
 */
uint8_t qspi_flash_read_sr(uint8_t reg)
{
    uint8_t byte;
    uint8_t com;

    switch (reg)
    {
    case 1:
        com = READ_STATUS_REG1_CMD; // read status register 1
        break;
    case 2:
        com = READ_STATUS_REG2_CMD; // read status register 2
        break;
    case 3:
        com = READ_STATUS_REG3_CMD; // read status register 3
        break;
    default:
        com = READ_STATUS_REG1_CMD;
        break;
    }

    SPI_FLASH->TRANSCTRL = (1 << 30) + // enable cmd
                           (0 << 29) + // enable addr
                           (2 << 24) + // trans mode = 2 (read only)
                           (0 << 22) + // dual mode 0 regular mode
                           (0 << 0);   // set read 1 byte

    SPI_FLASH->CMD = com; // read status

    byte = SPI_FLASH->DATA;

    while ((SPI_FLASH->STATUS & 0x01))
        ; // wait until QSPI_Ctroller active finish

    return byte;
}

uint8_t qspi_flash_chip_reset()
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | QSPI_TRMODE_NONE_DATA | QSPI_TRMODE_WRITE_ONLY;
    SPI_FLASH->CMD = 0x66;
    while (SPI_FLASH->STATUS & SPIFLASH_SR_PRGBUSY)
        ;
    SPI_FLASH->CMD = 0x99;
    while (SPI_FLASH->STATUS & SPIFLASH_SR_PRGBUSY)
        ;
    qspi_flash_read_status();

    return 0;
}

// void qspi_enable_mmap()
// {
//     SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
//                         QSPI_ADDRESS_SAME_DATA | QSPI_DATA_4_LINES | QSPI_TRMODE_DUMMY_READ |
//                         (2 << SPIFLASH_TRANSCTRL_DUMMYCNT_Pos);

//     SPI_FLASH->ADDR = 0;
//     SPI_FLASH->CMD = 0xEB;
//     SPI_FLASH->CONFIG |= 
// }