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
static void qspi_flash_write_cmd(uint32_t cmd);

static inline int qspi_flash_rx_fifo_empty(void)
{
    return (SPI_FLASH->STATUS & SPIFLASH_SR_RXFE) == SPIFLASH_SR_RXFE;
}

static inline int qspi_flash_tx_fifo_full(void)
{
    return (SPI_FLASH->STATUS & SPIFLASH_SR_TXFF) == SPIFLASH_SR_TXFF;
}

static inline int qspi_flash_is_active(void)
{
    return SPI_FLASH->STATUS & SPIFLASH_SR_PRGBUSY;
}

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

    while (qspi_flash_is_active())
        ; // wait until QSPI active finish

    buff = QSPI_ADDRESS_24_BITS | QSPI_DATA_8_BITS;
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

    while (qspi_flash_is_active())
        ; // wait until QSPI_Ctroller active finish
}

/**
 * @brief Switch QSPI-Flash mode between download and read/write/erase memory
 */
void change_mode_qspi_flash(void)
{
    // write back to qspi flash reg
    SPI_FLASH->MEMCTRL = 5; // (3-byte address + 1-byte 0) in Quad mode

    // wait the memCtrlChg become 0
    while (SPI_FLASH->MEMCTRL & (0x01 << 8))
        ;
}

/**
 * @brief Write data into a page of QSPI-Flash
 */
void qspi_flash_page_program(uint16_t wr_len, uint32_t address, uint8_t *buffer)
{
    if (NULL == buffer)
    {
        return;
    }

    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE |             // enable cmd
                           QSPI_ADDRESS_1LINE |                  // enable address
                           QSPI_TRMODE_WRITE_ONLY |              // trans mode = 1 (write only)
                           QSPI_DATA_1_LINE |                   // dual mode
                           QSPI_DATA_WRITE_TRANSMIT_NUM(wr_len); // set read trans byte count

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = PROGRAM_CMD; // page program cmd

    for (uint16_t i = 0; i < wr_len; i++)
    {
        // check the status of txfifo
        while (qspi_flash_tx_fifo_full())
            ;
        *((uint8_t *)&SPI_FLASH->DATA) = buffer[i];
    }

    while (qspi_flash_is_active())
        ; // wait until QSPI_ctroller active finish
}

/**
 * @brief Erase a 4K sector of QSPI-Flash
 */
void qspi_flash_4ksector_erase(uint32_t address)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_ADDRESS_1LINE |      // enable address
                           QSPI_TRMODE_NONE_DATA;

    SPI_FLASH->ADDR = address;
    SPI_FLASH->CMD = ERASE_4K_CMD; // 4K sector erase cmd

    while (qspi_flash_is_active())
        ;
}

/**
 * @brief Erase a 64K sector of QSPI-Flash
 */
void qspi_flash_64ksector_erase(uint32_t address)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_ADDRESS_1LINE |      // enable address
                           QSPI_TRMODE_NONE_DATA;
    SPI_FLASH->ADDR = address;

    SPI_FLASH->CMD = ERASE_64K_CMD; // 64K sector erase cmd

    while (qspi_flash_is_active())
        ;
}

void qspi_flash_32ksector_erase(uint32_t address)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_ADDRESS_1LINE |      // enable address
                           QSPI_TRMODE_NONE_DATA;

    SPI_FLASH->ADDR = address;

    SPI_FLASH->CMD = ERASE_32K_CMD; // 64K sector erase cmd

    while (qspi_flash_is_active())
        ;
}

/**
 * @brief Erase full chip of QSPI-Flash
 */
void qspi_flash_chip_erase(void)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | // enable cmd
                           QSPI_TRMODE_NONE_DATA;

    SPI_FLASH->CMD = ERASE_CHIP_CMD; // full chip erase cmd

    while (qspi_flash_is_active())
        ; // wait until QSPI_ctroller active finish
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
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | QSPI_TRMODE_NONE_DATA;
    SPI_FLASH->CMD = cmd; // Start to transfer
    // Check the status of Transfer
    while (qspi_flash_is_active())
        ;
}

void qspi_flash_write_enable(void)
{
    qspi_flash_write_cmd(WRITE_ENABLE_CMD);
}

void qspi_flash_write_disable(void)
{
    qspi_flash_write_cmd(WRITE_DISABLE_CMD);
}

/**
 * @brief Read QSPI-Flash status
 */
uint8_t qspi_flash_read_status(void)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | //
                           QSPI_ADDRESS_NONE |       //
                           QSPI_DATA_1_LINE |        //
                           QSPI_TRMODE_READ_ONLY |   //
                           QSPI_DATA_READ_TRANSMIT_NUM(1);

    SPI_FLASH->CMD = READ_STATUS_CMD; // read status

    while (qspi_flash_is_active())
        ;

    return SPI_FLASH->DATA;
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
        while (qspi_flash_is_busy())
            ;
        qspi_flash_write_cmd(WRITE_DISABLE_CMD);
    }
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

    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | //
                           QSPI_ADDRESS_NONE |       //
                           QSPI_DATA_1_LINE |        //
                           QSPI_TRMODE_READ_ONLY |   //
                           QSPI_DATA_WRITE_TRANSMIT_NUM(1);

    SPI_FLASH->CMD = com; // Start to transfer

    SPI_FLASH->DATA = byte;

    while (qspi_flash_is_active())
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

    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | //
                           QSPI_ADDRESS_NONE |       //
                           QSPI_DATA_1_LINE |        //
                           QSPI_TRMODE_READ_ONLY |   //
                           QSPI_DATA_READ_TRANSMIT_NUM(1);

    SPI_FLASH->CMD = com; // read status

    byte = SPI_FLASH->DATA;

    while (qspi_flash_is_active())
        ; // wait until QSPI_Ctroller active finish

    return byte;
}

uint8_t qspi_flash_chip_reset(void)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | QSPI_TRMODE_NONE_DATA;
    SPI_FLASH->CMD = 0x66;
    while (SPI_FLASH->STATUS & SPIFLASH_SR_PRGBUSY)
        ;
    SPI_FLASH->CMD = 0x99;
    while (SPI_FLASH->STATUS & SPIFLASH_SR_PRGBUSY)
        ;
    qspi_flash_read_status();

    return 0;
}

uint8_t qspi_flash_read_unique_id(uint8_t *write_buffer)
{
    SPI_FLASH->TRANSCTRL = QSPI_INSTRUCTION_1_LINE | //
                           QSPI_ADDRESS_NONE |       //
                           QSPI_DATA_1_LINE |        //
                           QSPI_TRMODE_DUMMY_READ |  //
                           QSPI_DUMMY_NUM(4) |       //
                           QSPI_DATA_READ_TRANSMIT_NUM(16);

    SPI_FLASH->CMD = 0x4b;

    for (uint16_t i = 0; i < 16; i++)
    {
        // check the status of txfifo
        while (qspi_flash_rx_fifo_empty())
            ;
        write_buffer[i] = SPI_FLASH->DATA;
    }

    return 0;
}

int qspi_flash_is_busy(void)
{
    return qspi_flash_read_status() & 0x01;
}
