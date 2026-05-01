#include "spiflash.h"
#include "GOWIN_M1_spi.h"

static uint8_t spi_transfer_byte(SPI_TypeDef *spi, uint8_t byte)
{
    spi->WDATA = byte;
    while (!(spi->STATUS & SPI_STATUS_RRDY));
    return spi->RDATA;
}


void spiflash_init(SPI_TypeDef *spi, uint8_t clk_div, uint8_t mode)
{
    uint32_t CR = SHIFT_DIR_MSB_FIRST;
    CR |= (clk_div << SPI_CR_CLKSEL_Pos);
    CR |= mode << SPI_CR_PHASE_Pos;

    spi->CTRL = CR;
}

void spiflash_read_unique_id(SPI_TypeDef *spi, uint8_t *id)
{
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_READ_UNIQUE_ID);
    spi_transfer_byte(spi, 0x00);
    spi_transfer_byte(spi, 0x00);
    spi_transfer_byte(spi, 0x00);
    spi_transfer_byte(spi, 0x00);
    for (int i = 0; i < 16; i++)
    {
        id[i] = spi_transfer_byte(spi, 0x00);
    }
    spi->SSMASK = 0;
}

uint8_t spiflash_read_status_reg(SPI_TypeDef *spi, uint8_t reg_cmd)
{
    spi->SSMASK = 1;
    spi_transfer_byte(spi, reg_cmd);
    uint8_t reg = spi_transfer_byte(spi, 0x00);
    spi->SSMASK = 0;
    return reg;
}

void spiflash_write_enable(SPI_TypeDef *spi)
{
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_WRITE_ENABLE);
    spi->SSMASK = 0;
}

void spiflash_write_disable(SPI_TypeDef *spi)
{
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_WRITE_DISABLE);
    spi->SSMASK = 0;
}

void spiflash_wait_busy(SPI_TypeDef *spi)
{
    while (spiflash_read_status_reg(spi, SPIFLASH_CMD_READ_STATUS_SR1) & 0x01);
}

void spiflash_write_status_reg(SPI_TypeDef *spi, uint8_t reg_cmd, uint8_t sr)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, reg_cmd);
    spi_transfer_byte(spi, sr);
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_erase_sector(SPI_TypeDef *spi, uint32_t addr)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_ERASE_SECTOR);
    spi_transfer_byte(spi, (addr >> 16) & 0xFF);
    spi_transfer_byte(spi, (addr >> 8) & 0xFF);
    spi_transfer_byte(spi, addr & 0xFF);
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_erase_block_32k(SPI_TypeDef *spi, uint32_t addr)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_ERASE_BLOCK_32K);
    spi_transfer_byte(spi, (addr >> 16) & 0xFF);
    spi_transfer_byte(spi, (addr >> 8) & 0xFF);
    spi_transfer_byte(spi, addr & 0xFF);
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_erase_block_64k(SPI_TypeDef *spi, uint32_t addr)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_ERASE_BLOCK_64K);
    spi_transfer_byte(spi, (addr >> 16) & 0xFF);
    spi_transfer_byte(spi, (addr >> 8) & 0xFF);
    spi_transfer_byte(spi, addr & 0xFF);
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_erase_chip(SPI_TypeDef *spi)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_ERASE_CHIP);
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_page_program(SPI_TypeDef *spi, uint32_t addr, const uint8_t *data, uint16_t len)
{
    spiflash_write_enable(spi);
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_PAGE_PROGRAM);
    spi_transfer_byte(spi, (addr >> 16) & 0xFF);
    spi_transfer_byte(spi, (addr >> 8) & 0xFF);
    spi_transfer_byte(spi, addr & 0xFF);
    for (uint16_t i = 0; i < len; i++)
    {
        spi_transfer_byte(spi, data[i]);
    }
    spi->SSMASK = 0;
    spiflash_wait_busy(spi);
}

void spiflash_read_data(SPI_TypeDef *spi, uint32_t addr, uint8_t *data, uint32_t len)
{
    spi->SSMASK = 1;
    spi_transfer_byte(spi, SPIFLASH_CMD_READ_DATA);
    spi_transfer_byte(spi, (addr >> 16) & 0xFF);
    spi_transfer_byte(spi, (addr >> 8) & 0xFF);
    spi_transfer_byte(spi, addr & 0xFF);
    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = spi_transfer_byte(spi, 0x00);
    }
    spi->SSMASK = 0;
}

