#include "upgrade.h"
#include <string.h>
#include "board.h"
#include "spiflash/spiflash.h"
#include "GOWIN_M1_dap.h"

#define FPGA_BITSTREAM_BEGIN_ADDR 0x00000000U
#define FPGA_BITSTREAM_END_ADDR 0x00100000U

typedef enum
{
    UPG_IDLE,
    UPG_ERASE,
    UPG_WAIT_DATA,
    UPG_PROGRAM,
    UPG_VERIFY,
} UPG_Status;

static volatile uint32_t upg_firm_size;
static volatile uint32_t upg_write_addr;
static volatile uint32_t upg_checksum = 0xFFFFFFFFU;
static volatile uint8_t upg_busy_flag;
static volatile UPG_Status status;


void upgrade_loop(void)
{
    switch (status)
    {
    case UPG_IDLE:
        break;
    case UPG_ERASE: {
        for (uint32_t addr = FPGA_BITSTREAM_BEGIN_ADDR; addr < FPGA_BITSTREAM_END_ADDR; addr += 0x10000)
        {
            spiflash_erase_block_64k(SPI, addr);
        }
        status = UPG_WAIT_DATA;
        upg_write_addr = FPGA_BITSTREAM_BEGIN_ADDR;
        break;
    }
    case UPG_WAIT_DATA: {
        break;
    }
    case UPG_PROGRAM: {
        spiflash_page_program(SPI, upg_write_addr, shared_buffer, 256);
        upg_write_addr += 256;
        if (upg_write_addr >= upg_firm_size)
        {
            status = UPG_VERIFY;
        }
        else
        {
            status = UPG_WAIT_DATA;
        }
        break;
    }
    case UPG_VERIFY: {
        uint32_t temp = 0;
        for (uint32_t addr = FPGA_BITSTREAM_BEGIN_ADDR; addr < (FPGA_BITSTREAM_BEGIN_ADDR + upg_firm_size); addr += 256)
        {
            uint32_t len = upg_firm_size - (addr - FPGA_BITSTREAM_BEGIN_ADDR);
            if (len > 256)
                len = 256;
            spiflash_read_data(SPI, addr, shared_buffer, len);
            temp = crc32(~temp, shared_buffer, len);
        }
        upg_checksum = temp;
        status = UPG_IDLE;
        break;
    }
    }
}

int upgrade_start(uint32_t firm_size)
{
    if (dap_gpio_is_spi_mode(DAP))
    {
        return -1;
    }
    if (status == UPG_IDLE)
    {
        upg_firm_size = firm_size;
        status = UPG_ERASE;
        upg_busy_flag = 1;
        upg_checksum = 0;
        return 0;
    }
    return -1;
}

volatile uint8_t *upgrade_get_buffer(void)
{
    if (status == UPG_WAIT_DATA)
        return shared_buffer;
    return NULL;
}

int upgrade_received_data(void)
{
    if (status == UPG_WAIT_DATA)
    {
        status = UPG_PROGRAM;
        return 0;
    }
    return -1;
}

uint32_t upgrade_get_verify_result(void)
{
    return upg_checksum;
}

void upgrade_reset(void)
{
    upg_checksum = 0;
    status = UPG_IDLE;
}
