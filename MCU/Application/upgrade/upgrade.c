#include "upgrade.h"
#include <GOWIN_M1_qspi_flash.h>
#include <string.h>
#include "board.h"

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
static volatile uint8_t upg_buffer[256];


void upgrade_loop(void)
{
    switch (status)
    {
    case UPG_IDLE:
        break;
    case UPG_ERASE: {
        for (uint32_t addr = FPGA_BITSTREAM_BEGIN_ADDR; addr < FPGA_BITSTREAM_END_ADDR; addr += 0x10000)
        {
            do
            {
                qspi_flash_write_enable();
            } while ((qspi_flash_read_status() & 0x02) == 0);
            qspi_flash_64ksector_erase(addr);
            while (qspi_flash_is_busy())
            {
            }
        }
        status = UPG_WAIT_DATA;
        upg_write_addr = FPGA_BITSTREAM_BEGIN_ADDR;
        break;
    }
    case UPG_WAIT_DATA: {
        break;
    }
    case UPG_PROGRAM: {
        do
        {
            qspi_flash_write_enable();
        } while ((qspi_flash_read_status() & 0x02) == 0);
        qspi_flash_page_program(256, upg_write_addr, upg_buffer);
        while (qspi_flash_is_busy())
        {
        }
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
            qspi_flash_fast_read_quad(addr, upg_buffer, len);
            temp = crc32(~temp, upg_buffer, len);
        }
        upg_checksum = temp;
        status = UPG_IDLE;
        break;
    }
    }
}

int upgrade_start(uint32_t firm_size)
{
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
        return upg_buffer;
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
