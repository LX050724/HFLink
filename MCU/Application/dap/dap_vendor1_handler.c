#include "GOWIN_M1_dap.h"
#include "board.h"
#include "dap.h"
#include "spiflash/spiflash.h"

#define DAP_FLASH_CMD_CONNECT 0x00
#define DAP_FLASH_CMD_READ_IDENTIFICATION_ID 0x01
#define DAP_FLASH_CMD_READ_SFDP 0x02
#define DAP_FLASH_CMD_READ_STATUS_REG 0x03
#define DAP_FLASH_CMD_WRITE_STATUS_REG 0x04
#define DAP_FLASH_CMD_WRITE_DATA 0x05
#define DAP_FLASH_CMD_READ_DATA 0x06
#define DAP_FLASH_CMD_EARSE 0x07
#define DAP_FLASH_CMD_EARSE_CHIP 0x08
#define DAP_FLASH_CMD_RESET_CHIP 0x09

void dap_vendor1_handler(DAP_TypeDef *dap)
{
    uint8_t sub_cmd = dap_read_data(dap);
    switch (sub_cmd)
    {
    case DAP_FLASH_CMD_CONNECT: {
        uint8_t enable = dap_read_data(dap);        
        if (enable)
        {
            uint8_t clk_div = dap_read_data(dap);
            uint8_t mode = dap_read_data(dap);
            switch (clk_div) {
                case 0x02:
                    clk_div = CLKSEL_CLK_DIV_2;
                    break;
                case 0x04:
                    clk_div = CLKSEL_CLK_DIV_4;
                    break;
                case 0x06:
                    clk_div = CLKSEL_CLK_DIV_6;
                    break;
                case 0x08:
                    clk_div = CLKSEL_CLK_DIV_8;
                    break;
                default:
                    clk_div = 0;
                    break;
            }

            if (clk_div && mode < 4)
            {
                spiflash_init(SPI, clk_div, mode);
                dap_gpio_enable_spi_mode(dap);
                dap_write_data(dap, 0x00);
            }
            else
            {
                dap_write_data(dap, 0xFF);
            }
        }
        else
        {
            spiflash_init(SPI, CLKSEL_CLK_DIV_2, 0);
            dap_gpio_disable_spi_mode(dap);
            dap_write_data(dap, 0x00);
        }
        break;
    }
    case DAP_FLASH_CMD_READ_IDENTIFICATION_ID: {
        // 读取Flash ID
        uint8_t id[3];
        spiflash_read_identification_id(SPI, id);
        dap_write_data(dap, id[0]);
        dap_write_data(dap, id[1]);
        dap_write_data(dap, id[2]);
        break;
    }
    case DAP_FLASH_CMD_READ_SFDP: {
        // 读取Flash SFDP
        spiflash_read_sfdp_table(SPI, 0, shared_buffer, sizeof(shared_buffer));
        for (uint32_t i = 0; i < sizeof(shared_buffer); i++)
        {
            dap_write_data(dap, shared_buffer[i]);
        }
        break;
    }
    case DAP_FLASH_CMD_READ_STATUS_REG: {
        uint8_t sr_index = dap_read_data(dap);
        const uint8_t sr_cmd[] = {
            SPIFLASH_CMD_READ_STATUS_SR1,
            SPIFLASH_CMD_READ_STATUS_SR2,
            SPIFLASH_CMD_READ_STATUS_SR3,
        };
        // 读取Flash状态寄存器
        if (sr_index < sizeof(sr_cmd))
        {
            uint8_t status = spiflash_read_status_reg(SPI, sr_cmd[sr_index]);
            dap_write_data(dap, 0x00);
            dap_write_data(dap, status);
        }
        else
        {
            // 无效的SR索引
            dap_write_data(dap, 0xFF);
        }
        break;
    }
    case DAP_FLASH_CMD_WRITE_STATUS_REG: {
        // 写入Flash状态寄存器
        uint8_t sr_index = dap_read_data(dap);
        uint8_t status = dap_read_data(dap);
        const uint8_t sr_cmd[] = {
            SPIFLASH_CMD_WRITE_STATUS_SR1,
            SPIFLASH_CMD_WRITE_STATUS_SR2,
            SPIFLASH_CMD_WRITE_STATUS_SR3,
        };
        if (sr_index < sizeof(sr_cmd))
        {
            spiflash_write_status_reg(SPI, sr_cmd[sr_index], status);
            dap_write_data(dap, 0x00);
        }
        else
        {
            // 无效的SR索引
            dap_write_data(dap, 0xFF);
        }
        break;
    }
    case DAP_FLASH_CMD_READ_DATA: {
        // 读取Flash数据
        uint32_t addr = dap_read_data32(dap);
        uint32_t len = dap_read_data32(dap);
        if (len > sizeof(shared_buffer))
        {
            dap_write_data(dap, 0xFF);
        }
        else
        {
            spiflash_read_data(SPI, addr, shared_buffer, len);
            dap_write_data(dap, 0x00);
            for (uint32_t i = 0; i < len; i++)
            {
                dap_write_data(dap, shared_buffer[i]);
            }
        }
        break;
    }
    case DAP_FLASH_CMD_EARSE: {
        uint32_t erased_addr = dap_read_data32(dap);
        uint32_t remaining_len = dap_read_data32(dap);

        if ((erased_addr & 0xFFF) != 0)
        {
            dap_write_data(dap, 0xFF);
            break;
        }

        while (remaining_len > 0)
        {
            if ((erased_addr % 0x10000 == 0) && (remaining_len >= 0x10000))
            {
                spiflash_erase_block_64k(SPI, erased_addr);
                erased_addr += 0x10000;
                remaining_len -= 0x10000;
            }
            else if ((erased_addr % 0x8000 == 0) && (remaining_len >= 0x8000))
            {
                spiflash_erase_block_32k(SPI, erased_addr);
                erased_addr += 0x8000;
                remaining_len -= 0x8000;
            }
            else
            {
                spiflash_erase_sector(SPI, erased_addr);
                erased_addr += 0x1000;
                remaining_len -= 0x1000;
            }
        }

        dap_write_data(dap, 0x00);
        break;
    }
    case DAP_FLASH_CMD_EARSE_CHIP: {
        // 擦除Flash芯片
        spiflash_erase_chip(SPI);
        dap_write_data(dap, 0x00);
        break;
    }
    case DAP_FLASH_CMD_RESET_CHIP: {
        // 重置Flash芯片
        spiflash_reset_chip(SPI);
        dap_write_data(dap, 0x00);
        break;
    }
    default: {
        // 无效的命令
        dap_write_data(dap, 0xFF);
        break;
    }
    }
}
