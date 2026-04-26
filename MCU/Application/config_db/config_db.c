#include "GOWIN_M1_qspi_flash.h"
#include "board.h"
#include "config_db.h"
#include <string.h>

#define FLASH_CONFIG_END_ADDR (0x800000)
#define FLASH_PAGE_SIZE 256
#define FLASH_CONFIG_ADDR (FLASH_CONFIG_END_ADDR - 4096)
#define CONFIG_MAGIC 0x12345678
#define CONFIG_VERSION 0x00000001

ConfigData_t global_config;

static uint32_t next_flash_addr;

int config_data_load()
{
    ConfigData_t temp_config;
    uint32_t last_valid_addr = 0;
    uint32_t crc;
    uint32_t addr;

    next_flash_addr = FLASH_CONFIG_ADDR;
    addr = FLASH_CONFIG_ADDR;

    while (addr < FLASH_CONFIG_END_ADDR)
    {
        qspi_flash_fast_read_quad(addr, (uint8_t *)&temp_config, sizeof(temp_config));
        crc = crc32(0xffffffff, (uint8_t *)&temp_config.clock_freq_mapping,
                    sizeof(temp_config) - offsetof(ConfigData_t, clock_freq_mapping));

        if (temp_config.magic == CONFIG_MAGIC && temp_config.version == CONFIG_VERSION &&
            crc == temp_config.crc32)
        {
            global_config = temp_config;
            last_valid_addr = addr;
        }
        
        addr += FLASH_PAGE_SIZE;
    }

    if (last_valid_addr != 0)
    {
        next_flash_addr = last_valid_addr + FLASH_PAGE_SIZE;
        if (next_flash_addr == FLASH_CONFIG_END_ADDR)
        {
            next_flash_addr = FLASH_CONFIG_ADDR;
        }
        return 0;
    }

    // 加载失败，使用默认配置
    next_flash_addr = FLASH_CONFIG_ADDR;
    global_config = (ConfigData_t){
        .magic = CONFIG_MAGIC,
        .version = CONFIG_VERSION,
        .clock_freq_mapping =
            {
                10000000,
                5000000,
                2000000,
                1000000,
                500000,
                200000,
                100000,
                50000,
                20000,
                10000,
                5000,
            },
        .nickname = "",
        .iodelay_param =
            {
                60,
                0,
                0,
                0,
                0,
                60,
            },
        .led_mode = 0,
        .freq_mapping_enable = 0,
        .supply5V_enable = 1,
        .indep_uart_enable = 1,
    };

    config_data_save();
    return -1;
}

int config_data_save()
{
    uint32_t crc = crc32(0xffffffff, (uint8_t *)&global_config.clock_freq_mapping,
                         sizeof(global_config) - offsetof(ConfigData_t, clock_freq_mapping));
    if (global_config.crc32 == crc)
    {
        return 0;
    }

    // 更新CRC32后再保存，以保证数据一致性
    global_config.crc32 = crc;

    // 如果当前页地址为0或配置区起始地址，则先擦除当前页，以保证数据写入成功
    if (next_flash_addr == 0 || next_flash_addr == FLASH_CONFIG_ADDR)
    {
        next_flash_addr = FLASH_CONFIG_ADDR;
        do
        {
            qspi_flash_write_enable();
        } while ((qspi_flash_read_status() & 0x02) == 0);
        qspi_flash_4ksector_erase(next_flash_addr);
        while (qspi_flash_is_busy())
        {
        }
    }

    // 写入数据
    do
    {
        qspi_flash_write_enable();
    } while ((qspi_flash_read_status() & 0x02) == 0);
    qspi_flash_page_program(FLASH_PAGE_SIZE, next_flash_addr, (uint8_t *)&global_config);
    while (qspi_flash_is_busy())
    {
    }

    // 更新页地址，如果超过配置区范围则回绕到起始地址
    next_flash_addr += FLASH_PAGE_SIZE;
    if (next_flash_addr == FLASH_CONFIG_END_ADDR)
    {
        next_flash_addr = FLASH_CONFIG_ADDR;
    }
    return 0;
}