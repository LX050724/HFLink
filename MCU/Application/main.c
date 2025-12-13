#include "GOWIN_M1.h"

#include "SEGGER_RTT.h"
#include "core_cm1.h"
#include <stdint.h>
#include <string.h>

#include "usb/usbd.h"
#include <GOWIN_M1_qspi_flash.h>


#define print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

static volatile uint32_t delay;

void delay_1ms(uint32_t count)
{
    delay = count;

    while (0U != delay)
    {
    }
}

void delay_decrement(void)
{
    if (0U != delay)
    {
        delay--;
    }
}

#define AHB_USBDevice ((USBD_TypeDef *)0x80000000)

uint8_t test_mem[2048];

uint32_t crc32(uint32_t init, uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint32_t crc = init; // Initial value
    while (length--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // 0xEDB88320= reverse 0x04C11DB7
            else
                crc = (crc >> 1);
        }
    }
    return ~crc;
}

int main()
{
    SEGGER_RTT_Init();

    NVIC_SetPriority(SysTick_IRQn, 15);

    SysTick->LOAD = 60000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // NVIC_SetPriority(UserInterrupt0_IRQn, 4);
    // NVIC_EnableIRQ(UserInterrupt0_IRQn);

    // spi_flash_init();

    qspi_flash_init();
    // qspi_flash_Enable();


    print("SPI_FLASH IDREV: %08x\n", SPI_FLASH->IDREV);
    print("SPI_FLASH Support Slave: %d\n", !!(SPI_FLASH->CONFIG & SPIFLASH_CONF_SLAVE_Msk));
    print("SPI_FLASH Support Mmap: %d\n", !!(SPI_FLASH->CONFIG & SPIFLASH_CONF_MMAP_Msk));
    print("SPI_FLASH Support direct IO: %d\n", !!(SPI_FLASH->CONFIG & SPIFLASH_CONF_DIRECT_IO_Msk));
    print("SPI_FLASH Support QuadSPI: %d\n", !!(SPI_FLASH->CONFIG & SPIFLASH_CONF_QUADIO_Msk));
    print("SPI_FLASH Support DualSPI: %d\n", !!(SPI_FLASH->CONFIG & SPIFLASH_CONF_DUALIO_Msk));
    print("SPI_FLASH TxFIFO Depth: %d\n", (SPI_FLASH->CONFIG & SPIFLASH_CONF_TX_DEPTH_Msk) >> SPIFLASH_CONF_TX_DEPTH_Pos);
    print("SPI_FLASH RxFIFO Depth: %d\n", (SPI_FLASH->CONFIG & SPIFLASH_CONF_RX_DEPTH_Msk) >> SPIFLASH_CONF_RX_DEPTH_Pos);

    qspi_flash_chip_reset();
    change_mode_qspi_flash();

    usbd_init_desc();
    usbd_enable();

    uint32_t address = 0;
    uint32_t c = 0xffffffff;

    // uint8_t sr2 = qspi_flash_read_sr(2);
    // SEGGER_RTT_printf(0, "SR2=%02x\n", sr2);
    // if ((sr2 & 0x02) == 0)
    // {
    //     SEGGER_RTT_printf(0, "enable quda\n");
    //     qspi_flash_Enable();
    //     qspi_flash_write_sr(2, sr2 | 0x02);
    //     qspi_flash_read_status();
    //     sr2 = qspi_flash_read_sr(2);
    //     SEGGER_RTT_printf(0, "SR2=%02x\n", sr2);
    // }

    while (1)
    {
        // uint8_t buf[16];
        // qspi_flash_fast_read_quad(address, buf, 16);
        // if (address + 16 > 895186)
        // {
        //     c = crc32(c, buf, 895186 % 16);
        //     SEGGER_RTT_printf(0, "crc32 %08x\n", c);
        //     while (1) {
            
        //     }
        // }
        // else
        // {
        //     c = crc32(c, buf, 16);
        // }

        // SEGGER_RTT_printf(0, "%08x: ", address);
        // for (int i = 0; i < 16; i++)
        // {
        //     SEGGER_RTT_printf(0, "%02x ", buf[i]);
        // }
        // SEGGER_RTT_printf(0, "\n");
        address += 16;
        // delay_1ms(1);
    }
}
