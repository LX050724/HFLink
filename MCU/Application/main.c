#include "GOWIN_M1.h"
#include "GOWIN_M1_usbd.h"
#include "core_cm1.h"
#include <stdint.h>
#include <string.h>

#include "usb/usbd.h"
#include <GOWIN_M1_qspi_flash.h>

#ifdef DEBUG
#include "SEGGER_RTT.h"
#define print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define print(fmt, ...)
#endif

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

uint8_t rbuff[2048];

int main()
{
#ifdef DEBUG
    SEGGER_RTT_Init();
#endif

    NVIC_SetPriority(SysTick_IRQn, 15);

    SysTick->LOAD = 60000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    NVIC_EnableIRQ(EXTINT_1_IRQn);
    DAP->CR = 0x80000001;
    DAP->TIMESTAMP = 0;
    print("DAP->TIME %08x\n", DAP->TIMESTAMP);

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
    print("SPI_FLASH TxFIFO Depth: %d\n",
          (SPI_FLASH->CONFIG & SPIFLASH_CONF_TX_DEPTH_Msk) >> SPIFLASH_CONF_TX_DEPTH_Pos);
    print("SPI_FLASH RxFIFO Depth: %d\n",
          (SPI_FLASH->CONFIG & SPIFLASH_CONF_RX_DEPTH_Msk) >> SPIFLASH_CONF_RX_DEPTH_Pos);
    print("DAP->TIME %08x\n", DAP->TIMESTAMP);
    // qspi_flash_chip_reset();
    // change_mode_qspi_flash();
    
    usbd_init_desc();
    usbd_enable(USBD);
    usbd_enable_it(USBD, USBD_CR_IT_EPOUT | USBD_CR_IT_EPIN | USBD_CR_IT_SETUP);

    NVIC_EnableIRQ(EXTINT_0_IRQn);
    print("CR:%08x\n", USBD->CR);
    
    uint32_t baud = (SystemCoreClock * 8) / 921600 + 1;
    baud = (baud >> 1) - 0x10;

    AXIS_UART->BAUD = baud;
    AXIS_UART->CR = 1;

    while (1)
    {
    }
}

uint16_t xxxx[16];
uint8_t xi;

void EXTINT_0_Handler(void)
{
    uint32_t usbd_sr = USBD->SR;
    usbd_clear_flag(USBD, usbd_sr & 0xF0000000);

    usbd_ep0_rx_irq_handler(USBD, usbd_sr);
    usbd_ep_readall(USBD, 0, rbuff, sizeof(rbuff));
}

void send_str(const char *s)
{
    int n = strlen(s);
    *((uint8_t *)&DAP->DR) = 0x00;
    *((uint8_t *)&DAP->DR) = n + 1;
    for (int i = 0; i <= n; i++)
        *((uint8_t *)&DAP->DR) = s[i];
}

void EXTINT_1_Handler(void)
{
    switch (DAP->CURCMD) {
        case 0x00: {
            uint8_t ID = *((uint8_t *)&DAP->DR);
            print("DAP_Info %02x\n", ID);
            switch (ID) {
                case 0x01:
                    send_str("HFLink");
                    break;
                case 0x02:
                    send_str("HFLink CMSIS-DAP");
                    break;
                case 0x03:
                    send_str("12345678");
                    break;
            }
            break;
        }
    
    }
    DAP->SR = 0x80000000;
}

