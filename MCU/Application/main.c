#include "GOWIN_M1.h"
#include "GOWIN_M1_axisuart.h"
#include "GOWIN_M1_dap.h"
#include "GOWIN_M1_usbd.h"
#include "core_cm1.h"
#include <stdint.h>
#include <string.h>

#include "dap/dap.h"
#include "usb/usbd_core.h"
#include "ads1115/ads1115.h"
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

int main(void)
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
    
    qspi_flash_init();
    qspi_flash_Enable();

    do {
        uint8_t flash_unique_id[16];
        qspi_flash_read_unique_id(flash_unique_id);
        usbd_set_serial_number(flash_unique_id);
    } while(0);

    
    usbd_init_desc();
    usbd_enable(USBD);
    usbd_enable_it(USBD, USBD_CR_IT_EPOUT | USBD_CR_IT_EPIN | USBD_CR_IT_SETUP);

    NVIC_EnableIRQ(EXTINT_0_IRQn);
    print("CR:%08x\n", USBD->CR);
    
    axisuart_set_baud(AXIS_UART, 115200);
    axisuart_enable(AXIS_UART);

    dap_baud_set_reload(DAP, 10);
    dap_baud_set_simpling_cmp(DAP, 10);
    DAP->SWJ.WAIT_RETRY = 100;
    DAP->SWJ.MATCH_RETRY = 100;
    dap_baud_start(DAP);

    DAP->GPIO.TCK_DELAY = 60; // 750ps
    
    GPIO_SetBit(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_SetOutEnable(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    // ads1115_i2c_init();


    while (1)
    {
        GPIO_SetBit(GPIO0, GPIO_Pin_1);
        delay_1ms(300);
        GPIO_ResetBit(GPIO0, GPIO_Pin_1);
        delay_1ms(300);
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
    *((uint8_t *)&DAP->DR) = n + 1;
    for (int i = 0; i <= n; i++)
        *((uint8_t *)&DAP->DR) = s[i];
}

void EXTINT_1_Handler(void)
{
    dap_irq_handler(DAP);
}

