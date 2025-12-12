#include "GOWIN_M1.h"

#include "SEGGER_RTT.h"
#include "core_cm1.h"
#include <stdint.h>
#include <string.h>

#include "usb/usbd.h"
#include <GOWIN_M1_qspi_flash.h>

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

int main()
{
    SEGGER_RTT_Init();

    NVIC_SetPriority(SysTick_IRQn, 15);

    SysTick->LOAD = 60000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // NVIC_SetPriority(UserInterrupt0_IRQn, 4);
    // NVIC_EnableIRQ(UserInterrupt0_IRQn);

    qspi_flash_init();
    // change_mode_qspi_flash();

    usbd_init_desc();
    usbd_enable();

    uint32_t address = 0;

    while (1)
    {
        uint8_t buf[16];
        qspi_flash_fast_read_quad(address, buf, 16);
        address += 16;
        
        SEGGER_RTT_printf(0, "%08x: ", address);
        for (int i = 0; i < 16; i++)
        {
            SEGGER_RTT_printf(0, "%02x ", buf[i]);
        }
        SEGGER_RTT_printf(0, "\n");
        delay_1ms(100);
    }
}
