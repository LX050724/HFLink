#include "GOWIN_M1.h"
#include "GOWIN_M1_axisuart.h"
#include "GOWIN_M1_dap.h"
#include "GOWIN_M1_usbd.h"
#include "core_cm1.h"
#include "upgrade/upgrade.h"
#include <stdint.h>
#include <string.h>

#include "ads1115/ads1115.h"
#include "dap/dap.h"
#include "usb/usbd_core.h"
#include <GOWIN_M1_qspi_flash.h>

#ifdef DEBUG
#include "SEGGER_RTT.h"
#define print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define print(fmt, ...)
#endif

static volatile uint32_t delay;

uint32_t crc32(uint32_t init, uint8_t *data, uint32_t length);

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

uint8_t rbuff[512];

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

    qspi_flash_init();
    qspi_flash_Enable();

    do
    {
        uint8_t flash_unique_id[16];
        qspi_flash_read_unique_id(flash_unique_id);
        usbd_set_serial_number(flash_unique_id);
    } while (0);

    NVIC_EnableIRQ(EXTINT_0_IRQn);
    print("CR:%08x\n", USBD->CR);

    usbd_init_desc();
    usbd_enable(USBD);
    // usbd_enable_it(USBD, USBD_CR_IT_EPOUT | USBD_CR_IT_EPIN | USBD_CR_IT_SETUP);

    axisuart_set_baud(AXIS_UART, 115200);
    axisuart_enable(AXIS_UART);

    // 加载DAP控制器默认参数
    dap_baud_set_reload(DAP, 10);
    dap_baud_set_simpling_cmp(DAP, 10);
    dap_swj_set_wait_retry(DAP, 100);
    dap_swj_set_match_retry(DAP, 100);
    dap_baud_start(DAP);

    //
    DAP->GPIO.TCK_DELAY = 60; // 750ps

    GPIO_SetBit(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_SetOutEnable(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    // ads1115_i2c_init();

    while (1)
    {
        upgrade_loop();
    }
}

void EXTINT_0_Handler(void)
{
    usbd_irq_handler(USBD);
}

void EXTINT_1_Handler(void)
{
    dap_irq_handler(DAP);
}

#define DAP_VENDOR_CMD_UPGRADE 0x00
#define DAP_VENDOR_CMD_SENSOR 0x01
#define DAP_VENDOR_CMD_CONFIG 0x02

void dap_vendor0_handler(DAP_TypeDef *dap)
{
    uint8_t cmd = dap_read_data(dap);

    switch (cmd)
    {
    case DAP_VENDOR_CMD_UPGRADE: {
        uint8_t sub_command = dap_read_data(dap);
        if (sub_command == 0x00)
        {
            uint32_t firm_size = dap_read_data32(dap);
            int ret = upgrade_start(firm_size);
            dap_write_data(dap, ret);
        }
        else if (sub_command == 0x01)
        {
            volatile uint8_t *buffer = upgrade_get_buffer();
            for (int i = 0; i < 256; i += 4)
            {
                if (buffer)
                {
                    buffer[i + 0] = dap_read_data(dap);
                    buffer[i + 1] = dap_read_data(dap);
                    buffer[i + 2] = dap_read_data(dap);
                    buffer[i + 3] = dap_read_data(dap);
                }
                else
                {
                    dap_read_data(dap);
                    dap_read_data(dap);
                    dap_read_data(dap);
                    dap_read_data(dap);
                }
            }

            if (buffer != NULL)
            {
                upgrade_received_data();
                dap_write_data(dap, 0x00);
            }
            else
            {
                dap_write_data(dap, 0xff);
            }
        }
        else if (sub_command == 0x02)
        {
            uint32_t checksum = upgrade_get_verify_result();
            dap_write_data32(dap, checksum);
        }
        else if (sub_command == 0xff)
        {
            upgrade_reset();
        }
        break;
    }
    }
}
