#include "GOWIN_M1.h"
#include "GOWIN_M1_axisuart.h"
#include "GOWIN_M1_dap.h"
#include "GOWIN_M1_usbd.h"
#include "core_cm1.h"
#include "soft_timer.h"
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

#define LED_RED_PIN GPIO_Pin_0
#define LED_GREEN_PIN GPIO_Pin_1
#define LED_BLUE_PIN GPIO_Pin_2
#define LED_ALL_PIN (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2)

#define POWER_CTL_PIN GPIO_Pin_3

uint8_t adc_channel;
// 0: VREF*4; 1: CURRENT
int16_t adc_result[2];

uint8_t led_status;

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

    axisuart_set_baud(AXIS_UART, 115200);
    axisuart_enable(AXIS_UART);

    // 加载DAP控制器默认参数
    dap_baud_set_reload(DAP, 10);
    dap_baud_set_simpling_cmp(DAP, 10);
    dap_swj_set_wait_retry(DAP, 100);
    dap_swj_set_match_retry(DAP, 100);
    dap_swd_set_trun_cycle(DAP, 0);
    dap_swd_enable_turn_clk(DAP);
    dap_swj_set_mode(DAP, DAP_SWJ_MODE_SWD);
    dap_gpio_enable_alone_uart(DAP);

    dap_baud_start(DAP);

    //
    DAP->GPIO.TCK_DELAY = 60; // 750ps

    GPIO_SetBit(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_SetOutEnable(GPIO0, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    ads1115_i2c_init();
    adc_channel = ADS1115_MUX_A0_GND;
    ads1115_start_conv(ADS1115_PGA_4096, adc_channel);

    while (1)
    {
        if (!ads1115_is_busy())
        {
            if (adc_channel == ADS1115_MUX_A0_GND)
            {
                adc_result[0] = ads1115_read_result();
                adc_channel = ADS1115_MUX_A1_GND;
                ads1115_start_conv(ADS1115_PGA_1024, adc_channel);
            }
            else
            {
                adc_result[1] = ads1115_read_result() - 433;
                adc_channel = ADS1115_MUX_A0_GND;
                ads1115_start_conv(ADS1115_PGA_4096, adc_channel);
            }
        }

        if (timer_get_tick(TIMER_LED) == 0)
        {
            timer_set_tick(TIMER_LED, 10);
            if (adc_result[0] < 1200 / 4)
            {
                GPIO_SetBit(GPIO0, LED_ALL_PIN);
                GPIO_ResetBit(GPIO0, LED_RED_PIN);
            }
            else
            {
                GPIO_SetBit(GPIO0, LED_ALL_PIN);
                GPIO_ResetBit(GPIO0, LED_GREEN_PIN);
            }
            led_status = !led_status;
        }

        upgrade_loop();
    }
}
