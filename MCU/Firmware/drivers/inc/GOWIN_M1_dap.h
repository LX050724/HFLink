#ifndef GOWIN_M1_DAP_H
#define GOWIN_M1_DAP_H

#include <GOWIN_M1.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_GPIO_0 0
#define DAP_GPIO_1 1
#define DAP_GPIO_2 2
#define DAP_GPIO_3 3
#define DAP_GPIO_4 4
#define DAP_GPIO_5 5
#define DAP_GPIO_6 6
#define DAP_GPIO_7 7

#define DAP_GPIO_DIR_INPUT 1
#define DAP_GPIO_DIR_OUTPUT 0

inline static void dap_write_data(DAP_TypeDef *dap, uint8_t data)
{
    *((uint8_t *)&dap->DR) = data;
}

inline static uint8_t dap_read_data(DAP_TypeDef *dap)
{
    return *((uint8_t *)&dap->DR);
}

inline static uint8_t dap_get_current_cmd(DAP_TypeDef *dap)
{
    return dap->CURCMD;
}

inline static void dap_baud_start(DAP_TypeDef *dap)
{
    dap->BAUD_GEN.CR |= 1;
}

inline static void dap_baud_set_div(DAP_TypeDef *dap, uint16_t div)
{
    dap->BAUD_GEN.DIV = div;
}

inline static void dap_baud_set_delay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->BAUD_GEN.DIV = delay;
}

inline static void dap_gpio_enable_directio(DAP_TypeDef *dap, uint8_t index)
{
    dap->GPIO.CR |= 1 << index;
}

inline static void dap_gpio_disable_directio(DAP_TypeDef *dap, uint8_t index)
{
    dap->GPIO.CR &= ~(1 << index);
}

inline static void dap_gpio_set_direction(DAP_TypeDef *dap, uint8_t index, uint8_t dir)
{
    if (dir)
    {
        dap->GPIO.DIR |= 1 << index;
    }
    else
    {
        dap->GPIO.DIR &= ~(1 << index);
    }
}

inline static void dap_gpio_set_idelay(DAP_TypeDef *dap, uint8_t index, uint8_t delay)
{
    dap->GPIO.IDELAY[index] = delay;
}

inline static void dap_gpio_set_odelay(DAP_TypeDef *dap, uint8_t index, uint8_t delay)
{
    dap->GPIO.ODELAY[index] = delay;
}

inline static void dap_gpio_set_pin(DAP_TypeDef *dap, uint8_t gpio)
{
    dap->GPIO.BS = gpio;
}

inline static void dap_gpio_reset_pin(DAP_TypeDef *dap, uint8_t gpio)
{
    dap->GPIO.BR = gpio;
}

inline static uint8_t dap_gpio_read_pin(DAP_TypeDef *dap)
{
    return dap->GPIO.DI;
}

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DAP_H */
