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

#define DAP_SWJ_MODE_SWD 1
#define DAP_SWJ_MODE_JTAG 0

inline static void dap_write_data(DAP_TypeDef *dap, uint8_t data)
{
    dap->DR.WD = data;
}

void dap_write_data16(DAP_TypeDef *dap, uint16_t data);
void dap_write_data32(DAP_TypeDef *dap, uint32_t data);

uint8_t dap_read_data(DAP_TypeDef *dap);
uint16_t dap_read_data16(DAP_TypeDef *dap);
uint32_t dap_read_data32(DAP_TypeDef *dap);

inline static uint32_t dap_timestamp_get(DAP_TypeDef *dap)
{
    return dap->TIMESTAMP;
}

inline static void dap_timestamp_reset(DAP_TypeDef *dap)
{
    dap->TIMESTAMP = 0;
}

inline static uint8_t dap_get_current_cmd(DAP_TypeDef *dap)
{
    return dap->CURCMD;
}

inline static void dap_baud_start(DAP_TypeDef *dap)
{
    dap->BAUD_GEN.CR |= 1;
}

inline static void dap_baud_stop(DAP_TypeDef *dap)
{
    dap->BAUD_GEN.CR &= ~1U;
}

inline static void dap_baud_set_reload(DAP_TypeDef *dap, uint16_t div)
{
    dap->BAUD_GEN.RELOAD = div;
}

inline static uint16_t dap_baud_get_reload(DAP_TypeDef *dap)
{
    return dap->BAUD_GEN.RELOAD;
}

inline static void dap_baud_set_simpling_cmp(DAP_TypeDef *dap, uint16_t cmp)
{
    dap->BAUD_GEN.SIMPLING_CMP = cmp;
}

inline static void dap_buad_set_simpling_delay(DAP_TypeDef *dap, uint8_t cmp)
{
    dap->BAUD_GEN.CR = (dap->BAUD_GEN.CR & 0x0000ffff) | (cmp << 16);
}

inline static void dap_gpio_enable_directio(DAP_TypeDef *dap, uint8_t index)
{
    dap->GPIO.CR |= 1 << index;
}

inline static void dap_gpio_disable_directio(DAP_TypeDef *dap, uint8_t index)
{
    dap->GPIO.CR &= ~(1 << index);
}

inline static void dap_gpio_enable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= 1;
}

inline static void dap_gpio_disable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~1;
}

inline static void dap_swj_set_mode(DAP_TypeDef *dap, uint8_t mode)
{
    dap->SWJ.CR = mode;
}

inline static void dap_swj_set_wait_retry(DAP_TypeDef *dap, uint16_t cnt)
{
    dap->SWJ.WAIT_RETRY = cnt;
}

inline static void dap_swj_set_match_retry(DAP_TypeDef *dap, uint16_t cnt)
{
    dap->SWJ.MATCH_RETRY = cnt;
}

inline static void dap_swd_enable_turn_clk(DAP_TypeDef *dap)
{
    dap->SWJ.SWD_CR |= 0x8;
}

inline static void dap_swd_disable_turn_clk(DAP_TypeDef *dap)
{
    dap->SWJ.SWD_CR &= ~0x8;
}

inline static void dap_swd_set_trun_cycle(DAP_TypeDef *dap, uint8_t turn_cycle)
{
    dap->SWJ.SWD_CR = (dap->SWJ.SWD_CR & ~0x03) | (turn_cycle & 0x03);
}

inline static void dap_swd_set_data_phase(DAP_TypeDef *dap, uint8_t data_phase)
{
    dap->SWJ.SWD_CR = (dap->SWJ.SWD_CR & ~0x04) | (data_phase << 2);
}

inline static void dap_swd_set_trun_data_phase(DAP_TypeDef *dap, uint8_t value)
{
    dap->SWJ.SWD_CR = value;
}

inline static void dap_gpio_set_tck_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TCK_DELAY = delay;
}

inline static void dap_gpio_set_tms_idelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_I_DELAY = delay;
}

inline static void dap_gpio_set_tms_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_O_DELAY = delay;
}

inline static void dap_gpio_set_tms_t_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_T_DELAY = delay;
}

inline static void dap_gpio_set_tdo_idelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TDO_DELAY = delay;
}

inline static void dap_gpio_set_tdi_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TDI_DELAY = delay;
}

inline static void dap_jtag_set_irlen(DAP_TypeDef *dap, uint8_t index, uint8_t irlen)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_LEN = irlen;
}

inline static void dap_jtag_set_ir_before_len(DAP_TypeDef *dap, uint8_t index, uint8_t before_len)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_BEFORE_LEN = before_len;
}

inline static void dap_jtag_set_ir_after_len(DAP_TypeDef *dap, uint8_t index, uint8_t after_len)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_AFTER_LEN = after_len;
}

inline static void dap_jtag_set_tap_num(DAP_TypeDef *dap, uint8_t num)
{
    dap->SWJ.JTAG_CR = (dap->SWJ.JTAG_CR & ~0x0F) | (num & 0x0f);
}

// inline static void dap_gpio_set_direction(DAP_TypeDef *dap, uint8_t index, uint8_t dir)
// {
//     if (dir)
//     {
//         dap->GPIO.DIR |= 1 << index;
//     }
//     else
//     {
//         dap->GPIO.DIR &= ~(1 << index);
//     }
// }

// inline static void dap_gpio_set_idelay(DAP_TypeDef *dap, uint8_t index, uint8_t delay)
// {
//     dap->GPIO.IDELAY[index] = delay;
// }

// inline static void dap_gpio_set_odelay(DAP_TypeDef *dap, uint8_t index, uint8_t delay)
// {
//     dap->GPIO.ODELAY[index] = delay;
// }

// inline static void dap_gpio_set_pin(DAP_TypeDef *dap, uint8_t gpio)
// {
//     dap->GPIO.BS = gpio;
// }

// inline static void dap_gpio_reset_pin(DAP_TypeDef *dap, uint8_t gpio)
// {
//     dap->GPIO.BR = gpio;
// }

// inline static uint8_t dap_gpio_read_pin(DAP_TypeDef *dap)
// {
//     return dap->GPIO.DI;
// }

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DAP_H */
