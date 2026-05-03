#ifndef GOWIN_M1_DAP_H
#define GOWIN_M1_DAP_H

#include <GOWIN_M1.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_GPIO_STATUS_SRST_O 0x0001
#define DAP_GPIO_STATUS_SRST_I 0x0002
#define DAP_GPIO_STATUS_TRST_O 0x0004
#define DAP_GPIO_STATUS_TRST_I 0x0008
#define DAP_GPIO_STATUS_TDI_O 0x0010
#define DAP_GPIO_STATUS_SWO_TDO_I 0x0020
#define DAP_GPIO_STATUS_SWDIO_TMS_I 0x0040
#define DAP_GPIO_STATUS_SWDIO_TMS_O 0x0080
#define DAP_GPIO_STATUS_SWDIO_TMS_T 0x0100
#define DAP_GPIO_STATUS_SWCLK_TCK_O 0x0200

#define DAP_GPIO_DO_SWCLK_TCK_O 0x01
#define DAP_GPIO_DO_SWDIO_TMS_T 0x02
#define DAP_GPIO_DO_SWDIO_TMS_O 0x04
#define DAP_GPIO_DO_TDI_O 0x08
#define DAP_GPIO_DO_TRST_O 0x10
#define DAP_GPIO_DO_SRST_O 0x20
#define DAP_GPIO_DO_POWER_CTL_O 0x80

#define DAP_GPIO_CR_INDEPENDENT_UART_MASK 0x01
#define DAP_GPIO_CR_DIRECTIO_MASK 0x02
#define DAP_GPIO_CR_SPI_MODE_MASK 0x04
#define DAP_GPIO_CR_LED_MODE_MASK 0x08

#define DAP_GPIO_CR_CALIB_MASK 0x10
#define DAP_GPIO_CR_CALIB_SELECT_MASK 0x60
#define DAP_GPIO_CR_CALIB_SELECT_CLK 0x00
#define DAP_GPIO_CR_CALIB_SELECT_TMS 0x20
#define DAP_GPIO_CR_CALIB_SELECT_TDO 0x40


#define DAP_GPIO_LED_R 0
#define DAP_GPIO_LED_G 1
#define DAP_GPIO_LED_B 2

#define DAP_GPIO_CR_LED_MODE_MANUAL 0x00
#define DAP_GPIO_CR_LED_MODE_DEFAULT DAP_GPIO_CR_LED_MODE_MASK

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

/*************************************************************************************************************************/

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

/*************************************************************************************************************************/

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

/*************************************************************************************************************************/

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

/*************************************************************************************************************************/

inline static void dap_gpio_enable_directio(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_DIRECTIO_MASK;
}

inline static void dap_gpio_disable_directio(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_DIRECTIO_MASK;
}

inline static void dap_gpio_enable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_INDEPENDENT_UART_MASK;
}

inline static void dap_gpio_disable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_INDEPENDENT_UART_MASK;
}

inline static void dap_gpio_enable_spi_mode(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_SPI_MODE_MASK;
}

inline static void dap_gpio_disable_spi_mode(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_SPI_MODE_MASK;
}

inline static void dap_gpio_set_led_mode(DAP_TypeDef *dap, uint8_t mode)
{
    dap->GPIO.CR = (dap->GPIO.CR & ~DAP_GPIO_CR_LED_MODE_MASK) | mode;
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

inline static void dap_gpio_set_led_cmp(DAP_TypeDef *dap, uint8_t R, uint8_t G, uint8_t B)
{
    dap->GPIO.LED_CMP[0] = R;
    dap->GPIO.LED_CMP[1] = G;
    dap->GPIO.LED_CMP[2] = B;
}

inline static void dap_gpio_set_pin(DAP_TypeDef *dap, uint8_t pin_mask)
{
    dap->GPIO.DIRECTIO_SET = pin_mask;
}

inline static void dap_gpio_reset_pin(DAP_TypeDef *dap, uint8_t pin_mask)
{
    dap->GPIO.DIRECTIO_RESET = pin_mask;
}

inline static uint8_t dap_gpio_write_pin(DAP_TypeDef *dap, uint8_t pin_mask, uint8_t value)
{
    if (value)
    {
        dap_gpio_set_pin(dap, pin_mask);
    }
    else
    {
        dap_gpio_reset_pin(dap, pin_mask);
    }
    return value;
}

inline static uint16_t dap_gpio_get_status(DAP_TypeDef *dap)
{
    return dap->GPIO.GPIO_STATUS;
}

inline static uint16_t dap_gpio_get_cali_simpling(DAP_TypeDef *dap)
{
    return dap->GPIO.CALI_SIMPLING;
}

inline static void dap_gpio_enable_calib_simpling(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_CALIB_MASK;
}

inline static void dap_gpio_disable_calib_simpling(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_CALIB_MASK;
}

inline static void dap_gpio_send_break(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_CALIB_MASK;
}

inline static void dap_gpio_set_calib_select(DAP_TypeDef *dap, uint8_t select)
{
    dap->GPIO.CR = (dap->GPIO.CR & ~DAP_GPIO_CR_CALIB_SELECT_MASK) | select;
}
/*************************************************************************************************************************/

inline static void dap_jtag_set_irlen(DAP_TypeDef *dap, uint8_t index, uint8_t irlen)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_LEN = irlen;
}

inline static uint8_t dap_jtag_get_irlen(DAP_TypeDef *dap, uint8_t index)
{
    return dap->SWJ.JTAG_IR_CONF[index].IR_LEN;
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
    dap->SWJ.JTAG_CR = (dap->SWJ.JTAG_CR & ~0x0F) | num;
}



#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DAP_H */
