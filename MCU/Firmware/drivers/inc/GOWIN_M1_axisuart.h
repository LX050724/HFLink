#ifndef GOWIN_M1_AXISUART_H
#define GOWIN_M1_AXISUART_H

#include <GOWIN_M1.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void axisuart_enable(AXIS_UART_TypeDef *uart)
{
    uart->CR |= AXISUART_CR_EN;
}

/**
 * @brief
 *
 * @param uart
 * @param party 0-None; 1-Odd; 2-Even
 */
void axisuart_set_party(AXIS_UART_TypeDef *uart, uint32_t party);

/**
 * @brief 
 * 
 * @param uart 
 * @param stop_bit 0-1b; 1:1.5b; 2-2b
 */
static inline void axisuart_set_stop_bit(AXIS_UART_TypeDef *uart, uint32_t stop_bit)
{
    uart->CR =
        (uart->CR & ~AXISUART_CR_STOP_BIT_Msk) | ((stop_bit << AXISUART_CR_STOP_BIT_Pos) & AXISUART_CR_STOP_BIT_Msk);
}

static inline void axisuart_set_rts(AXIS_UART_TypeDef *uart, uint8_t rts)
{
    uart->CR = (uart->CR & ~AXISUART_CR_RTS) | ((!!rts) << AXISUART_CR_RTS_Pos);
}

static inline void axisuart_set_dtr(AXIS_UART_TypeDef *uart, uint8_t dtr)
{
    uart->CR = (uart->CR & ~AXISUART_CR_DTR) | ((!!dtr) << AXISUART_CR_DTR_Pos);
}

void axisuart_set_baud(AXIS_UART_TypeDef *uart, uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_AXISUART_H */
