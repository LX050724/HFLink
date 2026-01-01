#include <GOWIN_M1_axisuart.h>

void axisuart_set_party(AXIS_UART_TypeDef *uart, uint32_t party)
{
    if (party == 0)
    {
        uart->CR &= ~AXISUART_CR_PARTY_EN;
    }
    else if (party == 1)
    {
        uart->CR |= AXISUART_CR_PARTY_EN | AXISUART_CR_PARTY_ODD;
    }
    else
    {
        uart->CR = AXISUART_CR_PARTY_EN | (uart->CR & ~AXISUART_CR_PARTY_ODD);
    }
}


void axisuart_set_baud(AXIS_UART_TypeDef *uart, uint32_t baud)
{
    baud = (SystemCoreClock * 8) / baud + 1;
    baud = (baud >> 1) - 0x10;
    uart->BAUD = baud;
}
