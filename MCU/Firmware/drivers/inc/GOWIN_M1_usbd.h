#ifndef GOWIN_M1_USBD_H
#define GOWIN_M1_USBD_H

#include <stdbool.h>
#include <GOWIN_M1.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void usbd_enable(USBD_TypeDef *usbd)
{
    usbd->CR |= USBD_CR_EN;
}

static inline void usbd_disable(USBD_TypeDef *usbd)
{
    usbd->CR &= ~USBD_CR_EN;
}


static inline uint32_t usbd_get_speed(USBD_TypeDef *usbd)
{
    return usbd->SR & USBD_SR_HISPEED;
}

static inline uint8_t usbd_get_active_ep(USBD_TypeDef *usbd)
{
    return (usbd->SR >> 3) & 0xf;
}


#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_USBD_H */
