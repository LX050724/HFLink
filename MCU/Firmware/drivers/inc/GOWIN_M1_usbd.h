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

static inline int usbd_ep_rxfifo_is_empty(USBD_TypeDef *usbd, uint8_t ep_index)
{
    return !!(usbd->EP_RFFE & (1 << ep_index));
}

static inline uint8_t usbd_ep_read_data(USBD_TypeDef *usbd, uint8_t ep_index)
{
    return *((volatile uint8_t *)&usbd->EP_DATA[ep_index]);
}

static inline void usbd_ep_write_data(USBD_TypeDef *usbd, uint8_t ep_index, uint8_t data)
{
    *((volatile uint8_t *)&usbd->EP_DATA[ep_index]) = data;
}

static inline uint16_t usbd_ep_get_rxnum(USBD_TypeDef *usbd, uint8_t ep_index)
{
    return usbd->EP_FIFONUM[ep_index].RN;
}

static inline uint16_t usbd_ep_get_txnum(USBD_TypeDef *usbd, uint8_t ep_index)
{
    return usbd->EP_FIFONUM[ep_index].TN;
}

static inline uint32_t usbd_is_active_flag(USBD_TypeDef *usbd, uint32_t it)
{
    return !!(usbd->SR & it);
}

static inline void usbd_enable_it(USBD_TypeDef *usbd, uint32_t it)
{
    usbd->CR |= it;
}

static inline uint32_t usbd_get_speed(USBD_TypeDef *usbd)
{
    return usbd->SR & USBD_SR_HISPEED;
}

uint32_t usbd_ep_readall(USBD_TypeDef *usbd, uint8_t ep_index, uint8_t *buf, size_t bsize);
uint32_t usbd_ep_write_buffer(USBD_TypeDef *usbd, uint8_t ep_index, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_USBD_H */
