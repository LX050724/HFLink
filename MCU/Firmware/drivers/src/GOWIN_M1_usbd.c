#include <GOWIN_M1_usbd.h>

uint32_t usbd_ep_readall(USBD_TypeDef *usbd, uint8_t ep_index, void *buf, uint32_t bsize)
{
    uint32_t n = 0;
    while (!usbd_ep_rxfifo_is_empty(usbd, ep_index) && n < bsize)
        ((uint8_t *)buf)[n++] = usbd_ep_read_data(usbd, ep_index);
    return n;
}

uint32_t usbd_ep_write_buffer(USBD_TypeDef *usbd, uint8_t ep_index, const void *buf, uint32_t len)
{
    const uint8_t *p = buf;
    while (len--)
        usbd_ep_write_data(usbd, ep_index, *p++);
    return 0;
}

uint32_t usbd_ep_read_buffer(USBD_TypeDef *usbd, uint8_t ep_index, void *buf, uint32_t bsize)
{
    uint32_t n = 0;
    while (!usbd_ep_rxfifo_is_empty(usbd, ep_index) && n < bsize)
        ((uint8_t *)buf)[n++] = usbd_ep_read_data(usbd, ep_index);
    return n;
}
