#include <GOWIN_M1_usbd.h>


uint32_t usbd_ep_readall(USBD_TypeDef *usbd, uint8_t ep_index, uint8_t *buf, size_t bsize)
{
    uint32_t n = 0;
    while (!usbd_ep_rxfifo_is_empty(usbd, ep_index) && n < bsize)
        buf[n++] = usbd_ep_read_data(usbd, ep_index);
    return n;
}

uint32_t usbd_ep_write_buffer(USBD_TypeDef *usbd, uint8_t ep_index, const uint8_t *buf, size_t len)
{
    while (len--)
        usbd_ep_write_data(usbd, ep_index, *buf++);
    return 0;
}
