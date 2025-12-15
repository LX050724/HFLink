#pragma once

#include "GOWIN_M1.h"
#ifdef __cplusplus
extern "C" {
#endif

#define DAP_IN_EP  0x83
#define DAP_OUT_EP 0x04

#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x80

#define USBD_VID           0x0D28
#define USBD_PID           0x0204
#define USBD_MAX_POWER     300
#define USBD_LANGID_STRING 1033

void usbd_init_desc(void);
void usbd_ep0_rx_irq_handler(USBD_TypeDef *usbd);

#ifdef __cplusplus
}
#endif
