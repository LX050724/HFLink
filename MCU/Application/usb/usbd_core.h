#pragma once

#include "GOWIN_M1.h"
#ifdef __cplusplus
extern "C" {
#endif

void usbd_set_serial_number(const uint8_t *data);
const char *usbd_get_serial_number_str(void);
void usbd_init_desc(void);
void usbd_irq_handler(USBD_TypeDef *usbd);

#ifdef __cplusplus
}
#endif
