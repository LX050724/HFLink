#include "GOWIN_M1.h"
#include "GOWIN_M1_axisuart.h"
#include "GOWIN_M1_usbd.h"
#include "usb_cdc.h"
#include "usb_def.h"
#include "usb_util.h"
#include "usbd_core.h"
#include <string.h>
#ifdef DEBUG
#include "SEGGER_RTT.h"
#endif

static char serial_number_dynamic[33]; // Dynamic serial number

static uint8_t desc_make_str(volatile uint8_t *desc, const char *str)
{
    uint8_t total_len = 2;
    volatile uint8_t *w_ptr = desc;
    *w_ptr++ = 0;
    *w_ptr++ = USB_DESCRIPTOR_TYPE_STRING;
    while (*str != 0)
    {
        *w_ptr++ = *str++;
        *w_ptr++ = 0;
        total_len += 2;
    }
    desc[0] = total_len;
    return total_len;
}

void usbd_set_serial_number(const uint8_t *data)
{
    const char *hex_str = "0123456789abcdef";
    for (int i = 0; i < 16; i++)
    {
        serial_number_dynamic[i * 2] = hex_str[data[i] >> 4];
        serial_number_dynamic[i * 2 + 1] = hex_str[(data[i] & 0x0f)];
    }
    serial_number_dynamic[32] = 0;
}

const char *usbd_get_serial_number_str(void)
{
    return serial_number_dynamic;
}

void usbd_init_desc(void)
{
    struct cdc_line_coding *line_coding = (struct cdc_line_coding *)&USBD->LINECODE;
    line_coding->dwDTERate = 115200;
    desc_make_str(USBD->SERIAL_DESC, serial_number_dynamic);
}

struct cdc_line_coding line_coding;
void cdc_acm_class_interface_request_handler(USBD_TypeDef *usbd, const struct usb_setup_packet *setup)
{
    (void)usbd;
    struct cdc_line_coding *line_coding = (struct cdc_line_coding *)&USBD->LINECODE;
    switch (setup->bRequest)
    {
    case CDC_REQUEST_SET_LINE_CODING: {
        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
        axisuart_set_baud(AXIS_UART, line_coding->dwDTERate);
        axisuart_set_party(AXIS_UART, line_coding->bParityType);
        axisuart_set_stop_bit(AXIS_UART, line_coding->bCharFormat);
        break;
    }
    case CDC_REQUEST_SET_CONTROL_LINE_STATE: {
        axisuart_set_dtr(AXIS_UART, setup->wValue & 0x0001);
        axisuart_set_rts(AXIS_UART, setup->wValue & 0x0002);
        break;
    }
    case CDC_REQUEST_SEND_BREAK:
        if (setup->wValue)
        {
            axisuart_send_break(AXIS_UART);
        }
        break;
    }
}

static struct usb_setup_packet setup;

void usbd_irq_handler(USBD_TypeDef *usbd)
{
    memcpy(&setup, (void *)usbd->SETUP, sizeof(setup));
    cdc_acm_class_interface_request_handler(usbd, &setup);
}
