#include "DAP_config.h"
#include "GOWIN_M1.h"
#include "GOWIN_M1_axisuart.h"
#include "GOWIN_M1_usbd.h"
#include "SEGGER_RTT.h"
#include "usb_cdc.h"
#include "usb_def.h"
#include "usb_util.h"
#include "usbd_core.h"
#include <string.h>

#define CMSIS_DAP_INTERFACE_SIZE (9 + 7 + 7 + 7)
#define CUSTOM_HID_LEN (9 + 9 + 7 + 7)

#define HIDRAW_INTERVAL 4

#define HID_CUSTOM_REPORT_DESC_SIZE 53

#define USBD_WINUSB_VENDOR_CODE 0x20
#define USBD_WEBUSB_VENDOR_CODE 0x21

#define USBD_WEBUSB_ENABLE 0
#define USBD_BULK_ENABLE 1
#define USBD_WINUSB_ENABLE 1

/* WinUSB Microsoft OS 2.0 descriptor sizes */
#define WINUSB_DESCRIPTOR_SET_HEADER_SIZE 10
#define WINUSB_FUNCTION_SUBSET_HEADER_SIZE 8
#define WINUSB_FEATURE_COMPATIBLE_ID_SIZE 20

#define FUNCTION_SUBSET_LEN 160
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132

#define USBD_WINUSB_DESC_SET_LEN                                                                                       \
    (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + USBD_WEBUSB_ENABLE * FUNCTION_SUBSET_LEN +                                    \
     USBD_BULK_ENABLE * FUNCTION_SUBSET_LEN)

#define USBD_NUM_DEV_CAPABILITIES (USBD_WEBUSB_ENABLE + USBD_WINUSB_ENABLE)

#define USBD_WEBUSB_DESC_LEN 24
#define USBD_WINUSB_DESC_LEN 28

#define USBD_BOS_WTOTALLENGTH                                                                                          \
    (0x05 + USBD_WEBUSB_DESC_LEN * USBD_WEBUSB_ENABLE + USBD_WINUSB_DESC_LEN * USBD_WINUSB_ENABLE)

#define USB_CONFIG_SIZE (9 + CMSIS_DAP_INTERFACE_SIZE + CDC_ACM_DESCRIPTOR_LEN + USBD_WEBUSB_ENABLE * 9)

#define INTF_NUM (1 + 2 + USBD_WEBUSB_ENABLE)

#define MSC_INTF_NUM (3)

#define WEBUSB_INTF_NUM (3)

// clang-format off
const uint8_t USBD_WinUSBDescriptorSetDescriptor[] = {
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE), /* wDescriptorType */
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */  /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),          /* wDescriptorSetTotalLength */
#if (USBD_WEBUSB_ENABLE)
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), // wLength
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), // wDescriptorType
    WEBUSB_INTF_NUM,                           // bFirstInterface USBD_WINUSB_IF_NUM
    0,                                         // bReserved
    WBVAL(FUNCTION_SUBSET_LEN),                // wSubsetLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  // wLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  // wDescriptorType
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        // CompatibleId
    0, 0, 0, 0, 0, 0, 0, 0,                    // SubCompatibleId
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), // wLength
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   // wDescriptorType
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), // wPropertyDataType
    WBVAL(42),                                 // wPropertyNameLength
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), // wPropertyDataLength
    '{', 0,
    '9', 0, '2', 0, 'C', 0, 'E', 0, '6', 0, '4', 0, '6', 0, '2', 0, '-', 0,
    '9', 0, 'C', 0, '7', 0, '7', 0, '-', 0,
    '4', 0, '6', 0, 'F', 0, 'E', 0, '-', 0,
    '9', 0, '3', 0, '3', 0, 'B', 0, '-',
    0, '3', 0, '1', 0, 'C', 0, 'B', 0, '9', 0, 'C', 0, '5', 0, 'A', 0, 'A', 0, '3', 0, 'B', 0, '9', 0,
    '}', 0, 0, 0, 0, 0,
#endif
#if USBD_BULK_ENABLE
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), /* wDescriptorType */
    0,                                         /* bFirstInterface USBD_BULK_IF_NUM*/
    0,                                         /* bReserved */
    WBVAL(FUNCTION_SUBSET_LEN),                /* wSubsetLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  /* wLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  /* wDescriptorType */
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        /* CompatibleId*/
    0, 0, 0, 0, 0, 0, 0, 0,                    /* SubCompatibleId*/
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), /* wLength */
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   /* wDescriptorType */
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), /* wPropertyDataType */
    WBVAL(42),                                 /* wPropertyNameLength */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), /* wPropertyDataLength */
    '{', 0,
    'C', 0, 'D', 0, 'B', 0, '3', 0, 'B', 0, '5', 0, 'A', 0, 'D', 0, '-', 0,
    '2', 0, '9', 0, '3', 0, 'B', 0, '-', 0,
    '4', 0, '6', 0, '6', 0, '3', 0, '-', 0,
    'A', 0, 'A', 0, '3', 0, '6', 0, '-',
    0, '1', 0, 'A', 0, 'A', 0, 'E', 0, '4', 0, '6', 0, '4', 0, '6', 0, '3', 0, '7', 0, '7', 0, '6', 0,
    '}', 0, 0, 0, 0, 0
#endif
};

const uint8_t USBD_BinaryObjectStoreDescriptor[] = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    USBD_NUM_DEV_CAPABILITIES,    /* bNumDeviceCaps */
#if (USBD_WEBUSB_ENABLE)
    USBD_WEBUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0x38, 0xB6, 0x08, 0x34,         /* PlatformCapabilityUUID */
    0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,
    WBVAL(0x0100), /* 1.00 */ /* bcdVersion */
    USBD_WEBUSB_VENDOR_CODE,  /* bVendorCode */
    1,                        /* iLandingPage */
#endif
#if (USBD_WINUSB_ENABLE)
    USBD_WINUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,         /* PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D,
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */ /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),         /* wDescriptorSetTotalLength */
    USBD_WINUSB_VENDOR_CODE,                 /* bVendorCode */
    0,                                       /* bAltEnumCode */
#endif
};


char serial_number_dynamic[] = "11111111110000000123456789ABCDEF"; // Dynamic serial number

typedef struct {
    const char *str;
    uint8_t len;
} USBD_desc_str_t;

USBD_desc_str_t string_descriptors[] = {
    {"\x09\x04", 2},
    {"HFLink", 9},
    {"HFLink CMSIS-DAP", 16},
    {"00000000000000000123456789ABCDEF", 32},
    {"HFLink CMSIS-DAP", 16},
    {"HFLink UART", 11},
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_1, 0x00, 0x00, 0x00, 0x01),
};

static const uint8_t device_descriptor[18] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x04),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_SWO_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x05),
        
#if CONFIG_CHERRYDAP_USE_CUSTOM_HID
    HID_DESC(),
#endif

#if USBD_WEBUSB_ENABLE
    USB_INTERFACE_DESCRIPTOR_INIT(WEBUSB_INTF_NUM, 0x00, 0x00, 0xff, 0x00, 0x00, 0x04),
#endif
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x04),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_SWO_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x05),

#if CONFIG_CHERRYDAP_USE_CUSTOM_HID
    HID_DESC(),
#endif

#if USBD_WEBUSB_ENABLE
    USB_INTERFACE_DESCRIPTOR_INIT(WEBUSB_INTF_NUM, 0x00, 0x00, 0xff, 0x00, 0x00, 0x04),
#endif
};

// clang-format on

static inline uint16_t usbd_calc_descaddr(volatile uint8_t *addr)
{
    return (uintptr_t)addr - (uintptr_t)&USBD->DESC_DATA;
}

static void __memcpy8(volatile uint8_t *desc, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; i++)
        desc[i] = src[i];
}

static uint8_t desc_make_str(volatile uint8_t **desc, const char *str)
{
    uint8_t total_len = 2;
    volatile uint8_t *w_ptr = *desc;
    *w_ptr++ = 0;
    *w_ptr++ = USB_DESCRIPTOR_TYPE_STRING;
    while (*str != 0)
    {
        *w_ptr++ = *str++;
        *w_ptr++ = 0;
        total_len += 2;
    }
    (*desc)[0] = total_len;
    *desc += total_len;
    return total_len;
}

uint16_t desc_strserial_addr;
uint16_t desc_exstr_addr[3];

void usbd_init_desc()
{
    volatile uint8_t *w_ptr = &USBD->DESC_DATA[0];
    USBD->DESC.DEV_LEN = sizeof(device_descriptor);
    USBD->DESC.DEV_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, device_descriptor, sizeof(device_descriptor));
    w_ptr += sizeof(device_descriptor);
    *w_ptr++ = 0;
    *w_ptr++ = 0;

    USBD->DESC.QUAL_LEN = sizeof(device_quality_descriptor);
    USBD->DESC.QUAL_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, device_quality_descriptor, sizeof(device_quality_descriptor));
    w_ptr += sizeof(device_quality_descriptor);
    *w_ptr++ = 0;
    *w_ptr++ = 0;

    USBD->DESC.FSCFG_LEN = sizeof(config_descriptor_fs);
    USBD->DESC.FSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, config_descriptor_fs, sizeof(config_descriptor_fs));
    w_ptr += sizeof(config_descriptor_fs);

    USBD->DESC.HSCFG_LEN = sizeof(config_descriptor_hs);
    USBD->DESC.HSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, config_descriptor_hs, sizeof(config_descriptor_hs));
    w_ptr += sizeof(config_descriptor_hs);

    USBD->DESC.OSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    *w_ptr++ = 0x07;

    USBD->DESC.STRLANG_ADDR = usbd_calc_descaddr(w_ptr);
    *w_ptr++ = 0x04;
    *w_ptr++ = 0x03;
    *w_ptr++ = 0x09;
    *w_ptr++ = 0x04;

    USBD->DESC.HIDRPT_LEN = 0;
    USBD->DESC.HIDRPT_ADDR = 0;

    USBD->DESC.BOS_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC.BOS_LEN = sizeof(USBD_BinaryObjectStoreDescriptor);
    __memcpy8((uint8_t *)w_ptr, USBD_BinaryObjectStoreDescriptor, sizeof(USBD_BinaryObjectStoreDescriptor));
    w_ptr += sizeof(USBD_BinaryObjectStoreDescriptor);

    USBD->DESC.STRVENDOR_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC.STRVENDOR_LEN = desc_make_str(&w_ptr, string_descriptors[1].str);
    USBD->DESC.STRPRODUCT_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC.STRPRODUCT_LEN = desc_make_str(&w_ptr, string_descriptors[2].str);

    desc_exstr_addr[0] = usbd_calc_descaddr(w_ptr);
    USBD->DESC.STRSERIAL_ADDR = desc_exstr_addr[0];
    USBD->DESC.STRSERIAL_LEN = desc_make_str(&w_ptr, string_descriptors[3].str);
    desc_exstr_addr[1] = usbd_calc_descaddr(w_ptr);
    desc_make_str(&w_ptr, string_descriptors[4].str);
    desc_exstr_addr[2] = usbd_calc_descaddr(w_ptr);
    desc_make_str(&w_ptr, string_descriptors[5].str);

    USBD->DESC.HASSTR = 1;
}

struct cdc_line_coding line_coding;
void cdc_acm_class_interface_request_handler(USBD_TypeDef *usbd, const struct usb_setup_packet *setup)
{

    switch (setup->bRequest)
    {
    case CDC_REQUEST_SET_LINE_CODING: {
        if (usbd_ep_rxfifo_is_empty(usbd, 0))
            break;
        usbd_ep_read_buffer(usbd, 0, &line_coding, sizeof(line_coding));
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
        axisuart_set_baud(AXIS_UART, line_coding.dwDTERate);
        axisuart_set_party(AXIS_UART, line_coding.bParityType);
        axisuart_set_stop_bit(AXIS_UART, line_coding.bCharFormat);
        break;
    }
    case CDC_REQUEST_GET_LINE_CODING: {
        usbd_ep_write_buffer(usbd, 0, (uint8_t *)&line_coding, sizeof(line_coding));
        break;
    }
    case CDC_REQUEST_SET_CONTROL_LINE_STATE: {
        axisuart_set_dtr(AXIS_UART, setup->wValue & 0x0001);
        axisuart_set_rts(AXIS_UART, setup->wValue & 0x0002);
        break;
    }
    case CDC_REQUEST_SEND_BREAK:
        break;
    }
}

uint8_t xxx[0x12];

const uint8_t *ep0_data_buf = NULL;
uint32_t ep0_data_buf_residue = 0;

void usbd_write_ldata(USBD_TypeDef *usbd, const uint8_t *data, size_t len)
{
    ep0_data_buf = data;
    ep0_data_buf_residue = len;

    if (len > 64)
        len = 64;
    usbd_ep_write_buffer(usbd, 0, ep0_data_buf, len);
    ep0_data_buf += len;
    ep0_data_buf_residue -= len;
}

void usbd_get_descriptor(USBD_TypeDef *usbd, struct usb_setup_packet *setup)
{
    uint8_t type = HI_BYTE(setup->wValue);
    uint8_t index = LO_BYTE(setup->wValue);
    switch (type)
    {
    case USB_DESCRIPTOR_TYPE_STRING:
        if (index > 2 && index < sizeof(string_descriptors) / sizeof(string_descriptors[0]))
        {
            usbd->DESC.STRSERIAL_LEN = string_descriptors[index].len * 2 + 2;
            usbd->DESC.STRSERIAL_ADDR = desc_exstr_addr[index - 3];
        }
        break;
    default:
        break;
    }
}

void usbd_std_device_req_handler(USBD_TypeDef *usbd, struct usb_setup_packet *setup)
{
    switch (setup->bRequest)
    {
    case USB_REQUEST_GET_DESCRIPTOR:
        usbd_get_descriptor(usbd, setup);
        break;
    }
}

void usbd_std_interface_req_handler(USBD_TypeDef *usbd, struct usb_setup_packet *setup)
{
    uint8_t type = HI_BYTE(setup->wValue);
    uint8_t intf_num = LO_BYTE(setup->wIndex);

    switch (setup->bRequest)
    {

        // case USB_REQUEST_GET_STATUS:
        //     break;
    }
}

void usbd_standard_request_handler(USBD_TypeDef *usbd, const struct usb_setup_packet *setup)
{
    switch (setup->bmRequestType & USB_REQUEST_RECIPIENT_MASK)
    {
    case USB_REQUEST_RECIPIENT_DEVICE:
        usbd_std_device_req_handler(usbd, setup);
        break;
    case USB_REQUEST_RECIPIENT_INTERFACE:
        usbd_std_interface_req_handler(usbd, setup);
        break;
    case USB_REQUEST_RECIPIENT_ENDPOINT:
        break;
    }
}

void usbd_class_request_handler(USBD_TypeDef *usbd, const struct usb_setup_packet *setup)
{

    if ((setup->bmRequestType & USB_REQUEST_RECIPIENT_MASK) == USB_REQUEST_RECIPIENT_INTERFACE)
    {
        uint8_t intf_num = setup->wIndex;
        if (intf_num == 1)
        {
            // 串口interface
            cdc_acm_class_interface_request_handler(usbd, setup);
        }
    }
    else if ((setup->bmRequestType & USB_REQUEST_RECIPIENT_MASK) == USB_REQUEST_RECIPIENT_ENDPOINT)
    {
    }
}

void usbd_vendor_request_handler(USBD_TypeDef *usbd, const struct usb_setup_packet *setup)
{
    if (setup->bRequest == USBD_WINUSB_VENDOR_CODE && setup->wIndex == WINUSB_REQUEST_GET_DESCRIPTOR_SET)
    {
        usbd_write_ldata(usbd, USBD_WinUSBDescriptorSetDescriptor, sizeof(USBD_WinUSBDescriptorSetDescriptor));
    }
    // else if (requset == USBD_WEBUSB_VENDOR_CODE && index == WEBUSB_REQUEST_GET_URL) {
    //     usbd_ep_write_buffer(usbd, 0, USBD_WinUSBDescriptorSetDescriptor, 64);
    // }
}

void usbd_setup_request_handler(USBD_TypeDef *usbd, uint32_t it_flag, const struct usb_setup_packet *setup)
{

    switch (setup->bmRequestType & USB_REQUEST_TYPE_MASK)
    {
    case USB_REQUEST_STANDARD:
        usbd_standard_request_handler(usbd, setup);
        break;
    case USB_REQUEST_CLASS:
        usbd_class_request_handler(usbd, setup);
        break;
    case USB_REQUEST_VENDOR:
        usbd_vendor_request_handler(usbd, setup);
        break;
    default:
        break;
    }
    // SEGGER_RTT_printf(0, "rt:%x r:%x v:%04x i:%04x l:%d s:%d:%d\n", setup->bmRequestType, setup->bRequest,
    //                   setup->wValue, setup->wIndex, setup->wLength, usbd_ep_get_txnum(usbd, 0),
    //                   usbd_ep_get_rxnum(usbd, 0));
}

struct usb_setup_packet setup;

void usbd_ep0_rx_irq_handler(USBD_TypeDef *usbd, uint32_t it_flag)
{
    if (it_flag & USBD_SR_IT_SETUP)
    {
        ((uint8_t *)&setup)[0] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[1] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[2] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[3] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[4] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[5] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[6] = usbd_ep_read_data(usbd, 0);
        ((uint8_t *)&setup)[7] = usbd_ep_read_data(usbd, 0);

        usbd_setup_request_handler(usbd, it_flag, &setup);
    }

    if (it_flag & USBD_SR_IT_EPIN)
    {
        if (ep0_data_buf_residue)
        {
            size_t len = ep0_data_buf_residue > 64 ? 64 : ep0_data_buf_residue;
            usbd_ep_write_buffer(usbd, 0, ep0_data_buf, len);
            ep0_data_buf += len;
            ep0_data_buf_residue -= len;
        }
    }

    if (it_flag & USBD_SR_IT_EPOUT)
    {
        uint8_t active_ep = (it_flag >> 3) & 0x0f;
        if (active_ep == 0 && !usbd_ep_rxfifo_is_empty(usbd, 0))
        {
            usbd_setup_request_handler(usbd, it_flag, &setup);
        }
    }
}
