#include "DAP_config.h"
#include "GOWIN_M1.h"
#include "GOWIN_M1_usbd.h"
#include "SEGGER_RTT.h"
#include "usb_cdc.h"
#include "usb_def.h"
#include "usb_util.h"
#include "usbd.h"
#include <string.h>

#define CMSIS_DAP_INTERFACE_SIZE (9 + 7 + 7)
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


char serial_number_dynamic[36] = "00000000000000000123456789ABCDEF"; // Dynamic serial number

typedef struct {
    const char *str;
    uint8_t len;
} USBD_desc_str_t;

USBD_desc_str_t string_descriptors[] = {
    {"\x09\x04", 2},
    {"undefined", 9},
    {"HFLink CMSIS-DAP", 16},
    {"00000000000000000123456789ABCDEF", 33}, 
    // {"HFLink WebUSB", 13},
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_1, 0x00, 0x00, 0x00, 0x01),
};

static const uint8_t device_descriptor[18] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x00),
    USB_INTERFACE_DESCRIPTOR_INIT(2, 0x00, 0x02, 0xff, 0xff, 0x00, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x00),
    USB_INTERFACE_DESCRIPTOR_INIT(2, 0x00, 0x02, 0xff, 0xff, 0x00, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
};

static const uint8_t other_speed_config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x00),
    USB_INTERFACE_DESCRIPTOR_INIT(2, 0x00, 0x02, 0xff, 0xff, 0x00, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
};

// clang-format on

static inline uint16_t usbd_calc_descaddr(volatile uint8_t *addr)
{
    return (uintptr_t)addr - USBD_BASE;
}

uint8_t xxx[0x12];

void usbd_get_descriptor(USBD_TypeDef *usbd, uint16_t value)
{
    uint8_t type = HI_BYTE(value);
    uint8_t index = LO_BYTE(value);
    xxx[type]++;
    switch (type)
    {
    case USB_DESCRIPTOR_TYPE_DEVICE:
        usbd_ep_write_buffer(usbd, 0, device_descriptor, sizeof(device_descriptor));
        break;
    case USB_DESCRIPTOR_TYPE_CONFIGURATION:
        if (usbd_get_speed(usbd))
            usbd_ep_write_buffer(usbd, 0, config_descriptor_hs, sizeof(config_descriptor_hs));
        else
            usbd_ep_write_buffer(usbd, 0, config_descriptor_fs, sizeof(config_descriptor_fs));
        break;
    case USB_DESCRIPTOR_TYPE_STRING:
        if (index == USB_STRING_LANGID_INDEX)
        {
            usbd_ep_write_data(usbd, 0, 4);
            usbd_ep_write_data(usbd, 0, USB_DESCRIPTOR_TYPE_STRING);
            usbd_ep_write_data(usbd, 0, string_descriptors[0].str[0]);
            usbd_ep_write_data(usbd, 0, string_descriptors[0].str[1]);
        }
        else if (index < sizeof(string_descriptors) / sizeof(string_descriptors[0]))
        {
            uint8_t len = string_descriptors[index].len;
            const char *str_ptr = string_descriptors[index].str;
            usbd_ep_write_data(usbd, 0, len * 2 + 2);
            usbd_ep_write_data(usbd, 0, USB_DESCRIPTOR_TYPE_STRING);
            for (uint8_t i = 0; i < len; i++)
            {
                usbd_ep_write_data(usbd, 0, str_ptr[i]);
                usbd_ep_write_data(usbd, 0, 0);
            }
        }
        break;
    case USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
        usbd_ep_write_buffer(usbd, 0, device_quality_descriptor, sizeof(device_quality_descriptor));
        break;
    case USB_DESCRIPTOR_TYPE_OTHER_SPEED:
        usbd_ep_write_buffer(usbd, 0, other_speed_config_descriptor, sizeof(other_speed_config_descriptor));
        break;
    case USB_DESCRIPTOR_TYPE_BINARY_OBJECT_STORE:
        usbd_ep_write_buffer(usbd, 0, USBD_BinaryObjectStoreDescriptor, sizeof(USBD_BinaryObjectStoreDescriptor));
        break;
    default:
        break;
    }

    usbd_ep_read_data(usbd, 0);
    usbd_ep_read_data(usbd, 0);
    usbd_ep_read_data(usbd, 0);
    usbd_ep_read_data(usbd, 0);
}

void usbd_std_device_req_handler(USBD_TypeDef *usbd)
{
    uint8_t requset = usbd_ep_read_data(usbd, 0);
    uint16_t value = usbd_ep_read_data(usbd, 0);
    value |= (uint16_t)usbd_ep_read_data(usbd, 0) << 8;

    switch (requset)
    {
    case USB_REQUEST_GET_DESCRIPTOR:
        usbd_get_descriptor(usbd, value);
        break;
    }
}

void usbd_standard_request_handler(USBD_TypeDef *usbd, uint8_t requset_type)
{
    switch (requset_type & USB_REQUEST_RECIPIENT_DEVICE)
    {
    case USB_REQUEST_RECIPIENT_DEVICE:
        usbd_std_device_req_handler(usbd);
        break;
    }
}

void usbd_class_request_handler(USBD_TypeDef *usbd, uint8_t requset_type)
{
}

void usbd_vendor_request_handler(USBD_TypeDef *usbd, uint8_t requset_type)
{
}

void usbd_ep0_rx_irq_handler(USBD_TypeDef *usbd)
{
    uint8_t requset_type = usbd_ep_read_data(usbd, 0);
    switch (requset_type & USB_REQUEST_TYPE_MASK)
    {
    case USB_REQUEST_STANDARD:
        usbd_standard_request_handler(usbd, requset_type);
        break;
    case USB_REQUEST_CLASS:
        usbd_class_request_handler(usbd, requset_type);
        break;
    case USB_REQUEST_VENDOR:
        usbd_vendor_request_handler(usbd, requset_type);
        break;
    default:
        break;
    }
}
