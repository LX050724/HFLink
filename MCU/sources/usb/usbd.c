#include "DAP_config.h"
#include "GOWIN_M1.h"
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

char *string_descriptors[] = {
    (char[]){ 0x09, 0x04 },             /* Langid */
    "undefined",                        /* Manufacturer */
    "HFLink CMSIS-DAP",              /* Product */
    "00000000000000000123456789ABCDEF", /* Serial Number */
    "HFLink WebUSB",
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

// clang-format on

static inline uint16_t usbd_calc_descaddr(volatile uint8_t *addr)
{
    return (uintptr_t)addr - USBD_BASE;
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

void usbd_init_desc()
{
    volatile uint8_t *w_ptr = &USBD->DESC_DATA[0];
    USBD->DESC_DEV_LEN = sizeof(device_descriptor);
    USBD->DESC_DEV_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, device_descriptor, sizeof(device_descriptor));
    w_ptr += sizeof(device_descriptor);
    *w_ptr++ = 0;
    *w_ptr++ = 0;

    USBD->DESC_QUAL_LEN = sizeof(device_quality_descriptor);
    USBD->DESC_QUAL_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, device_quality_descriptor, sizeof(device_quality_descriptor));
    w_ptr += sizeof(device_quality_descriptor);
    *w_ptr++ = 0;
    *w_ptr++ = 0;

    USBD->DESC_FSCFG_LEN = sizeof(config_descriptor_fs);
    USBD->DESC_FSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, config_descriptor_fs, sizeof(config_descriptor_fs));
    w_ptr += sizeof(config_descriptor_fs);

    USBD->DESC_HSCFG_LEN = sizeof(config_descriptor_hs);
    USBD->DESC_HSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    __memcpy8((uint8_t *)w_ptr, config_descriptor_hs, sizeof(config_descriptor_hs));
    w_ptr += sizeof(config_descriptor_hs);

    USBD->DESC_OSCFG_ADDR = usbd_calc_descaddr(w_ptr);
    *w_ptr++ = 0x07;

    USBD->DESC_STRLANG_ADDR = usbd_calc_descaddr(w_ptr);
    *w_ptr++ = 0x04;
    *w_ptr++ = 0x03;
    *w_ptr++ = 0x09;
    *w_ptr++ = 0x04;

    USBD->DESC_HIDRPT_LEN = 0;
    USBD->DESC_HIDRPT_ADDR = 0;

    USBD->DESC_BOS_ADDR = 0;
    USBD->DESC_BOS_LEN = 0;

    // USBD->DESC_BOS_ADDR = usbd_calc_descaddr(w_ptr);
    // USBD->DESC_BOS_LEN = sizeof(USBD_BinaryObjectStoreDescriptor);
    // __memcpy8((uint8_t *)w_ptr, USBD_BinaryObjectStoreDescriptor, sizeof(USBD_BinaryObjectStoreDescriptor));
    // w_ptr += sizeof(USBD_BinaryObjectStoreDescriptor);

    USBD->DESC_STRVENDOR_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC_STRVENDOR_LEN = desc_make_str(&w_ptr, string_descriptors[1]);
    USBD->DESC_STRPORDUCT_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC_STRPORDUCT_LEN = desc_make_str(&w_ptr, string_descriptors[2]);
    USBD->DESC_STRERTIAL_ADDR = usbd_calc_descaddr(w_ptr);
    USBD->DESC_STRERTIAL_LEN = desc_make_str(&w_ptr, string_descriptors[3]);

    USBD->DESC_HASSTR = 1;
}

void usbd_enable()
{
    USBD->CR |= USBD_CR_EN;
}
