from io import BytesIO, StringIO
from typing import List
from string import Template

# Useful define
USB_1_1 = 0x0110
USB_2_0 = 0x0200
# Set USB version to 2.1 so that the host will request the BOS descriptor
USB_2_1 = 0x0210
USB_3_0 = 0x0300
USB_3_1 = 0x0310
USB_3_2 = 0x0320

# Device speeds
USB_SPEED_UNKNOWN    =0x00 # Transfer rate not yet set
USB_SPEED_LOW        =0x01 # USB 1.1
USB_SPEED_FULL       =0x02 # USB 1.1
USB_SPEED_HIGH       =0x03 # USB 2.0
USB_SPEED_WIRELESS   =0x04 # Wireless USB 2.5
USB_SPEED_SUPER      =0x05 # USB 3.0
USB_SPEED_SUPER_PLUS =0x06 # USB 3.1

# Maximum number of devices per controller
USB_MAX_DEVICES = 127

# Default USB control EP, always 0 and 0x80
USB_CONTROL_OUT_EP0 = 0
USB_CONTROL_IN_EP0  = 0x80

#*< maximum packet size (MPS) for EP 0
USB_CTRL_EP_MPS = 64

#*< maximum packet size (MPS) for bulk EP
USB_BULK_EP_MPS_HS = 512
USB_BULK_EP_MPS_FS = 64

# USB PID Types
USB_PID_OUT   = 0x01 # Tokens
USB_PID_IN    = 0x09
USB_PID_SOF   = 0x05
USB_PID_SETUP = 0x0d

USB_PID_DATA0 = 0x03 # Data
USB_PID_DATA1 = 0x0b
USB_PID_DATA2 = 0x07
USB_PID_MDATA = 0x0f

USB_PID_ACK   = 0x02 # Handshake
USB_PID_NAK   = 0x0a
USB_PID_STALL = 0x0e
USB_PID_NYET  = 0x06

USB_PID_PRE      =  0x0c # Special
USB_PID_ERR      =  0x0c
USB_PID_SPLIT    =  0x08
USB_PID_PING     =  0x04
USB_PID_RESERVED =  0x00

USB_REQUEST_DIR_SHIFT =7                            # Bits 7: Request dir
USB_REQUEST_DIR_OUT   =(0 << USB_REQUEST_DIR_SHIFT) # Bit 7=0: Host-to-device
USB_REQUEST_DIR_IN    =(1 << USB_REQUEST_DIR_SHIFT) # Bit 7=1: Device-to-host
USB_REQUEST_DIR_MASK  =(1 << USB_REQUEST_DIR_SHIFT) # Bit 7=1: Direction bit

USB_REQUEST_TYPE_SHIFT =5 # Bits 5:6: Request type
USB_REQUEST_STANDARD   =(0 << USB_REQUEST_TYPE_SHIFT)
USB_REQUEST_CLASS      =(1 << USB_REQUEST_TYPE_SHIFT)
USB_REQUEST_VENDOR     =(2 << USB_REQUEST_TYPE_SHIFT)
USB_REQUEST_RESERVED   =(3 << USB_REQUEST_TYPE_SHIFT)
USB_REQUEST_TYPE_MASK  =(3 << USB_REQUEST_TYPE_SHIFT)

USB_REQUEST_RECIPIENT_SHIFT     =0 # Bits 0:4: Recipient
USB_REQUEST_RECIPIENT_DEVICE    =(0 << USB_REQUEST_RECIPIENT_SHIFT)
USB_REQUEST_RECIPIENT_INTERFACE =(1 << USB_REQUEST_RECIPIENT_SHIFT)
USB_REQUEST_RECIPIENT_ENDPOINT  =(2 << USB_REQUEST_RECIPIENT_SHIFT)
USB_REQUEST_RECIPIENT_OTHER     =(3 << USB_REQUEST_RECIPIENT_SHIFT)
USB_REQUEST_RECIPIENT_MASK      =(3 << USB_REQUEST_RECIPIENT_SHIFT)

# USB Standard Request Codes
USB_REQUEST_GET_STATUS          = 0x00
USB_REQUEST_CLEAR_FEATURE       = 0x01
USB_REQUEST_SET_FEATURE         = 0x03
USB_REQUEST_SET_ADDRESS         = 0x05
USB_REQUEST_GET_DESCRIPTOR      = 0x06
USB_REQUEST_SET_DESCRIPTOR      = 0x07
USB_REQUEST_GET_CONFIGURATION   = 0x08
USB_REQUEST_SET_CONFIGURATION   = 0x09
USB_REQUEST_GET_INTERFACE       = 0x0A
USB_REQUEST_SET_INTERFACE       = 0x0B
USB_REQUEST_SYNCH_FRAME         = 0x0C
USB_REQUEST_SET_ENCRYPTION      = 0x0D
USB_REQUEST_GET_ENCRYPTION      = 0x0E
USB_REQUEST_RPIPE_ABORT         = 0x0E
USB_REQUEST_SET_HANDSHAKE       = 0x0F
USB_REQUEST_RPIPE_RESET         = 0x0F
USB_REQUEST_GET_HANDSHAKE       = 0x10
USB_REQUEST_SET_CONNECTION      = 0x11
USB_REQUEST_SET_SECURITY_DATA   = 0x12
USB_REQUEST_GET_SECURITY_DATA   = 0x13
USB_REQUEST_SET_WUSB_DATA       = 0x14
USB_REQUEST_LOOPBACK_DATA_WRITE = 0x15
USB_REQUEST_LOOPBACK_DATA_READ  = 0x16
USB_REQUEST_SET_INTERFACE_DS    = 0x17

# USB Standard Feature selectors
USB_FEATURE_ENDPOINT_HALT  = 0x00
USB_FEATURE_SELF_POWERED   = 0x00
USB_FEATURE_REMOTE_WAKEUP  = 0x01
USB_FEATURE_TEST_MODE      = 0x02
USB_FEATURE_BATTERY        = 0x02
USB_FEATURE_BHNPENABLE     = 0x03
USB_FEATURE_WUSBDEVICE     = 0x03
USB_FEATURE_AHNPSUPPORT    = 0x04
USB_FEATURE_AALTHNPSUPPORT = 0x05
USB_FEATURE_DEBUGMODE      = 0x06

# USB GET_STATUS Bit Values
USB_GETSTATUS_ENDPOINT_HALT = 0x01
USB_GETSTATUS_SELF_POWERED  = 0x01
USB_GETSTATUS_REMOTE_WAKEUP = 0x02

# USB Descriptor Types
USB_DESCRIPTOR_TYPE_DEVICE                = 0x01
USB_DESCRIPTOR_TYPE_CONFIGURATION         = 0x02
USB_DESCRIPTOR_TYPE_STRING                = 0x03
USB_DESCRIPTOR_TYPE_INTERFACE             = 0x04
USB_DESCRIPTOR_TYPE_ENDPOINT              = 0x05
USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER      = 0x06
USB_DESCRIPTOR_TYPE_OTHER_SPEED           = 0x07
USB_DESCRIPTOR_TYPE_INTERFACE_POWER       = 0x08
USB_DESCRIPTOR_TYPE_OTG                   = 0x09
USB_DESCRIPTOR_TYPE_DEBUG                 = 0x0A
USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION = 0x0B
USB_DESCRIPTOR_TYPE_BINARY_OBJECT_STORE   = 0x0F
USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY     = 0x10
USB_DESCRIPTOR_TYPE_WIRELESS_ENDPOINTCOMP = 0x11

# Class Specific Descriptor
USB_CS_DESCRIPTOR_TYPE_DEVICE        = 0x21
USB_CS_DESCRIPTOR_TYPE_CONFIGURATION = 0x22
USB_CS_DESCRIPTOR_TYPE_STRING        = 0x23
USB_CS_DESCRIPTOR_TYPE_INTERFACE     = 0x24
USB_CS_DESCRIPTOR_TYPE_ENDPOINT      = 0x25

USB_DESCRIPTOR_TYPE_SUPERSPEED_ENDPOINT_COMPANION     = 0x30
USB_DESCRIPTOR_TYPE_SUPERSPEED_ISO_ENDPOINT_COMPANION = 0x31

# USB Device Classes
USB_DEVICE_CLASS_RESERVED      = 0x00
USB_DEVICE_CLASS_AUDIO         = 0x01
USB_DEVICE_CLASS_CDC           = 0x02
USB_DEVICE_CLASS_HID           = 0x03
USB_DEVICE_CLASS_MONITOR       = 0x04
USB_DEVICE_CLASS_PHYSICAL      = 0x05
USB_DEVICE_CLASS_IMAGE         = 0x06
USB_DEVICE_CLASS_PRINTER       = 0x07
USB_DEVICE_CLASS_MASS_STORAGE  = 0x08
USB_DEVICE_CLASS_HUB           = 0x09
USB_DEVICE_CLASS_CDC_DATA      = 0x0a
USB_DEVICE_CLASS_SMART_CARD    = 0x0b
USB_DEVICE_CLASS_SECURITY      = 0x0d
USB_DEVICE_CLASS_VIDEO         = 0x0e
USB_DEVICE_CLASS_HEALTHCARE    = 0x0f
USB_DEVICE_CLASS_DIAG_DEVICE   = 0xdc
USB_DEVICE_CLASS_WIRELESS      = 0xe0
USB_DEVICE_CLASS_MISC          = 0xef
USB_DEVICE_CLASS_APP_SPECIFIC  = 0xfe
USB_DEVICE_CLASS_VEND_SPECIFIC = 0xff

# usb string index define
USB_STRING_LANGID_INDEX    = 0x00
USB_STRING_MFC_INDEX       = 0x01
USB_STRING_PRODUCT_INDEX   = 0x02
USB_STRING_SERIAL_INDEX    = 0x03
USB_STRING_CONFIG_INDEX    = 0x04
USB_STRING_INTERFACE_INDEX = 0x05
USB_STRING_OS_INDEX        = 0x06
USB_STRING_MAX             = USB_STRING_OS_INDEX

# Devices supporting Microsoft OS Descriptors store special string
# descriptor at fixed index (0xEE). It is read when a new device is
# attached to a computer for the first time.
USB_OSDESC_STRING_DESC_INDEX = 0xEE

# bmAttributes in Configuration Descriptor
USB_CONFIG_REMOTE_WAKEUP = 0x20
USB_CONFIG_POWERED_MASK  = 0x40
USB_CONFIG_BUS_POWERED   = 0x80
USB_CONFIG_SELF_POWERED  = 0xC0

# bMaxPower in Configuration Descriptor
def USB_CONFIG_POWER_MA(mA):
    return ((mA) // 2)

# bEndpointAddress in Endpoint Descriptor
USB_ENDPOINT_DIRECTION_MASK = 0x80
def USB_ENDPOINT_OUT(addr):
    return ((addr) | 0x00)
def USB_ENDPOINT_IN(addr):
    return ((addr) | 0x80)

# USB endpoint direction and number.
USB_EP_DIR_MASK =0x80
USB_EP_DIR_IN   =0x80
USB_EP_DIR_OUT  =0x00

#* Get endpoint index (number) from endpoint address
def USB_EP_GET_IDX(ep):
    return ((ep) & ~USB_EP_DIR_MASK)
#* Get direction from endpoint address
def USB_EP_GET_DIR(ep):
    return ((ep)&USB_EP_DIR_MASK)
#* Get endpoint address from endpoint index and direction
def USB_EP_GET_ADDR(idx, dir):
    return ((idx) | ((dir)&USB_EP_DIR_MASK))
#* True if the endpoint is an IN endpoint
def USB_EP_DIR_IS_IN(ep):
    return (USB_EP_GET_DIR(ep) == USB_EP_DIR_IN)
#* True if the endpoint is an OUT endpoint
def USB_EP_DIR_IS_OUT(ep):
    return (USB_EP_GET_DIR(ep) == USB_EP_DIR_OUT)

# bmAttributes in Endpoint Descriptor
USB_ENDPOINT_TYPE_SHIFT       =0
USB_ENDPOINT_TYPE_CONTROL     =(0 << USB_ENDPOINT_TYPE_SHIFT)
USB_ENDPOINT_TYPE_ISOCHRONOUS =(1 << USB_ENDPOINT_TYPE_SHIFT)
USB_ENDPOINT_TYPE_BULK        =(2 << USB_ENDPOINT_TYPE_SHIFT)
USB_ENDPOINT_TYPE_INTERRUPT   =(3 << USB_ENDPOINT_TYPE_SHIFT)
USB_ENDPOINT_TYPE_MASK        =(3 << USB_ENDPOINT_TYPE_SHIFT)
def USB_GET_ENDPOINT_TYPE(x): 
    return ((x & USB_ENDPOINT_TYPE_MASK) >> USB_ENDPOINT_TYPE_SHIFT)

USB_ENDPOINT_SYNC_SHIFT              =2
USB_ENDPOINT_SYNC_NO_SYNCHRONIZATION =(0 << USB_ENDPOINT_SYNC_SHIFT)
USB_ENDPOINT_SYNC_ASYNCHRONOUS       =(1 << USB_ENDPOINT_SYNC_SHIFT)
USB_ENDPOINT_SYNC_ADAPTIVE           =(2 << USB_ENDPOINT_SYNC_SHIFT)
USB_ENDPOINT_SYNC_SYNCHRONOUS        =(3 << USB_ENDPOINT_SYNC_SHIFT)
USB_ENDPOINT_SYNC_MASK               =(3 << USB_ENDPOINT_SYNC_SHIFT)

USB_ENDPOINT_USAGE_SHIFT             =4
USB_ENDPOINT_USAGE_DATA              =(0 << USB_ENDPOINT_USAGE_SHIFT)
USB_ENDPOINT_USAGE_FEEDBACK          =(1 << USB_ENDPOINT_USAGE_SHIFT)
USB_ENDPOINT_USAGE_IMPLICIT_FEEDBACK =(2 << USB_ENDPOINT_USAGE_SHIFT)
USB_ENDPOINT_USAGE_MASK              =(3 << USB_ENDPOINT_USAGE_SHIFT)

USB_ENDPOINT_MAX_ADJUSTABLE =(1 << 7)

# wMaxPacketSize in Endpoint Descriptor
USB_MAXPACKETSIZE_SHIFT                        =0
USB_MAXPACKETSIZE_MASK                         =(0x7ff << USB_MAXPACKETSIZE_SHIFT)
USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT =11
USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_NONE  =(0 << USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT)
USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_ONE   =(1 << USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT)
USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_TWO   =(2 << USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT)
USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_MASK  =(3 << USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT)
def USB_GET_MAXPACKETSIZE(x):
    return ((x & USB_MAXPACKETSIZE_MASK) >> USB_MAXPACKETSIZE_SHIFT)
def USB_GET_MULT(x):
    return ((x & USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_MASK) >> USB_MAXPACKETSIZE_ADDITIONAL_TRANSCATION_SHIFT)

# bDevCapabilityType in Device Capability Descriptor
USB_DEVICE_CAPABILITY_WIRELESS_USB                =1
USB_DEVICE_CAPABILITY_USB_2_0_EXTENSION           =2
USB_DEVICE_CAPABILITY_SUPERSPEED_USB              =3
USB_DEVICE_CAPABILITY_CONTAINER_ID                =4
USB_DEVICE_CAPABILITY_PLATFORM                    =5
USB_DEVICE_CAPABILITY_POWER_DELIVERY_CAPABILITY   =6
USB_DEVICE_CAPABILITY_BATTERY_INFO_CAPABILITY     =7
USB_DEVICE_CAPABILITY_PD_CONSUMER_PORT_CAPABILITY =8
USB_DEVICE_CAPABILITY_PD_PROVIDER_PORT_CAPABILITY =9
USB_DEVICE_CAPABILITY_SUPERSPEED_PLUS             =10
USB_DEVICE_CAPABILITY_PRECISION_TIME_MEASUREMENT  =11
USB_DEVICE_CAPABILITY_WIRELESS_USB_EXT            =12

USB_BOS_CAPABILITY_EXTENSION =0x02
USB_BOS_CAPABILITY_PLATFORM  =0x05

# OTG SET FEATURE Constants
USB_OTG_FEATURE_B_HNP_ENABLE      =3 # Enable B device to perform HNP
USB_OTG_FEATURE_A_HNP_SUPPORT     =4 # A device supports HNP
USB_OTG_FEATURE_A_ALT_HNP_SUPPORT =5 # Another port on the A device supports HNP

# WinUSB Microsoft OS 2.0 descriptor request codes
WINUSB_REQUEST_GET_DESCRIPTOR_SET =0x07
WINUSB_REQUEST_SET_ALT_ENUM       =0x08

# WinUSB Microsoft OS 2.0 descriptor sizes
WINUSB_DESCRIPTOR_SET_HEADER_SIZE  =10
WINUSB_FUNCTION_SUBSET_HEADER_SIZE =8
WINUSB_FEATURE_COMPATIBLE_ID_SIZE  =20

# WinUSB Microsoft OS 2.0 Descriptor Types
WINUSB_SET_HEADER_DESCRIPTOR_TYPE       =0x00
WINUSB_SUBSET_HEADER_CONFIGURATION_TYPE =0x01
WINUSB_SUBSET_HEADER_FUNCTION_TYPE      =0x02
WINUSB_FEATURE_COMPATIBLE_ID_TYPE       =0x03
WINUSB_FEATURE_REG_PROPERTY_TYPE        =0x04
WINUSB_FEATURE_MIN_RESUME_TIME_TYPE     =0x05
WINUSB_FEATURE_MODEL_ID_TYPE            =0x06
WINUSB_FEATURE_CCGP_DEVICE_TYPE         =0x07

WINUSB_PROP_DATA_TYPE_REG_SZ       =0x01
WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ =0x07

# WebUSB Descriptor Types
WEBUSB_DESCRIPTOR_SET_HEADER_TYPE       =0x00
WEBUSB_CONFIGURATION_SUBSET_HEADER_TYPE =0x01
WEBUSB_FUNCTION_SUBSET_HEADER_TYPE      =0x02
WEBUSB_URL_TYPE                         =0x03

# WebUSB Request Codes
WEBUSB_REQUEST_GET_URL =0x02

# bScheme in URL descriptor
WEBUSB_URL_SCHEME_HTTP  =0x00
WEBUSB_URL_SCHEME_HTTPS =0x01

# WebUSB Descriptor sizes
WEBUSB_DESCRIPTOR_SET_HEADER_SIZE        =5
WEBUSB_CONFIGURATION_SUBSET_HEADER_SIZE  =4
WEBUSB_FUNCTION_SUBSET_HEADER_SIZE       =3


#----------------------------------------------------------------------------
#      Definitions  based on usbcdc11.pdf (www.usb.org)
#----------------------------------------------------------------------------
# Communication device class specification version 1.10
CDC_V1_10 = 0x0110
# Communication device class specification version 1.2
CDC_V1_2_0 =0x0120

# Communication interface class code
# (usbcdc11.pdf, 4.2, Table 15)
CDC_COMMUNICATION_INTERFACE_CLASS =0x02

# Communication interface class subclass codes
# (usbcdc11.pdf, 4.3, Table 16)
CDC_SUBCLASS_NONE       =0x00 # Reserved
CDC_SUBCLASS_DLC        =0x01 # Direct Line Control Model
CDC_SUBCLASS_ACM        =0x02 # Abstract Control Model
CDC_SUBCLASS_TCM        =0x03 # Telephone Control Model
CDC_SUBCLASS_MCM        =0x04 # Multi-Channel Control Model
CDC_SUBCLASS_CAPI       =0x05 # CAPI Control Model
CDC_SUBCLASS_ECM        =0x06 # Ethernet Networking Control Model
CDC_SUBCLASS_ATM        =0x07 # ATM Networking Control Model
                                     # 0x08-0x0d Reserved (future use)
CDC_SUBCLASS_MBIM       =0x0e # MBIM Control Model
                                     # 0x0f-0x7f Reserved (future use)
                                     # 0x80-0xfe Reserved (vendor specific)

CDC_DIRECT_LINE_CONTROL_MODEL         =0x01
CDC_ABSTRACT_CONTROL_MODEL            =0x02
CDC_TELEPHONE_CONTROL_MODEL           =0x03
CDC_MULTI_CHANNEL_CONTROL_MODEL       =0x04
CDC_CAPI_CONTROL_MODEL                =0x05
CDC_ETHERNET_NETWORKING_CONTROL_MODEL =0x06
CDC_ATM_NETWORKING_CONTROL_MODEL      =0x07
CDC_WIRELESS_HANDSET_CONTROL_MODEL    =0x08
CDC_DEVICE_MANAGEMENT                 =0x09
CDC_MOBILE_DIRECT_LINE_MODEL          =0x0A
CDC_OBEX                              =0x0B
CDC_ETHERNET_EMULATION_MODEL          =0x0C
CDC_NETWORK_CONTROL_MODEL             =0x0D

# Communication interface class control protocol codes
# (usbcdc11.pdf, 4.4, Table 17)
CDC_COMMON_PROTOCOL_NONE                            =0x00
CDC_COMMON_PROTOCOL_AT_COMMANDS                     =0x01
CDC_COMMON_PROTOCOL_AT_COMMANDS_PCCA_101            =0x02
CDC_COMMON_PROTOCOL_AT_COMMANDS_PCCA_101_AND_ANNEXO =0x03
CDC_COMMON_PROTOCOL_AT_COMMANDS_GSM_707             =0x04
CDC_COMMON_PROTOCOL_AT_COMMANDS_3GPP_27007          =0x05
CDC_COMMON_PROTOCOL_AT_COMMANDS_CDMA                =0x06
CDC_COMMON_PROTOCOL_ETHERNET_EMULATION_MODEL        =0x07
# NCM Communication Interface Protocol Codes
# (usbncm10.pdf, 4.2, Table 4-2)
CDC_NCM_PROTOCOL_NONE =0x00
CDC_NCM_PROTOCOL_OEM  =0xFE

# Data interface class code
# (usbcdc11.pdf, 4.5, Table 18)
CDC_DATA_INTERFACE_CLASS =0x0A

# Data Interface Sub-Class Codes *******************************************
CDC_DATA_SUBCLASS_NONE  =0x00

# Data interface class protocol codes
# (usbcdc11.pdf, 4.7, Table 19)
CDC_DATA_PROTOCOL_ISDN_BRI            =0x30
CDC_DATA_PROTOCOL_HDLC                =0x31
CDC_DATA_PROTOCOL_TRANSPARENT         =0x32
CDC_DATA_PROTOCOL_Q921_MANAGEMENT     =0x50
CDC_DATA_PROTOCOL_Q921_DATA_LINK      =0x51
CDC_DATA_PROTOCOL_Q921_MULTIPLEXOR    =0x52
CDC_DATA_PROTOCOL_V42                 =0x90
CDC_DATA_PROTOCOL_EURO_ISDN           =0x91
CDC_DATA_PROTOCOL_V24_RATE_ADAPTATION =0x92
CDC_DATA_PROTOCOL_CAPI                =0x93
CDC_DATA_PROTOCOL_HOST_BASED_DRIVER   =0xFD
CDC_DATA_PROTOCOL_DESCRIBED_IN_PUFD   =0xFE

# Type values for bDescriptorType field of functional descriptors
# (usbcdc11.pdf, 5.2.3, Table 24)
CDC_CS_INTERFACE =0x24
CDC_CS_ENDPOINT  =0x25

# Type values for bDescriptorSubtype field of functional descriptors
# (usbcdc11.pdf, 5.2.3, Table 25)
CDC_FUNC_DESC_HEADER                          =0x00
CDC_FUNC_DESC_CALL_MANAGEMENT                 =0x01
CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT     =0x02
CDC_FUNC_DESC_DIRECT_LINE_MANAGEMENT          =0x03
CDC_FUNC_DESC_TELEPHONE_RINGER                =0x04
CDC_FUNC_DESC_REPORTING_CAPABILITIES          =0x05
CDC_FUNC_DESC_UNION                           =0x06
CDC_FUNC_DESC_COUNTRY_SELECTION               =0x07
CDC_FUNC_DESC_TELEPHONE_OPERATIONAL_MODES     =0x08
CDC_FUNC_DESC_USB_TERMINAL                    =0x09
CDC_FUNC_DESC_NETWORK_CHANNEL                 =0x0A
CDC_FUNC_DESC_PROTOCOL_UNIT                   =0x0B
CDC_FUNC_DESC_EXTENSION_UNIT                  =0x0C
CDC_FUNC_DESC_MULTI_CHANNEL_MANAGEMENT        =0x0D
CDC_FUNC_DESC_CAPI_CONTROL_MANAGEMENT         =0x0E
CDC_FUNC_DESC_ETHERNET_NETWORKING             =0x0F
CDC_FUNC_DESC_ATM_NETWORKING                  =0x10
CDC_FUNC_DESC_WIRELESS_HANDSET_CONTROL_MODEL  =0x11
CDC_FUNC_DESC_MOBILE_DIRECT_LINE_MODEL        =0x12
CDC_FUNC_DESC_MOBILE_DIRECT_LINE_MODEL_DETAIL =0x13
CDC_FUNC_DESC_DEVICE_MANAGEMENT_MODEL         =0x14
CDC_FUNC_DESC_OBEX                            =0x15
CDC_FUNC_DESC_COMMAND_SET                     =0x16
CDC_FUNC_DESC_COMMAND_SET_DETAIL              =0x17
CDC_FUNC_DESC_TELEPHONE_CONTROL_MODEL         =0x18
CDC_FUNC_DESC_OBEX_SERVICE_IDENTIFIER         =0x19
CDC_FUNC_DESC_NCM                             =0x1A

# CDC class-specific request codes
# (usbcdc11.pdf, 6.2, Table 46)
# see Table 45 for info about the specific requests.
CDC_REQUEST_SEND_ENCAPSULATED_COMMAND      =0x00
CDC_REQUEST_GET_ENCAPSULATED_RESPONSE      =0x01
CDC_REQUEST_SET_COMM_FEATURE               =0x02
CDC_REQUEST_GET_COMM_FEATURE               =0x03
CDC_REQUEST_CLEAR_COMM_FEATURE             =0x04
CDC_REQUEST_SET_AUX_LINE_STATE             =0x10
CDC_REQUEST_SET_HOOK_STATE                 =0x11
CDC_REQUEST_PULSE_SETUP                    =0x12
CDC_REQUEST_SEND_PULSE                     =0x13
CDC_REQUEST_SET_PULSE_TIME                 =0x14
CDC_REQUEST_RING_AUX_JACK                  =0x15
CDC_REQUEST_SET_LINE_CODING                =0x20
CDC_REQUEST_GET_LINE_CODING                =0x21
CDC_REQUEST_SET_CONTROL_LINE_STATE         =0x22
CDC_REQUEST_SEND_BREAK                     =0x23
CDC_REQUEST_SET_RINGER_PARMS               =0x30
CDC_REQUEST_GET_RINGER_PARMS               =0x31
CDC_REQUEST_SET_OPERATION_PARMS            =0x32
CDC_REQUEST_GET_OPERATION_PARMS            =0x33
CDC_REQUEST_SET_LINE_PARMS                 =0x34
CDC_REQUEST_GET_LINE_PARMS                 =0x35
CDC_REQUEST_DIAL_DIGITS                    =0x36
CDC_REQUEST_SET_UNIT_PARAMETER             =0x37
CDC_REQUEST_GET_UNIT_PARAMETER             =0x38
CDC_REQUEST_CLEAR_UNIT_PARAMETER           =0x39
CDC_REQUEST_GET_PROFILE                    =0x3A
CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS =0x40
CDC_REQUEST_SET_ETHERNET_PMP_FILTER        =0x41
CDC_REQUEST_GET_ETHERNET_PMP_FILTER        =0x42
CDC_REQUEST_SET_ETHERNET_PACKET_FILTER     =0x43
CDC_REQUEST_GET_ETHERNET_STATISTIC         =0x44
CDC_REQUEST_SET_ATM_DATA_FORMAT            =0x50
CDC_REQUEST_GET_ATM_DEVICE_STATISTICS      =0x51
CDC_REQUEST_SET_ATM_DEFAULT_VC             =0x52
CDC_REQUEST_GET_ATM_VC_STATISTICS          =0x53
CDC_REQUEST_GET_NTB_PARAMETERS             =0x80
CDC_REQUEST_GET_NET_ADDRESS                =0x81
CDC_REQUEST_SET_NET_ADDRESS                =0x82
CDC_REQUEST_GET_NTB_FORMAT                 =0x83
CDC_REQUEST_SET_NTB_FORMAT                 =0x84
CDC_REQUEST_GET_NTB_INPUT_SIZE             =0x85
CDC_REQUEST_SET_NTB_INPUT_SIZE             =0x86
CDC_REQUEST_GET_MAX_DATAGRAM_SIZE          =0x87
CDC_REQUEST_SET_MAX_DATAGRAM_SIZE          =0x88
CDC_REQUEST_GET_CRC_MODE                   =0x89
CDC_REQUEST_SET_CRC_MODE                   =0x90

# Communication feature selector codes
# (usbcdc11.pdf, 6.2.2..6.2.4, Table 47)
CDC_ABSTRACT_STATE  =0x01
CDC_COUNTRY_SETTING =0x02

#* Control Signal Bitmap Values for SetControlLineState
SET_CONTROL_LINE_STATE_RTS =0x02
SET_CONTROL_LINE_STATE_DTR =0x01

# Feature Status returned for ABSTRACT_STATE Selector
# (usbcdc11.pdf, 6.2.3, Table 48)
CDC_IDLE_SETTING          =(1 << 0)
CDC_DATA_MULTPLEXED_STATE =(1 << 1)

# Control signal bitmap values for the SetControlLineState request
# (usbcdc11.pdf, 6.2.14, Table 51)
CDC_DTE_PRESENT      =(1 << 0)
CDC_ACTIVATE_CARRIER =(1 << 1)

# CDC class-specific notification codes
# (usbcdc11.pdf, 6.3, Table 68)
# see Table 67 for Info about class-specific notifications
CDC_NOTIFICATION_NETWORK_CONNECTION =0x00
CDC_RESPONSE_AVAILABLE              =0x01
CDC_AUX_JACK_HOOK_STATE             =0x08
CDC_RING_DETECT                     =0x09
CDC_NOTIFICATION_SERIAL_STATE       =0x20
CDC_CALL_STATE_CHANGE               =0x28
CDC_LINE_STATE_CHANGE               =0x29
CDC_CONNECTION_SPEED_CHANGE         =0x2A

# UART state bitmap values (Serial state notification).
# (usbcdc11.pdf, 6.3.5, Table 69)
CDC_SERIAL_STATE_OVERRUN        =(1 << 6) # receive data overrun error has occurred
CDC_SERIAL_STATE_OVERRUN_Pos    =(6)
CDC_SERIAL_STATE_OVERRUN_Msk    =(1 << CDC_SERIAL_STATE_OVERRUN_Pos)
CDC_SERIAL_STATE_PARITY         =(1 << 5) # parity error has occurred
CDC_SERIAL_STATE_PARITY_Pos     =(5)
CDC_SERIAL_STATE_PARITY_Msk     =(1 << CDC_SERIAL_STATE_PARITY_Pos)
CDC_SERIAL_STATE_FRAMING        =(1 << 4) # framing error has occurred
CDC_SERIAL_STATE_FRAMING_Pos    =(4)
CDC_SERIAL_STATE_FRAMING_Msk    =(1 << CDC_SERIAL_STATE_FRAMING_Pos)
CDC_SERIAL_STATE_RING           =(1 << 3) # state of ring signal detection
CDC_SERIAL_STATE_RING_Pos       =(3)
CDC_SERIAL_STATE_RING_Msk       =(1 << CDC_SERIAL_STATE_RING_Pos)
CDC_SERIAL_STATE_BREAK          =(1 << 2) # state of break detection
CDC_SERIAL_STATE_BREAK_Pos      =(2)
CDC_SERIAL_STATE_BREAK_Msk      =(1 << CDC_SERIAL_STATE_BREAK_Pos)
CDC_SERIAL_STATE_TX_CARRIER     =(1 << 1) # state of transmission carrier
CDC_SERIAL_STATE_TX_CARRIER_Pos =(1)
CDC_SERIAL_STATE_TX_CARRIER_Msk =(1 << CDC_SERIAL_STATE_TX_CARRIER_Pos)
CDC_SERIAL_STATE_RX_CARRIER     =(1 << 0) # state of receiver carrier
CDC_SERIAL_STATE_RX_CARRIER_Pos =(0)
CDC_SERIAL_STATE_RX_CARRIER_Msk =(1 << CDC_SERIAL_STATE_RX_CARRIER_Pos)

CDC_ECM_XMIT_OK                                     =(1 << 0)
CDC_ECM_RVC_OK                                      =(1 << 1)
CDC_ECM_XMIT_ERROR                                  =(1 << 2)
CDC_ECM_RCV_ERROR                                   =(1 << 3)
CDC_ECM_RCV_NO_BUFFER                               =(1 << 4)
CDC_ECM_DIRECTED_BYTES_XMIT                         =(1 << 5)
CDC_ECM_DIRECTED_FRAMES_XMIT                        =(1 << 6)
CDC_ECM_MULTICAST_BYTES_XMIT                        =(1 << 7)
CDC_ECM_MULTICAST_FRAMES_XMIT                       =(1 << 8)
CDC_ECM_BROADCAST_BYTES_XMIT                        =(1 << 9)
CDC_ECM_BROADCAST_FRAMES_XMIT                       =(1 << 10)
CDC_ECM_DIRECTED_BYTES_RCV                          =(1 << 11)
CDC_ECM_DIRECTED_FRAMES_RCV                         =(1 << 12)
CDC_ECM_MULTICAST_BYTES_RCV                         =(1 << 13)
CDC_ECM_MULTICAST_FRAMES_RCV                        =(1 << 14)
CDC_ECM_BROADCAST_BYTES_RCV                         =(1 << 15)
CDC_ECM_BROADCAST_FRAMES_RCV                        =(1 << 16)
CDC_ECM_RCV_CRC_ERROR                               =(1 << 17)
CDC_ECM_TRANSMIT_QUEUE_LENGTH                       =(1 << 18)
CDC_ECM_RCV_ERROR_ALIGNMENT                         =(1 << 19)
CDC_ECM_XMIT_ONE_COLLISION                          =(1 << 20)
CDC_ECM_XMIT_MORE_COLLISIONS                        =(1 << 21)
CDC_ECM_XMIT_DEFERRED                               =(1 << 22)
CDC_ECM_XMIT_MAX_COLLISIONS                         =(1 << 23)
CDC_ECM_RCV_OVERRUN                                 =(1 << 24)
CDC_ECM_XMIT_UNDERRUN                               =(1 << 25)
CDC_ECM_XMIT_HEARTBEAT_FAILURE                      =(1 << 26)
CDC_ECM_XMIT_TIMES_CRS_LOST                         =(1 << 27)
CDC_ECM_XMIT_LATE_COLLISIONS                        =(1 << 28)

CDC_ECM_MAC_STR_DESC                                ="010202030000"
CDC_ECM_MAC_ADDR0                                   =0x00 # 01
CDC_ECM_MAC_ADDR1                                   =0x02 # 02
CDC_ECM_MAC_ADDR2                                   =0x02 # 03
CDC_ECM_MAC_ADDR3                                   =0x03 # 00
CDC_ECM_MAC_ADDR4                                   =0x00 # 00
CDC_ECM_MAC_ADDR5                                   =0x00 # 00

CDC_ECM_NET_DISCONNECTED                            =0x00
CDC_ECM_NET_CONNECTED                               =0x01

CDC_ECM_ETH_STATS_RESERVED                          =0xE0
CDC_ECM_BMREQUEST_TYPE_ECM                          =0xA1

CDC_ECM_CONNECT_SPEED_UPSTREAM                      =0x004C4B40 # 5Mbps
CDC_ECM_CONNECT_SPEED_DOWNSTREAM                    =0x004C4B40 # 5Mbps

CDC_ECM_NOTIFY_CODE_NETWORK_CONNECTION              =0x00
CDC_ECM_NOTIFY_CODE_RESPONSE_AVAILABLE              =0x01
CDC_ECM_NOTIFY_CODE_CONNECTION_SPEED_CHANGE         =0x2A

CDC_NCM_NTH16_SIGNATURE             =0x484D434E
CDC_NCM_NDP16_SIGNATURE_NCM0        =0x304D434E
CDC_NCM_NDP16_SIGNATURE_NCM1        =0x314D434E



class USB_DESC_BytesIO(BytesIO):
    def write(self, buffer):
        if type(buffer) is int:
            return super().write(buffer.to_bytes(1))
        elif type(buffer) is str:
            return super().write(buffer.encode('utf-16-le'))
        elif hasattr(buffer, 'getvalue'):
            return super().write(buffer.getvalue())
        else:
            return super().write(buffer)
    
    def write_mulit(self, *args):
        n = 0
        for arg in args:
            n += self.write(arg)
        return n


def WBVAL(value: int):
    return value.to_bytes(2, 'little')


class USB_DEVICE_DESCRIPTOR_INIT:
    def __init__(self, bcdUSB, bDeviceClass, bDeviceSubClass, bDeviceProtocol, idVendor, idProduct, bcdDevice, bNumConfigurations):
        self.bcdUSB = bcdUSB
        self.bDeviceClass = bDeviceClass
        self.bDeviceSubClass = bDeviceSubClass
        self.bDeviceProtocol = bDeviceProtocol
        self.idVendor = idVendor
        self.idProduct = idProduct
        self.bcdDevice = bcdDevice
        self.bNumConfigurations = bNumConfigurations
        pass

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(0x12)                             # bLength
        buf.write(USB_DESCRIPTOR_TYPE_DEVICE)       # bDescriptorType
        buf.write(WBVAL(self.bcdUSB))               # bcdUSB
        buf.write(self.bDeviceClass)                # bDeviceClass
        buf.write(self.bDeviceSubClass)             # bDeviceSubClass
        buf.write(self.bDeviceProtocol)             # bDeviceProtocol
        buf.write(0x40)                             # bMaxPacketSize
        buf.write(WBVAL(self.idVendor))             # idVendor
        buf.write(WBVAL(self.idProduct))            # idProduct
        buf.write(WBVAL(self.bcdDevice))            # bcdDevice
        buf.write(USB_STRING_MFC_INDEX)             # iManufacturer
        buf.write(USB_STRING_PRODUCT_INDEX)         # iProduct
        buf.write(USB_STRING_SERIAL_INDEX)          # iSerial
        buf.write(self.bNumConfigurations)          # bNumConfigurations
        return buf.getvalue()
    
class USB_CONFIG_DESCRIPTOR_INIT:
    def __init__(self, bConfigurationValue, bmAttributes, bMaxPower, interfaces=[]):
        self.wTotalLength=0
        self.bNumInterfaces=0
        self.bConfigurationValue=bConfigurationValue
        self.bmAttributes=bmAttributes
        self.bMaxPower=bMaxPower
        self.interfaces: List = interfaces

    def append(self, interface):
        self.interfaces.append(interface)

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.seek(9, 0)

        self.bNumInterfaces = 0
        for interface in self.interfaces:
            interface.bInterfaceNumber = self.bNumInterfaces
            self.bNumInterfaces += interface.get_interface_num()
            buf.write(interface.getvalue())

        self.wTotalLength = buf.tell()
        buf.seek(0, 0)
        buf.write(0x09)                                     # bLength
        buf.write(USB_DESCRIPTOR_TYPE_CONFIGURATION)        # bDescriptorType
        buf.write(WBVAL(self.wTotalLength))                 # wTotalLength
        buf.write(self.bNumInterfaces)                      # bNumInterfaces
        buf.write(self.bConfigurationValue)                 # bConfigurationValue
        buf.write(0x00)                                     # iConfiguration
        buf.write(self.bmAttributes)                        # bmAttributes
        buf.write(USB_CONFIG_POWER_MA(self.bMaxPower))      # bMaxPower
        return buf.getvalue()
    
class USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT:
    def __init__(self, bcdUSB, bDeviceClass, bDeviceSubClass, bDeviceProtocol, bNumConfigurations):
        self.bcdUSB = bcdUSB
        self.bDeviceClass = bDeviceClass
        self.bDeviceSubClass = bDeviceSubClass
        self.bDeviceProtocol = bDeviceProtocol
        self.bNumConfigurations = bNumConfigurations

    def getvalue(self):
        buf = USB_DESC_BytesIO()   
        buf.write(0x0A)                                     # bLength
        buf.write(USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER)     # bDescriptorType
        buf.write(WBVAL(self.bcdUSB))                       # bcdUSB
        buf.write(self.bDeviceClass)                        # bDeviceClass
        buf.write(self.bDeviceSubClass)                     # bDeviceSubClass
        buf.write(self.bDeviceProtocol)                     # bDeviceProtocol
        buf.write(0x40)                                     # bMaxPacketSize
        buf.write(self.bNumConfigurations)                  # bNumConfigurations
        buf.write(0x00)                                     # bReserved
        return buf.getvalue()
    


class USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT:
    def __init__(self, wTotalLength, bNumInterfaces, bConfigurationValue, bmAttributes, bMaxPower):
        self.wTotalLength = wTotalLength
        self.bNumInterfaces = bNumInterfaces
        self.bConfigurationValue = bConfigurationValue
        self.bmAttributes = bmAttributes
        self.bMaxPower = bMaxPower

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(0x09)                            # bLength
        buf.write(USB_DESCRIPTOR_TYPE_OTHER_SPEED) # bDescriptorType
        buf.write(WBVAL(self.wTotalLength))             # wTotalLength
        buf.write(self.bNumInterfaces)                  # bNumInterfaces
        buf.write(self.bConfigurationValue)             # bConfigurationValue
        buf.write(0x00)                            # iConfiguration
        buf.write(self.bmAttributes)                    # bmAttributes
        buf.write(USB_CONFIG_POWER_MA(self.bMaxPower))    # bMaxPower
        return buf.getvalue()

class USB_INTERFACE_DESCRIPTOR_INIT:
    def __init__(self, bAlternateSetting, bNumEndpoints,                  
                                      bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol, iInterface, endpoints=[]): 
        self.bInterfaceNumber = 0
        self.bAlternateSetting = bAlternateSetting
        self.bNumEndpoints = bNumEndpoints
        self.bInterfaceClass = bInterfaceClass
        self.bInterfaceSubClass = bInterfaceSubClass
        self.bInterfaceProtocol = bInterfaceProtocol
        self.iInterface = iInterface
        self.endpoints: List = endpoints

    def get_interface_num(self):
        return 1

    def append(self, endpoint):
        self.endpoints.append(endpoint)

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.seek(9, 0)
        for endpoint in self.endpoints:
            buf.write(endpoint.getvalue())
        self.bNumEndpoints = len(self.endpoints)
        buf.seek(0, 0)
        buf.write(0x09)                         # bLength
        buf.write(USB_DESCRIPTOR_TYPE_INTERFACE)# bDescriptorType
        buf.write(self.bInterfaceNumber)             # bInterfaceNumber
        buf.write(self.bAlternateSetting)            # bAlternateSetting
        buf.write(self.bNumEndpoints)                # bNumEndpoints
        buf.write(self.bInterfaceClass)              # bInterfaceClass
        buf.write(self.bInterfaceSubClass)           # bInterfaceSubClass
        buf.write(self.bInterfaceProtocol)           # bInterfaceProtocol
        buf.write(self.iInterface)                     # iInterface
        return buf.getvalue()

class USB_ENDPOINT_DESCRIPTOR_INIT:
    def __init__(self, bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval):
        self.bEndpointAddress = bEndpointAddress
        self.bmAttributes = bmAttributes
        self.wMaxPacketSize = wMaxPacketSize
        self.bInterval = bInterval

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(0x07)                        # bLength
        buf.write(USB_DESCRIPTOR_TYPE_ENDPOINT)# bDescriptorType
        buf.write(self.bEndpointAddress)            # bEndpointAddress
        buf.write(self.bmAttributes)                # bmAttributes
        buf.write(WBVAL(self.wMaxPacketSize))       # wMaxPacketSize
        buf.write(self.bInterval)                     # bInterval
        return buf.getvalue()

class USB_IAD_INIT:
    def __init__(self, bFirstInterface, bInterfaceCount, bFunctionClass, bFunctionSubClass, bFunctionProtocol):
        self.bFirstInterface = bFirstInterface
        self.bInterfaceCount = bInterfaceCount
        self.bFunctionClass = bFunctionClass
        self.bFunctionSubClass = bFunctionSubClass
        self.bFunctionProtocol = bFunctionProtocol

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(0x08)                                     # bLength
        buf.write(USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION)# bDescriptorType
        buf.write(self.bFirstInterface)                          # bFirstInterface
        buf.write(self.bInterfaceCount)                          # bInterfaceCount
        buf.write(self.bFunctionClass)                           # bFunctionClass
        buf.write(self.bFunctionSubClass)                        # bFunctionSubClass
        buf.write(self.bFunctionProtocol)                        # bFunctionProtocol
        buf.write(0x00)                                       # iFunction
        return buf.getvalue()

class USB_LANGID_INIT:
    def __init__(self, id):
        self.id = id

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(0x04)                          # bLength
        buf.write(USB_DESCRIPTOR_TYPE_STRING)    # bDescriptorType
        buf.write(WBVAL(id))                       # wLangID0
        return buf.getvalue()

class USB_STRING:
    def __init__(self, string):
        self.string: str = string
    
    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write(len(self.string) * 2 + 2)
        buf.write(USB_DESCRIPTOR_TYPE_STRING)
        buf.write(self.string.encode('utf-16-le'))
        return buf.getvalue()

class CDC_ACM_DESCRIPTOR_INIT:
    def __init__(self, int_ep, out_ep, in_ep, wMaxPacketSize, str_idx):
        self.bInterfaceNumber = 0
        self.int_ep = int_ep
        self.out_ep = out_ep
        self.in_ep = in_ep
        self.wMaxPacketSize = wMaxPacketSize
        self.str_idx = str_idx

    def get_interface_num(self):
        return 2

    def getvalue(self):
        buf = USB_DESC_BytesIO()
        buf.write_mulit(
            0x08,                                             # bLength
            USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,        # bDescriptorType
            self.bInterfaceNumber,                            # bFirstInterface
            0x02,                                             # bInterfaceCount
            USB_DEVICE_CLASS_CDC,                             # bFunctionClass
            CDC_ABSTRACT_CONTROL_MODEL,                       # bFunctionSubClass
            CDC_COMMON_PROTOCOL_NONE,                         # bFunctionProtocol
            0x00,                                             # iFunction
            0x09,                                             # bLength
            USB_DESCRIPTOR_TYPE_INTERFACE,                    # bDescriptorType
            self.bInterfaceNumber,                            # bInterfaceNumber
            0x00,                                             # bAlternateSetting
            0x01,                                             # bNumEndpoints
            USB_DEVICE_CLASS_CDC,                             # bInterfaceClass
            CDC_ABSTRACT_CONTROL_MODEL,                       # bInterfaceSubClass
            CDC_COMMON_PROTOCOL_NONE,                         # bInterfaceProtocol
            self.str_idx,                                     # iInterface
            0x05,                                             # bLength
            CDC_CS_INTERFACE,                                 # bDescriptorType
            CDC_FUNC_DESC_HEADER,                             # bDescriptorSubtype
            WBVAL(CDC_V1_10),                                 # bcdCDC
            0x05,                                             # bLength
            CDC_CS_INTERFACE,                                 # bDescriptorType
            CDC_FUNC_DESC_CALL_MANAGEMENT,                    # bDescriptorSubtype
            0x00,                                             # bmCapabilities
            (self.bInterfaceNumber + 1),                      # bDataInterface
            0x04,                                             # bLength
            CDC_CS_INTERFACE,                                 # bDescriptorType
            CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT,        # bDescriptorSubtype
            0x02,                                             # bmCapabilities
            0x05,                                             # bLength
            CDC_CS_INTERFACE,                                 # bDescriptorType
            CDC_FUNC_DESC_UNION,                              # bDescriptorSubtype
            self.bInterfaceNumber,                            # bMasterInterface
            (self.bInterfaceNumber + 1),                      # bSlaveInterface0
            0x07,                                             # bLength
            USB_DESCRIPTOR_TYPE_ENDPOINT,                     # bDescriptorType
            self.int_ep,                                      # bEndpointAddress
            0x03,                                             # bmAttributes
            0x08, 0x00,                                       # wMaxPacketSize
            0x0a,                                             # bInterval
            0x09,                                             # bLength
            USB_DESCRIPTOR_TYPE_INTERFACE,                    # bDescriptorType
            (self.bInterfaceNumber + 1),                      # bInterfaceNumber
            0x00,                                             # bAlternateSetting
            0x02,                                             # bNumEndpoints
            CDC_DATA_INTERFACE_CLASS,                         # bInterfaceClass
            0x00,                                             # bInterfaceSubClass
            0x00,                                             # bInterfaceProtocol
            0x00,                                             # iInterface
            0x07,                                             # bLength
            USB_DESCRIPTOR_TYPE_ENDPOINT,                     # bDescriptorType
            self.out_ep,                                      # bEndpointAddress
            0x02,                                             # bmAttributes
            WBVAL(self.wMaxPacketSize),                       # wMaxPacketSize
            0x00,                                             # bInterval
            0x07,                                             # bLength
            USB_DESCRIPTOR_TYPE_ENDPOINT,                     # bDescriptorType
            self.in_ep,                                       # bEndpointAddress
            0x02,                                             # bmAttributes
            WBVAL(self.wMaxPacketSize),                       # wMaxPacketSize
            0x00                                              # bInterval
        )
        return buf.getvalue()


class USB_DESC:
    def __init__(self):
        self.device_descriptor = None
        self.device_descriptor_addr = None
        self.device_descriptor_len = None
        self.config_descriptor_hs = None
        self.config_descriptor_hs_addr = None
        self.config_descriptor_hs_len = None
        self.config_descriptor_fs = None
        self.config_descriptor_fs_addr = None
        self.config_descriptor_fs_len = None
        self.config_descriptor_os = 0x07
        self.config_descriptor_os_addr = None
        self.config_descriptor_hidrpt = None
        self.config_descriptor_hidrpt_addr = 0
        self.config_descriptor_hidrpt_len = 0
        self.device_quality_descriptor = None
        self.device_quality_descriptor_addr = None
        self.device_quality_descriptor_len = None
        self.USBD_BinaryObjectStoreDescriptor = None
        self.USBD_BinaryObjectStoreDescriptor_addr = None
        self.USBD_BinaryObjectStoreDescriptor_len = None
        self.USBD_WinUSBDescriptorSetDescriptor = None
        self.USBD_WinUSBDescriptorSetDescriptor_addr = None
        self.USBD_WinUSBDescriptorSetDescriptor_len = None
        self.str_lang = None
        self.str_lang_addr = None
        self.str_lang_len = None
        self.str_vendor = None
        self.str_vendor_addr = None
        self.str_vendor_len = None
        self.str_product = None
        self.str_product_addr = None
        self.str_product_len = None
        self.other_strings = []
        self.other_strings_addr = []
        self.other_strings_len = []
        self.data = None

    def generate_desc(self):
        rom_buffer = USB_DESC_BytesIO()

        self.device_quality_descriptor_addr = rom_buffer.tell()
        self.device_quality_descriptor_len = rom_buffer.write(self.device_quality_descriptor.getvalue())

        self.device_descriptor_addr = rom_buffer.tell()
        self.device_descriptor_len = rom_buffer.write(self.device_descriptor.getvalue())

        self.config_descriptor_hs_addr = rom_buffer.tell()
        self.config_descriptor_hs_len = rom_buffer.write(self.config_descriptor_hs.getvalue())

        self.config_descriptor_fs_addr = rom_buffer.tell()
        self.config_descriptor_fs_len = rom_buffer.write(self.config_descriptor_fs.getvalue())

        self.config_descriptor_os_addr = rom_buffer.tell()
        rom_buffer.write(self.config_descriptor_os)

        self.USBD_WinUSBDescriptorSetDescriptor_addr = rom_buffer.tell()
        self.USBD_WinUSBDescriptorSetDescriptor_len = rom_buffer.write(self.USBD_WinUSBDescriptorSetDescriptor)

        self.USBD_BinaryObjectStoreDescriptor_addr = rom_buffer.tell()
        self.USBD_BinaryObjectStoreDescriptor_len = rom_buffer.write(self.USBD_BinaryObjectStoreDescriptor)

        self.str_lang_addr = rom_buffer.tell()
        self.str_lang_len = rom_buffer.write(self.str_lang)

        self.str_vendor_addr = rom_buffer.tell()
        self.str_vendor_len = rom_buffer.write(self.str_vendor)

        self.str_product_addr = rom_buffer.tell()
        self.str_product_len = rom_buffer.write(self.str_product)

        self.other_strings_addr = []
        for s in self.other_strings:
            self.other_strings_addr.append(rom_buffer.tell())
            self.other_strings_len.append(rom_buffer.write(s))

        self.data = rom_buffer.getvalue()

        source = StringIO()

        temp = Template('''reg [7:0] desc_rom [0:${desc_len}];

assign desc_dev_addr = 16'd${device_descriptor_addr};
assign desc_dev_len = 16'd${device_descriptor_len};
assign desc_qual_addr = 16'd${device_quality_descriptor_addr};
assign desc_qual_len = 16'd${device_quality_descriptor_len};
assign desc_fscfg_addr = 16'd${config_descriptor_fs_addr};
assign desc_fscfg_len = 16'd${config_descriptor_fs_len};
assign desc_hscfg_addr = 16'd${config_descriptor_hs_addr};
assign desc_hscfg_len = 16'd${config_descriptor_hs_len};
assign desc_oscfg_addr = 16'd${config_descriptor_os_addr};
assign desc_hidrpt_addr = 16'd${config_descriptor_hidrpt_addr};
assign desc_hidrpt_len = 16'd${config_descriptor_hidrpt_len};
assign desc_bos_addr = 16'd${USBD_BinaryObjectStoreDescriptor_addr};
assign desc_bos_len = 16'd${USBD_BinaryObjectStoreDescriptor_len};
assign desc_strlang_addr = 16'd${str_lang_addr};
assign desc_strvendor_addr = 16'd${str_vendor_addr};
assign desc_strvendor_len = 16'd${str_vendor_len};
assign desc_strproduct_addr = 16'd${str_product_addr};
assign desc_strproduct_len = 16'd${str_product_len};
assign desc_strserial_addr = 16'h8000;
assign desc_strserial_len = 16'd66;
assign desc_have_strings = 1'd1;

localparam [15:0] desc_winusb_addr = 16'd${desc_winusb_addr};
localparam [15:0] desc_winusb_len = 16'd${desc_winusb_len};

'''
        )
        map = {
            'desc_len': len(self.data)-1,
            'device_descriptor_addr': self.device_descriptor_addr,
            'device_descriptor_len': self.device_descriptor_len,
            'device_quality_descriptor_addr': self.device_quality_descriptor_addr,
            'device_quality_descriptor_len': self.device_quality_descriptor_len,
            'config_descriptor_fs_addr': self.config_descriptor_fs_addr,
            'config_descriptor_fs_len': self.config_descriptor_fs_len,
            'config_descriptor_hs_addr': self.config_descriptor_hs_addr,
            'config_descriptor_hs_len': self.config_descriptor_hs_len,
            'config_descriptor_os_addr': self.config_descriptor_os_addr,
            'config_descriptor_hidrpt_addr': self.config_descriptor_hidrpt_addr,
            'config_descriptor_hidrpt_len': self.config_descriptor_hidrpt_len,
            'USBD_BinaryObjectStoreDescriptor_addr': self.USBD_BinaryObjectStoreDescriptor_addr,
            'USBD_BinaryObjectStoreDescriptor_len': self.USBD_BinaryObjectStoreDescriptor_len,
            'str_lang_addr': self.str_lang_addr,
            'str_vendor_addr': self.str_vendor_addr,
            'str_vendor_len': self.str_vendor_len,
            'str_product_addr': self.str_product_addr,
            'str_product_len': self.str_product_len,
            'desc_winusb_addr': self.USBD_WinUSBDescriptorSetDescriptor_addr,
            'desc_winusb_len': self.USBD_WinUSBDescriptorSetDescriptor_len,
        }
        source.write(temp.safe_substitute(map))

        print(f'initial begin', file=source)

        addr = 0
        with open("desc.mi", 'w+') as f:
            print('#File_format=Hex', file=f)
            print(f'#Address_depth={len(desc.data)}', file=f)
            print('#Data_width=8', file=f)
            for d in desc.data:
                print(f'    desc_rom[{addr:3}] = 8\'h{d:02x};', file=source)
                print(f'{d:02x}', file=f)
                addr += 1
        print(f'end', file=source)
        return source.getvalue()



DAP_IN_EP  =0x81
DAP_OUT_EP =0x02
DAP_SWO_EP =0x86

CDC_IN_EP  =0x83
CDC_OUT_EP =0x04
CDC_INT_EP =0x85

USBD_VID   =0x0D28
USBD_PID   =0x0204

device_quality_descriptor = USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(
    bcdUSB=USB_2_1,
    bDeviceClass=0x00,
    bDeviceSubClass=0x00,
    bDeviceProtocol=0x00,
    bNumConfigurations=0x01
)

device_descriptor = USB_DEVICE_DESCRIPTOR_INIT(
    bcdUSB=USB_2_1,
    bDeviceClass=0xEF,
    bDeviceSubClass=0x02,
    bDeviceProtocol=0x01,
    idVendor=USBD_VID,
    idProduct=USBD_PID,
    bcdDevice=0x0100,
    bNumConfigurations=1
)


config_descriptor_hs = USB_CONFIG_DESCRIPTOR_INIT(
    bConfigurationValue=0x01,
    bmAttributes=USB_CONFIG_BUS_POWERED,
    bMaxPower=300,
    interfaces=[
        USB_INTERFACE_DESCRIPTOR_INIT(
            bAlternateSetting=0,
            bNumEndpoints=3,
            bInterfaceClass=0xff,
            bInterfaceSubClass=0x00,
            bInterfaceProtocol=0x00,
            iInterface=0,
            endpoints=[
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_OUT_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_HS,
                    bInterval=0x00
                ),
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_IN_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_HS,
                    bInterval=0x00
                ),
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_SWO_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_HS,
                    bInterval=0x00
                ),
            ]
        ),
        CDC_ACM_DESCRIPTOR_INIT(
            int_ep=CDC_INT_EP,
            out_ep=CDC_OUT_EP,
            in_ep=CDC_IN_EP,
            wMaxPacketSize=USB_BULK_EP_MPS_HS,
            str_idx=0
        ),
    ]
)


config_descriptor_fs = USB_CONFIG_DESCRIPTOR_INIT(
    bConfigurationValue=0x01,
    bmAttributes=USB_CONFIG_BUS_POWERED,
    bMaxPower=300,
    interfaces=[
        USB_INTERFACE_DESCRIPTOR_INIT(
            bAlternateSetting=0,
            bNumEndpoints=3,
            bInterfaceClass=0xff,
            bInterfaceSubClass=0x00,
            bInterfaceProtocol=0x00,
            iInterface=0,
            endpoints=[
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_OUT_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_FS,
                    bInterval=0x00
                ),
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_IN_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_FS,
                    bInterval=0x00
                ),
                USB_ENDPOINT_DESCRIPTOR_INIT(
                    bEndpointAddress=DAP_SWO_EP,
                    bmAttributes=USB_ENDPOINT_TYPE_BULK,
                    wMaxPacketSize=USB_BULK_EP_MPS_FS,
                    bInterval=0x00
                ),
            ]
        ),
        CDC_ACM_DESCRIPTOR_INIT(
            int_ep=CDC_INT_EP,
            out_ep=CDC_OUT_EP,
            in_ep=CDC_IN_EP,
            wMaxPacketSize=USB_BULK_EP_MPS_FS,
            str_idx=0
        ),
    ]
)


# USBD_WinUSBDescriptorSetDescriptor



USBD_WEBUSB_ENABLE =0
USBD_BULK_ENABLE   =1
USBD_WINUSB_ENABLE =1


WINUSB_DESCRIPTOR_SET_HEADER_SIZE  =10
WINUSB_FUNCTION_SUBSET_HEADER_SIZE =8
WINUSB_FEATURE_COMPATIBLE_ID_SIZE  =20

FUNCTION_SUBSET_LEN = 160
DEVICE_INTERFACE_GUIDS_FEATURE_LEN = 132
USBD_WINUSB_DESC_SET_LEN = (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + USBD_WEBUSB_ENABLE * FUNCTION_SUBSET_LEN + USBD_BULK_ENABLE * FUNCTION_SUBSET_LEN)


buf = USB_DESC_BytesIO()
buf.write_mulit(
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE),  #wLength 
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE),  #wDescriptorType 
    0x00, 0x00, 0x03, 0x06, # >= Win 8.1       #dwWindowsVersion
    WBVAL(USBD_WINUSB_DESC_SET_LEN),           #wDescriptorSetTotalLength 
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), # wLength 
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), # wDescriptorType 
    0,                                         # bFirstInterface USBD_BULK_IF_NUM
    0,                                         # bReserved 
    WBVAL(FUNCTION_SUBSET_LEN),                # wSubsetLength 
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  # wLength 
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  # wDescriptorType 
    b'WINUSB', 0, 0,        # CompatibleId
    0, 0, 0, 0, 0, 0, 0, 0,                    # SubCompatibleId
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), # wLength 
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   # wDescriptorType 
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), # wPropertyDataType 
    WBVAL(42),                                 # wPropertyNameLength 
    'DeviceInterfaceGUIDs', 0, 0,
    WBVAL(80), #  wPropertyDataLength 
    '{CDB3B5AD-293B-4663-AA36-1AAE46463776}', 0, 0, 0, 0
)

USBD_WinUSBDescriptorSetDescriptor = buf.getvalue()

USBD_WINUSB_VENDOR_CODE = 0x20
USBD_WEBUSB_VENDOR_CODE = 0x21
USBD_NUM_DEV_CAPABILITIES = USBD_WEBUSB_ENABLE + USBD_WINUSB_ENABLE
USBD_WEBUSB_DESC_LEN =24
USBD_WINUSB_DESC_LEN =28
USBD_BOS_WTOTALLENGTH = (0x05 + USBD_WEBUSB_DESC_LEN * USBD_WEBUSB_ENABLE + USBD_WINUSB_DESC_LEN * USBD_WINUSB_ENABLE)


buf = USB_DESC_BytesIO()
buf.write_mulit(
    0x05,                         # bLength 
    0x0f,                         # bDescriptorType 
    WBVAL(USBD_BOS_WTOTALLENGTH), # wTotalLength 
    USBD_NUM_DEV_CAPABILITIES,    # bNumDeviceCaps 
    USBD_WINUSB_DESC_LEN,           # bLength 
    0x10,                           # bDescriptorType 
    USB_DEVICE_CAPABILITY_PLATFORM, # bDevCapabilityType 
    0x00,                           # bReserved 
    0xDF, 0x60, 0xDD, 0xD8,         # PlatformCapabilityUUID 
    0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D,
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06, # >= Win 8.1  # dwWindowsVersion
    WBVAL(USBD_WINUSB_DESC_SET_LEN),         # wDescriptorSetTotalLength 
    USBD_WINUSB_VENDOR_CODE,                 # bVendorCode 
    0                                       # bAltEnumCode
)
USBD_BinaryObjectStoreDescriptor = buf.getvalue()

# print("device_quality_descriptor:\n", device_quality_descriptor.getvalue().hex(' '))
# print("device_descriptor:\n", device_descriptor.getvalue().hex(' '))
# print("config_descriptor_hs:\n", config_descriptor_hs.getvalue().hex(' '))
# print("config_descriptor_fs:\n", config_descriptor_fs.getvalue().hex(' '))
# print("USBD_WinUSBDescriptorSetDescriptor:\n", USBD_WinUSBDescriptorSetDescriptor.hex(' '))
# print("USBD_BinaryObjectStoreDescriptor:\n", USBD_BinaryObjectStoreDescriptor.hex(' '))


desc = USB_DESC()
desc.device_descriptor = device_descriptor
desc.config_descriptor_hs = config_descriptor_hs
desc.config_descriptor_fs = config_descriptor_fs
desc.device_quality_descriptor = device_quality_descriptor
desc.USBD_BinaryObjectStoreDescriptor = USBD_BinaryObjectStoreDescriptor
desc.USBD_WinUSBDescriptorSetDescriptor = USBD_WinUSBDescriptorSetDescriptor
desc.str_lang = b'\x04\x03\x09\x04'
desc.str_vendor = USB_STRING("HFLink")
desc.str_product = USB_STRING("HFLink CMSIS-DAP Plus")
desc.other_strings = [
    # USB_STRING("HFLink CMSIS-DAP Plus"),
    # USB_STRING("HFLink"),
]


print(desc.generate_desc())

