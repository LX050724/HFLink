#include "DAP_config.h"
#include "SEGGER_RTT.h"
#include "dap.h"
#include <GOWIN_M1.h>
#include <GOWIN_M1_dap.h>
#include <string.h>

#define DAP_DEBUG(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

static void dap_return_n_string(DAP_TypeDef *dap, const char *str);
static void dap_get_info_handler(DAP_TypeDef *dap);
static void dap_connect_handler(DAP_TypeDef *dap);
static void dap_transfer_configure_handler(DAP_TypeDef *dap);
static void dap_host_status_handler(DAP_TypeDef *dap);
static void dap_swj_clock_handler(DAP_TypeDef *dap);
static void dap_swd_configure_handler(DAP_TypeDef *dap);
static void dap_swo_transport_handler(DAP_TypeDef *dap);
static void dap_swo_mode_handler(DAP_TypeDef *dap);

void dap_irq_handler(DAP_TypeDef *dap)
{
    uint8_t cmd = dap_get_current_cmd(dap);
    switch (cmd)
    {
    case ID_DAP_Info:
        dap_get_info_handler(dap);
        break;
    case ID_DAP_HostStatus:
        dap_host_status_handler(dap);
        break;
    case ID_DAP_Connect:
        dap_connect_handler(dap);
        break;
    // case ID_DAP_Disconnect:
    case ID_DAP_TransferConfigure:
        dap_transfer_configure_handler(dap);
        break;
    case ID_DAP_SWJ_Clock:
        dap_swj_clock_handler(dap);
        break;
    case ID_DAP_SWD_Configure:
        dap_swd_configure_handler(dap);
        break;
    case ID_DAP_SWO_Transport:
        dap_swo_transport_handler(dap);
        break;
    case ID_DAP_SWO_Mode:
        dap_swo_mode_handler(dap);
        break;
    default:
        DAP_DEBUG("unknown cmd %02x\n", cmd);
        dap_write_data(dap, 0xff);
    }

    DAP->SR = 0x80000000;
}

static void dap_get_info_handler(DAP_TypeDef *dap)
{
    uint8_t info_req = dap_read_data(dap);
    switch (info_req)
    {
    case DAP_ID_VENDOR:
        dap_return_n_string(dap, "HFLink");
        break;
    case DAP_ID_PRODUCT:
        dap_return_n_string(dap, "HFLink CMSIS-DAP");
        break;
    case DAP_ID_SER_NUM:
        dap_return_n_string(dap, "12345678");
        break;
    case DAP_ID_DAP_FW_VER:
        dap_return_n_string(dap, DAP_FW_VER);
        break;
    case DAP_ID_DEVICE_VENDOR:
        dap_return_n_string(dap, TARGET_DEVICE_VENDOR);
        break;
    case DAP_ID_DEVICE_NAME:
        dap_return_n_string(dap, TARGET_DEVICE_NAME);
        break;
    case DAP_ID_BOARD_VENDOR:
        dap_return_n_string(dap, TARGET_BOARD_VENDOR);
        break;
    case DAP_ID_BOARD_NAME:
        dap_return_n_string(dap, TARGET_BOARD_NAME);
        break;
    case DAP_ID_PRODUCT_FW_VER:
        dap_return_n_string(dap, "0.0.0");
        break;
    case DAP_ID_CAPABILITIES:
        // 支持全部特性
        dap_write_data(dap, 0x02);
        dap_write_data(dap, 0xff);
        dap_write_data(dap, 0x01);
        break;
    case DAP_ID_TIMESTAMP_CLOCK:
        dap_write_data(dap, 0x04);
        dap_write_data(dap, TIMESTAMP_CLOCK >> 0);
        dap_write_data(dap, TIMESTAMP_CLOCK >> 8);
        dap_write_data(dap, TIMESTAMP_CLOCK >> 16);
        dap_write_data(dap, TIMESTAMP_CLOCK >> 24);
        break;
    case DAP_ID_PACKET_COUNT:
        dap_write_data(dap, 0x01);
        dap_write_data(dap, DAP_PACKET_COUNT);
        break;
    case DAP_ID_PACKET_SIZE:
        dap_write_data(dap, 0x02);
        dap_write_data(dap, DAP_PACKET_SIZE & 0xff);
        dap_write_data(dap, DAP_PACKET_SIZE >> 8);
        break;
    default:
        DAP_DEBUG("unknown info id %02x\n", info_req);
        dap_write_data(dap, 0);
        break;
    }
}

static void dap_connect_handler(DAP_TypeDef *dap)
{
    uint8_t port = dap_read_data(dap);
    if (port == 0)
        port = DAP_DEFAULT_PORT;
    dap_write_data(dap, port);
}

static void dap_swj_clock_handler(DAP_TypeDef *dap)
{
    uint32_t clock = dap_read_data(dap);
    clock |= dap_read_data(dap) << 8;
    clock |= dap_read_data(dap) << 16;
    clock |= dap_read_data(dap) << 24;

    DAP_DEBUG("SWJ Clock %d\n", clock);
    dap_write_data(dap, 0);
}

static void dap_swd_configure_handler(DAP_TypeDef *dap)
{
    uint8_t configure = dap_read_data(dap);
    DAP_DEBUG("SWD tr %d, data phase %d\n", configure & 0x03, (configure >> 2) & 0x01);
    dap_write_data(dap, 0);
}

static void dap_swo_transport_handler(DAP_TypeDef *dap)
{
    uint8_t transport = dap_read_data(dap);
    DAP_DEBUG("SWO transport %d\n", transport);
    dap_write_data(dap, 0);
}

static void dap_swo_mode_handler(DAP_TypeDef *dap)
{
    uint8_t mode = dap_read_data(dap);
    DAP_DEBUG("SWO Mode %02x\n", mode);
    dap_write_data(dap, 0);
}

static void dap_transfer_configure_handler(DAP_TypeDef *dap)
{
    uint8_t idle_cycles = dap_read_data(dap);
    uint16_t wait_retry = dap_read_data(dap) | dap_read_data(dap) << 8;
    uint16_t match_retry = dap_read_data(dap) | dap_read_data(dap) << 8;

    DAP_DEBUG("Transfer idle %d cycles, wait retry %d, match retry %d\n", idle_cycles, wait_retry, match_retry);
    dap_write_data(dap, 0);
}

static void dap_host_status_handler(DAP_TypeDef *dap)
{
    uint8_t type = dap_read_data(dap);
    uint8_t status = dap_read_data(dap);
    DAP_DEBUG("Host status %02x, %02x\n", type, status);
    dap_write_data(dap, 0);
}

static void dap_return_n_string(DAP_TypeDef *dap, const char *str)
{
    if (str)
    {
        int n = strlen(str);
        dap_write_data(dap, n + 1);
        for (int i = 0; i <= n; i++)
        {
            dap_write_data(dap, str[i]);
        }
    }
    else
    {
        dap_write_data(dap, 0);
    }
}
