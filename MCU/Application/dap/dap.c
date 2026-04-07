#include "DAP_config.h"
#include "cmsis_compiler.h"
#include "dap.h"
#include "config_db/config_db.h"
#include "usb/usbd_core.h"
#include <GOWIN_M1.h>
#include <GOWIN_M1_dap.h>
#include <string.h>

#ifdef DEBUG
#include "SEGGER_RTT.h"
#define DAP_DEBUG(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define DAP_DEBUG(fmt, ...)
#endif

static void dap_return_n_string(DAP_TypeDef *dap, const char *str);
static void dap_get_info_handler(DAP_TypeDef *dap);
static void dap_connect_handler(DAP_TypeDef *dap);
static void dap_disconnect_handler(DAP_TypeDef *dap);
static void dap_transfer_configure_handler(DAP_TypeDef *dap);
static void dap_host_status_handler(DAP_TypeDef *dap);
static void dap_swj_clock_handler(DAP_TypeDef *dap);
static void dap_swd_configure_handler(DAP_TypeDef *dap);
static void dap_swo_transport_handler(DAP_TypeDef *dap);
static void dap_swo_mode_handler(DAP_TypeDef *dap);
void dap_vendor0_handler(DAP_TypeDef *dap);

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
    case ID_DAP_Disconnect:
        dap_disconnect_handler(dap);
        break;
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
    case ID_DAP_Vendor0:
        dap_vendor0_handler(dap);
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
        dap_return_n_string(dap, usbd_get_serial_number_str());
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
        dap_write_data(dap, 0x02);
        dap_write_data(dap, 0xB1);
        dap_write_data(dap, 0x01);
        break;
    case DAP_ID_TIMESTAMP_CLOCK:
        dap_write_data(dap, 0x04);
        dap_write_data32(dap, TIMESTAMP_CLOCK);
        break;
    case DAP_ID_PACKET_COUNT:
        dap_write_data(dap, 0x01);
        dap_write_data(dap, DAP_PACKET_COUNT);
        break;
    case DAP_ID_PACKET_SIZE:
        dap_write_data(dap, 0x02);
        dap_write_data16(dap, DAP_PACKET_SIZE);
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
    dap_swj_set_mode(dap, (port == 1) ? DAP_SWJ_MODE_SWD : DAP_SWJ_MODE_JTAG);
    dap_write_data(dap, port);
}

static void dap_disconnect_handler(DAP_TypeDef *dap)
{
    dap_write_data(dap, 0x00);
}

static void dap_swj_clock_handler(DAP_TypeDef *dap)
{
    uint32_t clock = dap_read_data32(dap);
    uint16_t reload = 0;

    if (global_config.freq_mapping_enable)
    {
        uint32_t origin_clock = clock;
        switch (clock)
        {
        case 10000000:
            clock = global_config.clock_freq_mapping[0];
            break;
        case 5000000:
            clock = global_config.clock_freq_mapping[1];
            break;
        case 2000000:
            clock = global_config.clock_freq_mapping[2];
            break;
        case 1000000:
            clock = global_config.clock_freq_mapping[3];
            break;
        case 500000:
            clock = global_config.clock_freq_mapping[4];
            break;
        case 200000:
            clock = global_config.clock_freq_mapping[5];
            break;
        case 100000:
            clock = global_config.clock_freq_mapping[6];
            break;
        case 50000:
            clock = global_config.clock_freq_mapping[7];
            break;
        case 20000:
            clock = global_config.clock_freq_mapping[8];
            break;
        case 10000:
            clock = global_config.clock_freq_mapping[9];
            break;
        case 5000:
            clock = global_config.clock_freq_mapping[10];
            break;
        default:
            break;
        }
        DAP_DEBUG("Clock Mapping %d -> %d\n", origin_clock, clock);
    }

    if (clock != 0)
    {
        reload = SystemCoreClock / clock;
    }
    
    if (reload == 0)
    {
        DAP_DEBUG("SWJ Clock %d, Error\n", clock);
        dap_write_data(dap, 0xff);
        return;
    }

    dap_baud_stop(dap);
    if (reload == 1)
    {
        // 60M特殊时序: 下降沿置位，上升沿滞后1周期采样
        // 采样时刻距离上升沿8.33ns
        dap_baud_set_reload(dap, reload);
        dap_baud_set_simpling_cmp(dap, reload);
        dap_buad_set_simpling_delay(dap, 1);
        // turn 2周期，第二周期不输出时钟
        dap_swd_set_trun_cycle(dap, 1);
        dap_swd_disable_turn_clk(dap);
    }
    else
    {
        // 下降沿置位，上升沿采样，无滞后
        dap_baud_set_reload(dap, reload);
        dap_baud_set_simpling_cmp(dap, reload);
        dap_buad_set_simpling_delay(dap, 0);
        // turn默认配置1周期，输出时钟
        dap_swd_set_trun_cycle(dap, 0);
        dap_swd_enable_turn_clk(dap);
    }
    dap_baud_start(dap);

    DAP_DEBUG("SWJ Clock %d, %d\n", clock, reload);
    dap_write_data(dap, 0);
}

static void dap_swd_configure_handler(DAP_TypeDef *dap)
{
    uint8_t configure = dap_read_data(dap);
    uint8_t turn_cycle = configure & 0x03;
    uint8_t data_phase = (configure >> 2) & 0x01;

    if (dap_baud_get_reload(dap) > 1 || turn_cycle > 0)
    {
        dap_swd_set_data_phase(dap, data_phase);
        dap_swd_set_trun_cycle(dap, turn_cycle);
        dap_swd_enable_turn_clk(dap);
        DAP_DEBUG("SWD tr %d, data phase %d\n", turn_cycle, data_phase);
    }
    else
    {
        DAP_DEBUG("SWD tr %d, data phase %d, ignore\n", turn_cycle, data_phase);
    }
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
    uint16_t wait_retry = dap_read_data16(dap);
    uint16_t match_retry = dap_read_data16(dap);
    dap_swj_set_match_retry(dap, match_retry);
    dap_swj_set_wait_retry(dap, wait_retry);
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

__WEAK void dap_vendor0_handler(DAP_TypeDef *dap)
{
    (void)dap;
}
