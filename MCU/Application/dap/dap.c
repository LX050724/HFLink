#include "DAP_config.h"
#include "cmsis_compiler.h"
#include "config_db/config_db.h"
#include "dap.h"
#include "usb/usbd_core.h"
#include <GOWIN_M1.h>
#include <GOWIN_M1_dap.h>
#include <string.h>
#include "board.h"

#ifdef DEBUG
#include "SEGGER_RTT.h"
#define DAP_DEBUG(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define DAP_DEBUG(fmt, ...)
#endif

typedef struct
{
    uint8_t mode;
    uint32_t baudrate;
} SWO_Config;

static SWO_Config swo_cfg;

static void dap_get_info_handler(DAP_TypeDef *dap);
static void dap_connect_handler(DAP_TypeDef *dap);
static void dap_disconnect_handler(DAP_TypeDef *dap);
static void dap_transfer_configure_handler(DAP_TypeDef *dap);
static void dap_host_status_handler(DAP_TypeDef *dap);
static void dap_swj_clock_handler(DAP_TypeDef *dap);
static void dap_swd_configure_handler(DAP_TypeDef *dap);
static void dap_jtag_configure_handler(DAP_TypeDef *dap);
static void dap_swo_transport_handler(DAP_TypeDef *dap);
static void dap_swo_mode_handler(DAP_TypeDef *dap);
static void dap_swo_baudrate_handler(DAP_TypeDef *dap);
static void dap_swo_control_handler(DAP_TypeDef *dap);
static void dap_swo_data_handler(DAP_TypeDef *dap);
void dap_vendor0_handler(DAP_TypeDef *dap);
void dap_vendor1_handler(DAP_TypeDef *dap);

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
    case ID_DAP_JTAG_Configure:
        dap_jtag_configure_handler(dap);
        break;
    case ID_DAP_SWO_Transport:
        dap_swo_transport_handler(dap);
        break;
    case ID_DAP_SWO_Mode:
        dap_swo_mode_handler(dap);
        break;
    case ID_DAP_SWO_Baudrate:
        dap_swo_baudrate_handler(dap);
        break;
    case ID_DAP_SWO_Control:
        dap_swo_control_handler(dap);
        break;
    case ID_DAP_SWO_Data:
        dap_swo_data_handler(dap);
        break;
    case ID_DAP_Vendor0:
        dap_vendor0_handler(dap);
        break;
    case ID_DAP_Vendor1:
        dap_vendor1_handler(dap);
        break;
    default:
        DAP_DEBUG("unknown cmd %02x\n", cmd);
        dap_write_data(dap, 0xff);
    }

    DAP->SR = 1;
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
        dap_write_data(dap, 0xFF);
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
    case DAP_ID_SWO_BUFFER_SIZE:
        dap_write_data(dap, 0x04);
        dap_write_data32(dap, SWO_BUFFER_SIZE);
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

    if (get_vtrg_voltage_mv() < 1650)
    {
        DAP_DEBUG("VTRG Voltage %dmV, too low\n", get_vtrg_voltage_mv());
        dap_write_data(dap, 0);
        return;
    }

    if (port == 0)
        port = DAP_DEFAULT_PORT;

    switch (port) {
        case 1:
            dap_swj_set_mode(dap, DAP_SWJ_MODE_SWD);
            dap_write_data(dap, 1);
            break;
        case 2:
            dap_swj_set_mode(dap, DAP_SWJ_MODE_JTAG);
            dap_write_data(dap, 2);
            break;
        default:
            // 未知端口
            DAP_DEBUG("Unknown port %d\n", port);
            dap_write_data(dap, 0);
            break;
    }
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

static void dap_jtag_configure_handler(DAP_TypeDef *dap)
{
    uint8_t irlen[8] = {0};
    uint8_t ir_before = 0;
    uint8_t ir_after = 0;
    uint8_t count = dap_read_data(dap);
    DAP_DEBUG("JTAG configure count %d\n", count);
    if (count > 8)
    {
        for (int i = 0; i < count; i++)
            dap_read_data(DAP);
        dap_write_data(dap, 0xff);
        return;
    }
    dap_jtag_set_tap_num(dap, count - 1);
    for (int i = 0; i < count; i++)
    {
        irlen[i] = dap_read_data(dap);
        dap_jtag_set_ir_before_len(dap, i, ir_before);
        dap_jtag_set_irlen(dap, i, irlen[i]);
        ir_before += irlen[i];
        ir_after += irlen[i];
    }

    for (int i = 0; i < count; i++)
    {
        ir_after -= irlen[i];
        dap_jtag_set_ir_after_len(dap, i, ir_after);
    }

    for (int i = 0; i < count; i++)
    {
        DAP_DEBUG("[%d] %d, %d, %d\n", i, dap->SWJ.JTAG_IR_CONF[i].IR_BEFORE_LEN, dap->SWJ.JTAG_IR_CONF[i].IR_LEN,
                  dap->SWJ.JTAG_IR_CONF[i].IR_AFTER_LEN);
    }

    dap_write_data(dap, 0);
}

static void dap_swo_transport_handler(DAP_TypeDef *dap)
{
    uint8_t transport = dap_read_data(dap);
    DAP_DEBUG("SWO transport %d\n", transport);

    if (transport == 0x02 || transport == 0x00)
    {
        dap_write_data(dap, 0);
    }
    else
    {
        DAP_DEBUG("Error: unsupported transport\n");
        dap_write_data(dap, 0xff);
    }
}

static void dap_swo_mode_handler(DAP_TypeDef *dap)
{
    uint8_t mode = dap_read_data(dap);

    switch (mode)
    {
    case 0x00:
        swo_cfg.mode = 0xff;
        DAP_DEBUG("SWO mode default\n");
        break;
    case 0x01:
        DAP_DEBUG("SWO mode uart\n");
        swo_cfg.mode = DAP_SWO_CR_MODE_UART;
        break;
    case 0x02:
        DAP_DEBUG("SWO mode manchester\n");
        swo_cfg.mode = DAP_SWO_CR_MODE_MANCHESTER;
        break;
    default:
        dap_write_data(dap, 0xff);
        return;
    }
    dap_write_data(dap, 0x00);
}

static void dap_swo_baudrate_handler(DAP_TypeDef *dap)
{
    uint32_t baudrate = dap_read_data32(dap);
    DAP_DEBUG("SWO baudrate %d\n", baudrate);

    if (baudrate > 100000000 || baudrate < 50000)
    {
        DAP_DEBUG("Error: baudrate is invalid\n");
        dap_write_data(dap, 0xff);
        return;
    }

    swo_cfg.baudrate = baudrate;
    uint32_t bit_time = 3200000000ULL / baudrate;
    if (swo_cfg.mode == DAP_SWO_CR_MODE_MANCHESTER)
    {
        bit_time &= 0xfff0;
        return;
    }

    uint32_t actual_baudrate = 3200000000ULL / bit_time;
    uint32_t diff = abs((int)actual_baudrate - (int)baudrate) * 10000 / baudrate;
    DAP_DEBUG("actual baudrate %u, err: %3d.%02d%%\n", actual_baudrate, diff / 100, diff % 100);
    DAP_DEBUG("bit time %d, jitter %d\n", bit_time >> 4, bit_time & 0xf);

    dap_write_data32(dap, baudrate);
}

static int dap_swo_apply_config(DAP_TypeDef *dap)
{
    if (dap_swo_is_enabled(dap))
    {
        DAP_DEBUG("Error: SWO is enabled\n");
        return -1;
    }

    if (swo_cfg.mode == 0xff)
    {
        DAP_DEBUG("Error: mode is not set\n");
        return -1;
    }

    if (swo_cfg.baudrate > 100000000 || swo_cfg.baudrate < 5000000)
    {
        DAP_DEBUG("Error: baudrate is invalid\n");
        return -1;
    }

    uint32_t baud_div = 3200000000ULL / swo_cfg.baudrate; // 波特率分频值
    uint16_t bt = (baud_div >> 4);                        // bit时钟周期数
    uint16_t bit_time0 = bt - 1;                          // 默认bit时间
    uint16_t bit_time1 = bt + bt / 16 - 1;                // 抖动注入bit时间
    uint16_t decision_low;                                // 死区时间
    uint16_t decision_high;                               // 超时时间

    // bit_time1 一定要大于 bit_time0
    if (bit_time1 == bit_time0)
    {
        bit_time1 = bt;
    }

    if (swo_cfg.mode == DAP_SWO_CR_MODE_UART)
    {
        dap_swo_set_mode(dap, DAP_SWO_CR_MODE_UART);
        dap_swo_set_jitter(dap, baud_div & 0xf); // 抖动注入比例，高波特率会增加误码
        decision_low = bt - bt / 2 - 1;          // NRZ死区时间  50%
        decision_high = bt + bt / 4 - 1;         // NRZ超时时间 125%

        // decision_low 不能大于 bit_time0
        if (decision_low >= bit_time0)
        {
            decision_low = bit_time0 - 1;
        }

        if ((baud_div & 0xf) != 0)
        {
            // 有注入时，decision_high 不能小于 bit_time1
            if (decision_high < bit_time1)
            {
                decision_high = bit_time1;
            }
        }
        else
        {
            // 无注入时，decision_high 不能小于 bit_time0
            if (decision_high < bit_time0)
            {
                decision_high = bit_time0;
            }
        }

        // decision_high 不能大于 bit_time0 两倍
        if (decision_high >= bit_time0 * 2)
        {
            decision_high = bit_time0 * 2 - 1;
        }
    }
    else if (swo_cfg.mode == DAP_SWO_CR_MODE_MANCHESTER)
    {
        dap_swo_set_mode(dap, DAP_SWO_CR_MODE_MANCHESTER);
        dap_swo_set_jitter(dap, 0);  // 曼彻斯特不使用jitter
        decision_low = bt / 2;       // 曼彻斯特死区时间  50%
        decision_high = bt + bt / 4; // 曼彻斯特超时时间 125%
    }
    else
    {
        DAP_DEBUG("Error: unsupported mode %d\n", swo_cfg.mode);
        return -1;
    }

    dap_swo_set_bit_time0(dap, bit_time0);
    dap_swo_set_bit_time1(dap, bit_time1);
    dap_swo_set_bit_decision_low(dap, decision_low);
    dap_swo_set_bit_decision_high(dap, decision_high);
    dap_swo_set_edge(dap, (bt / 200) & 0xf);

    DAP_DEBUG("SWO config applied mode %d\n", swo_cfg.mode);
    DAP_DEBUG("bit time %d / %d\n", bit_time0, bit_time1);
    DAP_DEBUG("decision %d / %d\n", decision_low, decision_high);
    DAP_DEBUG("jitter level %d\n", dap_swo_get_jitter(dap));
    DAP_DEBUG("edge %d\n", dap_swo_get_edge(dap));
    return 0;
}

static void dap_swo_control_handler(DAP_TypeDef *dap)
{
    uint8_t control = dap_read_data(dap);
    DAP_DEBUG("SWO control %d\n", control);
    if (control)
    {
        if (dap_swo_apply_config(dap) != 0)
        {
            DAP_DEBUG("Error: apply config failed\n");
            dap_write_data(dap, 0xff);
            return;
        }

        dap_swo_clear_fifo_start(dap);
        while (!dap_swo_fifo_clear_status(dap))
            ;
        dap_swo_clear_fifo_stop(dap);
        while (dap_swo_fifo_clear_status(dap))
            ;
        dap_swo_enable(dap);
    }
    else
    {
        dap_swo_disable(dap);
    }
    dap_write_data(dap, 0);
}

static void dap_swo_data_handler(DAP_TypeDef *dap)
{
    (void)dap_read_data16(dap);
    dap_write_data(dap, dap_swo_is_enabled(dap));
    dap_write_data16(dap, 0);
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

    if (global_config.led_mode == 1)
    {
        switch (type)
        {
        case DAP_DEBUGGER_CONNECTED:
            if (status)
            {
                dap_gpio_set_led_cmp(dap, 0, 0, 255);
            }
            else
            {
                dap_gpio_set_led_cmp(dap, 0, 255, 0);
            }
            break;
        case DAP_TARGET_RUNNING:
            if (status)
            {
                dap_gpio_set_led_cmp(dap, 200, 0, 255);
            }
            else
            {
                dap_gpio_set_led_cmp(dap, 0, 0, 255);
            }
            break;
        default:
            break;
        }
    }

    DAP_DEBUG("Host status %02x, %02x\n", type, status);
    dap_write_data(dap, 0);
}

void dap_return_n_string(DAP_TypeDef *dap, const char *str)
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

__WEAK void dap_vendor1_handler(DAP_TypeDef *dap)
{
    (void)dap;
}
