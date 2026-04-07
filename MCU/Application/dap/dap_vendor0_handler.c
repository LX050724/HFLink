
#include "GOWIN_M1.h"
#include "GOWIN_M1_dap.h"
#include "board.h"
#include "config_db/config_db.h"
#include "upgrade/upgrade.h"

#define DAP_VENDOR_CMD_UPGRADE 0x00
#define DAP_VENDOR_CMD_SENSOR 0x01
#define DAP_VENDOR_CMD_CONFIG 0x02

#define DAP_UPGRADE_SUB_CMD_RESET 0xff
#define DAP_UPGRADE_SUB_CMD_START 0x00
#define DAP_UPGRADE_SUB_CMD_DATA 0x01
#define DAP_UPGRADE_SUB_CMD_VERIFY 0x02

#define DAP_CONFIG_SUB_CMD_GET 0x00
#define DAP_CONFIG_SUB_CMD_SET 0x01
#define DAP_CONFIG_SUB_CMD_SAVE 0x02

#define DAP_CONFIG_ID_5VEN 0x00
#define DAP_CONFIG_ID_INDEP_UART 0x01
#define DAP_CONFIG_ID_FREQ_MAP_EN 0x02
#define DAP_CONFIG_ID_FREQ_MAP 0x03
#define DAP_CONFIG_ID_IODELAY 0x04
#define DAP_CONFIG_ID_LEDMODE 0x05
#define DAP_CONFIG_ID_NICKNAME 0x06

static void dap_upgrade_handler(DAP_TypeDef *dap);
static void dap_config_handler(DAP_TypeDef *dap);

void dap_vendor0_handler(DAP_TypeDef *dap)
{
    uint8_t sub_cmd = dap_read_data(dap);

    switch (sub_cmd)
    {
    case DAP_VENDOR_CMD_UPGRADE: {
        dap_upgrade_handler(dap);
        break;
    }
    case DAP_VENDOR_CMD_CONFIG: {
        dap_config_handler(dap);
        break;
    }
    }
}

static void dap_upgrade_handler(DAP_TypeDef *dap)
{
    uint8_t sub_command = dap_read_data(dap);
    if (sub_command == DAP_UPGRADE_SUB_CMD_START)
    {
        uint32_t firm_size = dap_read_data32(dap);
        int ret = upgrade_start(firm_size);
        dap_write_data(dap, ret);
    }
    else if (sub_command == DAP_UPGRADE_SUB_CMD_DATA)
    {
        volatile uint8_t *buffer = upgrade_get_buffer();
        for (int i = 0; i < 256; i += 4)
        {
            if (buffer)
            {
                buffer[i + 0] = dap_read_data(dap);
                buffer[i + 1] = dap_read_data(dap);
                buffer[i + 2] = dap_read_data(dap);
                buffer[i + 3] = dap_read_data(dap);
            }
            else
            {
                dap_read_data(dap);
                dap_read_data(dap);
                dap_read_data(dap);
                dap_read_data(dap);
            }
        }

        if (buffer != NULL)
        {
            upgrade_received_data();
            dap_write_data(dap, 0x00);
        }
        else
        {
            dap_write_data(dap, 0xff);
        }
    }
    else if (sub_command == DAP_UPGRADE_SUB_CMD_VERIFY)
    {
        uint32_t checksum = upgrade_get_verify_result();
        dap_write_data32(dap, checksum);
    }
    else if (sub_command == DAP_UPGRADE_SUB_CMD_RESET)
    {
        upgrade_reset();
    }
}

static void dap_config_handler(DAP_TypeDef *dap)
{
    uint8_t config_id = dap_read_data(dap);
    uint8_t op_cmd = dap_read_data(dap);

    if (op_cmd == DAP_CONFIG_SUB_CMD_SAVE)
    {
        config_data_save();
        dap_write_data(dap, 0x00);
        return;
    }

    print("config %d\n", config_id);


    switch (config_id)
    {
    case DAP_CONFIG_ID_5VEN: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            dap_write_data(dap, global_config.supply5V_enable);
        }
        else
        {
            global_config.supply5V_enable = dap_read_data(dap);
            dap_write_data(dap, 0x00);
        }
        uint8_t en = global_config.supply5V_enable;
        if (en == 1)
        {
            GPIO_WriteBits(GPIO0, POWER_CTL_PIN);
        }
        else if (en == 0)
        {
            GPIO_ResetBit(GPIO0, POWER_CTL_PIN);
        }
        break;
    }
    case DAP_CONFIG_ID_INDEP_UART: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            dap_write_data(dap, global_config.indep_uart_enable);
        }
        else
        {
            global_config.indep_uart_enable = dap_read_data(dap);
            dap_write_data(dap, 0x00);
        }
        uint8_t en = global_config.indep_uart_enable;
        if (en == 1)
        {
            dap_gpio_enable_independent_uart(dap);
        }
        else if (en == 0)
        {
            dap_gpio_disable_independent_uart(dap);
        }
        break;
    }
    case DAP_CONFIG_ID_FREQ_MAP_EN: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            dap_write_data(dap, global_config.freq_mapping_enable);
        }
        else
        {
            global_config.freq_mapping_enable = dap_read_data(dap);
            dap_write_data(dap, 0x00);
        }
        break;
    }
    case DAP_CONFIG_ID_FREQ_MAP: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            for (int i = 0; i < 11; i++)
            {
                dap_write_data32(dap, global_config.clock_freq_mapping[i]);
            }
        }
        else
        {
            for (int i = 0; i < 11; i++)
            {
                global_config.clock_freq_mapping[i] = dap_read_data32(dap);
            }
            dap_write_data(dap, 0x00);
        }
        for (int i = 0; i < 11; i++)
        {
            print("%d: %d\n", i, global_config.clock_freq_mapping[i]);
        }
        break;
    }
    case DAP_CONFIG_ID_IODELAY: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            for (int i = 0; i < 6; i++)
            {
                dap_write_data(dap, global_config.iodelay_param[i]);
            }
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                global_config.iodelay_param[i] = dap_read_data(dap);
            }
            dap_write_data(dap, 0x00);

            DAP->GPIO.TCK_DELAY = global_config.iodelay_param[0];
            DAP->GPIO.TMS_T_DELAY = global_config.iodelay_param[1];
            DAP->GPIO.TMS_O_DELAY = global_config.iodelay_param[2];
            DAP->GPIO.TMS_I_DELAY = global_config.iodelay_param[3];
            DAP->GPIO.TDO_DELAY = global_config.iodelay_param[4];
            DAP->GPIO.TDI_DELAY = global_config.iodelay_param[5];
        }
        break;
    }
    case DAP_CONFIG_ID_LEDMODE: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            dap_write_data(dap, global_config.led_mode);
        }
        else
        {
            global_config.led_mode = dap_read_data(dap);
            dap_write_data(dap, 0x00);
        }
        break;
    }
    case DAP_CONFIG_ID_NICKNAME: {
        if (op_cmd == DAP_CONFIG_SUB_CMD_GET)
        {
            for (size_t i = 0; i < sizeof(global_config.nickname); i++)
            {
                dap_write_data(dap, global_config.nickname[i]);
            }
        }
        else
        {
            for (size_t i = 0; i < sizeof(global_config.nickname); i++)
            {
                global_config.nickname[i] = dap_read_data(dap);
            }
            dap_write_data(dap, 0x00);
        }
        print("nickname: %s\n", global_config.nickname);
        break;
    }
    default:
        // 不支持的参数
        dap_write_data(dap, 0xff);
        break;
        ;
    }
}
