
#include "GOWIN_M1.h"
#include "GOWIN_M1_dap.h"
#include "board.h"
#include "easyflash.h"
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

static char dap_bool_env(DAP_TypeDef *dap, const char *name)
{
    uint8_t get_set = dap_read_data(dap);
    uint8_t en = 0;
    if (get_set == DAP_CONFIG_SUB_CMD_GET)
    {
        dap_write_data(dap, ef_get_env(name)[0]);
    }
    else
    {
        en = dap_read_data(dap);
        ef_set_env_blob(name, &en, 1);
        // 协议返回0
        dap_write_data(dap, 0x00);
    }
    return en;
}

static int dap_binary_env(DAP_TypeDef *dap, const char *name, void *buf, size_t len)
{
    uint8_t get_set = dap_read_data(dap);
    if (get_set == DAP_CONFIG_SUB_CMD_GET)
    {
        ef_get_env_blob(name, buf, len, NULL);
        for (size_t i = 0; i < len; i++)
        {
            dap_write_data(dap, ((uint8_t *)buf)[i]);
        }
        return 0;
    }
    else
    {
        for (size_t i = 0; i < len; i++)
        {
            ((uint8_t *)buf)[i] = dap_read_data(dap);
        }
        ef_set_env_blob(name, buf, len);
        // 协议返回0
        dap_write_data(dap, 0x00);
        return 1;
    }
}

static void dap_config_handler(DAP_TypeDef *dap)
{
    uint8_t config_id = dap_read_data(dap);
    print("config %d\n", config_id);

    switch (config_id)
    {
    case DAP_CONFIG_ID_5VEN: {
        char en = dap_bool_env(dap, "5V_EN");
        if (en == '1')
        {
            GPIO_WriteBits(GPIO0, POWER_CTL_PIN);
        }
        else if (en == '0')
        {
            GPIO_ResetBit(GPIO0, POWER_CTL_PIN);
        }
        break;
    }
    case DAP_CONFIG_ID_INDEP_UART: {
        char en = dap_bool_env(dap, "INDEP_UART");
        if (en == '1')
        {
            dap_gpio_enable_independent_uart(dap);
        }
        else if (en == '0')
        {
            dap_gpio_disable_independent_uart(dap);
        }
        break;
    }
    case DAP_CONFIG_ID_FREQ_MAP_EN: {
        dap_bool_env(dap, "FREQ_MAP_EN");
        break;
    }
    case DAP_CONFIG_ID_FREQ_MAP: {
        uint32_t freq_map[11] = {};
        if (dap_binary_env(dap, "FREQ_MAP", freq_map, sizeof(freq_map)) == 1)
        {
            for (int i = 0; i < 11; i++)
            {
                print("%d: %d\n", i, freq_map[i]);
            }
        }
        break;
    }
    case DAP_CONFIG_ID_IODELAY: {
        uint8_t iodelay_param[6] = {};
        if (dap_binary_env(dap, "IODELAY", iodelay_param, sizeof(iodelay_param)) == 1)
        {
            DAP->GPIO.TCK_DELAY = iodelay_param[0];
            DAP->GPIO.TMS_T_DELAY = iodelay_param[1];
            DAP->GPIO.TMS_O_DELAY = iodelay_param[2];
            DAP->GPIO.TMS_I_DELAY = iodelay_param[3];
            DAP->GPIO.TDO_DELAY = iodelay_param[4];
            DAP->GPIO.TDI_DELAY = iodelay_param[5];
        }
        break;
    }
    case DAP_CONFIG_ID_LEDMODE: {
        uint8_t led_mode = 0;
        dap_binary_env(dap, "LEDMODE", &led_mode, sizeof(led_mode));
        break;
    }
    case DAP_CONFIG_ID_NICKNAME: {
        char name[32] = "";
        dap_binary_env(dap, "NICKNAME", &name, sizeof(name));
        break;
    }
    default:
        // 不支持的参数
        dap_write_data(dap, 0xff);
        break;
        ;
    }
}
