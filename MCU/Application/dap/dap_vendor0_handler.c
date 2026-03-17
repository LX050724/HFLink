
#include "GOWIN_M1.h"
#include "GOWIN_M1_dap.h"
#include "upgrade/upgrade.h"

#define DAP_VENDOR_CMD_UPGRADE 0x00
#define DAP_VENDOR_CMD_SENSOR 0x01
#define DAP_VENDOR_CMD_CONFIG 0x02

void dap_vendor0_handler(DAP_TypeDef *dap)
{
    uint8_t cmd = dap_read_data(dap);

    switch (cmd)
    {
    case DAP_VENDOR_CMD_UPGRADE: {
        uint8_t sub_command = dap_read_data(dap);
        if (sub_command == 0x00)
        {
            uint32_t firm_size = dap_read_data32(dap);
            int ret = upgrade_start(firm_size);
            dap_write_data(dap, ret);
        }
        else if (sub_command == 0x01)
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
        else if (sub_command == 0x02)
        {
            uint32_t checksum = upgrade_get_verify_result();
            dap_write_data32(dap, checksum);
        }
        else if (sub_command == 0xff)
        {
            upgrade_reset();
        }
        break;
    }
    }
}
