#include <GOWIN_M1_dap.h>

uint8_t dap_read_data(DAP_TypeDef *dap)
{
    uint16_t tmp;
    do
    {
        tmp = dap->DR.RD;
    } while ((tmp & 0x100) == 0);
    return tmp;
}

uint16_t dap_read_data16(DAP_TypeDef *dap)
{
    uint32_t tmp = dap_read_data(dap);
    tmp |= dap_read_data(dap) << 8;
    return tmp;
}
uint32_t dap_read_data32(DAP_TypeDef *dap)
{
    uint32_t tmp = dap_read_data(dap);
    tmp |= dap_read_data(dap) << 8;
    tmp |= dap_read_data(dap) << 16;
    tmp |= dap_read_data(dap) << 24;
    return tmp;
}

void dap_write_data16(DAP_TypeDef *dap, uint16_t data)
{
    dap_write_data(dap, data >> 0);
    dap_write_data(dap, data >> 8);
}

void dap_write_data32(DAP_TypeDef *dap, uint32_t data)
{
    dap_write_data(dap, data >> 0);
    dap_write_data(dap, data >> 8);
    dap_write_data(dap, data >> 16);
    dap_write_data(dap, data >> 24);
}
