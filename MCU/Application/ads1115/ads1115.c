#include "ads1115.h"
#include <GOWIN_M1_i2c.h>

#define ADS1115_I2C I2C
#define ADS1115_ADDR 0x48



static int ads1115_write_reg(uint8_t addr, uint16_t data)
{
    data = __REVSH(data);
    I2C_SendData(ADS1115_I2C, ADS1115_ADDR, addr, (uint8_t *)&data, 2);
    return 0;
}

static uint16_t ads1115_read_reg(uint8_t addr)
{
    uint16_t data;
    I2C_ReceiveData(ADS1115_I2C, ADS1115_ADDR, addr, (uint8_t *)&data, 2);
    return __REVSH(data);
}


int ads1115_start_conv(uint8_t pga, uint8_t channel)
{
    ADS1115_Config_t conf = {.DATA = 0x8583 };
    conf.REG.MUX = channel;
    conf.REG.PGA = pga;
    conf.REG.OS = 1;
    return ads1115_write_reg(ADS1115_CONF_REG, conf.DATA);
}

int ads1115_is_busy(void)
{
    ADS1115_Config_t conf;
    conf.DATA = ads1115_read_reg(ADS1115_CONF_REG);
    return (conf.REG.OS == 0);
}

int16_t ads1115_read_result(void)
{
    return ads1115_read_reg(ADS1115_CONV_REG);
}

int ads1115_i2c_init(void)
{
    I2C_Init(ADS1115_I2C, 400);
    return 0;
}
