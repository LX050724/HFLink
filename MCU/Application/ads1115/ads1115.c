#include <GOWIN_M1_i2c.h>
#include <SEGGER_RTT.h>

#define ADS1115_I2C I2C
#define ADS1115_ADDR 0x48

#define ADS1115_CONV_REG 0
#define ADS1115_CONF_REG 1
#define ADS1115_LO_THRESH_REG 2
#define ADS1115_HI_THRESH_REG 3

#define ADS1115_MUX_A0_A1 0
#define ADS1115_MUX_A0_A3 1
#define ADS1115_MUX_A1_A3 2
#define ADS1115_MUX_A2_A3 3
#define ADS1115_MUX_A0_GND 4
#define ADS1115_MUX_A1_GND 5
#define ADS1115_MUX_A2_GND 6
#define ADS1115_MUX_A3_GND 7

#define ADS1115_PGA_6144 0
#define ADS1115_PGA_4096 1
#define ADS1115_PGA_2048 2
#define ADS1115_PGA_1024 3
#define ADS1115_PGA_512 4
#define ADS1115_PGA_256 5

#define ADS1115_MODE_CONTINUOUS 0
#define ADS1115_MODE_SINGLE 1

#define ADS1115_DATA_RATE_8SPS 0
#define ADS1115_DATA_RATE_16SPS 1
#define ADS1115_DATA_RATE_32SPS 2
#define ADS1115_DATA_RATE_64SPS 3
#define ADS1115_DATA_RATE_128SPS 4
#define ADS1115_DATA_RATE_250SPS 5
#define ADS1115_DATA_RATE_475SPS 6
#define ADS1115_DATA_RATE_860SPS 7

#define ADS1115_COMP_MODE_TRADITINOAL 0
#define ADS1115_COMP_MODE_WINDOW 1

#define ADS1115_COMP_POL_LOW 0
#define ADS1115_COMP_POL_HIGH 1

#define ADS1115_COMP_LAT_DISABLE 0
#define ADS1115_COMP_LAT_ENABLE 1

#define ADS1115_COMP_QUE_1 0
#define ADS1115_COMP_QUE_2 1
#define ADS1115_COMP_QUE_4 2
#define ADS1115_COMP_QUE_DISABLE 3

typedef union {
    uint16_t DATA;
    struct {
        uint16_t COMP_QUE : 2;
        uint16_t COMP_LAT : 1;
        uint16_t COMP_POL : 1;
        uint16_t COMP_MODE : 1;
        uint16_t DR : 3;
        uint16_t MODE : 1;
        uint16_t PGA : 3;
        uint16_t MUX : 3;
        uint16_t OS : 1;
    } REG;
} ADS1115_Config_t;

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

void delay_1ms(uint32_t count);

int ads1115_i2c_init(void)
{
    I2C_Init(ADS1115_I2C, 400);

    ADS1115_Config_t conf = {.DATA = 0x8583 };


    while (1) {
        conf.REG.MUX = ADS1115_MUX_A1_GND;
        conf.REG.PGA = ADS1115_PGA_1024;
        conf.REG.OS = 1;
        ads1115_write_reg(ADS1115_CONF_REG, conf.DATA);
    
        do {
            delay_1ms(5);
            conf.DATA = ads1115_read_reg(ADS1115_CONF_REG);
        } while (conf.REG.OS == 0);
    
        uint32_t res = ads1115_read_reg(ADS1115_CONV_REG);
        SEGGER_RTT_printf(0, "%d\n", (res >> 4) - 20);
    }

    return 0;
}
