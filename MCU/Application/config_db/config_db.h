#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t magic;       // 0x12345678
    uint32_t version;     // 0x00000001
    uint32_t crc32;       // crc32 of the whole struct with this field set to 
    uint32_t clock_freq_mapping[11];
    uint8_t nickname[32];
    uint8_t iodelay_param[6];
    uint8_t led_mode;
    uint8_t freq_mapping_enable : 1;
    uint8_t supply5V_enable : 1;
    uint8_t indep_uart_enable : 1;
} ConfigData_t;

extern ConfigData_t global_config;

int config_data_load(void);
int config_data_save(void);

#ifdef __cplusplus
}
#endif
