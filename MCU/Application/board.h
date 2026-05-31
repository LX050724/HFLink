#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DEBUG
#include "SEGGER_RTT.h"
#define print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define print(fmt, ...)
#endif

extern uint8_t shared_buffer[256];

uint32_t crc32(uint32_t init, uint8_t *data, uint32_t length);

uint16_t get_5v_supply_current_ma(void);
uint16_t get_usb_voltage_mv(void);
uint16_t get_vtrg_voltage_mv(void);

#ifdef __cplusplus
}
#endif
