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

uint32_t crc32(uint32_t init, uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif
