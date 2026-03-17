#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TIMER_DELAY,
    TIMER_LED,
    TIMER_ADC,
    TIMER_MAX
} TIMER;

void timer_set_tick(TIMER t, uint32_t value);
uint32_t timer_get_tick(TIMER t);
void timer_systick(void);

#ifdef __cplusplus
}
#endif
