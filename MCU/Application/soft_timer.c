
#include "soft_timer.h"
#include <cmsis_compiler.h>
#include <stdint.h>


static volatile uint32_t timer_list[TIMER_MAX];

void delay_1ms(uint32_t count)
{
    timer_list[TIMER_DELAY] = count;
    while (timer_list[TIMER_DELAY])
    {
    }
}

void timer_set_tick(TIMER t, uint32_t value)
{
    timer_list[t] = value;
}

uint32_t timer_get_tick(TIMER t)
{
    return timer_list[t];
}

void timer_systick(void)
{
    for (int i = 0; i < TIMER_MAX; i++)
    {
        if (timer_list[i] > 0)
            timer_list[i]--;
    }
}
