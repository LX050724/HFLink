#pragma once


#ifdef DEBUG
#include "SEGGER_RTT.h"
#define print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define print(fmt, ...)
#endif

#define LED_RED_PIN GPIO_Pin_0
#define LED_GREEN_PIN GPIO_Pin_1
#define LED_BLUE_PIN GPIO_Pin_2
#define LED_ALL_PIN (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2)

#define POWER_CTL_PIN GPIO_Pin_3
