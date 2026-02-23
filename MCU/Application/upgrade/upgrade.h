#pragma once


#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

void upgrade_loop(void);
int upgrade_start(uint32_t firm_size);
volatile uint8_t *upgrade_get_buffer(void);
int upgrade_received_data(void);
uint32_t upgrade_get_verify_result(void);
void upgrade_reset(void);

#ifdef __cplusplus
}
#endif
