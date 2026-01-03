#ifndef GOWIN_M1_DAP_H
#define GOWIN_M1_DAP_H

#include <GOWIN_M1.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

inline static void dap_write_data(DAP_TypeDef *dap, uint8_t data)
{
    *((uint8_t *)&dap->DR) = data;
}

inline static uint8_t dap_read_data(DAP_TypeDef *dap)
{
    return *((uint8_t *)&dap->DR);
}

inline static uint8_t dap_get_current_cmd(DAP_TypeDef *dap)
{
    return dap->CURCMD;
}

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DAP_H */
