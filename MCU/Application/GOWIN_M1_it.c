#include "dap/dap.h"
#include "soft_timer.h"
#include "usb/usbd_core.h"
#include <GOWIN_M1.h>

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
        ;
}

void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
        ;
}

void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
        ;
}

void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
        ;
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    timer_systick();
}

void EXTINT_0_Handler(void)
{
    usbd_irq_handler(USBD);
}

void EXTINT_1_Handler(void)
{
    dap_irq_handler(DAP);
}

void EXTINT_2_Handler()
{

}

void EXTINT_3_Handler()
{

}
