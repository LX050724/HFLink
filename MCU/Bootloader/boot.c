#include <GOWIN_M1_qspi_flash.h>
#include <cmsis_compiler.h>

#define APP_ADDR 0x4D400000
#define ITCM_ADDR 0x00000000

int main(void)
{
    qspi_flash_init();

    uint32_t msp = *(uint32_t *)APP_ADDR;
    uint32_t entry = *(uint32_t *)(APP_ADDR + 4);

    if (msp != 0xffffffff && entry != 0xffffffff)
    {
        // for (uint32_t i = 0; i < 48; i++)
        //     ((uint32_t *)ITCM_ADDR)[i] = ((uint32_t *)APP_ADDR)[i];

        // __disable_irq();
        // __set_MSP(msp);
        // ((void (*)(void))entry)();
    }

    while (1)
    {
    }
}
