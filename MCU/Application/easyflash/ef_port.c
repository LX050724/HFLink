/*
 * This file is part of the EasyFlash Library.
 *
 * Copyright (c) 2015-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-01-16
 */

#include "GOWIN_M1_qspi_flash.h"
#include <easyflash.h>
#include <stdarg.h>
#ifdef PRINT_DEBUG
#include "SEGGER_RTT.h"
#endif

const uint32_t FREQ_MAP_DEFAULT[11] = {
    10000000, 5000000, 2000000, 1000000, 500000, 200000, 100000, 50000, 20000, 10000, 5000,
};
const uint8_t IODELAY_DEFAULT[6] = {60};

static const ef_env default_env_set[] = {
    {"5V_EN", "1", 1},
    {"INDEP_UART", "1", 1},
    {"FREQ_MAP_EN", "0", 1},
    {"FREQ_MAP", (void *)FREQ_MAP_DEFAULT, sizeof(FREQ_MAP_DEFAULT)},
    {"IODELAY", (void *)IODELAY_DEFAULT, sizeof(IODELAY_DEFAULT)},
    {"LEDMODE", (uint8_t []){0x00}, 1},
};

/**
 * Flash port for hardware initialize.
 *
 * @param default_env default ENV set for user
 * @param default_env_size default ENV size
 *
 * @return result
 */
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size)
{
    EfErrCode result = EF_NO_ERR;

    *default_env = default_env_set;
    *default_env_size = sizeof(default_env_set) / sizeof(default_env_set[0]);

    return result;
}

/**
 * Read data from flash.
 * @note This operation's units is word.
 *
 * @param addr flash address
 * @param buf buffer to store read data
 * @param size read bytes size
 *
 * @return result
 */
EfErrCode ef_port_read(uint32_t addr, uint32_t *buf, size_t size)
{
    EfErrCode result = EF_NO_ERR;

    /* You can add your code under here. */
    qspi_flash_fast_read_quad(addr, (uint8_t *)buf, size);
    return result;
}

/**
 * Erase data on flash.
 * @note This operation is irreversible.
 * @note This operation's units is different which on many chips.
 *
 * @param addr flash address
 * @param size erase bytes size
 *
 * @return result
 */
EfErrCode ef_port_erase(uint32_t addr, size_t size)
{
    EfErrCode result = EF_NO_ERR;

    /* make sure the start address is a multiple of EF_ERASE_MIN_SIZE */
    EF_ASSERT(addr % EF_ERASE_MIN_SIZE == 0);
    while (size)
    {
        qspi_flash_write_enable();
        qspi_flash_4ksector_erase(addr);
        while (qspi_flash_is_busy())
        {
        }
        addr += EF_ERASE_MIN_SIZE;
        size -= EF_ERASE_MIN_SIZE;
    }
    /* You can add your code under here. */

    return result;
}
/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size)
{
    EfErrCode result = EF_NO_ERR;

    /* You can add your code under here. */
    while (size) {
        size_t wsize = size > 256 ? 256 : size;
        qspi_flash_write_enable();
        qspi_flash_page_program(size, addr, (uint8_t *)buf);
        while (qspi_flash_is_busy())
        {
        }
        addr += wsize;
        size -= wsize;
    }
    return result;
}

static uint32_t lock_status;

/**
 * lock the ENV ram cache
 */
void ef_port_env_lock(void)
{
    if (lock_status == 0)
        __disable_irq();
    lock_status = (lock_status << 1) | 1;
}

/**
 * unlock the ENV ram cache
 */
void ef_port_env_unlock(void)
{
    lock_status = lock_status >> 1;
    if (lock_status == 0)
        __enable_irq();
}

/**
 * This function is print flash debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 *
 */
void ef_log_debug(const char *file, const long line, const char *format, ...)
{
    (void)file;
    (void)line;
#ifdef PRINT_DEBUG
    va_list args;
    va_start(args, format);
    SEGGER_RTT_vprintf(0, format, &args);
    va_end(args);
#else
    (void)format;
#endif
}

/**
 * This function is print flash routine info.
 *
 * @param format output format
 * @param ... args
 */
void ef_log_info(const char *format, ...)
{
#ifdef PRINT_DEBUG
    va_list args;
    va_start(args, format);
    SEGGER_RTT_vprintf(0, format, &args);
    va_end(args);
#else
    (void)format;
#endif
}
/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void ef_print(const char *format, ...)
{
#ifdef PRINT_DEBUG
    va_list args;
    va_start(args, format);
    SEGGER_RTT_vprintf(0, format, &args);
    va_end(args);
#else
    (void)format;
#endif
}
