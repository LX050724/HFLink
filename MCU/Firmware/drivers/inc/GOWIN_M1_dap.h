#ifndef GOWIN_M1_DAP_H
#define GOWIN_M1_DAP_H

#include <GOWIN_M1.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_GPIO_STATUS_SRST_O 0x0001
#define DAP_GPIO_STATUS_SRST_I 0x0002
#define DAP_GPIO_STATUS_TRST_O 0x0004
#define DAP_GPIO_STATUS_TRST_I 0x0008
#define DAP_GPIO_STATUS_TDI_O 0x0010
#define DAP_GPIO_STATUS_SWO_TDO_I 0x0020
#define DAP_GPIO_STATUS_SWDIO_TMS_I 0x0040
#define DAP_GPIO_STATUS_SWDIO_TMS_O 0x0080
#define DAP_GPIO_STATUS_SWDIO_TMS_T 0x0100
#define DAP_GPIO_STATUS_SWCLK_TCK_O 0x0200

#define DAP_GPIO_DO_SWCLK_TCK_O 0x01
#define DAP_GPIO_DO_SWDIO_TMS_T 0x02
#define DAP_GPIO_DO_SWDIO_TMS_O 0x04
#define DAP_GPIO_DO_TDI_O 0x08
#define DAP_GPIO_DO_TRST_O 0x10
#define DAP_GPIO_DO_SRST_O 0x20
#define DAP_GPIO_DO_POWER_CTL_O 0x80

#define DAP_GPIO_CR_INDEPENDENT_UART_MASK 0x01
#define DAP_GPIO_CR_DIRECTIO_MASK 0x02
#define DAP_GPIO_CR_SPI_MODE_MASK 0x04
#define DAP_GPIO_CR_LED_MODE_MASK 0x08

#define DAP_GPIO_CR_CALIB_MASK 0x10
#define DAP_GPIO_CR_CALIB_SELECT_MASK 0x60
#define DAP_GPIO_CR_CALIB_SELECT_CLK 0x00
#define DAP_GPIO_CR_CALIB_SELECT_TMS 0x20
#define DAP_GPIO_CR_CALIB_SELECT_TDO 0x40

#define DAP_GPIO_LED_R 0
#define DAP_GPIO_LED_G 1
#define DAP_GPIO_LED_B 2

#define DAP_GPIO_CR_LED_MODE_MANUAL 0x00
#define DAP_GPIO_CR_LED_MODE_DEFAULT DAP_GPIO_CR_LED_MODE_MASK

#define DAP_SWJ_MODE_SWD 1
#define DAP_SWJ_MODE_JTAG 0

#define DAP_SWO_CR_EN_MASK            0x0001
#define DAP_SWO_CR_MODE_MASK          0x0002
#define DAP_SWO_CR_JITTER_Pos            2
#define DAP_SWO_CR_JITTER_MASK           (0xf << DAP_SWO_CR_JITTER_Pos)
#define DAP_SWO_CR_EDGE_Pos              8
#define DAP_SWO_CR_EDGE_MASK             (0xf << DAP_SWO_CR_EDGE_Pos)

#define DAP_SWO_CR_MODE_UART             0
#define DAP_SWO_CR_MODE_MANCHESTER       2

/** @brief 向 DAP 数据寄存器写入单个字节。
 *  @param dap DAP 实例指针。
 *  @param data 要写入的 8 位数据。
 */
inline static void dap_write_data(DAP_TypeDef *dap, uint8_t data)
{
    dap->DR.WD = data;
}

/** @brief 向 DAP 数据寄存器写入 16 位值。
 *  @param dap  DAP 实例指针。
 *  @param data 要写入的 16 位数据（低位在前）。
 */
void dap_write_data16(DAP_TypeDef *dap, uint16_t data);

/** @brief 向 DAP 数据寄存器写入 32 位值。
 *  @param dap  DAP 实例指针。
 *  @param data 要写入的 32 位数据（低位在前）。
 */
void dap_write_data32(DAP_TypeDef *dap, uint32_t data);

/** @brief 从 DAP 数据寄存器读取单个字节（阻塞）。
 *  @param dap DAP 实例指针。
 *  @return 读取到的 8 位数据。
 */
uint8_t dap_read_data(DAP_TypeDef *dap);

/** @brief 从 DAP 数据寄存器读取 16 位值（阻塞）。
 *  @param dap DAP 实例指针。
 *  @return 读取到的 16 位数据（低位在前）。
 */
uint16_t dap_read_data16(DAP_TypeDef *dap);

/** @brief 从 DAP 数据寄存器读取 32 位值（阻塞）。
 *  @param dap DAP 实例指针。
 *  @return 读取到的 32 位数据（低位在前）。
 */
uint32_t dap_read_data32(DAP_TypeDef *dap);

/** @brief 获取当前时间戳计数器值。
 *  @param dap DAP 实例指针。
 *  @return 当前时间戳值。
 */
inline static uint32_t dap_timestamp_get(DAP_TypeDef *dap)
{
    return dap->TIMESTAMP;
}

/** @brief 将时间戳计数器清零。
 *  @param dap DAP 实例指针。
 */
inline static void dap_timestamp_reset(DAP_TypeDef *dap)
{
    dap->TIMESTAMP = 0;
}

/** @brief 获取当前正在处理的命令 ID。
 *  @param dap DAP 实例指针。
 *  @return 当前命令 ID。
 */
inline static uint8_t dap_get_current_cmd(DAP_TypeDef *dap)
{
    return dap->CURCMD;
}

/*************************************************************************************************************************/

/** @brief 启动波特率发生器。
 *  @param dap DAP 实例指针。
 */
inline static void dap_baud_start(DAP_TypeDef *dap)
{
    dap->BAUD_GEN.CR |= 1;
}

/** @brief 停止波特率发生器。
 *  @param dap DAP 实例指针。
 */
inline static void dap_baud_stop(DAP_TypeDef *dap)
{
    dap->BAUD_GEN.CR &= ~1U;
}

/** @brief 设置波特率发生器重载（分频）值。
 *  @param dap DAP 实例指针。
 *  @param div 重载/分频值。
 */
inline static void dap_baud_set_reload(DAP_TypeDef *dap, uint16_t div)
{
    dap->BAUD_GEN.RELOAD = div;
}

/** @brief 获取当前波特率发生器重载（分频）值。
 *  @param dap DAP 实例指针。
 *  @return 当前重载/分频值。
 */
inline static uint16_t dap_baud_get_reload(DAP_TypeDef *dap)
{
    return dap->BAUD_GEN.RELOAD;
}

/** @brief 设置波特率发生器采样比较值。
 *  @param dap DAP 实例指针。
 *  @param cmp 采样比较值。
 */
inline static void dap_baud_set_simpling_cmp(DAP_TypeDef *dap, uint16_t cmp)
{
    dap->BAUD_GEN.SIMPLING_CMP = cmp;
}

/** @brief 设置波特率发生器采样延迟。
 *  @param dap DAP 实例指针。
 *  @param cmp 采样延迟值（写入 CR 的高 16 位）。
 */
inline static void dap_buad_set_simpling_delay(DAP_TypeDef *dap, uint8_t cmp)
{
    dap->BAUD_GEN.CR = (dap->BAUD_GEN.CR & 0x0000ffff) | (cmp << 16);
}

/*************************************************************************************************************************/

/** @brief 设置 SWJ 接口模式（SWD 或 JTAG）。
 *  @param dap  DAP 实例指针。
 *  @param mode SWJ 模式：#DAP_SWJ_MODE_SWD 或 #DAP_SWJ_MODE_JTAG。
 */
inline static void dap_swj_set_mode(DAP_TypeDef *dap, uint8_t mode)
{
    dap->SWJ.CR = mode;
}

/** @brief 设置 SWJ WAIT 重试次数。
 *  @param dap DAP 实例指针。
 *  @param cnt WAIT 重试次数。
 */
inline static void dap_swj_set_wait_retry(DAP_TypeDef *dap, uint16_t cnt)
{
    dap->SWJ.WAIT_RETRY = cnt;
}

/** @brief 设置 SWJ MATCH 重试次数。
 *  @param dap DAP 实例指针。
 *  @param cnt MATCH 重试次数。
 */
inline static void dap_swj_set_match_retry(DAP_TypeDef *dap, uint16_t cnt)
{
    dap->SWJ.MATCH_RETRY = cnt;
}

/** @brief 使能 SWD Turnaround 时钟输出。
 *  @param dap DAP 实例指针。
 */
inline static void dap_swd_enable_turn_clk(DAP_TypeDef *dap)
{
    dap->SWJ.SWD_CR |= 0x8;
}

/** @brief 禁用 SWD Turnaround 时钟输出。
 *  @param dap DAP 实例指针。
 */
inline static void dap_swd_disable_turn_clk(DAP_TypeDef *dap)
{
    dap->SWJ.SWD_CR &= ~0x8;
}

/*************************************************************************************************************************/

/** @brief 设置 SWD Turnaround 周期数。
 *  @param dap        DAP 实例指针。
 *  @param turn_cycle Turnaround 周期数（0~3）。
 */
inline static void dap_swd_set_trun_cycle(DAP_TypeDef *dap, uint8_t turn_cycle)
{
    dap->SWJ.SWD_CR = (dap->SWJ.SWD_CR & ~0x03) | (turn_cycle & 0x03);
}

/** @brief 设置 SWD 数据相位（单比特/多比特）。
 *  @param dap        DAP 实例指针。
 *  @param data_phase 数据相位设置（0: 单比特, 1: 多比特）。
 */
inline static void dap_swd_set_data_phase(DAP_TypeDef *dap, uint8_t data_phase)
{
    dap->SWJ.SWD_CR = (dap->SWJ.SWD_CR & ~0x04) | (data_phase << 2);
}

/** @brief 直接设置 SWD Turnaround 与数据相位配置。
 *  @param dap   DAP 实例指针。
 *  @param value 写入 SWD CR 寄存器的原始值。
 */
inline static void dap_swd_set_trun_data_phase(DAP_TypeDef *dap, uint8_t value)
{
    dap->SWJ.SWD_CR = value;
}

/*************************************************************************************************************************/

/** @brief 使能 GPIO 直连 I/O 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_enable_directio(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_DIRECTIO_MASK;
}

/** @brief 禁用 GPIO 直连 I/O 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_disable_directio(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_DIRECTIO_MASK;
}

/** @brief 使能 GPIO 独立 UART 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_enable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_INDEPENDENT_UART_MASK;
}

/** @brief 禁用 GPIO 独立 UART 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_disable_independent_uart(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_INDEPENDENT_UART_MASK;
}

/** @brief 使能 GPIO SPI 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_enable_spi_mode(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_SPI_MODE_MASK;
}

/** @brief 禁用 GPIO SPI 模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_disable_spi_mode(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_SPI_MODE_MASK;
}

/** @brief 检测 GPIO SPI 模式是否已使能。
 *  @param dap DAP 实例指针。
 *  @return true 表示 SPI 模式已使能，false 表示未使能。
 */
inline static bool dap_gpio_is_spi_mode(DAP_TypeDef *dap)
{
    return (dap->GPIO.CR & DAP_GPIO_CR_SPI_MODE_MASK) != 0;
}

/** @brief 设置 GPIO LED 控制模式。
 *  @param dap  DAP 实例指针。
 *  @param mode LED 模式：#DAP_GPIO_CR_LED_MODE_MANUAL 或 #DAP_GPIO_CR_LED_MODE_DEFAULT。
 */
inline static void dap_gpio_set_led_mode(DAP_TypeDef *dap, uint8_t mode)
{
    dap->GPIO.CR = (dap->GPIO.CR & ~DAP_GPIO_CR_LED_MODE_MASK) | mode;
}

/** @brief 设置 TCK 输出延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TCK 输出延迟值。
 */
inline static void dap_gpio_set_tck_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TCK_DELAY = delay;
}

/** @brief 设置 TMS 输入延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TMS 输入延迟值。
 */
inline static void dap_gpio_set_tms_idelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_I_DELAY = delay;
}

/** @brief 设置 TMS 输出延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TMS 输出延迟值。
 */
inline static void dap_gpio_set_tms_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_O_DELAY = delay;
}

/** @brief 设置 TMS 三态输出延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TMS 三态输出延迟值。
 */
inline static void dap_gpio_set_tms_t_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TMS_T_DELAY = delay;
}

/** @brief 设置 TDO 输入延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TDO 输入延迟值。
 */
inline static void dap_gpio_set_tdo_idelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TDO_DELAY = delay;
}

/** @brief 设置 TDI 输出延迟。
 *  @param dap   DAP 实例指针。
 *  @param delay TDI 输出延迟值。
 */
inline static void dap_gpio_set_tdi_odelay(DAP_TypeDef *dap, uint8_t delay)
{
    dap->GPIO.TDI_DELAY = delay;
}

/** @brief 设置 RGB LED PWM 比较值。
 *  @param dap DAP 实例指针。
 *  @param R   红色分量 PWM 值。
 *  @param G   绿色分量 PWM 值。
 *  @param B   蓝色分量 PWM 值。
 */
inline static void dap_gpio_set_led_cmp(DAP_TypeDef *dap, uint8_t R, uint8_t G, uint8_t B)
{
    dap->GPIO.LED_CMP[0] = R;
    dap->GPIO.LED_CMP[1] = G;
    dap->GPIO.LED_CMP[2] = B;
}

/** @brief 置位（拉高）指定的 GPIO 直连 I/O 引脚。
 *  @param dap      DAP 实例指针。
 *  @param pin_mask 要置位的引脚位掩码。
 */
inline static void dap_gpio_set_pin(DAP_TypeDef *dap, uint8_t pin_mask)
{
    dap->GPIO.DIRECTIO_SET = pin_mask;
}

/** @brief 复位（拉低）指定的 GPIO 直连 I/O 引脚。
 *  @param dap      DAP 实例指针。
 *  @param pin_mask 要复位的引脚位掩码。
 */
inline static void dap_gpio_reset_pin(DAP_TypeDef *dap, uint8_t pin_mask)
{
    dap->GPIO.DIRECTIO_RESET = pin_mask;
}

/** @brief 向指定的 GPIO 直连 I/O 引脚写入值。
 *  @param dap      DAP 实例指针。
 *  @param pin_mask 要写入的引脚位掩码。
 *  @param value    非零表示置位，零表示复位。
 *  @return 写入的值。
 */
inline static uint8_t dap_gpio_write_pin(DAP_TypeDef *dap, uint8_t pin_mask, uint8_t value)
{
    if (value)
    {
        dap_gpio_set_pin(dap, pin_mask);
    }
    else
    {
        dap_gpio_reset_pin(dap, pin_mask);
    }
    return value;
}

/** @brief 获取当前 GPIO 状态寄存器的值。
 *  @param dap DAP 实例指针。
 *  @return GPIO 状态寄存器值。
 */
inline static uint16_t dap_gpio_get_status(DAP_TypeDef *dap)
{
    return dap->GPIO.GPIO_STATUS;
}

/** @brief 获取 GPIO 校准采样结果。
 *  @param dap DAP 实例指针。
 *  @return 校准采样值。
 */
inline static uint16_t dap_gpio_get_cali_simpling(DAP_TypeDef *dap)
{
    return dap->GPIO.CALI_SIMPLING;
}

/** @brief 使能 GPIO 校准采样模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_enable_calib_simpling(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_CALIB_MASK;
}

/** @brief 禁用 GPIO 校准采样模式。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_disable_calib_simpling(DAP_TypeDef *dap)
{
    dap->GPIO.CR &= ~DAP_GPIO_CR_CALIB_MASK;
}

/** @brief 在 GPIO 上发送 Break 条件（UART Break）。
 *  @param dap DAP 实例指针。
 */
inline static void dap_gpio_send_break(DAP_TypeDef *dap)
{
    dap->GPIO.CR |= DAP_GPIO_CR_CALIB_MASK;
}

/** @brief 设置 GPIO 校准选择源。
 *  @param dap    DAP 实例指针。
 *  @param select 校准源：#DAP_GPIO_CR_CALIB_SELECT_CLK、
 *                #DAP_GPIO_CR_CALIB_SELECT_TMS 或 #DAP_GPIO_CR_CALIB_SELECT_TDO。
 */
inline static void dap_gpio_set_calib_select(DAP_TypeDef *dap, uint8_t select)
{
    dap->GPIO.CR = (dap->GPIO.CR & ~DAP_GPIO_CR_CALIB_SELECT_MASK) | select;
}
/*************************************************************************************************************************/

/** @brief 设置指定 TAP 索引的 JTAG IR 长度。
 *  @param dap   DAP 实例指针。
 *  @param index JTAG TAP 索引（0~7）。
 *  @param irlen IR 长度（以 bit 为单位）。
 */
inline static void dap_jtag_set_irlen(DAP_TypeDef *dap, uint8_t index, uint8_t irlen)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_LEN = irlen;
}

/** @brief 获取指定 TAP 索引的 JTAG IR 长度。
 *  @param dap   DAP 实例指针。
 *  @param index JTAG TAP 索引（0~7）。
 *  @return IR 长度（以 bit 为单位）。
 */
inline static uint8_t dap_jtag_get_irlen(DAP_TypeDef *dap, uint8_t index)
{
    return dap->SWJ.JTAG_IR_CONF[index].IR_LEN;
}

/** @brief 设置指定 TAP 的 JTAG IR 前导长度（前导 bypass 位）。
 *  @param dap        DAP 实例指针。
 *  @param index      JTAG TAP 索引（0~7）。
 *  @param before_len 该 TAP IR 之前的前导 bypass 位数。
 */
inline static void dap_jtag_set_ir_before_len(DAP_TypeDef *dap, uint8_t index, uint8_t before_len)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_BEFORE_LEN = before_len;
}

/** @brief 设置指定 TAP 的 JTAG IR 后继长度（后继 bypass 位）。
 *  @param dap       DAP 实例指针。
 *  @param index     JTAG TAP 索引（0~7）。
 *  @param after_len 该 TAP IR 之后的后继 bypass 位数。
 */
inline static void dap_jtag_set_ir_after_len(DAP_TypeDef *dap, uint8_t index, uint8_t after_len)
{
    dap->SWJ.JTAG_IR_CONF[index].IR_AFTER_LEN = after_len;
}

/** @brief 设置 JTAG 扫描链上 TAP 设备的数量。
 *  @param dap DAP 实例指针。
 *  @param num TAP 设备数量减 1（0~15）。
 */
inline static void dap_jtag_set_tap_num(DAP_TypeDef *dap, uint8_t num)
{
    dap->SWJ.JTAG_CR = (dap->SWJ.JTAG_CR & ~0x0F) | num;
}

/*************************************************************************************************************************/

/** @brief 使能 SWO 跟踪捕获。
 *  @param dap DAP 实例指针。
 */
inline static void dap_swo_enable(DAP_TypeDef *dap)
{
    dap->SWO.CR |= DAP_SWO_CR_EN_MASK;
}

/** @brief 禁用 SWO 跟踪捕获。
 *  @param dap DAP 实例指针。
 */
inline static void dap_swo_disable(DAP_TypeDef *dap)
{
    dap->SWO.CR &= ~DAP_SWO_CR_EN_MASK;
}

/** @brief 检测 SWO 跟踪捕获是否已使能。
 *  @param dap DAP 实例指针。
 *  @return true 表示 SWO 已使能，false 表示未使能。
 */
inline static bool dap_swo_is_enabled(DAP_TypeDef *dap)
{
    return (dap->SWO.CR & DAP_SWO_CR_EN_MASK) != 0;
}

/** @brief 设置 SWO 跟踪模式（UART 或 Manchester）。
 *  @param dap  DAP 实例指针。
 *  @param mode SWO 模式：#DAP_SWO_MODE_UART 或 #DAP_SWO_MODE_MANCHESTER。
 */
inline static void dap_swo_set_mode(DAP_TypeDef *dap, uint8_t mode)
{
    dap->SWO.CR = (dap->SWO.CR & ~DAP_SWO_CR_MODE_MASK) | (mode & DAP_SWO_CR_MODE_MASK);
}

/** @brief 获取当前 SWO 跟踪模式。
 *  @param dap DAP 实例指针。
 *  @return 当前 SWO 模式：#DAP_SWO_MODE_UART (0) 或 #DAP_SWO_MODE_MANCHESTER (2)。
 */
inline static uint8_t dap_swo_get_mode(DAP_TypeDef *dap)
{
    return dap->SWO.CR & DAP_SWO_CR_MODE_MASK;
}

/** @brief 设置 SWO 抖动补偿值。
 *  @param dap    DAP 实例指针。
 *  @param jitter 抖动补偿值（0~15）。
 */
inline static void dap_swo_set_jitter(DAP_TypeDef *dap, uint8_t jitter)
{
    dap->SWO.CR = (dap->SWO.CR & ~DAP_SWO_CR_JITTER_MASK) | ((jitter << DAP_SWO_CR_JITTER_Pos) & DAP_SWO_CR_JITTER_MASK);
}

/** @brief 获取当前 SWO 抖动补偿值。
 *  @param dap DAP 实例指针。
 *  @return 抖动补偿值（0~15）。
 */
inline static uint8_t dap_swo_get_jitter(DAP_TypeDef *dap)
{
    return (dap->SWO.CR & DAP_SWO_CR_JITTER_MASK) >> DAP_SWO_CR_JITTER_Pos;
}

/** @brief 设置 SWO 过采样边沿检测宽度。
 *  @param dap  DAP 实例指针。
 *  @param edge 边沿检测宽度（0~15）。
 */
inline static void dap_swo_set_edge(DAP_TypeDef *dap, uint8_t edge)
{
    dap->SWO.CR = (dap->SWO.CR & ~DAP_SWO_CR_EDGE_MASK) | ((edge << DAP_SWO_CR_EDGE_Pos) & DAP_SWO_CR_EDGE_MASK);
}

/** @brief 获取当前 SWO 过采样边沿检测宽度。
 *  @param dap DAP 实例指针。
 *  @return 边沿检测宽度（0~15）。
 */
inline static uint8_t dap_swo_get_edge(DAP_TypeDef *dap)
{
    return (dap->SWO.CR & DAP_SWO_CR_EDGE_MASK) >> DAP_SWO_CR_EDGE_Pos;
}

/** @brief 设置 SWO 位时间 0 寄存器。
 *  @param dap DAP 实例指针。
 *  @param bt  位时间 0 的值。
 */
inline static void dap_swo_set_bit_time0(DAP_TypeDef *dap, uint16_t bt)
{
    dap->SWO.BIT_TIME0 = bt;
}

/** @brief 设置 SWO 位时间 1 寄存器。
 *  @param dap DAP 实例指针。
 *  @param bt  位时间 1 的值。
 */
inline static void dap_swo_set_bit_time1(DAP_TypeDef *dap, uint16_t bt)
{
    dap->SWO.BIT_TIME1 = bt;
}

/** @brief 获取当前 SWO 位时间 0 寄存器的值。
 *  @param dap DAP 实例指针。
 *  @return 位时间 0 的值。
 */
inline static uint16_t dap_swo_get_bit_time0(DAP_TypeDef *dap)
{
    return dap->SWO.BIT_TIME0;
}

/** @brief 获取当前 SWO 位时间 1 寄存器的值。
 *  @param dap DAP 实例指针。
 *  @return 位时间 1 的值。
 */
inline static uint16_t dap_swo_get_bit_time1(DAP_TypeDef *dap)
{
    return dap->SWO.BIT_TIME1;
}

/** @brief 设置 SWO 位判决高阈值。
 *  @param dap DAP 实例指针。
 *  @param val 高阈值。
 */
inline static void dap_swo_set_bit_decision_high(DAP_TypeDef *dap, uint16_t val)
{
    dap->SWO.BIT_DECISION_HIGH = val;
}

/** @brief 设置 SWO 位判决低阈值。
 *  @param dap DAP 实例指针。
 *  @param val 低阈值。
 */
inline static void dap_swo_set_bit_decision_low(DAP_TypeDef *dap, uint16_t val)
{
    dap->SWO.BIT_DECISION_LOW = val;
}

/** @brief 获取当前 SWO 位判决高阈值。
 *  @param dap DAP 实例指针。
 *  @return 高阈值。
 */
inline static uint16_t dap_swo_get_bit_decision_high(DAP_TypeDef *dap)
{
    return dap->SWO.BIT_DECISION_HIGH;
}

/** @brief 获取当前 SWO 位判决低阈值。
 *  @param dap DAP 实例指针。
 *  @return 低阈值。
 */
inline static uint16_t dap_swo_get_bit_decision_low(DAP_TypeDef *dap)
{
    return dap->SWO.BIT_DECISION_LOW;
}

/*************************************************************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DAP_H */
