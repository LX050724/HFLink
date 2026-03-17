#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// ADS1115 寄存器地址定义
#define ADS1115_CONV_REG 0          /**< 转换结果寄存器 */
#define ADS1115_CONF_REG 1          /**< 配置寄存器 */
#define ADS1115_LO_THRESH_REG 2     /**< 低阈值寄存器 */
#define ADS1115_HI_THRESH_REG 3     /**< 高阈值寄存器 */

// 多路复用器设置
#define ADS1115_MUX_A0_A1 0         /**< A0-A1 差分输入 */
#define ADS1115_MUX_A0_A3 1         /**< A0-A3 差分输入 */
#define ADS1115_MUX_A1_A3 2         /**< A1-A3 差分输入 */
#define ADS1115_MUX_A2_A3 3         /**< A2-A3 差分输入 */
#define ADS1115_MUX_A0_GND 4        /**< A0-GND 单端输入 */
#define ADS1115_MUX_A1_GND 5        /**< A1-GND 单端输入 */
#define ADS1115_MUX_A2_GND 6        /**< A2-GND 单端输入 */
#define ADS1115_MUX_A3_GND 7        /**< A3-GND 单端输入 */

// 可编程增益放大器设置
#define ADS1115_PGA_6144 0          /**< ±6.144V 范围 */
#define ADS1115_PGA_4096 1          /**< ±4.096V 范围 */
#define ADS1115_PGA_2048 2          /**< ±2.048V 范围 */
#define ADS1115_PGA_1024 3          /**< ±1.024V 范围 */
#define ADS1115_PGA_512 4           /**< ±0.512V 范围 */
#define ADS1115_PGA_256 5           /**< ±0.256V 范围 */

// 转换模式
#define ADS1115_MODE_CONTINUOUS 0   /**< 连续转换模式 */
#define ADS1115_MODE_SINGLE 1       /**< 单次转换模式 */

// 数据速率设置
#define ADS1115_DATA_RATE_8SPS 0    /**< 8 SPS */
#define ADS1115_DATA_RATE_16SPS 1   /**< 16 SPS */
#define ADS1115_DATA_RATE_32SPS 2   /**< 32 SPS */
#define ADS1115_DATA_RATE_64SPS 3   /**< 64 SPS */
#define ADS1115_DATA_RATE_128SPS 4  /**< 128 SPS */
#define ADS1115_DATA_RATE_250SPS 5  /**< 250 SPS */
#define ADS1115_DATA_RATE_475SPS 6  /**< 475 SPS */
#define ADS1115_DATA_RATE_860SPS 7  /**< 860 SPS */

// 比较器模式
#define ADS1115_COMP_MODE_TRADITINOAL 0  /**< 传统比较器模式 */
#define ADS1115_COMP_MODE_WINDOW 1        /**< 窗口比较器模式 */

// 比较器极性
#define ADS1115_COMP_POL_LOW 0       /**< 低电平有效 */
#define ADS1115_COMP_POL_HIGH 1      /**< 高电平有效 */

// 比较器锁存
#define ADS1115_COMP_LAT_DISABLE 0   /**< 禁用锁存 */
#define ADS1115_COMP_LAT_ENABLE 1    /**< 启用锁存 */

// 比较器队列
#define ADS1115_COMP_QUE_1 0         /**< 1 次转换后断言 */
#define ADS1115_COMP_QUE_2 1         /**< 2 次转换后断言 */
#define ADS1115_COMP_QUE_4 2         /**< 4 次转换后断言 */
#define ADS1115_COMP_QUE_DISABLE 3   /**< 禁用比较器 */


/**
 * @brief ADS1115 配置寄存器联合体，用于配置和读取配置数据
 */
typedef union {
    uint16_t DATA;
    struct {
        uint16_t COMP_QUE : 2;   /**< 比较器队列设置 */
        uint16_t COMP_LAT : 1;   /**< 比较器锁存 */
        uint16_t COMP_POL : 1;   /**< 比较器极性 */
        uint16_t COMP_MODE : 1;  /**< 比较器模式 */
        uint16_t DR : 3;         /**< 数据速率 */
        uint16_t MODE : 1;       /**< 转换模式 */
        uint16_t PGA : 3;        /**< 可编程增益放大器 */
        uint16_t MUX : 3;        /**< 多路复用器设置 */
        uint16_t OS : 1;         /**< 操作状态 */
    } REG;
} ADS1115_Config_t;

/**
 * @brief 初始化 ADS1115 的 I2C 接口
 * 
 * @return int 返回 0 表示成功，非 0 表示失败
 */
int ads1115_i2c_init(void);

/**
 * @brief 开始 ADS1115 的模数转换
 * 
 * @param pga 增益设置，使用 ADS1115_PGA_* 宏定义
 * @param channel 通道选择，使用 ADS1115_MUX_* 宏定义
 * @return int 返回 0 表示成功，非 0 表示失败
 */
int ads1115_start_conv(uint8_t pga, uint8_t channel);

/**
 * @brief 检查 ADS1115 是否正在进行转换
 * 
 * @return int 返回 1 表示忙碌，0 表示空闲
 */
int ads1115_is_busy(void);

/**
 * @brief 读取 ADS1115 的转换结果
 * 
 * @return uint16_t 16 位转换结果
 */
int16_t ads1115_read_result(void);

#ifdef __cplusplus
}
#endif
