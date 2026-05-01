#pragma once

#include <stdint.h>
#include "GOWIN_M1_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPIFLASH_CMD_READ_STATUS_SR1    0x05
#define SPIFLASH_CMD_READ_STATUS_SR2    0x35
#define SPIFLASH_CMD_READ_STATUS_SR3    0x15
#define SPIFLASH_CMD_WRITE_STATUS_SR1    0x01
#define SPIFLASH_CMD_WRITE_STATUS_SR2    0x31
#define SPIFLASH_CMD_WRITE_STATUS_SR3    0x11
#define SPIFLASH_CMD_WRITE_ENABLE      0x06
#define SPIFLASH_CMD_WRITE_DISABLE     0x04
#define SPIFLASH_CMD_READ_UNIQUE_ID    0x4B
#define SPIFLASH_CMD_PAGE_PROGRAM      0x02
#define SPIFLASH_CMD_READ_DATA         0x03
#define SPIFLASH_CMD_ERASE_SECTOR      0x20
#define SPIFLASH_CMD_ERASE_BLOCK_32K   0x52
#define SPIFLASH_CMD_ERASE_BLOCK_64K   0xD8
#define SPIFLASH_CMD_ERASE_CHIP        0xC7

/**
 * @brief 初始化SPIFlash设备
 * @param spi SPI外设指针
 * @param clk_div 时钟分频因子（1-255）
 * @param mode SPI模式（0-3）
 */
void spiflash_init(SPI_TypeDef *spi, uint8_t clk_div, uint8_t mode);

/**
 * @brief 从Flash设备读取唯一ID
 * @param spi SPI外设指针
 * @param id 16字节唯一ID缓存区指针
 */
void spiflash_read_unique_id(SPI_TypeDef *spi, uint8_t *id);

/**
 * @brief 读取指定的状态寄存器
 * @param spi SPI外设指针
 * @param reg_cmd 状态寄存器读取命令
 * @return 状态寄存器的值
 */
uint8_t spiflash_read_status_reg(SPI_TypeDef *spi, uint8_t reg_cmd);

/**
 * @brief 写入指定的状态寄存器
 * @param spi SPI外设指针
 * @param reg_cmd 状态寄存器写入命令
 * @param sr 要写入的状态寄存器值
 */
void spiflash_write_status_reg(SPI_TypeDef *spi, uint8_t reg_cmd, uint8_t sr);

/**
 * @brief 使能Flash设备的写操作
 * @param spi SPI外设指针
 */
void spiflash_write_enable(SPI_TypeDef *spi);

/**
 * @brief 禁止Flash设备的写操作
 * @param spi SPI外设指针
 */
void spiflash_write_disable(SPI_TypeDef *spi);

/**
 * @brief 等待Flash设备就绪（非busy状态）
 * @param spi SPI外设指针
 */
void spiflash_wait_busy(SPI_TypeDef *spi);

/**
 * @brief 擦除指定地址的扇区（4KB）
 * @param spi SPI外设指针
 * @param addr 扇区地址（必须4KB对齐）
 */
void spiflash_erase_sector(SPI_TypeDef *spi, uint32_t addr);

/**
 * @brief 擦除指定地址的32KB块
 * @param spi SPI外设指针
 * @param addr 块地址（必须32KB对齐）
 */
void spiflash_erase_block_32k(SPI_TypeDef *spi, uint32_t addr);

/**
 * @brief 擦除指定地址的64KB块
 * @param spi SPI外设指针
 * @param addr 块地址（必须64KB对齐）
 */
void spiflash_erase_block_64k(SPI_TypeDef *spi, uint32_t addr);

/**
 * @brief 擦除整个Flash芯片
 * @param spi SPI外设指针
 */
void spiflash_erase_chip(SPI_TypeDef *spi);

/**
 * @brief 在指定地址编程一页（最多256字节）
 * @param spi SPI外设指针
 * @param addr 页地址（必须256字节对齐）
 * @param data 数据缓存区指针
 * @param len 要编程的字节数（最大256）
 */
void spiflash_page_program(SPI_TypeDef *spi, uint32_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief 从Flash设备指定地址开始读取数据
 * @param spi SPI外设指针
 * @param addr 读取起始地址
 * @param data 接收数据缓存区指针
 * @param len 要读取的字节数
 */
void spiflash_read_data(SPI_TypeDef *spi, uint32_t addr, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
