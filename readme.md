# HFLink

完全使用FPGA实现的高速DAP

## 硬件能力：

|                                 | HFLink Plus          | 敬请期待 |
| ------------------------------- | -------------------- | -------- |
| **硬件规格：**                  |                      |          |
| **Power supply**                | USB                  |          |
| **Download speed into RAM**     | 1.8MB/s@30MHz SWD    |          |
| **Max. target interface speed** | 30MHz（60MHz待开发） |          |
| **Max. VCOM speed/baudrate**    | 15 MBd               |          |
| **Supported target voltage**    | 1.2 V - 5 V          |          |
| **Ethernet**                    | ❌️                    |          |
| **USB**                         | ✅️                    |          |
| **WiFi**                        | ❌️                    |          |
| **目标接口：**                  |                      |          |
| **SWD**                         | ✅️                    |          |
| **JTAG**                        | 待开发               |          |
| **VCOM**                        | ✅️                    |          |

## 已完成的功能：

- 虚拟串口：最大15M波特率

- SWD接口：最大30Mhz速度，1.8MB/s RAM读取速度
- 固件升级

## TODO：

- 60MHz速度调试 ⬅️正在进行
- 串口Break信号
- USB支持多个字符串描述符
- JTAG
- SWO：UART编码和曼彻斯特编码
- LED功能
- SPI（可能？）

## 构建方法：

1. 打开高云FPGA工程，调整include路径配置
2. 重新生成M1 IP核，其他IP核不需要生成
3. 综合生成FPGA比特流
4. 编译MCU代码自动生成合并MCU程序的比特流文件