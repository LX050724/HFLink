# HFLink

完全使用FPGA实现的高速DAP

## 硬件能力：

|                                 | HFLink Plus        | 敬请期待 |
| ------------------------------- | ------------------ | -------- |
| **硬件规格：**                  |                    |          |
| **Power supply**                | USB                |          |
| **Download speed into RAM**     | 2.9MB/s@60MHz SWD |          |
| **Max. target interface speed** | 60MHz              |          |
| **Max. VCOM speed/baudrate**    | 15 MBd             |          |
| **Supported target voltage**    | 1.2 V - 5 V        |          |
| **Ethernet**                    | ❌️                  |          |
| **USB**                         | ✅️                  |          |
| **WiFi**                        | ❌️                  |          |
| **目标接口：**                  |                    |          |
| **SWD**                         | ✅️                  |          |
| **JTAG**                        | ✅️                  |          |
| **VCOM**                        | ✅️                  |          |

## LED说明：

- 默认模式
  红色：目标电压低
  绿色：目标电压正常
  黄色：复位信号低
  熄灭/不规律闪烁：DAP正在执行命令，熄灭时间表示执行时间
- CMSIS-DAP模式
  绿色：空闲
  蓝色：已连接
  紫色：运行中

## 已完成的功能：

- 虚拟串口：最大15M波特率，支持Break信号
- SWD/JTAG接口：最大60Mhz速度，OpenOCD 2.9MB/s RAM写入速度
- 配置上位机
- 固件升级
- LED功能 

## TODO：

- VTRG电压&5V电流读取 ⬅️正在进行
- SPI ⬅️正在进行
- 线路延迟自标定⬅️正在进行
- SWO：UART编码和曼彻斯特编码
- USB支持多个字符串描述符（需求低）

## 构建&下载方法：

1. 打开高云FPGA工程，调整include路径配置
2. 重新生成M1 IP核，其他IP核不需要生成
3. 综合生成FPGA比特流
4. 编译MCU代码自动生成合并MCU程序的比特流文件`bitstream.bin`
5. 高云下载器设置`Debugging/Temporary Mode` -> `Set ExFlash QE For Arora V`使能Flash的Quad模式（仅需一次）
6. 高云下载器设置`External Flash Mode Arora V` -> `exFlash Erase,Program,Verify Arora V`进行外部Flash下载