from cmath import rect
import re
from typing import List
import usb.core
import asyncio
from io import BytesIO

DAP_PORT_DEFAULT=0
DAP_PORT_SWD=1
DAP_PORT_JTAG=2

DAP_TRANS_AP = 1
DAP_TRANS_DP = 0
DAP_TRANS_READ = 2
DAP_TRANS_WRITE = 0
DAP_TRANS_A0 = 0
DAP_TRANS_A4 = 4
DAP_TRANS_A8 = 8
DAP_TRANS_AC = 12

DAP_TRANS_R_IDCODE = DAP_TRANS_READ | DAP_TRANS_DP | DAP_TRANS_A0
DAP_TRANS_W_SELECT = DAP_TRANS_WRITE | DAP_TRANS_DP | DAP_TRANS_A8
DAP_TRANS_W_SELECT1 = DAP_TRANS_WRITE | DAP_TRANS_DP | DAP_TRANS_A4
DAP_TRANS_W_DLCR = DAP_TRANS_WRITE | DAP_TRANS_DP | DAP_TRANS_A4


# CMSIS-DAP SWO 传输类型
DAP_SWO_TRANSPORT_NONE    = 0x00
DAP_SWO_TRANSPORT_CAPTURE = 0x01
DAP_SWO_TRANSPORT_STREAM  = 0x02

# CMSIS-DAP SWO 模式
DAP_SWO_MODE_DEFAULT    = 0x00
DAP_SWO_MODE_UART       = 0x01
DAP_SWO_MODE_MANCHESTER = 0x02

# CMSIS-DAP SWO 控制
DAP_SWO_CTRL_DISABLE = 0x00
DAP_SWO_CTRL_ENABLE  = 0x01

class DAP_SWD_Seqence:
    DIR_OUTPUT = 0
    DIR_INPUT = 0x80

    def __init__(self, dir, bit_num: int, data: bytes|None=None):
        self.dir = dir
        self.bit_num = bit_num
        self.byte_num = (bit_num + 7) // 8
        self.data = data[0:self.byte_num]
        if (bit_num == 64):
            bit_num = 0
        if dir == DAP_SWD_Seqence.DIR_OUTPUT:
            self.cmd = int.to_bytes(bit_num & 0x1f | dir) + data
        else:
            self.cmd = int.to_bytes(bit_num & 0x1f | dir)

class DAP_JTAG_Seqence:
    def __init__(self, bit_num: int, data: bytes, tms: bool=False, tdo_capture: bool=False):
        self.bit_num = bit_num
        if (bit_num == 64):
            bit_num = 0
        seq_info = bit_num & 0x3f
        if tms:
            seq_info |= 0x40
        if tdo_capture:
            seq_info |= 0x80
        self.data = data[0:self.bit_num//8+1]
        self.cmd = seq_info.to_bytes(1) + bytes(data)
        self.success = False
        self.read_data = None
        if tdo_capture:
            self.read_num = bit_num
        else:
            self.read_num = 0
class DAP_Requset:
    def __init__(self, req:int, data: int|None=None):
        self.req = req
        self.data = data
        self.cmd = self.req.to_bytes()
        if (self.req & DAP_TRANS_READ) == 0:
            self.cmd += data.to_bytes(4, 'little')
        self.success = False
        self.read_data = 0

class DAP:
    def __init__(self, dev: usb.core.Device):
        cfg = dev.get_active_configuration()
        dap_intf = cfg[(0,0)]
        cdc_intf = cfg[(2,0)]
        dap_out_ep = dap_intf[0]
        dap_in_ep = dap_intf[1]
        self._dev = dev
        self._dap_out_ep = dap_out_ep
        self._dap_in_ep = dap_in_ep

        # SWO 端点
        self._dap_swo_ep = dap_intf[2]

        for _ in range(16):
            self._read()

        self.exec_num = 0
        self.exec_count = 0
        self.exec_buffer = None

    def _write(self, data):
        if self.exec_count < self.exec_num:
            r = self.exec_buffer.write(data)
            self.exec_count += 1

            if self.exec_count == self.exec_num:
                data = self.exec_buffer.getvalue()
                print('->:', data.hex())
                self._dev.write(self._dap_out_ep, data)
            return r
        else:
            print('->:', data.hex())
            return self._dev.write(self._dap_out_ep.bEndpointAddress, data)
    
    def _read(self, n=512, timeout=50):
        try:
            d = self._dev.read(self._dap_in_ep.bEndpointAddress, n, timeout=timeout)
            print('<-:', bytes(d).hex())
        except usb.core.USBTimeoutError:
            d = None
        return d

    def _DAP_Info(self, index: int):
        self._write(b'\x00' + index.to_bytes(signed=False))
        data = self._read()
        if (data[0] != 0):
            raise RuntimeError(f"unexpect respone {data[0]:02x}")
        dlen = data[1]
        if (dlen != len(data) - 2):
            raise RuntimeError(f"recv err {dlen}:{len(data)}")
        if (dlen == 0):
            return b''
        return bytes(data[2:2+dlen])
        
    def get_vendor_name(self):
        return self._DAP_Info(1).decode()
    
    def get_produce_name(self):
        return self._DAP_Info(2).decode()
    
    def get_serial_number(self):
        return self._DAP_Info(3).decode()
    
    def get_dap_protocol_version(self):
        return self._DAP_Info(4).decode()
    
    def get_target_device_vendor(self):
        return self._DAP_Info(5).decode()
    
    def get_target_device_name(self):
        return self._DAP_Info(6).decode()
    
    def get_target_board_vendor(self):
        return self._DAP_Info(7).decode()
    
    def get_target_board_name(self):
        return self._DAP_Info(8).decode()
    
    def get_product_firmware_version(self):
        return self._DAP_Info(9).decode()
    
    def get_capabilities(self):
        return self._DAP_Info(0xf0)
    
    def get_timestamp_freq(self):
        return int.from_bytes(self._DAP_Info(0xf1), 'little')
    
    def get_max_packet_count(self):
        return int.from_bytes(self._DAP_Info(0xfe), 'little')
    
    def get_max_packet_size(self):
        return int.from_bytes(self._DAP_Info(0xff), 'little')
    
    def connect(self, port=DAP_PORT_DEFAULT):
        self._write(b'\x02' + port.to_bytes(1))
        return self._read(2)
    
    def disconnect(self):
        self._write(b'\x03')
        return self._read(2)
    
    def start_exec(self, num: int):
        self.exec_num = num
        self.exec_count = 0
        self.exec_buffer = BytesIO()
        self.exec_buffer.write(b'\x7f' + num.to_bytes(1))

    def switch_swd(self):
        return self.swj_seqence(17*8, b'\xff\xff\xff\xff\xff\xff\xff\x9e\xe7\xff\xff\xff\xff\xff\xff\xff\x00')

    def swd_read_idcode(self):
        ret = self.transfer_block(1, DAP_TRANS_READ | DAP_TRANS_DP)
        return int.from_bytes(ret[-4:].tobytes(), 'little')

    def transfer_abort(self):
        self._write(b'x07')

    def swj_clock(self, clock: int):
        self._write(b'\x11' + clock.to_bytes(4, 'little'))
        return self._read(2)
    
    def swj_pins(self, output: int, select: int, delay_us: int):
        self._write(b'\x10' + output.to_bytes() + select.to_bytes() + delay_us.to_bytes(4, 'little'))
        return self._read(2)[1]

    def delay(self, time_us: int):
        self._write(b'\x09' + time_us.to_bytes(length=2, byteorder='little'))
        self._read(timeout=int(time_us/1000*3))

    def delay3(self, time_us: int):
        self._write(b'\x7e\x03' + (b'\x09' + time_us.to_bytes(length=2, byteorder='little')) * 3)
        self._read(timeout=int(time_us/1000*3))
        # self._read(timeout=1000)
    
    def swj_seqence(self, bit_num: int, data: bytes):
        byte_num = (bit_num + 7) // 8
        if bit_num == 256:
            bit_num = 0
        self._write(b'\x12' + bit_num.to_bytes(1) + data[0:byte_num])
        self._read(2, 1000)

    def swj_read(self):
        self._read(2, 1000)

    def swd_seqence(self, seq: List[DAP_SWD_Seqence]):
        cmd = b'\x1d' + len(seq).to_bytes(1)
        rd_len = 2
        for item in seq:
            cmd += item.cmd
            if item.dir == DAP_SWD_Seqence.DIR_INPUT:
                rd_len += item.byte_num
        if rd_len > 512 or len(cmd) > 512:
            raise RuntimeError("报文超长")
        self._write(cmd)
        # self._read(rd_len)
        return rd_len

    def transfer_block(self, trans_num, requse, data:List[int]|int|None=None):
        cmd = b'\x06\x00' + int.to_bytes(trans_num, 2, 'little') + int.to_bytes(requse, 1)
        if (requse & DAP_TRANS_READ) == 0:
            if type(data) is list:
                cmd += bytes([ i.to_bytes(4, 'little') for i in data ])
            elif type(data) is int:
                cmd += data.to_bytes(4, 'little')
        self._write(cmd)
        return self._read()

    def transfer(self, requsets: List[DAP_Requset]):
        cmd = b'\x05\x00' + len(requsets).to_bytes(1)
        for req in requsets:
            cmd += req.cmd
        self._write(cmd)
        data = self.read()
        if data[0] != 5:
            raise RuntimeError('not match')
        ret_num = data[1]
        # data.pop(3)
        # for i in range(ret_num):
        #     requsets[i].success = True
        #     if requsets[i].req & DAP_TRANS_READ:
        #         requsets[i].read_data = int.from_bytes(data.pop(4).to_bytes(), 'little')

    def swd_configure(self, turn: int, data_phase: int):
        cmd = b'\x13' + ((turn & 0x03) | (data_phase & 0x01) << 2).to_bytes(1)
        self.write(cmd)
        return self.read()
    
    def jtag_seqence(self, seq: List[DAP_JTAG_Seqence]):
        cmd = b'\x14' + len(seq).to_bytes(1)
        recv_len = 2
        for item in seq:
            cmd += item.cmd
            recv_len += item.read_num
        self._write(cmd)
        recv_data = BytesIO(self._read(recv_len, 1000))
        recv_data.read(1)
        status = recv_data.read(1)
        for i in range(len(seq)):
            if seq[i].read_num > 0:
                seq[i].read_data = recv_data.read((seq[i].read_num + 7) // 8)
            else:
                seq[i].read_data = None
        return status

    def jtag_idcode(self, jtag_index: int):
        cmd = b'\x16' + jtag_index.to_bytes(1)
        self._write(cmd)
        read_data = self._read(6, 1000).tobytes()
        if len(read_data) != 6 or read_data[0] != 0x16:
            raise RuntimeError('not match')
        if read_data[1] != 0:
            raise RuntimeError(f'status error: 0x{read_data[1]:02x}')
        return int.from_bytes(read_data[2:], 'little')
    
    def jtag_configure(self, ir_len: List[int]):
        cmd = b'\x15' + len(ir_len).to_bytes(1) + bytes(ir_len)
        self._write(cmd)
        return self._read(2, 1000)

    def read(self, n=512, timeout=10):
        return self._read(n, timeout)
    def write(self, data):
        self._write(data)

    # ========================================================================
    # SWO 追踪接口 (CMSIS-DAP v2 / HFLink 扩展)
    # ========================================================================

    def swo_transport(self, transport: int):
        """配置 SWO 传输模式
        Args:
            transport: DAP_SWO_TRANSPORT_NONE(0) / CAPTURE(1) / STREAM(2)
        """
        self._write(b'\x17' + transport.to_bytes(1))
        return self._read(2)

    def swo_mode(self, mode: int):
        """配置 SWO 编码模式
        Args:
            mode: DAP_SWO_MODE_DEFAULT(0) / UART(1) / MANCHESTER(2)
        """
        self._write(b'\x18' + mode.to_bytes(1))
        return self._read(2)

    def swo_baudrate(self, baudrate: int):
        """设置 SWO 波特率 (最大 100MHz)
        Firmware 根据 3.2GHz 基频自动计算 bit_time / decision_low / decision_high
        """
        self._write(b'\x19' + baudrate.to_bytes(4, 'little'))
        data = self._read(5)
        if data[0] == 0x19:
            actual = int.from_bytes(data[1:5], 'little')
            return actual
        return None

    def swo_control(self, enable: bool):
        """启停 SWO 采集
        enable=True: 应用 swo_mode + swo_baudrate 配置，使能 SWO 解码器
        enable=False: 关闭 SWO
        """
        ctrl = DAP_SWO_CTRL_ENABLE if enable else DAP_SWO_CTRL_DISABLE
        self._write(b'\x1a' + ctrl.to_bytes(1))
        return self._read(2)

    def swo_status(self) -> bool:
        """查询 SWO 是否已使能"""
        self._write(b'\x1b')
        data = self._read(2)
        return (data[0] == 0x1b and (data[1] & 0x01) != 0)

    def swo_data(self, max_count: int = 512) -> bytes | None:
        """读取 SWO 追踪数据
        Args:
            max_count: 最大期望字节数 (16-bit LE)
        Returns:
            追踪数据字节串，无数据时返回 None
        """
        self._write(b'\x1c' + max_count.to_bytes(2, 'little'))
        data = self._read(max_count + 4, timeout=100)
        if data is None or len(data) < 4:
            return None
        if data[0] != 0x1c:
            return None
        count = int.from_bytes(data[1:3], 'little')
        if count == 0:
            return None
        return bytes(data[3:3 + count])

    def swo_read_swo_ep(self, max_size: int = 4096, timeout: int = 1000) -> bytes | None:
        """从 SWO 专用 BULK IN 端点 (EP6 0x86) 直接读取追踪数据流"""
        try:
            data = self._dev.read(self._dap_swo_ep.bEndpointAddress, max_size, timeout=timeout)
            return bytes(data)
        except usb.core.USBTimeoutError:
            return None

    def swo_init_uart(self, baudrate: int = 1000000):
        """一站式初始化 UART 模式 SWO (传输+模式+波特率+使能)"""
        self.swo_transport(DAP_SWO_TRANSPORT_CAPTURE)
        self.swo_mode(DAP_SWO_MODE_UART)
        self.swo_baudrate(baudrate)
        return self.swo_control(True)

    def swo_init_manchester(self, baudrate: int = 1000000):
        """一站式初始化 Manchester 模式 SWO"""
        self.swo_transport(DAP_SWO_TRANSPORT_CAPTURE)
        self.swo_mode(DAP_SWO_MODE_MANCHESTER)
        self.swo_baudrate(baudrate)
        return self.swo_control(True)

    def swo_disable(self):
        """关闭 SWO"""
        self.swo_control(False)
        self.swo_transport(DAP_SWO_TRANSPORT_NONE)