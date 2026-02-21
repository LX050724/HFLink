from typing import List
import usb.core
import asyncio
from io import BytesIO
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



class DAP:
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

    def __init__(self, dev: usb.core.Device):
        cfg = dev.get_active_configuration()
        dap_intf = cfg[(0,0)]
        cdc_intf = cfg[(2,0)]
        dap_out_ep = dap_intf[0]
        dap_in_ep = dap_intf[1]
        self._dev = dev
        self._dap_out_ep = dap_out_ep
        self._dap_in_ep = dap_in_ep
        self._dap_swo_ep = None
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
        self.swj_seqence(17*8, b'\xff\xff\xff\xff\xff\xff\xff\x9e\xe7\xff\xff\xff\xff\xff\xff\xff\x00')

    def swd_read_idcode(self):
        ret = self.swd_transfer_block(1, self.DAP_TRANS_READ | self.DAP_TRANS_DP)
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

    def swd_transfer_block(self, trans_num, requse, data:bytes=None):
        cmd = b'\x06\x00' + int.to_bytes(trans_num, 2, 'little') + int.to_bytes(requse)
        if requse & self.DAP_TRANS_WRITE:
            cmd += data[0:trans_num*4]
        self._write(cmd)
        return self._read()

    def read(self, n=512, timeout=10):
        return self._read(n, timeout)
    def write(self, data):
        self._write(data)