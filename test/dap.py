from typing import List
import usb.core
import asyncio

class DAP_SWD_Seqence:
    DIR_OUTPUT = 0
    DIR_INPUT = 0x80

    def __init__(self, dir, bit_num: int, data: bytes):
        self.dir = dir
        self.bit_num = bit_num
        self.data = data
        self.byte_num = (bit_num + 7) // 8
        if (bit_num == 64):
            bit_num = 0
        if dir == DAP_SWD_Seqence.DIR_OUTPUT:
            self.cmd = int.to_bytes(bit_num & 0x1f | dir) + data
        else:
            self.cmd = int.to_bytes(bit_num & 0x1f | dir)



class DAP:
    def __init__(self, dev: usb.core.Device):
        cfg = dev.get_active_configuration()
        dap_intf = cfg[(0,0)]
        cdc_intf = cfg[(2,0)]
        [dap_out_ep, dap_in_ep, dap_swo_ep] = dap_intf
        self._dev = dev
        self._dap_out_ep = dap_out_ep
        self._dap_in_ep = dap_in_ep
        self._dap_swo_ep = dap_swo_ep
        for _ in range(8):
            self._read()

    def _write(self, data):
        print('->:', data.hex())
        return self._dev.write(self._dap_out_ep.bEndpointAddress, data)
    
    def _read(self, n=512, timeout=10):
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
        # self._read(2, 1000)

    def swj_read(self):
        self._read(2, 1000)

    def swd_seqence(self, seq: List[DAP_SWD_Seqence]):
        cmd = b'\x1d' + len(seq).to_bytes(1)
        rd_len = 0
        for item in seq:
            cmd += item.cmd
            if item.dir == DAP_SWD_Seqence.DIR_INPUT:
                rd_len += item.byte_num
        self._write(cmd)
        self._read(2 + rd_len)