import usb.core
import asyncio



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
        self._read()

    def _write(self, data):
        print('->:', data.hex())
        return self._dev.write(self._dap_out_ep.bEndpointAddress, data)
    
    def _read(self, n=512, timeout=1):
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
    