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

    def _write(self, data):
        print('w:', data.hex())
        return self._dev.write(self._dap_out_ep.bEndpointAddress, data)
    
    def _read(self, n=512, timeout=None):
        try:
            d = self._dev.read(self._dap_in_ep.bEndpointAddress, n, timeout=timeout)
            print('r:', bytes(d).hex())
        except usb.core.USBTimeoutError:
            d = None
        return d

    def _DAP_Info(self, index: int):
        data = b'\x00' + index.to_bytes(signed=False)
        self._write(data)
        return self._read()
    
    def _DAP_Info_str(self, index: int):
        data = self._DAP_Info(index)
        dlen = data[1] - 1
        if (dlen + 1 != len(data) - 2):
            raise RuntimeError(f"recv err {dlen}:{len(data)}")
        return bytes(data[2:2+dlen]).decode()
        
    def get_vendor_name(self):
        return self._DAP_Info_str(1)
    
    def get_produce_name(self):
        return self._DAP_Info_str(2)
    
    def get_serial_number(self):
        return self._DAP_Info_str(3)
    
    def get_dap_protocol_version(self):
        return self._DAP_Info_str(4)
    
    def get_target_device_vendor(self):
        return self._DAP_Info_str(5)
    
    def get_target_device_name(self):
        return self._DAP_Info_str(6)
    
    def get_target_board_vendor(self):
        return self._DAP_Info_str(7)
    
    def get_target_board_name(self):
        return self._DAP_Info_str(8)
    
    def delay(self, time_us: int):
        self._write(b'\x09' + time_us.to_bytes(length=2, byteorder='little'))
        self._read(timeout=int(time_us/1000*3))
        # self._read(timeout=1000)
    
    def delay3(self, time_us: int):
        self._write(b'\x7f\x03' + (b'\x09' + time_us.to_bytes(length=2, byteorder='little')) * 3)
        self._read(timeout=int(time_us/1000*6))