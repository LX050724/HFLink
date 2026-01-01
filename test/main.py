import libusb_package
import usb.core
import usb
from dap import DAP
import time

if __name__ == '__main__':
    for dev in usb.core.find(find_all=True, backend=libusb_package.get_libusb1_backend(), product='HFLink CMSIS-DAP'):
        dap = DAP(dev)
        # print("  vendor name:", dap.get_vendor_name())
        # print(" produce name:", dap.get_produce_name())
        # print("serial number:", dap.get_serial_number())

        t = time.time()
        dap.delay3(1)
        dt = time.time() - t
        print(f"{dt*1e6:.2f}us")