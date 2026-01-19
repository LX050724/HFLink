import libusb_package
import usb.core
import usb
from dap import DAP
import time

if __name__ == '__main__':
    for dev in usb.core.find(find_all=True, backend=libusb_package.get_libusb1_backend(), product='HFLink CMSIS-DAP'):
        dap = DAP(dev)
        for _ in range(500):
            print("produce name             :", dap.get_produce_name())
            print("vendor name              :", dap.get_vendor_name())
            print("serial number            :", dap.get_serial_number())
            print("product firmware version :", dap.get_product_firmware_version())
            print("timestamp freqence       :", dap.get_timestamp_freq())
            print("max packet count         :", dap.get_max_packet_count())
            print("max packet size          :", dap.get_max_packet_size())

            dap.delay3(100)
            # break
