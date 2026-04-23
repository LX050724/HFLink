import libusb_package
import usb.core
import usb
from dap import *
import time
import random
import os
import sys
from zlib import crc32


if __name__ == '__main__':
    for dev in usb.core.find(find_all=True, backend=libusb_package.get_libusb1_backend(), product='HFLink CMSIS-DAP Plus'):
    # for dev in usb.core.find(find_all=True, backend=libusb_package.get_libusb1_backend(), product='CherryUSB CMSIS-DAP'):
        dap = DAP(dev)

        # dap.transfer_abort()

        # for _ in range(500):
        print("produce name             :", dap.get_produce_name())
        print("vendor name              :", dap.get_vendor_name())
        print("serial number            :", dap.get_serial_number())
        print("product firmware version :", dap.get_product_firmware_version())
        print("timestamp freqence       :", dap.get_timestamp_freq())
        print("max packet count         :", dap.get_max_packet_count())
        print("max packet size          :", dap.get_max_packet_size())
        
        
        # dap.disconnect()
        dap.connect(DAP_PORT_JTAG)
        dap.swj_clock(10000000)
        dap.jtag_configure([4, 5])

        while True:
            print(f"{dap.jtag_idcode(0):08x}")
            time.sleep(0.1)

        

        # print(dap.swj_pins(0x80, 0x80, 1000))

        # dap.read()

        # dap.swj_seqence()

        # dap.swj_clock(20000000)
        dap.switch_swd()
        print(f"{dap.swd_read_idcode():08x}")

        print('DAP_TRANS_W_SELECT')
        dap.transfer_block(1, DAP_TRANS_W_SELECT, 1)
        print('DAP_TRANS_W_DLCR')
        dap.transfer_block(1, DAP_TRANS_W_DLCR, 1 << 8)
        print('swd_configure')
        dap.swd_configure(1, 0)
        print('DAP_TRANS_W_SELECT')
        dap.transfer_block(1, DAP_TRANS_W_SELECT, 0)
        print('swd_read_idcode')
        dap.swj_clock(60000000)

        time.sleep(5)

        while True:
            print(f"{dap.swd_read_idcode():08x}")
            time.sleep(0.1)



        exit()


        # while True:
           


            # dap.start_exec(2)
            # dap.swj_seqence(256, bytes(range(256)))
            # dap.swj_seqence(256, bytes(range(256)))
            # dap.swj_seqence(256, bytes(range(256)))
            # dap.swj_seqence(256, bytes(range(256)))
            # dap.swj_seqence(8, bytes(range(256)))
            # dap.swj_seqence(8, bytes(range(256)))
            # dap.swj_seqence(8, bytes(range(256)))
            # dap.swj_seqence(8, bytes(range(256)))
            # dap.swj_seqence(127, b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x55'*2)
            # dap.swj_seqence(127, b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x55'*2)
            # dap.swj_seqence(127, b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x55'*2)
            # dap.swj_seqence(127, b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x55'*2)
            # dap.read()

            # dap.swj_read()
            # time.sleep(0.1)
            # dap.swj_read()
            # dap.swj_read()
            # dap.swj_read()
            # dap.swj_read()
            # dap.swj_read()
            # dap.swj_read()
            # dap.swj_read()
            # continue

            # continue
            
            # rd = []
            # seq = DAP_SWD_Seqence(DAP_SWD_Seqence.DIR_OUTPUT, 64, random.randbytes(8))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))
            # rd.append(dap.swd_seqence([ seq for _ in range(32)]))

            # for i in rd:
            #     dap.read(i)


            # seq = DAP_SWD_Seqence(DAP_SWD_Seqence.DIR_INPUT, 64)
            # dap.swd_seqence([ seq for _ in range(63)])
            # dap.swj_read()
            # time.sleep(0.1)

            # dap.delay3(0xffff)
            # exit()

        # dap.delay3(100)
            # break
