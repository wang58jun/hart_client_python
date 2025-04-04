"""
HART Multiplexer Class

Created by J.W
 - Initial: 2025-March

"""

from htTpMst import *

class HtMux:

    def __init__(self, com_port="/dev/ttyS0", baud_rate=57600, address=0, verbose=False):
        """
        initial of the HART Multiplexer
        :param com_port: COM port in string
        :param baud_rate: baud rate (int)
        :param address: HART Multiplexer address, 0 ~ 63
        :param verbose: verbose information
        """
        self.htTpMst = HtTpMst(com_port=com_port, baud_rate=baud_rate, master_str="P", retry_num=3, verbose=verbose)
        self.address = address
        self.verbose = verbose
        self.manuID = 0
        self.devTypeCode = 0
        self.uID = 0
        self.htRev = 0
        self.devRev = 0
        self.subDev = [bytearray(5)] * 256

    def connect(self):
        """
        Connect the HART Multiplexer at the specific address
        :return:
            RET_OK - connect correctly
            RET_ERROR - connect fail
        """
        ret = RET_ERROR
        # Open the serial port at first
        if self.htTpMst.port_open() == RET_OK:
            self.htTpMst.run()  # run HART TP-DLL Master Stack
            while self.htTpMst.rx_status is None:
                time.sleep(0.1)  # waiting for the TP-DLL ready

            # Polling the multiplexer
            if self.htTpMst.poll_device(self.address) != RET_OK:
                if self.verbose:
                    print(f"Failed to find the multiplexer at address {self.address}.")
                self.disconnect()
            else:
                ret = RET_OK
        return ret

    def disconnect(self):
        self.__del__()

    def __del__(self):
        self.htTpMst.port_close()  # close the TP-DLL Master Stack, and close the serial port
        time.sleep(0.5)  #waiting for the HART TP-DLL to be closed completely

    def command_x(self, cmd, data):
        time.sleep(0.3)
        return self.htTpMst.snd_rcv_cmd(self.address, cmd, data)

    """
    Universal commands
    """
    def command0(self):
        return self.command_x(0x00, b'\x00')

    def get_device_ident(self):
        ret = RET_ERROR
        rx_pdu = self.command0()
        if rx_pdu[0] > 0x02:
            self.manuID = int.from_bytes(rx_pdu[20:22], "big")
            self.devTypeCode = int.from_bytes(rx_pdu[4:6], "big")
            self.uID = int.from_bytes(rx_pdu[12:15], "big")
            self.htRev = int.from_bytes(rx_pdu[7:8], "big")
            self.devRev = int.from_bytes(rx_pdu[8:9], "big")
            ret = RET_OK
        return ret

    """
    Device specific commands
    """
    def command155(self, tx_pdu):
        rx_pdu = bytearray(b'\x02\x84\x00')
        for i in range(15):
            rx_pdu = self.command_x(155, tx_pdu)
            if rx_pdu[0] > 0x02: break
            if rx_pdu[1] != 0x20 and rx_pdu[1] != 0x21 and rx_pdu[1] != 0x22: break
        return rx_pdu

    def poll_sub_device(self, channel_num):
        ret = RET_ERROR
        poll_pdu = bytearray(b'\x0C\x00\xff\xff\xff\xff\xff\xff\x02\x80\x00\x00\x82')
        poll_pdu[1] = channel_num
        rx_pdu = htMux.command155(poll_pdu)
        if rx_pdu[0] > 0x02 and rx_pdu[3] == channel_num:
            if rx_pdu[4] == 0x06 and rx_pdu[6] == 0x00:
                n = 6+1+1+2
                self.subDev[channel_num] = rx_pdu[n+1:n+3] + rx_pdu[n+9:n+12]
                ret = RET_OK
        return ret

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='HART Multiplexer Demo')
    parser.add_argument('-c', '--port', type=str, default='COM1', help="UART port, default=COM1")
    parser.add_argument('-b', '--rate', type=int, default=19200, help="UART baud rate, default=19200")
    parser.add_argument('-a', '--address', type=int, default=0, help="Multiplexer address, default=0")
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    print(args)

    # Create a HART Multiplexer instance
    htMux = HtMux(com_port=args.port, baud_rate=args.rate, address=args.address, verbose=args.verbose)
    # Connect the HART Multiplexer
    if htMux.connect() != RET_OK:
        print(f"Failed to connect with the HART Multiplexer. Quit!")
        sys.exit(-1)  # fail to open the port

    # HART Multiplexer identification
    if htMux.get_device_ident() == RET_ERROR:
        print(f"Failed to find HART Multiplexer at address-{args.address}. Quit!")
        htMux.disconnect()
        time.sleep(1)
        sys.exit(-1)
    else:
        print(f"Found HART Multiplexer at address {args.address}:")
        print(f"************************")
        print(f" ->Manufacturer ID: {hex(htMux.manuID)}")
        print(f" ->Device Type Code: {hex(htMux.devTypeCode)}")
        print(f" ->Unique ID: {hex(htMux.uID)}")
        print(f" ->HART Revision: {hex(htMux.htRev)}")
        print(f" ->Device Revision: {hex(htMux.devRev)}")
        print(f"************************")

    # Commands scan
    snd_data = bytearray(b'\x00')
    for ii in range(42):
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = htMux.command_x(ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")
    for ii in range(43,49):
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = htMux.command_x(ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")
    print(f"************************")

    # Poll sub-devices
    for ii in range(32):
        print(f"Polling field device at channel {ii} ...")
        if htMux.poll_sub_device(ii) == RET_OK:
            print(f" ->FD {array_to_hex_str(htMux.subDev[ii])} found at channel {ii}")
    print(f"************************")

    time.sleep(1)
    htMux.disconnect()
