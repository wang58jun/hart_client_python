"""
HART Multiplexer Class

Created by J.W
 - Initial: 2025-March
 - v0.1: 2025-April - sub-device pass-through command support

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
        self.subDev_Total = 0
        self.master_str = MASTER_STR[1]
        self.subDev_uid = []

        # create the sub-device uID list
        for i in range(256):
            self.subDev_uid.append([])
            for j in range(64):
                self.subDev_uid[i].append(bytearray(5))

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
        """
        Send specific command to the HART Mux and return the response
        :param cmd: command number
        :param data: data including byte count
        :return: response data from the HART Mux
        """
        time.sleep(0.3)
        return self.htTpMst.snd_rcv_cmd(self.address, cmd, data)

    def command_x_pend(self, cmd, data):
        """
        Same as "command_x" except will be pending for busy or delay response
        :param cmd: command number
        :param data: data including byte count
        :return: response data from the HART Mux
        """
        rx_adu = bytearray(b'\x02\x84\x00')
        for i in range(15):
            rx_adu = self.command_x(cmd, data)
            if rx_adu[0] > 0x02: break
            if rx_adu[1] != 0x20 and rx_adu[1] != 0x21 and rx_adu[1] != 0x22: break
        return rx_adu

    def forward_command_x(self, channel_num, poll_addr, cmd, data):
        """
        Forward specific command with data to the sub-device via command 155
        Note: Before calling this function, be sure the requested sub-device was identified
                by using 'poll_sub_device' or 'refresh_channel_list' or 'refresh_sub_device_list' function
        :param channel_num: channel number of the sub-device
        :param poll_addr: loop polling address of the sub-device
        :param cmd: command number for sub-device
        :param data: data including byte count
        :return: response data from the sub-device
        """
        cmd155_adu = bytearray(b'\x00\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\x82\x00\x00\x00\x00\x00\x00')
        cmd155_adu[1] = channel_num
        cmd155_adu[12:17] = self.subDev_uid[channel_num][poll_addr]
        cmd155_adu[12] = set_master_type(cmd155_adu[12], self.master_str)
        cmd155_adu[17] = cmd
        cmd155_adu += data
        cmd155_adu[0] = len(cmd155_adu)
        cmd155_adu += check_byte(cmd155_adu[11:])
        time.sleep(0.3)
        return self.command_x(155, cmd155_adu)

    def forward_command_x_pend(self, channel_num, poll_addr, cmd, data):
        """
        Same as "forward_command_x" except will be pending for busy or delay response
        :param channel_num: channel number of the sub-device
        :param poll_addr: loop polling address of the sub-device
        :param cmd: command number for sub-device
        :param data: data including byte count
        :return: response data from the sub-device
        """
        rx_adu = bytearray(b'\x02\x84\x00')
        for i in range(15):
            rx_adu = self.forward_command_x(channel_num, poll_addr, cmd, data)
            if rx_adu[0] > 0x02: break
            if rx_adu[1] != 0x20 and rx_adu[1] != 0x21 and rx_adu[1] != 0x22: break
        return rx_adu

    def io_sub_command_x(self, channel_num, poll_addr, cmd, data):
        """
        Pass through specific command with data to the sub-device
        Note: Before calling this function, be sure the requested sub-device was identified
                by using 'poll_sub_device' or 'refresh_channel_list' or 'refresh_sub_device_list' function
        :param channel_num: channel number of the sub-device
        :param poll_addr: loop polling address of the sub-device
        :param cmd: command number
        :param data: data including byte count
        :return: response data from the sub-device
        """
        time.sleep(0.3)
        return self.htTpMst.pass_through_cmd(5, self.master_str, self.subDev_uid[channel_num][poll_addr], cmd, data)

    def io_sub_command_x_pend(self, channel_num, poll_addr, cmd, data):
        """
        Same as "io_sub_command_x" except will be pending for busy or delay response
        :param channel_num: channel number of the sub-device
        :param poll_addr: loop polling address of the sub-device
        :param cmd: command number
        :param data: data including byte count
        :return: response data from the sub-device
        """
        rx_adu = bytearray(b'\x02\x84\x00')
        for i in range(15):
            rx_adu = self.io_sub_command_x(channel_num, poll_addr, cmd, data)
            if rx_adu[0] > 0x02: break
            if rx_adu[1] != 0x20 and rx_adu[1] != 0x21 and rx_adu[1] != 0x22: break
        return rx_adu

    def get_device_ident(self):
        """
        Identify the HART Mux device
        :return: RET_OK or RET_ERROR
        """
        ret = RET_ERROR
        rx_adu = self.command_x_pend(0x00, b'\x00')
        if rx_adu[0] > 0x02:
            self.manuID = int.from_bytes(rx_adu[20:22], "big")
            self.devTypeCode = int.from_bytes(rx_adu[4:6], "big")
            self.uID = int.from_bytes(rx_adu[12:15], "big")
            self.htRev = int.from_bytes(rx_adu[7:8], "big")
            self.devRev = int.from_bytes(rx_adu[8:9], "big")
            ret = self.read_hart_mux_para()
        return ret

    def poll_sub_device(self, channel_num, poll_addr):
        """
        Identify the sub-device at specific channel and polling address
        :param channel_num: channel number of the sub-device
        :param poll_addr: loop polling address of the sub-device
        :return: RET_OK if the specific sub-device was identified correctly; otherwise RET_ERROR
        """
        ret = RET_ERROR
        poll_adu = bytearray(b'\x0F\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\x02\x80\x00\x00')
        poll_adu[1] = channel_num
        poll_adu[12] = set_master_type(poll_addr, self.master_str)
        poll_adu += check_byte(poll_adu[11:])
        rx_adu = self.command_x_pend(155, poll_adu)
        if rx_adu[0] > 0x02 and rx_adu[3] == channel_num:
            if rx_adu[4] == 0x06 and rx_adu[6] == 0x00:
                n = 6+1+1+2
                self.subDev_uid[channel_num][poll_addr] = rx_adu[n+1:n+3] + rx_adu[n+9:n+12]
                ret = RET_OK
        return ret

    def read_hart_mux_para(self):
        """
        Update the HART Mux parameters by issuing command 128
        :return: RET_OK or RET_ERROR
        """
        ret = RET_ERROR
        self.subDev_Total = 0
        rx_adu = self.command_x_pend(128, b'\x00')
        if rx_adu[0] >= 20:
            self.subDev_Total = rx_adu[11] * 256 + rx_adu[12]
            if rx_adu[15] == 0:
                self.master_str = MASTER_STR[0]
            else:
                self.master_str = MASTER_STR[1]
            ret = RET_OK
        return ret

    def refresh_channel_list(self, channel_num):
        """
        Refresh the specific channel list by issuing command 129
        :param channel_num: channel nuber
        :return: sub devices list [poll_addr, (sub_device_ext_addr)]
        """
        sub_dev_list = []
        if channel_num > 255:
            return sub_dev_list

        tx_adu = bytearray(b'\x01')
        tx_adu += channel_num.to_bytes(1, 'big')
        rx_adu = self.command_x_pend(129, tx_adu)
        if rx_adu[0] >= 5:
            idx = 5  # current idx in the receive pdu
            n = rx_adu[idx]  # byte number of the rest
            while n >= 8:
                a = rx_adu[idx+1]  # sub-device polling address
                if a < 64:
                    self.subDev_uid[channel_num][a] = rx_adu[idx+2:idx+7]
                    sub_dev_list.append([a, rx_adu[idx+2:idx+7]])
                    if self.verbose:
                        print(f" ->Sub-dev {array_to_hex_str(rx_adu[idx+2:idx+7])} found at channel {channel_num} address {a}")
                if n > 8:
                    n = rx_adu[idx+9]
                else:
                    n = 0

        # Update the online sub-device number
        self.read_hart_mux_para()
        return sub_dev_list

    def refresh_sub_device_list(self):
        """
        Refresh the sub devices list by issuing command 130/131
        :return: sub devices list [channel, poll_addr, (sub_device_ext_addr)]
        """
        sub_dev_list = []

        # Get the online sub-device number at first
        ret = self.read_hart_mux_para()

        if ret == RET_OK and self.subDev_Total > 0:
            # update the list by using command 130 & 131
            n = self.subDev_Total
            idx = 0
            while n > 0:
                tx_adu = bytearray(b'\x02')
                tx_adu += idx.to_bytes(2, 'big')
                rx130_adu = self.command_x_pend(130, tx_adu)  # Command 130
                if rx130_adu[0] >= 4:
                    i = int((rx130_adu[0] - 4) / 5)  # sub-device number in response
                    if i == 0:
                        break
                    for j in range(i):
                        tx_adu = bytearray(b'\x06')
                        tx_adu += rx130_adu[5+j*5:5+j*5+5]  # sub-device unique address
                        tx_adu += b'\x00'
                        rx131_adu = self.command_x_pend(131, tx_adu)  # Command 131
                        if rx131_adu[0] >= 8 and rx131_adu[8] >= 5 and rx131_adu[9] == 0:
                            c = rx131_adu[10]  # Channel number
                            a = rx131_adu[11]  # Polling address
                            if a < 64:
                                self.subDev_uid[c][a] = tx_adu[1:6]
                                sub_dev_list.append([c, a, tx_adu[1:6]])
                                if self.verbose:
                                    print(f" ->Sub-dev {array_to_hex_str(tx_adu[1:6])} found at channel {c} address {a}")
                    n -= i
                    idx += i
                else:  # command 130 response error
                    break
        return sub_dev_list

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
        print(f" ->Master Type: {htMux.master_str}")
        print(f" ->Total Sub-dev: {htMux.subDev_Total}")
        print(f"************************")

    # HART MUX universal commands scan
    print(f"\r\nCommands scan...")
    snd_data = bytearray(b'\x00')
    for ii in range(42):
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = htMux.command_x_pend(ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")
    for ii in range(43,49):
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = htMux.command_x_pend(ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")
    print(f"************************")

    # Poll sub-devices by using command 155
    print(f"\r\nPolling sub devices by command 155...")
    for ii in range(32):
        loop_addr = 0
        print(f"Polling field device at channel {ii} address {loop_addr} ...")
        if htMux.poll_sub_device(ii, loop_addr) == RET_OK:
            print(f" ->FD {array_to_hex_str(htMux.subDev_uid[ii][loop_addr])} found at channel {ii} address {loop_addr}")
            print(f" -> forward command 1 to the sub-dev via command 155...")
            rcv_data = htMux.forward_command_x_pend(ii, loop_addr, 1, b'\x00')
            print(f" -> command 155 response: {array_to_hex_str(rcv_data)}")
    print(f"************************")

    # Refresh the sub device list by using command 129
    print(f"\r\nRefresh sub-device list by going through channels")
    for ii in range(32):
        print(f"Read channel {ii} information...")
        sub_list = htMux.refresh_channel_list(ii)
        for jj in sub_list:
            print(f" ->FD {array_to_hex_str(jj[1])} found at channel {ii} address {jj[0]}")
            print(f" -> send command 2 to the sub-dev...")
            rcv_data = htMux.io_sub_command_x_pend(ii, jj[0], 2, b'\x00')
            print(f" -> command 2 response: {array_to_hex_str(rcv_data)}")
    print(f"************************")

    # Update the sub device list by using command 130/131
    print(f"\r\nReading sub-device list by command 130/131")
    sub_list = htMux.refresh_sub_device_list()
    for ii in sub_list:
        print(f" ->FD {array_to_hex_str(ii[2])} found at channel {ii[0]} address {ii[1]}")
        print(f" -> send command 3 to sub-dev...")
        rcv_data = htMux.io_sub_command_x_pend(ii[0], ii[1], 3, b'\x00')
        print(f" -> command 3 response: {array_to_hex_str(rcv_data)}")
    print(f"************************")

    print(f"\nTotal {htMux.subDev_Total} sub devices are online...")

    time.sleep(1)
    htMux.disconnect()
