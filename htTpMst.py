"""
HtTpMst will run the HART TP-DLL layer master state machine

Created by J.W
 - Initial: 2024-Dec

"""

import sys
import serial
import threading
import time
import binascii
import argparse
from Consts import *


BUF_LEN = 512

RX_STA_NONE = 0
RX_STA_SOM  = 1
RX_STA_ADDRESS = 2
RX_STA_CMD = 3
RX_STA_BC = 4
RX_STA_DATA = 5
RX_STA_CHK = 6


def array_to_hex_str(array):
    return binascii.hexlify(array)  # , '-')


def set_master_type(a, master_str):
    b = a & 0x3f  # polling address
    if master_str != MASTER_STR[0]:
        b |= MASTER_PRI  # master type bit
    return b


def check_byte(array):
    c = bytearray(b'\x00')
    for i in range(len(array)):
        c[0] ^= array[i]
    return c


class HtTpMst:

    def __init__(self, com_port="/dev/ttyS0", baud_rate=9600, master_str=MASTER_STR[1], retry_num=3,
                 verbose=False):
        """
        initial of the HART TP-DLL simple master
        :param com_port: COM port in string
        :param baud_rate: baud rate (int)
        :param master_str: master type string: "P" or "S"
        :param retry_num: xmt retry number
        :param verbose: verbose information
        """
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.ser = None
        self.master_str = master_str
        self.retry_num = retry_num
        self.unique_addr = [bytearray(5)] * 64
        self.preambles = [5] * 64
        self.xmt_pdu = bytearray(BUF_LEN)
        self.rcv_pdu = bytearray(BUF_LEN)
        self.rx_thread = None
        self.rx_status = None
        self.rx_cnt = 0
        self.exit_evt = threading.Event()
        self.verbose = verbose

    def port_open(self):
        """
        Open COM port
        :return:
            RET_OK - COM port was opened correctly
            RET_ERROR - Opened fail
        """
        # configure the serial connections (the parameters differs on the device you are connecting to)
        ret = RET_ERROR
        # noinspection PyBroadException
        try:
            self.ser = serial.Serial(
                port=self.com_port,
                baudrate=self.baud_rate,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            ret = RET_OK
        except:
            print(f"Failed to open {self.com_port}. Please check the port.")
        return ret

    def port_close(self):
        self.__del__()

    def __del__(self):
        self.exit_evt.set()  # Inform the run() to stop
        if self.ser is not None:
            self.ser.close()
        self.ser = None

    def poll_device(self, poll_addr):
        """
        Polling slave device at specific address
        :param poll_addr: device polling address, 0 ~ 63
        :return:
            RET_OK - device was found in the specific polling address
            RET_ERROR - no device found
        """
        self.xmt_pdu.clear()
        self.xmt_pdu = b'\x02'  # STX in polling address
        b = set_master_type(poll_addr, self.master_str)
        self.xmt_pdu += b.to_bytes(1, 'big')
        self.xmt_pdu += b'\x00\x00'  # command 0, no data

        # Send & receive with 10 preambles
        if self.tp_xmt_rcv(10) == RET_OK:
            if self.rcv_pdu[0] == 0x06 and (self.rcv_pdu[1] & 0x3f) == poll_addr and self.rcv_pdu[3] >= 14:
                # Correct response with data
                self.unique_addr[poll_addr] = self.rcv_pdu[7:9] + self.rcv_pdu[15:18]
                self.preambles[poll_addr] = self.rcv_pdu[9]
                return RET_OK
        return RET_ERROR

    def snd_rcv_cmd(self, addr, cmd, data):
        """
        Send a specific command, and return the response
        :param addr: device slave address, 0 ~ 63
        :param cmd: 1 byte command number
        :param data: application layer payload, the first byte is the byte count
        :return: responded data from device, or an error information
        """
        self.xmt_pdu = b'\x82'  # STX in unique address
        b = set_master_type(self.unique_addr[addr][0], self.master_str)
        self.xmt_pdu += b.to_bytes(1, 'big')
        self.xmt_pdu += self.unique_addr[addr][1:]
        self.xmt_pdu += cmd.to_bytes(1, 'big')
        self.xmt_pdu += data

        rsp_data = bytearray(b'\x02\x84\x00')
        if self.tp_xmt_rcv(self.preambles[addr]+1) == RET_OK:  # send and check
            if self.rcv_pdu[6] == cmd:
                rsp_data = self.rcv_pdu[7:-1]
        return rsp_data

    def tp_xmt_rcv(self, pre_len):
        """
        Send TP-DLL payload out and pending for receiving
        :param pre_len: Preamble length for transmitting
        :return:
            RET_OK - successful transmitting & receiving
            RET_ERROR - failed in transmitting and/or receiving
        """
        # fill the serial port write buffer
        wt_buf = bytearray()
        for i in range(pre_len):  # Preamble
            wt_buf += b'\xff'
        wt_buf += self.xmt_pdu  # TP-DLL pdu
        wt_buf += check_byte(self.xmt_pdu)  # check byte

        ret = RET_ERROR
        n = self.retry_num
        while ret == RET_ERROR and n > 0:
            # Start to transmitting
            if self.verbose: print(f"STX: {array_to_hex_str(wt_buf)}")
            self.ser.reset_output_buffer()
            self.ser.write(wt_buf)

            # Pending on receiving
            self.rx_status = RX_STA_SOM  # Start to receive
            self.ser.reset_input_buffer()  # Flush rx buff
            self.rcv_pdu.clear()
            self.rx_cnt = 0
            for i in range(15): # pending for receiving
                time.sleep(0.1)
                if self.rx_status == RX_STA_NONE:
                    # Rx finished
                    if self.verbose: print(f"ACK: {array_to_hex_str(self.rcv_pdu)}")
                    c = check_byte(self.rcv_pdu)  # last check byte
                    if c[0]: print("Check byte error")
                    else:
                        if self.rcv_pdu[0] == 0x06: c = self.rcv_pdu[4] & 0x80  # communication byte
                        else: c = self.rcv_pdu[8] & 0x80
                        if c: print("Communication error")
                        else:
                            ret = RET_OK
                    break
            self.rx_status = RX_STA_NONE  # Reset Rx status
            n -= 1
        return ret

    def data_indicate(self, number, ba):
        """
        Called by the receiving thread to indicate bytes being received
        :param number: byte numbers being received
        :param ba: bytes array being received
        :return: none
        """
        # bytes received
        for i in range(number):
            # byte check
            if self.rx_status == RX_STA_SOM:
                if ba[i] == 0xff:  # Preamble byte
                    self.rx_cnt += 1
                elif ba[i] == 0x06 or ba[i] == 0x86:  # Delimiter byte
                    if self.rx_cnt >= 2:
                        self.rx_status = RX_STA_ADDRESS
                    self.rx_cnt = 0
                else:  # other byte
                    self.rx_cnt = 0
            elif self.rx_status == RX_STA_ADDRESS:
                if self.rcv_pdu[0] == 0x06:  # short address
                    self.rx_status = RX_STA_CMD
                else:
                    self.rx_cnt += 1
                    if self.rx_cnt >= 5:  # long address length
                        self.rx_cnt = 0
                        self.rx_status = RX_STA_CMD
            elif self.rx_status == RX_STA_CMD:
                self.rx_status = RX_STA_BC
            elif self.rx_status == RX_STA_BC:
                if ba[i] and ba[i] < (BUF_LEN-8):
                    self.rx_cnt = ba[i]  # byte count is not zero
                    self.rx_status = RX_STA_DATA
                else:
                    self.rx_status = RX_STA_CHK
            elif self.rx_status == RX_STA_DATA:
                self.rx_cnt -= 1
                if self.rx_cnt <= 0:
                    self.rx_status = RX_STA_CHK
            elif self.rx_status == RX_STA_CHK:
                self.rcv_pdu += ba[i].to_bytes(1, 'big')  # save the last check byte
                self.rx_status = RX_STA_NONE
            else:
                self.rx_cnt = 0
                self.rx_status = RX_STA_NONE
            # Save into rx buffer
            if self.rx_status >= RX_STA_ADDRESS:
                self.rcv_pdu += ba[i].to_bytes(1, 'big')

    def rcv_loop(self):
        """
        Serial port reading loop
        :return: none
        """
        print("Start Master TP-DLl RCV_MSG")
        self.rx_status = RX_STA_NONE
        while True:
            if self.exit_evt.is_set():
                print("htTpRx got exit event, exiting")
                break
            n = self.ser.in_waiting
            if n:
                ba = self.ser.read(n)  # read data out
                if self.rx_status != RX_STA_NONE:
                    self.data_indicate(n, ba)

    def run(self):  # Run the TP-DLL master state machine
        if self.ser is None:
            self.port_open()
        if self.rx_thread is None:
            self.rx_thread = threading.Thread(target=self.rcv_loop)  # Create Rx thread
            self.rx_thread.start()  # Start Rx


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='HART Master TP-DLL Simple Implementation')
    parser.add_argument('-c', '--port', type=str, default='COM1', help="UART port, default=COM1")
    parser.add_argument('-b', '--rate', type=int, default=1200, help="UART baud rate, default=9600")
    parser.add_argument('-m', '--master', type=str, default=MASTER_STR[1], help="HART master type (P/S), default=P")
    parser.add_argument('-r', '--retry', type=int, default=3, help="HART STX retry number, default=3")
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    print(args)

    htTpMst = HtTpMst(com_port=args.port, baud_rate=args.rate, master_str=args.master, retry_num=args.retry,
                      verbose=args.verbose)
    if htTpMst.port_open() != RET_OK:
        sys.exit(-1)  # fail to open the port
    htTpMst.run()

    while htTpMst.rx_status is None:
        time.sleep(0.1)

    slave_addr = 0
    if htTpMst.poll_device(slave_addr) != RET_OK:
        print(f"Failed to find a device at polling address {slave_addr}.")
        htTpMst.exit_evt.set()
        sys.exit(-1)
    print(f"Found a device {array_to_hex_str(htTpMst.unique_addr[slave_addr])} at polling address {slave_addr}")

    snd_data = bytearray(b'\x00')
    for ii in range(4):
        time.sleep(2)
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = htTpMst.snd_rcv_cmd(slave_addr, ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")

    time.sleep(2)
    cmd_num = 9
    snd_data = bytearray(b'\x04\x00\x01\x02\x03')
    print(f"Send command {cmd_num}: {array_to_hex_str(snd_data)}")
    rcv_data = htTpMst.snd_rcv_cmd(slave_addr, cmd_num, snd_data)
    print(f"Command {cmd_num} response: {array_to_hex_str(rcv_data)}")

    time.sleep(2)
    htTpMst.port_close()  # close
