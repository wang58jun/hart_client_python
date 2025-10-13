"""
hipCli: implementation of a simple HART-IP client

Created by J.W
 - Initial: 2025-Oct

"""

import sys
import socket
import time
import binascii
import argparse
from Consts import *


BUF_LEN = 1024

HIP_STA_NONE = 0
HIP_STA_SOCK_CONNECTED  = 1
HIP_STA_SESSION_CONNECTED = 2

HIP_HEADER_LEN = 8

HIP_MSG_SES_INIT = 0
HIP_MSG_SES_CLOS = 1
HIP_MSG_KEEP_ALIVE = 2
HIP_MSG_TP_PDU = 3
HIP_MSG_DIR_PDU = 4
HIP_MSG_ADT_LOG = 5


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


class HipCli:

    def __init__(self, ip=DEFAULT_SRV, port=DEFAULT_PORT, protocol=IP_PROTO[1], retry_num=3,
                 verbose=False):
        """
        initial of the HART TP-DLL simple master
        :param ip: HART-IP device (server) IP address
        :param port: port number
        :param protocol: IP protocol (UDP or TCP)
        :param retry_num: UDP retry number
        :param verbose: verbose information
        """
        self.sock = None
        self.status = HIP_STA_NONE
        self.local_ip = None
        self.protocol = protocol
        self.retry_num = retry_num
        self.srv_addr = (ip, port)
        self.host_name = ""
        self.unique_addr = bytearray(5)
        self.xmt_pdu = bytearray(BUF_LEN)
        self.rcv_pdu = bytearray(BUF_LEN)
        self.seq_num = 0000
        self.verbose = verbose

    def port_open(self):
        """
        Client IP socket connection
        :return:
            RET_OK - IP port was connected correctly
            RET_ERROR - Opened or connection fail
        """
        ret = RET_ERROR
        # noinspection PyBroadException
        try:
            if self.protocol == IP_PROTO[0]:
                self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            else:
                self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
                self.sock.connect(self.srv_addr)
            self.sock.settimeout(5)  # 5s time-out in receiving
            self.local_ip = socket.gethostbyname(socket.gethostname())
            self.status = HIP_STA_SOCK_CONNECTED
            ret = RET_OK
        except:
            self.sock = None
            self.status = HIP_STA_NONE
            print(f"Failed to connect {self.srv_addr}. Please check the port.")
        return ret

    def port_close(self):
        self.__del__()

    def __del__(self):
        if self.status >= HIP_STA_SESSION_CONNECTED:
            self.xmt_pdu = b''
            self._hip_xmt_rcv(HIP_MSG_SES_CLOS)  # Close session
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        self.status = HIP_STA_NONE

    def _hip_xmt_rcv(self, msg_id):
        """
        Send hip payload out and pending for receiving
        :param msg_id: HART-IP header message ID
        :return:
            RET_OK - successful transmitting & receiving
            RET_ERROR - failed in transmitting and/or receiving
        """
        wt_buf = b'\x01\x00'  # HART-IP revision number and request message
        wt_buf += msg_id.to_bytes(1, 'big')  # Message ID
        wt_buf += bytearray(b'\x00')  # Status code
        wt_buf += self.seq_num.to_bytes(2, 'big')  # Sequence number
        l = HIP_HEADER_LEN + len(self.xmt_pdu)
        wt_buf += l.to_bytes(2, 'big')
        wt_buf += self.xmt_pdu

        ret = RET_ERROR
        n = self.retry_num
        while ret == RET_ERROR and n > 0:
            # Start to transmitting
            if self.verbose: print(f"hip-STX: {array_to_hex_str(wt_buf)}")
            if self.protocol == IP_PROTO[0]:
                l = self.sock.sendto(wt_buf, self.srv_addr)
            else:
                l = self.sock.send(wt_buf)
            if l > 0:
                # Pending on receiving
                self.rcv_pdu = b''
                try:
                    if self.protocol == IP_PROTO[0]:
                        ba, _ = self.sock.recvfrom(BUF_LEN)  # read data
                    else:
                        ba = self.sock.recv(BUF_LEN)
                    if ba:
                        # bytes received
                        l = len(ba)
                        if l >= HIP_HEADER_LEN and l >= (ba[6] * 256 + ba[7]):
                            self.rcv_pdu = ba
                            if self.verbose: print(f"hip-ACK: {array_to_hex_str(self.rcv_pdu)}")
                            if self.rcv_pdu[1] == 0x01 and self.rcv_pdu[2] == wt_buf[2] and self.rcv_pdu[4:6] == wt_buf[4:6]:
                                ret = RET_OK
                                self.seq_num += 1
                                if self.rcv_pdu[2] == HIP_MSG_SES_INIT: self.status = HIP_STA_SESSION_CONNECTED
                                if self.rcv_pdu[2] == HIP_MSG_SES_CLOS: self.status = HIP_STA_SOCK_CONNECTED
                                break
                except socket.timeout:
                    print("Socket receiving timeout occurred!")
            n -= 1
        return ret


    def tp_poll_device(self):
        """
        Polling slave device at address 0
        :return:
            RET_OK - device was found in the specific polling address
            RET_ERROR - no device found
        """
        time.sleep(0.5)  # Pending for Rx thread ready
        self.xmt_pdu = b'\x01\x00\x09\x27\xC0'  # Keep alive: 10min
        if self._hip_xmt_rcv(HIP_MSG_SES_INIT) == RET_OK:
            self.xmt_pdu = b'\x02\x80'  # STX in polling address, primary master
            self.xmt_pdu += b'\x00\x00'  # command 0, no data
            # Send & receive in TP_DLL mode
            if self._tp_xmt_rcv() == RET_OK:
                if self.rcv_pdu[0] == 0x06 and (self.rcv_pdu[1] & 0x3f) == 0x00 and self.rcv_pdu[3] >= 14:
                    # Correct response with data
                    self.unique_addr = self.rcv_pdu[7:9] + self.rcv_pdu[15:18]
                    return RET_OK
        return RET_ERROR

    def tp_snd_rcv_cmd(self, cmd, data):
        """
        Send a specific command, and return the response
        :param cmd: 1 byte command number
        :param data: application layer payload, the first byte is the byte count
        :return: responded data from device, or an error information
        """
        self.xmt_pdu = b'\x82'  # STX in unique address
        self.xmt_pdu += set_master_type(self.unique_addr[0], MASTER_STR[1]).to_bytes(1, 'big')
        self.xmt_pdu += self.unique_addr[1:]
        self.xmt_pdu += cmd.to_bytes(1, 'big')
        self.xmt_pdu += data

        rsp_data = bytearray(b'\x02\x84\x00')
        if self._tp_xmt_rcv() == RET_OK:  # send and check
            if self.rcv_pdu[6] == cmd:
                rsp_data = self.rcv_pdu[7:-1]
        return rsp_data

    def tp_pass_through_cmd(self, master_str, unique_addr, cmd, data):
        """
        Send a pass through command with specific unique address, and return the response
        :param master_str: 'S' or 'P'
        :param unique_addr: sub-device 5 bytes unique address
        :param cmd: 1 byte command number
        :param data: application layer payload, the first byte is the byte count
        :return: responded data from device, or an error information
        """
        self.xmt_pdu = b'\x82'  # STX in unique address
        self.xmt_pdu += set_master_type(unique_addr[0], master_str).to_bytes(1, 'big')
        self.xmt_pdu += unique_addr[1:]
        self.xmt_pdu += cmd.to_bytes(1, 'big')
        self.xmt_pdu += data

        rsp_data = bytearray(b'\x02\x84\x00')
        if self._tp_xmt_rcv() == RET_OK:  # send and check
            if self.rcv_pdu[6] == cmd:
                rsp_data = self.rcv_pdu[7:-1]
        return rsp_data

    def _tp_xmt_rcv(self):
        """
        Send TP-DLL payload out and pending for receiving
        :return:
            RET_OK - successful transmitting & receiving
            RET_ERROR - failed in transmitting and/or receiving
        """
        # fill the serial port write buffer
        self.xmt_pdu += check_byte(self.xmt_pdu)  # check byte

        ret = RET_ERROR
        # Start to transmitting
        if self.verbose: print(f"tp-STX: {array_to_hex_str(self.xmt_pdu)}")
        if self._hip_xmt_rcv(HIP_MSG_TP_PDU) is RET_OK:
            l = self.rcv_pdu[6] * 256 + self.rcv_pdu[7]
            if l > HIP_HEADER_LEN and self.rcv_pdu[1] == 0x01 and self.rcv_pdu[2] == HIP_MSG_TP_PDU:
                # TP PDU response received
                self.rcv_pdu = self.rcv_pdu[8:]  # Only left the TP-DLL PDU
                if self.verbose: print(f"tp-ACK: {array_to_hex_str(self.rcv_pdu)}")
                c = check_byte(self.rcv_pdu)  # last check byte
                if c[0]: print("Check byte error")
                else:
                    if self.rcv_pdu[0] == 0x06: c = self.rcv_pdu[4] & 0x80  # communication byte
                    else: c = self.rcv_pdu[8] & 0x80
                    if c: print("Communication error")
                    else:
                        ret = RET_OK
        return ret

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='HART-IP Client Simple Implementation')
    parser.add_argument('-i', '--IP', type=str, default=DEFAULT_SRV, help="IP address, default=192.168.0.42")
    parser.add_argument('-p', '--port', type=int, default=DEFAULT_PORT, help="port number, default=5094")
    parser.add_argument('-t', '--type', type=str, default=IP_PROTO[1], help="Protocol type (UDP/TCP), default=T")
    parser.add_argument('-r', '--retry', type=int, default=3, help="UDP retry number, default=3")
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    print(args)

    hipClient = HipCli(ip=args.IP, port=args.port, protocol=args.type, retry_num=args.retry,
                      verbose=args.verbose)
    if hipClient.port_open() != RET_OK:
        sys.exit(-1)  # fail to open the port

    if hipClient.tp_poll_device() != RET_OK:
        print(f"Failed to poll a device at IP address {hipClient.srv_addr}.")
        sys.exit(-1)
    print(f"Found a device {array_to_hex_str(hipClient.unique_addr)} at IP address {hipClient.srv_addr}")

    snd_data = bytearray(b'\x00')
    for ii in range(4):
        time.sleep(2)
        print(f"Send command {ii}: {array_to_hex_str(snd_data)}")
        rcv_data = hipClient.tp_snd_rcv_cmd(ii, snd_data)
        print(f"Command {ii} response: {array_to_hex_str(rcv_data)}")

    time.sleep(2)
    cmd_num = 9
    snd_data = bytearray(b'\x04\x00\x01\x02\x03')
    print(f"Send command {cmd_num}: {array_to_hex_str(snd_data)}")
    rcv_data = hipClient.tp_snd_rcv_cmd(cmd_num, snd_data)
    print(f"Command {cmd_num} response: {array_to_hex_str(rcv_data)}")

    time.sleep(2)
    hipClient.port_close()  # close
