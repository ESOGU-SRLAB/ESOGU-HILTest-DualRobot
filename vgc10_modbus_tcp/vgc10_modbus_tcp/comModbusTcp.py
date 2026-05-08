#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
Raw socket implementation compatible with OnRobot Compute Box.
"""

import socket
import struct
import threading


SLAVE_ID = 66  # Tool1=65 veya Tool2=66; ham socket testi ile 66 çalıştı


class communication:

    def __init__(self, dummy=False):
        self.sock = None
        self.dummy = dummy
        self.lock = threading.Lock()
        self._tid = 0

    def _next_tid(self):
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid

    def _send_recv(self, pdu):
        """Sends a Modbus TCP frame and returns the response PDU bytes."""
        tid = self._next_tid()
        mbap = struct.pack('>HHH', tid, 0, len(pdu))
        self.sock.sendall(mbap + pdu)
        # Read MBAP header first
        header = b''
        while len(header) < 6:
            chunk = self.sock.recv(6 - len(header))
            if not chunk:
                raise ConnectionError("Socket closed")
            header += chunk
        _, _, length = struct.unpack('>HHH', header)
        body = b''
        while len(body) < length:
            chunk = self.sock.recv(length - len(body))
            if not chunk:
                raise ConnectionError("Socket closed")
            body += chunk
        return body  # unit_id(1) + func_code(1) + data(...)

    def connectToDevice(self, ip, port):
        if self.dummy:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((str(ip), int(port)))

    def disconnectFromDevice(self):
        if self.dummy:
            return
        if self.sock:
            self.sock.close()
            self.sock = None

    def sendCommand(self, message):
        """Writes control registers (address 0~1) on the gripper."""
        if self.dummy:
            return
        if not message:
            return

        reg_a = message[0] + message[1]  # rmca<<8 | rvca  (combined)
        reg_b = message[2] + message[3]  # rmcb<<8 | rvcb
        values = [reg_a, reg_b]
        count = len(values)
        # FC 16 (0x10): Write Multiple Registers
        # address=0, count=2
        data = struct.pack(f'>BBHHB{count}H', SLAVE_ID, 0x10, 0, count, count * 2, *values)
        print(f"[comModbusTcp] sendCommand: reg_a={reg_a:#06x} reg_b={reg_b:#06x}")
        with self.lock:
            try:
                resp = self._send_recv(data)
                if resp[1] & 0x80:
                    print(f"[comModbusTcp] sendCommand ERROR: exception code {resp[2]}")
                else:
                    print(f"[comModbusTcp] sendCommand OK")
            except Exception as e:
                print(f"[comModbusTcp] sendCommand exception: {e}")

    def getStatus(self):
        """Reads status registers (address 258~259) from the gripper."""
        response = [0, 0]
        if self.dummy:
            return response
        # FC 3 (0x03): Read Holding Registers
        pdu = struct.pack('>BBHH', SLAVE_ID, 0x03, 258, 2)
        with self.lock:
            try:
                resp = self._send_recv(pdu)
                if resp[1] & 0x80:
                    print(f"[comModbusTcp] getStatus ERROR: exception code {resp[2]}")
                    return response
                n = resp[2]
                response = list(struct.unpack(f'>{n//2}H', resp[3:3+n]))
            except Exception as e:
                print(f"[comModbusTcp] getStatus exception: {e}")
        return response
