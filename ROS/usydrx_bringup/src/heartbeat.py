#!/usr/bin/env python3

from lib2to3.pytree import Base
from functools import reduce
import operator
import socket
import rospy
import time
HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 12345  # The port used by the server


class HeartbeatClient:
    
    def __init__(self) -> None:
        rospy.init_node("heartbeat")

        self.latitude = 0.0
        self.longitude = 0.0

        self.lat_indicator = "S"
        self.long_indicator = "E"
        self.team_id = "USYD"

        self.system_mode = 1  # 1 = Remote, #2 = Autonomous #3 = Killed
        self.uav_status = 1 # 1 = Stowed, #2 = Deplotyed #3 = Faulted

    def get_date(self):
        today = time.strftime("%d%m%y")
        return today

    def get_time(self):
        time_str = time.strftime("%H%M%S")
        return time_str

    def get_hb_string(self):

        today = self.get_date()

        time_str = self.get_time()

        hb_string = f"RXHRB,{today},{time_str},{self.latitude:5f},{self.lat_indicator},{self.longitude:.5f},{self.long_indicator},{self.team_id},{self.system_mode},{self.uav_status}"

        checksum = self.get_checksum(hb_string)

        hb_msg = f"${hb_string}*{checksum}\r\n"

        return hb_msg

    def entrance_gate_msg(self, entrance_gate, exit_gate):

        today = self.get_date()

        time_str = self.get_time()

        hb_string = f"RXGAT,{today},{time_str},{self.team_id},{entrance_gate},{exit_gate}"

        checksum = self.get_checksum(hb_string)

        hb_msg = f"${hb_string}*{checksum}\r\n"

        return hb_msg

    def follow_the_path(self, finished):

        today = self.get_date()

        time_str = self.get_time()

        hb_string = f"RXPTH,{today},{time_str},{self.team_id},{finished}"

        checksum = self.get_checksum(hb_string)

        hb_msg = f"${hb_string}*{checksum}\r\n"

        return hb_msg
        print(time_str)

    def get_checksum(self, nmea_str):
        # this returns a 2 digit hexadecimal string to use as a checksum.
        sum = hex(reduce(operator.xor, map(ord, nmea_str), 0))[2:].upper()
        if len(sum) == 2:
            return sum
        else:
            return '0' + sum


    def start(self):
        while True:
            try:
                
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, PORT))
                    while True:
                        msg = self.get_hb_string()
                        print(f"sending {msg}")

                        s.sendall(msg.encode())
                        time.sleep(1.0)

            except {ConnectionError, BrokenPipeError} as e:
                print(f"Error {e}")
            time.sleep(1.0)

if __name__ == "__main__":
    hbc = HeartbeatClient()
    hbc.start()
    # print(hbc.get_hb_string())