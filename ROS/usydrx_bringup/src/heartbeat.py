#!/usr/bin/env python3

from lib2to3.pytree import Base
from functools import reduce
import operator
import socket
import rospy
from std_msgs.msg import String, Int32
import time
from sensor_msgs.msg import NavSatFix
HOST = "robot.server"  # The server's hostname or IP address
# HOST = "127.0.0.1"  # The server's hostname or IP address

PORT = 12345  # The port used by the server


class HeartbeatClient:
    
    def __init__(self, s) -> None:
        rospy.init_node("heartbeat")
        self.socket = s

        self.latitude = 0.0
        self.longitude = 0.0

        self.lat_indicator = "S"
        self.long_indicator = "E"
        self.team_id = "USYD"

        self.system_mode = 3  # 1 = Remote, #2 = Autonomous #3 = Killed
        self.uav_status = 1 # 1 = Stowed, #2 = Deplotyed #3 = Faulted

        rospy.Subscriber("/light_sequence", String, self.light_sequence_cb)
        rospy.Subscriber("/fix", NavSatFix, self.gps_cb)
        rospy.Subscriber("/relay_state", String, self.relay_state_cb)
        rospy.Subscriber("/entrance_gate", Int32, self.entrance_gate_msg_cb)
        rospy.Subscriber("/dock_status", String, self.dock_status_cb)

    def dock_status_cb(self, data: String):
        msg = data.data
        hb = self.get_dock_string(msg)
        self.send_msg(hb)


        
    def gps_cb(self, data: NavSatFix):
        self.longitude = abs(data.longitude)
        self.latitude = abs(data.latitude)

    def relay_state_cb(self, data: String):
        if data.data == "AMBER":
            self.system_mode = 1
        elif data.data == "RED":
            self.system_mode = 3
        elif data.data == "GREEN":
            self.system_mode = 2
        else:
            self.system_mode = 3


    def entrance_gate_msg_cb(self, data: Int32):
        gate = data.data
        message = self.get_entrance_gate_msg(gate, gate)
        self.send_msg(message)
        pass

    def light_sequence_cb(self, data):

        print("got light sequence")
        msg = self.light_buoy_generate(data.data)
        self.send_msg(msg)
        pass


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

    def get_entrance_gate_msg(self, entrance_gate, exit_gate):

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

    def light_buoy_generate(self, rgb):
        today = self.get_date()

        time_str = self.get_time()

        hb_string = f"RXCOD,{today},{time_str},{self.team_id},{rgb}"

        checksum = self.get_checksum(hb_string)

        hb_msg = f"${hb_string}*{checksum}\r\n"

        return hb_msg

    def get_dock_string(self, msg):
        today = self.get_date()

        time_str = self.get_time()

        hb_string = f"RXDOK,{today},{time_str},{self.team_id},{msg}"

        checksum = self.get_checksum(hb_string)

        hb_msg = f"${hb_string}*{checksum}\r\n"

        return hb_msg

    def get_checksum(self, nmea_str):
        # this returns a 2 digit hexadecimal string to use as a checksum.
        sum = hex(reduce(operator.xor, map(ord, nmea_str), 0))[2:].upper()
        if len(sum) == 2:
            return sum
        else:
            return '0' + sum

    def send_msg(self, msg):
        print("sending", msg)
        self.socket.sendall(msg.encode())



    def start(self):

        st = time.time()

        # self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.socket.accept()
        # self.socket.connect((HOST, PORT))

   
               
        while not rospy.is_shutdown():
            msg = self.get_hb_string()
            self.send_msg(msg)

            # msg = self.light_buoy_generate("RGB")
            # print(f"sending {msg}")

            # s.sendall(msg.encode())
            # time.sleep(1.0)

            # if time.time() - st > 1:
            #     self.system_mode = 2

            time.sleep(1.0)

if __name__ == "__main__":
    print("Starting server")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        # s.sendall(b"Hello, world")
        hbc = HeartbeatClient(s)
        hbc.start()
    # print(hbc.get_hb_string())