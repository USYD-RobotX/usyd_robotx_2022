#!/usr/bin/env python3

import os
import sys
# sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../LCM/acfr/lcmtypes'))
from acfrlcm import wam_v_thruster_alloc_t
import time
import rospy
import lcm

from std_msgs.msg import Float32, Empty



max_cmd = 0.99
class ControlToLCM():
    lcm_cmd_mag = 25000

    def __init__(self):
        self.cmd_msg = [(0, 0.0), (0,0.0),(0, 0),(0, 0)] # rear_left, rear_right, front_left, front_right"

        self.timeout = 0.5


        
        right_front_topic = "/wamv/thrusters/right_front_thrust_cmd"
        right_rear_topic = "/wamv/thrusters/right_rear_thrust_cmd"
        left_front_topic = "/wamv/thrusters/left_front_thrust_cmd"
        left_rear_topic = "/wamv/thrusters/left_rear_thrust_cmd"


        rospy.init_node("lcm_control_bridge")

        rospy.Subscriber(right_front_topic, Float32, self.receive_right_front)
        rospy.Subscriber(right_rear_topic, Float32, self.receive_right_rear)
        rospy.Subscriber(left_front_topic, Float32, self.receive_left_front)
        rospy.Subscriber(left_rear_topic, Float32, self.receive_left_rear)

        self.lc = lcm.LCM()

    def receive_right_front(self, data):
        print("rec_rfront")
        self.cmd_msg[3] = (time.perf_counter(), max(-max_cmd, min(max_cmd, (time.perf_counter()), data.data)))

    def receive_right_rear(self, data):
        print("rec_rrear")

        self.cmd_msg[1] = (time.perf_counter(), max(-max_cmd, min(max_cmd, (time.perf_counter()), data.data)))

    def receive_left_front(self, data):
        print("rec_lfront")

        self.cmd_msg[2] = (time.perf_counter(), max(-max_cmd, min(max_cmd, (time.perf_counter()), data.data)))

    def receive_left_rear(self, data):
        print("rec_lrear")

        self.cmd_msg[0] = (time.perf_counter(), max(-max_cmd, min(max_cmd, (time.perf_counter()), data.data)))


    def publish_lcm(self):

        cmd_msg = wam_v_thruster_alloc_t()
        cmd_msg.utime = int(time.time()*1000000)
      
        for time_stamp, cmd in self.cmd_msg:
           if time.perf_counter() - time_stamp > self.timeout:
               print("Timeout")
               cmd_msg.run_mode = 0
               cmd_msg.stern_port = 0
               cmd_msg.stern_stbd = 0

               cmd_msg.bow_port = 0
               cmd_msg.bow_stbd = 0
               self.lc.publish("WAMV.THRUSTER_ALLOC", cmd_msg.encode())
               return

        cmd_msg.run_mode = wam_v_thruster_alloc_t.RUN

        cmd_msg.stern_port = float(self.cmd_msg[0][1] * self.lcm_cmd_mag)
        cmd_msg.stern_stbd = float(self.cmd_msg[1][1] * self.lcm_cmd_mag)

        cmd_msg.bow_port = float(self.cmd_msg[2][1] * self.lcm_cmd_mag)
        cmd_msg.bow_stbd = float(self.cmd_msg[3][1] * self.lcm_cmd_mag)

        self.lc.publish("WAMV.THRUSTER_ALLOC", cmd_msg.encode())
        print("Thruster alloc")
        
    def start(self):
        rate = 20

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.publish_lcm()
            r.sleep()

if __name__ == "__main__":
    ctl = ControlToLCM()
    ctl.start()
