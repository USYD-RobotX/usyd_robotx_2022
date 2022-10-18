#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Empty

from scipy.spatial.transform import Rotation as R
import numpy as np

from simple_pid import PID

import math
import time

FREQ=10

def get_angle_difference(a,b):
    difference = min((a-b) % (2*math.pi), -((b-a) % (2*math.pi)), key=abs)
    return difference


class SimplePIDHeadingController:

    # kp = 3
    # kd = 0
    # ki = 0

    # current_heading = None

    # desired_heading = None

    def __init__(self):

        rospy.init_node("simple_pid_heading_controller")

        right_front_topic = "/wamv/thrusters/right_front_thrust_cmd"
        right_rear_topic = "/wamv/thrusters/right_rear_thrust_cmd"
        left_front_topic = "/wamv/thrusters/left_front_thrust_cmd"
        left_rear_topic = "/wamv/thrusters/left_rear_thrust_cmd"

        self.right_front_pub = rospy.Publisher(right_front_topic, Float32, queue_size = 10)
        self.right_rear_pub = rospy.Publisher(right_rear_topic, Float32, queue_size = 10)
        self.left_front_pub = rospy.Publisher(left_front_topic, Float32, queue_size = 10)
        self.left_rear_pub = rospy.Publisher(left_rear_topic, Float32, queue_size = 10)

        self.debug = rospy.Publisher("/debug_heading", Float32, queue_size = 10)

        rospy.Subscriber("/wamv/odom", Odometry, self.receive_odom)

        # self.pid = PID(3, 0, 1.5, setpoint=0)
        # self.pid.sample_time = 1/FREQ
        # self.pid.output_limits = (-1, 1)
        # self.time_data = []
        # self.heading_data = []

        # self.st = time.perf_counter()

    def receive_odom(self, odom_msg):
        pass
        # _odom_quat = odom_msg.pose.pose.orientation
        # odom_quat = [_odom_quat.x, _odom_quat.y, _odom_quat.z, _odom_quat.w]
        
        # heading = R.from_quat(odom_quat).as_euler('xyz')[2]

        # self.current_heading = heading

        # if self.desired_heading is None:
        #     self.desired_heading = heading + 2
        # if self.desired_heading is None or self.current_heading is None:
        #     pass
        # else:
        #     self.time_data.append(time.perf_counter() - self.st)
        #     self.heading_data.append(get_angle_difference(self.desired_heading, self.current_heading))

    def run_control_loop(self):
        # if self.desired_heading is None or self.current_heading is None:
        #     return
        
        # self.pid.setpoint = 0

        # error = get_angle_difference(self.desired_heading, self.current_heading)

        # thrust_val = self.pid(-error)

        # print(error)

        # thrust_val = self.kp * error

        # self.send_thrust(thrust_val)

        desired_thrust = np.array([[0], [0], [200]])
        
        self.send_thrust(desired_thrust)
        pass

    def linearise_thrust(self, val):
        val = val/250
        max_thrust = 250
        min_thrust = -100
        if val > 0:
            return val
        else:
            return  max(-1, min(1, val * abs(max_thrust)/abs(min_thrust)))

    def send_thrust(self, val):

        d1 = 1.0027135
        d2 = 1.2
        d3 = 1.0

        thrust_val = val 

        theta_0 = 1.5707
        theta_1 = 1.5707


        thrust_matrix = np.array(
            [
                [1, 1, 0, 0],
                [0, 0, 1, -1],
                [d1, -d1, -d2, d2]
            ])

            

        thrust_matrix_inv = np.linalg.pinv(thrust_matrix)

        thrusts = thrust_matrix_inv @ thrust_val


        print(thrusts)

        f_bl = thrusts[0,0]
        f_br = thrusts[1,0]
        f_fl = thrusts[2,0]
        f_fr = thrusts[3,0]
        self.right_front_pub.publish(self.linearise_thrust(f_fr))
        self.left_front_pub.publish(self.linearise_thrust(f_fl))

        self.right_rear_pub.publish(self.linearise_thrust(f_br))
        self.left_rear_pub.publish(self.linearise_thrust(f_bl))

    
    def start(self):
        r = rospy.Rate(FREQ)
        CTRL_TIME = 1000
        st = time.perf_counter()

        while time.perf_counter() - st < CTRL_TIME:
            self.run_control_loop()
            r.sleep()

        # print(self.time_data)
        print(self.heading_data)

if __name__ == "__main__":
    sphc = SimplePIDHeadingController()
    sphc.start()