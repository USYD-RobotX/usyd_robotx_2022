#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Empty

from scipy.spatial.transform import Rotation as R

from simple_pid import PID

import math
import time

import matplotlib.pyplot as plt
plt.style.use(['science','no-latex'])



FREQ=20

def get_angle_difference(a,b):
    difference = min((a-b) % (2*math.pi), -((b-a) % (2*math.pi)), key=abs)
    return difference


class SimplePIDHeadingController:

    kp = 3
    kd = 0
    ki = 0

    l_r = 1.02

    l_f = 1.2

    current_heading = None

    desired_heading = None

    current_dx = None
    current_dy = None

    m_11 = 200
    m_22 = 250
    m_33 = 80

    d_11 = 70
    d_22 = 100

    d_33 = 50



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

        rospy.Subscriber("/wamv/odom", Odometry, self.receive_odom)

        self.pid = PID(3, 0, 1.5, setpoint=0)
        self.pid.sample_time = 1/FREQ
        self.pid.output_limits = (-1, 1)

        self.time_data = []
        self.heading_data = []
        self.st = time.perf_counter()

    def receive_odom(self, odom_msg):
        _odom_quat = odom_msg.pose.pose.orientation
        odom_quat = [_odom_quat.x, _odom_quat.y, _odom_quat.z, _odom_quat.w]
        
        heading = R.from_quat(odom_quat).as_euler('xyz')[2]

        self.current_heading = heading
        self.current_dx = odom_msg.twist.twist.linear.x
        self.current_dy = -odom_msg.twist.twist.linear.y
        self.current_r = -odom_msg.twist.twist.angular.z

        if self.desired_heading is None:
            self.desired_heading = heading + 2

        if self.desired_heading is None or self.current_heading is None:
            pass
        else:
            self.time_data.append(time.perf_counter() - self.st)
            self.heading_data.append(get_angle_difference(self.desired_heading, self.current_heading))

    def run_control_loop(self):
        if self.desired_heading is None or self.current_heading is None:
            return
        

        k_1 = 1.0
        k_2 = 0.5
        error = get_angle_difference(self.desired_heading, self.current_heading)

        torque_val = self.m_33 * ((((self.m_11 - self.m_22) * self.current_dx * self.current_dy - self.d_33* self.current_r)/self.m_33) + (k_1*self.current_r) + k_2*(self.current_r + k_1 * error))

        print(error, torque_val)
       
        # self.pid.setpoint = 0



        # thrust_val = self.pid(-error)

        # print(error)

        # # thrust_val = self.kp * error

        self.send_thrust(torque_val/15)

    def linearise_thrust(self, val):
        max_thrust = 250
        min_thrust = -100
        if val > 0:
            return val
        else:
            return  max(-1, min(1, val * abs(max_thrust)/abs(min_thrust)))

    def send_thrust(self, val):
        val = max(-1, min(1, val))
        self.right_front_pub.publish(self.linearise_thrust(-val) *(self.l_r /self.l_f))
        self.left_front_pub.publish(self.linearise_thrust(val) * (self.l_r /self.l_f))

        self.right_rear_pub.publish(self.linearise_thrust(val))
        self.left_rear_pub.publish(self.linearise_thrust(-val))



    def start(self):
        r = rospy.Rate(FREQ)
        CTRL_TIME = 10
        st = time.perf_counter()

        while time.perf_counter() - st < CTRL_TIME:
            self.run_control_loop()
            r.sleep()

        print(self.time_data)
        print(self.heading_data)


if __name__ == "__main__":
    sphc = SimplePIDHeadingController()
    sphc.start()