#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32, Empty
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from simple_pid import PID
import time
import sys

from scipy.spatial.transform import Rotation as R




class WAMVController():

    previous_time = None
    twist: Twist

    desired_twist: Twist

    # a = 0.05
    a = 1

   


    def __init__(self):
        self.desired_twist = Twist()
        self.twist = Twist()
        right_front_topic = "/wamv/thrusters/right_front_thrust_cmd"
        right_rear_topic = "/wamv/thrusters/right_rear_thrust_cmd"
        left_front_topic = "/wamv/thrusters/left_front_thrust_cmd"
        left_rear_topic = "/wamv/thrusters/left_rear_thrust_cmd"
        self.right_front_pub = rospy.Publisher(right_front_topic, Float32, queue_size = 10)
        self.right_rear_pub = rospy.Publisher(right_rear_topic, Float32, queue_size = 10)
        self.left_front_pub = rospy.Publisher(left_front_topic, Float32, queue_size = 10)
        self.left_rear_pub = rospy.Publisher(left_rear_topic, Float32, queue_size = 10)

        rospy.init_node('twist_controller', anonymous=True)

        rospy.Subscriber("wamv/odom", Odometry, self.receive_odom)

        rospy.Subscriber("wamv/cmd_vel", Twist, self.receive_cmd_vel)

        rospy.Subscriber("control_shutdown", Empty, self.control_shutdown)

        self.debug_pub = rospy.Publisher("debug", Float32, queue_size=10)
        self.time_data = []
        self.vel_data = []
        self.angular_controller()

        


    def control_shutdown(self, cb):
        print("SHUTTING DOWN")
        rospy.signal_shutdown("ASKED TO SHUTDOWN")
        
    def receive_odom(self, odom: Odometry):

        # Apply a low pass
        if self.twist is None or self.previous_time is None:
            self.twist = odom.twist.twist
            self.previous_time = time.time()
            return

        dt = time.time() - self.previous_time
        previous_time = time.time()

        # self.twist = Twist()

        # just do low pass on x and y for now

        self.twist.angular = odom.twist.twist.angular
        self.twist.linear.z = odom.twist.twist.linear.z

        rc = dt*((1-self.a)/self.a)

        self.twist.linear.x = odom.twist.twist.linear.x * (dt / (rc + dt)) + self.twist.linear.x * (rc / (rc + dt))
        self.twist.linear.y = odom.twist.twist.linear.y * (dt / (rc + dt)) + self.twist.linear.y * (rc / (rc + dt))

        # self.debug_pub.publish(self.twist.linear.x)

    def receive_cmd_vel(self, cmd_vel: Twist):
        self.desired_twist = cmd_vel

    def command_thrust2(self, fx, fy, T):
        d1 = 1.0027135
        d2 = 1.2
        d3 = 1.0

        thrust_val = np.array([[fx], [fy], [T]])

        theta_0 = 1.5707
        theta_1 = 1.5707


        thrust_matrix = np.array(
            [
                [1, 1, 0],
                [0, 0, 1,],
                [-d1, d1, d2]
            ])

        thrust_matrix_inv = np.linalg.inv(thrust_matrix)

        thrusts = thrust_matrix_inv @ thrust_val

        min_cap = 20

        f_b = thrusts[2,0]

        f_fl = f_b/2
        f_fr = -f_b/2

        if abs(thrusts[0,0]) > min_cap:
            f_bl = thrusts[0,0]
        else: 
            f_bl = 0
        if abs(thrusts[1,0]) > min_cap:
            f_br = thrusts[1,0]
        else: 
            f_br = 0
        # if abs(thrusts[2,0]) > min_cap:
        #     f_fl = thrusts[2,0]
        # else: 
        #     f_fl = 0
        # if abs(thrusts[3,0]) > min_cap:
        #     f_fr = thrusts[3,0]
        # else: 
        #     f_fr = 0
        
        # f_br = thrusts[1,0]
        # f_fl = thrusts[2,0]
        # f_fr = thrusts[3,0]

    

        
        self.right_front_pub.publish(self.linearise_thrust(f_fr))
        self.left_front_pub.publish(self.linearise_thrust(f_fl))

        self.right_rear_pub.publish(self.linearise_thrust(f_br))
        self.left_rear_pub.publish(self.linearise_thrust(f_bl))
        pass


    def command_thrust(self, fx, fy, T):
        d1 = 1.0027135
        d2 = 1.2
        d3 = 1.0

        thrust_val = np.array([[fx], [fy], [T]])

        theta_0 = 1.5707
        theta_1 = 1.5707


        thrust_matrix = np.array(
            [
                [1, 1, 0, 0],
                [0, 0, 1, -1],
                [-d1, d1, -d2, d2]
            ])

        thrust_matrix_inv = np.linalg.pinv(thrust_matrix)

        thrusts = thrust_matrix_inv @ thrust_val

        min_cap = 20

        if abs(thrusts[0,0]) > min_cap:
            f_bl = thrusts[0,0]
        else: 
            f_bl = 0
        if abs(thrusts[1,0]) > min_cap:
            f_br = thrusts[1,0]
        else: 
            f_br = 0
        if abs(thrusts[2,0]) > min_cap:
            f_fl = thrusts[2,0]
        else: 
            f_fl = 0
        if abs(thrusts[3,0]) > min_cap:
            f_fr = thrusts[3,0]
        else: 
            f_fr = 0
        
        # f_br = thrusts[1,0]
        # f_fl = thrusts[2,0]
        # f_fr = thrusts[3,0]

    

        
        self.right_front_pub.publish(self.linearise_thrust(f_fr))
        self.left_front_pub.publish(self.linearise_thrust(f_fl))

        self.right_rear_pub.publish(self.linearise_thrust(f_br))
        self.left_rear_pub.publish(self.linearise_thrust(f_bl))
        pass

    def linearise_thrust(self, val):
        val = val/250
        max_thrust = 250
        min_thrust = -100
        if val > 0:
            return val
        else:
            return  max(-1, min(1, val * abs(max_thrust)/abs(min_thrust)))
    def angular_controller(self):
        freq = 30
        r = rospy.Rate(freq)

        angle_pid = PID(7000.0, 2000, 0.0, setpoint=0)
        angle_pid.sample_time = 1/freq
        angle_pid.output_limits = (-10000, 10000)

        x_pid = PID(200.0, 120, 5, setpoint=0)
        x_pid.sample_time = 1/freq
        x_pid.output_limits = (-10000, 10000)
        
        y_pid = PID(200.0, 120, 10, setpoint=0)
        y_pid.sample_time = 1/freq
        y_pid.output_limits = (-10000, 10000)
        st = time.perf_counter()

        while not rospy.is_shutdown():
            desired_angle_rate = self.desired_twist.angular.z
            desired_angle_rate = 0.5

            desired_vel = (self.desired_twist.linear.x, self.desired_twist.linear.y)

            desired_vel = (0, 0.0)

            current_vel = (self.twist.linear.x, self.twist.linear.y)
            
            current_rot_speed = self.twist.angular.z

            angle_pid.setpoint = desired_angle_rate
            v = angle_pid(current_rot_speed)
            #v = 0

            x_pid.setpoint = desired_vel[0]
            _x = x_pid(current_vel[0])
            # print(_x)
            # x = 0
            self.time_data.append(time.perf_counter() - st)

            self.vel_data.append(current_rot_speed)

            y_pid.setpoint = desired_vel[1]
            _y = y_pid(current_vel[1])
            self.debug_pub.publish(_y/400)
            # print(_x, _y, v)
            # print(desired_angle_rate - current_rot_speed)
            # self.command_thrust(_x, _y, v)
            self.command_thrust(_x, _y, v)

            
            # _y = 0


            # theta = self.right_front_controller.angle_to_base + math.pi/2
            # x = math.cos(theta) * v + _x 
            # y = math.sin(theta) * v + _y

            # self.right_front_controller.thrust_xy(x, y)

            # theta = self.left_front_controller.angle_to_base + math.pi/2
            # x = math.cos(theta) * v + _x 
            # y = math.sin(theta) * v + _y
            # self.left_front_controller.thrust_xy(x, y)

            # theta = self.right_rear_controller.angle_to_base + math.pi/2
            # x = math.cos(theta) * v + _x 
            # y = math.sin(theta) * v + _y
            # self.right_rear_controller.thrust_xy(x, y)

            # theta = self.left_rear_controller.angle_to_base + math.pi/2
            # x = math.cos(theta) * v + _x 
            # y = math.sin(theta) * v + _y
            # self.left_rear_controller.thrust_xy(x, y)
            r.sleep()

            if time.perf_counter() -st > 10:
                print(self.time_data)
                print(self.vel_data)
                return

    @staticmethod
    def get_rot_matrix(theta):
        return np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])
    

if __name__ == "__main__":
    a = WAMVController()