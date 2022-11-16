#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32, Empty
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from simple_pid.pid import PID
import time
from geometry_msgs.msg import Twist, Pose, PoseStamped

import sys

from scipy.spatial.transform import Rotation as R

from constants import *


MAX_X = 2.0
MAX_Y = 0.4
MAX_R = 0.2


def difference_between_angles(a, b):
    difference = min((a-b) % (2*math.pi), -((b-a) % (2*math.pi)), key=abs)
    return difference

# used for simple_pid
def pi_clip(angle):
    if angle>0:
        if angle>math.pi:
            return angle - 2*math.pi
    else:
        if angle< -math.pi:
            return angle + 2*math.pi

    return angle



class WAMVController():

    previous_time = None
    twist: Twist

    desired_twist: Twist

    # a = 0.05
    a = 1

    prev_1 = 0
    prev_2 = 0
    prev_3 = 0
    prev_4 = 0

    initial_pose = None

    current_pose = None

    desired_pose = None


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

        # rospy.Subscriber("wamv/odom", Odometry, self.receive_odom)

        # rospy.Subscriber("wamv/cmd_vel", Twist, self.receive_cmd_vel)


        rospy.Subscriber("/wamv/odom", Odometry, self.receive_odom)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.receive_desired_pose)

        rospy.Subscriber("control_shutdown", Empty, self.control_shutdown)

        self.debug_pub = rospy.Publisher("debug", Float32, queue_size=10)
        self.time_data = []
        self.vel_data = []

        x = np.arange(501)/500
        A = 0.01
        K = 59.82
        B = 5.0
        v = 0.38
        C = 0.56
        M = 0.28
        T = (A + (K-A)/np.power(C+np.exp(-B*(x-M)),1/v) ) / 250
        self.fwd_thrust_table = list(zip(list(x), list(T)))

        x = np.arange(501)/-500 
        A = -199.13
        K = -0.09
        B = 8.84
        v = 5.34
        C = 0.99
        M = -0.57
        T = (A + (K-A)/np.power(C+np.exp(-B*(x-M)),1/v) ) / 100

        self.back_thrust_table = list(zip(list(x), list(T)))

        # print(self.back_thrust_table)

        # self.angular_controller()

        self.position_control_loop()


        fwd_thrust_table = []



    def control_shutdown(self, cb):
        print("SHUTTING DOWN")
        rospy.signal_shutdown("ASKED TO SHUTDOWN")
        
    def receive_odom(self, odom: Odometry):
        # print(odom)

        self.current_pose = odom.pose.pose

    def receive_desired_pose(self, pose):
        print("received", pose)
        self.desired_pose = pose.pose

    def command_thrust(self, fx, fy, T):

        # print(fx, fy, T)
        
        d1 = 1.0027135
        d2 = 1.2
        d3 = 1.0

        thrust_val = np.array([[fx], [fy], [T]])

        thrust_matrix = np.array(
            [
                [1, 1, 0],
                [0, 0, 1,],
                [-d1, d1, d2]
            ])

        thrust_matrix_inv = np.linalg.inv(thrust_matrix)

        thrusts = thrust_matrix_inv @ thrust_val

        min_cap = 0
        bow_min_cap = 0

        f_b = thrusts[2,0]

        f_fl = f_b/2
        f_fr = -f_b/2



        
        # d1 = 1.0027135
        # d2 = 1.2
        # d3 = 1.0

        # thrust_val = np.array([[fx], [fy], [T]])

        # theta_0 = 1.5707
        # theta_1 = 1.5707


        # thrust_matrix = np.array(
        #     [
        #         [1, 1, 0, 0],
        #         [0, 0, 1, -1],
        #         [-d1, d1, d2, -d2]
        #     ])

        

        # thrust_matrix_inv = np.linalg.pinv(thrust_matrix)

        # thrusts = thrust_matrix_inv @ thrust_val

        # min_cap = 20

        if abs(thrusts[0,0]) > min_cap:
            f_bl = thrusts[0,0]
        else: 
            f_bl = 0
        if abs(thrusts[1,0]) > min_cap:
            f_br = thrusts[1,0]
        else: 
            f_br = 0
        if abs(thrusts[2,0]) > min_cap:
            f_fl = thrusts[2,0]/2
        else: 
            f_fl = 0
        if abs(thrusts[2,0]) > min_cap:
            f_fr = -thrusts[2,0]/2
        else: 
            f_fr = 0
        
        # f_br = thrusts[1,0]
        # f_fl = thrusts[2,0]
        # f_fr = thrusts[3,0]

        # print(f"{f_fr:.2f}, {f_fl:.2f}, {f_br:.2f}, {f_bl:.2f}")

    

        fr = self.linearise_thrust_bow(f_fr)
        fl = self.linearise_thrust_bow(f_fl)
        br = self.linearise_thrust(f_br)
        bl = self.linearise_thrust(f_bl)

        # print(f"{fr:.2f}, {fl:.2f}, {br:.2f}, {bl:.2f}")
        
        a = 0.05
        fr = fr*(a) + self.prev_1*(1-a)
        fl = fl*(a) + self.prev_2*(1-a)
        br = br*(a) + self.prev_3*(1-a)
        bl = bl*(a) + self.prev_4*(1-a)


        min_1 = 0.3
        min_2 = 0
        min_3 = 0
        min_4 = 0

        if abs(fr)<min_1:
            # print("fr", fr)
            self.prev_1 = fr
            fr = 0
            self.right_front_pub.publish(fr)
            # self.prev_1 = fr

        else:


            self.right_front_pub.publish(fr)
            self.prev_1 = fr

        self.left_front_pub.publish(fl)
        self.right_rear_pub.publish(br)
        self.left_rear_pub.publish(bl)

        
        self.prev_2 = fl
        self.prev_3 = br
        self.prev_4 = bl

        print(f"Thrust motors {fr:.2f}, {fl:.2f}, {br:.2f}, {bl:.2f}")
        # self.right_front_pub.publish(f_fr)
        # self.left_front_pub.publish(f_fl)
        # self.right_rear_pub.publish(f_br)
        # self.left_rear_pub.publish(f_bl)
        pass

    def linearise_thrust(self, thrust):
        
        # thrust = val/250
        max_thrust = 500
        min_thrust = -400
        # if val > 0:
        #     return val
        # else:
        #     return  max(-1, min(1, val * abs(max_thrust)/abs(min_thrust)))
        cmd = self.linearise(thrust, max_thrust, min_thrust)
        return max(-1, min(1, cmd))


    def linearise_thrust_bow(self, thrust):
        # val = val/125
        max_thrust = 90
        min_thrust = -90

        cmd = self.linearise(thrust, max_thrust, min_thrust)


        return max(-1, min(1, cmd))

        # if val > 0:
        #     return val
        # else:
        #     return  max(-1, min(1, val * abs(max_thrust)/abs(min_thrust)))

    def linearise(self, thrust, max_thrust, min_thrust):

        if thrust > 0:
            _thrust = thrust/max_thrust
            for x, t_thrust in self.fwd_thrust_table:
                if t_thrust == self.fwd_thrust_table[-1][1] or t_thrust > _thrust:
                    return x
        elif thrust < 0:
            _thrust = -thrust/min_thrust

            for x, t_thrust in self.back_thrust_table:
                if t_thrust == self.back_thrust_table[-1][1] or t_thrust < _thrust:
                    return x 
        else:
            return 0

    def position_control_loop(self, freq=50):
        print("doing control")
        r = rospy.Rate(freq)
        max_vel = 1  # m/s
        max_rot = 0.4  # rot/s


        angle_pid = PID(200, 0.00, 10, setpoint=0)
        angle_pid.sample_time = 1/freq
        angle_pid.error_map = pi_clip

        angle_pid.output_limits = (-6000, 6000)

        x_pid = PID(20, 0.00, 10, setpoint=0)
        x_pid.sample_time = 1/freq
        
        y_pid = PID(20, 0.00, 10, setpoint=0)
        y_pid.sample_time = 1/freq

        x_pid.output_limits = (-600, 600)

        y_pid.output_limits = (-600, 600)
        st = None

        time_data = []

        pos_data = []
        while not rospy.is_shutdown():

            print("doing control")

            if self.current_pose is None or self.desired_pose is None:

                # print("is none", self.current_pose, self.desired_pose)
                time.sleep(1/20)
                continue
            
            pass
            if st is None:
                st = time.perf_counter()

            # time_data.append(time.perf_counter() - st)


            
            desired_xyz = np.array([self.desired_pose.position.x, self.desired_pose.position.y, self.desired_pose.position.z])
            current_xyz = np.array([self.current_pose.position.x,self.current_pose.position.y, self.current_pose.position.z])

            # d_xy = current_xyz[0:2] - desired_xyz[0:2]

            desired_quat = np.array([self.desired_pose.orientation.x, self.desired_pose.orientation.y, self.desired_pose.orientation.z, self.desired_pose.orientation.w])
            desired_orient_z = R.from_quat(desired_quat).as_euler('xyz')[2]

            curr_quat = np.array([self.current_pose.orientation.x,self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            current_orient = R.from_quat(curr_quat)
            curr_orient_z = current_orient.as_euler('xyz')[2]

            # self.debug_pub.publish(pi_clip(desired_orient_z- curr_orient_z))

            

            # d_rz = difference_between_angles(curr_orient_z, desired_orient_z)

            d_xy = current_xyz - desired_xyz

            diff_xyz = current_orient.inv().apply(d_xy)

            

            # desired_xyz_wamv_frame = current_orient.apply(desired_xyz)

            # print(diff_xyz)


            angle_pid.setpoint = desired_orient_z
            angular_vel_z = angle_pid(curr_orient_z)

            # pos_data.append((diff_xyz[0], diff_xyz[1], difference_between_angles(desired_orient_z,curr_orient_z)))
            diff_xyz[0] = 0
            diff_xyz[1] = 0.0

            x_pid.setpoint = 0
            x_vel = x_pid(diff_xyz[0])

            # self.debug1_pub.publish(diff_xyz[0])

            

            y_pid.setpoint = 0
            y_vel = y_pid(diff_xyz[1])

            # self.debug2_pub.publish(diff_xyz[1])

            # print(x_vel, y_vel)
            dist_thresh = 0.05
            angle_thresh = 0.05
            if abs(diff_xyz[0]) < dist_thresh and abs(diff_xyz[1]) < dist_thresh and abs(difference_between_angles(desired_orient_z,curr_orient_z)) < angle_thresh:
                
                # print(diff_xyz[0])
                # print(difference_between_angles(desired_orient_z,curr_orient_z))
                self.command_thrust(0, 0, 0)
                print("Zero Vel")
            else:
                cmd_vel = Twist()
                cmd_vel.linear.x = x_vel
                cmd_vel.linear.y = y_vel
                cmd_vel.angular.z = angular_vel_z
                print(f"Forces: {x_vel:.2f}, {x_vel:.2f}, {angular_vel_z:.2f}")

                # x_vel = 700
                # y_vel = 0
                # angular_vel_z = 0
                self.command_thrust(x_vel, y_vel, angular_vel_z)

                # print("Cmd vel", cmd_vel)
                # self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(1/20)



    def angular_controller(self):
        freq = 30
        rate = rospy.Rate(freq)

        angle_pid = PID(300.0, 0, 0.0, setpoint=0)
        angle_pid.sample_time = 1/freq
        angle_pid.output_limits = (-10000, 10000)

        x_pid = PID(140.0, 0, 30, setpoint=0)
        x_pid.sample_time = 1/freq
        x_pid.output_limits = (-1000, 1000)
        
        y_pid = PID(140.0, 0, 30, setpoint=0)
        y_pid.sample_time = 1/freq
        y_pid.output_limits = (-1000, 1000)
        st = time.perf_counter()


        while not rospy.is_shutdown():
            desired_angle_rate = max(min(MAX_R, self.desired_twist.angular.z), -MAX_X)
            # desired_angle_rate = 0.4

            desired_vel = (self.desired_twist.linear.x, self.desired_twist.linear.y)

            # desired_vel = (0.0, 0.4)

            current_vel = (self.twist.linear.x, self.twist.linear.y)

            # print(current_vel)
            
            current_rot_speed = self.twist.angular.z

            angle_pid.setpoint = desired_angle_rate

            self.time_data.append(time.perf_counter() - st)

            self.vel_data.append((self.twist.linear.x, self.twist.linear.y, current_rot_speed))

            u_d = desired_vel[0]
            v_d = desired_vel[1]
            r_d = desired_angle_rate

            u = current_vel[0]
            v = current_vel[1]
            r = current_rot_speed


            k_u = 0.5
            k_a_max = 0.3
            u_dot_a_max = 0.4


            u_dot_d = u_dot_a_max * math.tanh(k_a_max * (u_d - u) / u_dot_a_max)
            # F_x = m_11*(u_dot_d+ k_u*(u_d - u)) -(m_22*v*r-d_11*u)

            k_v = 0.2
            k_v_max = 0.3
            v_dot_a_max = 0.2
            v_dot_d = v_dot_a_max * math.tanh(k_v_max * (v_d - v) / v_dot_a_max)

            # F_y = m_22*(v_dot_d + k_v*(v_d - v)) - (-m_11*u*r - d_22*v)

            k_r = 0.5
            k_r_max = 0.3
            r_dot_a_max = 0.2

            r_dot_d = r_dot_a_max * math.tanh(k_r_max * (r_d - r) / r_dot_a_max)

            # T_z = m_33*(r_dot_d + k_r * (r_d - r)) - ((m_11 - m_22)*u*v - d_33 * r)
            v = angle_pid(current_rot_speed)
            # #v = 0

            x_pid.setpoint = desired_vel[0]
            _x = x_pid(current_vel[0])
            # # print(_x)
            # # x = 0

   

            y_pid.setpoint = desired_vel[1]
            _y = y_pid(current_vel[1])
            # self.debug_pub.publish(_y/400)
            # print("what,",_x, _y, v)
            # print(desired_angle_rate - current_rot_speed)

            
            if abs(desired_vel[0]) <0.01:
                if abs(current_vel[0]) < 0.1:
                    _x = 0
                    x_pid.reset()

            if abs(desired_vel[1]) <0.01:
                if abs(current_vel[1]) < 0.1:
                    _y = 0
                    y_pid.reset()
            if abs(desired_angle_rate) <0.01:
                if abs(current_rot_speed) < 0.1:
                    v = 0
                    angle_pid.reset()

            # _x = 0
            # _y = 80
            # v = 0
            print(f"Forces: {_x:.2f}, {_y:.2f}, {v:.2f}")

            self.command_thrust(_x, _y, v)
            # print(f"After")

            # print(F_x, F_y, T_z)
            # self.command_thrust(F_x, F_y, T_z)


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
            time.sleep(1/30)
        
            # if time.perf_counter() -st > 10:
            #     print(self.time_data)
            #     print(self.vel_data)
            #     return

            # if time.perf_counter() -st > 10:
            #     print(self.time_data)
            #     print(self.vel_data)
            #     return

    @staticmethod
    def get_rot_matrix(theta):
        return np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])
    

if __name__ == "__main__":
    a = WAMVController()