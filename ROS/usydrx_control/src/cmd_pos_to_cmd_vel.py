#!/usr/bin/env python3

from calendar import c
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from simple_pid import PID

import numpy as np
import math
from std_msgs.msg import Float32

import time



from scipy.spatial.transform import Rotation as R


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

class PositionController():

    initial_pose = None

    current_pose = None

    desired_pose = None

    def __init__(self):
        rospy.init_node('position_controller', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.receive_odom)

        rospy.Subscriber("desired_pose", Pose, self.receive_desired_pose)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.debug1_pub = rospy.Publisher("debug1", Float32, queue_size=10)

        self.debug2_pub = rospy.Publisher("debug2", Float32, queue_size=10)

        

    def receive_odom(self, odom: Odometry):
        # print(odom)

        self.current_pose = odom.pose.pose

    def receive_desired_pose(self, pose: Pose):
        print("received", pose)
        self.desired_pose = pose

    def position_control_loop(self, freq=50):

        r = rospy.Rate(freq)
        max_vel = 1  # m/s
        max_rot = 0.4  # rot/s


        angle_pid = PID(0.3, 0.00, 0.2, setpoint=0)
        angle_pid.sample_time = 1/freq
        angle_pid.error_map = pi_clip

        angle_pid.output_limits = (-max_rot, max_rot)

        x_pid = PID(0.3, 0.00, 0.2, setpoint=0)
        x_pid.sample_time = 1/freq
        
        y_pid = PID(0.3, 0.00, 0.4, setpoint=0)
        y_pid.sample_time = 1/freq

        x_pid.output_limits = (-max_vel, max_vel)

        y_pid.output_limits = (-max_vel, max_vel)
        st = None

        time_data = []

        pos_data = []
        while not rospy.is_shutdown():
            if self.current_pose is None or self.desired_pose is None:
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


            x_pid.setpoint = 0
            x_vel = x_pid(diff_xyz[0])

            self.debug1_pub.publish(diff_xyz[0])

            

            y_pid.setpoint = 0
            y_vel = y_pid(diff_xyz[1])

            # self.debug2_pub.publish(diff_xyz[1])


            # print(x_vel, y_vel)
            dist_thresh = 0.2
            angle_thresh = 0.05
            if abs(diff_xyz[0]) < dist_thresh and abs(diff_xyz[1]) < dist_thresh and abs(difference_between_angles(desired_orient_z,curr_orient_z)) < angle_thresh:
                
                print(diff_xyz[0])

                print(difference_between_angles(desired_orient_z,curr_orient_z))


                cmd_vel = Twist()
                cmd_vel.linear.x = 0

                cmd_vel.linear.y = 0

                cmd_vel.angular.z = 0

                self.cmd_vel_pub.publish(cmd_vel)
                print("Close enough")
            else:
                cmd_vel = Twist()
                cmd_vel.linear.x = x_vel

                cmd_vel.linear.y = y_vel

                cmd_vel.angular.z = angular_vel_z

                # print("Cmd vel", cmd_vel)
                self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(1/20)


            # if time.perf_counter() -st > 20:
            #     print(time_data)
            #     print(pos_data)
            #     return


            ##################################################            
            # init_quat = np.array([self.initial_pose.orientation.x, self.initial_pose.orientation.y, self.initial_pose.orientation.z, self.initial_pose.orientation.w])
            # init_rot = R.from_quat(init_quat)
            # init_orient_z = init_rot.as_euler('xyz')[2]

            # current_quat = np.array([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
            # current_rot = R.from_quat(current_quat)

            # curr_orient_z = current_rot.as_euler('xyz')[2]

            # # print(curr_orient_z - init_orient_z)


            # # TODO need to do angle wrapping
            # d_rz = curr_orient_z - init_orient_z
            # a = curr_orient_z
            # b = init_orient_z

            # difference = min((a-b) % (2*math.pi), -((b-a) % (2*math.pi)), key=abs)
            # d_rz = difference
            
            # print(d_rz)


            # # TODO Rotate dx dy into frame of boat.

            # p_xy = 0.5

            # p_rz = 0.5

            # d_xy = d_xy * p_xy

            # d_rz = d_rz * p_rz

            # cmd_vel = Twist()

            # # cmd_vel.linear.x = d_xy[0]

            # # cmd_vel.linear.y = d_xy[1]

            # cmd_vel.angular.z = -d_rz

            # self.cmd_vel_pub.publish(cmd_vel)





    pass

if __name__ == "__main__":

    pc = PositionController()

    pc.position_control_loop()