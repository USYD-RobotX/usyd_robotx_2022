#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
import math
import tf

from geometry_msgs.msg import Pose, PoseStamped
# from geometry_msgs.msg import Twist
import numpy as np

import time

from typing import List

from scipy.spatial.transform import Rotation as R

def difference_between_angles(a, b):
    difference = min((a-b) % (2*math.pi), -((b-a) % (2*math.pi)), key=abs)
    return difference


RADIUS_THRESHOLD = 4  #m

THETA_THRESHOLD = math.radians(45)  # 45 degrees


class PathFollower():

    current_pose = None
    
    def __init__(self):

        rospy.init_node('path_follower', anonymous=True)
        
        self.current_path: List[Pose] = []

        self.desired_pose_pub = rospy.Publisher("desired_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.receive_odom)
        

        rospy.Subscriber("desired_path", Path, self.receive_desired_path)


        pass
    def receive_odom(self, odom: Odometry):
        self.current_pose = odom.pose.pose

    def receive_desired_path(self, path: Path):

        # print("RECEIVED DESIRE PATH", path)

        self.current_path = []

        for pose_stamped in path.poses:
            self.current_path.append(pose_stamped.pose)

        print(self.current_path)
        # if just one path, just go to it.

        pass

    def control_desired_pose_loop(self):
        
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            if len(self.current_path) == 0 or self.current_pose is None:
                r.sleep()
                continue
            

            desired_pose = self.current_path[0]
            current_pose_xy = np.array([self.current_pose.position.x, self.current_pose.position.y])


            curr_quat = np.array([self.current_pose.orientation.x,self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            current_orient = R.from_quat(curr_quat)
            curr_rot = current_orient.as_euler('xyz')[2]

            desired_xy = np.array([desired_pose.position.x, desired_pose.position.y])
            desired_quat = np.array([desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w])

            desired_orient = R.from_quat(desired_quat)
            desired_rot = desired_orient.as_euler('xyz')[2]

            d_xy = current_pose_xy - desired_xy

            distance = np.linalg.norm(d_xy)

            diff_rot = abs(difference_between_angles(curr_rot, desired_rot))
            
            if distance < RADIUS_THRESHOLD and diff_rot < THETA_THRESHOLD:
                if len(self.current_path) == 1:

                    dp = PoseStamped()
                    dp.header.frame_id = "map"
                    dp.header.stamp = rospy.Time.now()
                    dp.pose = self.current_path[0]

                    self.desired_pose_pub.publish(dp)
                del self.current_path[0]
                # print("DELETE FIRST PATH")
                continue
            

            
            # Go to path
            # print(f"PUBLISHING {desired_pose}")
            dp = PoseStamped()
            dp.header.frame_id = "map"
            dp.header.stamp = rospy.Time.now()
            dp.pose = desired_pose

            self.desired_pose_pub.publish(dp)

            r.sleep()
            # current_pose_rot = self.current_pose.
            

            
            # 1. Check if within threshold.
            # if so, continue and go to the next waypoint.


if __name__ == "__main__":
    pf = PathFollower()

    pf.control_desired_pose_loop()