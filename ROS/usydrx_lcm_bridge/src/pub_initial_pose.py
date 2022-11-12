#!/usr/bin/env python3

import os
import sys
# sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../LCM/acfr/lcmtypes'))
from scipy.spatial.transform import Rotation
import numpy as np
import lcm
import time

import rospy

from acfrlcm import auv_acfr_nav_t


from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header

from robot_localization.srv import SetPose

class PubInitialPose():
    
    def __init__(self):

        print("Publish initial pose")
        rospy.init_node("init_pose")
        time.sleep(2)
        self.do_exit = False

        self.lc = lcm.LCM()
        subscription = self.lc.subscribe("WAMV.ACFR_NAV", self.lcm_nav_handler)


    def unwrap(self, h):
        while h > np.pi:
            h -= 2*np.pi
        while h < -np.pi:
            h += 2*np.pi
        return h

    def lcm_nav_handler(self, channel, data):
        msg = auv_acfr_nav_t.decode(data)

        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header.frame_id = 'odom'
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = msg.y
        pose_msg.pose.pose.position.y = msg.x
        pose_msg.pose.pose.position.z = 0

        heading_enu = self.unwrap(-(msg.heading - np.pi/2))

        quat = Rotation.from_euler('xyz', [msg.pitch, msg.roll, heading_enu]).as_quat()
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)


        rospy.loginfo("Setting initial pose")
        rospy.wait_for_service('set_pose')
        try:
            set_pose_proxy = rospy.ServiceProxy('set_pose', SetPose)
            success = set_pose_proxy(pose_msg)
            self.do_exit = True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


        sys.exit(0)


    

        



    def start(self):
        while not rospy.is_shutdown():
            self.lc.handle()
            if self.do_exit:
                sys.exit(0)


if __name__ == "__main__":
    pi = PubInitialPose()
    pi.start()