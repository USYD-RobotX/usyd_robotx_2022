#!/usr/bin/env python3

import os
import sys
# sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../LCM/acfr/lcmtypes'))
from scipy.spatial.transform import Rotation
import numpy as np
import lcm

from acfrlcm import auv_acfr_nav_t
from senlcm import xsens_t
from senlcm import gpsd3_t

from acfrlcm import relay_status_t, relay_command_t


import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header, String
import math

RELAY_LIGHT_RED = 14
RELAY_LIGHT_AMBER= 21
RELAY_LIGHT_GREEN=22

class NavDataBridge():
    
    def __init__(self):

        print("Running the LCM NAV Data Bridge")
        rospy.init_node("nav_data_bridge")

        gps_odom_topic = "gps_odom"
        self.heading = 0.

        self.gps_odom_pub = rospy.Publisher(gps_odom_topic, Odometry, queue_size=10)
        self.gps_fix_pub = rospy.Publisher("fix", NavSatFix, queue_size=1)
        self.imu_pub = rospy.Publisher("xsens_imu", Imu, queue_size=1)
        self.h_pub = rospy.Publisher("gps_heading", Imu, queue_size=1)
        self.relay_pub = rospy.Publisher("/relay_state", String, queue_size=1)
        

        self.lc = lcm.LCM()
        # subscription = self.lc.subscribe("WAMV.ACFR_NAV", self.lcm_nav_handler)

        gpsd_sub = self.lc.subscribe("WAMV.GPSD_CLIENT", self.lcm_gps_handler)
        isub = self.lc.subscribe("WAMV.XSENS", self.xsens_handler)
        relay_sub = self.lc.subscribe('WAMV.RELAY_CONTROL', self.handle_relay_status)

    def handle_relay_status(self, channel_name, data):
        smsg = String()
        msg = relay_command_t.decode(data)
        if msg.relay_number == RELAY_LIGHT_RED:
            if msg.relay_request == 0:
                # print("RED")
                smsg.data = "RED"
                self.relay_pub.publish(smsg)
        elif msg.relay_number == RELAY_LIGHT_AMBER:
            if msg.relay_request == 1:
                # print("AMBER")
                smsg.data = "AMBER"
                self.relay_pub.publish(smsg)
        elif msg.relay_number == RELAY_LIGHT_GREEN:
            if msg.relay_request == 1:
                # print("GREEN")
                smsg.data = "GREEN"
                self.relay_pub.publish(smsg)

        
    

    def xsens_handler(self, channel, data):
        msg = xsens_t.decode(data)

        imsg = Imu()
        imsg.header.stamp = rospy.Time.now()
        imsg.header.frame_id = 'xsens_imu'
        heading_enu = self.unwrap(-(msg.heading - np.pi/2))
        quat = Rotation.from_euler('xyz', [msg.pitch, msg.roll, heading_enu]).as_quat()

        imsg.orientation.x = quat[0]
        imsg.orientation.y = quat[1]
        imsg.orientation.z = quat[2]
        imsg.orientation.w = quat[3]

        imsg.angular_velocity.x = msg.gyr_y
        imsg.angular_velocity.y = msg.gyr_x
        imsg.angular_velocity.z = -msg.gyr_z

        imsg.linear_acceleration.x = msg.acc_y
        imsg.linear_acceleration.y = msg.acc_x
        imsg.linear_acceleration.z = -msg.acc_z

        self.imu_pub.publish(imsg)

    prev_heading = 0
    def lcm_gps_handler(self, channel, data):
        msg = gpsd3_t.decode(data) 

        dn = msg.ned.relPosN
        de = msg.ned.relPosE

        heading_enu = math.atan2(dn, de) - math.pi/2

        if heading_enu == self.prev_heading:
            return

        self.prev_heading = heading_enu

        imu_msg = Imu()
        imu_msg.header.frame_id = "novatel"
        imu_msg.header.stamp = rospy.Time.now()
        q2 = Rotation.from_euler('xyz', [0., 0., heading_enu]).as_quat()
        imu_msg.orientation.x = q2[0]
        imu_msg.orientation.y = q2[1]
        imu_msg.orientation.z = q2[2]
        imu_msg.orientation.w = q2[3]
        self.h_pub.publish(imu_msg)

        
        fix_msg = NavSatFix()
        fix_msg.header.stamp = rospy.Time.now()
        fix_msg.header.frame_id = "novatel"
        fix_msg.status.status = 0
        fix_msg.status.service = 1

        fix_msg.latitude = msg.fix.latitude * 180. / np.pi
        fix_msg.longitude = msg.fix.longitude * 180 / np.pi
        fix_msg.altitude = 0.
        self.gps_fix_pub.publish(fix_msg)


    def lcm_nav_handler_OLD(self, channel, data):
        msg = auv_acfr_nav_t.decode(data)

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = msg.x
        odom_msg.pose.pose.position.y = msg.y
        odom_msg.pose.pose.position.z = 0

        quat = Rotation.from_euler('xyz', [0, 0, msg.heading]).as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = msg.vx
        odom_msg.twist.twist.linear.y = msg.vy
        odom_msg.twist.twist.linear.y = 0

        odom_msg.twist.twist.angular.x = msg.rollRate
        odom_msg.twist.twist.angular.x = msg.pitchRate
        odom_msg.twist.twist.angular.x = msg.headingRate

        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "gps_odom"
        odom_msg.child_frame_id = 'gps'
        self.gps_odom_pub.publish(odom_msg)




        # print("publishing")

    def unwrap(self, h):
        while h > np.pi:
            h -= 2*np.pi
        while h < -np.pi:
            h += 2*np.pi
        return h

    def lcm_nav_handler(self, channel, data):
        msg = auv_acfr_nav_t.decode(data)

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = msg.y
        odom_msg.pose.pose.position.y = msg.x
        odom_msg.pose.pose.position.z = 0

        heading_enu = self.unwrap(-(msg.heading - np.pi/2))

        quat = Rotation.from_euler('xyz', [msg.pitch, msg.roll, heading_enu]).as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = msg.vy
        odom_msg.twist.twist.linear.y = msg.vx
        odom_msg.twist.twist.linear.z = 0

        odom_msg.twist.twist.angular.x = msg.pitchRate
        odom_msg.twist.twist.angular.x = msg.rollRate
        odom_msg.twist.twist.angular.z = -msg.headingRate

        odom_msg.header.stamp = rospy.Time.now()
        # odom_msg.header.frame_id = "odom_acfr"
        # self.gps_odom_pub.publish(odom_msg)
        # print("publishing")

        # if self.heading != heading_enu:
        #     imu_msg = Imu()
        #     imu_msg.header.frame_id = "novatel"
        #     imu_msg.header.stamp = rospy.Time.now()
        #     q2 = Rotation.from_euler('xyz', [0., 0., heading_enu]).as_quat()
        #     imu_msg.orientation.x = quat[0]
        #     imu_msg.orientation.y = quat[1]
        #     imu_msg.orientation.z = quat[2]
        #     imu_msg.orientation.w = quat[3]
        #     self.h_pub.publish(imu_msg)

        



    

        



    def start(self):
        while not rospy.is_shutdown():
            self.lc.handle()


if __name__ == "__main__":
    nvb = NavDataBridge()
    nvb.start()