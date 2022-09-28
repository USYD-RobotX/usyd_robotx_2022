import os
import sys
# sys.path.insert(0, '/home/auv/git/acfr-lcm/build/lib/python2.7/dist-packages/perls/lcmtypes')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../LCM/acfr/lcmtypes'))
from scipy.spatial.transform import Rotation

import lcm

from acfrlcm import auv_acfr_nav_t

import rospy

from nav_msgs.msg import Odometry

class NavDataBridge():
    
    def __init__(self):

        print("Running the LCM NAV Data Bridge")
        rospy.init_node("nav_data_bridge")

        gps_odom_topic = "gps_odom"

        self.gps_odom_pub = rospy.Publisher(gps_odom_topic, Odometry, queue_size=10)

        self.lc = lcm.LCM()
        subscription = self.lc.subscribe("WAMV.ACFR_NAV", self.lcm_nav_handler)

    def lcm_nav_handler(self, channel, data):
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



    def start(self):
        while not rospy.is_shutdown():
            self.lc.handle()


if __name__ == "__main__":
    nvb = NavDataBridge()
    nvb.start()