#!/usr/bin/env python3

from calendar import c
from webbrowser import Galeon
import rospy

from geometry_msgs.msg import Twist, Pose, PoseStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPose

import numpy as np
import math
from std_msgs.msg import Float32

# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
# reload(gc)
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
# reload(axy)
import rospy
import tf
from nav_msgs.msg import Odometry


def get_xy_based_on_lat_long(lat,lon, name):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    olat = -33.724223
    olon = 150.679736
    
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

    rospy.loginfo("#########  "+name+"  ###########")  
    rospy.loginfo("LAT COORDINATES ==>"+str(lat)+","+str(lon))  
    rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
    rospy.loginfo("COORDINATES AXY==>"+str(xa)+","+str(ya))
    rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

    return xg2, yg2

class StationGoal():


    goal_pose = None

    def __init__(self):
        rospy.init_node('goal_relayer', anonymous=True)

        rospy.Subscriber("/vrx/station_keeping/goal", GeoPoseStamped, self.receive_desired_pose)

        self.desired_pose_pub = rospy.Publisher("/wamv/desired_pose", Pose, queue_size=10)

    def receive_desired_pose(self, goalGeoPose: GeoPoseStamped):
        print("Geo Pose received", goalGeoPose)

        goal_lat = goalGeoPose.pose.position.latitude
        goal_lon = goalGeoPose.pose.position.longitude
        goal_x, goal_y = get_xy_based_on_lat_long(goal_lat,goal_lon, "Goal XY")

        # Create empty goal_pose to send as station keeping target
        self.goal_pose = Pose()

        # X and Y offsets to fix incorrect lat long to xy conversion
        xOffset = 905.923+0.3
        yOffset = -172.276

        self.goal_pose.position.x = goal_x + xOffset
        self.goal_pose.position.y = goal_y + yOffset
        self.goal_pose.position.z = 0

        self.goal_pose.orientation = goalGeoPose.pose.orientation

        # self.goal_pose = None


    def goal_relayer(self, freq=50):

        r = rospy.Rate(freq)

        while not rospy.is_shutdown():
            if self.goal_pose is None:
                r.sleep()
                continue
            
            print("publish check", self.goal_pose)
            self.desired_pose_pub.publish(self.goal_pose)
            self.goal_pose = None
            r.sleep()


    pass

if __name__ == "__main__":
    sg = StationGoal()

    sg.goal_relayer()
