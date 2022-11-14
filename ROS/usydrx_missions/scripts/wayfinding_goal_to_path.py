#!/usr/bin/env python3

from calendar import c
from importlib.resources import path
from webbrowser import Galeon
import rospy

from geometry_msgs.msg import Twist, Pose, PoseStamped
from geographic_msgs.msg import GeoPath, GeoPose
from nav_msgs.msg import Path

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

class WayfindingGoal():

    goal_path = None

    def __init__(self):
        rospy.init_node('goal_relayer', anonymous=True)

        rospy.Subscriber("/vrx/wayfinding/waypoints", GeoPath, self.receive_desired_pose)

        self.desired_path_pub = rospy.Publisher("/wamv/desired_path", Path, queue_size=10)

    def receive_desired_pose(self, goalGeoPath: GeoPath):
        print("Geo Path Received\n", goalGeoPath)

        # Create Path object to be sent to desired_path
        tempGoalPath = Path()

        # Declare header info for desired Path
        tempGoalPath.header.stamp = rospy.Time.now()
        tempGoalPath.header.frame_id = "map"

        i = 0

        # Convert each geoPose in geoPath to a PoseStamped object for a Path object
        for goalGeoPose in goalGeoPath.poses:

            goal_lat = goalGeoPose.pose.position.latitude
            goal_lon = goalGeoPose.pose.position.longitude
            goal_x, goal_y = get_xy_based_on_lat_long(goal_lat,goal_lon, "Goal XY")

            # Create empty goal_pose to send as station keeping target
            goal_pose = None
            goal_pose = PoseStamped()

            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"

            # X and Y offsets to fix incorrect lat long to xy conversion
            xOffset = 905.923+0.3
            yOffset = -172.276

            goal_pose.pose.position.x = goal_x + xOffset
            goal_pose.pose.position.y = goal_y + yOffset
            goal_pose.pose.position.z = 0

            goal_pose.pose.orientation = goalGeoPose.pose.orientation

            tempGoalPath.poses.append(goal_pose)
        
        self.goal_path = tempGoalPath


    def goal_relayer(self, freq=30):

        r = rospy.Rate(freq)

        while not rospy.is_shutdown():
            if self.goal_path is None:
                r.sleep()
                continue
            
            # print("publish check", self.goal_path)
            self.desired_path_pub.publish(self.goal_path)
            self.goal_path = None
            r.sleep()

    pass

if __name__ == "__main__":
    wg = WayfindingGoal()

    wg.goal_relayer()