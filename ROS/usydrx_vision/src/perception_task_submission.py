#! /usr/bin/env python3
from __future__ import print_function
import imp
from tkinter import LEFT

import roslib
roslib.load_manifest('usyd_vrx_vision')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped,PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from camera import Camera, camera
from buoy_detector import Buoy_detector
from object_classifier import ObjectClassifier
from vrx_gazebo.msg import Task
import geonav_transform.geonav_conversions as gc
import alvinxy.alvinxy as axy
 
class PerceptionTask():
    def __init__(self):
        self.landmark_pub = rospy.Publisher("/vrx/perception/landmark",GeoPoseStamped,queue_size=10)
        self.cameras = []
        self.cameras.append(Camera("/wamv/sensors/cameras/front_camera/image_raw"))
        self.task_info = None
        self.buoy_detector = Buoy_detector()
        self.object_classifier = ObjectClassifier()
        #setting origin latitude and longitude
        self.origin_lat = -33.7227685
        self.origin_lon = 150.6739913
        self.landmark = GeoPoseStamped()
    #receive data from /vrx/task/info
    def togglePublisher(self,data):
        self.toggle.publish("reset")        

    def identify_object(self):
        self.object_classifier.object_classifier((self.cameras))

        print("waiting for object list in perception!")
        while(self.object_classifier.classified_objects is None):
            continue

        print("Received in perception")

        for object in self.object_classifier.classified_objects.objects:
            if(object.best_confidence>10):
                x = object.pose.position.x                                          #un-comment after subscribing to actual location topic 
                y = object.pose.position.y                                         #un-comment after subscribing to actual location topic
                #converting position to lan,lon
                obj_lat,obj_lon = gc.xy2ll(x,y,self.origin_lat,self.origin_lon)     #un-comment after subscribing to actual location topic
                obj_alt = 0                                                         #un-comment after subscribing to actual location topic  

                print("Found Marker: ", object.best_guess)
                self.landmark.header.seq = 1
                self.landmark.header.stamp = rospy.Time.now()
                self.landmark.header.frame_id = object.best_guess
                self.landmark.pose.position.latitude = obj_lat
                self.landmark.pose.position.longitude = obj_lon
                self.landmark.pose.position.altitude = obj_alt

                self.landmark.pose.orientation.x = 0
                self.landmark.pose.orientation.y = 0
                self.landmark.pose.orientation.z = 0
                self.landmark.pose.orientation.w = 0
                self.landmark_pub.publish(self.landmark)

             
if __name__ == '__main__':
    rospy.init_node('perception_task', anonymous=True)
    r = rospy.Rate(1)
    perception_task = PerceptionTask()
    while not rospy.is_shutdown():
        perception_task.identify_object()
        r.sleep()
    pass   
    cv2.destroyAllWindows()
