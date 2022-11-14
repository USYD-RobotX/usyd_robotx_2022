#! /usr/bin/env python3
from __future__ import print_function
from ctypes.wintypes import POINT
from dataclasses import dataclass
from dis import dis
from distutils.log import info
import queue
from re import X
from turtle import shape

from matplotlib import image

import roslib
import tf
import math
roslib.load_manifest('usyd_vrx_vision')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped,PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from camera  import Camera
from buoy_detector import Buoy_detector
from usyd_vrx_msgs.msg import ObjectArray
import time

class ObjectClassifier():
  def __init__(self):
    self.bridge = CvBridge()
    self.unclassified_objects = None
    self.classified_objects = None
    rospy.Subscriber("/wamv/unclassified_objects",ObjectArray,self.callback)
    
    self.classified_objects_publisher = rospy.Publisher("/wamv/classified_objects", ObjectArray, queue_size=10)
    self.image_pub = rospy.Publisher("/wamv/object_classifier_image",Image, queue_size=10)
    # rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
    self.tflistener = tf.TransformListener()
    self.identifier = Buoy_detector()
    print("Waiting for object list...")
    while(self.unclassified_objects is None):
      continue
    self.classified_objects = self.unclassified_objects
    rospy.Subscriber("/wamv/reset_objects",String,self.reset_callback)
    print("Recieved an object list...")
    

  def reset_callback(self,data):
    print("reset")
    self.classified_objects.objects = []
  def callback(self,data):
    self.unclassified_objects = data    
  
    # print(self.unclassified_objects.objects[-1].pose)
  
  def pixels_from_pose(self,object_camera_pose,projection_mat):
      object_pos = [object_camera_pose.pose.position.x,object_camera_pose.pose.position.y,object_camera_pose.pose.position.z]
      object_pos.append(1)
      object_pos_mat = np.transpose(np.array(object_pos))
      
      #projection matrix
      projection_mat.shape = (3,4)  
      uvw_mat     =  projection_mat@object_pos_mat

      #pixel position of the object
      pixel_x = int(uvw_mat[0]/uvw_mat[2])
      pixel_y = int(uvw_mat[1]/uvw_mat[2])

      return (pixel_x,pixel_y)
  def update_objects(self):

    print("waiting for object_list")
    while(self.classified_objects is None):
      continue
    isPresent = False
    for unclassified_object in self.unclassified_objects.objects:
      for classified_object in self.classified_objects.objects:
        if classified_object.name == unclassified_object.name:
          isPresent = True
          if unclassified_object.best_confidence >= classified_object.best_confidence:
            classified_object.best_guess = unclassified_object.best_guess
      if isPresent is False:
        self.classified_objects.objects.append(unclassified_object)
      isPresent = False
  

  def object_classifier(self,cameras):
    self.update_objects()
    print("Object Classifier")

    start_time = time.time()
    for object in self.classified_objects.objects:
      for camera in cameras:

      #Iterating every object in the array
        #Pose Stamped from Pose message
        object_pose_stamped = PoseStamped()
        object_pose_stamped.pose = object.pose
        object_pose_stamped.header.frame_id = "map"
        object_pose_stamped.header.stamp = camera.stamp

        #transform pose from map frame to current camera frame        
        try:
          object_camera_pose = self.tflistener.transformPose(camera.frame_id,object_pose_stamped)
        except BaseException as e:
          print(f"Error looking up transform {e}")
          continue
        #getting the image from camera
        cv_image = camera.image.copy()
        display_image = cv_image.copy()
        projection_mat  =  np.array(camera.Proj3d_mat)

        dist_from_camera = object_camera_pose.pose.position.z
        print("Camera: \n",object_camera_pose)
        if(dist_from_camera> 100):
          print("Too Far from the boat")
          self.publish_image(display_image,camera)
          continue
        
        pixel_x,pixel_y  = self.pixels_from_pose(object_camera_pose,projection_mat)
        print ("Pixels (x,y) = ",pixel_x,pixel_y)
        

        #check if the object is in camera's frame
        if pixel_x >= camera.width or pixel_x <=200 or pixel_y <=200 or pixel_y >= camera.height:
          # print("Given point not found through this camera")
          self.publish_image(display_image,camera)
          continue
        else:
          display_image = cv2.circle(display_image, (pixel_x,pixel_y), radius=2, color=(0, 0, 255), thickness=2)
          #crop range for the region of interest
          x_crop_range=[]
          y_crop_range=[]
          # w,h = fx(object_camera_pose.z)

          #setting crop range
          if(dist_from_camera>50 and dist_from_camera<=100):
            height = width = 25
          elif(dist_from_camera>20 and dist_from_camera<=50):
            height=width=50
          elif(dist_from_camera>0 and dist_from_camera<=20):
            height = width = 150
          else:
            continue

          x_crop_range.append(pixel_x-width)
          x_crop_range.append(pixel_x+width)
          y_crop_range.append(pixel_y-height)
          y_crop_range.append(pixel_y+height)

          for x in x_crop_range:
            if x <= 0:
              x =0
            elif x >= camera.width:
              x = camera.width
          for y in y_crop_range:
            if y <= 0:
              y =0
            elif y >= camera.width:
              y = camera.width
          
          #Crop the Image
          crop_img = cv_image[y_crop_range[0]:y_crop_range[1],x_crop_range[0]:x_crop_range[1]]

          #identify the object in the ROI
          try:
            object_identification = self.identifier.buoy_detection(crop_img)
          except BaseException as e:
            print("Error classifiying", e)
            continue

          if object_identification is None:
            print("No mask Found")
            self.publish_image(display_image,camera)
            continue
          else:
            object_name, confidence = object_identification

            print(f"found {object_name}, {confidence}")

          x = pixel_x
          y = pixel_y
          w = width
          h = height
          # x,y,w,h = bounding_rect

          cv2.rectangle(display_image, (x-w, y-w), 
                                        (x + w, y + h),
                                        (0, 255, 0), 2)
          cv2.putText(display_image, object_name, (x-10, y-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0))
          cv2.putText(display_image, str('%.2f' % confidence)+"%", (x+w+1, y+h+1),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0))
          #publishing the image for debugging
          self.publish_image(display_image,camera)

          if(confidence>=object.best_confidence):
            object.best_guess = object_name
            object.best_confidence = confidence
      print("Length of object classifier: ",len(self.classified_objects.objects))
      self.classified_objects_publisher.publish(self.classified_objects)
    
  def publish_image(self,image,camera):
    # window_name = camera.sub_topic.replace("/wamv/sensors/cameras/","")
    # window_name = window_name.replace("/image_raw","")
    # cv2.imshow(window_name,image)
    # cv2.waitKey(10)
    camera.image_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


def main(args):
 
  rospy.init_node('object_classifier', anonymous=True)
  r = rospy.Rate(50)
  oc = ObjectClassifier()
  print("Object Classifier")
  cameras = []
  cameras.append(Camera("/wamv/sensors/cameras/left_camera/image_raw"))
  cameras.append(Camera("/wamv/sensors/cameras/front_camera/image_raw"))
  cameras.append(Camera("/wamv/sensors/cameras/right_camera/image_raw"))
  while not rospy.is_shutdown():
    oc.object_classifier(cameras)
    r.sleep()
  pass   


if __name__ == '__main__':
    main(sys.argv)
