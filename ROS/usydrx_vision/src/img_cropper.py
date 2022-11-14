#! /usr/bin/env python
from __future__ import print_function
from ctypes.wintypes import POINT
from dataclasses import dataclass
from distutils.log import info
from re import X

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


class Image_cropper():
  def __init__(self):
    self.bridge = CvBridge()
    self.poseStamped = None
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
    self.tflistener = tf.TransformListener()

  
  def callback(self,data):
    self.poseStamped = data    

  def image_cropper(self,camera):
    while(self.poseStamped is None):
      continue
    
    try:
        print("Image Cropper")
        object_ps = self.tflistener.transformPose(camera.frame_id,self.poseStamped)
        print("Camera: \n",object_ps)
        # print("original pose",self.pose)
        cv_image = camera.image
        object_pos = [object_ps.pose.position.x,object_ps.pose.position.y,object_ps.pose.position.z]
        object_pos.append(1)
        object_pos_mat = np.transpose(np.array(object_pos))
        projection_mat  =  np.array(camera.Proj3d_mat)
        projection_mat.shape = (3,4)  
        uvw_mat     =  projection_mat@object_pos_mat
        # print(uvw_mat)
        pixel_x = int(uvw_mat[0]/uvw_mat[2])
        pixel_y = int(uvw_mat[1]/uvw_mat[2])
        print ("Pixels (x,y) = ",pixel_x,pixel_y)
        if pixel_x >= camera.width or pixel_x <=0 or pixel_y <=0 or pixel_y >= camera.height:
          print("Not found through this camera")
          return None
        x_crop_range=[]
        y_crop_range=[]
        x_crop_range.append(pixel_x-50)
        x_crop_range.append(pixel_x+50)
        y_crop_range.append(pixel_y-100)
        y_crop_range.append(pixel_y+100)
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
        print(x_crop_range,y_crop_range)
        crop_img = cv_image[y_crop_range[0]:y_crop_range[1],x_crop_range[0]:x_crop_range[1]]
        cv2.rectangle(cv_image, (x_crop_range[0],y_crop_range[0]), (x_crop_range[1],y_crop_range[1]), (255,0,0), 2)
        
        # cv2.imshow("Cropped Image",crop_img)
        # cv2.imshow("Original image",cv_image)
        # cv2.waitKey(1)
        return crop_img
    except CvBridgeError as e:
      print(e)


def main(args):
 
  rospy.init_node('buoy_detector', anonymous=True)
  r = rospy.Rate(50)
  ic = []
  bd = Buoy_detector()
  # ic.append(image_cropper(Camera("LEFT")))
  ic.append(Image_cropper(Camera("FRONT")))
  # ic.append(image_cropper(Camera("RIGHT")))
  while not rospy.is_shutdown():
    for x in ic:
     if x.camera.image is None and x.camera.Proj3d_mat is None:
       print("waiting for image")
       r.sleep()
       continue
     pass
    #  ic = image_cropper(x)
    #  x.image_cropper()
    
     bd.buoy_detection(x.image_cropper())
     r.sleep()
  pass   
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)