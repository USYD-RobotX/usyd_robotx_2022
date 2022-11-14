#! /usr/bin/env python
from __future__ import print_function
from locale import normalize
from tkinter import LEFT
from turtle import color
from simplejson import load
import roslib
roslib.load_manifest('usyd_vrx_vision')
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import os
class Buoy_detector():
  
  def __init__(self):
    dirname = os.path.dirname(__file__)
    self.file_list = [dirname+"/templates/marker_buoy.png",dirname+"/templates/cone_marker_far.png",dirname+"/templates/cone_marker_mid.png",dirname+"/templates/round_buoy_far.png",dirname+"/templates/round_buoy_mid.png",dirname+"/templates/round_buoy.png"]
    self.template_label = ["marker_buoy","marker_buoy","marker_buoy","round_buoy","round_buoy","round_buoy"]
    self.shape_templates = self.load_templates(self.file_list)
    self.color_templates  = self.load_colour_templates(color_list=["red","green","black","white","orange"])
  
  def load_templates(self,file_list):
    templates = []
    for i in range(0,len(file_list)):
      image = cv2.imread(file_list[i],cv2.IMREAD_GRAYSCALE)
      label = self.template_label[i]
      template = image,label
      templates.append(template)
    return templates

  def load_colour_templates(self,color_list):
    color_template = []
    for color in color_list:
      if color == "red":
        #range for red color
        lower = np.array([160, 50, 70], np.uint8)
        upper = np.array([180, 255, 255], np.uint8)
      elif color == "green":
        #range for green color
        lower = np.array([36, 150, 70], np.uint8)
        upper = np.array([89, 255, 255], np.uint8)
      elif color == "black":
        #range for black color
        lower = np.array([0, 0, 0], np.uint8)
        upper = np.array([5, 20, 20], np.uint8)
      elif color == "white":
        #range for white color
        sensitivity = 70
        lower = np.array([0,0,100])
        upper = np.array([0,sensitivity,255])
      elif color == "orange":
        #range for orange color
        lower = np.array([3, 50, 70], np.uint8)
        upper = np.array([24, 255, 255], np.uint8)
      else:
        print("Invalid Color in color_list")
        quit()
      color_range = (color,lower,upper)
      color_template.append(color_range)
    return color_template
 
  
  def mask_image(self,image):

    hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    area_max = 0
    for color_range in self.color_templates:
      # if color_range[0] == 'white':
      #     cv2.imshow("b_image", image)
      #     cv2.waitKey(1)
      color_name,lower,upper = color_range
      mask = cv2.inRange(hsv_image,lower,upper)
      kernal = np.ones((5, 5), "uint8")
      mask = cv2.dilate(mask, kernal)
      cv2.bitwise_and(image,image,mask)
      # if color_range[0] == 'white':
      #     cv2.imshow("a_image", mask)
      #     cv2.waitKey(1)

      contours, hierarchy = cv2.findContours(mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
      
      for pic, contour in enumerate(contours):
          area = cv2.contourArea(contour)
          if(area > area_max):
            final_mask = mask
            area_max = area
            final_color = color_name
            mask_contours = contours
            final_contour = contour
    if area_max > 150:
      return(final_mask,final_color,mask_contours,final_contour)    
    else:
    #if no color found
      return None

  def shape_match(self,image):
    min_match = 100
    matched_label = None
    for template in self.shape_templates:
      shape,label = template
      match = cv2.matchShapes(image,shape,1,0.0)
      # print(match,label)
      if match < min_match:
        min_match = match
        matched_label = label
    
    match_percentage = (1-10*min_match)*100
    if matched_label is None:
      return None
    else:
      return (matched_label,match_percentage)

 
  def buoy_detection(self,image):
    if image is None:
      return

    display_image = image.copy()
    color_detection = self.mask_image(image)

    if color_detection is None:
      return None
    else:
      masked_image,mask_color,contours,obj_contour= self.mask_image(image)
    # cv2.imshow("masked_image",masked_image)
    # fill shape
    cv2.fillPoly(masked_image, pts=contours, color=(255,255,255))
    bounding_rect = cv2.boundingRect(obj_contour)
    x,y,w,h = bounding_rect
    display_image = cv2.rectangle(display_image, (x, y), 
                                        (x + w, y + h),
                                        (0, 255, 0), 1)
    img_cropped_bounding_rect = masked_image[bounding_rect[1]:bounding_rect[1] + bounding_rect[3], bounding_rect[0]:bounding_rect[0] + bounding_rect[2]]
    crop_image = display_image[bounding_rect[1]:bounding_rect[1] + bounding_rect[3], bounding_rect[0]:bounding_rect[0] + bounding_rect[2]]
    # resize all to same size
    # cv2.imshow("cropped_image",crop_image)
    normalised_image = cv2.resize(img_cropped_bounding_rect, (300, 300))
    # cv2.imwrite("normalised_image.png",normalised_image)
    object_details = self.shape_match(normalised_image)
    if object_details is None:
      return None
    else:
      object_type,confidence = object_details
    object_name =  "mb"+ "_"+ object_type+"_"+mask_color
    cv2.putText(display_image, object_name, (x-10, y-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0))
    cv2.putText(display_image, str('%.2f' % confidence)+"%", (x+w+1, y+h+1),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0))                          
    # cv2.imshow("Display Image",display_image)
    return (object_name,confidence)


    


def main(args):
 
  bd = Buoy_detector()
  image = cv2.imread("/home/hrithik/Pictures/tree.png")
  print(bd.buoy_detection(image))  
  # cv2.imshow("Original Image",image)
  cv2.waitKey(10000)
  # bd.buoy_detection(image)
  

if __name__ == '__main__':
    main(sys.argv)


    #old color range
        #range for red color
        # lower = np.array([160, 230, 80], np.uint8)
        # upper = np.array([180, 255, 120], np.uint8)

        #range for green color
        #range for green color
        # lower = np.array([60, 200, 40], np.uint8)
        # upper = np.array([90, 240, 56], np.uint8)

        #range for black color
        # lower = np.array([0, 0, 0], np.uint8)
        # upper = np.array([5, 20, 20], np.uint8)

        #range for white color
        # sensitivity = 70
        # lower = np.array([0,0,100])
        # upper = np.array([0,sensitivity,255])

        #range for orange color
        # lower = np.array([3, 230, 90], np.uint8)
        # upper = np.array([7, 255, 170], np.uint8)