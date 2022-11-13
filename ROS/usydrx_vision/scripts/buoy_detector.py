#!/usr/bin/env python3

##Author: Hrithik Selvaraj
##Last Updated: Nov 12, 2022
##Email: hrithik.sel@gmail.com

##### Buoy Detector#####
## Code to detect the type of buoy from a given image

## Loading the libraries
import sys
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


class BuoyDetector():
  
  # Initializing the class
  def __init__(self):
    self.color_templates  = self.load_colour_templates(color_list=["red","green"])


  #Loading color templates
  def load_colour_templates(self,color_list):
    color_template = []
    for color in color_list:
      color = color.lower()
      if color == "red":
        #range for red color
        lower = np.array([0, 40, 54], np.uint8)
        upper = np.array([15, 255, 255], np.uint8)
      elif color == "green":
        #range for green color
        lower = np.array([70, 15, 40], np.uint8)
        upper = np.array([88, 255, 255], np.uint8)
      elif color == "black":
        #range for black color
        lower = np.array([0, 0, 0], np.uint8)
        upper = np.array([5, 20, 20], np.uint8)
      elif color == "white":
        #range for white color
        sensitivity = 70  
        lower = np.array([23,sensitivity,40])
        upper = np.array([30,255,255])
      elif color == "orange":
        #range for orange color
        lower = np.array([6, 90, 40], np.uint8)
        upper = np.array([15, 255, 255], np.uint8)
      else:
        print("Invalid Color in color_list")
        quit()
      color_range = (color,lower,upper)
      color_template.append(color_range)
    return color_template
 
  def canny_edge(self,image):

    kernel = np.ones((5,5),np.float32)/25
    dst = cv.filter2D(image,-1,kernel)
    # cv.imshow("Window",dst)
    # edge = cv.Canny(dst,10,80)
    # cv.imshow("canny",edge)

  def hist_color(self,image):
   kernel = np.ones((5,5),np.float32)/25
   dst = cv.filter2D(image,-1,kernel)
   hsv_image = cv.cvtColor(dst,cv.COLOR_BGR2HSV)
   hist = cv.calcHist(hsv_image, [0], None, [256], [0,255])
  #  print(np.where(hist==max(hist)))
   print(hist)

  # Function to create mask of the image
  def mask_image_hsv(self,image):    
    
    #converting the color space to HSV
    hsv_image = cv.cvtColor(image,cv.COLOR_BGR2HSV)
    area_max = 0
    mask_contours = []

    #Looping through all the colors to find the best color
    for color_range in self.color_templates:
      color_name,lower,upper = color_range
      mask = cv.inRange(hsv_image,lower,upper)
      
      kernal = np.ones((10, 10), "uint8")
 
      
      # mask = cv.morphologyEx(mask,cv.MORPH_CLOSE, kernal)
      mask = cv.morphologyEx(mask,cv.MORPH_CLOSE, kernal)
      # mask = cv.dilate(mask, kernal)
     
      cv.bitwise_and(image,image,mask)
      contours, hierarchy = cv.findContours(mask,
                                            cv.RETR_TREE,
                                            cv.CHAIN_APPROX_SIMPLE)
      
      for pic, contour in enumerate(contours):
          area = cv.contourArea(contour)
          if(area > 500 ):
            
            final_mask = mask
            area_max = area
            final_color = color_name
            mask_contours.append(contour)
            final_contour = contour
          else:
            cv.drawContours(mask, [contour], -1, 0, -1)
                
    if area_max > 500:
      # cv.imshow("Mask",final_mask)
      return(final_mask,final_color,mask_contours,final_contour)    
    else:
    #if no color found
      return None

  # Function to match the shape template
  def shape_match(self,image):
    min_match = 500
    matched_label = None
    for template in self.shape_templates:
      shape,label = template
      match = cv.matchShapes(image,shape,1,0.0)
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
    
    kernel = np.ones((5,5),np.float32)/40
    image = cv.filter2D(image,-1,kernel)    
    
    kernal = np.ones((10, 10), "uint8")
    display_image = image.copy()
    color_detection = self.mask_image_hsv(image)

    if color_detection is None:
      return display_image
    else:
      final_mask,mask_color,contours,obj_contour= self.mask_image_hsv(image)
    
   
    # cv.imshow("masked_image",final_mask)
   
    final_mask = cv.dilate(final_mask, kernal)
    contour = np.concatenate(contours)
    # final_mask = masked_image
    contours, hierarchy = cv.findContours(final_mask,
                                            cv.RETR_TREE,
                                            cv.CHAIN_APPROX_SIMPLE)
    # fill shape
    cv.fillPoly(final_mask, pts=contours, color=(255,255,255))
    # cv.imshow("masked_image_after_filling",final_mask)

    
    bounding_rect = cv.boundingRect(contour)
    x,y,w,h = bounding_rect
    display_image = cv.rectangle(display_image, (x, y), 
                                        (x + w, y + h),
                                        (0, 255, 0), 2)
    display_image = cv.putText(display_image, mask_color, (x-10, y-3),
                      cv.FONT_HERSHEY_SIMPLEX, 
                      2, (0, 255, 0),3)                                          

  
    # cv.imshow("Display Image",display_image)
    return (mask_color)

def main(args):

  
  # rospy.init_node('ColorThresholder', anonymous=True)
  
  ct = BuoyDetector()
  image = cv.imread("red_1.png")
  
  # ct.hist_color(image)

  while(1):
    # ct.canny_edge(image)
    print(ct.buoy_detection(image))
    cv.waitKey(5)

if __name__ == '__main__':
   main(sys.argv)