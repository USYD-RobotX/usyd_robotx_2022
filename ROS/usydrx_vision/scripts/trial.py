#!/usr/bin/env python3

from buoy_detector import BuoyDetector
import cv2 
import rospy
# from image_proc_tools import CannyDetector,HSVColorThresholder
from find_ROI import ROIfinder
from Camera import Camera
import numpy as np
import matplotlib.pyplot as plt
import sys
from buoy_scanner import lightScanner


def adjust_gamma(image, gamma=0.4):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)


def main(args):

  rospy.init_node('Trial', anonymous=True)
  r = rospy.Rate(50) 
  b = BuoyDetector()
  scanner = lightScanner()
  image = cv2.imread("red_light.png")
  
  # cam_centre = Camera("/wamv/sensors/cameras/left_camera/image_raw")
  # cam_right= Camera("/wamv/sensors/cameras/right_camera/image_raw")
  while not rospy.is_shutdown():
    
    cv2.imshow("winf",image)
    # b.mask_image_hsv(cam_centre.image)
    print(scanner.scanBuoy(image))
    cv2.waitKey(10)
    r.sleep()
  pass   

if __name__ == '__main__':
    main(sys.argv)