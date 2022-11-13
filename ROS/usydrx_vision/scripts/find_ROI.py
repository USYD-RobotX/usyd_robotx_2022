from dis import dis
import imp
import rospy
import cv2
import numpy as np
from Camera import Camera





class ROIfinder:
    def __init__(self) -> None:
        self.debug = False
        pass

    def find_uv(self,camera: Camera, point):
        
        image = camera.image

        u,v = camera.camera_model.project3dToPixel(point)
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blurred = cv2.GaussianBlur(gray, (7, 7), 2)
        # mask = cv2.Canny(blurred,self.threshold_1,self.threshold_2)
        

        # contours, hierarchy = cv2.findContours(mask,
        #                                     cv2.RETR_TREE,
        #                                     cv2.CHAIN_APPROX_SIMPLE)
        return u,v
        

    def find_xyz(self,camera: Camera, pixel):
        pixel = camera.camera_model.rectifyPoint(pixel)
        image = camera.rect_image
        disp_image = image
        # pixel_size = 5.6e-6
        # camera.camera_model.P *= pixel_size 
        x,y,z = camera.camera_model.projectPixelTo3dRay(pixel)
        
        
        pixel_size = 1
        if(self.debug):
            disp_image = cv2.putText(disp_image,"x: "+ str(x/pixel_size) +" y: "+ str((y/pixel_size)) + " z: " + str(z/pixel_size), (100,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            cv2.imshow("Position of Object",disp_image)
        return None # return cropped image

    
