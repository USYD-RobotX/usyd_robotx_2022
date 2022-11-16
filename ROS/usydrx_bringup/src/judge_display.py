#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
import numpy as np



class judgeDisplay():
    def __init__(self) -> None:

        rospy.init_node("light_sequence_viewer")

        self.color_code = "None"
        
        white_img = np.zeros([500,800,3],dtype=np.uint8)
        self.whitebg = white_img.fill(255)
        self.color_code = "AAA"

        rospy.Subscriber("/light_sequence", String, self.code_callback)


    def code_callback(self,data):
        print("got code")
        self.color_code = data.data


    def display_results(self):
        
        #change default only if it is not nonce
        if self.color_code is not None:
            self.color_code = self.color_code.upper()
        
        if len(self.color_code) != 3:
            self.color_code = "AAA"
                
        white_image = np.zeros([800,1700,3],dtype=np.uint8)
        white_image.fill(255)

        cv2.putText(white_image, 'Scan The Code - USYD SEALIONS', (300,100), cv2.FONT_HERSHEY_SIMPLEX, 
                   2, (0,0,0), 2, cv2.LINE_AA)
        
        white_image = cv2.rectangle(white_image, (200,200), (500,300), (0,0,0), 2)
        white_image = cv2.rectangle(white_image, (700,200), (1000,300), (0,0,0), 2)
        white_image = cv2.rectangle(white_image, (1200,200), (1500,300), (0,0,0), 2)

        for i in range(3):

            #Bounding Box for the colors
            white_image = cv2.rectangle(white_image, (200+i*500,400), (500+i*500,700), (0,0,0), 2)
            
            color = (255,255,255)
            text = ["First","Second","Third"]
            

            # Color and Text
            if self.color_code[i] == "R":
                text[i]= "Red"
                color = (0,0,255)
            elif self.color_code[i] == "G":
                text[i]= "Green"
                color = (0,255,0)
            elif self.color_code[i] == "B":
                text[i]= "Blue"
                color = (255,0,0)

            # Display Text
            cv2.putText(white_image, text[i], (330+i*500,250), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (0,0,0), 2, cv2.LINE_AA)
            

            white_image[400:700,200+i*500:500+i*500] = color       
        cv2.imshow("White",white_image)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            return
        
if __name__ == "__main__":
    jd = judgeDisplay()

    while not rospy.is_shutdown():
        jd.display_results()
        