#!/usr/bin/env python
from vrx_msgs.srv import ClassifyBuoy,ClassifyBuoyResponse
import rospy
import rospkg
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError

def imageClient():
    rospy.wait_for_service('wamv/scan_buoy')
    try:
        rospack = rospkg.RosPack()
        image = cv2.imread(rospack.get_path('usydrx_vision')+'/Images/seq_'+'green'+'.png',cv2.IMREAD_COLOR)
        # cv2.imshow('img', image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print("Here")

        scanBuoy = rospy.ServiceProxy('wamv/scan_buoy', ClassifyBuoy)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        # print("Here")
        res = scanBuoy(image_message, 0)
        print(res)
    except rospy.ServiceException:
        print ("Service call failed:")

if __name__ == "__main__":
    imageClient()