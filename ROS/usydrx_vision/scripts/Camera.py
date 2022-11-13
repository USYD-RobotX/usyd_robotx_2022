#### Camera Module ####
## Creating an instance of class camera("LEFT" or "RIGHT" or "FRONT")
## will subscribe to the camera topic and converts to cv_image matrix
## which can be accessed from camera.image
from dataclasses import replace
import queue
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError


#sub_topic = <camera_name>/image_raw/compressed - (using compressed instead of raw, as it gives 8-bit image format for cheap computation)
  
class Camera():
  def __init__(self,sub_topic):

    #initializing variables
    self.image = None   
    self.frame_id = None
    self.Proj3d_mat = None
    self.height = None
    self.width = None

    #cv_bridge to convert ros image topic to opencv image
    self.bridge = CvBridge()
    self.sub_topic  = sub_topic
    self.camera_info_sub_topic = sub_topic.replace("image_raw","camera_info")
    
    #subscribers for image and info
    rospy.Subscriber(self.camera_info_sub_topic,CameraInfo,self.camera_info_callback)
    rospy.Subscriber(self.sub_topic,Image,self.camera_callback)
    rospy.sleep(1)
    

    #program waits until the image topic is published
    print("waiting for image...")
    while(self.image is None):
      continue
    print("Received Image")
    self.image_publisher = rospy.Publisher(self.sub_topic.replace("image_raw","annotated_image"),Image,queue_size = 10)    
    self.image_republisher = rospy.Publisher(self.sub_topic+"/rotated",Image,queue_size = 10)    

  def camera_callback(self,data):
    try:
        self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        if(self.sub_topic == "/wamv/sensors/cameras/left_camera/image_raw"):
          (h, w) = self.image.shape[:2]
          (cX, cY) = (w // 2, h // 2)
          M = cv2.getRotationMatrix2D((cX, cY), -10, 1.0)
          rotated = cv2.warpAffine(self.image, M, (w, h))
          # cv2.imshow("Rotated",rotated)
          # cv2.waitKey(10)

          self.image_republisher.publish(self.bridge.cv2_to_imgmsg(rotated, "bgr8"))
        if(self.sub_topic == "/wamv/sensors/cameras/right_camera/image_raw"):
          (h, w) = self.image.shape[:2]
          (cX, cY) = (w // 2, h // 2)
          M = cv2.getRotationMatrix2D((cX, cY), 10, 1.0)
          rotated = cv2.warpAffine(self.image, M, (w, h))
          # cv2.imshow("Rotated",rotated)
          # cv2.waitKey(10)
          self.image_republisher.publish(self.bridge.cv2_to_imgmsg(rotated, "bgr8"))
    except CvBridgeError as e:
        print(e)

  def camera_info_callback(self,data):
    self.Proj3d_mat = data.P
    self.height = data.height
    self.width= data.width
    self.frame_id = data.header.frame_id
    self.stamp = data.header.stamp


 

