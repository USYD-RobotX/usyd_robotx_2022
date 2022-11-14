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


  
class Camera():
  def __init__(self,sub_topic):
    self.bridge = CvBridge()
    self.sub_topic  = sub_topic
    self.camera_info_sub_topic = sub_topic.replace("image_raw","camera_info")
    self.image = None   
    self.frame_id = None
    self.Proj3d_mat = None
    self.height = None
    self.width = None
    rospy.Subscriber(self.camera_info_sub_topic,CameraInfo,self.camera_info_callback)
    rospy.Subscriber(self.sub_topic,Image,self.camera_callback)
    rospy.sleep(1)
    self.image_publisher = rospy.Publisher(self.sub_topic.replace("image_raw","classifier_feed"),Image,queue_size = 10)    
    print("waiting for image...")
    while(self.image is None):
      continue

  def camera_callback(self,data):
    try:
        self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")

    except CvBridgeError as e:
        print(e)
  
  def camera_info_callback(self,data):
    self.Proj3d_mat = data.P
    self.height = data.height
    self.width= data.width
    self.frame_id = data.header.frame_id
    self.stamp = data.header.stamp
class camera_info():
  def __init__(self,name):
    if name == "LEFT":
      self.sub_topic = "/wamv/sensors/cameras/left_camera/camera_info"
    elif name == "RIGHT":
      self.sub_topic = "/wamv/sensors/cameras/right_camera/camera_info"
    elif name == "FRONT":
      self.sub_topic = "/wamv/sensors/cameras/front_camera/camera_info"
    else:
      print("Invalid Camera") #need to print as an error message
    rospy.Subscriber(self.sub_topic,CameraInfo,self.callback)
  
  def callback(self,data):
    self.cam_info = data

class camera():
  def __init__(self,name):
    self.name = name
    if name == "LEFT":
      self.sub_topic = "/wamv/sensors/cameras/left_camera/image_raw"
    elif name == "RIGHT":
      self.sub_topic = "/wamv/sensors/cameras/right_camera/image_raw"
    elif name == "FRONT":
      self.sub_topic = "/wamv/sensors/cameras/front_camera/image_raw"
    else:
      print("Invalid Camera") #need to print as an error message 
    self.image = None   
    self.cbflag = False 
 


