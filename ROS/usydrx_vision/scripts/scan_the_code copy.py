
import rospy

from buoy_scanner import lightScanner
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String 
import time

msg_dict = {"red": "R", "green": "G", "blue": "B"}

class ScanCode:

    def __init__(self):
        rospy.init_node("scanner")
        self.bridge = CvBridge()
        self.scanner = lightScanner()

        self.pub = rospy.Publisher("/light_sequence", String, queue_size = 10)

    def start(self):
        while not rospy.is_shutdown():
            try:
                sequence = self.scanBuoy()
                if sequence is not None:
                    sequence_str = f"{msg_dict[sequence[0]]}{msg_dict[sequence[1]]}{msg_dict[sequence[2]]}"
                    # sequence_str = "RGB"
                    print("sending", sequence_str)
                    self.pub.publish(sequence_str)
                else:
                    print("Failed sequencer")
            except KeyError:
                pass

            time.sleep(1)

    def scanBuoy(self):
            #scanner = Scanner()
            found = False
            sequence = []
            last_colour = "none"
            count = 0
            rospy.loginfo("Attempting to Scan buoy")

            while(found == False):
                ros_image = rospy.wait_for_message("/wamv/sensors/cameras/front_camera/image_raw" ,Image)
                image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
                print("scanning")
                colour = self.scanner.scanBuoy(image)
                print("Got", colour)
                count += 1

                # if count > 10:
                #     return ["red", "green", "blue"]
                #rospy.loginfo("Detected colour %s",colour)
                # print("attemping to scan")
                # time.sleep(5)
                # return ["red", "green", "blue"]

                if colour[0:4] == "none":
                    if last_colour[0:4] != "none":
                        #if it is the first out of a reading
                        last_colour = "none"
                        count = 0
                    count = count +1

                    if count > 100:
                        print("Cant find")



                    sequence = []
                    continue
                if colour != last_colour :
                    sequence.append(colour)
                    last_colour = colour
                    count = 0
                else:
                    count = count+1

                if len(sequence)>=3:
                    print("Detected sequence is %s, %s,%s,",sequence[0],sequence[1],sequence[2])
                    found = True
                    return sequence

                if count > 100:
                    print("No change in colour found for 100 frames")
                    return None
                    # return ["red", "green", "blue"]



if __name__ == "__main__":
    sc = ScanCode()
    while not rospy.is_shutdown():
        print("scanning")
        values = (sc.start())
        

