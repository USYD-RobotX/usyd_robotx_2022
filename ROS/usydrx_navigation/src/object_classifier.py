


from operator import rshift
import rospy
from usyd_vrx_msgs.msg import Object, ObjectArray

class ObjectClassifier:
    def __init__(self) -> None:
        rospy.init_node("object_classifier")

        rospy.Subscriber("wamv/unclassified_objects", ObjectArray, self.unclassified_objects_callback)
        pass



    def unclassified_objects_callback(self, unclassified_objects_array):

        print(unclassified_objects_array)
        


    
if __name__ == "__main__":
    oc = ObjectClassifier()
    while not rospy.is_shutdown():
        rospy.spin()