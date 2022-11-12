#!/usr/bin/env python3

from visualization_msgs.msg import Marker, MarkerArray
from usyd_vrx_msgs.msg import ObjectArray
import rospy

class ObjectVisualiser():

    def __init__(self) -> None:

        rospy.init_node("object_visualiser")
        self.maker_array_pub = rospy.Publisher("wamv/visualisation/objects", MarkerArray, queue_size=10)

        rospy.Subscriber("/wamv/classified_objects", ObjectArray, self.object_array_callback)

    def object_array_callback(self, object_array: ObjectArray):
        marker_array = MarkerArray()
        for object in object_array.objects:
            if "buoy" in object.best_guess:
                
                marker = self.generate_buoy_marker(object.pose.position.x, object.pose.position.y, object.name, object.best_guess)
                marker_array.markers.append(marker)

                text_marker = self.generate_buoy_text_marker(object.pose.position.x, object.pose.position.y, f"{object.best_guess}: {object.name}", object.best_guess)
                marker_array.markers.append(text_marker)
                # print("Rec buoy")

            if "dock" in object.best_guess:
                print("got dock")
                marker = self.generate_dock_marker(object.pose, object.name)
                marker_array.markers.append(marker)
        
        self.maker_array_pub.publish(marker_array)
        pass


    @staticmethod
    def generate_buoy_marker(x,y,name,type):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle_visualsiation"

        marker.id = int(name.split(" ")[-1])

        marker.type = Marker.CYLINDER
        marker.action = Marker.MODIFY


        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 5.0

        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1

        marker.lifetime = rospy.Time(5)
        return marker

    @staticmethod
    def generate_buoy_text_marker(x,y,name,type):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle_visualsiation_text"

        marker.id = int(name.split(" ")[-1])

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD


        marker.pose.position.x = x
        marker.pose.position.y = y

        marker.pose.orientation.w = 1

        marker.text = name

        marker.pose.orientation.w = 0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 5.0

        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1

        marker.lifetime = rospy.Time(5)
        return marker

    @staticmethod
    def generate_dock_marker(pose, name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle_visualsiation_text"

        marker.id = int(name.split(" ")[-1])

        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = "package://usyd_vrx_navigation/meshes/dock_2022.dae"
        marker.pose = pose
        marker.pose.position.z = 0.5

        marker.text = name

        # marker.pose.orientation.w = 0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1

        marker.lifetime = rospy.Time(5)
        return marker


if __name__ == "__main__":
    ov = ObjectVisualiser()

    rospy.spin()
