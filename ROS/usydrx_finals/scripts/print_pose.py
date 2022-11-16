#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def pose_cb(data):
    print(data)
    pose = data.pose
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    msg = f"Pose(Point({x},{y},{z}), Quaternion({qx},{qy},{qz},{qw}))"

    print(msg)


if __name__ == "__main__":
    rospy.init_node("print_poser")
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, pose_cb)

    rospy.spin()