#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32, Header
from nav_msgs.msg import Path, Odometry
import math
import tf


from geometry_msgs.msg import Pose, PoseStamped, Quaternion
# from geometry_msgs.msg import Twist
import numpy as np

from scipy.spatial.transform import Rotation as R


class PoseStampedToPath:

    current_pose = None
    def __init__(self):

        rospy.init_node('pose_stamped_to_path', anonymous=True)

        self.path_pub = rospy.Publisher("/wamv/desired_path", Path, queue_size=10)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pose_stamped_receive)

        rospy.Subscriber("/wamv/odom", Odometry, self.odom_receive)


    def odom_receive(self, odom: Odometry):
        # print("received odom")
        self.current_pose = odom.pose.pose

    def pose_stamped_receive(self, pose_stamped):

        if self.current_pose:

            # print("doing pose")

            path = Path()
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "map"

            path.header = h

            init_pose = PoseStamped()
            init_pose.header.frame_id = "map"
            init_pose.header.stamp = rospy.Time.now()
            init_pose.pose.position = self.current_pose.position

            init_pose.pose.orientation = pose_stamped.pose.orientation

            curr_xy = np.array([self.current_pose.position.x, self.current_pose.position.y])

            dest_xy = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y])

            diff_xy = dest_xy - curr_xy
            orient_z = math.atan2(diff_xy[1], diff_xy[0])

            orien_quat = R.from_euler('xyz', [0,0,orient_z]).as_quat()
            orientation = Quaternion(*list(orien_quat))

            distance = np.linalg.norm(diff_xy)

            print(distance)

            dist_int = math.ceil(distance)

            init_pose.pose.orientation = orientation

            path.poses = [init_pose]

            for _i in range(dist_int):
                i = _i+1
                new_xy = curr_xy + (diff_xy/dist_int) * i

                # print(new_xy)

                new_pose = PoseStamped()
                new_pose.header.frame_id = "map"
                new_pose.header.stamp = rospy.Time.now()
                new_pose.pose.position.x = new_xy[0]
                new_pose.pose.position.y = new_xy[1]
                new_pose.pose.orientation = orientation
                path.poses.append(new_pose)

            path.poses.append(pose_stamped)

            self.path_pub.publish(path)



if __name__ == "__main__":
    print("Hello")
    
    pstp = PoseStampedToPath()

    while not rospy.is_shutdown():
        rospy.spin()