from geometry_msgs.msg import PoseStamped, Pose
from re import T
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import String
import math
import tf
import time
from scipy.spatial.transform import Rotation as R

def quatToEuler(quat):

    if quat.x is None:
        #Must be a in tf form.
        quaternion = quat
    else:
        quaternion = (
        quat.x,
        quat.y,
        quat.z,
        quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    return euler[0],euler[1],euler[2] #RPY

def eulerToQuat(euler):
    q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


class DockingSolution:


    current_pose = None
    def __init__(self):
        rospy.init_node("docking_solution")

        rospy.Subscriber("/wamv/odom", Odometry, self.odom_cb)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)

        print("Waiting for odom")
        ros_image = rospy.wait_for_message("/wamv/odom", Odometry)
        print("Got odom")


    def odom_cb(self, data):
        # print("got current pose")
        self.current_pose = data.pose.pose

    def go_to_pose(self, target, dist_range=2, angle_range=0.2):
        
        self.pub_goal(target)

        while not rospy.is_shutdown():
            if self.in_range(target, dist_range, angle_range):
                print("IN TARGET")
                return True
            else:
                time.sleep(0.5)
                print("Going to wp")

            
    def in_range(self, target, dist_thresh = 0.5, ang_thresh = 0.4):
        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)

        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= ang_thresh):
            return True
        else:
            return False

    def translatePose(self,pose,x,y, yaw):
        """Return a pose that is moved relative to the given coordinates
        Y is ahead and x is to the right
        """
        _,_,current_yaw = quatToEuler(pose.orientation)
        new_pose = Pose()
        new_pose.position.y = pose.position.y + y*math.sin(current_yaw) - x*math.cos(current_yaw)
        new_pose.position.x = pose.position.x + y*math.cos(current_yaw) + x*math.sin(current_yaw)
        new_pose.orientation = eulerToQuat([0,0,current_yaw + yaw])

        return new_pose

    def pub_goal(self, target):
        goal  = PoseStamped()
        print(target)
        goal.pose = target
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)

    def wait_for_auto(self):
        print("waiting for auto")
        while not rospy.is_shutdown():
            str_msg = rospy.wait_for_message("/relay_state", String)
            print("got msg", str_msg)
            if str_msg.data == "GREEN":
                print("In AUTO")
                return True


    def start_docking(self):
        init_wp = Pose()
        init_wp.position.x= -88.04598300659012
        init_wp.position.y= 55.949736008471476
        init_wp.position.z= -0.0032290087531347115
        init_wp.orientation.x= -0.0037506085453014783
        init_wp.orientation.y= 0.008951646844160988
        init_wp.orientation.z= 0.7758399107350608
        init_wp.orientation.w= 0.6308551607658687

        wp2 = self.translatePose(init_wp, 0, 0, -1.15707)

        before_dock = Pose()
        before_dock.position.x= -53.244731748078934
        before_dock.position.y= 78.76219442719189
        before_dock.position.z= 0.0018826054630917475
        before_dock.orientation.x= 0.008768889507804231
        before_dock.orientation.y= 0.003969704953675528
        before_dock.orientation.z= -0.5651634040939038
        before_dock.orientation.w= 0.8249228295376312

        before_before = self.translatePose(before_dock, 0, 0, 1.5707)




        inside_dock = Pose()
        inside_dock.position.x= -43.982836170938945
        inside_dock.position.y= 57.39829510189041
        inside_dock.position.z= -0.022037335647920486
        inside_dock.orientation.x= 0.00814107289305371
        inside_dock.orientation.y= 0.004903033451636412
        inside_dock.orientation.z= -0.5711863892790585
        inside_dock.orientation.w= 0.8207653695773683

        wp3 = self.translatePose(inside_dock, 1, -14, 1.5707)
        wp4 = self.translatePose(inside_dock, 1, -12, 0)
        # wp5 = self.translatePose(inside_dock, 3, -12, -0.4)
        # time.sleep(1)
        # print("going_to_init")
        self.go_to_pose(init_wp, 10, 0.7)

        self.go_to_pose(wp2, 10, 0.4)

        self.go_to_pose(wp3, 5, 0.3)
        self.go_to_pose(wp4, 1, 0.3)
        self.go_to_pose(inside_dock, 2, 0.3)




        # time.sleep(3)
        # print("going_to_init_rotated")

        # self.go_to_pose(rotated_from_init, 3, 0.3)
        # time.sleep(3)

        # # print("going_to_before_dock")
        # # self.go_to_pose(before_before, 3, 0.3)
        # # time.sleep(3)

        # # self.go_to_pose(before_dock, 3, 0.3)
        # # time.sleep(3)
        # # print("going_to_in_dock")

        # self.go_to_pose(just_before_dock, 3, 0.3)
        # time.sleep(3)
        # print("going_to_in_dock")

        # self.go_to_pose(inside_dock)



        # while(True):
        #     print("Starting Docking")

        #     time.sleep(1)
        #     print("going_to_init")
        #     self.pub_goal(init_wp)

        #     time.sleep(1)
        #     self.pub_goal(wp2)
        #     time.sleep(1)
        #     self.pub_goal(wp3)
        #     time.sleep(1)
        #     self.pub_goal(wp4)
        #     time.sleep(1)
        #     self.pub_goal(inside_dock)
            # print("going_to_init_rotated")

            # self.pub_goal(rotated_from_init)
            # time.sleep(1)

            # print("going_to_in_dock")
            # self.pub_goal(before_before)
            # time.sleep(1)

            # self.pub_goal(before_dock)
            # time.sleep(1)

            # self.pub_goal(just_before_dock)

            # time.sleep(1)
            # self.pub_goal(inside_dock)
            # break
    


if __name__ == "__main__":
    ds = DockingSolution()

    ds.wait_for_auto()
    ds.start_docking()
