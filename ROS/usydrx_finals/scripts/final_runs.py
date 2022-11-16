#!/usr/bin/env python3
from cv2 import BaseCascadeClassifier
from geometry_msgs.msg import PoseStamped, Pose
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Int32
import math
import tf
import time
from scipy.spatial.transform import Rotation as R

class PathType:
    GLOBAL = 1# uses costmap to generate path
    STRAIGHT = 2 # straight line to desired point using path following
    PURE = 3 # Go straight to waypoint using holonomics

PREVIEW = False

ENTRANCE_GATE = 1 # 1, 2 or 3

PATH_TYPE = PathType.GLOBAL  

# Position to LOOK At the light buoy from
LIGHT_BUOY_POSE = Pose(Point(229.5974163375557, 175.85892895334368, -0.0013672306272796394), Quaternion(-0.0016108224776182323, 0.007911579800763978, 0.5278171950906738, 0.8493196222404233))

RANDOM_LIGHT_BUOY_GUESS = "RBG"



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


class FinalsSolution:


    current_pose = None
    def __init__(self):
        rospy.init_node("docking_solution")

        rospy.Subscriber("/wamv/odom", Odometry, self.odom_cb)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
        self.pure_pub = rospy.Publisher("/wamv/desired_pose", PoseStamped, queue_size = 10)
        self.straight_pub = rospy.Publisher("/straight_path", PoseStamped, queue_size = 10)
        self.light_buoy_pub = rospy.Publisher("/light_sequence", String, queue_size=10)



        self.entrance_gate_hb_pub = rospy.Publisher("/entrance_gate", Int32, queue_size=10)

        print("Waiting for odom")
        self.current_pose = self.translate_pose(rospy.wait_for_message("/wamv/odom", Odometry).pose.pose, 0, 0, 0)
        print("Got odom")


    def odom_cb(self, data):
        # print("got current pose")
        self.current_pose = self.translate_pose(data.pose.pose, 0, 0, 0)

    def go_to_pose(self, target, dist_range=5, angle_range=0.3, path_type=PATH_TYPE):
        
        self.publish_pose(target, path_type)
        

        if PREVIEW:
            time.sleep(2)
            return

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

    def translate_pose(self,pose,x,y, yaw):
        """Return a pose that is moved relative to the given coordinates
        x is ahead and y is to the left
        """
        _,_,current_yaw = quatToEuler(pose.orientation)
        new_pose = Pose()
        R.from_euler('xyz', [0,0,current_yaw])

        new_pose.position.y = pose.position.y + x*math.sin(current_yaw) + y*math.cos(current_yaw)
        new_pose.position.x = pose.position.x + x*math.cos(current_yaw) - y*math.sin(current_yaw)
        new_pose.orientation = eulerToQuat([0,0,current_yaw + yaw])

        return new_pose

    def publish_pose(self, target, path_type=PATH_TYPE):

        goal  = PoseStamped()
        goal.pose = target
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        


        if path_type == PathType.GLOBAL:
            print("Using Global Planner")
            self.goal_pub.publish(goal)
        elif path_type == PathType.STRAIGHT:
            print("Using Straight Planner")
            self.straight_pub.publish(goal)
        elif path_type == PathType.PURE:
            print("Using Pure Desired Pose")
            self.pure_pub.publish(goal)
        else:
            print("Error going to pose, not the right type")


    def wait_for_auto(self):
        print("Waiting for Auto Mode")
        while not rospy.is_shutdown():
            str_msg = rospy.wait_for_message("/relay_state", String)
            print("Got Relay Message: ", str_msg)
            if str_msg.data == "GREEN":
                print("In AUTO")
                return True
    

    starting_position = None
    def navigate_entrance(self):
        self.entrance_gate_hb_pub.publish(ENTRANCE_GATE)
        self.starting_position = self.current_pose
        after_entrance_gate = self.translate_pose(self.starting_position, 20, 0, 0)

        print("Entrance Gate: Starting at current position")
        self.go_to_pose(self.starting_position , 5, 0.3, PathType.STRAIGHT)
        print("Entrance Gate: Going to position after entrance gate")
        self.go_to_pose(after_entrance_gate, 5, 0.3)


    def navigate_exit(self):
        if self.starting_position is None:
            print("Starting position is None")
            return

        self.entrance_gate_hb_pub.publish(ENTRANCE_GATE)
        exit_position = self.translate_pose(self.starting_position, 0, 0, math.pi)
        before_exit_gate = self.translate_pose(exit_position, -20, 0, 0)
        print("Exit Gate: Going to position before entrance gate")
        self.go_to_pose(before_exit_gate, 5, 0.3)
        print("Exit Gate: Going to position after entrance gate")
        self.go_to_pose(exit_position, 5, 0.3, PathType.STRAIGHT)


    def scan_the_code(self):

        from scan_the_code import ScanCode

        ls = ScanCode()
        try:
            sequence = ls.scanBuoy()
            if sequence is not None:
                msg_dict = {"red": "R", "green": "G", "blue": "B"}
                sequence_str = f"{msg_dict[sequence[0]]}{msg_dict[sequence[1]]}{msg_dict[sequence[2]]}"
                return sequence_str
            return RANDOM_LIGHT_BUOY_GUESS
        except BaseException as e:
            print(e)
            return RANDOM_LIGHT_BUOY_GUESS

    def execute_scan_the_code(self):

        # self.go_to_pose(LIGHT_BUOY_POSE, 0.6)
        print("Scanning the code")
        time.sleep(5)
        code = self.scan_the_code()
        self.light_buoy_pub.publish(code)
        print("Scanned the code")
        return

    def do_finals(self):
        self.navigate_entrance()

        self.execute_scan_the_code()

        self.navigate_exit()       
        # print(self.scan_the_code())

if __name__ == "__main__":
    ds = FinalsSolution()

    time.sleep(0.2)

    # ds.wait_for_auto()
    # ds.start_docking()
    ds.do_finals()
