#!/usr/bin/env python3

from re import T
import rospy
from usyd_vrx_msgs.msg import ObjectArray, Object
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
import math
import tf

from scipy.spatial.transform import Rotation as R
# from buoy_scanner import Scanner
from cv_bridge import CvBridge, CvBridgeError

# from sensor_msgs.msg import Image

# from vrx_gazebo.srv import ColorSequence, ColorSequenceRequest, ColorSequenceResponse

import numpy as np

import math

# from placard_classifier import PlacardClassifier
import cv2

class Mission():

    def __init__(self):
        print("Initalising Mission Base")

        self.object_list = []
        self.used_objects = []
        self.unused_objects = []
        self.current_pose = Pose()
        self.tf_listener = tf.TransformListener()
        self.object_sub = rospy.Subscriber("/wamv/classified_objects", ObjectArray, self.objectsCb)
        self.odom_sub = rospy.Subscriber("/wamv/odom/", Odometry, self.odomCb)
        #self.waypoint_pub = rospy.Publisher("waypoints_cmd", WaypointRoute,queue_size = 10)
        self.marker_pub = rospy.Publisher("nav_marker", Marker, queue_size=10)
        self.goal_pub = rospy.Publisher("wamv/global_planner/goal", PoseStamped, queue_size = 10)
        self.direct_pub = rospy.Publisher("wamv/desired_pose", Pose, queue_size = 10)


    def getDist(self,pose1,pose2):
        return math.sqrt((pose1.position.x-pose2.position.x)**2 + (pose1.position.y-pose2.position.y)**2 )

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

    def navigateTo(self, target, wait=True, timeout = 0, dist_thresh = 1, ang_thresh = 0.4, repubish = False):

        self.navigate_to(target, timeout=timeout, dist_thresh=dist_thresh, ang_thresh=ang_thresh)
        return
        self.publishMarker(target)
        # Navigate to the location, if wait is True: Wait until destination is reached, if not,
        self.navigateToDirect(target,wait=False,timeout=0)

        print("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
        waypoint_msg = WaypointRoute()
        waypoint_msg.speed = 3
        ##For Now, waypoints are 2 waypoints. At nav waypoint then a nav station
        goal  = PoseStamped()
        goal.pose = target
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        self.goal_pub.publish(goal)
        start = rospy.Time.now().secs
        rate = rospy.Rate(20)
        expired = False
        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait and not expired:
            rate.sleep()
            if timeout != 0 and rospy.Time.now().secs-start > timeout:
                expired = True

        #Republish a waypoint in its own position if wanted to wait, else, it will exit the function but continue on its trajectory.
        if repubish:
            waypoint_msg = WaypointRoute()
            waypoint_msg.speed = 2
            loc0 = Waypoint()
            loc0.pose = self.current_pose
            loc0.nav_type = Waypoint.NAV_STATION
            loc0.station_duration = -1
            waypoint_msg.waypoints=[loc0]
            self.waypoint_pub.publish(waypoint_msg)

        if wait:
            print("Arrived at target")
        return

    def navigateToDirect(self, target, wait=True, timeout = 0, dist_thresh = 1, ang_thresh = 0.4):
        """ Navigate to the location, if wait is True: Wait until destination is reached, if not,  Not using the mission planner"""
        self.publishMarker(target)
        print("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
        waypoint_msg = WaypointRoute()
        waypoint_msg.speed = 2
        ##For Now, waypoints are 2 waypoints. At nav waypoint then a nav station

        loc2 = Waypoint()
        loc2.pose = target
        loc2.nav_type = Waypoint.NAV_STATION
        loc2.station_duration = -1
        waypoint_msg.waypoints=[loc2]
        self.waypoint_pub.publish(waypoint_msg)

        start = rospy.Time.now().secs
        rate = rospy.Rate(20)
        expired = False
        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait and not expired:
            #print(target,self.current_pose)
            rate.sleep()
            if timeout != 0 and (rospy.Time.now().secs-start) > timeout:
                print("Timeing out: %f", rospy.Time.now().secs-start)
                expired = True

        #Republish a waypoint in its own position if wanted to wait, else, it will exit the function but continue on its trajectory.
        if wait:
            waypoint_msg = WaypointRoute()
            waypoint_msg.speed = 2
            loc0 = Waypoint()
            loc0.pose = self.current_pose
            loc0.nav_type = Waypoint.NAV_STATION
            loc0.station_duration = -1
            waypoint_msg.waypoints=[loc0]
            self.waypoint_pub.publish(waypoint_msg)


        print("Arrived at target")
        return

    def navigate_to(self, target, timeout = 30, dist_thresh = 1, ang_thresh = 0.4, wait=True):
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = "map"
        desired_pose.pose = target
        start = rospy.Time.now().secs

        self.goal_pub.publish(desired_pose)
        rate = rospy.Rate(20)

        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait:
            #print(target,self.current_pose)
            rate.sleep()
            if timeout != 0 and (rospy.Time.now().secs-start) > timeout:
                print("Timeing out: %f", rospy.Time.now().secs-start)
                break

        print("Arrived at target")
        return

    def navigate_to_direct(self, target, timeout = 30, dist_thresh = 1, ang_thresh = 0.4, wait=True):

        start = rospy.Time.now().secs

        self.direct_pub.publish(target)
        rate = rospy.Rate(20)

        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait:
            #print(target,self.current_pose)
            rate.sleep()
            if timeout != 0 and (rospy.Time.now().secs-start) > timeout:
                print("Timeing out: %f", rospy.Time.now().secs-start)
                break

        print("Arrived at target")
        return

    def inRange(self, target, dist_thresh = 1.2, ang_thresh = 0.4):

        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)
        #print("Range: %f", dist)
        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= ang_thresh):
            return True
        else:
            return False


    def odomCb(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
        return


    def objectsCb(self,msg):

        # print("Got objects")
        self.object_list = msg.objects

        #Remove unused objects from a list.

        self.unused_objects = self.object_list[:]
        for i in self.unused_objects:
            if i is None:
                self.unused_objects.remove(i)

        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id

            for i in self.used_objects:
                if i is not None:
                    if i.frame_id == id:
                        #If there is a match, remove the obect
                        self.unused_objects.remove(object)
                        break
        return


    def updateUnused(self):
        for object in self.unused_objects[:]:
            #Check if object in in the used object list.
            id = object.frame_id
            if object is not None:
                for i in self.used_objects:
                    #print("Checking unused: %s with used %s",id,i.frame_id)
                    if i is not None:
                        if i.frame_id == id:
                            #If there is a match, remove the obect
                            print("Found used object %s",object.frame_id)
                            try:
                                self.unused_objects.remove(object)
                                break
                            except:
                                continue
                    else:
                        self.used_objects.remove(i)

        return

    def publishMarker(self, pose,id = 0):

        if pose is None:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nav_task"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action=Marker.ADD
        marker.pose = pose
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)
    #
    # def findObject(self,object_list,frame_id):
    #     if object_list is None:
    #         rospy.logwarn("Find Closest passed empty list")
    #         return None
    #     for object in object_list:
    #         #Get distance of object
    #         if object.best_guess in accepted_objects and object.confidences[0]>conf_thresh:
    #



    def exploreFor(self,type="object",conf_thresh = 0.3):
        """ Try find an object by navigating around"""
        print("Looking for %s", type)
        current_target_object = Object()
        current_target_object.frame_id = ""
        target = None
        random_target = None
        object_search_list_used = []
        object = None
        count = 0
        while object is None:
            object_search_list = self.unused_objects[:]
            for object in object_search_list[:]:
                for i in object_search_list_used:
                    if i.frame_id == object.frame_id:
                        object_search_list.remove(object)
                        break
            #Execute explore for object
            object = self.findClosest(object_search_list, type=type, conf_thresh = conf_thresh)
            if object is None:
                #print("Cant find, looking for low confidnece")
                guess = self.findClosest(object_search_list, type=type, conf_thresh = 0)
                if guess is None:
                    #print("Cant find, looking for Any Object")
                    guess = self.findClosest(object_search_list, type="object", conf_thresh = 0)
                    if guess is None:
                        rospy.logwarn("No Objects Found... at all")
                        #Go to a random posiiont:
                        count = count +1

                        if (random_target is None or self.inRange(random_target,dist_thresh = 6, ang_thresh = 1)) and count >10:
                            rospy.logwarn("Cannot find any new things to investigate, lets try a random position")
                            random_target = Pose()
                            random_target.position.x = 0
                            random_target.position.y = 50
                            random_target.orientation=self.current_pose.orientation
                            self.navigate_to(random_target, wait=False)
                            count = 0
                        rospy.sleep(0.5)
                        continue


                count = 0
                if guess is not None and current_target_object.frame_id != guess.frame_id:
                    print("Getting a new location to try ")
                    target = Pose()
                    target.position = guess.pose.position
                    target.orientation = self.current_pose.orientation
                    target = self.translatePose(target,0,-15,0)
                    current_target_object = guess
                    self.navigate_to(target,wait=False)

                if target is not None and current_target_object is not None and (self.inRange(target) or (len(current_target_object.confidences) != 0 and current_target_object.confidences[0]>0.8)):
                    #If it is in at the nav location or the classificaiton is very high
                        ##Remove the item in object search list:
                    print("Removing object because its not what we are looking for")
                    object_search_list_used.append(current_target_object)

                #Update the object search list




                rospy.sleep(0.5)

            else:
                print("Found object %s whilst exploring",type)
                break


        return object


    def findClosest(self,object_list, type="object",frame="base_link", conf_thresh = 0,ignore_land = False):

        if object_list is None:
            rospy.logwarn("Find Closest passed empty list")
            return None
        tf_listener = self.tf_listener
        closest = None
        min_dist = None
        if type =="object":
            accepted_objects = ["dock", "buoy", "mb_marker_buoy_red", "mb_marker_buoy_white", "mb_marker_buoy_black", "mb_marker_buoy_green", "mb_marker_buoy_orange", "mb_round_buoy_red", "mb_round_buoy_white", "mb_round_buoy_black", "mb_round_buoy_green", "mb_round_buoy_orange"]
        elif type == "buoy":
            accepted_objects = ["buoy", "mb_marker_buoy_red", "white_marker_buoy", "mb_marker_buoy_black", "mb_marker_buoy_green", "mb_marker_buoy_orange", "mb_round_buoy_red", "mb_round_buoy_white", "mb_round_buoy_black", "mb_round_buoy_green", "mb_round_buoy_orange"]
        elif type == "totem":
            accepted_objects = ["mb_marker_buoy_red", "white_marker_buoy", "mb_marker_buoy_black", "mb_marker_buoy_green", "mb_round_buoy_orange"]
        elif type == "polyform":
            accepted_objects= ["mb_round_buoy_red",  "mb_round_buoy_black", "mb_round_buoy_orange"]
        elif type == "nav":
            accepted_objects = ["mb_marker_buoy_red", "white_marker_buoy", "mb_marker_buoy_black", "mb_marker_buoy_green", "mb_marker_buoy_orange", "mb_round_buoy_red",  "mb_round_buoy_black", "mb_round_buoy_orange", "buoy"]
        elif type == "white":
            accepted_objects = ["mb_marker_buoy_white"]
        elif type == "green":
            accepted_objects = ["mb_marker_buoy_green"]
        elif type == "red":
            accepted_objects = ["mb_marker_buoy_red", "mb_marker_buoy_orange"]
        elif type == "black":
            accepted_objects = ["mb_marker_buoy_black", "mb_round_buoy_black"]
        else:
            accepted_objects = [type]

        for object in object_list:
            #Get distance of object
            if object.best_guess in accepted_objects and object.best_confidence>conf_thresh:

               
                object_pose = object.pose

                x = object_pose.position.x - self.current_pose.position.x
                y = object_pose.position.y - self.current_pose.position.y

                dist = math.sqrt(x**2 +y*2)
                if min_dist is None or dist<min_dist:
                    closest = object
                    min_dist = dist

        if closest is None:
            print("object type :%s not found", type)
        return closest


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

class Gymkhanna(Mission):

    def __init__(self):
        rospy.init_node("scan_dock_deliver")

        print("Initalizing  Entrance Gate")

        # rospy.Subscriber("/wamv/sensors/pingers/pinger/range_bearing", RangeBearing, self.pinger_cb)
        super().__init__()

    def findGate(self,left_colour, right_colour,thresh=0.4):
        left = self.findClosest(self.unused_objects,type=left_colour, conf_thresh = thresh)
        right = self.findClosest(self.unused_objects,type=right_colour, conf_thresh = thresh)

        if left and right:
            # Get distance, verify it is within acceptable range
            #Then navigate
            dist = math.sqrt((left.pose.position.x-right.pose.position.x)**2 + (left.pose.position.y-right.pose.position.y)**2 )
            if dist < 28 and dist > 3:
                print("Found %s and %s buoys ID %s and ID %s", left_colour , right_colour,left.frame_id, right.frame_id )
                self.used_objects.append(left)
                self.used_objects.append(right)
                self.updateUnused()
                #Navigate to the gate in between
                return self.getGatePose(left,right)

            else:
                #Distance between buoys are not correct.
                #Use the Closest one.
                print("Distance between found buoys is %f and it is too long", dist)
                try:
                    trans = [left.pose.position.x, left.pose.position.y]#$ , tf_listener.lookupTransform("base_link",left.frame_id, rospy.Time(0))
                    trans2 = [right.pose.position.x, right.pose.position.y] # tf_listener.lookupTransform("base_link",right.frame_id, rospy.Time(0))
                except Exception as e:
                    rospy.logwarn("Transform Lookup Error")
                    print(e)
                    return None

                dist_left = math.sqrt(trans[0]**2 +trans[1]**2)
                dist_right = math.sqrt(trans2[0]**2 +trans2[1]**2)
                if dist_left <dist_right:
                    left = None
                else:
                    right = None



        if left is None and right:

            #CHECK IF BLUE TOTEM IS VIABLE
            left = self.findClosest(self.unused_objects,type="green_marker_buoy", conf_thresh = thresh)
            if left is not None:
                dist = math.sqrt((left.pose.position.x-right.pose.position.x)**2 + (left.pose.position.y-right.pose.position.y)**2 )
                if dist < 22 and dist > 5:
                    self.used_objects.append(left)
                    self.used_objects.append(right)
                    self.updateUnused()
                    #Navigate to the gate in between
                    return self.getGatePose(left,right)
                else:
                    left = None


            #Try find the missing left object.
            #Navigate to the buoy in two second and try find the right buoy
            print("No %s Buoy found trying to find left", left_colour)
            attempts = 0
            while left is None:
                print("Navigating closer to see if it works")
                dist = self.getDist(right.pose, self.current_pose)
                if dist < 10:
                    left = self.findClosest(self.unused_objects,type=left_colour)
                    right = self.findClosest(self.unused_objects,type=right_colour)
                target = Pose()
                target.position = right.pose.position
                target.orientation = self.current_pose.orientation
                target = self.translatePose(target,-9,0,0)
                self.publishMarker(target)
                self.navigateTo(target, timeout = 2, dist_thresh = 6)
                if self.getDist(self.current_pose,target)<8:
                    left = self.findClosest(self.unused_objects,type="object", frame=right.frame_id, conf_thresh = 0)
                    break;
                left = self.findClosest(self.unused_objects,type=left_colour, conf_thresh = thresh-0.1)
                if left is not None:
                    dist = math.sqrt((left.pose.position.x-right.pose.position.x)**2 + (left.pose.position.y-right.pose.position.y)**2 )
                    if dist < 22 and dist > 3:
                        break
                    else:
                        left = None
                attempts = attempts+1

            self.used_objects.append(right)
            self.used_objects.append(left)
            self.updateUnused()

            return self.getGatePose(left,right)
            #self.updateUnused()


        elif right is None and left:

            print("No %s Buoy found trying to find right", right_colour)

            while right is None:
                print("Navigating closer to left see if it works")
                dist = self.getDist(left.pose, self.current_pose)
                if dist < 10:
                    left = self.findClosest(self.unused_objects,type=left_colour)
                    right = self.findClosest(self.unused_objects,type=right_colour)
                target = Pose()
                target.position = left.pose.position
                target.orientation = self.current_pose.orientation
                target = self.translatePose(target,+6,0,0)
                self.publishMarker(target)
                self.navigate_to(target,timeout = 2, dist_thresh = 6)

                if self.getDist(self.current_pose,target)<8:
                    right = self.findClosest(self.unused_objects,type="object", frame_id=left.frame_id, conf_thresh = 0)
                    break;

                right = self.findClosest(self.unused_objects,type=right_colour, conf_thresh = thresh-0.1)
                if right is not None:
                    dist = math.sqrt((left.pose.position.x-right.pose.position.x)**2 + (left.pose.position.y-right.pose.position.y)**2 )
                    if dist < 17 and dist > 3:
                        break
                    else:
                        left = None
            self.used_objects.append(right)
            self.used_objects.append(left)
            self.updateUnused()
            return self.getGatePose(left,right)


        elif right is None and left is None:
            print("Cannot find %s buoy and %s buoy", left_colour, right_colour)
            print(self.unused_objects)
            return

        print("Found %s and %s buoys ID %s and ID %s", left_colour , right_colour,left.frame_id, right.frame_id )
        self.updateUnused()
        #Navigate to the gate in between
        return self.getGatePose(left,right)


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

    def getGatePose(self,left_buoy,right_buoy):
        global tf_listener
        if left_buoy is None or right_buoy is None:
            rospy.logwarn("GetGatePose Recievned a none type")
            return None
        target = Pose()
        target.position.x = 0.0
        target.position.y = 0.0
        target.position.x = (left_buoy.pose.position.x + right_buoy.pose.position.x) /2.0
        target.position.y = (left_buoy.pose.position.y + right_buoy.pose.position.y) /2.0
        target.position.z = 0.0

        # if they are the same type, the orientation is the same as the curret position orientation
        if (left_buoy.best_guess == right_buoy.best_guess):
            target.orientation = self.current_pose.orientation
        else:
            yaw = 0.0
            yaw = math.atan2(left_buoy.pose.position.y - right_buoy.pose.position.y, left_buoy.pose.position.x - right_buoy.pose.position.x)-1.507
            print("Got gate location x: %f, y: %f, yaw: %f",target.position.x ,target.position.y,yaw)
            target.orientation = eulerToQuat([0,0,yaw])
        #print(left_buoy,right_buoy, target)
        return target



    def inRange(self, target, dist_thresh = 0.5, ang_thresh = 0.4):
        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)

        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= ang_thresh):
            return True
        else:
            return False

    def start(self):
        print("Entrance Gates")


        print("Sleeping for 5 seconds")
        rospy.sleep(5)

        target = None
        # while target is None:
        #     target_buoy = self.findClosest(self.unused_objects, "buoy")
        #     if target_buoy is not None:
        #         target=target_buoy.pose
        #     rospy.sleep(0.01)

        # target.orientation = self.current_pose.orientation
        # target = self.translatePose(target,0,-10,0)

        # print("Navigating to nearest buoy for 5 seconds")
        # self.navigate_to(target,timeout = 30, dist_thresh = 6)
        # print("Waiting 5 seconds")
        # rospy.sleep(5)

        print("attempting to find red/white gate")
        entrance_gate = self.findGate("red","white", thresh=0.2)

        print("attempting to find red/white gate")

        entrance_gate_target = self.translatePose(entrance_gate, 4, 0, 0)
        if target is None:
            print("Could not find bouy")
            return

        print("navigating to find red/white gate")

        self.navigate_to(entrance_gate_target,dist_thresh = 2, ang_thresh = 1)

        print("naavigated to find red/white gate")

        print("finding black pole")

        black_target = self.findClosest(self.unused_objects, "black", thresh=0.4)

        print("found_black_pole")


        black_target_pose = Pose()
        black_target_pose.position = black_target.pose.position
        black_target_pose.orientation = self.current_pose.orientation
        # target = self.translatePose(target,-9,0,0)

        print("navigting to left side")


        first_waypoint = self.translatePose(black_target_pose, 10, 10, 0)
        self.navigate_to(first_waypoint,dist_thresh = 2, ang_thresh = 1)

        print("navigting to right side")

        second_waypoint =  self.translatePose(black_target_pose, 10, -10, 3.14159)
        self.navigate_to(second_waypoint,dist_thresh = 2, ang_thresh = 1)

        print("navigting to exit side")
        exit_gate = self.translatePose(entrance_gate, -4, 0, 3.14159)

        self.navigate_to(exit_gate,dist_thresh = 2, ang_thresh = 1)
        print("Done")

        # attempts = 0
        # #rospy.sleep(5)
        # thresh = 0.4
        # while True:

        #     target = self.findGate("green","red", thresh = thresh)
        #     if target is None:
        #         rospy.sleep(5)
        #         print("Found no target, waiting 5 seconds")
        #         attempts = attempts +1
        #         if attempts < 4:
        #             thresh = thresh-0.05
        #             continue
        #         else:
        #             break

        #     target = self.translatePose(target,0,-1,0)
        #     self.publishMarker(target)
        #     self.navigate_to(target,dist_thresh = 2, ang_thresh = 1)

        #     target = self.translatePose(target,0,4,0)
        #     self.publishMarker(target)
        #     self.navigate_to_direct(target,dist_thresh = 2, ang_thresh = 1)

        #         #rospy.sleep(5)
        # print("No  more docks found")

        # target = self.findGate("black","red")
        # if target is None:
        #     return
        # target = self.translatePose(target,0,0.1,0)
        # self.publishMarker(target)
        # self.navigateTo(target)

    pinger_pos = None
    def pinger_cb(self, data):
        print("pinger", data)

if __name__ == "__main__":
    g = Gymkhanna()
    g.start()

    rospy.spin()
