#!/usr/bin/env python3


import rospy
from usyd_vrx_msgs.msg import ObjectArray, Object
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
import math
import tf

from scipy.spatial.transform import Rotation as R
from buoy_scanner import Scanner
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from vrx_gazebo.srv import ColorSequence, ColorSequenceRequest, ColorSequenceResponse

import numpy as np

import math

from placard_classifier import PlacardClassifier
import cv2

class Mission():

    def __init__(self):
        print("Initalising Mission Base")

        self.object_list = []
        self.used_objects = []
        self.unused_objects = []
        self.current_pose = Pose()
        self.tf_listener = tf.TransformListener()
        self.object_sub = rospy.Subscriber("/wamv/unclassified_objects", ObjectArray, self.objectsCb)
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
        self.navigate_to(target, wait=True, timeout = timeout, dist_thresh = dist_thresh, ang_thresh = ang_thresh)
        return
        self.publishMarker(target)
        # Navigate to the location, if wait is True: Wait until destination is reached, if not,
        self.navigateToDirect(target,wait=False,timeout=0)

        rospy.loginfo("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
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
            rospy.loginfo("Arrived at target")
        return

    def navigateToDirect(self, target, wait=True, timeout = 0, dist_thresh = 1, ang_thresh = 0.4):
        """ Navigate to the location, if wait is True: Wait until destination is reached, if not,  Not using the mission planner"""
        self.publishMarker(target)
        rospy.loginfo("Navigating to a location x: %f. y:%f", target.position.x, target.position.y)
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
                rospy.loginfo("Timeing out: %f", rospy.Time.now().secs-start)
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


        rospy.loginfo("Arrived at target")
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
                rospy.loginfo("Timeing out: %f", rospy.Time.now().secs-start)
                break

        rospy.loginfo("Arrived at target")
        return

    def navigate_to_direct(self, target, timeout = 30, dist_thresh = 1, ang_thresh = 0.4, wait=True):

        start = rospy.Time.now().secs

        self.direct_pub.publish(target)
        rate = rospy.Rate(20)

        while (not self.inRange(target,dist_thresh = dist_thresh, ang_thresh = ang_thresh)) and wait:
            #print(target,self.current_pose)
            rate.sleep()
            if timeout != 0 and (rospy.Time.now().secs-start) > timeout:
                rospy.loginfo("Timeing out: %f", rospy.Time.now().secs-start)
                break

        rospy.loginfo("Arrived at target")
        return

    def inRange(self, target, dist_thresh = 1.2, ang_thresh = 0.4):

        dist = math.sqrt((self.current_pose.position.x-target.position.x)**2 + (self.current_pose.position.y-target.position.y)**2)
        #rospy.loginfo("Range: %f", dist)
        angle = abs(quatToEuler(self.current_pose.orientation)[2] - quatToEuler(target.orientation)[2])

        if (dist<=dist_thresh) and (angle <= ang_thresh):
            return True
        else:
            return False


    def odomCb(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
        return


    def objectsCb(self,msg):
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
                            rospy.loginfo("Found used object %s",object.frame_id)
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
        rospy.loginfo("Looking for %s", type)
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
                #rospy.loginfo("Cant find, looking for low confidnece")
                guess = self.findClosest(object_search_list, type=type, conf_thresh = 0)
                if guess is None:
                    #rospy.loginfo("Cant find, looking for Any Object")
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
                            self.navigateTo(random_target, wait=False)
                            count = 0
                        rospy.sleep(0.5)
                        continue


                count = 0
                if guess is not None and current_target_object.frame_id != guess.frame_id:
                    rospy.loginfo("Getting a new location to try ")
                    target = Pose()
                    target.position = guess.pose.position
                    target.orientation = self.current_pose.orientation
                    target = self.translatePose(target,0,-15,0)
                    current_target_object = guess
                    self.navigateTo(target,wait=False)

                if target is not None and current_target_object is not None and (self.inRange(target) or (len(current_target_object.confidences) != 0 and current_target_object.confidences[0]>0.8)):
                    #If it is in at the nav location or the classificaiton is very high
                        ##Remove the item in object search list:
                    rospy.loginfo("Removing object because its not what we are looking for")
                    object_search_list_used.append(current_target_object)

                #Update the object search list




                rospy.sleep(0.5)

            else:
                rospy.loginfo("Found object %s whilst exploring",type)
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
            accepted_objects = ["dock", "buoy", "scan_buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark46104", "surmark950400", "surmark950410"]
        elif type == "buoy":
            accepted_objects = ["buoy", "scan_buoy", "yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem", "polyform_a3", "polyform_a5", "polyform_a7", "surmark46104", "surmark950400", "surmark950410"]
        elif type == "totem":
            accepted_objects = ["yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem"]
        elif type == "polyform":
            accepted_objects= ["polyform_a3", "polyform_a5", "polyform_a7"]
        elif type == "nav":
            accepted_objects = ["surmark46104", "surmark950400","surmark950410","blue_totem", "polyform_a7", "buoy"]
        elif type == "white":
            accepted_objects = ["surmark46104"]
        elif type == "green":
            accepted_objects = ["surmark950400","blue_totem"]
        elif type == "red":
            accepted_objects = ["surmark950410"]
        else:
            accepted_objects = [type]

        for object in object_list:
            #Get distance of object
            if object.best_guess in accepted_objects and object.best_confidence>conf_thresh:

               
                object_pose = object.pose

                x = object_pose.position.x
                y = object_pose.position.x

                dist = math.sqrt(x**2 +y*2)
                if min_dist is None or dist<min_dist:
                    closest = object
                    min_dist = dist

        if closest is None:
            rospy.loginfo("object type :%s not found", type)
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


class ScanDockDeliver(Mission):
    pass
    def __init__(self):
        rospy.init_node("scan_dock_deliver")

        try:
            rospy.wait_for_service('/vrx/scan_dock_deliver/color_sequence',timeout = 10)
            self.report_seqence_service = rospy.ServiceProxy('/vrx/scan_dock_deliver/color_sequence',ColorSequence)
        except:
            rospy.logwarn("Failed to register service")
            self.report_seqence_service = None
        rospy.loginfo("Initalizing Scan and Docking")
        self.bridge = CvBridge()
        self.scanner = Scanner()
        self.placardClassifier = PlacardClassifier()

        self.shutdown_pub = rospy.Publisher("wamv/control_shutdown", Empty, queue_size=10)
        self.shoot_pub = rospy.Publisher("/dock_msg", Bool, queue_size=10)


        rospy.Subscriber("/wamv/sensors/cameras/front_camera/image_raw", Image, self.image_cb)
        super().__init__()

    def start(self):
        rospy.loginfo("Starting Scan and Docking")


        

        rospy.loginfo("Sleeping for 5 seconds")
        rospy.sleep(5)

        scan = self.exploreFor(type="buoy",conf_thresh = 0.1)

        if scan is None:
            rospy.logwarn("No buoys found")
            return

        scan_buoy_pose = Pose()
        scan_buoy_pose.position = scan.pose.position

        orientation = eulerToQuat([0, 0, math.atan2(scan_buoy_pose.position.y - self.current_pose.position.y, scan_buoy_pose.position.x - self.current_pose.position.x)])

        scan_buoy_pose.orientation = orientation


        target = self.translatePose(scan_buoy_pose,0,-8,0)


        print(target)
        
        print("Navigating to target")
        self.navigate_to(target)

        rospy.sleep(3)

        sequence = self.scanBuoy()

        print("THE SEQUENCE IS", sequence)
        if self.report_seqence_service:
            self.reportSequence(sequence)

        dock = self.exploreFor(type="dock", conf_thresh=0.35)

        dock = self.findClosest(self.unused_objects, type="dock",conf_thresh = 0.35)

        if dock is None:
            dock = self.findClosest(self.unused_objects, type="object")

            if dock is None:
                rospy.logwarn("Cannot find anything")
                return


        print("FOUND DOCK")

        dock_target = Pose()
        dock_target.position = dock.pose.position
        dock_target.orientation =  eulerToQuat([0, 0, quatToEuler(dock.pose.orientation)[2] + 3.14159])
        target = self.translatePose(dock_target,0,-20,0)
        self.navigate_to(target)


        dock = self.findClosest(self.unused_objects, type="dock",conf_thresh = 0.35)

        if dock is None:
            dock = self.findClosest(self.unused_objects, type="object")

            if dock is None:
                rospy.logwarn("Cannot find anything")
                return

        dock_target = Pose()
        dock_target.position = dock.pose.position
        dock_target.orientation =  eulerToQuat([0, 0, quatToEuler(dock.pose.orientation)[2] + 3.14159])


        dock_color = sequence[0]

        pattern_map = {'red': 'circle', 'green': 'triangle', 'blue': 'cross', 'yellow': 'rectangle'}
        pattern = pattern_map[sequence[2]]


        print(f"Target Dock is {dock_color}, {pattern}")

        target_dock_string = f"{dock_color}_{pattern}"

        station_1 = self.translatePose(dock_target,6.25,-9,0)
        station_2 = self.translatePose(dock_target, 0,-9,0)
        station_3 = self.translatePose(dock_target, -6.25,-9,0)

        self.navigate_to_direct(station_1)

        rospy.sleep(5)
        label = self.checkPlacard()

        if label == target_dock_string:
            station_1_dock = self.translatePose(dock_target,6.25,-1,0)
            self.navigate_to_direct(station_1_dock)
        else:
            self.navigate_to_direct(station_2)
            rospy.sleep(5)
            label = self.checkPlacard()

            if label == target_dock_string:
                station_2_dock = self.translatePose(dock_target, 0,-1,0)
                self.navigate_to_direct(station_2_dock)
            else:
                self.navigate_to_direct(station_3)
                rospy.sleep(5)
                label = self.checkPlacard()

                if label == target_dock_string:
                    station_3_dock = self.translatePose(dock_target, -6.25,-1,0)
                    self.navigate_to_direct(station_3_dock)
                else:
                    print("NO soltuions found docking anyway")
                    station_3_dock = self.translatePose(dock_target, -6.25,-1,0)

                    self.navigate_to_direct(station_3_dock)
                    
        self.shutdown_pub.publish(Empty())

        self.shoot_pub.publish(Bool(data=True))
        return



        
        # self.reportSequence(sequence)

        # curr_x = self.current_pose.position.x
        # curr_y = self.current_pose.position.y

        # theta = math.atan2(scan_buoy_pose.position.y - curr_y,scan_buoy_pose.position.x - curr_x)

        # target = Pose()
        # target.position.x = curr_x
        # target.position.y = curr_y
        # orien = R.from_euler('xyz', [0, 0, theta]).as_quat()
        # target.orientation.x = orien[0]
        # target.orientation.y = orien[1]
        # target.orientation.z = orien[2]
        # target.orientation.w = orien[3]

        # self.navigate_to(target)
        # # look at the target

    def image_cb(self, image):
        self.last_front_cam_img = image

    last_front_cam_img = None

    def scanBuoy(self):
        #scanner = Scanner()
        found = False
        sequence = []
        last_colour = "none"
        count = 0
        rospy.loginfo("Attempting to Scan buoy")

        last_img = None

        while(found == False):
            rospy.sleep(0.001)
            ros_image = self.last_front_cam_img
            if ros_image is last_img:
                continue

            if self.last_front_cam_img is None:
                continue
            last_img = ros_image

            
            image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            colour = self.scanner.scanBuoy(image)
            # print(colour)
            #rospy.loginfo("Detected colour %s",colour)
            if colour == "none":
                if last_colour != "none":
                    #if it is the first out of a reading
                    last_colour = "none"
                    count = 0
                count = count +1
                sequence = []
                continue
            if colour != last_colour :
                sequence.append(colour)
                last_colour = colour
                count = 0
            else:
                count = count+1

            if len(sequence)>=3:
                rospy.loginfo("Detected sequence is %s, %s,%s,",sequence[0],sequence[1],sequence[2])
                found = True
                return sequence

            if count > 100:
                rospy.logwarn("No change in colour found for 100 frames")
                return ["red", "green", "blue"]

        return sequence

    def reportSequence(self,sequence):
        rospy.logdebug("Reporting the sequence")
        msg = ColorSequence()
        msg.color1 = sequence[0]
        msg.color2 = sequence[1]
        msg.color3 = sequence[2]
        self.report_seqence_service(sequence[0], sequence[1], sequence[2])
        return

    def checkPlacard(self):
        label = ""
        attempts = 0
        last_img = None
        while label == "":
            # self.logDock("Classifying placard")
            rospy.sleep(0.001)
            ros_image = self.last_front_cam_img
            if ros_image is last_img:
                continue

            if self.last_front_cam_img is None:
                continue
            last_img = ros_image
            res = None

            image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            image2 = image[:, image.shape[1]//4:(3*image.shape[1]//4), :]
            
            label, _, _ = self.placardClassifier.classifyPlacard(image2)

            if attempts > 60:
                rospy.logwarn("Could not classify placard")
                image_path = '/home/johnsumskas/images/placard.png'
                # cv2.imwrite(image_path,image)
                break
            attempts = attempts+1




        # while res is None or res.success==False:
        #   self.logDock("Attempting to Classify")
        #   try:
        #     classifyPlacard = rospy.ServiceProxy('/wamv/wamv/classify_placard', ClassifyPlacard)
        #     res = classifyPlacard(ros_img)
        #   except rospy.ServiceException, e:
        #     self.logDock("Service call to /wamv/classify_placard failed: %s"%e)
        #     return False
        #
        # self.logDock("Placard classifier result: %s"%res.label)

        return label
        if label == self.placard_symbol:
            return True
        else:
            return False

if __name__ == "__main__":
    sdd = ScanDockDeliver()
    sdd.start()

    rospy.spin()