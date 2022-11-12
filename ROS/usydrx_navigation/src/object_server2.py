#!/usr/bin/env python3

from tokenize import String
import rospy

from nav_msgs.msg import OccupancyGrid

import scipy.cluster.hierarchy as hcluster

import numpy as np
import imageio
import math
import os

from matplotlib import pyplot as plt

from scipy.spatial.transform import Rotation as R

from usyd_vrx_msgs.msg import ObjectArray, Object
from std_msgs.msg import String


from objhelper.qhull_2d import *
from objhelper.min_bounding_rect import *

from geometry_msgs.msg import Pose


OCCUPANCY_GRID_VALUE_THRESHOLD = rospy.get_param('~threshold', 40)

DIST_THRESH = rospy.get_param('~distance_threshold',5); #Distance between clusters before it is condidered seperate


CLOSE_OBJECT_DISTANCE_THRESH = 2.0  # Distance Threshold to conider detected object the same object


BUOY_RADIUS = 3  # m or above
DOCK_RADIUS = (8, 30)  # min, max
LAND_RADIUS = 30  # m or above

class Obstacle:
    def __init__(self, frame_id) -> None:
        self.x = 0
        self.y = 0
        self.orientation = R.from_euler('xyz', [0,0,0]).as_quat()
        self.points = []
        self.radius = None
        self.parent_frame = "map"
        self.frame_id = f"{frame_id}"
        self.confidence = 0
        self.obj_type = ""

        self.time = rospy.Time.now()

    def generate_object(self):
        obj = Object()
        obj.frame_id = self.frame_id
        obj.name = f"{self.obj_type} {self.frame_id}"
        obj.best_guess = self.obj_type

        obj.best_confidence = self.confidence

        obj.pose = Pose()
        # obj.pose.position.x = self.x
        # obj.pose.position.y = self.y

        obj.pose.position.x = self.x # + 195
        obj.pose.position.y =  self.y # -104.8

        obj.pose.orientation.x = self.orientation[0]
        obj.pose.orientation.y = self.orientation[1]
        obj.pose.orientation.z = self.orientation[2]
        obj.pose.orientation.w = self.orientation[3]

        return obj

        pass

    def classify(self):

        obj_type = None
        confidence = 0
        
        if self.radius < BUOY_RADIUS:
            obj_type = "buoy"
            confidence = 0.2
        
        elif  DOCK_RADIUS[0] < self.radius < DOCK_RADIUS[1]:
            obj_type = "dock"
            confidence = 0.5

            # Clasify dock orientation

            points = np.array(self.points)
            try:
                hull_points = qhull2D(points)
            except RecursionError:
                print("GOT RECURSTION ERROR FOR OBJECT SERVER")
                return
            hull_points = hull_points[::-1]
            (rot_angle, area, length, width, center_point, corner_points) = minBoundingRect(hull_points)
            
            if length < width:
                rot_angle = rot_angle + 1.5707
                

            print(f"Found dock {corner_points}")

            


            rot_mat = np.array([[math.cos(rot_angle), -math.sin(rot_angle)],
                                 [math.sin(rot_angle), math.cos(rot_angle)]])

            _points = points - np.array(center_point)
            new_points = _points @ rot_mat


            test_points = [(0, 4), (-6, 4), (6, 4)]
            
            # p.min(np.linalg.norm(_p, axis=1))

            test_1 = np.min(np.linalg.norm(new_points- np.array((0, 4)), axis=1))

            test_2 = np.min(np.linalg.norm(new_points- np.array((-6, 4)), axis=1))

            test_3 = np.min(np.linalg.norm(new_points- np.array((6, 4)), axis=1))

            sum_1 = test_1 + test_2 + test_3


            test_4 = np.min(np.linalg.norm(new_points- np.array((0, -4)), axis=1))

            test_5 = np.min(np.linalg.norm(new_points- np.array((-6, -4)), axis=1))

            test_6 = np.min(np.linalg.norm(new_points- np.array((6, -4)), axis=1))

            sum_2 = test_4 + test_5 + test_6


            if sum_2 > sum_1:
                rot_angle = rot_angle - 3.14159

            dock_width = 8
            dock_length = 30

            self.orientation = R.from_euler('xyz', [0,0,rot_angle+1.5707]).as_quat()

            # print()W

        elif self.radius > LAND_RADIUS:
            obj_type = "land"
            confidence = 0.8

        else:
            obj_type = "unknown"
            confidence = 0.1

        if confidence > self.confidence:
            self.confidence = confidence
            self.obj_type = obj_type

class ObjectServer:

    map = None

    map_land_mask = None

    def __init__(self) -> None:
        rospy.init_node("object_server")
        sub = rospy.Subscriber("wamv/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/wamv/reset_objects",String,self.reset_callback)
        self.load_map_land_mask()
        self.obstacles = []
        self.cumulative_count = 0

        self.object_pub = rospy.Publisher("wamv/unclassified_objects", ObjectArray, queue_size=10)

    def reset_callback(self,data):
        self.obstacles = []
        self.map = None
    def load_map_land_mask(self):

        curr_path = os.path.dirname(__file__)
        im = imageio.imread(os.path.join(curr_path,'usyd_vrx_map.png'))

        im_grey = im[:,:,0:3].sum(axis=2)/3
        land_mask = np.less(im_grey, 200)
        land_mask = np.flip(land_mask, axis=0)

        self.map_land_mask = land_mask

        # plt.imshow(land_mask, interpolation='nearest')
        # plt.show()
        # print(im)

        pass

    def map_callback(self, data):
        # print("Received Map")
        self.map = data
        pass


    def process_map(self):
        if self.map is None:
            return
        my_map = self.map
        points_x = []
        points_y = []
        my_data = []
        info = my_map.info
        r = 0
        c = 0
        count  = 0
        #print(my_map)
        #Put map into a list of points.

        map_data_np = np.reshape(np.array(my_map.data), (1024, 1024))

        map_data_bool = np.greater(map_data_np, OCCUPANCY_GRID_VALUE_THRESHOLD)

        map_data_bool_non_land = np.logical_and(map_data_bool, np.logical_not(self.map_land_mask))


        # plt.imshow(map_data_np, interpolation='nearest')
        # plt.show()

        # data = map_data_np[self.map_land_mask]


        for r, row in enumerate(map_data_bool_non_land):
            for c, i in enumerate(row):
                if(i):
                    points_x.append(r*info.resolution)
                    points_y.append(c*info.resolution)
                    my_data.append((c*info.resolution, r*info.resolution))
                # if self.map_land_mask[r,c]:
                #     print(i)
                # # if in the land mask, continue
                #     continue
                # if i > OCCUPANCY_GRID_VALUE_THRESHOLD: #If the value of the cell is > THRESHOLD append
                

        # for i in my_map.data:
        #     if self.map_land_mask[r,c]:
        #         # if in the land mask, continue
        #         r = r+1
        #         if r == info.width:
        #             r = 0
        #             c = c +1
        #         continue
        #     if i > OCCUPANCY_GRID_VALUE_THRESHOLD: #If the value of the cell is > THRESHOLD append
        #         points_x.append(r*info.resolution)
        #         points_y.append(c*info.resolution)
        #         my_data.append((r*info.resolution,c*info.resolution))

            
        #     r = r+1
        #     if r == info.width:
        #         r = 0
        #         c = c +1
                
        if len(my_map.data)==0:
            rospy.logwarn("Object server recieved empty Map")
            return
        #Apply a distance threshold Cluster on the objects.
        #print(my_data,thresh)
        if len(my_data) == 0 :
            return
        try:
            clust = hcluster.fclusterdata(my_data, DIST_THRESH, criterion="distance")
        except Exception as e:
            #rospy.logwarn("Error occured clustering")
            my_data = np.concatenate((my_data, my_data))
            try:
                clust = hcluster.fclusterdata(my_data, DIST_THRESH, criterion="distance")
            except Exception as e:
                rospy.logwarn("Second attempt no working")
            #return
        clusters = {}
        count = 0
        for point in my_data:
            cluster_num = clust[count]
            if cluster_num not in clusters:
                clusters[cluster_num] = [point]
            else:
                clusters[cluster_num].append(point)
            count = count+1
        # print(len(clusters))
        #Iterate through the different colusters get the average centre point distance and size
        for cluster in clusters:
            sum_x = 0
            sum_y = 0
            for point in clusters[cluster]:
                sum_x = sum_x + point[0]
                sum_y = sum_y + point[1]
            avg = (sum_x/len(clusters[cluster]), sum_y/len(clusters[cluster]))

            
            max_dist = 0
            for point in clusters[cluster]:
                dist = math.sqrt((avg[0] - point[0])**2 + (avg[1] - point[1])**2)
                if dist>max_dist:
                    max_dist = dist

            #Get the Distance of the x and y axis
            x = avg[0] + info.origin.position.x
            y = avg[1] + info.origin.position.y

            #If the object is close to an already found object. Consider it the same object.
            updated = False
            current_frames = []
            name = ""
            for my_obj in self.obstacles:
                frame_id = my_obj.frame_id
                current_frames.append(frame_id)
                #Distance difference for it to be considered a new object
                thresh_dist = CLOSE_OBJECT_DISTANCE_THRESH
                dist = math.sqrt((my_obj.x-x)**2 + (my_obj.y-y)**2)
                if (dist<thresh_dist):
                    my_obj.x = x
                    my_obj.y = y

                    my_obj.radius = max_dist
                    my_obj.points = clusters[cluster]
                    updated = True
                    name = my_obj.frame_id
                    my_obj.time = rospy.Time.now()

                    # reclassify
                    my_obj.classify()
                    break
            if updated == False:
                #print("Adding new object", self.cumulative_count)
                frame_id = str(self.cumulative_count)
                self.cumulative_count=self.cumulative_count+1
                self.add_object(clusters[cluster],max_dist,x,y,frame_id)
                
        # print("STOP HERE")
        self.publish_objects()
        self.cleanup()

    def add_object(self, points,rad,x,y,frame_id):
        obstacle = Obstacle(frame_id)
        obstacle.x = x
        obstacle.y = y
        obstacle.radius = rad
        obstacle.points = points
        self.obstacles.append(obstacle)

        obstacle.classify()

    def publish_objects(self):
        object_array = ObjectArray()
        for obstacle in self.obstacles:
            object_array.objects.append(obstacle.generate_object())
        self.object_pub.publish(object_array)

    def start(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            r.sleep()
            self.process_map()
            # rospy.spin()
        pass

    def cleanup(self):
        """Method to clean up any objects that are old"""
        expire_time = 5
        for i in self.obstacles:
            time_diff = rospy.Time.now().secs - i.time.secs
            #print(i.object.frame_id, time_diff)
            if time_diff > expire_time:
                rospy.logdebug("Removing expired Object")
                self.obstacles.remove(i)


if __name__ == "__main__":

    object_server = ObjectServer()
    # object_server.cameraInit()
    # rate = rospy.Rate(30)
    # sub = rospy.Subscriber("wamv/map",OccupancyGrid, object_server.callback)

    object_server.start()