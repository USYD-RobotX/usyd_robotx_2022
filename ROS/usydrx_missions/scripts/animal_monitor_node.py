#!/usr/bin/env python3

from importlib.resources import path
import math
import rospy
import tf
# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoseStamped

from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from geographic_msgs.msg import GeoPath, GeoPose
from nav_msgs.msg import Path, Odometry
import numpy as np
import math
from std_msgs.msg import Float32



def get_xy_based_on_lat_long(lat,lon, name):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    olat = -33.724223
    olon = 150.679736
    
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

    # rospy.loginfo("#########  "+name+"  ###########")  
    # rospy.loginfo("LAT COORDINATES ==>"+str(lat)+","+str(lon))  
    # rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
    # rospy.loginfo("COORDINATES AXY==>"+str(xa)+","+str(ya))
    # rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

    return xg2, yg2



class AnimalWayfinder():

    goal_path = None
    animal_locations_recieved = 0
    animal_pose_list = Path()
    last_goal_path = Path()
    odom = Odometry()
    initial_animal_poses = Path()
    initial_pose_flag = 0
    delta = Path()

    animal_pose_count = 0
    goal_reached = 0
    


    def __init__(self):
        rospy.init_node('goal_relayer', anonymous=True)

        rospy.Subscriber("/vrx/wildlife/animals/poses", GeoPath, self.receive_desired_pose)
        rospy.Subscriber("/wamv/odom", Odometry, self.receive_odom)

        self.desired_path_pub = rospy.Publisher("/wamv/desired_path", Path, queue_size=10)
        

    def receive_odom(self, odom_recieved: Odometry):
        self.odom = odom_recieved


    def receive_desired_pose(self, goalGeoPath: GeoPath):


        #print("Animal List Recieved\n", goalGeoPath)

        # Declare header info for desired Path
        self.animal_pose_list = Path()

        self.animal_pose_list.header.stamp = rospy.Time.now()
        self.animal_pose_list.header.frame_id = "map"

        self.animal_pose_count = 0

        # Convert each geoPose in geoPath to a PoseStamped object for a Path object
        for animalGeoPose in goalGeoPath.poses:

            if animalGeoPose.header.frame_id in ["platypus","turtle"]:

                # print("#############################")
                # print(animalGeoPose.header.frame_id)
                # print("#############################")
                goal_lat = animalGeoPose.pose.position.latitude
                goal_lon = animalGeoPose.pose.position.longitude
                goal_x, goal_y = get_xy_based_on_lat_long(goal_lat,goal_lon, "Goal XY")

                # Create empty goal_pose to send as station keeping target
                animal_pose = None
                animal_pose = PoseStamped()

                animal_pose.header.stamp = rospy.Time.now()
                animal_pose.header.frame_id = animalGeoPose.header.frame_id

                # X and Y offsets to fix incorrect lat long to xy conversion
                xOffset = 905.923+0.3
                yOffset = -172.276

                animal_pose.pose.position.x = goal_x + xOffset
                animal_pose.pose.position.y = goal_y + yOffset
                animal_pose.pose.position.z = 0

                animal_pose.pose.orientation = animalGeoPose.pose.orientation

                
                self.animal_pose_list.poses.append(animal_pose)

                #count animal poses
                self.animal_pose_count = self.animal_pose_count+1
        
        #set flag -only get animal locations once
        self.animal_locations_recieved = 1
        
        if (self.animal_pose_count) and (self.initial_pose_flag == 0):
            self.initial_animal_poses = self.animal_pose_list
            self.initial_pose_flag = 1

        ## calcualte differential
        
        self.delta = None
        self.delta = Path()
        j = 0 
        while j < self.animal_pose_count:
            delta_x = self.animal_pose_list.poses[j].pose.position.x - self.initial_animal_poses.poses[j].pose.position.x
            delta_y = self.animal_pose_list.poses[j].pose.position.y - self.initial_animal_poses.poses[j].pose.position.y
            delta_z = 0

            temp = None
            temp = PoseStamped()

            temp.header.frame_id = rospy.Time.now()
            temp.header.stamp = self.animal_pose_list.poses[j].header.frame_id
            temp.pose.position.x = delta_x
            temp.pose.position.y = delta_y
            temp.pose.position.z = 0

            self.delta.poses.append(temp)

            j = j + 1


            #Generate path around animals here (5 meters clockwise working)

            # tempCircularPath = Path()
            # # Declare header info for desired Path
            # tempCircularPath.header.stamp = rospy.Time.now()
            # tempCircularPath.header.frame_id = "map"

            # for animal in animal_pose_list.poses:

            #     if animal.header.frame_id == "turtle":
            #         clockwise = False
            #         for pose in self.generate_circular_path_around_pose(animal,12.0,clockwise).poses:
            #             tempCircularPath.poses.append(pose)
            #     elif animal.header.frame_id == "platypus":
            #         clockwise = True
            #         for pose in self.generate_circular_path_around_pose(animal,12.0,clockwise).poses:
            #             tempCircularPath.poses.append(pose)
     

            #self.goal_path = tempCircularPath	

    def generate_circular_path_around_pose(self,target_pose, radius, clockwise=True, path_length=10):
        """
		Generate a circular path around some arbitrary position
		with a specified radius, clockwise (default: True), and circle accuracy 
		i.e. path_length (default: 8). Return the circular path.
		"""

        #centre = PoseStamped()       
        #centre = target_pose.position
		
        self.initial_animal_poses = self.animal_pose_list

        intervals = list(range(path_length))

        if clockwise:
            intervals.reverse()

		
        tempGoalPath = Path()

        # Declare header info for desired Path
        tempGoalPath.header.stamp = rospy.Time.now()
        tempGoalPath.header.frame_id = "map"

        rospy.loginfo("Generating Circular Coordinates")
        for n in intervals:

            # Create empty goal_pose to send as station keeping target
            goal_pose = PoseStamped()

            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"

            angle = (n/path_length) * ((2*math.pi)) 
			

            x1 = radius * math.cos(angle) + target_pose.pose.position.x
            y1 = radius * math.sin(angle) + target_pose.pose.position.y
            z1 = target_pose.pose.position.z
            
			
            goal_pose.pose.position.x = x1
            goal_pose.pose.position.y = y1
            goal_pose.pose.position.z = z1

            goal_pose.pose.orientation = target_pose.pose.orientation

            tempGoalPath.poses.append(goal_pose)

            rospy.loginfo("COORDINATES ==>"+str(x1)+","+str(y1))


        return tempGoalPath



    def intercept_animal(self, range ,target_pose: PoseStamped):
         
        # target_pose.pose.position.x
        # target_pose.pose.position.y

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"



        diff_x = self.odom.pose.pose.position.x - target_pose.pose.position.x 
        diff_y = self.odom.pose.pose.position.y - target_pose.pose.position.y

        temp_arg = math.atan(diff_y/diff_x)

        #Adjust for quadrants
        if (diff_x > 0)  and (diff_y > 0):
            arg = temp_arg
        elif (diff_x < 0)  and (diff_y < 0): 
            arg = temp_arg + math.pi
        elif (diff_x < 0)  and (diff_y > 0): 
            arg = temp_arg + math.pi
        elif (diff_x > 0)  and (diff_y < 0): 
            arg = temp_arg


        x1 = (range * math.cos(arg) ) + target_pose.pose.position.x
        y1 = (range * math.sin(arg) ) + target_pose.pose.position.y
        z1 = target_pose.pose.position.z

        goal_pose.pose.position.x = x1
        goal_pose.pose.position.y = y1
        goal_pose.pose.position.z = z1

        goal_pose.pose.orientation = target_pose.pose.orientation

        return goal_pose


    def check_goal_reached(self, range):
        
        if self.last_goal_path.poses:
            delta_x = self.odom.pose.pose.position.x - self.last_goal_path.poses[-1].pose.position.x
            delta_y = self.odom.pose.pose.position.y - self.last_goal_path.poses[-1].pose.position.y
            (delta_x**2)+(delta_y**2)

            # check distance
            dist = math.sqrt( (delta_x**2)+(delta_y**2) )

            if dist <= range:
                self.goal_reached = 1
            else:
                self.goal_reached = 0
        else:
            self.goal_reached = 0


    def intercept_circular_path(self, path : Path):
        """
		Find an entry point into the circle, i.e.
		the closest pose to the ship, such that the ship can
		enter from there. Then reorder the path to make the entry point
		the first pose. Return the reordered path.
		"""
		
        origin = self.odom.pose.pose

        num_waypoints = len(path.poses)    
    
        min_distance_from_ship = math.inf
        entry_point_index = None

        for i in range(len(path.poses)):

            pose_stamped = path.poses[i]

            x_diff = abs(pose_stamped.pose.position.x - origin.position.x)
            y_diff = abs(pose_stamped.pose.position.y - origin.position.y)

            distance_from_ship = math.sqrt(x_diff**2 + y_diff**2)

            if distance_from_ship < min_distance_from_ship:
                min_distance_from_ship = distance_from_ship
                entry_point_index = i

        ordered_path = None
        ordered_path = Path()
        ordered_path.header.stamp = rospy.Time.now()
        ordered_path.header.frame_id = "map" 

    
        for k in range(entry_point_index, num_waypoints):
            ordered_path.poses.append(path.poses[k])

        for k in range(entry_point_index):
            ordered_path.poses.append(path.poses[k])

        return ordered_path



    def goal_relayer(self, freq=30):

        r = rospy.Rate(freq)
        tempPath = Path()
        i = 0
        animal_index = 0 
        num_circle_points = 0
        circling_state_finished = 0
        calculate_circle = 0


        while not rospy.is_shutdown():

            #print("animal pose count= ", self.animal_pose_count)
            self.check_goal_reached(1)

            if (animal_index < self.animal_pose_count) and (i == 1):
                if self.goal_reached:
                    print("GOAL REACHED")
                    self.goal_reached = 0

                    if circling_state_finished:
                        print("Goto next animal")
                        tempPath = None
                        tempPath = Path()
                        tempPath.header.stamp = rospy.Time.now()
                        tempPath.header.frame_id = "map" 
                        tempPath.poses.append(self.intercept_animal(12,self.animal_pose_list.poses[animal_index]))

                        self.goal_path = tempPath

                        circling_state_finished = 0
                        calculate_circle = 1


                    elif calculate_circle:
                        print("calculating circle path")
                        circlePath = Path()
                        unorderedCirclePath = Path()    

                        if self.animal_pose_list.poses[animal_index].header.frame_id == "platypus":
                            unorderedCirclePath = self.generate_circular_path_around_pose(self.animal_pose_list.poses[animal_index], 7, True)
                        else:
                            unorderedCirclePath = self.generate_circular_path_around_pose(self.animal_pose_list.poses[animal_index], 7, False)

                        circlePath = self.intercept_circular_path(unorderedCirclePath)

                        num_circle_points = len(circlePath.poses)
                        circle_path_index = 0
                        
                        tempPath = None
                        tempPath = Path()
                        tempPath.header.stamp = rospy.Time.now()
                        tempPath.header.frame_id = "map" 
                        tempPath.poses.append(circlePath.poses[circle_path_index])
                        self.goal_path = tempPath
                        circle_path_index = circle_path_index + 1

                        calculate_circle = 0
                        circling_state_finished = 0

                    else:

                        if circle_path_index < num_circle_points:
                            tempPath = None
                            tempPath = Path()
                            tempPath.header.stamp = rospy.Time.now()
                            tempPath.header.frame_id = "map" 

                            tempPos = None
                            tempPos = PoseStamped()
                            tempPos.header.stamp = rospy.Time.now()
                            tempPos.header.frame_id = "map"

                            x1 = circlePath.poses[circle_path_index].pose.position.x + self.delta.poses[animal_index].pose.position.x
                            y1 = circlePath.poses[circle_path_index].pose.position.y + self.delta.poses[animal_index].pose.position.y
                            z1 = circlePath.poses[circle_path_index].pose.position.z
            
			
                            tempPos.pose.position.x = x1
                            tempPos.pose.position.y = y1
                            tempPos.pose.position.z = z1

                            tempPos.pose.orientation = circlePath.poses[circle_path_index].pose.orientation

                            tempPath.poses.append(tempPos)


                            self.goal_path = tempPath
                            circle_path_index = circle_path_index + 1
                        
                        else:
                            circle_path_index = 0
                            circling_state_finished = 1
                            animal_index = animal_index + 1


            #if animal locations are recieved
            if self.animal_locations_recieved and self.animal_pose_list.poses:

                if i == 0:
                    tempPath = None
                    tempPath = Path()
                    tempPath.header.stamp = rospy.Time.now()
                    tempPath.header.frame_id = "map" 
                    tempPath.poses.append(self.intercept_animal(12,self.animal_pose_list.poses[0]))

                    self.goal_path = tempPath
                    #self.animal_locations_recieved = 0
                    i = 1

                    calculate_circle = 1
                


            if self.goal_path is None:
                r.sleep()
                continue
            
            # print("publish check", self.goal_path)
            self.desired_path_pub.publish(self.goal_path)
            self.last_goal_path = self.goal_path
            self.goal_path = None
            r.sleep()







    pass	
		
	

# def callback(msg):
# 	animals = msg.poses
# 	for animal in animals:
# 		name = animal.header.frame_id
		
# 		path = generate_circular_path_around_pose(animal.pose, radius=5, clockwise=True, path_length=8)

# 		#path = reorder_circle_path_by_entry_point(path, origin)
		
# 		rospy.loginfo(path)

# def main():
# 	rospy.init_node("animal_monitor")
# 	rospy.Subscriber("/vrx/wildlife/animals/poses", GeoPath, callback)
# 	rospy.spin()

if __name__ == "__main__":

	an = AnimalWayfinder()

	an.goal_relayer()
    


