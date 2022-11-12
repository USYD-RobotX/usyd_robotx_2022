#!/usr/bin/env python3
from defer import return_value
import rospy
from nav_msgs.msg import OccupancyGrid 

import numpy as np

from matplotlib import pyplot as plt
from matplotlib import animation

import time

from skimage.segmentation import flood, flood_fill
from skimage.measure import label


class ObjectIdentifier():

    def __init__(self) -> None:
        rospy.init_node("object_identifier")
        self.data = np.ones((1028,1028)) * -1

        print("SHOWING")

        rospy.Subscriber("/wamv/map", OccupancyGrid, self.map_callback)

        # self.im = plt.imshow(self.data)
        # plt.ion()
        # plt.show(block=False)

        fig = plt.figure()
        self.ax = fig.add_subplot(111)
        plt.ion()

        self.im = self.ax.imshow(self.data, cmap='gray')
        fig.show()
        fig.canvas.draw()

        # for i in range(0,100):
        self.ax.clear()
        self.ax.imshow(self.data, cmap='gray')
        fig.canvas.draw()


        # self.im.show()
        # self.im.canvas.draw()

    

        # while True:
        #     self.im.canvas.draw()
        #     # plt.show(block=False)
        #     time.sleep(1)

        
        # time.sleep(5)
        # print("CONTINUED")


        
        # plt.show()

    def map_callback(self, map):
        # print(map.header)
        size = (1028, 1028)
        data = np.array(map.data)

        data = np.reshape(data, size)

        thresh = 50

        data[data <thresh] = 0
        data[data >=thresh] = 1

        # self.data 
        print("GOT MAP")

        self.analyse_map(data)
        # time.sleep(0.1)

    def analyse_map(self, data):
        # first=(x_coords[4], y_coords[4])
        # print(first)

        

        self.data, numbers = label(data, return_num=True)

        print(data)

        pass
    
    def update_plot(self):
        self.ax.clear()
        self.ax.imshow(self.data)
        print("UPDATING PLOT")
        # self.im.set_data(self.data)
        # plt.draw()
        # plt.pause(0.001) 
        pass
    pass



if __name__ == "__main__":

    oi = ObjectIdentifier()

   

    # fig = plt.figure()

    
    

    while not rospy.is_shutdown():
        # rospy.spin()
        plt.pause(.001)
        # time.sleep(0.1)

        oi.update_plot()


        # plt.show()
    pass