#!/usr/bin/env python3

import time

from pynput import keyboard

import rospy

from std_msgs.msg import Float32


PRINT_STRING = """ 
                                  ^
   'x'<- ->'s'                'q' |
        X===========================)
             |-------------|   X  |
             |              | 'w' v
             |               |
             |              | 'e'  ^
             |-------------|   X  |
        X===========================)
   'c'<- ->'d'                'r' |
                                  v
 
"""

class KeyboardControl:

    front_cmd_scale = 3.0
    rear_cmd_scale = 5.0

    right_front_cmd = 0
    right_rear_cmd = 0

    left_front_cmd = 0
    left_rear_cmd = 0

    right_front_keys = ("r", "e")
    left_front_keys = ("q", "w")
    right_rear_keys = ("d","c")
    left_rear_keys = ("s", "x")

    
    def __init__(self):
        
        print("Welcome to Keyboard WAMV Control")

        print(PRINT_STRING)

        rospy.init_node("wamv_keyboard_control")

        right_front_topic = "/wamv/thrusters/right_front_thrust_cmd"
        right_rear_topic = "/wamv/thrusters/right_rear_thrust_cmd"
        left_front_topic = "/wamv/thrusters/left_front_thrust_cmd"
        left_rear_topic = "/wamv/thrusters/left_rear_thrust_cmd"

        self.right_front_pub = rospy.Publisher(right_front_topic, Float32, queue_size = 10)
        self.right_rear_pub = rospy.Publisher(right_rear_topic, Float32, queue_size = 10)
        self.left_front_pub = rospy.Publisher(left_front_topic, Float32, queue_size = 10)
        self.left_rear_pub = rospy.Publisher(left_rear_topic, Float32, queue_size = 10)

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        
        listener.start()


    def run(self):
        while not rospy.is_shutdown():
            
            self.right_front_pub.publish(self.right_front_cmd)
            self.left_front_pub.publish(self.left_front_cmd)
            self.right_rear_pub.publish(self.right_rear_cmd)
            self.left_rear_pub.publish(self.left_rear_cmd)

            time.sleep(1/20)

    def on_press(self, key):
        try:
            # print('alphanumeric key {0} pressed'.format(
            #     key.char))
            pressed_key = key.char
            if pressed_key == self.right_front_keys[0]:
                self.right_front_cmd = self.front_cmd_scale * 1.0
            elif pressed_key == self.right_front_keys[1]:
                self.right_front_cmd = self.front_cmd_scale * -1.0


            elif pressed_key == self.left_front_keys[0]:
                self.left_front_cmd = self.front_cmd_scale * 1.0
            elif pressed_key == self.left_front_keys[1]:
                self.left_front_cmd = self.front_cmd_scale * -1.0


            elif pressed_key == self.right_rear_keys[0]:
                self.right_rear_cmd = self.rear_cmd_scale * 1.0
            elif pressed_key == self.right_rear_keys[1]:
                self.right_rear_cmd = self.rear_cmd_scale * -1.0


            elif pressed_key == self.left_rear_keys[0]:
                self.left_rear_cmd = self.rear_cmd_scale * 1.0
            elif pressed_key == self.left_rear_keys[1]:
                self.left_rear_cmd = self.rear_cmd_scale * -1.0

        except AttributeError:
            pass
            # print('special key {0} pressed'.format(
            #     key))

    def on_release(self, key):
        # print('{0} released'.format(
        #     key))
        try:
            unpressed_key = key.char

            if unpressed_key in self.right_front_keys:
                self.right_front_cmd = 0

            elif unpressed_key in self.left_front_keys:
                self.left_front_cmd = 0

            elif unpressed_key in self.right_rear_keys:
                self.right_rear_cmd = 0

            elif unpressed_key in self.left_rear_keys:
                self.left_rear_cmd = 0

        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            return False



if __name__ == "__main__":
    k_c = KeyboardControl()

    k_c.run()
    
