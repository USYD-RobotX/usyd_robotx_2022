#! /usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String,Duration
from vrx_gazebo.msg import Task



class ResetTopic():
    def __init__(self):
        rospy.Subscriber("/vrx/task/info",Task,self.taskCallback)
        self.toggle = rospy.Publisher("/wamv/reset_objects",String,queue_size=10)
        self.task_info = None
        x = Duration()
        
        while(self.task_info is None):
            continue
        while(self.task_info.state != "running"):
            continue
        
        
    def taskCallback(self,data):
        self.task_info = data    
    
    def togglePublisher(self):
        self.toggle.publish("reset")

if __name__=='__main__':
    rospy.init_node('reset_publisher')
    print("Initialised Reset Publisher")
    rt = ResetTopic()
    rt.togglePublisher()
    while not rospy.is_shutdown():
        elapsed_time = int(rt.task_info.elapsed_time.to_sec())
        if(elapsed_time%5 == 0):
            rt.togglePublisher()
        rospy.sleep(rospy.Duration(secs=1))
    pass   
    
    