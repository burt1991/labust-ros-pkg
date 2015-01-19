#!/usr/bin/env python

import rospy, subprocess
from std_msgs.msg import Bool
from diagnostic_msgs.msg import *

#tmp
import time

class RosbagMonitor():
     
    def __init__(self):
        self.setup()
        pass
    
    def setup(self):
        
        #self.actions = ["",""]
        self.rosbagActions = {'start': 'rosbag record -a', 'split': ''}
        
        rospy.init_node('rosbag_monitor', anonymous=True)
        
        #topic_name = rospy.get_param('~topic_name')
        
        #self.pubRosbagStatus = rospy.Publisher("externalEvent", ExternalEvent, queue_size=20)
        
        #self.subRosbagSplit = rospy.Subscriber("rosbag_split", msgType, self.callback)
        
        #proc = subprocess.Popen(['rosbag', 'record', '-a'])
        
        proc = subprocess.Popen(self.rosbagActions['start'].split(" "))
        
        #time.sleep(1.0)
        time.sleep(10)
        print("test")
        
        proc.terminate()
        
    
        rospy.spin()
        pass
    
    def rosbagSplit(self, data):
        
        if data:
            pass
        pass


if __name__ == '__main__':
    try:
        RBM = RosbagMonitor()
    except rospy.ROSInterruptException: pass


