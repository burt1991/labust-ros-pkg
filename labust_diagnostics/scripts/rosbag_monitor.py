#!/usr/bin/env python
import rospy, subprocess
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from diagnostic_msgs.msg import *

class RosbagMonitor():
     
    def __init__(self):
        self.setup()
        pass
    
    def setup(self):
            
        rospy.init_node('rosbag_monitor', anonymous=True)
        save_directory = rospy.get_param('~save_directory')  
        self.rosbagActions = {'start': 'rosbag record -a -o '+str(save_directory)+'rosbag', 'split': ''}
        self.loggerActive = False;
        ''' Publishers and subscribers ''' 
        #self.pubRosbagStatus = rospy.Publisher("externalEvent", ExternalEvent, queue_size=20)
        self.subRosbagAction = rospy.Subscriber("rosbagAction", String, self.onRosbagActionCallback)  
        rospy.spin()
        pass
    
    def onRosbagActionCallback(self, msg):
        if msg.data=='stop':
            if self.loggerActive == True:
                rospy.logerr("Stopping Rosbag logger...")
                self.proc.terminate()
                self.proc.wait()
                self.loggerActive = False
            else:
                rospy.logerr("No Active log...")
        elif msg.data=='start':
            if self.loggerActive == True:
                self.proc.terminate()
                self.proc.wait()
            rospy.logerr("Starting Rosbag logger...")
            self.proc = subprocess.Popen(self.rosbagActions['start'].split(" "))
            self.loggerActive = True

if __name__ == '__main__':
    try:
        RBM = RosbagMonitor()
    except rospy.ROSInterruptException: pass


