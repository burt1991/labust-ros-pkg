#!/usr/bin/env python
import rospy

from misc_msgs.msg import ExternalEvent

from std_msgs.msg import *
from auv_msgs.msg import *


class TopicTransform():
    
    def __init__(self):
        self.setup()

    def callback(self,data):
   
        tmpList = self.msg_field.split("/")
        
        parentData = data
        print tmpList
        for field in tmpList:
            print field
            parentData = getattr(parentData,field)
        
        
        eventData = ExternalEvent()
        eventData.id = self.eventID
        eventData.value = parentData
        
        #print eventData
        self.pubExternalEvent.publish(eventData)
        
    def setup(self):
        
        # in ROS, nodes are unique named. If two nodes with the same
        # node are launched, the previous one is kicked off. The 
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaenously.
        rospy.init_node('topic_transform', anonymous=True)
        
        topic_name = rospy.get_param('~topic_name')
        self.eventID = rospy.get_param('~event')
       
        tmp = ""
        tmp = rospy.get_param('~msg_type')  
        tmpList = tmp.split("/")
        msg_package = tmpList[0]
        #print msg_package
        #rospy.logerr("I heard %s",msg_package)
        msg_type = tmpList[1]
        #rospy.logerr("I heard %s",msg_type)
        
        self.msg_field = ""
        self.msg_field = rospy.get_param('~msg_field')
        
               
        if msg_package == "std_msgs":
            msgType = getattr(std_msgs.msg, msg_type)
            
        elif msg_package == "auv_msgs":
            msgType = getattr(auv_msgs.msg, msg_type)
            
            
        self.pubExternalEvent = rospy.Publisher("externalEvent", ExternalEvent, queue_size=20)
        
        rospy.Subscriber(topic_name, msgType, self.callback)
    
    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
if __name__ == '__main__':
    try:
        TT = TopicTransform()
    except rospy.ROSInterruptException: pass
    
    
  
