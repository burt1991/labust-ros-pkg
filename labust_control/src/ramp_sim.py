#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import BodyForceReq;
      
class MessageTransformer:
    def __init__(self):
        self.sub = rospy.Subscriber("tauOut", BodyForceReq, self.onTau)
        self.pub = rospy.Publisher("tauAch", BodyForceReq)
        
        self.t = [0,0,0,0,0,0,0]
        #25% per second
        self.maxRise = 25;
           
           
           
    def calculateThrust(self,X,Y,N):
        self.t[0] = 0.25*(X+Y+N);
        self.t[1] = 0.25*(X-Y-N);
        self.t[2] = 0.25*(X+Y-N);
        self.t[3] = 0.25*(X-Y+N);
           
           
    def onTau(data):
        tauAch = BodyForceReq()
        
        self.calculateThrust(data.wrench.force.x/4, data.wrench.force.y/4, data.wrench.torque.z/4)
                    

if __name__ == "__main__":
    rospy.init_node("ramp_sim");
    rospy.spin();
        
        