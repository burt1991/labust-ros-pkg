#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function
import os, sys

import rospy
import labust_gui

from python_qt_binding import loadUi, QtCore, QtGui
from qt_gui.plugin import Plugin

from sensor_msgs.msg import Joy
from auv_msgs.msg import NavSts
           
class JoyGui(QtGui.QWidget):
   
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)

    def setup(self, name, ros):
        self.setObjectName(name)
        self.setWindowTitle(name)
        self._setup();
        self._connect_signals(ros)   
    
    def unload(self):
        pass
    
    def _connect_signals(self, ros):
        self.surgeSlider.valueChanged.connect(ros.onSurgeChanged)
        self.swaySlider.valueChanged.connect(ros.onSwayChanged)
        self.torqueSlider.valueChanged.connect(ros.onTorqueChanged)
        self.stopButton.clicked.connect(self.onStop)
        
           
    @QtCore.pyqtSlot()   
    def onStop(self):
        self.surgeSlider.setValue(0)
        self.swaySlider.setValue(0)
        self.torqueSlider.setValue(0)
                                 
    def _setup(self):
        pass

class JoyGuiROS(QtCore.QObject):
            
        def __init__(self):
            QtCore.QObject.__init__(self)
            
            self.joyPub = rospy.Publisher("joy", Joy)
            self.joyOut = Joy()
            self.joyOut.axes = [0,0,0,0,0,0]
        
        def setup(self, gui):
            self._connect_signals(gui)
            self._subscribe()
            pass
        
        @QtCore.pyqtSlot(int)
        def onSurgeChanged(self, value):
            self.joyOut.axes[1] = value/100.0;
            
        @QtCore.pyqtSlot(int)
        def onSwayChanged(self, value):
            self.joyOut.axes[0] = -value/100.0;
            
        @QtCore.pyqtSlot(int)
        def onTorqueChanged(self, value):
            self.joyOut.axes[2] = -value/100.0;
            
        def publish(self):
            self.joyPub.publish(self.joyOut)            
        
        def unload(self):
            self.state.unregister();e
            self.joyPub.unregister();
            pass

        def _connect_signals(self, gui):
            pass
        
        def onNavSts(self,data):
            self.publish()
            
        def _subscribe(self):
            self.state = rospy.Subscriber("stateHat",NavSts,self.onNavSts)
            pass
                                            
        def _unsubscribe(self):
            pass


class JoyRqt(Plugin):
    def __init__(self, context):
        name = "Joy"
        resource = os.path.join(os.path.dirname(
                        os.path.realpath(__file__)), 
                        "resource/" + name + ".ui")
        GuiT = JoyGui
        RosT = JoyGuiROS
        
        super(JoyRqt, self).__init__(context)
        self.setObjectName(name)

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print("arguments: ", args)
            print("unknowns: ", unknowns)

        # Create QWidget
        self._gui = GuiT()
        self._ros = RosT()

        loadUi(resource, self._gui)
        self._ros.setup(self._gui)
        self._gui.setup(name + "Rqt", self._ros)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + 
                                        (" (%d)" % context.serial_number()))
        context.add_widget(self._gui) 

if __name__ == '__main__':
    from labust_rqt import rqt_plugin_meta
    name = "Joy"
    resource = rqt_plugin_meta.resource_rpath(name, __file__)
    rqt_plugin_meta.launch_standalone(name = name,
                    GuiT = JoyGui,
                    RosT = JoyROS,
                    resource = resource);     