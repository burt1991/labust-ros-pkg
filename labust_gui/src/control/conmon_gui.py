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

from navcon_msgs.srv import RegisterController_v3
from navcon_msgs.srv import RegisterController_v3Response
from navcon_msgs.msg import ControllerState
from navcon_msgs.msg import ControllerInfo

from std_msgs.msg import String

class ConMonGui(QtGui.QWidget):
    
    onSendState = QtCore.pyqtSignal(object, int, name='onSendState')
    
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)
        
        self.parents = dict();

    def setup(self, name, ros):
        self.setObjectName(name)
        self.setWindowTitle(name)
        self._setup();
        self._connect_signals(ros)   
        
    @QtCore.pyqtSlot(object)
    def onNewController(self, req):
        if not self.parents.has_key(req.parent):
            self.parents[req.parent] = QtGui.QTreeWidgetItem()
            self.parents[req.parent].setText(0, req.parent)
            self.controllerTree.addTopLevelItem(self.parents[req.parent]) 
        
        item = QtGui.QTreeWidgetItem()
        item.setText(0, req.name)
        self.parents[req.parent].addChild(item) 
        
    @QtCore.pyqtSlot()
    def onSend(self):
        self.onSendState.emit(self.nameEdit.text(), self.stateCombo.currentIndex())
            
    def unload(self):
        pass
    
    def _connect_signals(self, ros):
        self.sendButton.clicked.connect(self.onSend)
        self.onSendState.connect(ros.onNewState)
        pass
                    
    def _setup(self):
        self.stateCombo.addItem("DISABLED")
        self.stateCombo.addItem("MANUAL")
        self.stateCombo.addItem("EXTERNAL")
        self.stateCombo.addItem("TRACKING")
        pass


class ConMonROS(QtCore.QObject):
        onNewController = QtCore.pyqtSignal(object, name = "onNewController")
        
        def __init__(self):
            QtCore.QObject.__init__(self)
             
        def setup(self, gui):
            self._connect_signals(gui)
            self._subscribe()
            pass
        
        def unload(self):
            pass

        def _connect_signals(self, gui):
            self.onNewController.connect(gui.onNewController)
            pass
        
        def onRegisterController(self, req):
            self.onNewController.emit(req)
            return RegisterController_v3Response()
        
        @QtCore.pyqtSlot(object, int)
        def onNewState(self, name, state):
            out = ControllerState()
            out.name = [name];
            info = ControllerInfo()
            info.state = state
            out.info = [info]
            self.stateOut.publish(out)
            
        def _subscribe(self):
            self.srv = rospy.Service('register_controller', 
                                     RegisterController_v3, 
                                     self.onRegisterController)
            self.stateOut = rospy.Publisher('controller_state',ControllerState)
                  
                                            
        def _unsubscribe(self):
            pass


class ConMonRqt(Plugin):
    def __init__(self, context):
        name = "ConMon"
        resource = os.path.join(os.path.dirname(
                        os.path.realpath(__file__)), 
                        "resource/" + name + ".ui")
        GuiT = ConMonGui
        RosT = ConMonROS
        
        super(ConMonRqt, self).__init__(context)
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
    name = "ConMon"
    resource = rqt_plugin_meta.resource_rpath(name, __file__)
    rqt_plugin_meta.launch_standalone(name = name,
                    GuiT = IdentificationGui,
                    RosT = IdentificationROS,
                    resource = resource);     