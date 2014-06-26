#!/usr/bin/env python

import os, sys, subprocess, rospy
from PySide import QtCore, QtGui, QtUiTools

from std_msgs.msg import Bool
from std_msgs.msg import String
from auv_msgs.msg import NavSts
from misc_msgs.msg import StartParser 
from misc_msgs.msg import StartNeptusParser 



class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent) 

        self.loadUiWidget(os.path.join(
                        os.path.dirname(os.path.realpath(__file__)), "parsegui.ui"))
        
        self.missionTab = MissionTab(self)
        self.neptusTab = NeptusTab(self)

        self.initROS()
        

    def loadUiWidget(self,uifilename, parent=None):
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(uifilename)
        uifile.open(QtCore.QFile.ReadOnly)
        self.ui = loader.load(uifile, parent)
        uifile.close()
        return 

    def initROS(self):
        # Publishers
        self.pubStartNeptusParse = rospy.Publisher('/startNeptusParse', StartNeptusParser)
        self.pubStartParse = rospy.Publisher('/startParse', StartParser)
        self.pubEventString = rospy.Publisher('/eventString', String)

        # Subscribers
        rospy.Subscriber("stateHat", NavSts, self.onStateHatCallback)

        # Init node
        rospy.init_node('parseGUI')
        
        # Get parameters
        self.labustMissionPath = rospy.get_param('~labust_mission_path', '')
    
    def onStateHatCallback(self, msg):
        
        self.neptusTab.onStateHatCallback(msg)
        self.missionTab.onStateHatCallback(msg)

        
######################################################################
        
class NeptusTab():
    def __init__(self, parent):
        
        self.parent = parent
        self.ui = parent.ui
        self.ui.browseButton.clicked.connect(self.browseFiles)
        self.ui.startButton.clicked.connect(self.startParse)
        self.ui.stopButton.clicked.connect(self.stopMission)
 #       self.ui.pauseButton.clicked.connect(self.pauseMission)
 
        self.firstPass = True 
      
         
    def browseFiles(self):
        #fileName = QtGui.QFileDialog.getOpenFileName(self,
        #str("Open Image"), str("/home/filip"), str("Image Files (*.png *.jpg *.bmp)"))
        filename = QtGui.QFileDialog.getOpenFileName(self.parent,
        str("Open Neptus mission file"), str("/home/"), str(""))
        print "Opened file: "+filename[0]
        self.filename = filename[0]
        self.ui.fileName.setText(str(filename[0]))
        bashCmd = str(self.parent.labustMissionPath+"scripts/unzipMission.bash "+filename[0])
        subprocess.call(bashCmd,shell=True)

    def startParse(self):
        # Dodaj provjeru je li uspjelo unzipanje
            
        missionData = StartNeptusParser()
        missionData.fileName = self.parent.labustMissionPath+"data/extracted/mission.nmis"
        
        missionData.origin.latitude = self.originLat
        missionData.origin.longitude = self.originLon
        
        if self.ui.radioButtonRelative.isChecked():
            missionData.relative = True
            missionData.customStartFlag = False
            missionData.customStart.latitude = self.Xpos
            missionData.customStart.longitude = self.Ypos
        elif self.ui.radioButtonAbsolute.isChecked():
            missionData.relative = False
            missionData.customStartFlag = False
        elif self.ui.radioButtonLatLon.isChecked():
            missionData.relative = False
            missionData.customStartFlag = True   
            missionData.customStart.latitude = self.startLat
            missionData.customStart.longitude = self.startLon
            #missionData.lat = self.lat
            #missionData.lon = self.lon
        else:
            missionData.relative = False
            missionData.customStartFlag = True           
            missionData.customStart.latitude = self.startLat
            missionData.customstart.longitude = self.startLon
    
        self.parent.pubStartNeptusParse.publish(missionData)
        
    def stopMission(self):
        data = String()
        data = "/STOP";
        self.parent.pubEventString.publish(data)
        
    def onStateHatCallback(self, msg):
        
        if self.firstPass:
            if msg.origin.latitude != 0 and msg.origin.longitude != 0:
                self.firstPass = False
            else:
                print "Waiting for origin"
            self.startLat = msg.origin.latitude
            self.startLon = msg.origin.longitude
            self.originLat = msg.origin.latitude
            self.originLon = msg.origin.longitude
            
            self.ui.lineEditLat.setText(str(self.startLat))
            self.ui.lineEditLon.setText(str(self.startLon))
        
        self.Xpos = msg.position.north
        self.Ypos = msg.position.east
        
        #self.lat = msg.global_position.latitude
        #self.lon = msg.global_position.longitude
        
######################################################################
        
class MissionTab():
    def __init__(self, parent):
        
        self.parent = parent
        self.ui = parent.ui
        self.ui.browseButtonMission.clicked.connect(self.browseFiles)
        self.ui.startButtonMission.clicked.connect(self.startParse)
        self.ui.stopButtonMission.clicked.connect(self.stopMission)
 #       self.ui.pauseButtonMission.clicked.connect(self.pauseMission)
        
    def browseFiles(self):

        filename = QtGui.QFileDialog.getOpenFileName(self.parent,
        str("Select XML mission file"), str("/home/"), str(""))
        print "Opened file: "+filename[0]
        self.filename = filename[0]
        self.ui.fileNameMission.setText(str(filename[0]))

    def startParse(self):
             
        missionData = StartParser()
        missionData.fileName = self.filename 
        
        if self.ui.radioButtonMissionRelative.isChecked():
            missionData.relative = True
            missionData.startPosition.north = self.Xpos
            missionData.startPosition.east = self.Ypos
        elif self.ui.radioButtonMissionAbsolute.isChecked():
            missionData.relative = False
        else:
            missionData.relative = True           
            #missionData.lat = self.startLat
            #missionData.lon = self.startLon
             
        self.parent.pubStartParse.publish(missionData)
        
        data = String()
        data = "/START_DISPATCHER"
        self.parent.pubEventString.publish(data)
        
    def stopMission(self):
        data = String()
        data = "/STOP";
        self.parent.pubEventString.publish(data)
         
#    def pauseMission(self):
#         data = String()
#         data = "/STOP";
#         self.pubStopMission.publish(data)

    def onStateHatCallback(self, msg):      
        self.Xpos = msg.position.north
        self.Ypos = msg.position.east
        
######################################################################
        
class PrimitiveTab():
    def __init__(self, parent):
        print "bla"

######################################################################
        
if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = ControlMainWindow()
    MainWindow.ui.show()
    sys.exit(app.exec_())
