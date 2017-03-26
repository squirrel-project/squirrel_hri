#!/usr/bin/python
#
# GUI to control stuff on the SQUIRREL robot
# author: Michael Zillich
# date: March 2017

import sys
from PyQt4 import QtCore, QtGui, uic
import rospy
import std_msgs
#.msg import String
#from std_msgs.msg import Float64
#from std_msgs.msg import Bool

form_class = uic.loadUiType("expression_gui.ui")[0]
 
class MyWindowClass(QtGui.QMainWindow, form_class):
    expr_pub = None
    tilt_pub = None
    wiggle_pub = None

    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

        # sound out
        self.helloButton.clicked.connect(self.helloButton_clicked)
        self.hereButton.clicked.connect(self.hereButton_clicked)
        self.ohnoButton.clicked.connect(self.ohnoButton_clicked)
        self.yeahButton.clicked.connect(self.yeahButton_clicked)
        self.whatButton.clicked.connect(self.whatButton_clicked)
        # camera movement
        self.upButton.clicked.connect(self.upButton_clicked)
        self.straightButton.clicked.connect(self.straightButton_clicked)
        self.downButton.clicked.connect(self.downButton_clicked)
        # base wiggling
        self.wiggleNoButton.clicked.connect(self.wiggleNoButton_clicked)
        self.wiggleErrorButton.clicked.connect(self.wiggleErrorButton_clicked)
        # safety reset
        self.safetyResetButton.clicked.connect(self.safetyResetButton_clicked)

        rospy.init_node('expression_gui')
        self.expr_pub = rospy.Publisher('/expression', std_msgs.msg.String, queue_size=10)
        self.tilt_pub = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.wiggle_pub = rospy.Publisher('/motion_expression', std_msgs.msg.String, queue_size=10)
        self.reset_safety_pub = rospy.Publisher('/reset_safety', std_msgs.msg.Bool, queue_size=10)
        self.arm_mode_pub = rospy.Publisher('/real/robotino/settings/switch_mode', std_msgs.msg.Int32, queue_size=10, latch=True)

    def helloButton_clicked(self):
        print "HELLO"
        msg = std_msgs.msg.String()
        msg.data = "HELLO"
        self.expr_pub.publish(msg)

    def hereButton_clicked(self):
        print "HERE_HERE"
        msg = std_msgs.msg.String()
        msg.data = "HERE_HERE"
        self.expr_pub.publish(msg)

    def ohnoButton_clicked(self):
        print "OH_NO"
        msg = std_msgs.msg.String()
        msg.data = "OH_NO"
        self.expr_pub.publish(msg)

    def yeahButton_clicked(self):
        print "YEAH"
        msg = std_msgs.msg.String()
        msg.data = "YEAH"
        self.expr_pub.publish(msg)

    def whatButton_clicked(self):
        print "WHAT"
        msg = std_msgs.msg.String()
        msg.data = "WHAT"
        self.expr_pub.publish(msg)

    def upButton_clicked(self):
        print "camera up"
        msg = std_msgs.msg.Float64()
        msg.data = -0.2
        self.tilt_pub.publish(msg)

    def straightButton_clicked(self):
        print "camera straight"
        msg = std_msgs.msg.Float64()
        msg.data = 0.0
        self.tilt_pub.publish(msg)

    def downButton_clicked(self):
        print "camera down"
        msg = std_msgs.msg.Float64()
        msg.data = 0.8
        self.tilt_pub.publish(msg)

    def wiggleNoButton_clicked(self):
        print "wiggle no"
        msg = std_msgs.msg.String()
        msg.data = "no"
        self.wiggle_pub.publish(msg)

    def wiggleErrorButton_clicked(self):
        print "wiggle error"
        msg = std_msgs.msg.String()
        msg.data = "error"
        self.wiggle_pub.publish(msg)

    def safetyResetButton_clicked(self):
        print "reset safety"
        # reset the wrist safety node
        msg = std_msgs.msg.Bool
        msg.data = True
        self.reset_safety_pub.publish(msg)
        # reset the robot controller (works by switching the mode)
        # self.mode_pub.publish(data=0)

app = QtGui.QApplication(sys.argv)
myWindow = MyWindowClass(None)
myWindow.show()
app.exec_()

