#!/usr/bin/python
 
import sys
from PyQt4 import QtCore, QtGui, uic
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

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

        rospy.init_node('expression_gui')
        self.expr_pub = rospy.Publisher('/expression', String, queue_size=10)
        self.tilt_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
        self.wiggle_pub = rospy.Publisher('/motion_expression', String, queue_size=10)

    def helloButton_clicked(self):
        print "HELLO"
        msg = String()
        msg.data = "HELLO"
        self.expr_pub.publish(msg)

    def hereButton_clicked(self):
        print "HERE_HERE"
        msg = String()
        msg.data = "HERE_HERE"
        self.expr_pub.publish(msg)

    def ohnoButton_clicked(self):
        print "OH_NO"
        msg = String()
        msg.data = "OH_NO"
        self.expr_pub.publish(msg)

    def yeahButton_clicked(self):
        print "YEAH"
        msg = String()
        msg.data = "YEAH"
        self.expr_pub.publish(msg)

    def whatButton_clicked(self):
        print "WHAT"
        msg = String()
        msg.data = "WHAT"
        self.expr_pub.publish(msg)

    def upButton_clicked(self):
        print "camera up"
        msg = Float64()
        msg.data = -0.2
        self.tilt_pub.publish(msg)

    def straightButton_clicked(self):
        print "camera straight"
        msg = Float64()
        msg.data = 0.0
        self.tilt_pub.publish(msg)

    def downButton_clicked(self):
        print "camera down"
        msg = Float64()
        msg.data = 0.8
        self.tilt_pub.publish(msg)

    def wiggleNoButton_clicked(self):
        print "wiggle no"
        msg = String()
        msg.data = "no"
        self.wiggle_pub.publish(msg)

    def wiggleErrorButton_clicked(self):
        print "wiggle error"
        msg = String()
        msg.data = "error"
        self.wiggle_pub.publish(msg)

app = QtGui.QApplication(sys.argv)
myWindow = MyWindowClass(None)
myWindow.show()
app.exec_()

