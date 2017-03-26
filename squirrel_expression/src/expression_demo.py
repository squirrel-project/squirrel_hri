#!/usr/bin/python

# If low arousal for some time, then HERE_HERE
# If arousal then CHEERING
#
# GUI to control stuff on the SQUIRREL robot
# author: Michael Zillich
# date: March 2017

import sys
import rospy
import std_msgs
import squirrel_vad_msgs
from squirrel_vad_msgs.msg import RecognisedResult
from squirrel_hri_msgs.msg import Expression

AROUSAL_THR = 0.5

class TestBehaviour(object):
    def __init__(self):
        rospy.Subscriber("/arousal", RecognisedResult, self.arousalCallback)
        self.expression_pub = rospy.Publisher('/expression', std_msgs.msg.String, queue_size=1)
        self.have_arousal = False
        rospy.sleep(2)
        self.expression_pub.publish(std_msgs.msg.String(Expression.NEUTRAL))
        #self.arousal_time = 0.0

    def arousalCallback(self, msg):
        if msg.label > AROUSAL_THR:
            self.expression_pub.publish(std_msgs.msg.String(Expression.CHEERING))
            self.have_arousal = True
            #self.arousal_time = rospy.get_time()

    def idleLoop(self):
        while not rospy.is_shutdown():
            if not self.have_arousal:
                self.expression_pub.publish(std_msgs.msg.String(Expression.HERE_HERE))
            self.have_arousal = False
            rospy.sleep(2)
            self.expression_pub.publish(std_msgs.msg.String(Expression.NEUTRAL))
            rospy.sleep(10)

if __name__ == '__main__':
    rospy.init_node('expression_demo', anonymous=True)
    rospy.loginfo("starting expression_demo")
    b = TestBehaviour()
    b.idleLoop()
