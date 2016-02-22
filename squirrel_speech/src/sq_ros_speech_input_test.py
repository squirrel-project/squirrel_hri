#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import std_msgs.msg

from squirrel_speech_msgs.msg import RecognizedSpeech

default_lang = "de"


print("SQUIRREL SPEECH TEST INPUT ------------------------------------------------")



def tester():

    pub = rospy.Publisher('squirrel_speech_recognized_speech', RecognizedSpeech, queue_size=5) 
    msg = RecognizedSpeech()

    rospy.init_node('squirrel_speech_test_input', anonymous=True)

    while not rospy.is_shutdown():

        try:
            value_str = raw_input('Enter msg: ')

            value = unicode(value_str,"utf-8")    #Necessary for Python 2.7x
            #value = value.encode("utf-8")

            value = value.lower()

            msg.recognized_speech = value
            msg.is_recognized = True
            msg.speaker_ID = 0
            
            he = std_msgs.msg.Header()
            he.stamp = rospy.Time.now()
            msg.header = he
            

            print("\033[0;32m", end="")  #green
            rospy.loginfo(msg)
            print("\033[0;39m", end="")    #default

            pub.publish(msg)

        except (KeyboardInterrupt,SystemExit):
            break


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass



