#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from std_msgs.msg import String


print("SQUIRREL SPEECH TEST INPUT ------------------------------------------------")
print("Deutsch") 
print("---------------------------------------------------------------------------") 


def tester():
    pub = rospy.Publisher('topic_rec_speech', String, queue_size=5)  
    rospy.init_node('speech_recognizer_node', anonymous=True)


    while not rospy.is_shutdown():

        try:
            value_str = raw_input('Enter msg: ')
            #value_str = "halllo"

            #print(type(value_str))
            value = unicode(value_str,"utf-8")    #Necessary for Python 2.7x
            #print(type(value))
            #value = value.encode("utf-8")

            value = value.lower()

            
            print("\033[0;32m", end="")  #green
            rospy.loginfo(value)
            print("\033[0;37m", end="")    #white

            pub.publish(value)

        except (KeyboardInterrupt,SystemExit):
            break


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass



