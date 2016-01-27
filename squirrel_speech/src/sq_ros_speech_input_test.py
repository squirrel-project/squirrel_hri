#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from std_msgs.msg import String

import sys

default_lang = "de"

arg_num = len(sys.argv) #first argument is always script name and also being counted in number of arguments.
arg_list = str(sys.argv)

if arg_num > 1:
    arg_lang = str(sys.argv[1])
else:
    arg_lang = default_lang
 
if arg_num > 2:
    print("Too many Arguments!")


print("SQUIRREL SPEECH TEST INPUT ------------------------------------------------")
if arg_num == 1: 
	arg_lang == "de"

if arg_lang != "de" and arg_lang != "de-DE" and arg_lang != "de-AT":
    if arg_lang != "en" and arg_lang != "en-GB" and arg_lang != "en-US": 
        if arg_lang != "nl" and arg_lang != "nl-NL":
            print("Language not supported... (*might work anyway - see google speech language list)")
	
print("Used Language: " + arg_lang) 

print("---------------------------------------------------------------------------") 


def tester():
    pub = rospy.Publisher('topic_rec_speech', String, queue_size=5)  
    rospy.init_node('speech_recognizer_node', anonymous=True)


    while not rospy.is_shutdown():

        try:
            value_str = raw_input('Enter msg: ')


            value = unicode(value_str,"utf-8")    #Necessary for Python 2.7x
            #value = value.encode("utf-8")

            value = value.lower()

            print("\033[0;32m", end="")  #green
            rospy.loginfo(value)
            print("\033[0;39m", end="")    #default

            pub.publish(value)

        except (KeyboardInterrupt,SystemExit):
            break


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass



