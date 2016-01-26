#!/usr/bin/env python
# -*- coding: utf-8 -*-

#magic line for Umlaut

import rospy
from std_msgs.msg import String


print("SQUIRREL SPEECH PARSER ----------------------------------------------------")
print("Deutsch") 
print("---------------------------------------------------------------------------") 

robot_syn =("roboter","kenny","robotino", "can i" , "kevin", "jenny")
gehe_syn  =("gehe","geht","geh")
stop_syn  =("stopp", "stop", "halt","anhalten", "halte an")
ja_syn =("ja", " ok ", "okay", "bestätige") 

start_syn =("start", "starte", "beginnen", "beginne", "beginn", "big in")



def callback(data):

    init_inp = data.data
    inp = init_inp

    command = ""


    # Synonyms-------------------------------------------------------------------
    
    for syn in robot_syn:
        #print(syn)
        if syn in inp:
            inp = inp.replace(syn,robot_syn[0])
            #print("Replacing " + syn + " with " + robot_syn[0])
            



    # Commands-------------------------------------------------------------------

    if "roboter ja" in inp:
        command = "Bestätigung"








    print("-------------------------------------------------------")
    print("CallerID: " + rospy.get_caller_id() )
    print("Received: " + init_inp )
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("- - - - - - - - - - - - - - - - - - - - - - - - - - - -")
    print("After replacing Synonyms:  " + inp)
    print("Issued Command: " + "\033[0;32m" + command + "\033[0;37m")
    print("-------------------------------------------------------\n\n")

def listener():
    rospy.init_node('speech_parser_node', anonymous=True)
    rospy.Subscriber("topic_rec_speech", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
