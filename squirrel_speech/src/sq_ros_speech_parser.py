#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import rospkg
import std_msgs.msg
from read_voc import read_voc

from squirrel_speech_msgs.msg import RecognizedSpeech
from squirrel_speech_msgs.msg import RecognizedCommand

print("SQUIRREL SPEECH PARSER ----------------------------------------------------")
print("Deutsch") 
print("---------------------------------------------------------------------------") 


rospack = rospkg.RosPack()
path_to_pkg = rospack.get_path('squirrel_speech_rec')
#print(path_to_pkg)
path_to_voc = path_to_pkg+"/database/german/vocabulary_with_synonyms.txt"
#print(path_to_voc)


synonyms = read_voc(path_to_voc)

pub = rospy.Publisher('squirrel_speech_recognized_commands', RecognizedCommand, queue_size=5)  
msg = RecognizedCommand()

def callback(data):

    if(data.is_recognized):
        init_inp = data.recognized_speech
        inp = init_inp
        speaker_ID = data.speaker_ID

        command = ""

        print("------------------------------------------------")
        # Synonyms-------------------------------------------------------------------

        for trigger in synonyms:
            #print(trigger + "-----------------" )
            for syn in synonyms[trigger]:
                #print("\t" + syn)
                if (syn in inp) and (len(syn)>0):
                    inp = inp.replace(syn,trigger)
                    print("Replacing " + syn + " with " + trigger)
        
        # Commands-------------------------------------------------------------------

        if "hallo roboter" in inp:
            command = "hallo"

        if "ja" in inp:
            command = "yes"

        if "nein" in inp:
            command = "no"

        if "start" in inp:
            command = "start"

        if "programm beenden" in inp:
            command = "end_program"

        if "gehe" in inp:
            command = "go"

        if "vor" in inp:
            command = "forward"

        if "links" in inp:
            command = "left"

        if "rechts" in inp:
            command = "right"

        if "stop" in inp:
            command = "stop"

        if "komm" in inp:
            command = "come"

        if "YELLING" in inp:
            command = "stop"

        msg.recognized_speech = init_inp
        msg.parsed_speech = inp
        msg.int_command = command
        msg.is_command = (command != "")
        msg.speaker_ID = speaker_ID
        
        he = std_msgs.msg.Header()
        he.stamp = rospy.Time.now()
        msg.header = he

        
        if(msg.is_command):
            print("\033[0;32m", end="")  #green
        else:
            print("\033[0;31m", end="")  #rot
        rospy.loginfo(msg)
        print("\033[0;39m", end="")    #default
        print("")
        pub.publish(msg)




def listener():
    rospy.init_node('squirrel_speech_parser', anonymous=True)
    rospy.Subscriber("squirrel_speech_recognized_speech", RecognizedSpeech, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
