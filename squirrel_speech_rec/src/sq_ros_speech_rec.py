#!/usr/bin/env python

from __future__ import print_function

import rospy
import speech_recognition as sr
import std_msgs.msg

import sys
import pyaudio

from squirrel_speech_msgs.msg import RecognizedSpeech
from squirrel_hri_msgs.msg import Expression


def recognizer():

    print("SQUIRREL SPEECH RECOGNITION -----------------------------------------------")

    #from subprocess import call
    #call(["pulseaudio", "--kill"])
    #call(["jack_control", "start"])

    pub = rospy.Publisher('squirrel_speech_recognized_speech', RecognizedSpeech, queue_size=5)
    msg = RecognizedSpeech()
    expression_pub = rospy.Publisher('/expression', std_msgs.msg.String, queue_size=5)

    rospy.init_node('squirrel_speech_recognizer', anonymous=False)

    arg_lang = rospy.get_param('~language', 'de')
    dev_id = rospy.get_param('~device_id', 1)
    speaker_id = rospy.get_param('~speaker_id', 1)
    sample_rate = rospy.get_param('~sample_rate', 16000)
    chunk_size = rospy.get_param('~chunk_size', 8192)

    if arg_lang != "de" and arg_lang != "de-DE":
        if arg_lang != "en" and arg_lang != "en-GB" and arg_lang != "en-US": 
            if arg_lang != "nl" and arg_lang != "nl-NL":
                print("Language not supported... (*might work anyway - see google speech language list)")

    print("Used Language: " + arg_lang) 
    print("Google API") 
    print("audio device ID: ", dev_id)
    print(" ") 
    print("---------------------------------------------------------------------------") 

    r = sr.Recognizer()
    m = sr.Microphone(int(dev_id), int(sample_rate), int (chunk_size))

    utterance_cnt = 1
    with m as source:
        r.adjust_for_ambient_noise(source)
        print("Set minimum energy threshold to {}".format(r.energy_threshold))

        print("---------------------------------------------------------------------------") 

        while not rospy.is_shutdown():
            # LOOP -----------------------------------------------------------------------
            try:
                print("Ready!")
                (audio, yelling) = r.listen(source)

                if yelling:
                    # say "oh no!"
                    expression_pub.publish(std_msgs.msg.String(Expression.OH_NO))

                    print("detected YELLING")
                    msg.recognized_speech = "YELLING"
                    msg.is_recognized = True
                    msg.speaker_ID = speaker_id
                    he = std_msgs.msg.Header()
                    he.stamp = rospy.Time.now()
                    msg.header = he

                    wav_filename = "sample" + str(utterance_cnt) + ".wav"
                    f = open(wav_filename, 'w')
                    f.write(audio.get_wav_data())
                    f.close()
                    text_filename = "sample" + str(utterance_cnt) + ".txt"
                    f = open(text_filename, 'w')
                    f.write("YELLING")
                    f.close()
                    utterance_cnt = utterance_cnt + 1

                else:
                    # say "hm?" "what?"
                    expression_pub.publish(std_msgs.msg.String(Expression.WHAT))

                    print("Recognition...")
 
                    value = r.recognize_google(audio, None, arg_lang)
                    value = value.encode("utf-8")
                    value = value.lower()
 
                    msg.recognized_speech = value
                    msg.is_recognized = True
                    msg.speaker_ID = speaker_id
                    he = std_msgs.msg.Header()
                    he.stamp = rospy.Time.now()
                    msg.header = he

                    wav_filename = "sample" + str(utterance_cnt) + ".wav"
                    f = open(wav_filename, 'w')
                    f.write(audio.get_wav_data())
                    f.close()
                    text_filename = "sample" + str(utterance_cnt) + ".txt"
                    f = open(text_filename, 'w')
                    f.write(value)
                    f.close()
                    utterance_cnt = utterance_cnt + 1

                print("\033[0;32m", end="")  #green
                rospy.loginfo(msg)
                print("\033[0;39m", end="")    #default

                pub.publish(msg)

            except sr.UnknownValueError:
                msg.recognized_speech = ""
                msg.is_recognized = False
                msg.speaker_ID = speaker_id
                he = std_msgs.msg.Header()
                he.stamp = rospy.Time.now()
                msg.header = he

                print("\033[0;31m", end="")  #red
                print("ERROR: Unrecognizable", end="")
                rospy.loginfo(msg)
                print("\033[0;39m")   #default

                pub.publish(msg)

            except sr.RequestError as e:
                print("\033[0;31m", end="")  #red
                print("ERROR: Couldn't request results from Google Speech")
                print("Recognition service - {0}".format(e))
                print("\033[0;39m")   #default


if __name__ == '__main__':
    try:
        recognizer()
    except rospy.ROSInterruptException:
        pass

