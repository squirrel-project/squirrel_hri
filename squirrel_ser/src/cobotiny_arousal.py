#!/usr/bin/env python
import argparse
import sys
import rospy
import numpy as np
import copy
from std_msgs.msg import Int32,Float32,String
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from squirrel_vad_msgs.msg import RecognisedResult 
from threading import Lock
import rospkg
#import yaml
import pdb

class SoundViz(object):
	def __init__(self, a_min, a_max, v_min, v_max, threshold = 0.1, frequency = 1.0, window_size = 5):
		self.lock = Lock()
		self.lock.acquire()
		self.marker_pub = rospy.Publisher("/cobotnity_arousal", Float32, queue_size=1)
		self.ang1 = 0
		self.ang2 = 0
		self.arousal = 0
		self.valence = 0
		self.duration = 0.0
		self.a_min = a_min
		self.a_max = a_max
		self.v_min = v_min
		self.v_max = v_max
		self.window_size = window_size
		self.window = []
		self.threshold = threshold
		self.vad_sub = rospy.Subscriber("speech_duration", Float32, self.vad_cb, queue_size=1)

		self.arousal_sub = rospy.Subscriber("arousal", RecognisedResult, self.arousal_cb, queue_size=1)
		self.valence_sub = rospy.Subscriber("valence", RecognisedResult, self.valence_cb, queue_size=1)
		
		self.pub_tmr = rospy.Timer(rospy.Duration(frequency), self.tmr_cb, oneshot=False)
		self.lock.release()
	
	def vad_cb(self, msg):
		self.duration = msg

	def arousal_cb(self, msg):
		self.arousal = msg.label
		self.arousal = (self.arousal - self.a_min)/float(self.a_max - self.a_min)

	def valence_cb(self, msg):
		self.valence = msg.label
		self.valence = (self.valence - self.v_min)/float(self.v_max - self.v_min)	
	
	def tmr_cb(self, ev):
		self.lock.acquire()
		
		if len(self.window) > self.window_size:
			del self.window[0]
		self.window.append(self.arousal)
		
		avg_arousal = 0.0
		for arousal in self.window:
			avg_arousal += arousal

		avg_arousal = avg_arousal / len(self.window)

		self.marker_pub.publish(avg_arousal)
		#rospy.loginfo("avg_arousal: " + str(avg_arousal))
		self.lock.release()

if __name__ == '__main__':

	sys.argv[len(sys.argv) - 1] = '--name'
	sys.argv[len(sys.argv) - 2] = '--default'

	parser = argparse.ArgumentParser()

	#options for VAD
	#automatic gain normalisation
	parser.add_argument("-a_min", "--a_min", dest= 'a_min', type=float, help="min value of arousal", default=-1.0)
	parser.add_argument("-a_max", "--a_max", dest= 'a_max', type=float, help="max value of arousal", default=1.0)
	
	parser.add_argument("-v_min", "--v_min", dest= 'v_min', type=float, help="min value of valence", default=-1.0)
	parser.add_argument("-v_max", "--v_max", dest= 'v_max', type=float, help="max value of valence", default=1.0)

	parser.add_argument("-th", "--threshold", dest= 'threshold', type=float, help="threshold", default=0.3)
	parser.add_argument("-fq", "--frequency", dest= 'frequency', type=float, help="frequency", default=1.0)
	parser.add_argument("-win", "--window", dest= 'window', type=int, help="window", default=5)
	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")

	#parser.add_argument("-h", "--help", help="help", action="store_true")

	args = parser.parse_args()
		
	rospy.init_node('cobotiny_arousal', anonymous=True)
	sv = SoundViz(args.a_min, args.a_max, args.v_min, args.v_max, args.threshold, args.frequency, args.window)
	rospy.spin()