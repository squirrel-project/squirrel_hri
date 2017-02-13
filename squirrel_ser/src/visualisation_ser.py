#!/usr/bin/env python
import argparse
import sys
import rospy
import numpy as np
import copy
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from squirrel_vad_msgs.msg import RecognisedResult 
from threading import Lock
import rospkg
#import yaml
import pdb

class SoundViz(object):
	def __init__(self, g_min = -1.1, g_max = 1.1):
		self.lock = Lock()
		self.lock.acquire()
		self.marker_pub = rospy.Publisher("ser_feature_markers", MarkerArray, queue_size=1)
		self.ang1 = 0
		self.ang2 = 0
		self.speech = 0
		self.arousal = 0
		self.valence = 0
		self.minerg = g_min
		self.maxerg = g_max

		self.vad_sub = rospy.Subscriber("vad", Int32, self.vad_cb, queue_size=1)
		self.arousal_sub = rospy.Subscriber("arousal", RecognisedResult, self.arousal_cb, queue_size=1)
		self.valence_sub = rospy.Subscriber("valence", RecognisedResult, self.valence_cb, queue_size=1)
		
		self.pub_tmr = rospy.Timer(rospy.Duration(0.05), self.tmr_cb, oneshot=False)
		self.lock.release()

	def vad_cb(self, msg):
		self.speech = msg

	def arousal_cb(self, msg):
		self.arousal = msg.label
		self.arousal = (self.arousal - self.minerg)/float(self.maxerg - self.minerg)

	def valence_cb(self, msg):
		self.valence = msg.label
		self.valence = (self.valence - self.minerg)/float(self.maxerg - self.minerg)	
	
	def tmr_cb(self, ev):
		self.lock.acquire()
		
		ma1 = Marker()
		ma1.header.frame_id = "sound"
		ma1.header.stamp = rospy.Time.now()
		ma1.ns = "my_namespace"
		ma1.id = 0
		ma1.type = Marker.SPHERE
		ma1.action = Marker.ADD
		ma1.pose.position.x = 0
		ma1.pose.position.y = 0
		ma1.pose.position.z = 0
		if self.speech:
			ma1.scale.x = 1*(1 + self.arousal * 2)
			ma1.scale.y = 1*(1 + self.arousal * 2)
			ma1.scale.z = 1*(1 + self.arousal * 2)
		else:
			ma1.scale.x = 0.5*(1 + self.arousal)
			ma1.scale.y = 0.5*(1 + self.arousal)
			ma1.scale.z = 0.5*(1 + self.arousal)

		ma1.color.a = 1.0
		ma1.color.r = 1.0 if self.valence < 0.0 else 0.0
		ma1.color.g = 1.0 if self.valence > 0.0 else 0.0
		ma1.color.b = 0.5

		ma1.lifetime = rospy.Duration(0.1)
		q1 = quaternion_from_euler(0, 0, self.ang1)
		ma1.pose.orientation = Quaternion(x = q1[0],
											  y = q1[1],
											  z = q1[2],
											  w = q1[3])

		ma2 = copy.deepcopy(ma1)
		ma2.id = 1
		q2 = quaternion_from_euler(0, 0, self.ang2)
		ma2.pose.orientation = Quaternion(x = q2[0],
											  y = q2[1],
											  z = q2[2],
											  w = q2[3])

		ma_array = MarkerArray()
		ma_array.markers.append(ma1)
			#ma_array.markers.extend([ma1, ma2])
		self.marker_pub.publish(ma_array)
		self.ang2 = 0
		self.ang1 = 0
		self.arousal = 0
		self.speech = 0
		self.lock.release()

if __name__ == '__main__':

	sys.argv[len(sys.argv) - 1] = '--name'
	sys.argv[len(sys.argv) - 2] = '--default'

	parser = argparse.ArgumentParser()

	#options for VAD
	#automatic gain normalisation
	parser.add_argument("-g_min", "--gain_min", dest= 'g_min', type=float, help="min value of automatic gain normalisation", default=-1.1)
	parser.add_argument("-g_max", "--gain_max", dest= 'g_max', type=float, help="max value of automatic gain normalisation", default=1.1)

	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")

	#parser.add_argument("-h", "--help", help="help", action="store_true")

	args = parser.parse_args()

	rospy.init_node('ser_features_visualizer', anonymous=True)
	sv = SoundViz(args.g_min, args.g_max)
	rospy.spin()