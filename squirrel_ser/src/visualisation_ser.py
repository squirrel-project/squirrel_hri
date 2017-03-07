#!/usr/bin/env python
import argparse
import sys
import rospy
import numpy as np
import copy
from std_msgs.msg import Int32,Float32
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from squirrel_vad_msgs.msg import RecognisedResult 
from threading import Lock
import rospkg
#import yaml
import pdb

class SoundViz(object):
	def __init__(self, a_min, a_max, v_min, v_max, threshold = 0.1):
		self.lock = Lock()
		self.lock.acquire()
		self.marker_pub = rospy.Publisher("ser_feature_markers", MarkerArray, queue_size=1)
		self.ang1 = 0
		self.ang2 = 0
		self.arousal = 0
		self.valence = 0
		self.duration = 0.0
		self.a_min = a_min
		self.a_max = a_max
		
		self.v_min = v_min
		self.v_max = v_max

		self.threshold = threshold
		self.vad_sub = rospy.Subscriber("speech_duration", Float32, self.vad_cb, queue_size=1)

		self.arousal_sub = rospy.Subscriber("arousal", RecognisedResult, self.arousal_cb, queue_size=1)
		self.valence_sub = rospy.Subscriber("valence", RecognisedResult, self.valence_cb, queue_size=1)
		
		self.pub_tmr = rospy.Timer(rospy.Duration(0.05), self.tmr_cb, oneshot=False)
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
		
		rospy.loginfo(self.duration)
		ma1.scale.x = 1*(1 + self.arousal * 2)
		ma1.scale.y = 1*(1 + self.arousal * 2)
		ma1.scale.z = 1*(1 + self.arousal * 2)
		
		ma1.color.a = 1.0
		ma1.color.r = 1.0 if self.valence < self.threshold else 0.0
		ma1.color.g = 1.0 if self.valence > self.threshold else 0.0
		ma1.color.b = 1.0 if self.valence == 0.0 else 0.0

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
	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")

	#parser.add_argument("-h", "--help", help="help", action="store_true")

	args = parser.parse_args()

	rospy.init_node('ser_features_visualizer', anonymous=True)
	sv = SoundViz(args.a_min, args.a_max, args.v_min, args.v_max, args.threshold)
	rospy.spin()