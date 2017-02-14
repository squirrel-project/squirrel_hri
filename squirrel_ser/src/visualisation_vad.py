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
from teresa_ut_vad_msgs.msg import vad_sync
from squirrel_vad_msgs.msg import RecognisedResult 
from threading import Lock
import rospkg
#import yaml
import pdb

class SoundViz(object):
	def __init__(self, g_min = 100, g_max = 5000):
		self.lock = Lock()
		self.lock.acquire()
		self.marker_pub = rospy.Publisher("vad_feature_markers", MarkerArray, queue_size=1)
		self.ang1 = 0
		self.ang2 = 0
		self.speech = 0
		self.erg = 0
		#rospack = rospkg.RosPack()
		#base_path = rospack.get_path("decision_making_uva")
		#with open(base_path + "/cfg/sound_normalisation_params.yaml") as f:
		#	yml = yaml.load(f)
		#	ergs = yml['sound_normalisation_params']
		self.minerg = g_min
		self.maxerg = g_max
		#self.dir_sub = rospy.Subscriber("sound_direction", Int32, self.dir_cb, queue_size=1)
		self.vad_sub = rospy.Subscriber("voice_detector_sync", vad_sync, self.vad_cb, queue_size=1)
		#self.arousal_sub = rospy.Subscriber("arousal", RecognisedResult, self.arousal_cb, queue_size=1)
		#self.valence_sub = rospy.Subscriber("valence", RecognisedResult, self.valence_cb, queue_size=1)

		self.pub_tmr = rospy.Timer(rospy.Duration(0.05), self.tmr_cb, oneshot=False)
		self.lock.release()
	def vad_cb(self, msg):
		self.speech = msg.vad
		self.erg = (self.maxerg - msg.energy)/float(self.maxerg - self.minerg)

	def arousal_cb(self, msg):
		self.arousal = msg.label
	
	def valence_cb(self, msg):
		self.valence = msg.label

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
			ma1.scale.x = 1*(1 + self.erg)
			ma1.scale.y = 1*(1 + self.erg)
			ma1.scale.z = 1*(1 + self.erg)
		else:
			ma1.scale.x = 0.1*(1 + self.erg)
			ma1.scale.y = 0.1*(1 + self.erg)
			ma1.scale.z = 0.1*(1 + self.erg)

		ma1.color.a = 1.0
		ma1.color.r = 0.0 if self.speech else 1.0
		ma1.color.g = 1.0 if self.speech else 0.0
		ma1.color.b = 0.0
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
		self.erg = 0
		self.speech = 0
		self.lock.release()

if __name__ == '__main__':

	sys.argv[len(sys.argv) - 1] = '--name'
	sys.argv[len(sys.argv) - 2] = '--default'

	parser = argparse.ArgumentParser()

	#options for VAD
	#automatic gain normalisation
	parser.add_argument("-g_min", "--gain_min", dest= 'g_min', type=int, help="min value of automatic gain normalisation", default=100)
	parser.add_argument("-g_max", "--gain_max", dest= 'g_max', type=int, help="max value of automatic gain normalisation", default=5000)

	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")

	#parser.add_argument("-h", "--help", help="help", action="store_true")

	args = parser.parse_args()

	rospy.init_node('sound_features_visualizer', anonymous=True)
	sv = SoundViz(args.g_min, args.g_max)
	rospy.spin()