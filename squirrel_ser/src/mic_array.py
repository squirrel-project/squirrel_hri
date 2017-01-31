#!/usr/bin/env python
import rospy
import sys
import struct
import serial
from std_msgs.msg import Int32

def get_sound_direction():
	pub = rospy.Publisher('sound_direction', Int32, queue_size=10)
	rospy.init_node('get_sound_direction')
	device = sys.argv[1]
	try:
		ser = serial.Serial(device, 2400, timeout=0)
	except:
	print "Error loading the mic on: " + device
	return 0
	rate = rospy.Rate(500) 
	while not rospy.is_shutdown():
	data = ser.read(1)
	try:
		 data_num = struct.unpack(">1B", data)[0]
			 pub.publish(data_num)
	except:
		 pass
		rate.sleep()
	ser.close()

if __name__ == '__main__':
	try:
		get_sound_direction()
	except rospy.ROSInterruptException:
		pass
