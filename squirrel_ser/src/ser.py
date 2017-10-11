#!/usr/bin/env python
import pyaudio
import wave
import audioop
import webrtcvad
import argparse
import thread
import sys
import os
import numpy as np
from decoding import Decoder 
import wave
import rospy
import sys
import struct
import rospkg

from threading import Thread
from std_msgs.msg import Int32, Float32, Header
from squirrel_vad_msgs.msg import RecognisedResult 

def listup_devices():
	p = pyaudio.PyAudio()
	info = p.get_host_api_info_by_index(0)
	numdevices = info.get('deviceCount')
	for i in range(0, numdevices):
		if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
			print "Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'), " - ch: ", p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels'), " sr: ", p.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate')

def find_device_id(name):
	p = pyaudio.PyAudio()
	info = p.get_host_api_info_by_index(0)
	numdevices = info.get('deviceCount')
	for i in range(0, numdevices):
		if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
			print "Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'), " - ch: ", p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels'), " sr: ", p.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate')
			if name in p.get_device_info_by_host_api_device_index(0, i).get('name'):
				print name, " is found and will be used as an input device."
				return i
	print "There is no such a device named ", name
	return -1

def broadcast_result(task_publisher, task_outputs):
	for id in range(0, len(task_publisher)):
		output = task_outputs[id]
		msg = RecognisedResult()
		he = Header()
		he.stamp = rospy.Time.now()
		msg.header = he
		msg.label = output
		task_publisher[id].publish(msg)

def dummy_result(task_publisher):
	for id in range(0, len(task_publisher)):
		msg = RecognisedResult()
		he = Header()
		he.stamp = rospy.Time.now()
		msg.header = he
		msg.label = -1.0
		task_publisher[id].publish(msg)

def decay_result(task_publisher, prev_task_outputs, decay = 0.7):
	for task_id in range(0, len(task_publisher)):
		output = prev_task_outputs[task_id]
		if output < 0.001:
			output = 0.0
		else:
			output = output * decay
	return prev_task_outputs

def predict(dec, pyaudio, path, frames, rate = 16000,  reg = None, format = pyaudio.paInt16, g_min_max = None, save = False):
	wf = wave.open(path, 'wb')
	wf.setnchannels(1)
	wf.setsampwidth(pyaudio.get_sample_size(format))
	wf.setframerate(rate)
	wf.writeframes(b''.join(frames))
	wf.close()

	results = dec.predict_file(path, g_min_max = g_min_max)
	
	if save == False:
		os.remove(path)

	if reg:
		task_outputs = dec.returnDiff(results)
	else:
		task_outputs = dec.returnLabel(results)

	return task_outputs

def ser(args):

	#set temporay folder
	rospack = rospkg.RosPack()
	data_path = rospack.get_path("squirrel_ser") + "/data"

	try:
		os.mkdir(data_path)
	except:
		rospy.loginfo('Data folder already exists')

	if args.save:
		save = True
	else:
		save = False

	#ros node initialisation
	#tasks parsing
	tasks = args.tasks.split(",")
	task_publisher = []

	for task in tasks:
		task_n_class = task.split(":")
		task_publisher.append(rospy.Publisher(task_n_class[0], RecognisedResult, queue_size=10))
	
	duration_pub = rospy.Publisher('speech_duration', Float32, queue_size=10)
	
	rospy.init_node('ser')
	rate = rospy.Rate(500)

	#audio device setup
	format = pyaudio.paInt16
	
	n_channel = args.channels
	sample_rate = args.sample_rate
	frame_duration = args.frame_duration
	frame_len = (sample_rate * frame_duration / 1000)
	chunk = frame_len / 2
	vad_mode = args.vad_mode

	#feature extraction setting
	min_voice_frame_len = frame_len * (args.vad_duration / frame_duration)
	feat_path = args.feat_path

	#initialise vad
	vad = webrtcvad.Vad()
	vad.set_mode(vad_mode)

	#automatic gain normalisation
	if args.g_min and args.g_max:
		g_min_max = (args.g_min, args.g_max)
	else:
		g_min_max = None

	if args.model_file:
		#initialise model
		if args.stl:
		   dec = Decoder(model_file = args.model_file, elm_model_files = args.elm_model_file, context_len = args.context_len, max_time_steps = args.max_time_steps, tasks = args.tasks, sr = args.sample_rate)
		else:
		   dec = Decoder(model_file = args.model_file, elm_model_files = args.elm_model_file, context_len = args.context_len, max_time_steps = args.max_time_steps, tasks = args.tasks, stl = False, sr = args.sample_rate)
			
	p = pyaudio.PyAudio()

	#open mic
	if args.device_id is None:
		args.device_id = find_device_id("pulse")
		if args.device_id == -1:
			rospy.loginfo("There is no default device!, please check the configuration")
			sys.exit(-1)
			
	#open mic
	s = p.open(format = format, channels = n_channel,rate = sample_rate,input = True, input_device_index = args.device_id,frames_per_buffer = chunk)
	#s = p.open(format = format, channels = n_channel, rate = sample_rate, input = True, frames_per_buffer = chunk)
	
	rospy.loginfo("---MIC Starting---")

	is_currently_speech = False
	total_frame_len = 0
	frames = ''
	prev_task_outputs = None
	speech_frame_len = 0

	while not rospy.is_shutdown():
		try:
			data = s.read(chunk)
		except:
			rospy.loginfo(sys.exc_info()[0])
			rospy.loginfo("overflow, needs a higer priority")
			
			break

		#check gain
		mx = audioop.max(data, 2)
		#vad
		is_speech = vad.is_speech(data, sample_rate)
		if mx < args.min_energy:
			is_speech = 0

		rospy.logdebug('gain: %d, vad: %d', mx, is_speech)
			
		if args.sync:#synchronous mode
			if is_speech == 1:
				speech_frame_len = speech_frame_len + frame_len

			if frames == '': 
				frames = data
			else:
				frames = frames + data

			total_frame_len = total_frame_len + frame_len

			if args.model_file and total_frame_len > min_voice_frame_len:		
				
				if float(speech_frame_len)/total_frame_len > args.speech_ratio:

					task_outputs = predict(dec, p, data_path + "/" + str(rospy.Time.now()) + '.wav', frames, reg = args.reg, g_min_max = g_min_max, save = save)
					#rospy.loginfo(str(task_outputs))
					#broadcast results
					broadcast_result(task_publisher, task_outputs)
				else:
					dummy_result(task_publisher)

				#initialise threshold values
				duration_pub.publish(float(speech_frame_len)/total_frame_len)
			
				total_frame_len = 0
				speech_frame_len = 0
				frames = ''

		else:#asynchronous mode

			#Speech starts
			if is_currently_speech == False and is_speech == True:
				is_currently_speech = True
				rospy.loginfo("speech starts")
				frames = data
				total_frame_len = total_frame_len + frame_len
			elif is_currently_speech == True and is_speech == True:
				#rospy.loginfo("speech keeps coming")

				frames = frames + data
				total_frame_len = total_frame_len + frame_len
			elif is_currently_speech == True and is_speech == False:
				#Speech ends
				is_currently_speech = False
				total_frame_len = total_frame_len + frame_len
				rospy.loginfo("Detected speech duration: %d", total_frame_len)

				duration_pub.publish(total_frame_len)

				#if duration of speech is longer than a minimum speech
				if args.model_file and total_frame_len > min_voice_frame_len:		
					
					task_outputs = predict(dec, p, data_path + "/" + str(rospy.Time.now()) + '.wav', frames, reg = args.reg, g_min_max = g_min_max)
					rospy.loginfo(str(task_outputs))

					#broadcast results
					broadcast_result(task_publisher, task_outputs)
				#initialise threshold values
				total_frame_len = 0
				continue

		rate.sleep()

	rospy.loginfo("---done---")

	s.close()
	p.terminate()	

if __name__ == '__main__':

	sys.argv[len(sys.argv) - 1] = '--name'
	sys.argv[len(sys.argv) - 2] = '--default'

	parser = argparse.ArgumentParser()

	#options for VAD
	parser.add_argument("-ch", "--channels", dest= 'channels', type=int, help="number of channels", default=1)
	parser.add_argument("-sr", "--sample_rate", dest= 'sample_rate', type=int, help="number of samples per sec, only accept [8000|16000|32000]", default=16000)
	parser.add_argument("-fd", "--frame_duration", dest= 'frame_duration', type=int, help="a duration of a frame msec, only accept [10|20|30]", default=20)
	parser.add_argument("-vm", "--vad_mode", dest= 'vad_mode', type=int, help="vad mode, only accept [0|1|2|3], 0 more quiet 3 more noisy", default=0)
	parser.add_argument("-vd", "--vad_duration", dest= 'vad_duration', type=int, help="minimum length(ms) of speech for emotion detection", default=500)
	parser.add_argument("-me", "--min_energy", dest= 'min_energy', type=int, help="minimum energy of speech for emotion detection", default=100)
	parser.add_argument("-d_id", "--device_id", dest= 'device_id', type=int, help="device id for microphone", default=None)
	#automatic gain normalisation
	parser.add_argument("-g_min", "--gain_min", dest= 'g_min', type=float, help="min value of automatic gain normalisation", default=-1.37686)
	parser.add_argument("-g_max", "--gain_max", dest= 'g_max', type=float, help="max value of automatic gain normalisation", default=1.55433)
	parser.add_argument("-s_ratio", "--speech_ratio", dest= 'speech_ratio', type=float, help="speech ratio", default=0.3)

	#options for Model
	parser.add_argument("-fp", "--feat_path", dest= 'feat_path', type=str, help="temporay feat path", default='./temp.csv')
	parser.add_argument("-md", "--model_file", dest= 'model_file', type=str, help="keras model path")
	parser.add_argument("-elm_md", "--elm_model_file", dest= 'elm_model_file', type=str, help="elm model_file")
	parser.add_argument("-c_len", "--context_len", dest= 'context_len', type=int, help="context window's length", default=10)
	parser.add_argument("-m_t_step", "--max_time_steps", dest= 'max_time_steps', type=int, help="maximum time steps for DNN", default=500)
	#parser.add_argument("-n_class", "--n_class", dest= 'n_class', type=int, help="number of class", default=2)
	parser.add_argument("-tasks", "--tasks", dest = "tasks", type=str, help ="tasks (arousal:2,valence:2)", default='emotion_category')
	parser.add_argument("--stl", help="only for single task learning model", action="store_true")
	parser.add_argument("--reg", help="regression mode", action="store_true")
	
	parser.add_argument("--sync", help="sync", action="store_true")
	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")
	parser.add_argument("--save", help="save voice files", action="store_true")

	#parser.add_argument("-h", "--help", help="help", action="store_true")

	args = parser.parse_args()

	if len(sys.argv) == 1:
		parser.print_help()
		listup_devices()	   
		sys.exit(1)
	try:
		ser(args)
	except rospy.ROSInterruptException:
		pass
