#!/usr/bin/env python
import pyaudio
import wave
import audioop
import webrtcvad
import argparse
import thread
import sys
import numpy as np
from decoding import Decoder 
import wave
import rospy
import sys
import struct
from threading import Thread

from std_msgs.msg import Int32, Header
from squirrel_vad_msgs.msg import RecognisedResult 

def listup_devices():
	p = pyaudio.PyAudio()
	#list up devices
	for i in range(p.get_device_count()):
		print p.get_device_info_by_index(i)
 
class DNNThread(Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs, Verbose)
        self._return = None
    def run(self):
        if self._Thread__target is not None:
            self._return = self._Thread__target(*self._Thread__args,
                                                **self._Thread__kwargs)
    def join(self):
        Thread.join(self)
        return self._return

def broadcast_result(task_publisher, task_outputs):
	for id in range(0, len(task_publisher)):
		for frame_output in task_outputs[id]:
			msg = RecognisedResult()
			he = Header()
			he.stamp = rospy.Time.now()
			msg.header = he
			msg.label = frame_output
			task_publisher[id].publish(msg)

def decay_result(task_publisher, prev_task_outputs, decay = 0.7):
	for task_id in range(0, len(task_publisher)):
		outputs = prev_task_outputs[task_id]
		for frame_id in range(0, len(outputs)):
			if outputs[frame_id] < 0.001:
				outputs[frame_id] = 0.0
			else:
				outputs[frame_id] = outputs[frame_id] * decay
	return prev_task_outputs

def predict(dec, pyaudio, path, frames, rate = 16000,  reg = None, format = pyaudio.paInt16, g_min_max = None):
	wf = wave.open(path, 'wb')
	wf.setnchannels(1)
	wf.setsampwidth(pyaudio.get_sample_size(format))
	wf.setframerate(rate)
	wf.writeframes(b''.join(frames))
	wf.close()

	results = dec.predict_file(path, g_min_max = g_min_max)
	if reg:
		task_outputs = dec.returnDiff(results)
	else:
		task_outputs = dec.returnLabel(results)

	return task_outputs

def ser(args):
	#ros node initialisation

	#tasks parsing
	tasks = args.tasks.split(",")
	task_publisher = []

	for task in tasks:
		task_n_class = task.split(":")
		task_publisher.append(rospy.Publisher(task_n_class[0], RecognisedResult, queue_size=10))
	
	duration_pub = rospy.Publisher('speech_duration', Int32, queue_size=10)
	vad_pub = rospy.Publisher('vad', Int32, queue_size=10)
	
	rospy.init_node('ser')
	
	rate = rospy.Rate(500)

	#audio device setup
	format = pyaudio.paInt16
	#format = pyaudio.paFloat32
	n_channel = 1
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
	#s = p.open(format = format, channels = n_channel,rate = sample_rate,input = True, input_device_index = args.device_id,frames_per_buffer = chunk)
	s = p.open(format = format, 
		   channels = n_channel,
		   rate = sample_rate,
		   input = True, 
		   frames_per_buffer = chunk)
	
	rospy.loginfo("---MIC Starting---")

	is_currently_speech = False
	total_frame_len = 0
	frames = ''
	prev_task_outputs = None

	while not rospy.is_shutdown():
		data = s.read(chunk)
		#check gain
		mx = audioop.max(data, 2)
		#vad
		is_speech = vad.is_speech(data, sample_rate)
		if mx < args.min_energy:
			is_speech = 0

		rospy.logdebug('gain: %d, vad: %d', mx, is_speech)
		
		vad_pub.publish(is_speech)
		

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
				
				task_outputs = predict(dec, p, args.save + "/" + str(rospy.Time.now()) + '.wav', frames, reg = args.reg, g_min_max = g_min_max)
				rospy.loginfo(str(task_outputs))

				prev_task_outputs = task_outputs
				#broadcast results
				broadcast_result(task_publisher, task_outputs)
			#initialise threshold values
			total_frame_len = 0
			continue

		if prev_task_outputs and args.decay != 0.0:
			prev_task_outputs = decay_result(task_publisher, prev_task_outputs, decay = args.decay)	
			broadcast_result(task_publisher, prev_task_outputs)

	rospy.loginfo("---done---")

	s.close()
	p.terminate()	

if __name__ == '__main__':

	sys.argv[len(sys.argv) - 1] = '--name'
	sys.argv[len(sys.argv) - 2] = '--default'

	parser = argparse.ArgumentParser()

	#options for VAD
	parser.add_argument("-sr", "--sample_rate", dest= 'sample_rate', type=int, help="number of samples per sec, only accept [8000|16000|32000]", default=16000)
	parser.add_argument("-fd", "--frame_duration", dest= 'frame_duration', type=int, help="a duration of a frame msec, only accept [10|20|30]", default=20)
	parser.add_argument("-vm", "--vad_mode", dest= 'vad_mode', type=int, help="vad mode, only accept [0|1|2|3], 0 more quiet 3 more noisy", default=0)
	parser.add_argument("-vd", "--vad_duration", dest= 'vad_duration', type=int, help="minimum length(ms) of speech for emotion detection", default=500)
	parser.add_argument("-me", "--min_energy", dest= 'min_energy', type=int, help="minimum energy of speech for emotion detection", default=100)
	parser.add_argument("-d_id", "--device_id", dest= 'device_id', type=int, help="device id for microphone", default=0)
	#automatic gain normalisation
	parser.add_argument("-g_min", "--gain_min", dest= 'g_min', type=float, help="min value of automatic gain normalisation", default=-1.37686)
	parser.add_argument("-g_max", "--gain_max", dest= 'g_max', type=float, help="max value of automatic gain normalisation", default=1.55433)
	parser.add_argument("-decay", "--decay", dest= 'decay', type=float, help="decay", default=0.9)

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
	parser.add_argument("-save", "--save", dest = "save", type=str, help ="save directory", default='./')

	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--name", help="name", action="store_true")

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
