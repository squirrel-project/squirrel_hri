from __future__ import print_function
import numpy as np
import keras
import tensorflow as tf
import sys
import argparse
import rospkg

from keras import backend as K
from keras.models import Model
from feat_ext import *
from elm import ELM
from high_level import *



class Decoder(object):
	def __init__(self, model_file = './model.h5', elm_model_files = None, feat_path = './temp.csv', context_len = 5, max_time_steps = 300, elm_hidden_num = 50, stl = True, elm_main_task_id = -1, sr = 16000, tasks = 'arousal:2,valence:2'):
		
		self.stl = stl
		rospack = rospkg.RosPack()
		self.base_path = rospack.get_path("squirrel_ser") + "/model/"

		self.model = keras.models.load_model(self.base_path + model_file)
		self.elm_model_files = elm_model_files
		self.sess = tf.Session()
		self.elm_model = []
		self.tasks = []
		self.tasks_names = []
		self.total_high_level_feat = 0

		for task in tasks.split(","):
			task_n_class = task.split(':') 
			self.tasks.append(int(task_n_class[1]))
			self.tasks_names.append(task_n_class[0])
			self.total_high_level_feat = self.total_high_level_feat + int(task_n_class[1])            
		
		if self.elm_model_files != None:
			print("elm model is loaded")
			elm_tasks = elm_model_files.split(',')
			if len(elm_tasks) == len(self.tasks):
				print("#tasks: ", len(self.tasks))

				for i in range(0, len(self.tasks)):
					elm_model_task = ELM(self.sess, 1, self.total_high_level_feat * 4, elm_hidden_num, self.tasks[i], task = self.tasks_names[i])
					elm_path = self.base_path + elm_tasks[i] 
					elm_model_task.load(elm_path)

					self.elm_model.append(elm_model_task)	
				self.elm_hidden_num = elm_hidden_num
				self.elm_main_task_id = elm_main_task_id
			else:
				print("mismatch between tasks and elm models")
				exit()

		self.sr = sr
		self.feat_path = feat_path
		self.context_len = context_len
		self.max_time_steps = max_time_steps   
		self.model.summary()

	def predict(self, frames):
		mspec = extract_melspec_frame(frames, file = self.feat_path, n_mels = 80, sr = self.sr)
		temporal_feat = self.build_temporal_feat(mspec)
		return self.temporal_predict(temporal_feat)

    
	def predict_file(self, input_file, g_min_max = None):
		mspec = extract_melspec_file(input_file, file = self.feat_path, n_mels = 80, min_max = g_min_max)
		temporal_feat = self.build_temporal_feat(mspec)
		return self.temporal_predict(temporal_feat)
	
	def predict_long_file(self, input_file, g_min_max = None):
		mspec = extract_melspec_file(input_file, file = self.feat_path, n_mels = 80, min_max = g_min_max)

		n_turns = mspec.shape[0] / self.max_time_steps
		result = []
		for i in range(0, n_turns):
			start = i * self.max_time_steps
			end = (i + 1) * self.max_time_steps
			temporal_feat = self.build_temporal_feat(mspec[start:end])
			result.append(self.temporal_predict(temporal_feat))

		return result

	def temporal_predict(self, temporal_feat):	
		#print("temporal feat shape: ", temporal_feat.shape)
		predictions = self.model.predict(temporal_feat)

		if self.elm_model_files == None:
			preds = []
			#print("no elm post")
			
			#print(str(predictions))
			for i in range(0, len(self.tasks)):
				#print("shape", predictions[i][0].shape)
				preds.append(predictions[i][0])
		else:
			feat_test = high_level_feature_mtl(predictions, threshold = 0.3, stl = self.stl, main_task_id = self.elm_main_task_id)
			preds = []
			#print("feat: ", str(feat_test))
			for i in range(0, len(self.tasks)):
				elm_predictions = self.elm_model[i].test(feat_test)
				#print("shape", elm_predictions.shape)
				preds.append(elm_predictions)
		
		return preds

	def build_temporal_feat(self, mspec):
		input_dim = mspec.shape[1]
		max_t_steps = int(self.max_time_steps / self.context_len)
		feat = np.zeros((1, max_t_steps, 1, self.context_len, input_dim))
		for t_steps in range(max_t_steps):
			if t_steps * self.context_len < mspec.shape[0] - self.context_len:
				if mspec.shape[1] != input_dim:
					print('inconsistent dim')
					break
				for c in range(self.context_len):
					feat[0, t_steps, 0, c, ] = mspec[t_steps * self.context_len + c]
		return feat

	def returnLabel(self, result):
		labels = []
		
		#multi-tasks output format
		for task in result:
			label = np.argmax(task, 1)
			most_frequent = np.bincount(label).argmax()
			labels.append(most_frequent)
		
		#but results are always multi-tasks format
		return labels

	def returnDiff(self, result):
		labels = []
		#multi-tasks output format
		
		for task in result:
			values = task.T
			label = values[len(values) - 1] - values[0]
			avg = np.mean(label)
			labels.append(avg)
		
		#but results are always multi-tasks format
		return labels		

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-wav", "--wave", dest= 'wave', type=str, help="wave file", default='./test.wav')
	parser.add_argument("-l_wav", "--long_wave", dest= 'long_wave', type=str, help="long wave file")
	parser.add_argument("-wav_list", "--wave_list", dest= 'wave_list', type=str, help="wave file list", default='./test.wav.txt')
	parser.add_argument("-g_min", "--gain_min", dest= 'g_min', type=float, help="min value of automatic gain normalisation", default=-1.37686)
	parser.add_argument("-g_max", "--gain_max", dest= 'g_max', type=float, help="max value of automatic gain normalisation", default=1.55433)
	parser.add_argument("-md", "--model_file", dest= 'model_file', type=str, help="model file", default='./model.h5')
	parser.add_argument("-elm_md", "--elm_model_files", dest= 'elm_model_files', type=str, help="elm_model_file")
	parser.add_argument("-c_len", "--context_len", dest= 'context_len', type=int, help="context_len", default=5)
	parser.add_argument("-m_t_step", "--max_time_steps", dest= 'max_time_steps', type=int, help="max_time_steps", default=500)
	parser.add_argument("-tasks", "--tasks", dest = "tasks", type=str, help ="tasks (arousal:2,valence:2)", default='emotion_category')
	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--stl", help="stl", action="store_true")
	parser.add_argument("--reg", help="regression mode", action="store_true")
	args = parser.parse_args()
	if len(sys.argv) == 1:
		parser.print_help()
		sys.exit(1)

	if args.g_min and args.g_max:
		g_min_max = (args.g_min, args.g_max)
	else:
		g_min_max = None

	if args.stl:
		dec = Decoder(model_file = args.model_file, elm_model_files = args.elm_model_files, feat_path = './temp.csv', context_len = args.context_len, max_time_steps = args.max_time_steps, tasks=args.tasks)
	else:
		dec = Decoder(model_file = args.model_file, elm_model_files = args.elm_model_files, feat_path = './temp.csv', context_len = args.context_len, max_time_steps = args.max_time_steps, tasks=args.tasks, stl = False)
	
	if args.long_wave:
		results = dec.predict_long_file(args.long_wave, g_min_max)
		output = open(args.long_wave + '.out', 'w')

		for result in results:
			if args.reg:
				result = dec.returnDiff(result)
			else:
				result = dec.returnLabel(result)
			print(result)
			for task_id in range(0, len(result)):
				output.write(str(result[task_id]) + '\t')
			output.write('\n')
		output.close()
		print('batch decoding is done, total number of outputs: ', len(results))
		
	elif args.wave_list:
		inputs = open(args.wave_list, 'r')
		output = open(args.wave_list + '.out', 'w')
		
		for input in inputs:
			input = input.rstrip()
			result = dec.predict_file(input, g_min_max)
			print(result)
			if args.reg:
				result = dec.returnDiff(result)
			else:
				result = dec.returnLabel(result)
			print(result)
			output.write(input + '\t')
			for task_id in range(0, len(result)):
				output.write(str(result[task_id]) + '\t')
			output.write('\n')

		output.close()
	else:
		result = dec.predict_file(args.wave)
		print(result)
		if args.reg:
			result = dec.returnDiff(result)
		else:
			result = dec.returnLabel(result)
	