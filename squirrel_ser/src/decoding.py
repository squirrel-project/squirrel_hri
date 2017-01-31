from __future__ import print_function
import numpy as np
import keras
import tensorflow as tf
import sys
import argparse

from keras import backend as K
from keras.models import Model
from feat_ext import *
from elm import ELM
from high_level import *



class Decoder(object):
	def __init__(self, model_file = './model.h5', elm_model_file = './model.elm.ckpt', feat_path = './temp.csv', context_len = 5, max_time_steps = 300, n_class = 3, elm_hidden_num = 50, stl = True, elm_main_task_id = 0, sr = 16000):
		
		self.stl = stl
		self.model = keras.models.load_model(model_file)
		self.elm_model_file = elm_model_file
		self.sess = tf.Session()
		if self.elm_model_file != None:
			self.elm_model = ELM(self.sess, 1, n_class * 4, elm_hidden_num, n_class)
			self.elm_model.load(elm_model_file)
			self.elm_hidden_num = elm_hidden_num
			self.elm_main_task_id = elm_main_task_id

		self.sr = sr
		self.feat_path = feat_path
		self.context_len = context_len
		self.max_time_steps = max_time_steps   
		self.model.summary()

	def predict(self, frames):
		mspec = extract_melspec_frame(frames, file = self.feat_path, n_mels = 80, sr = self.sr)
		temporal_feat = self.build_temporal_feat(mspec)
		return self.temporal_predict(temporal_feat)

    
	def predict_file(self, input_file):
		mspec = extract_melspec_file(input_file, file = self.feat_path, n_mels = 80)
		temporal_feat = self.build_temporal_feat(mspec)
		return self.temporal_predict(temporal_feat)
	
	def temporal_predict(self, temporal_feat):	
		predictions = self.model.predict(temporal_feat)

		if self.elm_model_file == None:
			return predictions
		else:
			feat_test = high_level_feature_mtl(predictions, threshold = 0.3, stl = self.stl, main_task_id = self.elm_main_task_id)
			preds = self.elm_model.test(feat_test)
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
		if (len(result.shape) == 3):
			for task in result:
				label = np.argmax(task, 1)
				labels.append(label)
		else:#single task output
			labels.append(np.argmax(result, 1))

		#but results are always multi-tasks format
		return labels
				

if __name__ == "__main__":
	print('example of decoding')
	parser = argparse.ArgumentParser()
	parser.add_argument("-wav", "--wave", dest= 'wave', type=str, help="wave file", default='./test.wav')
	parser.add_argument("-md", "--model_file", dest= 'model_file', type=str, help="model file", default='./model.h5')
	parser.add_argument("-elm_md", "--elm_model_file", dest= 'elm_model_file', type=str, help="elm_model_file")
	parser.add_argument("-c_len", "--context_len", dest= 'context_len', type=int, help="context_len", default=5)
	parser.add_argument("-m_t_step", "--max_time_steps", dest= 'max_time_steps', type=int, help="max_time_steps", default=5)
	parser.add_argument("-n_class", "--n_class", dest= 'n_class', type=int, help="number of class", default=2)
	parser.add_argument("--default", help="default", action="store_true")
	parser.add_argument("--stl", help="stl", action="store_true")

	args = parser.parse_args()
	if len(sys.argv) == 1:
		parser.print_help()
		sys.exit(1)

	if args.stl:
		dec = Decoder(model_file = args.model_file, elm_model_file = args.elm_model_file, feat_path = './temp.csv', context_len = args.context_len, max_time_steps = args.max_time_steps, n_class = args.n_class)
	else:
		dec = Decoder(model_file = args.model_file, elm_model_file = args.elm_model_file, feat_path = './temp.csv', context_len = args.context_len, max_time_steps = args.max_time_steps, n_class = args.n_class, stl = False)
	
	result = dec.predict_file(args.wave)
	print(result)
	print(dec.returnLabel(result))
	
	

