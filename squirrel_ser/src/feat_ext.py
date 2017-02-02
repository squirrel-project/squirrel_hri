import numpy as np
import librosa
from sklearn.decomposition import PCA
 
def extract_melspec_frame(frames, file = None, sr = 16000, n_fft=512, hop_length=512, n_mels=40, fmax= 8000):

	mel = librosa.feature.melspectrogram(y=frames, sr=sr, n_fft=n_fft, hop_length=hop_length, n_mels=n_mels, fmax= fmax) 
	mel = mel.T
    
	if file != None:
		np.savetxt(file, mel, fmt='%.8e', delimiter=';', newline='\n', header='', footer='')
        
	return mel

def extract_log_spectrogram_frame(frames, file = None, sr = 16000, n_fft=512, hop_length=512):

	#spec = librosa.feature.logfsgram(y=frames, sr=sr, S=None, n_fft=n_fft, hop_length=hop_length)
	spec = np.abs(librosa.stft(frames, n_fft = n_fft))
	log_spec = librosa.logamplitude(spec**2)
	log_spec = log_spec.T

	if file != None:
		np.savetxt(file, log_spec, fmt='%.8e', delimiter=';', newline='\n', header='', footer='')

	return log_spec 

def extract_pca_whitenining(frames, pca_components = 60):
	pca = PCA(n_components=pca_components, whiten = True)
	return pca.fit_transform(frames)

def extract_pca_logspec_frame(frames, file = None, sr = 16000, n_fft=512, hop_length=512, pca_components = 60):
	spec = extract_log_spectrogram_frame(frames, file = None, sr = sr, n_fft=n_fft, hop_length=hop_length)
	pca_spec = extract_pca_whitenining(spec, pca_components= pca_components)

	if file != None:
		np.savetxt(file, pca_spec, fmt='%.8e', delimiter=';', newline='\n', header='', footer='')

	return pca_spec

def extract_melspec_file(path, file = None, n_fft=512, hop_length=512, n_mels=40, fmax= 8000):

	y, sr = librosa.load(path)
	mel = extract_melspec_frame(y, file = file, sr = sr, n_fft = n_fft, hop_length = hop_length, n_mels = n_mels, fmax = fmax)
	return mel

def extract_log_spectrogram_file(path, file = None, n_fft=512, hop_length=512, n_mels=40, fmax= 8000):

	y, sr = librosa.load(path)
	spec = extract_log_spectrogram_frame(y, file = file, sr = sr, n_fft = n_fft, hop_length = hop_length )
	return spec

def extract_pca_logspec_file(path, file = None, n_fft=512, hop_length=512, fmax= 8000, pca_components = 60):

	y, sr = librosa.load(path)
	spec = extract_pca_logspec_frame(y, file = file, sr = sr, n_fft = n_fft, hop_length = hop_length, pca_components =pca_components)
	return spec