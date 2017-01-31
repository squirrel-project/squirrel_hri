import numpy as np
import librosa

def extract_melspec_file(path, file = None, n_fft=512, hop_length=512, n_mels=40, fmax= 8000):

	y, sr = librosa.load(path)
	mel = librosa.feature.melspectrogram(y=y, sr=sr, n_fft=n_fft, hop_length=hop_length, n_mels=n_mels, fmax= fmax) 
	mel = mel.T

	if file != None:
		np.savetxt(file, mel, fmt='%.8e', delimiter=';', newline='\n', header='', footer='')

	return mel 

def extract_melspec_frame(frames, file = None, sr = 16000, n_fft=512, hop_length=512, n_mels=40, fmax= 8000):

	mel = librosa.feature.melspectrogram(y=frames, sr=sr, n_fft=n_fft, hop_length=hop_length, n_mels=n_mels, fmax= fmax) 
	mel = mel.T
    
	if file != None:
		np.savetxt(file, mel, fmt='%.8e', delimiter=';', newline='\n', header='', footer='')
        
	return mel