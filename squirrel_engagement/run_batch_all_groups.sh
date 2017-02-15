for i in 1 2 3 4 5
do
   echo "Welcome $i decoding"
   java -jar BatchDecorder.jar -voice:./backup/speech_${i}A_1.wav.feat -voice:./backup/speech_${i}A_2.wav.feat -voice:./backup/speech_${i}A_3.wav.feat -feat_ext_config:./config/feat_ext.config -svm_config:./config/svm.rank.config -decayRate:1.0 -windowSize:5000
	cp ./log.0.txt ./backup/log_${i}A_1.txt
	cp ./log.1.txt ./backup/log_${i}A_2.txt
	cp ./log.2.txt ./backup/log_${i}A_3.txt

	java -jar BatchDecorder.jar -voice:./backup/speech_${i}B_1.wav.feat -voice:./backup/speech_${i}B_2.wav.feat -voice:./backup/speech_${i}B_3.wav.feat -feat_ext_config:./config/feat_ext.config -svm_config:./config/svm.rank.config -decayRate:1.0 -windowSize:5000
	cp ./log.0.txt ./backup/log_${i}B_1.txt
	cp ./log.1.txt ./backup/log_${i}B_2.txt
	cp ./log.2.txt ./backup/log_${i}B_3.txt

done

