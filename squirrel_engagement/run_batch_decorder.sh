if [ $# == 0 ]; then
	SESSION_ID=TEMP
fi

if [ $# == 1 ]; then
SESSION_ID=$1
fi



#run batch decoding
java -jar BatchDecorder.jar -voice:/Users/kimj/workplace/workspace_sq/squirrel_hri/squirrel_engagement/speech_1.wav.feat -voice:/Users/kimj/workplace/workspace_sq/squirrel_hri/squirrel_engagement/speech_2.wav.feat -voice:/Users/kimj/workplace/workspace_sq/squirrel_hri/squirrel_engagement/speech_3.wav.feat -feat_ext_config:./config/feat_ext.config -svm_config:./config/svm.rank.config -decayRate:1.0 -windowSize:5000

echo "Session ID: "${SESSION_ID}
#plotting results
plot.sh

#back up all recordings and feature files
cp ./speech_1.wav ./backup/speech_${SESSION_ID}_1.wav
cp ./speech_2.wav ./backup/speech_${SESSION_ID}_2.wav
cp ./speech_3.wav ./backup/speech_${SESSION_ID}_3.wav

cp ./speech_1.wav.feat ./backup/speech_${SESSION_ID}_1.wav.feat
cp ./speech_2.wav.feat ./backup/speech_${SESSION_ID}_2.wav.feat
cp ./speech_3.wav.feat ./backup/speech_${SESSION_ID}_3.wav.feat

cp ./log.0.txt ./backup/log_${SESSION_ID}_1.txt
cp ./log.1.txt ./backup/log_${SESSION_ID}_2.txt
cp ./log.2.txt ./backup/log_${SESSION_ID}_3.txt

