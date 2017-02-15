java -jar LiveEngagementDetection.jar -mode:SVM -config:./config/vad.default.config.xml -ch:1 -rate:16000 -folder:./data/  -dev:1  -minDurationMSec:150 -feat_ext_config:./config/feat_ext.config -svm_config:./config/svm.rank.config -decayRate:0.5 -windowSize:5000 -record_wav:/Users/kimj/workplace/workspace_sq/squirrel_hri/squirrel_engagement/speech_1.wav

