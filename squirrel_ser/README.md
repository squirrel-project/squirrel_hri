<a id="top"/> 
# squirrel_ser
This folder has source codes for deep convolutional neural network-based speech emotion recognition. Note that this module relies on many machine learning packages and platforms such as Google Tensorflow and Keras, which is comptutationally expensive without GPU supports. Hence, it may not be operationable on the robot, rather deployment on an external machine is recommended. This module requires two trained models: keras-model and elm-model. Keras-model is either convolutional DNN or convolutional LSTM, which output frame-level prediction. If elm-model is given, it proceeds the outputs of keras-model to predict utterance-level prediction. (The latter gives better results so far). Performance varies on speakers and environment.

Maintainer: [**batikim09**](https://github.com/**github-user**/) (**batikim09**) - **j.kim@utwente.nl**

##Contents
1. <a href="#1--installation-requirements">Installation Requirements</a>

2. <a href="#2--build">Build</a>

3. <a href="#3--usage">Usage</a>

4. <a href="#3--references">References</a>

## 1. Installation Requirements <a id="1--installation-requirements"/>
####Debian packages

Please run the following steps BEFORE you run catkin_make.

`sudo apt-get install python-pip python-dev libhdf5-dev portaudio19-dev'

Next, using pip, install all pre-required modules.
pip install -r requirements.txt

## 2. Build <a id="2--build"/>

Please use catkin_make to build this.

## 3. Usage <a id="3--usage"/>
For a quick start, run in the terminal:

roslunach squirrel_ser ser.launch

For a visualisation,

rosluanch squirrel_ser viz.launch

To get information of parameters, 

rosrun squirrel_ser ser.py

usage: --default [-h] [-sr SAMPLE_RATE] [-fd FRAME_DURATION] [-vm VAD_MODE]
                 [-vd VAD_DURATION] [-me MIN_ENERGY] [-d_id DEVICE_ID]
                 [-g_min G_MIN] [-g_max G_MAX] [-fp FEAT_PATH]
                 [-md MODEL_FILE] [-elm_md ELM_MODEL_FILE]
                 [-c_len CONTEXT_LEN] [-m_t_step MAX_TIME_STEPS]
                 [-tasks TASKS] [--stl] [--reg] [-save SAVE] [--default]
                 [--name]

optional arguments:
  
  -h, --help            show this help message and exit
  
  -sr SAMPLE_RATE, --sample_rate SAMPLE_RATE
                        number of samples per sec, only accept
                        [8000|16000|32000]
  
  -fd FRAME_DURATION, --frame_duration FRAME_DURATION
                        a duration of a frame msec, only accept [10|20|30]
  
  -vm VAD_MODE, --vad_mode VAD_MODE
                        vad mode, only accept [0|1|2|3], 0 more quiet 3 more
                        noisy
  
  -vd VAD_DURATION, --vad_duration VAD_DURATION
                        minimum length(ms) of speech for emotion detection
  
  -me MIN_ENERGY, --min_energy MIN_ENERGY
                        minimum energy of speech for emotion detection
  
  -d_id DEVICE_ID, --device_id DEVICE_ID
                        device id for microphone
  
  -g_min G_MIN, --gain_min G_MIN
                        min value of automatic gain normalisation
  
  -g_max G_MAX, --gain_max G_MAX
                        max value of automatic gain normalisation
  
  -fp FEAT_PATH, --feat_path FEAT_PATH
                        temporay feat path
  
  -md MODEL_FILE, --model_file MODEL_FILE
                        keras model path
  
  -elm_md ELM_MODEL_FILE, --elm_model_file ELM_MODEL_FILE
                        elm model_file
  
  -c_len CONTEXT_LEN, --context_len CONTEXT_LEN
                        context window's length
  
  -m_t_step MAX_TIME_STEPS, --max_time_steps MAX_TIME_STEPS
                        maximum time steps for DNN
  
  -tasks TASKS, --tasks TASKS
                        tasks (arousal:2,valence:2)
  
  --reg                 regression mode
  
  --save SAVE, --save SAVE
                        save directory
  
  --default             default
  
  --name                name
  
  
To run voice detection with default parameters,

rosrun squirrel_ser ser.py --default

(in this case, it performs only voice detection but not emotion detection since models are not specified.)

To see a published topic,

rostopic echo /arousal

rostopic echo /valence

ROS Messages are defined in :

squirrel_common/squirrel_vad_msgs/RecognisedResult.msg

To see visualisation (MarkerArray), set fixed frame as 'sound', first and add the marker by topic name.

<a href="#top">top</a>
