<a id="top"/> 
# squirrel_vad
This folder has source codes for gmm based voice activity detection module which is runnable in ROS.

Maintainer: [**batikim09**](https://github.com/**github-user**/) (**batikim09**) - **j.kim@utwente.nl**

##Contents
1. <a href="#1--installation-requirements">Installation Requirements</a>

2. <a href="#2--build">Build</a>

3. <a href="#3--usage">Usage</a>

## 1. Installation Requirements <a id="1--installation-requirements"/>
####Debian packages

Please run the following steps BEFORE you run catkin_make.

`sudo apt-get install python-pip python-dev libhdf5-dev portaudio19-dev'

Next, using pip, install all pre-required modules.
pip install -r requirements.txt

## 2. Build <a id="2--build"/>

Please use catkin_make to build this.

## 3. Usage <a id="3--usage"/>
To get information of parameters, 

rosrun squirrel_ser ser.py

optional arguments are:
  -sr SAMPLE_RATE, --sample_rate SAMPLE_RATE
                        number of samples per sec(8000,16000,32000 only)
  -fd FRAME_DURATION, --frame_duration FRAME_DURATION
                        a duration of a frame (10,20,30msec)
  -vm VAD_MODE, --vad_mode VAD_MODE
                        vad mode(0,1,2,3)
  -vd VAD_DURATION, --vad_duration VAD_DURATION
                        vad duration(1000)
  -d_id DEVICE_ID, --device_id DEVICE_ID
                        device id
  -fp FEAT_PATH, --feat_path FEAT_PATH
                        feat path
  -md MODEL_FILE, --model_file MODEL_FILE
                        model path
  -elm_md ELM_MODEL_FILE, --elm_model_file ELM_MODEL_FILE
                        elm_model_file
  -c_len CONTEXT_LEN, --context_len CONTEXT_LEN
                        context_len
  -m_t_step MAX_TIME_STEPS, --max_time_steps MAX_TIME_STEPS
                        max_time_steps
  -n_class N_CLASS, --n_class N_CLASS
                        number of class
  -task TASK, --task TASK
                        tasks (arousal,valence)
  --stl                 stl
  
To run vad with default parameters,

rosrun squirrel_ser ser.py
(in this case, it performs only voice detection but not emotion detection since models are not specified.)

For a complete operation of emotion detection,
You must specify locations of models by passing arguments to ser.py
Particularly, you have to provide the relative paths of models from your current directory or absolute paths. For example,


To see a published topic,

rostopic echo /emotional_category

ROS Messages are defined in :

squirrel_common/squirrel_vad_msgs

<a href="#top">top</a>
