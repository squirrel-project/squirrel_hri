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
This ROS package requires Ubuntu 14.02 or later version with g++4.8.

This ROS package requires external libraries such as pulse-audio and libboost-all-dev.

Please run the following steps BEFORE you run catkin_make.

`sudo apt-get install pulseaudio libpulse-dev libboost-all-dev'

## 2. Build <a id="2--build"/>

Please use catkin_make to build this.

catkin_make will automatically build libvad first because of its dependency on libvad.

## 3. Usage <a id="3--usage"/>
To get information of parameters, 

rosrun squirrel_vad voice_detector --help

Then, it will displays possible parameters:

Usage: options_description [options]
Allowed options:
  --help                  produce help message
  --window_len arg (=20)  window_len ms
  --mode arg (=0)         mode
  --threshold arg (=100)  threshold for energy
  --minLength arg (=500)  minimum length of an utterance(ms)
  --device arg (=default) device name (pactl list)

To run vad with default parameters,

rosrun squirrel_vad voice_detector

To see a published topic,

rostopic echo /voice_detector

ROS Messages are defined in :

squirrel_common/squirrel_vad_msgs

<a href="#top">top</a>
