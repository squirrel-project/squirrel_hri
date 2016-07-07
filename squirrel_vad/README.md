<a id="top"/> 
# squirrel_vad
This folder has source codes for gmm based voice activity detection module which is runnable in ROS.

Maintainer: [**batikim09**](https://github.com/**github-user**/) (**batikim09**) - **j.kim@utwente.nl**

##Contents
1. <a href="#1--installation-requirements">Installation Requirements</a>

2. <a href="#2--build">Build</a>

## 1. Installation Requirements <a id="1--installation-requirements"/>
####Debian packages
This ROS package requires external libraries such as pulse-audio and libboost-all-dev.

Please run the following steps BEFORE you run catkin_make.

`sudo apt-get install pulseaudio libpulse-dev libboost-all-dev'

## 2. Build <a id="2--build"/>

Please use catkin_make to build this.
catkin_make will automatically build libvad first because of its dependency on libvad.

<a href="#top">top</a>
