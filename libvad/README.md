<a id="top"/> 
# libvad
This folder has source codes for gmm based voice activity detection module which will be used on squirrel_vad (ROS executable binary). You do not have to build this separately. Just run catkin_make in your catkin_ws/. 

Maintainer: [**batikim09**](https://github.com/**github-user**/) (**batikim09**) - **j.kim@utwente.nl**

##Contents
1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--build">Build</a>

## 1. Installation Requirements <a id="1--installation-requirements"/>
####Debian packages

To utilise c+11 functions which is not supported by ROS yet, this library requires g++-4.8.

`sudo apt-get install g++-4.8'

Before you run catkin_make, g++-4.8 must be installed in your system. 

g++-4.8 will not be used for any other modules but only for this.

## 2. Build <a id="2--build"/>

You do not have to build this separtely. Use catkin_make.  

<a href="#top">top</a>
