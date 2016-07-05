<a id="top"/> 
# squirrel_gmm_vad
This folder has source codes for gmm based voice activity detection module. Using CMAKE, a shared library (.so) and an example application can be built. The shared library will be used for squirrel_vad which is a ROS-excutable binary.

Maintainer: [**batikim09**](https://github.com/**github-user**/) (**batikim09**) - **j.kim@utwente.nl**

##Contents
1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--build">Build</a>

## 1. Installation <a id="1--installation-requirements"/>
####Debian packages

To utilise c+11 functions which is not supported by ROS yet, this library must be built separately.

g++-4.8, pulse-audio, boost are required.

`sudo apt-get install g++-4.8 pulse-audio libboost-all-dev'

## 2. Build <a id="2--build"/>

Please follow the steps.
 
mkdir ./build

cd ./build

cmake -DCMAKE_CXX_COMPILER=g++-4.8 ..

make

sudo make install
<a href="#top">top</a>
