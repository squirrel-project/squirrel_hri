// ShirtDetection v1.2: Find the approximate color type of a person's tshirt. by Shervin Emami (shervin.emami@gmail.com), 30th Aug 2010.

// If trying to debug the color detector code, enable SHOW_DEBUG_IMAGE:
#define SHOW_DEBUG_IMAGE

//#include <stdio.h>
//#include <tchar.h>

#include <cstdio>       // Used for "printf"
#include <string>       // Used for C++ strings
#include <iostream>     // Used for C++ cout print statements
#include <ros/package.h>
#include <squirrel_person_tracker/people_recognition.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "shirt_detector");
    ros::NodeHandle nh_;
    ShirtDetector sd((ros::package::getPath("squirrel_person_tracker") + "/config/haarcascades_frontalfaces_alt.xml").c_str(),
                     nh_);
    ros::spin();
}

