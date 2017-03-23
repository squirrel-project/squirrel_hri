#ifndef INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_
#define INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>       // Used for "printf"
#include <string>       // Used for C++ strings
#include <iostream>     // Used for C++ cout print statements
#include <ros/ros.h>
#include <squirrel_person_tracker/ImageUtils.h>         // Used for easy image cropping, resizing, rotating, etc.
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <squirrel_person_tracker_msgs/ShirtMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;
// Various color types for detected shirt colors.
enum {
    cBLACK = 0, cWHITE, cGREY, cRED, cORANGE, cYELLOW, cGREEN, cAQUA, cBLUE, cPURPLE, cPINK, NUM_COLOR_TYPES
};

char* sCTypes[NUM_COLOR_TYPES] = {"Black", "White", "Grey", "Red", "Orange", "Yellow", "Green", "Aqua", "Blue", "Purple", "Pink"};
uchar cCTHue[NUM_COLOR_TYPES] = {0, 0, 0, 0, 20, 30, 55, 85, 115, 138, 161};
uchar cCTSat[NUM_COLOR_TYPES] = {0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255};
uchar cCTVal[NUM_COLOR_TYPES] = {0, 255, 120, 255, 255, 255, 255, 255, 255, 255, 255};

class ShirtDetector {
    CascadeClassifier cascadeFace;
    std::string cascadeFileFace;
public:
    ShirtDetector(const std::string& file_path, ros::NodeHandle& nh);
    void getImgForShirt(const sensor_msgs::ImageConstPtr& imgIn, std::vector<Rect>& rectShirt,
                        squirrel_person_tracker_msgs::ShirtMsg& msg);
    int getPixelColorType(int H, int S, int V);
    void sync_cb(const sensor_msgs::ImageConstPtr& imgIn, const sensor_msgs::PointCloud2ConstPtr& pcloud);
    void pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud, const int u, const int v, geometry_msgs::Point &p);
    ros::NodeHandle nh_, gh;
    ros::Publisher imagePub_;
    ros::Publisher poseArrayPub_;
    ros::Publisher shirtPub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > pc_sub_;
    std::string camera_frame_;
};

#endif /* INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_ */
