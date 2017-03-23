#ifndef INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_
#define INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

// #include <aruco_simple/cameraparameters.h>
class ShirtDetector
{
  CascadeClassifier cascadeFace;
  std::string cascadeFileFace;
public:

        // void getImgForShirt(cv::Mat* imgIn, aruco::CameraParameters camParam_);
        ShirtDetector(std::string file_path){
          cascadeFileFace = file_path;
        };
        void getImgForShirt(const sensor_msgs::ImageConstPtr& imgIn);	// remove aruco reference
};


#endif /* INCLUDE_POSE_TRACKING_PEOPLE_RECOGNITION_H_ */
