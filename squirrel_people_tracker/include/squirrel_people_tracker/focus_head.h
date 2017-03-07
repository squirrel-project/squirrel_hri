#ifndef FOCUS_HEAD_H
#define FOCUS_HEAD_H

#include <vector>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

class FocusHead
{
public:
  FocusHead();
  ~FocusHead();

  void initialize(int argc, char **argv);
  void moveHead();

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void faceCallback(const people_msgs::PositionMeasurement::ConstPtr &msg);
  void legCallback(const people_msgs::PositionMeasurementArray::ConstPtr &msg);

  bool set_focus_on_head(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
  /* === ROS NODE HANDLE === */
  ros::NodeHandle *nh_;

  /* === SUBSCRIBERS === */
  ros::Subscriber cloud_sub_;  // subscriber for head locatlization 
  ros::Subscriber face_sub_;  // subscriber for face detections
  ros::Subscriber leg_sub_;  // subscriber for face detections

  /* === SERVICES === */
  ros::ServiceServer enable_focus_on_head_srv_;

  /* === MISC ===*/
  boost::mutex mutex_;
  std::vector<people_msgs::PositionMeasurement> possible_positions_;
  double height_;

  bool enabled_;
};
#endif  // FOCUS_HEAD_H
