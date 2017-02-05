#ifndef FOCUS_HEAD_H
#define FOCUS_HEAD_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>

class FocusHead
{
public:
  FocusHead();
  ~FocusHead();

  void initialize(int argc, char **argv);

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  bool set_focus_on_head(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
  /* === ROS NODE HANDLE === */
  ros::NodeHandle *nh_;

  /* === SUBSCRIBERS === */
  ros::Subscriber cloud_sub_;  // subscriber for face detections

  /* === SERVICES === */
  ros::ServiceServer enable_focus_on_head_srv_;

  /* === MISC ===*/
  boost::mutex mutex_;

  bool enabled_;
};
#endif  // FOCUS_HEAD_H
