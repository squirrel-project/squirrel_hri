#ifndef SQUIRREL_PEOPLE_TRACKER_H
#define SQUIRREL_PEOPLE_TRACKER_H

#include <ros/ros.h>
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/People.h"
#include <std_srvs/SetBool.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "mongodb_store/message_store.h"
#include <vector>

class PeopleTracker
{
public:
  PeopleTracker();
  ~PeopleTracker();

  void initialize(int argc, char **argv);
  void update_people_in_db();

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void legCallBack(const people_msgs::People &msg);

  bool set_face_detection(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool set_adapt_to_height(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool set_skeleton_tracking(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
  /* === ROS NODE HANDLE === */
  ros::NodeHandle *nh_;

  /* === SUBSCRIBERS === */
  ros::Subscriber laser_legs_sub_;  // subscriber for the leg detections
  ros::Subscriber cloud_sub_;

  /* === PUBLISHERS === */
  ros::Publisher people_array_;  // publisher for tracked people_msgs

  /* === SERVICES === */
  ros::ServiceServer set_face_detection_srv_;
  ros::ServiceServer set_skeleton_tracking_srv_;
  ros::ServiceServer set_adapt_to_height_srv_;

  /* === MISC ===*/
  boost::mutex mutex_;
  // mongodb_store::MessageStoreProxy message_store;

  bool leg_detection_enabled_;
  bool face_detection_enabled_;
  bool skeleton_tracking_enabled_;
  bool adapt_to_height_enabled_;
  std::vector<people_msgs::Person> last_leg_observation_;
};
#endif  // SQUIRREL_PEOPLE_TRACKER_H
