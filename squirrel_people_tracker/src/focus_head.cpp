#include "squirrel_people_tracker/focus_head.h"

FocusHead::FocusHead()
{
  nh_ = new ros::NodeHandle ("~");
}

FocusHead::~FocusHead()
{
  if(nh_)
    delete nh_;
}

void FocusHead::initialize(int argc, char **argv)
{
  enable_focus_on_head_srv_ = nh_->advertiseService("focus_on_head", &FocusHead::set_focus_on_head, this);

  cloud_sub_ = nh_->subscribe("input_cloud", 100, &FocusHead::cloudCallback, this);
  face_sub_ = nh_->subscribe("attention_tracker/faces/people_tracker_measurements_array", 1000,
                              &FocusHead::cloudCallback, this);
  leg_sub_ = nh_->subscribe("legs", 100, &FocusHead::legCallback, this);

  ROS_INFO("Ready to receive service calls");
}

void FocusHead::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO("inside cloudCallback");
}

void FocusHead::faceCallback(const people_msgs::PositionMeasurement::ConstPtr &msg)
{
  ROS_INFO("inside faceCallback");
}

void FocusHead::legCallback(const people_msgs::PositionMeasurementArray::ConstPtr &msg)
{
  ROS_INFO("inside legCallback");
  boost::mutex::scoped_lock lock(this->mutex_);
  for (size_t i = 0; i < msg->people.size(); ++i)
  {
    possible_positions_.push_back(msg->people[i]); 
  }
  
}

bool FocusHead::set_focus_on_head(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  this->enabled_ = req.data;
  resp.success = true;
  return true;
}

void FocusHead::moveHead()
{
  double rotate = 0.f;
  boost::mutex::scoped_lock lock(this->mutex_);
  for (size_t i = 0; i < possible_positions_.size(); ++i)
  {
    rotate = possible_positions_[i].pos.y/possible_positions_[i].pos.x;
  }
}

