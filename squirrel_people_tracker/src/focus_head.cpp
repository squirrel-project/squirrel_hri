#include "squirrel_people_tracker/focus_head.h"

FocusHead::FocusHead()
{
}

FocusHead::~FocusHead()
{
  if (nh_)
    delete nh_;
}

void FocusHead::initialize(int argc, char **argv)
{
  ros::init(argc, argv, "focus_head");
  nh_ = new ros::NodeHandle("~");

  // Only use laser leg detection
  nh_->param("face_detection", enabled_, false);

  enable_focus_on_head_srv_ = nh_->advertiseService("focus_on_head", &FocusHead::set_focus_on_head, this);

  // cloud_sub_ = nh_->subscribe("input_cloud", 100, &FocusHead::cloudCallback, this);
  cloud_sub_ = nh_->subscribe("attention_tracker/faces/people_tracker_measurements_array", 1000,
                              &FocusHead::cloudCallback, this);

  ROS_INFO("Ready to receive service calls");
  ros::spin();
}

void FocusHead::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  ROS_INFO("inside callback");
}

void FocusHead::laser_leg_Callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  ROS_INFO("inside callback");
}

bool FocusHead::set_focus_on_head(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  this->enabled_ = req.data;
  resp.success = true;
  return true;
}
