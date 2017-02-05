#include "squirrel_people_tracker/squirrel_people_tracker.h"
#include "people_msgs/People.h"
#include "people_msgs/Person.h"
#include "people_msgs/PositionMeasurement.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

PeopleTracker::PeopleTracker()
{
  // message_store(nh_);
}

PeopleTracker::~PeopleTracker()
{
  if (nh_)
    delete nh_;
}

void PeopleTracker::initialize(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_people_tracker");
  nh_ = new ros::NodeHandle("~");

  // Only use laser leg detection
  nh_->param("leg_detection", leg_detection_enabled_, true);
  nh_->param("face_detection", face_detection_enabled_, false);
  nh_->param("adapt_to_height", adapt_to_height_enabled_, false);
  nh_->param("skeleton_tracking", skeleton_tracking_enabled_, false);

  set_face_detection_srv_ =
      nh_->advertiseService("people_tracker/set_face_detection", &PeopleTracker::set_face_detection, this);
  set_skeleton_tracking_srv_ =
      nh_->advertiseService("people_tracker/set_skeleton_tracking", &PeopleTracker::set_skeleton_tracking, this);
  set_adapt_to_height_srv_ =
      nh_->advertiseService("people_tracker/set_set_adapt_to_height", &PeopleTracker::set_adapt_to_height, this);

  laser_legs_sub_ = nh_->subscribe("leg_persons", 1000, &PeopleTracker::legCallBack, this);
  cloud_sub_ = nh_->subscribe("input_cloud", 100, &PeopleTracker::cloudCallback, this);

  ROS_INFO("Ready to receive service calls");
  ros::spin();
}

void PeopleTracker::legCallBack(const people_msgs::People &msg)
{
  if (!adapt_to_height_enabled_)
  {
    return;
  }
  boost::mutex::scoped_lock lock(this->mutex_);
  last_leg_observation_.clear();
  for (size_t i = 0; msg.people.size(); ++i)
  {
    ROS_INFO("possible person at %.3f %.3f", msg.people[i].position.x, msg.people[i].position.y);
    // only check the people that are in front of the robot and 0.3 meters to
    // the left or the right from the hokuyo_link
    if (abs(msg.people[i].position.y) < 0.3)
    {
      last_leg_observation_.push_back(msg.people[i]);
    }
  }
}
void PeopleTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  ROS_INFO("inside PeopleTracker::cloudCallback");

  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;

  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 *cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pass.setInputCloud(cloudPtr);

  // These are not in the same coordinate system as the pointcloud
  float x = 0.0;
  float z = 0.0;
  for (size_t i = 0; last_leg_observation_.size(); ++i)
  {
    // get the distance to the observed leg
    // These are not in the same coordinate system as the pointcloud
    x = last_leg_observation_[i].position.y;
    z = last_leg_observation_[i].position.x + 0.2;  // add 0.2 meters to the legs
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, z);
    pass.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered, *temp_cloud);
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*temp_cloud, min_p, max_p);
    ROS_INFO("heighest point at %.3f %.3f, %.3f", max_p.x, max_p.y, max_p.z);

    /* TODO
      call ViewController service to look at point
    */
  }
}

void PeopleTracker::update_people_in_db()
{
  /* TODO
    Create the object
    Add the object to the mongodb
  */
}

bool PeopleTracker::set_face_detection(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  this->face_detection_enabled_ = req.data;
  resp.success = true;
  return true;
}

bool PeopleTracker::set_adapt_to_height(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  this->face_detection_enabled_ = req.data;
  resp.success = true;
  return true;
}

bool PeopleTracker::set_skeleton_tracking(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  this->skeleton_tracking_enabled_ = req.data;
  resp.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_people_tracker");

  PeopleTracker people_tracker;
  people_tracker.initialize(argc, argv);

  return 0;
}
