#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <squirrel_person_tracker/squirrel_person_tracker_nodelet.h>

PLUGINLIB_EXPORT_CLASS(SquirrelPersonTrackerNodelet, nodelet::Nodelet)

SquirrelPersonTrackerNodelet::SquirrelPersonTrackerNodelet() :
    Nodelet(), listener(NULL)
{

}

SquirrelPersonTrackerNodelet::~SquirrelPersonTrackerNodelet()
{
  if (listener)
  {
    delete listener;
    listener = NULL;
  }
}

void SquirrelPersonTrackerNodelet::onInit()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  bool is_srv_used;
  if (!nh.getParam("is_srv_used", is_srv_used))
  {
    is_srv_used = false;
    ROS_INFO("is_srv_used is not set. The person tracker will run without the service option");
  }
  else if (is_srv_used)
  {
    ROS_INFO("The person tracker will runs with the service option");
  }
  else
  {
    ROS_INFO("The person tracker will runs without the service option");
  }

  if (!listener)
  {
    listener = new SquirrelTracker(getPrivateNodeHandle());
  }

  if (!is_srv_used)
  {
    NODELET_INFO("Frame: %s", listener->frame_id.c_str());

    listener->runSquirrelTracker();
  }
  else
  {
    ros::ServiceServer service = nh.advertiseService("activate", &SquirrelPersonTrackerNodelet::activate, this);
    ROS_INFO("Squirrel person tracker is ready to be activated.");
    ros::spin();
  }
}

bool SquirrelPersonTrackerNodelet::activate(squirrel_person_tracker_msgs::SetBool::Request& req,
                                            squirrel_person_tracker_msgs::SetBool::Response& res)
{
  if (req.track)
  {
    NODELET_INFO("Frame: %s", listener->frame_id.c_str());
    NODELET_INFO("Squirrel person tracker is activate.");
    listener->runSquirrelTracker();
    res.success = true;
    res.message = "Tracking is activated";
  }
  else
  {
    res.success = false;
    res.message = "Tracking is deactivated";
    listener->stopSquirrelTracker();
    NODELET_INFO("Squirrel person tracker is not activate.");
  }

  return true;
}
