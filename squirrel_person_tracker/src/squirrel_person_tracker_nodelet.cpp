#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <squirrel_person_tracker/squirrel_person_tracker_nodelet.h>
//PLUGINLIB_DECLARE_CLASS(squirrel_tracker, squirrel_tracker_nodelet, squirrel_tracker_nodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(SquirrelPersonTrackerNodelet, nodelet::Nodelet)

SquirrelPersonTrackerNodelet::SquirrelPersonTrackerNodelet() : Nodelet(), listener(NULL) {

}

SquirrelPersonTrackerNodelet::~SquirrelPersonTrackerNodelet() {
  if (listener) {
    delete listener;
    listener = NULL;
  }
}

void SquirrelPersonTrackerNodelet::onInit()
{
  if (!listener) {
    listener = new SquirrelTracker(getPrivateNodeHandle());
  }

  NODELET_INFO("Frame: %s", listener->frame_id.c_str());

  listener->runSquirrelTracker();

}
