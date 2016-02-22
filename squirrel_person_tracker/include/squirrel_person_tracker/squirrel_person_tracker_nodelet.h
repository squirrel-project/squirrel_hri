#include <nodelet/nodelet.h>
#include "squirrel_person_tracker/squirrel_person_tracker.h"
#include <squirrel_person_tracker_msgs/SetBool.h>
#include <ros/ros.h>


class SquirrelPersonTrackerNodelet : public nodelet::Nodelet
{
public:
  SquirrelPersonTrackerNodelet();
  virtual ~SquirrelPersonTrackerNodelet();
  virtual void onInit();
  bool activate(squirrel_person_tracker_msgs::SetBool::Request& req, squirrel_person_tracker_msgs::SetBool::Response& res);
private:
  SquirrelTracker *listener;
};

