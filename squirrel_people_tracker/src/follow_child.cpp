#include "squirrel_people_tracker/follow_child.h"

ChildFollowingAction::~ChildFollowingAction(void)
{
}

ChildFollowingAction::ChildFollowingAction(std::string name) : as_(nh_, name, false), action_name_(name)
{
  distance_ = 0.4;
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&ChildFollowingAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ChildFollowingAction::preemptCB, this));

  // subscribe to the data topic of interest
  sub_ = nh_.subscribe("", 1, &ChildFollowingAction::analysisCB, this);
  as_.start();
}

void ChildFollowingAction::goalCB()
{
  goal_ = as_.acceptNewGoal();
  // start a new ros timer to stop as soon as the defined timeout has been reached
  nh_.createTimer(ros::Duration(goal->time_standing_still), ChildFollowingAction::timerCB);
}

void ChildFollowingAction::timerCB(const ros::TimerEvent &eventconst ros::TimerEvent &event)
{
  result_.final_location = feedback_.final_location;
  as_.setAborted(result_, "Timeout reached.");
}

void ChildFollowingAction::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  as_.setPreempted();
}

void ChildFollowingAction::analysisCB(const people_msgs::PositionMeasurementArray::ConstPtr &msg)
{
  // make sure that the action hasn't been canceled
  if (!as_.isActive())
    return;

  for (size_t i = 0; i < msg->people.size(); ++i)
  {
    if (strcmp(msg->people[i].object_id, goal_->child_id)
    {
      for (size_t j = 0; i < goal_->target_locations.size(); ++j)
      {
        double distx = pow((target_locations[j].point x - msg->people[i].pos.x), 2);
        double disty = pow((target_locations[j].point y - msg->people[i].pos.y), 2);
        if ((abs(sqrt(distx - disty))) < target_distance_)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          ROS_INFO("Child is at (x,y): (%f, %f)", msg->people[i].x, msg->people[i].y);
          // set the action state to succeeded
          as_.setSucceeded(result_);
        }
      }
      // calculate a point between the child and the robot
      // (x, y) = (c_x +- d/sqrt(2), c_y +- d/sqrt(2))
      point_.header.stamp = ros::Time::now();
      point_.point.x = msg->people[i].pos.x - distance_ / sqrt(2);
      point_.point.y = msg->people[i].pos.y - distance_ / sqrt(2);

      // store last position in result_
      feedback_.final_location.header.stamp = point_.header.stamppoint_.header.stamp;
      feedback_.final_location.point.x = point_.point.x;
      feedback_.final_location.point.y = point_.point.y;

    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_child_follower");
  ChildFollowingAction follow_child(ros::this_node::getName());
  ros::spin();
  return 0;
}
