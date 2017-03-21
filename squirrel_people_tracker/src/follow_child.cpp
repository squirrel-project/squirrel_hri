#include "squirrel_people_tracker/follow_child.h"
#include <string>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ChildFollowingAction::~ChildFollowingAction(void)
{
  if (move_base_ac_)
    delete move_base_ac_;
}

ChildFollowingAction::ChildFollowingAction(std::string name) : as_(nh_, name, false), action_name_(name)
{
  move_base_ac_ = new MoveBaseClient("move_base", true);
  while (!move_base_ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  distance_ = 0.4;
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&ChildFollowingAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ChildFollowingAction::preemptCB, this));

  // subscribe to the data topic of interest
  sub_ = nh_.subscribe("people_tracker_measurements", 1, &ChildFollowingAction::analysisCB, this);
  as_.start();
}

void ChildFollowingAction::goalCB()
{
  goal_ = as_.acceptNewGoal();
  //nh_.createTimer(ros::Duration(goal_->time_standing_still), &ChildFollowingAction::timerCB, this);
}

void ChildFollowingAction::timerCB(const ros::TimerEvent &event)
{
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
  ROS_INFO("Data received. processing ...");
  // make sure that the action hasn't been canceled
  if (!as_.isActive())
  {
    ROS_INFO("Action server is no longer active. Exiting.");
    return;
  }

  int index = 0;
  double dist = 10.0;
  for (size_t i = 0; i < msg->people.size(); i++)
  {
    double k = 10.0;
    k = (sqrt(msg->people[i].pos.x*msg->people[i].pos.x + msg->people[i].pos.y*msg->people[i].pos.y));
    ROS_INFO("k: %f, fabs(k): %f", k, fabs(k) );
    if (k < dist)
    {
      index = i;
      dist = k;
    }
  }
  ROS_INFO("smallest distance is: %f from index %d", dist, index);

  // calculate a point between the child and the robot
  // (x, y) = (c_x +- d/sqrt(2), c_y +- d/sqrt(2))
  point_.header.stamp = ros::Time::now();
  point_.pose.position.x = msg->people[index].pos.x - 1.0 / sqrt(2);
  point_.pose.position.y = msg->people[index].pos.y - 1.0 / sqrt(2);
  ROS_INFO("Setting nav goal to (x, y): (%f, %f)", point_.pose.position.x, point_.pose.position.y);

  // store last position in result_
  result_.final_location.header.stamp = point_.header.stamp;
  result_.final_location.pose.position.x = point_.pose.position.x;
  result_.final_location.pose.position.y = point_.pose.position.y;

  move_base_goal_.target_pose.header.frame_id = "hokuyo_link";
  move_base_goal_.target_pose.header.stamp = ros::Time::now();

  move_base_goal_.target_pose.pose.position.x = point_.pose.position.x;
  move_base_goal_.target_pose.pose.position.y = point_.pose.position.y;
  move_base_goal_.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  move_base_ac_->sendGoal(move_base_goal_);
  ros::Duration(1.5).sleep();

  ROS_INFO("%lu",  goal_->target_locations.size());
  for (size_t j = 0; j < goal_->target_locations.size(); j++)
  {
    double distance = 0.001;
    double distx = 0.01;
    double disty = 0.01;
    distx = pow((goal_->target_locations[j].x - msg->people[index].pos.x), 2);
    disty = pow((goal_->target_locations[j].y - msg->people[index].pos.y), 2);
    distance = (sqrt(distx + disty));
    ROS_INFO("Distance to target position is %f", distance);
    if (fabs(distance) < distance_)
    {
      ROS_INFO("Set success");
      // set the action state to succeeded
      as_.setSucceeded(result_);
      return;
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
