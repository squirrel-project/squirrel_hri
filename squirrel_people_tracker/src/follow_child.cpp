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
  new MoveBaseClient("move_base", true);
  while (!move_base_ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  distance_ = 0.4;
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&ChildFollowingAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ChildFollowingAction::preemptCB, this));

  // subscribe to the data topic of interest
  sub_ = nh_.subscribe("people_tracker_filter", 1, &ChildFollowingAction::analysisCB, this);
  as_.start();
}

void ChildFollowingAction::goalCB()
{
  goal_ = as_.acceptNewGoal();
  nh_.createTimer(ros::Duration(goal_->time_standing_still), &ChildFollowingAction::timerCB, this);
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
  // make sure that the action hasn't been canceled
  if (!as_.isActive())
    return;

  for (size_t i = 0; i < msg->people.size(); ++i)
  {
    if (msg->people[i].object_id == "test")
      ;  // goal_->child_id_to_follow)
    {
      for (size_t j = 0; i < goal_->target_locations.size(); ++j)
      {
        double distx = pow((goal_->target_locations[j].x - msg->people[i].pos.x), 2);
        double disty = pow((goal_->target_locations[j].y - msg->people[i].pos.y), 2);
        if ((abs(sqrt(distx - disty))) < target_distance_)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          ROS_INFO("Child is at (x,y): (%f, %f)", msg->people[i].pos.x, msg->people[i].pos.y);
          // set the action state to succeeded
          as_.setSucceeded(result_);
        }
      }
      // calculate a point between the child and the robot
      // (x, y) = (c_x +- d/sqrt(2), c_y +- d/sqrt(2))
      point_.header.stamp = ros::Time::now();
      point_.pose.position.x = msg->people[i].pos.x - distance_ / sqrt(2);
      point_.pose.position.y = msg->people[i].pos.y - distance_ / sqrt(2);
      ROS_INFO("Setting nav goal to (x, y): (%f, %f)", point_.pose.position.x, point_.pose.position.y);

      // store last position in result_
      result_.final_location.header.stamp = point_.header.stamp;
      result_.final_location.pose.position.x = point_.pose.position.x;
      result_.final_location.pose.position.y = point_.pose.position.y;

      move_base_goal_.target_pose.header.frame_id = "hokuyo_link";
      move_base_goal_.target_pose.header.stamp = ros::Time::now();

      move_base_goal_.target_pose.pose.position.x = point_.point_.pose.position.x;
      move_base_goal_.target_pose.pose.position.y = point_.point_.pose.position.y;
      move_base_goal_.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal");
      move_base_ac_->sendGoal(move_base_goal_);

      ros::Duration(1.0).sleep();
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
