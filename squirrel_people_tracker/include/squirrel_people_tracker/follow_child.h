/**
 * FollowPerson.h
 *
 * Attends to the legs in the laser data and faces in the camera data.
 *
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef FOLLOW_CHILD_H
#define FOLLOW_CHILD_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <squirrel_hri_msgs/FollowChildAction.h>

class ChildFollowingAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<squirrel_hri_msgs::ChildFollowingAction> as_;  // NodeHandle instance must be
                                                                               // created before this line.
                                                                               // Otherwise strange error
                                                                               // occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_hri_msgs::FollowChildFeedback feedback_;
  squirrel_hri_msgs::FollowChildResult result_;
  squirrel_hri_msgs::FollowChildGoal goal_;
  geometry_msgs::PoseStamped point_;
  double distance_;
  double target_distance_;

public:
  tf::StampedTransform transform;
  tf::TransformListener listener;

  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;
  ros::Publisher pan_pub_;
  ros::Publisher tilt_pub_;
  ros::Timer timer;

  void goalCB();
  void preemptCB();
  void analysisCB(const people_msgs::PositionMeasurementArray::ConstPtr &msg);
  void timerCB(const ros::TimerEvent &event);

  ChildFollowingAction(std::string name);
  ~ChildFollowingAction();
};

#endif  // FOLLOW_CHILD_H
