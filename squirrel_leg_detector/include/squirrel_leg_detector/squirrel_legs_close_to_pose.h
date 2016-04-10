/**
 * leg_detector.h
 *
 * 
 * @author Markus "Bajo" Bajones bajones@acin.tuwien.ac.at
 * @date April 2016
 */

#ifndef LEG_PROXIMITY_H
#define LEG_PROXIMITY_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <people_msgs/People.h>

class LegProximity
{
public:
  LegProximity();
  virtual ~LegProximity();
  void initialise(int argc, char **argv);
  void run();

private:
  ros::NodeHandle nh_;
  ros::Subscriber peopleSub_;
  ros::Publisher proximityPub_;
  geometry_msgs::Pose target_pose_;
  double threshold_;

  void legCallback(const people_msgs::People::ConstPtr& peopleMsg);
};

#endif
