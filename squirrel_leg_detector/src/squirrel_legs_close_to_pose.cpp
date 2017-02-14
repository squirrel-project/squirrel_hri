/**
 * leg_detector.h
 *
 * Detects persons as pairs of legs in 2D laser range data.
 *
 * @author Markus "Bajo" Bajones bajones@acin.tuwien.ac.at
 * @date April 2016
 */

#include <stdlib.h>
#include <stdio.h>

#include <cmath>   

#include <std_msgs/Bool.h>
#include <squirrel_leg_detector/squirrel_legs_close_to_pose.h>
#include <squirrel_speech_msgs/RecognizedCommand.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <people_msgs/People.h>
#include <tf/transform_listener.h>

LegProximity::LegProximity()
{
  target_point_;
  target_point_.point.x = 0.0;
  target_point_.point.y = 0.0;
}

LegProximity::~LegProximity()
{
}

void LegProximity::initialise(int argc, char **argv)
{
  nh_.param("threshold", threshold_, 0.3);

  std::vector<double> points_list;
  double sum = 0;
  nh_.getParam("target_pose", points_list);
  target_point_.point.x = points_list[0];
  target_point_.point.y = points_list[1];

  peopleSub_ = nh_.subscribe("/leg_persons", 1, &LegProximity::legCallback, this);
  proximityPub_ = nh_.advertise<squirrel_speech_msgs::RecognizedCommand>("/squirrel_speech_rec/squirrel_speech_recognized_commands", 1);;
}


void LegProximity::legCallback(const people_msgs::People::ConstPtr& peopleMsg)
{
  std::cout << peopleMsg->header << std::endl;
  for(unsigned i=0; i < peopleMsg->people.size(); i++)
  {
    geometry_msgs::PointStamped in_point;
    in_point.header = peopleMsg->header;
    in_point.point.x = peopleMsg->people[i].position.x;
    in_point.point.y = peopleMsg->people[i].position.y;
    geometry_msgs::PointStamped out_point;
    try{
      listener.transformPoint("/map", in_point.header.stamp, in_point, in_point.header.frame_id, out_point);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    
    // check for every detected person if they are within the threshold of the target pose
    if (std::abs(out_point.point.x - target_point_.point.x ) < threshold_  &&
       std::abs(out_point.point.y - target_point_.point.y ) < threshold_ ) 
    {
      ROS_INFO("squirrel_legs_close_to_pose_node: Somebody is close to the target zone");
      ROS_INFO("person is at x: %f, y: %f", out_point.point.x, out_point.point.y);
      // Now publish this information
      squirrel_speech_msgs::RecognizedCommand message;
      message.header.stamp = ros::Time::now();
      message.int_command = "gehe";
      message.is_command = true;
      proximityPub_.publish(message);
    }
    else
    {
      ROS_INFO("squirrel_legs_close_to_pose_node: No person near target zone");
      ROS_INFO("target zone is at x: %f, y: %f", target_point_.point.x, target_point_.point.y);
      ROS_INFO("person is at x: %f, y: %f", out_point.point.x, out_point.point.y);
    } 
  }  
}


void LegProximity::run()
{
  ros::spin();
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "squirrel_legs_close_to_pose_node");
  LegProximity lp;
  lp.initialise(argc, argv);
  lp.run();
  exit(0);
}
