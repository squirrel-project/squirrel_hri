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
#include <people_msgs/People.h>

LegProximity::LegProximity()
{
  target_pose_;
  target_pose_.position.x = 0.0;
  target_pose_.position.y = 0.0;
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
  target_pose_.position.x = points_list[0];
  target_pose_.position.y = points_list[1];

  peopleSub_ = nh_.subscribe("/leg_persons", 1, &LegProximity::legCallback, this);
  proximityPub_ = nh_.advertise<squirrel_speech_msgs::RecognizedCommand>("/squirrel_speech_rec/squirrel_speech_recognized_commands", 1);;
}


void LegProximity::legCallback(const people_msgs::People::ConstPtr& peopleMsg)
{
  std::cout << peopleMsg->header << std::endl;
  for(unsigned i=0; i < peopleMsg->people.size(); i++)
  {
    // check for every detected person if they are within the threshold of the target pose
    if (std::abs(peopleMsg->people[i].position.x - target_pose_.position.x ) < threshold_  &&
       std::abs(peopleMsg->people[i].position.y - target_pose_.position.y ) < threshold_ ) 
     {
       ROS_INFO("squirrel_legs_close_to_pose_node: Somebody is close to the target zone");
       ROS_INFO("target zone is at x: %f, y: %f", target_pose_.position.x, target_pose_.position.y);
       std::cout << "target zone: x: " << target_pose_.position.x << ", y: " << target_pose_.position.y << std::endl;
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
