#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <squirrel_speech_msgs/RecognizedSpeech.h>
#include <string>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void display_text_in_rviz(std::string speech, bool rec)
{
    marker.header.stamp = ros::Time::now();

    marker.action = visualization_msgs::Marker::ADD;


    if(rec)
    {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    }
    else
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    }

    marker.text = speech;
    marker_pub.publish(marker);
}


void callback(const squirrel_speech_msgs::RecognizedSpeech::ConstPtr& msg)
{
  std::string speech;
  bool rec;

  speech = msg->recognized_speech;
  rec    = msg->is_recognized;
    
  //ROS_INFO("%s",speech.c_str());

  display_text_in_rviz(speech, rec);
}

// ==================================================================================


int main( int argc, char** argv )
{
  ros::init(argc, argv, "speech_visu");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("speech_visualization", 1);


  marker.header.frame_id = "/base_link";
  marker.ns = "speech_visu";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;    

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 1.8;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
      
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.lifetime = ros::Duration(3.0);

  ros::Subscriber sub = n.subscribe("squirrel_speech_recognized_speech", 1, callback);

  ros::spin();

  return 0;
}


