#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    distance_ = 0.8;
    ac_ = new MoveBaseClient("move_base", true);
    while(!ac_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/published_topic", 1);
    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    //Topic you want to subscribe
    sub_ = n_.subscribe("people_tracker_measurements", 1000, &SubscribeAndPublish::personCB, this);
    last_goal_.position.x = 1000.0;
    last_goal_.position.y = 1000.0;

  }
  ~SubscribeAndPublish()
  {
    if (ac_)
      delete ac_;
  }

  void personCB(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped robot_pose, child_pose, tmp_pose, out_pose;
    move_base_msgs::MoveBaseGoal move_base_goal_;
    
    if (msg->people.size() == 0)
      return;
    
    int index = 0;
    double alpha = 0.0;
    tmp_pose.header.stamp = ros::Time(0);
    tmp_pose.header.frame_id = "hokuyo_link";
    tmp_pose.pose.position.x = msg->people[index].pos.x;
    tmp_pose.pose.position.y = msg->people[index].pos.y;
    tmp_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(alpha);
    ROS_INFO("Person detected at (x, y): (%f, %f) hokuyo_link", tmp_pose.pose.position.x, tmp_pose.pose.position.y);

// calculate a point between the child and the robot
    alpha = atan2(tmp_pose.pose.position.y, tmp_pose.pose.position.x);
    double k = sqrt(tmp_pose.pose.position.x * tmp_pose.pose.position.x + tmp_pose.pose.position.y * tmp_pose.pose.position.y);
    
    tmp_pose.header.stamp = ros::Time(0);
    tmp_pose.header.frame_id = "hokuyo_link";
    tmp_pose.pose.position.x = (k - distance_)*cos(alpha);
    tmp_pose.pose.position.y = (k - distance_)*sin(alpha);
    tmp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(alpha);
    ROS_INFO("x: %f", tmp_pose.pose.position.x);
    ROS_INFO("y: %f", tmp_pose.pose.position.y);
    ROS_INFO("Setting nav goal to (x, y): (%f, %f) hokuyo_link", tmp_pose.pose.position.x, tmp_pose.pose.position.y);
    
    try{
      ros::Time now = ros::Time(0);
      tfl_.waitForTransform("hokuyo_link", "map",
                              now, ros::Duration(3.0));
      tfl_.transformPose("map", tmp_pose, out_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time(0);
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = out_pose.pose.position.x;
    marker.pose.position.y = out_pose.pose.position.y;
    marker.pose.position.z = out_pose.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0; 
    vis_pub_.publish(marker);
    ros::Duration(2.0).sleep();

    if ((last_goal_.position.x - out_pose.pose.position.x) < 0.25 && 
        (last_goal_.position.y - out_pose.pose.position.y) < 0.25 )
    {
      ROS_INFO("current goal and last goal are close to each other. Do not send new goal");
      return;
    }
    ROS_INFO("Setting nav goal to (x, y): (%f, %f)", out_pose.pose.position.x, out_pose.pose.position.y);

    last_goal_.position = out_pose.pose.position;
    last_goal_.orientation = out_pose.pose.orientation;

    move_base_goal_.target_pose.header.frame_id = out_pose.header.frame_id;
    move_base_goal_.target_pose.header.stamp = out_pose.header.stamp;
    move_base_goal_.target_pose.pose.position.x = out_pose.pose.position.x;
    move_base_goal_.target_pose.pose.position.y = out_pose.pose.position.y;
    move_base_goal_.target_pose.pose.orientation = out_pose.pose.orientation;

    ROS_INFO("Sending goal");
    //this->ac_->sendGoal(move_base_goal_);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Publisher vis_pub_;
  geometry_msgs::Pose last_goal_;
 
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;
  tf::TransformListener tfl_;
  double distance_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  ros::init(argc, argv, "child_follow");
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();
  return 0;
}
