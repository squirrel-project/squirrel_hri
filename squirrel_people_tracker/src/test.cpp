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

    //Topic you want to subscribe
    sub_ = n_.subscribe("people_tracker_measurements", 1000, &SubscribeAndPublish::personCB, this);

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
    ROS_INFO("msg contains %ld data points", msg->people.size());
    if (msg->people.size() == 0)
      return;
    
    ros::Time t = ros::Time(0);
    /*
    tf::StampedTransform transform;
    tfl_.waitForTransform("hokuyo_link", "map", t, ros::Duration(1.0));
    tfl_.lookupTransform("hokuyo_link", "map", t, transform);
    */

    // get the Euler angle (radian) between 
    double alpha = 0.0;
    alpha = atan2(msg->people[index].pos.x, msg->people[index].pos.y);

    // calculate a point between the child and the robot
    // (x, y) = (c_x +- d/sqrt(2), c_y +- d/sqrt(2))
    tmp_pose.header.stamp = ros::Time(0);
    tmp_pose.header.frame_id = "hokuyo_link";
    tmp_pose.pose.position.x = msg->people[index].pos.x - distance_;
    tmp_pose.pose.position.y = msg->people[index].pos.y - sin(alpha) * distance_ ;
    tmp_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(alpha);
    
    try{
      tfl_.transformPose("map", tmp_pose, out_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    ROS_INFO("Setting nav goal to (x, y): (%f, %f)", out_pose.pose.position.x, out_pose.pose.position.y);

    move_base_goal_.target_pose.header.frame_id = out_pose.header.frame_id;
    move_base_goal_.target_pose.header.stamp = out_pose.header.stamp;
    move_base_goal_.target_pose.pose.position.x = out_pose.pose.position.x;
    move_base_goal_.target_pose.pose.position.y = out_pose.pose.position.y;
    move_base_goal_.target_pose.pose.orientation = out_pose.pose.orientation;

    ROS_INFO("Sending goal");
    this->ac_->sendGoal(move_base_goal_);
    ros::Duration(2.0).sleep();
    
    pub_.publish(out_pose);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
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
