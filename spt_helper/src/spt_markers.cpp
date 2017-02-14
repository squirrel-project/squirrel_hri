#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

geometry_msgs::Pose spt_PointPose;

void spt_callback(const geometry_msgs::PoseStamped& point)
{
  spt_PointPose = point.pose;
}

float euclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
  float distance;
  distance = sqrt(pow((pose2.position.x - pose1.position.x), 2) + pow((pose2.position.y - pose1.position.y), 2));
  return distance;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spt_markers");
  ros::NodeHandle n;
  //ros::Rate r(30);
  spt_PointPose.position.x = 50;
  spt_PointPose.orientation.w = 1;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 0.0f;
  color.r = 1.0f;
  color.g = 0.0f;


  visualization_msgs::Marker markerCube;
  visualization_msgs::Marker markerSphere;
  visualization_msgs::Marker markerCylinder;

  markerCube.header.frame_id = "camera_depth_frame";
  markerSphere.header.frame_id = "camera_depth_frame";
  markerCylinder.header.frame_id = "camera_depth_frame";
  markerCube.ns = markerSphere.ns = markerCylinder.ns = "basic_shapes";
  markerCube.id = 0;
  markerSphere.id = 1;
  markerCylinder.id = 2;

  markerCube.type = visualization_msgs::Marker::CUBE;
  markerSphere.type = visualization_msgs::Marker::SPHERE;
  markerCylinder.type = visualization_msgs::Marker::CYLINDER;

  markerCube.color = color;
  markerSphere.color = color;
  markerCylinder.color = color;

  markerCube.pose.position.x = 2.43;
  markerCube.pose.position.y = 1.03;
  // markerCube.pose.position.z = -0.785;
  markerCube.pose.orientation.x = 0.0;
  markerCube.pose.orientation.y = 0.0;
  markerCube.pose.orientation.z = 0.0;
  markerCube.pose.orientation.w = 1.0;
  markerCube.scale.x = 0.20;
  markerCube.scale.y = 0.20;
  markerCube.scale.z = 0.20;

  markerSphere.pose.position.x = 1.90;
  markerSphere.pose.position.y = -0.82;
//  markerSphere.pose.position.z = 0;
  markerSphere.pose.orientation.x = 0.0;
  markerSphere.pose.orientation.y = 0.0;
  markerSphere.pose.orientation.z = 0.0;
  markerSphere.pose.orientation.w = 1.0;
  markerSphere.scale.x = 0.20;
  markerSphere.scale.y = 0.20;
  markerSphere.scale.z = 0.20;

  markerCylinder.pose.position.x = 1.40;
  markerCylinder.pose.position.y = 0.23;
//  markerCylinder.pose.position.z = 0;
  markerCylinder.pose.orientation.x = 0.0;
  markerCylinder.pose.orientation.y = 0.0;
  markerCylinder.pose.orientation.z = 0.0;
  markerCylinder.pose.orientation.w = 1.0;
  markerCylinder.scale.x = 0.20;
  markerCylinder.scale.y = 0.20;
  markerCylinder.scale.z = 0.20;

  markerCube.lifetime = ros::Duration();
  markerSphere.lifetime = ros::Duration();
  markerCylinder.lifetime = ros::Duration();
//
//  ENDE
//
  ros::Subscriber sub = n.subscribe("/squirrel_person_tracker/pointing_pose", 10, spt_callback);

  while (ros::ok())
  {

    markerCube.header.stamp = markerSphere.header.stamp = markerCylinder.header.stamp = ros::Time::now();

    markerCube.action = visualization_msgs::Marker::ADD;
    markerSphere.action = visualization_msgs::Marker::ADD;
    markerCylinder.action = visualization_msgs::Marker::ADD;

    visualization_msgs::MarkerArray markerArray;

    markerCylinder.pose.position.z = markerCube.pose.position.z = markerSphere.pose.position.z = spt_PointPose.position.z+0.1;

    markerArray.markers.reserve(3);
    markerArray.markers.push_back(markerCube);
    markerArray.markers.push_back(markerCylinder);
    markerArray.markers.push_back(markerSphere);

    if (euclideanDistance(spt_PointPose, markerCube.pose) < 0.3)
    {
      markerCube.color.r = 0.0f;
      markerCube.color.g = 1.0f;
    }
    else
    {
      markerCube.color.r = 1.0f;
      markerCube.color.g = 0.0f;
    }

    if (euclideanDistance(spt_PointPose, markerSphere.pose) < 0.3)
    {
      markerSphere.color.r = 0.0f;
      markerSphere.color.g = 1.0f;
    }
    else
    {
      markerSphere.color.r = 1.0f;
      markerSphere.color.g = 0.0f;
    }

    if (euclideanDistance(spt_PointPose, markerCylinder.pose) < 0.3)
    {
      markerCylinder.color.r = 0.0f;
      markerCylinder.color.g = 1.0f;
    }
    else
    {
      markerCylinder.color.r = 1.0f;
      markerCylinder.color.g = 0.0f;
    }

    marker_pub.publish(markerArray);
   // r.sleep();
    ros::spinOnce();
  }

}
