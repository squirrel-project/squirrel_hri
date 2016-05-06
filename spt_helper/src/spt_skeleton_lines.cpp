#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

geometry_msgs::Point transformToGpoint(const std::string& parent_frame, const std::string& child_frame,
                                       const tf::TransformListener& listener, tf::StampedTransform& transform,
                                       geometry_msgs::Point& tmpPoint)
{
  if (listener.canTransform(parent_frame, child_frame, ros::Time(0)))
  {
    try
    {

      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
    }
    catch (const tf::LookupException& e)
    {
      ROS_ERROR("Could not lookup transform %s -> %s: %s", parent_frame.c_str(), child_frame.c_str(), e.what());
      return tmpPoint;
    }
    tmpPoint.x = transform.getOrigin().x();
    tmpPoint.y = transform.getOrigin().y();
    tmpPoint.z = transform.getOrigin().z();
    return tmpPoint;
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "spt_skeleton_lines");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("skeleton_lines", 10);
  std::string frame_id = "/kinect_depth_optical_frame";

  ros::Rate r(30);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::vector<geometry_msgs::Point> joint_points;
  geometry_msgs::Point tmpPoint;

  while (ros::ok())
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time(0);
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;

    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.05;

    // Line list is red
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    joint_points.push_back(transformToGpoint(frame_id, "head", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "neck", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "torso", listener, transform, tmpPoint));

    joint_points.push_back(transformToGpoint(frame_id, "left_shoulder", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "left_elbow", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "left_hand", listener, transform, tmpPoint));

    joint_points.push_back(transformToGpoint(frame_id, "right_shoulder", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "right_elbow", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "right_hand", listener, transform, tmpPoint));

    joint_points.push_back(transformToGpoint(frame_id, "left_hip", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "left_knee", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "left_foot", listener, transform, tmpPoint));

    joint_points.push_back(transformToGpoint(frame_id, "right_hip", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "right_knee", listener, transform, tmpPoint));
    joint_points.push_back(transformToGpoint(frame_id, "right_foot", listener, transform, tmpPoint));

    line_list.points.push_back(joint_points[0]);
    line_list.points.push_back(joint_points[1]);

    line_list.points.push_back(joint_points[1]);
    line_list.points.push_back(joint_points[2]);

    line_list.points.push_back(joint_points[1]);
    line_list.points.push_back(joint_points[3]);

    line_list.points.push_back(joint_points[3]);
    line_list.points.push_back(joint_points[4]);

    line_list.points.push_back(joint_points[4]);
    line_list.points.push_back(joint_points[5]);

    line_list.points.push_back(joint_points[1]);
    line_list.points.push_back(joint_points[6]);

    line_list.points.push_back(joint_points[6]);
    line_list.points.push_back(joint_points[7]);

    line_list.points.push_back(joint_points[7]);
    line_list.points.push_back(joint_points[8]);

    line_list.points.push_back(joint_points[2]);
    line_list.points.push_back(joint_points[9]);

    line_list.points.push_back(joint_points[9]);
    line_list.points.push_back(joint_points[10]);

    line_list.points.push_back(joint_points[10]);
    line_list.points.push_back(joint_points[11]);

    line_list.points.push_back(joint_points[2]);
    line_list.points.push_back(joint_points[12]);

    line_list.points.push_back(joint_points[12]);
    line_list.points.push_back(joint_points[13]);

    line_list.points.push_back(joint_points[13]);
    line_list.points.push_back(joint_points[14]);

    marker_pub.publish(line_list);

    line_list.points.clear();
    joint_points.clear();

    r.sleep();

  }
}

