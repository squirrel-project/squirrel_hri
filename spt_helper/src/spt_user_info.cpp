#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <squirrel_person_tracker_msgs/SkeletonVector.h>

geometry_msgs::Point gtransformToGpoint(const geometry_msgs::TransformStamped& transform,
                                        geometry_msgs::Point& tmpPoint)
{
  tmpPoint.x = transform.transform.translation.x;
  tmpPoint.y = transform.transform.translation.y;
  tmpPoint.z = transform.transform.translation.z;
  return tmpPoint;
}

void createLineSkel(const squirrel_person_tracker_msgs::Skeleton& skel, visualization_msgs::Marker& line_list)
{

  geometry_msgs::Point tmpPoint;
  std::vector<geometry_msgs::Point> joint_points;
  geometry_msgs::TransformStamped transform;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  for (int i = 0; i < skel.skeleton_joints.size(); ++i)
  {
    joint_points.push_back(gtransformToGpoint(skel.skeleton_joints[i].joint, tmpPoint));
  }

  if (!joint_points.empty())
  {
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
  }
}

void skelVecCB(const squirrel_person_tracker_msgs::SkeletonVector::ConstPtr skel_vec, std::string frame_id,
               const ros::Publisher& marker_pub, const ros::Publisher& point_pub)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time(0);
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.05;

  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = frame_id;
  sphere_list.header.stamp = ros::Time(0);
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;
  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::POINTS;
  sphere_list.scale.x = 0.15;
  sphere_list.scale.y = 0.15;
  sphere_list.color.r = 1;
  sphere_list.color.g = 0;
  sphere_list.color.b = 0;
  sphere_list.color.a = 1;

  for (int i = 0; i < skel_vec->skeleton_vector.size(); ++i)
  {
    createLineSkel(skel_vec->skeleton_vector[i], line_list);
    if (skel_vec->skeleton_vector[i].isPointing)
    {
      sphere_list.header = skel_vec->skeleton_vector[i].pointingPoint.header;
      sphere_list.points.push_back(skel_vec->skeleton_vector[i].pointingPoint.point);
    }
  }
  point_pub.publish(sphere_list);
  marker_pub.publish(line_list);
  sphere_list.points.clear();
  line_list.points.clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spt_skeleton_lines");
  ros::NodeHandle n;
  std::string frame_id = "/kinect_depth_optical_frame";
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("skeleton_lines", 10);
  ros::Publisher point_pub = n.advertise<visualization_msgs::Marker>("pointing_point", 10);
  ros::Subscriber subSkelVec = n.subscribe<squirrel_person_tracker_msgs::SkeletonVector>(
      "/squirrel_person_tracker/user_information", 1, boost::bind(&skelVecCB, _1, frame_id, marker_pub, point_pub));
  ros::spin();
}

