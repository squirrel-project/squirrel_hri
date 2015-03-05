General Description:

PARAMETERS:

name="camera_frame_id" type="str" value="camera_depth_frame"

#############################################################
PUBLISHED TOPICS:


name="detected_pose": Publish geometry_msgs::PoseArray messages representing the center of mass of the users.

name="filtered_cloud": Publish sensor_msgs::PointCloud2 messages. The filtered cloud is the original point cloud minus the points corresponding to users and measurement errors.

name="pointing_pose": Publish geometry_msgs::PoseStamped messages. The XYZ information contains the position in space the user is pointing to represented in the coordinate system of the depth sensor.

name="head_hand_points": Publish squirrel_person_tracker::HeadHandPoints messages. The points contain the position in space of the head and hand joint represented in the coordinate system of the depth sensor.

name="tracking_state": This message holds the description of the state of progress while tracking.
uint8 NO_USER          = 1-> No users in the field of view of the sensor.
uint8 VISIBLE_USER     = 2-> Users are detected in the field of view of the sensor.
uint8 WAVE_USER        = 3-> A wave gesture is detected by single user. Next, try to track the skeleton joints of this user, PSI Pose necessary.
uint8 SKEL_TRACK_USER  = 4-> The skeleton joints of the waving user are being tracked.
uint8 POINT_USER       = 5-> Pointing gesture of the user is detected by squirrel_tracker.

#############################################################
SUBSCRIBED TOPICS:
---

#############################################################


HOW TO LAUNCH:

One may launch the squirrel_tracker as a standalone package with:

"roslaunch squirrel_tracker squirrel_tracker.launch" .

In order to launch the squirrel_tracker and the openni2_launch package simultaneously, use the follwing approach:

 roslaunch squirrel_tracker squirrel_tracker_nodelet.launch

##############################################################
############ squirrel_tracker_nodelet.launch #################

<launch>

 <include file="$(find openni2_launch)/launch/openni2.launch" />

 <node pkg="nodelet" type="nodelet" name="squirrel_tracker" args="load squirrel_tracker/squirrel_tracker_nodelet /camera/camera_nodelet_manager" output="screen" cwd="ROS_HOME">
<param name="camera_frame_id" type="str" value="camera_depth_frame" />
 </node>  
           
</launch>
##############################################################
##############################################################






