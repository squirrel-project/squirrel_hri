squirrel_person_tracker
===================

People tracking and pose estimation package.

## Nodelet 

The package provides a nodelet that uses the NiTE people tracking in the same process as the openni2_launch package. 

#### Parameters

- `/squirrel_person_tracker/camera_optical_frame_id`: camera_depth_frame.

#### Subscribed topics
- no subscription to ROS topics.

#### Published topics
- `/squirrel_person_tracker/detected_pose` : [geometry_msgs/PoseArray] Is an array of center of mass of all the users in the field of view.
- `/squirrel_person_tracker/head_hand_points`: [squirrel_person_tracker_msgs/HeadHandPoints] Is the hand and head pose of a pointing user. Only one pointing user at a time can be detected.
- `/squirrel_person_tracker/pointing_pose` : [geometry_msgs/PointStamped] Is the pointing position on the floor. Pulished when users perform a pointing gesture. 
- `/squirrel_person_tracker/tracking_state` : [squirrel_person_tracker_msgs/State] This message holds the description of the state of progress while tracking.
- `/squirrel_person_tracker/user_information` : [squirrel_person_tracker_msgs/SkeletonVector] Contains all the user information in the field of fiew of the sensor.
- `/squirrel_person_tracker/filtered_clud` : [sensor_msgs/PointCloud2 ] The filtered cloud is the original point cloud minus the points corresponding to users and measurement errors.

#### State of progress
- uint8 NO_USER          = 1-> No users in the field of view of the sensor.

- uint8 VISIBLE_USER     = 2-> Users are detected in the field of view of the sensor.

- uint8 WAVE_USER        = 3-> A wave gesture is detected by single user. Next, try to track the skeleton joints of this user, PSI Pose necessary.

- uint8 SKEL_TRACK_USER  = 4-> The skeleton joints of the waving user are being tracked.

- uint8 POINT_USER       = 5-> Pointing gesture of the user is detected by squirrel_tracker.

#### Services
- `/squirrel_person_tracker/activate`: [squirrel_person_tracker_msgs/SetBool] Is a service to activate/deactivate the squirrel_person_tracker nodelet.

### Launch file arguments 
- `camera_id`: [string] camera name.
- `is_srv_used`: [bool] If true, the squirrel_person_tracker can be activated/deactivated by /squirrel_person_tracker/activate service.
- `static_plane`: [bool] If the argument is true the static plane, in the point-normal form, is defined  by the base_link origin and the base_link z-axis. If false, the NiTe plane-detection define the plane of the floor.
- `pub_filtered_pc`: If true the nodelet will publish the filtered_cloud topic.
- `publish_skeleton`: Publish the skeleton joints from all users in the field of view, as tf topic. This topic is used for quick checks and visualization.
- `track_wave_user`: If true, the nodelet will detect the wave gesture of a user and also provide information about the state of progress.