# spt_helper
Helper package for the squirrel_person_tracker. This package visualize the user skeleton and the pointing targets.

#### Nodes

- `spt_skeleton`: Visualize the skeleton in rviz as lines between joints using the /tf topic.

- `spt_user_info`: Visualize the skeleton of all users in the field of view and the pointing point if a pointing gesture is performed.


#### Subscribed topics
- `/squirrel_person_tracker/user_information`: [squirrel_person_tracker_msgs/SkeletonVector]. Skeleton user information. 

#### Published topics
- `/skeleton_lines`: [visualization_msgs/Marker]. Visualize the skeleton as marker lines. 
- `/pointing_point`: [visualization_msgs/Marker]. Visualize the pointing point, when one or several users perform a pointing gesture.