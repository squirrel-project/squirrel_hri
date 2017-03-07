/**
 * FollowPerson.cpp
 *
 * Attends to the legs in the laser data and faces in the camera data.
 *
 *
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @author Markus Bajones bajones@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <math.h>
#include <ios>
#include <robotino_msgs/LookAtPosition.h>
#include <squirrel_people_tracker/follow_child.h>
#include <people_msgs/People.h>
#include <tf/LinearMath/Transform.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "squirrel_follow_person");
  FollowChild f;
  ros::spin();

  return 0;
}
