#include "squirrel_people_tracker/follow_person.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "squirrel_follow_person");
  FollowPerson f;
  ros::spin();

  return 0;
}
