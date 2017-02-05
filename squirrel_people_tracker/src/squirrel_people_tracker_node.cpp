#include "squirrel_people_tracker/squirrel_people_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_people_tracker");

  PeopleTracker people_tracker;
  people_tracker.initialize(argc, argv);

  return 0;
}
