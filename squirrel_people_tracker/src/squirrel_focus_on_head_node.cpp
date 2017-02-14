#include "squirrel_people_tracker/focus_head.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_focus_head");

  FocusHead focus_on_head;
  focus_on_head.initialize(argc, argv);
  
  ros::spin();

  return 0;
}
