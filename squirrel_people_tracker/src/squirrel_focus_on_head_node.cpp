#include "squirrel_people_tracker/focus_head.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "squirrel_focus_on_head");
  FocusHead fh;
  ros::spin();

  return 0;
}
