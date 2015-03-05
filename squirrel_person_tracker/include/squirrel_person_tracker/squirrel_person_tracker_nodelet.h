#include <nodelet/nodelet.h>
#include "squirrel_person_tracker/squirrel_person_tracker.h"

class SquirrelPersonTrackerNodelet : public nodelet::Nodelet
{
public:
  SquirrelPersonTrackerNodelet();
  virtual ~SquirrelPersonTrackerNodelet();
  virtual void onInit();
private:
  SquirrelTracker *listener;
};

