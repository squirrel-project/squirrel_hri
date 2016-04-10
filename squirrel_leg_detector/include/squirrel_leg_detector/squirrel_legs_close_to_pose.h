/**
 * leg_detector.h
 *
 * 
 * @author Markus "Bajo" Bajones bajones@acin.tuwien.ac.at
 * @date April 2016
 */

#ifndef LEG_PROXIMITY_H
#define LEG_PROXIMITY_H

#include <ros/ros.h>

class LegProximity
{
public:
  LegProximity();
  virtual ~LegProximity();
  void initialise(int argc, char **argv);
  void run();

private:
  people2D_engine *ppl2D_;
  ros::NodeHandle nh_;
  ros::Subsciber peoplePub_;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
};

#endif
