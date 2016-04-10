/**
 * leg_detector.h
 *
 * Detects persons as pairs of legs in 2D laser range data.
 *
 * @author Markus Bajones bajones@acin.tuwien.ac.at
 * @date April 2016
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ios>
#include <std_msgs/String.h>
#include <squirrel_leg_detector/leg_detector.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

LegProximity::LegProximity()
{
  ppl2D_ = 0;
}

LegProximity::~LegProximity()
{
  delete ppl2D_;
}

void LegProximity::initialise(int argc, char **argv)
{
  sw_param_str sw_param;

  // load trained person model
  nh_.param<std::string>("model_file", sw_param.modelfile, "MODEL_FILE_NOT_SET");
  // segmentation distance, default 0.2m
  nh_.param("segmentation_distance", sw_param.dseg, 0.2);
  // feature set mix, default 0:all
  nh_.param("feature_mix", sw_param.featuremix, 0);

  ppl2D_ = new people2D_engine(sw_param);
  ppl2D_->set_featureset();

  std::vector<double> my_double_list;
double sum = 0;
nh.getParam("my_double_list", my_double_list);
for(unsigned i=0; i < my_double_list.size(); i++) {
  sum += my_double_list[i];

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "squirrel_legs_close_to_pose_node");
  LegProximity lp;
  lp.initialise(argc, argv);
  lp.run();
  exit(0);
}
