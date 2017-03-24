#include <sstream>
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
#include <squirrel_expression/ExpressionServer.h>
#include <squirrel_hri_msgs/Expression.h>

namespace SQUIRREL_expression
{
  ExpressionServer::ExpressionServer(ros::NodeHandle &nh)
  {
    if(!nh.getParam("/squirrel_expression/sound_directory", sound_dir))
    {
      ROS_ERROR("You have to specify 'sound_directory', where all sound samples are stored.");
      exit(1);
    }

    sound_files[squirrel_hri_msgs::Expression::GREETING_IN] = "greeting_in-01.wav";
    sound_files[squirrel_hri_msgs::Expression::GREETING_OUT] = "greeting_out-01.wav";
    sound_files[squirrel_hri_msgs::Expression::OK] = "ok.wav";
    sound_files[squirrel_hri_msgs::Expression::YES] = "yes.wav";
    sound_files[squirrel_hri_msgs::Expression::NO] = "no+sound-01.wav";
    sound_files[squirrel_hri_msgs::Expression::CHEERING_POSITIVE] = "cheering_positive-01.wav";
    sound_files[squirrel_hri_msgs::Expression::OH_NO] = "nono.wav";
    sound_files[squirrel_hri_msgs::Expression::CONFUSED] = "confused-02.wav";
    sound_files[squirrel_hri_msgs::Expression::SURPRISED] = "surprised.wav";
    sound_files[squirrel_hri_msgs::Expression::ASKING] = "asking-01.wav";
    sound_files[squirrel_hri_msgs::Expression::NEED_TO_THINK_HARDER] = "need_to_think_harder.wav";
    sound_files[squirrel_hri_msgs::Expression::HERE_HERE] = "herehere.wav";
    sound_files[squirrel_hri_msgs::Expression::OUCH] = "ouch.wav";

    // for these expressions there is no associated sound
    sound_files[squirrel_hri_msgs::Expression::ANTAGONIST] = "";
    sound_files[squirrel_hri_msgs::Expression::REFERENTIAL_INFO] = "";
    sound_files[squirrel_hri_msgs::Expression::HANDOVER_OBJECT_DOWN] = "";
    sound_files[squirrel_hri_msgs::Expression::HAND_TO_HAND] = "";
    sound_files[squirrel_hri_msgs::Expression::PUSHING] = "";

    // HACK: sound play has problems on the Robotino, so just directly use aplay
    // This should be fixed at some point.
    // sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 10, true);
  }

  void ExpressionServer::performExpression(const std_msgs::String::ConstPtr& msg)
  {
    // perform expression
    ROS_INFO("Expressions: make expression '%s'", msg->data.c_str());
    std::string filename = sound_files[msg->data];
    if(!filename.empty())
    {
      std::stringstream path;
      path << sound_dir << "/" << filename;
      /*sound_play::SoundRequest sound_msg;
      sound_msg.sound = -2; // play file
      sound_msg.command = 1; // play once
      sound_msg.arg = path.str();
      sound_pub.publish(sound_msg);*/
      // HACK: sound play has problems on the Robotino, so just directly use aplay
      // This should be fixed at some point.
      std::stringstream cmd;
      cmd << "aplay -D sysdefault " << path.str();
      system(cmd.str().c_str());
    }
    else
    {
      ROS_ERROR("unknown expression '%s'", msg->data.c_str());
    }
  }

  void ExpressionServer::performNod()
  {
    // nod the head
    ros::Rate nodRate(2.5);
    std_msgs::Float64 ht;
    for(int i=0;i<3;i++)
    {
      ht.data = 0.5;
      head_tilt_pub.publish(ht);
      nodRate.sleep();
      ht.data = 0.0;
      head_tilt_pub.publish(ht);
      nodRate.sleep();
    }
    nodRate.sleep();
    ht.data = 0.6;
    head_tilt_pub.publish(ht);
  }

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "squirrel_expression");
  ros::NodeHandle nh;

  // listen for expression commands
  SQUIRREL_expression::ExpressionServer es(nh);
  ros::Subscriber expressionSub = nh.subscribe("/expression", 1, &SQUIRREL_expression::ExpressionServer::performExpression, &es);

  ROS_INFO("Expressions: Ready to receive");

  ros::spin();
  return 0;
}
