#include <string>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#ifndef SQUIRREL_expression
#define SQUIRREL_expression

/**
 * This file defines the ExpressionServer class.
 * The expression server listens for expression commands,
 * and makes the robotino dance.
 */
namespace SQUIRREL_expression {

	class ExpressionServer
	{

	private:

		// topics
		ros::Publisher head_tilt_pub;
		ros::Publisher sound_pub;
		
		std::string sound_dir;
		std::map<std::string, std::string> sound_files;

		// expressions
		void performNod();

	public:

		/* constructor */
		ExpressionServer(ros::NodeHandle &nh);

		/* expression command callback method*/
		void performExpression(const std_msgs::String::ConstPtr& msg);
	};
}
#endif
