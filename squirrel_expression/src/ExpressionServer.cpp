#include "ros/ros.h"
#include "squirrel_expression/ExpressionServer.h"
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

namespace SQUIRREL_expression {

	/* constructor */
	ExpressionServer::ExpressionServer(ros::NodeHandle &nh) {

		// setup publishers
		head_tilt_pub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10, true);

		// setup service servers
		// setup service clients
	}
	
	void ExpressionServer::performExpression(const std_msgs::String::ConstPtr& msg) {

		// perform expression
		ROS_INFO("Expressions: make expression %s", msg->data.c_str());
		if("ok" == msg->data) performNod();
	}

	void ExpressionServer::performNod() {
		// nod the head
		ros::Rate nodRate(3);
		std_msgs::Float64 ht;
		for(int i=0;i<2;i++) {
			ht.data = 0.5;
			head_tilt_pub.publish(ht);
			nodRate.sleep();
			ht.data = 0.0;
			head_tilt_pub.publish(ht);
			nodRate.sleep();
		}
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
