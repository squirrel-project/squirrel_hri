// MotionExpressionServer.cpp --- 
// 
// Filename: MotionExpressionServer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Tue Jan 26 17:43:46 2016 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// Copyright (c) 2016, Federico Boniardi
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 

// Code:

#include "squirrel_expression/MotionExpressionServer.h"

namespace SQUIRREL_expression {

MotionExpressionServer::MotionExpressionServer( ros::NodeHandle& nh ) :
    dist_(std::numeric_limits<double>::max())
{
  dist_sub_ = nh.subscribe("/distance_sensors",1,&MotionExpressionServer::pointCloudCallback_, this);
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  motion_parser_["motion_1"] = MOTION_1;
  motion_parser_["motion_2"] = MOTION_2;
}

MotionExpressionServer::~MotionExpressionServer( void )
{
  dist_sub_.shutdown();
  cmd_pub_.shutdown();
}

void MotionExpressionServer::performMotionExpression( const std_msgs::String::ConstPtr& str )
{
  performMotion_(motion_parser_[str->data]);
}

void MotionExpressionServer::performMotion_( motion_t motion )
{
  switch ( motion ) {
    case MOTION_1: {
      ROS_INFO("%s: Perfoming motion 1", ros::this_node::getName().c_str());
      while ( true /*finished*/ ) {     
        if ( dist_ < 0.20 ) {
          geometry_msgs::Twist stop;
          cmd_pub_.publish(stop);
          break;
        }
      }
      break;
    }
    case MOTION_2: {
      ROS_INFO("%s: Perfoming motion 1", ros::this_node::getName().c_str());
      break;
    }
  }      
}

void MotionExpressionServer::pointCloudCallback_( const sensor_msgs::PointCloud::ConstPtr& pcl )
{
  double dist = std::numeric_limits<double>::max();
  for (size_t i=0; i<pcl->points.size(); ++i) {
    double di = l2norm_(pcl->points[i]);
    dist = di < dist ? di : dist;
  }
  dist_ = dist;
}

}  // namespace SQUIRREL_expression



/*-------------*/
/* Main method */
/*-------------*/

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "motion_expression_server");
  ros::NodeHandle nh;

  SQUIRREL_expression::MotionExpressionServer mes(nh);
  ros::Subscriber sub_ = nh.subscribe("/motion_expression",1, &SQUIRREL_expression::MotionExpressionServer::performMotionExpression, &mes);
  
  ROS_INFO("MotionExpressions: Ready to receive");
  
  ros::spin();

  return EXIT_SUCCESS;
}

// 
// MotionExpressionServer.cpp ends here
