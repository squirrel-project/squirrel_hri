// MotionExpressionServer.h --- 
// 
// Filename: MotionExpressionServer.h
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Tue Jan 26 17:31:15 2016 (+0100)
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

#ifndef MOTIONEXPRESSIONSERVER_H_
#define MOTIONEXPRESSIONSERVER_H_

#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>

#include <climits>
#include <map>
#include <string>

namespace SQUIRREL_expression {

class MotionExpressionServer
{
 public:
  MotionExpressionServer( ros::NodeHandle& );
  virtual ~MotionExpressionServer( void );  

  void performMotionExpression( const std_msgs::String::ConstPtr& );

 private:
  typedef enum {MOTION_1, MOTION_2} motion_t;

  ros::Subscriber dist_sub_;
  ros::Publisher cmd_pub_;

  std::map<std::string,motion_t> motion_parser_;

  double dist_;

  void performMotion_( motion_t );
  void pointCloudCallback_( const sensor_msgs::PointCloud::ConstPtr& );

  inline double l2norm_( const geometry_msgs::Point32& p )
  {
    return std::sqrt(p.x*p.x+p.y*p.y);
  };
};


}  // namespace 

#endif /* MOTIONEXPRESSIONSERVER_H_ */

// 
// MotionExpressionServer.h ends here
