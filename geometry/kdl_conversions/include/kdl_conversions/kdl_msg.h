/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Adam Leeper
 */

#ifndef CONVERSIONS_KDL_MSG_H
#define CONVERSIONS_KDL_MSG_H

#include "kdl/frames.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

#include <ros/macros.h>

namespace tf {
/// Conversion functions from/to the corresponding KDL and geometry_msgs types.

/// Converts a geometry_msgs Point into a KDL Vector
void pointMsgToKDL(const geometry_msgs::Point &m, KDL::Vector &k);

/// Converts a KDL Vector into a geometry_msgs Vector3
void pointKDLToMsg(const KDL::Vector &k, geometry_msgs::Point &m);

/// Converts a Pose message into a KDL Frame
void poseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k);

/// Converts a KDL Frame into a Pose message 
void poseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m);

/// Converts a quaternion message into a KDL Rotation
void quaternionMsgToKDL(const geometry_msgs::Quaternion &m, KDL::Rotation &k);

/// Converts a KDL Rotation into a message quaternion
void quaternionKDLToMsg(const KDL::Rotation &k, geometry_msgs::Quaternion &m);

/// Converts a Transform message into a KDL Frame
void transformMsgToKDL(const geometry_msgs::Transform &m, KDL::Frame &k);

/// Converts a KDL Frame into a Transform message 
void transformKDLToMsg(const KDL::Frame &k, geometry_msgs::Transform &m);

/// Converts a Twist message into a KDL Twist
void twistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k);

/// Converts a KDL Twist into a Twist message
void twistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m);

/// Converts a Vector3 message into a KDL Vector
void vectorMsgToKDL(const geometry_msgs::Vector3 &m, KDL::Vector &k);

/// Converts a KDL Vector into a Vector3 message
void vectorKDLToMsg(const KDL::Vector &k, geometry_msgs::Vector3 &m);

/// Converts a Wrench message into a KDL Wrench
void wrenchMsgToKDL(const geometry_msgs::Wrench &m, KDL::Wrench &k);

/// Converts a KDL Wrench into a Wrench message
void wrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::Wrench &m);


//Deprecated methods use above:
/// Converts a Pose message into a KDL Frame
ROS_DEPRECATED void PoseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k);

/// Converts a KDL Frame into a Pose message 
ROS_DEPRECATED void PoseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m);



/// Converts a Twist message into a KDL Twist
ROS_DEPRECATED void TwistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k);

/// Converts a KDL Twist into a Twist message
ROS_DEPRECATED void TwistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m);



}


#endif



