/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 */

#ifndef CONVERSIONS_TF_KDL_H
#define CONVERSIONS_TF_KDL_H

#include "kdl_conversions/kdl_msg.h" //backwards compatability for deprecated methods
#include "tf/transform_datatypes.h"
#include "kdl/frames.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <ros/macros.h>

namespace tf
{

/// Converts a tf Pose into a KDL Frame
void poseTFToKDL(const tf::Pose &t, KDL::Frame &k);

/// Converts a KDL Frame into a tf Pose
void poseKDLToTF(const KDL::Frame &k, tf::Pose &t);

/// Converts a tf Quaternion into a KDL Rotation
void quaternionTFToKDL(const tf::Quaternion &t, KDL::Rotation &k);

/// Converts a tf Quaternion into a KDL Rotation
void quaternionKDLToTF(const KDL::Rotation &k, tf::Quaternion &t);

/// Converts a tf Transform into a KDL Frame
void transformTFToKDL(const tf::Transform &t, KDL::Frame &k);

/// Converts a KDL Frame into a tf Transform
void transformKDLToTF(const KDL::Frame &k, tf::Transform &t);

/// Converts a tf Vector3 into a KDL Vector
void vectorTFToKDL(const tf::Vector3 &t, KDL::Vector &k);

/// Converts a tf Vector3 into a KDL Vector
void vectorKDLToTF(const KDL::Vector &k, tf::Vector3 &t);

/* DEPRECATED FUNCTIONS */
/// Starting from a Pose from A to B, apply a Twist with reference frame A and reference point B, during a time t.
ROS_DEPRECATED geometry_msgs::Pose addDelta(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const double &t);


/// Converts a tf Pose into a KDL Frame
ROS_DEPRECATED void PoseTFToKDL(const tf::Pose &t, KDL::Frame &k);
void inline PoseTFToKDL(const tf::Pose &t, KDL::Frame &k) {poseTFToKDL(t, k);};

/// Converts a KDL Frame into a tf Pose
ROS_DEPRECATED void PoseKDLToTF(const KDL::Frame &k, tf::Pose &t);
void inline PoseKDLToTF(const KDL::Frame &k, tf::Pose &t) {poseKDLToTF(k, t);};

/// Converts a tf Quaternion into a KDL Rotation
ROS_DEPRECATED void QuaternionTFToKDL(const tf::Quaternion &t, KDL::Rotation &k);
void inline QuaternionTFToKDL(const tf::Quaternion &t, KDL::Rotation &k) {quaternionTFToKDL(t, k);};

/// Converts a tf Quaternion into a KDL Rotation
ROS_DEPRECATED void QuaternionKDLToTF(const KDL::Rotation &k, tf::Quaternion &t);
void inline QuaternionKDLToTF(const KDL::Rotation &k, tf::Quaternion &t) {quaternionKDLToTF(k, t);};

/// Converts a tf Transform into a KDL Frame
ROS_DEPRECATED void TransformTFToKDL(const tf::Transform &t, KDL::Frame &k);
void inline TransformTFToKDL(const tf::Transform &t, KDL::Frame &k) {transformTFToKDL(t, k);};

/// Converts a KDL Frame into a tf Transform
ROS_DEPRECATED void TransformKDLToTF(const KDL::Frame &k, tf::Transform &t);
void inline TransformKDLToTF(const KDL::Frame &k, tf::Transform &t)  {transformKDLToTF(k, t);};

/// Converts a tf Vector3 into a KDL Vector
ROS_DEPRECATED void VectorTFToKDL(const tf::Vector3 &t, KDL::Vector &k);
void inline VectorTFToKDL(const tf::Vector3 &t, KDL::Vector &k) {vectorTFToKDL(t, k);};

/// Converts a tf Vector3 into a KDL Vector
ROS_DEPRECATED void VectorKDLToTF(const KDL::Vector &k, tf::Vector3 &t);
void inline VectorKDLToTF(const KDL::Vector &k, tf::Vector3 &t) {vectorKDLToTF(k, t);};


} // namespace tf

#endif
