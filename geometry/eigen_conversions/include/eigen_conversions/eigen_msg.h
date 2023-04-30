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

/*
 * Author: Stuart Glaser
 */

#ifndef EIGEN_MSG_CONVERSIONS_H
#define EIGEN_MSG_CONVERSIONS_H

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tf {

/// Converts a Point message into an Eigen Vector
void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e);

/// Converts an Eigen Vector into a Point message
void pointEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m);

/// Converts a Pose message into an Eigen Affine3d
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e);

/// Converts a Pose message into an Eigen Isometry3d
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d &e);

/// Converts an Eigen Affine3d into a Pose message
void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m);

/// Converts an Eigen Isometry3d into a Pose message
void poseEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Pose &m);

/// Converts a Quaternion message into an Eigen Quaternion
void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e);

/// Converts an Eigen Quaternion into a Quaternion message
void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::Quaternion &m);

/// Converts a Transform message into an Eigen Affine3d
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Affine3d &e);

/// Converts a Transform message into an Eigen Isometry3d
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Isometry3d &e);

/// Converts an Eigen Affine3d into a Transform message
void transformEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Transform &m);

/// Converts an Eigen Isometry3d into a Transform message
void transformEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Transform &m);

/// Converts a Twist message into an Eigen matrix
void twistMsgToEigen(const geometry_msgs::Twist &m, Eigen::Matrix<double,6,1> &e);

/// Converts an Eigen matrix into a Twist message
void twistEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Twist &m);

/// Converts a Vector message into an Eigen Vector
void vectorMsgToEigen(const geometry_msgs::Vector3 &m, Eigen::Vector3d &e);

/// Converts an Eigen Vector into a Vector message
void vectorEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Vector3 &m);

/// Converts a Wrench message into an Eigen matrix
void wrenchMsgToEigen(const geometry_msgs::Wrench &m, Eigen::Matrix<double,6,1> &e);

/// Converts an Eigen matrix into a Wrench message
void wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Wrench &m);

/// Converts an Eigen matrix into a Float64MultiArray message
template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}

} // namespace

#endif
