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
 */

#include "kdl_conversions/kdl_msg.h"

namespace tf {

  void pointMsgToKDL(const geometry_msgs::Point &m, KDL::Vector &k)
  {
    k[0] = m.x;
    k[1] = m.y;
    k[2] = m.z;
  }

  void pointKDLToMsg(const KDL::Vector &k, geometry_msgs::Point &m)
  {
    m.x = k[0];
    m.y = k[1];
    m.z = k[2];
  }

  void poseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k)
  {
    k.p[0] = m.position.x;
    k.p[1] = m.position.y;
    k.p[2] = m.position.z;
    
    k.M = KDL::Rotation::Quaternion( m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  void poseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m)
  {
    m.position.x = k.p[0];
    m.position.y = k.p[1];
    m.position.z = k.p[2];
    
    k.M.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  void quaternionMsgToKDL(const geometry_msgs::Quaternion &m, KDL::Rotation &k)
  {
    k = KDL::Rotation::Quaternion(m.x, m.y, m.z, m.w);
  }

  void quaternionKDLToMsg(const KDL::Rotation &k, geometry_msgs::Quaternion &m)
  {
    k.GetQuaternion(m.x, m.y, m.z, m.w);
  }

  void transformMsgToKDL(const geometry_msgs::Transform &m, KDL::Frame &k)
  {
    k.p[0] = m.translation.x;
    k.p[1] = m.translation.y;
    k.p[2] = m.translation.z;
    
    k.M = KDL::Rotation::Quaternion( m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w);
  }

  void transformKDLToMsg(const KDL::Frame &k, geometry_msgs::Transform &m)
  {
    m.translation.x = k.p[0];
    m.translation.y = k.p[1];
    m.translation.z = k.p[2];
    
    k.M.GetQuaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w);
  }

  void twistKDLToMsg(const KDL::Twist &t, geometry_msgs::Twist &m)
  {
    m.linear.x = t.vel[0];
    m.linear.y = t.vel[1];
    m.linear.z = t.vel[2];
    m.angular.x = t.rot[0];
    m.angular.y = t.rot[1];
    m.angular.z = t.rot[2];
  }

  void twistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &t)
  {
    t.vel[0] = m.linear.x;
    t.vel[1] = m.linear.y;
    t.vel[2] = m.linear.z;
    t.rot[0] = m.angular.x;
    t.rot[1] = m.angular.y;
    t.rot[2] = m.angular.z;
  }

  void vectorMsgToKDL(const geometry_msgs::Vector3 &m, KDL::Vector &k)
  {
    k[0] = m.x;
    k[1] = m.y;
    k[2] = m.z;
  }

  void vectorKDLToMsg(const KDL::Vector &k, geometry_msgs::Vector3 &m)
  {
    m.x = k[0];
    m.y = k[1];
    m.z = k[2];
  }

  void wrenchMsgToKDL(const geometry_msgs::Wrench &m, KDL::Wrench &k)
  {
    k[0] = m.force.x;
    k[1] = m.force.y;
    k[2] = m.force.z;
    k[3] = m.torque.x;
    k[4] = m.torque.y;
    k[5] = m.torque.z;
  }

  void wrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::Wrench &m)
  {
    m.force.x  = k[0];
    m.force.y  = k[1];
    m.force.z  = k[2];
    m.torque.x = k[3];
    m.torque.y = k[4];
    m.torque.z = k[5];
  }


///// Deprecated methods for backwards compatability

/// Converts a Pose message into a KDL Frame
void PoseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k) { poseMsgToKDL(m, k);}

/// Converts a KDL Frame into a Pose message 
void PoseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m) { poseKDLToMsg(k, m);}

/// Converts a Twist message into a KDL Twist
void TwistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k) {twistMsgToKDL(m, k);};

/// Converts a KDL Twist into a Twist message
void TwistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m){twistKDLToMsg(k, m);};


}  // namespace tf

