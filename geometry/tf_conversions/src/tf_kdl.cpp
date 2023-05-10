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

#include "tf_conversions/tf_kdl.h"
#include "kdl_conversions/kdl_msg.h"

namespace tf {

  void poseTFToKDL(const tf::Pose& t, KDL::Frame& k)
  {
    for (unsigned int i = 0; i < 3; ++i)
      k.p[i] = t.getOrigin()[i];
    for (unsigned int i = 0; i < 9; ++i)
      k.M.data[i] = t.getBasis()[i/3][i%3];
  }

  void poseKDLToTF(const KDL::Frame& k, tf::Pose& t)
  {
    t.setOrigin(tf::Vector3(k.p[0], k.p[1], k.p[2]));
    t.setBasis(tf::Matrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
                           k.M.data[3], k.M.data[4], k.M.data[5],
                           k.M.data[6], k.M.data[7], k.M.data[8]));
  }

  void quaternionTFToKDL(const tf::Quaternion& t, KDL::Rotation& k)
  {
    k = KDL::Rotation::Quaternion(t[0], t[1], t[2], t[3]);
  }

  void quaternionKDLToTF(const KDL::Rotation &k, tf::Quaternion &t)
  {
    double x, y, z, w;
    k.GetQuaternion(x, y, z, w);
    t = tf::Quaternion(x, y, z, w);
  }

  void transformTFToKDL(const tf::Transform &t, KDL::Frame &k)
  {
    for (unsigned int i = 0; i < 3; ++i)
      k.p[i] = t.getOrigin()[i];
    for (unsigned int i = 0; i < 9; ++i)
      k.M.data[i] = t.getBasis()[i/3][i%3];
  }

  void transformKDLToTF(const KDL::Frame &k, tf::Transform &t)
  {
    t.setOrigin(tf::Vector3(k.p[0], k.p[1], k.p[2]));
    t.setBasis(tf::Matrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
                           k.M.data[3], k.M.data[4], k.M.data[5],
                           k.M.data[6], k.M.data[7], k.M.data[8]));
  }

  void vectorTFToKDL(const tf::Vector3& t, KDL::Vector& k)
  {
    k[0] = t[0];
    k[1] = t[1];
    k[2] = t[2];
  }

  void vectorKDLToTF(const KDL::Vector& k, tf::Vector3& t)
  {
    t[0] = k[0];
    t[1] = k[1];
    t[2] = k[2];
  }
 
 
  /* DEPRECATED FUNCTIONS */
  geometry_msgs::Pose addDelta(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const double &t)
  {
    geometry_msgs::Pose result;
    KDL::Twist kdl_twist;
    KDL::Frame kdl_pose_id, kdl_pose;

    poseMsgToKDL(pose,kdl_pose);
    twistMsgToKDL(twist,kdl_twist);
    kdl_pose = KDL::addDelta(kdl_pose_id,kdl_twist,t)*kdl_pose;
    poseKDLToMsg(kdl_pose,result);
    return result;
  }

}  // namespace tf

