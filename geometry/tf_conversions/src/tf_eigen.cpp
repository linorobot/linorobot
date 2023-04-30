/*
 * Copyright (c) 2009-2012, Willow Garage, Inc.
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

// Author: Adam Leeper, Stuart Glaser

#include "tf_conversions/tf_eigen.h"

namespace tf {

  void matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e)
  {
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        e(i,j) = t[i][j];
  }
  
  void matrixEigenToTF(const Eigen::Matrix3d &e, tf::Matrix3x3 &t)
  {
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        t[i][j] =  e(i,j);
  }

  void poseTFToEigen(const tf::Pose &t, Eigen::Affine3d &e)
  {
    transformTFToEigen(t, e);
  }

  void poseTFToEigen(const tf::Pose &t, Eigen::Isometry3d &e)
  {
    transformTFToEigen(t, e);
  }

  void poseEigenToTF(const Eigen::Affine3d &e, tf::Pose &t)
  {
    transformEigenToTF(e, t);
  }

  void poseEigenToTF(const Eigen::Isometry3d &e, tf::Pose &t)
  {
    transformEigenToTF(e, t);
  }

  void quaternionTFToEigen(const tf::Quaternion& t, Eigen::Quaterniond& e)
  {
    e = Eigen::Quaterniond(t[3],t[0],t[1],t[2]);
  }
  
  void quaternionEigenToTF(const Eigen::Quaterniond& e, tf::Quaternion& t)
  {
    t[0] = e.x();
    t[1] = e.y();
    t[2] = e.z();
    t[3] = e.w();
  }

  namespace {
    template<typename Transform>
    void transformTFToEigenImpl(const tf::Transform &t, Transform & e)
    {
      for(int i=0; i<3; i++)
      {
        e.matrix()(i,3) = t.getOrigin()[i];
        for(int j=0; j<3; j++)
        {
          e.matrix()(i,j) = t.getBasis()[i][j];
        }
      }
      // Fill in identity in last row
      for (int col = 0 ; col < 3; col ++)
        e.matrix()(3, col) = 0;
      e.matrix()(3,3) = 1;
    }

    template<typename T>
    void transformEigenToTFImpl(const T &e, tf::Transform &t)
    {
      t.setOrigin(tf::Vector3( e.matrix()(0,3), e.matrix()(1,3), e.matrix()(2,3)));
      t.setBasis(tf::Matrix3x3(e.matrix()(0,0), e.matrix()(0,1), e.matrix()(0,2),
                               e.matrix()(1,0), e.matrix()(1,1), e.matrix()(1,2),
                               e.matrix()(2,0), e.matrix()(2,1), e.matrix()(2,2)));
    }
  }

  void transformTFToEigen(const tf::Transform &t, Eigen::Affine3d &e)
  {
    transformTFToEigenImpl(t, e);
  };

  void transformTFToEigen(const tf::Transform &t, Eigen::Isometry3d &e)
  {
    transformTFToEigenImpl(t, e);
  };

  void transformEigenToTF(const Eigen::Affine3d &e, tf::Transform &t)
  {
    transformEigenToTFImpl(e, t);
  }

  void transformEigenToTF(const Eigen::Isometry3d &e, tf::Transform &t)
  {
    transformEigenToTFImpl(e, t);
  }

  void vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3d& e)
  {
    e(0) = t[0];
    e(1) = t[1];
    e(2) = t[2];
  }

  void vectorEigenToTF(const Eigen::Vector3d& e, tf::Vector3& t)
  {
    t[0] = e(0);
    t[1] = e(1);
    t[2] = e(2);
  }

} // namespace tf
