/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <vector>
#include <sys/time.h>
#include <cstdio>
#include <gtest/gtest.h>

#include "tf/LinearMath/Transform.h"

double epsilon = 10E-6;


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
}

void testQuatRPY(tf::Quaternion q_baseline)
{
  q_baseline.normalize();
  tf::Matrix3x3 m(q_baseline);
  double roll, pitch, yaw;

  for (int solution = 1 ; solution < 2 ; ++solution)
  {
    m.getRPY(roll, pitch, yaw, solution);
    tf::Quaternion q_from_rpy;
    q_from_rpy.setRPY(roll, pitch, yaw);
    // use angleShortestPath() because angle() can return PI for equivalent
    // quaternions
    double angle1 = q_from_rpy.angleShortestPath(q_baseline);
    ASSERT_NEAR(0.0, angle1, epsilon);
    //std::printf("%f, angle between quaternions\n", angle1);

    tf::Matrix3x3 m2;
    m2.setRPY(roll, pitch, yaw);
    tf::Quaternion q_from_m_from_rpy;
    m2.getRotation(q_from_m_from_rpy);
    // use angleShortestPath() because angle() can return PI for equivalent
    // quaternions
    double angle2 = q_from_m_from_rpy.angleShortestPath(q_baseline);
    ASSERT_NEAR(0.0, angle2, epsilon);
    //std::printf("%f, angle between quaternions\n", angle2);
  }
}

TEST(tf, Quaternion)
{  
  unsigned int runs = 400;
  seed_rand();
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
  
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    tf::Matrix3x3 mat;
    tf::Quaternion q_baseline;
    q_baseline.setRPY(zvalues[i],yvalues[i],xvalues[i]);
    mat.setRotation(q_baseline);
    tf::Quaternion q_from_m;
    mat.getRotation(q_from_m);
    double angle = q_from_m.angle(q_baseline);
    ASSERT_NEAR(0.0, angle, epsilon);
    testQuatRPY(q_baseline);
  } 

  // test some corner cases
  testQuatRPY(tf::Quaternion( 0.5,  0.5,  0.5, -0.5));
  testQuatRPY(tf::Quaternion( 0.5,  0.5,  0.5,  0.5));
  testQuatRPY(tf::Quaternion( 0.5, -0.5,  0.5,  0.5));
  testQuatRPY(tf::Quaternion( 0.5,  0.5, -0.5,  0.5));
  testQuatRPY(tf::Quaternion(-0.5,  0.5,  0.5,  0.5));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
