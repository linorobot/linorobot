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

#include <time.h>
#include <tf_conversions/tf_kdl.h>
#include <gtest/gtest.h>

using namespace tf;

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(TFKDLConversions, tf_kdl_vector)
{
  tf::Vector3 t;
  t[0] = gen_rand(-10,10);
  t[1] = gen_rand(-10,10);
  t[2] = gen_rand(-10,10);

  KDL::Vector k;
  VectorTFToKDL(t,k);

  ASSERT_NEAR(t[0],k[0],1e-6);
  ASSERT_NEAR(t[1],k[1],1e-6);
  ASSERT_NEAR(t[2],k[2],1e-6);
}

TEST(TFKDLConversions, tf_kdl_rotation)
{
  tf::Quaternion t;
  t[0] = gen_rand(-1.0,1.0);
  t[1] = gen_rand(-1.0,1.0);
  t[2] = gen_rand(-1.0,1.0);
  t[3] = gen_rand(-1.0,1.0);
  t.normalize();

  KDL::Rotation k;
  QuaternionTFToKDL(t,k);

  double x,y,z,w;
  k.GetQuaternion(x,y,z,w);

  ASSERT_NEAR(fabs(t[0]),fabs(x),1e-6);
  ASSERT_NEAR(fabs(t[1]),fabs(y),1e-6);
  ASSERT_NEAR(fabs(t[2]),fabs(z),1e-6);
  ASSERT_NEAR(fabs(t[3]),fabs(w),1e-6);
  ASSERT_NEAR(t[0]*t[1]*t[2]*t[3],x*y*z*w,1e-6);

  
}

TEST(TFKDLConversions, tf_kdl_transform)
{
  tf::Transform t;
  tf::Quaternion tq;
  tq[0] = gen_rand(-1.0,1.0);
  tq[1] = gen_rand(-1.0,1.0);
  tq[2] = gen_rand(-1.0,1.0);
  tq[3] = gen_rand(-1.0,1.0);
  tq.normalize();
  t.setOrigin(tf::Vector3(gen_rand(-10,10),gen_rand(-10,10),gen_rand(-10,10)));
  t.setRotation(tq);

  //test tf->kdl
  KDL::Frame k;
  TransformTFToKDL(t,k);

  for(int i=0; i < 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],k.p[i],1e-6);
    for(int j=0; j < 3; j++)
    {      
      ASSERT_NEAR(t.getBasis()[i][j],k.M(i,j),1e-6);
    }
  }
  //test kdl->tf
  tf::Transform r;
  TransformKDLToTF(k,r);

  for(int i=0; i< 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],r.getOrigin()[i],1e-6);
    for(int j=0; j < 3; j++)
    {
      ASSERT_NEAR(t.getBasis()[i][j],r.getBasis()[i][j],1e-6);
    }

  }  
}

TEST(TFKDLConversions, tf_kdl_pose)
{
  tf::Pose t;
  tf::Quaternion tq;
  tq[0] = gen_rand(-1.0,1.0);
  tq[1] = gen_rand(-1.0,1.0);
  tq[2] = gen_rand(-1.0,1.0);
  tq[3] = gen_rand(-1.0,1.0);
  tq.normalize();
  t.setOrigin(tf::Vector3(gen_rand(-10,10),gen_rand(-10,10),gen_rand(-10,10)));
  t.setRotation(tq);

  //test tf->kdl                                                                                                              
  KDL::Frame k;
  PoseTFToKDL(t,k);

  for(int i=0; i < 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],k.p[i],1e-6);
    for(int j=0; j < 3; j++)
    {
      ASSERT_NEAR(t.getBasis()[i][j],k.M(i,j),1e-6);
    }
  }

  //test kdl->tf                                                                                                              
  tf::Pose r;
  PoseKDLToTF(k,r);

  for(int i=0; i< 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],r.getOrigin()[i],1e-6);
    for(int j=0; j < 3; j++)
    {
      ASSERT_NEAR(t.getBasis()[i][j],r.getBasis()[i][j],1e-6);
    }

  }
}

TEST(TFKDLConversions, msg_kdl_twist)
{
  geometry_msgs::Twist m;
  m.linear.x = gen_rand(-10,10);
  m.linear.y = gen_rand(-10,10);
  m.linear.z = gen_rand(-10,10);
  m.angular.x = gen_rand(-10,10);
  m.angular.y = gen_rand(-10,10);
  m.angular.y = gen_rand(-10,10);

  //test msg->kdl 
  KDL::Twist t;
  TwistMsgToKDL(m,t);

  ASSERT_NEAR(m.linear.x,t.vel[0],1e-6);
  ASSERT_NEAR(m.linear.y,t.vel[1],1e-6);
  ASSERT_NEAR(m.linear.z,t.vel[2],1e-6);
  ASSERT_NEAR(m.angular.x,t.rot[0],1e-6);
  ASSERT_NEAR(m.angular.y,t.rot[1],1e-6);
  ASSERT_NEAR(m.angular.z,t.rot[2],1e-6);


  //test msg->kdl                                                                                                             
  geometry_msgs::Twist r;
  TwistKDLToMsg(t,r);

  ASSERT_NEAR(r.linear.x,t.vel[0],1e-6);
  ASSERT_NEAR(r.linear.y,t.vel[1],1e-6);
  ASSERT_NEAR(r.linear.z,t.vel[2],1e-6);
  ASSERT_NEAR(r.angular.x,t.rot[0],1e-6);
  ASSERT_NEAR(r.angular.y,t.rot[1],1e-6);
  ASSERT_NEAR(r.angular.z,t.rot[2],1e-6);

}



int main(int argc, char **argv){
/* initialize random seed: */
  srand ( time(NULL) );
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
