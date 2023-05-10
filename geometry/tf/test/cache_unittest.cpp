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

#include <gtest/gtest.h>
#include <tf/tf.h>
#include <sys/time.h>
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Matrix3x3.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
}

using namespace tf;


TEST(TimeCache, Repeatability)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  StampedTransform t;
  t.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    t.frame_id_ = ss.str();
    t.stamp_ = ros::Time().fromNSec(i);
    
    TransformStorage stor(t, i, 0);

    cache.insertData(stor);
  }

  TransformStorage out;
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    cache.getData(ros::Time().fromNSec(i), out);
    EXPECT_EQ(out.frame_id_, i);
    EXPECT_EQ(out.stamp_, ros::Time().fromNSec(i));
  }
  
}


TEST(TimeCache, RepeatabilityReverseInsertOrder)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  StampedTransform t;
  t.setIdentity();
  
  for ( int i = runs -1; i >= 0 ; i-- )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    t.stamp_ = ros::Time().fromNSec(i);

    TransformStorage stor(t, i, 0);

    cache.insertData(stor);
  }

  TransformStorage out;
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    cache.getData(ros::Time().fromNSec(i), out);
    EXPECT_EQ(out.frame_id_, i);
    EXPECT_EQ(out.stamp_, ros::Time().fromNSec(i));
  }
  
}

TEST(TimeCache, RepeatabilityRandomInsertOrder)
{
  seed_rand();
  
  tf::TimeCache  cache;
  double my_vals[] = {13,2,5,4,9,7,3,11,15,14,12,1,6,10,0,8};
  std::vector<double> values (my_vals, my_vals + sizeof(my_vals)/sizeof(double)); 
  unsigned int runs = values.size();

  StampedTransform t;
  t.setIdentity();

  for ( uint64_t i = 0; i <runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    t.stamp_ = ros::Time().fromNSec(i);

    TransformStorage stor(t, i, 0);
    
    cache.insertData(stor);
  }

  TransformStorage out;
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    cache.getData(ros::Time().fromNSec(i), out);
    EXPECT_EQ(out.frame_id_, i);
    EXPECT_EQ(out.stamp_, ros::Time().fromNSec(i));
  }
}

TEST(TimeCache, ZeroAtFront)
{
  uint64_t runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  StampedTransform t;
  t.setIdentity();
  
  for ( uint64_t i = 0; i <runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    t.stamp_ = ros::Time().fromNSec(i);

    TransformStorage stor(t, i, 0);
    
    cache.insertData(stor);
  }

  t.stamp_ = ros::Time().fromNSec(runs);
  TransformStorage stor(t, runs, 0);
  cache.insertData(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
  }

  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs));


  t.stamp_ = ros::Time().fromNSec(runs+1);
  TransformStorage stor2(t, runs+1, 0);
  cache.insertData(stor2);

  //Make sure we get a different value now that a new values is added at the front
  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.frame_id_, runs+1);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs+1));
}

TEST(TimeCache, CartesianInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 200;

  StampedTransform t;
  t.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    for (uint64_t step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      t.setOrigin(tf::Vector3(xvalues[step], yvalues[step], zvalues[step]));
      t.stamp_ = ros::Time().fromNSec(step * 100 + offset);
      TransformStorage stor(t, 2, 0);
      cache.insertData(stor);
    }
    
    TransformStorage out;
    for (int pos = 0; pos < 100 ; pos++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), out);
      double x_out = out.translation_.x();
      double y_out = out.translation_.y();
      double z_out = out.translation_.z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0, z_out, epsilon);
    }
    

    cache.clearList();
  }
}


/* TEST IS BROKEN, NEED TO PREVENT THIS
///\brief Make sure we dont' interpolate across reparented data
TEST(TimeCache, ReparentingInterpolationProtection)
{
  double epsilon = 1e-6;
  uint64_t offset = 555;

  tf::TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  StampedTransform t;
  t.setIdentity();

  for (uint64_t step = 0; step < 2 ; step++)
  {
    xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
    t.setOrigin(tf::Vector3(xvalues[step], yvalues[step], zvalues[step]));
    t.stamp_ = ros::Time().fromNSec(step * 100 + offset);
    TransformStorage stor(t, step + 4, 0);
    cache.insertData(stor);
  }
  
  TransformStorage out;
  for (int pos = 0; pos < 100 ; pos ++)
  {
    EXPECT_TRUE(cache.getData(ros::Time().fromNSec(offset + pos), out));
    double x_out = out.translation_.x();
    double y_out = out.translation_.y();
    double z_out = out.translation_.z();
    EXPECT_NEAR(xvalues[0], x_out, epsilon);
    EXPECT_NEAR(yvalues[0], y_out, epsilon);
    EXPECT_NEAR(zvalues[0], z_out, epsilon);
  }
  
  for (int pos = 100; pos < 120 ; pos ++)
  {
    EXPECT_TRUE(cache.getData(ros::Time().fromNSec(offset + pos), out));
    double x_out = out.translation_.x();
    double y_out = out.translation_.y();
    double z_out = out.translation_.z();
    EXPECT_NEAR(xvalues[1], x_out, epsilon);
    EXPECT_NEAR(yvalues[1], y_out, epsilon);
    EXPECT_NEAR(zvalues[1], z_out, epsilon);
  }
}

// EXTRAPOLATION DISABLED
TEST(TimeCache, CartesianExtrapolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-5;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 555;

  StampedTransform t;
  t.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    for (uint64_t step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      t.setOrigin(tf::Vector3(xvalues[step], yvalues[step], zvalues[step]));
      t.stamp_ = ros::Time().fromNSec(step * 100 + offset);
      TransformStorage stor(t, 2, 0);
      cache.insertData(stor);
    }
    
    TransformStorage out;
    for (int pos = -200; pos < 300 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), out);
      double x_out = out.translation_.x();
      double y_out = out.translation_.y();
      double z_out = out.translation_.z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0, z_out, epsilon);
    }
    
    cache.clearList();
  }
}
*/

TEST(Bullet, Slerp)
{
  uint64_t runs = 100;
  seed_rand();

  tf::Quaternion q1, q2;
  q1.setEuler(0,0,0);
  
  for (uint64_t i = 0 ; i < runs ; i++)
  {
    q2.setEuler(1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX);
    
    
    tf::Quaternion q3 = slerp(q1,q2,0.5);
    
    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
  }

}


TEST(TimeCache, AngularInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> yawvalues(2);
  std::vector<double> pitchvalues(2);
  std::vector<double> rollvalues(2);
  uint64_t offset = 200;

  std::vector<tf::Quaternion> quats(2);

  StampedTransform t;
  t.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    for (uint64_t step = 0; step < 2 ; step++)
    {
      yawvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX / 100.0;
      pitchvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      rollvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      quats[step].setRPY(yawvalues[step], pitchvalues[step], rollvalues[step]);
      t.setRotation(quats[step]);
      t.stamp_ = ros::Time().fromNSec(offset + (step * 100)); //step = 0 or 1
      TransformStorage stor(t, 3, 0);
      cache.insertData(stor);
    }
    
    TransformStorage out;
    for (int pos = 0; pos < 100 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), out); //get the transform for the position
      tf::Quaternion quat = out.rotation_; //get the quaternion out of the transform

      //Generate a ground truth quaternion directly calling slerp
      tf::Quaternion ground_truth = quats[0].slerp(quats[1], pos/100.0);
      
      //Make sure the transformed one and the direct call match
      EXPECT_NEAR(0, angle(ground_truth, quat), epsilon);
    }
    
    cache.clearList();
  }
}

TEST(TimeCache, DuplicateEntries)
{

  TimeCache cache;

  StampedTransform t;
  t.setIdentity();
  t.stamp_ = ros::Time().fromNSec(1);
  TransformStorage stor(t, 3, 0);
  cache.insertData(stor);
  cache.insertData(stor);


  cache.getData(ros::Time().fromNSec(1), stor);
  
  EXPECT_TRUE(!std::isnan(stor.translation_.x()));
  EXPECT_TRUE(!std::isnan(stor.translation_.y()));
  EXPECT_TRUE(!std::isnan(stor.translation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.x()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.y()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.w()));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
