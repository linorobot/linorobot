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

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

void generate_rand_vectors(double scale, uint64_t runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}


using namespace tf;


TEST(tf_benchmark, canTransformCacheLength0)
{
  tf::Transformer mTR(true);

  uint64_t runs = 100000;
  ros::WallTime start_time = ros::WallTime::now();


  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    EXPECT_FALSE(mTR.canTransform("target","source_frame", ros::Time::now()));
  }
  ros::WallDuration run_duration = ros::WallTime::now() - start_time;
  double frequency = (double)runs / run_duration.toSec() ;
  printf("can frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);
}


TEST(tf_benchmark, canTransformCacheLength10000)
{
  tf::Transformer mTR(true);

  unsigned int cache_length = 10000;
  for (unsigned int i = 0; i < cache_length; i++)
  {
    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0+i), "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  uint64_t runs = 100000;
  ros::WallTime start_time = ros::WallTime::now();


  //Worst case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    EXPECT_TRUE(mTR.canTransform("child","my_parent", ros::Time().fromNSec(10)));
  }
  ros::WallDuration run_duration = ros::WallTime::now() - start_time;
  double frequency = (double)runs / run_duration.toSec() ;
  printf("Worst Case Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);

  //Worst case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    EXPECT_TRUE(mTR.canTransform("child","my_parent", ros::Time().fromNSec(10+cache_length/2)));
  }
  run_duration = ros::WallTime::now() - start_time;
  frequency = (double)runs / run_duration.toSec() ;
  printf("Intermediate Case Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);

  //Best case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    EXPECT_TRUE(mTR.canTransform("child","my_parent", ros::Time().fromNSec(10+cache_length -1)));
  }
  run_duration = ros::WallTime::now() - start_time;
  frequency = (double)runs / run_duration.toSec() ;
  printf("Best Case Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);
}

TEST(tf_benchmark, lookupTransformCacheLength10000)
{
  tf::Transformer mTR(true);

  unsigned int cache_length = 10000;
  for (unsigned int i = 0; i < cache_length; i++)
  {
    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0+i), "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  uint64_t runs = 100000;
  ros::WallTime start_time = ros::WallTime::now();

  StampedTransform rv;
  //Worst case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    mTR.lookupTransform("child","my_parent", ros::Time().fromNSec(10), rv);
  }
  ros::WallDuration run_duration = ros::WallTime::now() - start_time;
  double frequency = (double)runs / run_duration.toSec() ;
  printf("Worst Case Lookup Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);

  //Worst case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    mTR.lookupTransform("child","my_parent", ros::Time().fromNSec(10+cache_length/2), rv);
  }
  run_duration = ros::WallTime::now() - start_time;
  frequency = (double)runs / run_duration.toSec() ;
  printf("Intermediate Case Lookup Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);

  //Best case
  start_time = ros::WallTime::now();
  for (uint64_t i = 0 ; i < runs; i++)
  {
    mTR.lookupTransform("child","my_parent", ros::Time().fromNSec(10+cache_length -1), rv);
  }
  run_duration = ros::WallTime::now() - start_time;
  frequency = (double)runs / run_duration.toSec() ;
  printf("Best Case Lookup Frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 10000.0);
}

TEST(tf_benchmark, benchmarkExhaustiveSearch)
{
  uint64_t runs = 40000;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10.0 + i), "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  ros::WallTime start_time = ros::WallTime::now();
  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<tf::Transform> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0 + i), "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
  Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(runs), "child");
  Stamped<Pose> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("child",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
  ros::WallDuration run_duration = ros::WallTime::now() - start_time;
  double frequency = (double)runs / run_duration.toSec() ;
  printf("exhaustive search frequency %.2f KHz\n", frequency / 1000.0);
  EXPECT_GT( frequency, 500.0);
  
}

int main(int argc, char **argv){
	ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
