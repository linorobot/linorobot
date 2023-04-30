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
#include <ros/ros.h>
#include "tf/LinearMath/Vector3.h"

#include "rostest/permuter.h"


using namespace tf;

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
}

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


void push_back_i(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  /*
     "a"
     v   (1,0)
     "b"
     v   (1,0)
     "c"
  */

  children.push_back("b");
  parents.push_back("a");
  dx.push_back(1.0);
  dy.push_back(0.0);
  children.push_back("c");
  parents.push_back("b");
  dx.push_back(1.0);
  dy.push_back(0.0);
}


void push_back_y(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
    /*
      "a"
      v  (1,0)
      "b" ------(0,1)-----> "d"
      v  (1,0)              v  (0,1)
      "c"                   "e"
    */
    // a>b
    children.push_back("b");
    parents.push_back("a");
    dx.push_back(1.0);
    dy.push_back(0.0);
     // b>c
    children.push_back("c");
    parents.push_back("b");
    dx.push_back(1.0);
    dy.push_back(0.0);
     // b>d
    children.push_back("d");
    parents.push_back("b");
    dx.push_back(0.0);
    dy.push_back(1.0);
     // d>e
    children.push_back("e");
    parents.push_back("d");
    dx.push_back(0.0);
    dy.push_back(1.0);
}

void push_back_v(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  /*
    "a" ------(0,1)-----> "f"
    v  (1,0)              v  (0,1)
    "b"                   "g"
    v  (1,0)
    "c"
  */
  // a>b
  children.push_back("b");
  parents.push_back("a");
  dx.push_back(1.0);
  dy.push_back(0.0);
  // b>c
  children.push_back("c");
  parents.push_back("b");
  dx.push_back(1.0);
  dy.push_back(0.0);
  // a>f
  children.push_back("f");
  parents.push_back("a");
  dx.push_back(0.0);
  dy.push_back(1.0);
  // f>g
  children.push_back("g");
  parents.push_back("f");
  dx.push_back(0.0);
  dy.push_back(1.0);

}

void push_back_1(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  children.push_back("2");
  parents.push_back("1");
  dx.push_back(1.0);
  dy.push_back(0.0);
}

void setupTree(tf::Transformer& mTR, const std::string& mode, const ros::Time & time, const ros::Duration& interpolation_space = ros::Duration())
{
  ROS_DEBUG("Clearing Buffer Core for new test setup");
  mTR.clear();

  ROS_DEBUG("Setting up test tree for formation %s", mode.c_str());

  std::vector<std::string> children;
  std::vector<std::string> parents;
  std::vector<double> dx, dy;

  if (mode == "i")
  {
    push_back_i(children, parents, dx, dy);
  }
  else if (mode == "y")
  {
    push_back_y(children, parents, dx, dy);
  }

  else if (mode == "v")
  {
    push_back_v(children, parents, dx, dy);
  }

  else if (mode == "ring_45")
  {
    /* Form a ring of transforms at every 45 degrees on the unit circle.  */

    std::vector<std::string> frames;

    frames.push_back("a");
    frames.push_back("b");
    frames.push_back("c");
    frames.push_back("d");
    frames.push_back("e");
    frames.push_back("f");
    frames.push_back("g");
    frames.push_back("h");
    frames.push_back("i");

    for (uint8_t iteration = 0; iteration < 2; ++iteration)
    {
      double direction = 1;
      std::string frame_prefix;
      if (iteration == 0)
      {
        frame_prefix = "inverse_";
        direction = -1;
      }
      else
        frame_prefix ="";
      for (uint64_t i = 1; i <  frames.size(); i++)
      {
    	  StampedTransform ts;
    	  ts.setIdentity();
    	  ts.setOrigin(tf::Vector3(direction * ( sqrt(2)/2 - 1), direction * sqrt(2)/2, 0));
          ts.setRotation(tf::Quaternion(0, 0, sin(direction * M_PI/8), cos(direction * M_PI/8)));
          if (time > ros::Time() + (interpolation_space * .5))
            ts.stamp_ = time - (interpolation_space * .5);
          else
            ts.stamp_ = ros::Time();

          ts.child_frame_id_ = frame_prefix + frames[i];
          if (i > 1)
            ts.frame_id_ = frame_prefix + frames[i-1];
          else
            ts.frame_id_ = frames[i-1]; // connect first frame
          
          EXPECT_TRUE(mTR.setTransform(ts, "authority"));
          if (interpolation_space > ros::Duration())
            ts.stamp_ = time + interpolation_space * .5;
      }
    }
    return; // nonstandard setup return before standard executinog
  }
  else if (mode == "1")
  {
    push_back_1(children, parents, dx, dy);

  }
  else if (mode =="1_v")
  {
    push_back_1(children, parents, dx, dy);
    push_back_v(children, parents, dx, dy);
  }
  else
    EXPECT_FALSE("Undefined mode for tree setup.  Test harness improperly setup.");


  /// Standard
  for (uint64_t i = 0; i <  children.size(); i++)
  {
    StampedTransform ts;
    ts.setIdentity();
    ts.setOrigin(tf::Vector3(dx[i], dy[i], 0));
    if (time > ros::Time() + (interpolation_space * .5))
      ts.stamp_ = time - (interpolation_space * .5);
    else
      ts.stamp_ = ros::Time();

    ts.frame_id_ = parents[i];
    ts.child_frame_id_ = children[i];
    EXPECT_TRUE(mTR.setTransform(ts, "authority"));
    if (interpolation_space > ros::Duration())
    {
      ts.stamp_ = time + interpolation_space * .5;
      EXPECT_TRUE(mTR.setTransform(ts, "authority"));

    }
  }
}

#define CHECK_QUATERNION_NEAR(_q1, _q2, _epsilon)        \
    EXPECT_NEAR(_q1.angle(_q2), 0, _epsilon);            \


#define CHECK_TRANSFORMS_NEAR(_out, _expected, _eps)                        \
  EXPECT_NEAR(_out.getOrigin().x(), _expected.getOrigin().x(), epsilon);    \
  EXPECT_NEAR(_out.getOrigin().y(), _expected.getOrigin().y(), epsilon);    \
  EXPECT_NEAR(_out.getOrigin().z(), _expected.getOrigin().z(), epsilon);    \
  CHECK_QUATERNION_NEAR(_out.getRotation(),  _expected.getRotation(), _eps);


// Simple test with compound transform
TEST(tf, lookupTransform_compount)
{
	/*
	 * Frames
	 *
	 * root->a
	 *
	 * root->b->c->d
	 *
	 */

	double epsilon = 2e-5; // Larger epsilon for interpolation values

    tf::Transformer mTR;

    StampedTransform tsa;
    tsa.frame_id_ = "root";
    tsa.child_frame_id_  = "a";
    tsa.setOrigin(tf::Vector3(1,1,1));
    tf::Quaternion q1;
    q1.setRPY(0.25, .5, .75);
    tsa.setRotation(q1);
    EXPECT_TRUE(mTR.setTransform(tsa, "authority"));

    StampedTransform tsb;
    tsb.frame_id_ = "root";
    tsb.child_frame_id_  = "b";
    tsb.setOrigin(tf::Vector3(-1, 0, -1));
    tf::Quaternion q2;
    q2.setRPY(1.0, 0.25, 0.5);
    tsb.setRotation(q2);
    EXPECT_TRUE(mTR.setTransform(tsb, "authority"));

    StampedTransform tsc;
    tsc.frame_id_ = "b";
    tsc.child_frame_id_  = "c";
    tsc.setOrigin(tf::Vector3(0.0, 2.0, 0.5));
    tf::Quaternion q3;
    q3.setRPY(0.25, 0.75, 1.25);
    tsc.setRotation(q3);
    EXPECT_TRUE(mTR.setTransform(tsc, "authority"));

    StampedTransform tsd;
    tsd.frame_id_ = "c";
    tsd.child_frame_id_  = "d";
    tsd.setOrigin(tf::Vector3(0.5, -1, 1.5));
    tf::Quaternion q4;
    q4.setRPY(-0.5, 1.0, -0.75);
    tsd.setRotation(q4);
    EXPECT_TRUE(mTR.setTransform(tsd, "authority"));


    tf::Transform expected_ab, expected_bc, expected_cb, expected_ac, expected_ba, expected_ca, expected_ad, expected_da, expected_bd, expected_db, expected_rootd, expected_rootc;

    expected_ab = tsa.inverse() * tsb;
    expected_ac = tsa.inverse() * tsb * tsc;
    expected_ad = tsa.inverse() * tsb * tsc * tsd;
    expected_cb = tsc.inverse();
    expected_bc = tsc;
    expected_bd = tsc * tsd;
    expected_db = expected_bd.inverse();
    expected_ba = tsb.inverse() * tsa;
    expected_ca = tsc.inverse() * tsb.inverse() * tsa;
    expected_da = tsd.inverse() * tsc.inverse() * tsb.inverse() * tsa;
    expected_rootd = tsb * tsc * tsd;
    expected_rootc = tsb * tsc;

    // root -> b -> c
    StampedTransform out_rootc;
    mTR.lookupTransform("root", "c", ros::Time(), out_rootc);
    CHECK_TRANSFORMS_NEAR(out_rootc, expected_rootc, epsilon);

    // root -> b -> c -> d
    StampedTransform out_rootd;
    mTR.lookupTransform("root", "d", ros::Time(), out_rootd);
    CHECK_TRANSFORMS_NEAR(out_rootd, expected_rootd, epsilon);

    // a <- root -> b
    StampedTransform out_ab;
    mTR.lookupTransform("a", "b", ros::Time(), out_ab);
    CHECK_TRANSFORMS_NEAR(out_ab, expected_ab, epsilon);

    StampedTransform out_ba;
    mTR.lookupTransform("b", "a", ros::Time(), out_ba);
    CHECK_TRANSFORMS_NEAR(out_ba, expected_ba, epsilon);

    // a <- root -> b -> c
    StampedTransform out_ac;
    mTR.lookupTransform("a", "c", ros::Time(), out_ac);
    CHECK_TRANSFORMS_NEAR(out_ac, expected_ac, epsilon);

    StampedTransform out_ca;
    mTR.lookupTransform("c", "a", ros::Time(), out_ca);
    CHECK_TRANSFORMS_NEAR(out_ca, expected_ca, epsilon);

    // a <- root -> b -> c -> d
    StampedTransform out_ad;
    mTR.lookupTransform("a", "d", ros::Time(), out_ad);
    CHECK_TRANSFORMS_NEAR(out_ad, expected_ad, epsilon);

    StampedTransform out_da; 
    mTR.lookupTransform("d", "a", ros::Time(), out_da);
    CHECK_TRANSFORMS_NEAR(out_da, expected_da, epsilon);

    // b -> c
    StampedTransform out_cb;
    mTR.lookupTransform("c", "b", ros::Time(), out_cb);
    CHECK_TRANSFORMS_NEAR(out_cb, expected_cb, epsilon);

    StampedTransform out_bc;
    mTR.lookupTransform("b", "c", ros::Time(), out_bc);
    CHECK_TRANSFORMS_NEAR(out_bc, expected_bc, epsilon);

    // b -> c -> d
    StampedTransform out_bd;
    mTR.lookupTransform("b", "d", ros::Time(), out_bd);
    CHECK_TRANSFORMS_NEAR(out_bd, expected_bd, epsilon);

    StampedTransform out_db;
    mTR.lookupTransform("d", "b", ros::Time(), out_db);
    CHECK_TRANSFORMS_NEAR(out_db, expected_db, epsilon);
}

// Time varying transforms, testing interpolation
TEST(tf, lookupTransform_helix_configuration)
{
	double epsilon = 2e-5; // Larger epsilon for interpolation values

    tf::Transformer mTR;

    ros::Time     t0        = ros::Time() + ros::Duration(10);
    ros::Duration step      = ros::Duration(0.05);
    ros::Duration half_step = ros::Duration(0.025);
    ros::Time     t1        = t0 + ros::Duration(5.0);

    /*
     * a->b->c
     *
     * b.z = vel * (t - t0)
     * c.x = cos(theta * (t - t0))
     * c.y = sin(theta * (t - t0))
     *
     * a->d
     *
     * d.z = 2 * cos(theta * (t - t0))
     * a->d transforms are at half-step between a->b->c transforms
     */

    double theta = 0.25;
    double vel   = 1.0;

    for (ros::Time t = t0; t <= t1; t += step)
    {
    	ros::Time t2 = t + half_step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        StampedTransform ts;
        ts.setIdentity();
        ts.frame_id_        = "a";
        ts.stamp_           = t;
        ts.child_frame_id_  = "b";
        ts.setOrigin(tf::Vector3(0.0, 0.0, vel * dt));
        EXPECT_TRUE(mTR.setTransform(ts, "authority"));

        StampedTransform ts2;
        ts2.setIdentity();
        ts2.frame_id_        = "b";
        ts2.stamp_           = t;
        ts2.child_frame_id_  = "c";
        ts2.setOrigin(tf::Vector3(cos(theta*dt), sin(theta*dt),0));
        tf::Quaternion q;
        q.setRPY(0,0,theta*dt);
        ts2.setRotation(q);
        EXPECT_TRUE(mTR.setTransform(ts2, "authority"));

        StampedTransform ts3;
        ts3.setIdentity();
        ts3.frame_id_        = "a";
        ts3.stamp_           = t2;
        ts3.child_frame_id_  = "d";
        ts3.setOrigin(tf::Vector3(0, 0, cos(theta*dt2)));
        EXPECT_TRUE(mTR.setTransform(ts3, "authority"));
    }

    for (ros::Time t = t0 + half_step; t < t1; t += step)
    {
    	ros::Time t2 = t + half_step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        StampedTransform out_ab;
        mTR.lookupTransform("a", "b", t, out_ab);
        tf::Transform expected_ab;
        expected_ab.setIdentity();
        expected_ab.setOrigin(tf::Vector3(0.0, 0.0, vel*dt));
        CHECK_TRANSFORMS_NEAR(out_ab, expected_ab, epsilon);

        StampedTransform out_ac;
        mTR.lookupTransform("a", "c", t, out_ac);
        tf::Transform expected_ac;
        expected_ac.setOrigin(tf::Vector3(cos(theta*dt), sin(theta*dt), vel*dt));
        tf::Quaternion q;
        q.setRPY(0,0,theta*dt);
        expected_ac.setRotation(q);
        CHECK_TRANSFORMS_NEAR(out_ac, expected_ac, epsilon);

        StampedTransform out_ad;
        mTR.lookupTransform("a", "d", t, out_ad);
        EXPECT_NEAR(out_ad.getOrigin().z(), cos(theta*dt), epsilon);

        StampedTransform out_cd;
        mTR.lookupTransform("c", "d", t2, out_cd);
        tf::Transform expected_cd;
        expected_cd.setOrigin(tf::Vector3(-1, 0, cos(theta * dt2) - vel * dt2));
        tf::Quaternion q2;
        q2.setRPY(0,0,-theta*dt2);
        expected_cd.setRotation(q2);
        CHECK_TRANSFORMS_NEAR(out_cd, expected_cd, epsilon);
    }

    // Advanced API
    for (ros::Time t = t0 + half_step; t < t1; t += (step + step))
    {
    	ros::Time t2 = t + step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        StampedTransform out_cd2;
        mTR.lookupTransform("c", t, "d", t2, "a", out_cd2);
        tf::Transform expected_cd2;
        expected_cd2.setOrigin(tf::Vector3(-1, 0, cos(theta*dt2) - vel*dt));
        tf::Quaternion mq2;
        mq2.setRPY(0,0,-theta*dt);
        expected_cd2.setRotation(mq2);
        CHECK_TRANSFORMS_NEAR(out_cd2, expected_cd2, epsilon);
    }

}

TEST(tf, lookupTransform_ring45)
{
  double epsilon = 1e-6;
  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.01));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  frames.push_back("d");
  frames.push_back("e");
  frames.push_back("f");
  frames.push_back("g");
  frames.push_back("h");
  frames.push_back("i");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {
    tf::Transformer mTR;
    setupTree(mTR, "ring_45", eval_time, interpolation_space);

    StampedTransform out_xfm;
    mTR.lookupTransform(source_frame, target_frame, eval_time, out_xfm);

    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    if (source_frame != target_frame)
    	EXPECT_EQ(out_xfm.stamp_, 		eval_time);
    EXPECT_TRUE(out_xfm.frame_id_       == source_frame || out_xfm.frame_id_       == "/" + source_frame) << "Expected frame_id_ to equal source_frame: " << out_xfm.frame_id_ << ", " << source_frame << std::endl;
    EXPECT_TRUE(out_xfm.child_frame_id_ == target_frame || out_xfm.child_frame_id_ == "/" + target_frame) << "Expected child_frame_id_ to equal target_frame: " << out_xfm.child_frame_id_ << ", " << target_frame << std::endl;

    //Zero distance or all the way
    if (source_frame == target_frame               ||
        (source_frame == "a" && target_frame == "i") ||
        (source_frame == "i" && target_frame == "a") ||
        (source_frame == "a" && target_frame == "inverse_i") ||
        (source_frame == "inverse_i" && target_frame == "a") )
    {
      tf::Transform expected;
      expected.setIdentity();
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 1
    else if ((source_frame == "a" && target_frame =="b") ||
             (source_frame == "b" && target_frame =="c") ||
             (source_frame == "c" && target_frame =="d") ||
             (source_frame == "d" && target_frame =="e") ||
             (source_frame == "e" && target_frame =="f") ||
             (source_frame == "f" && target_frame =="g") ||
             (source_frame == "g" && target_frame =="h") ||
             (source_frame == "h" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI/8),cos(M_PI/8)), tf::Vector3(sqrt(2)/2 - 1, sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 1
    else if ((source_frame == "b" && target_frame =="a") ||
             (source_frame == "c" && target_frame =="b") ||
             (source_frame == "d" && target_frame =="c") ||
             (source_frame == "e" && target_frame =="d") ||
             (source_frame == "f" && target_frame =="e") ||
             (source_frame == "g" && target_frame =="f") ||
             (source_frame == "h" && target_frame =="g") ||
             (source_frame == "i" && target_frame =="h")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI/8),cos(-M_PI/8)), tf::Vector3(sqrt(2)/2 - 1, -sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 2
    else if ((source_frame == "a" && target_frame =="c") ||
             (source_frame == "b" && target_frame =="d") ||
             (source_frame == "c" && target_frame =="e") ||
             (source_frame == "d" && target_frame =="f") ||
             (source_frame == "e" && target_frame =="g") ||
             (source_frame == "f" && target_frame =="h") ||
             (source_frame == "g" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI/4),cos(M_PI/4)), tf::Vector3(-1, 1, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 2
    else if ((source_frame == "c" && target_frame =="a") ||
             (source_frame == "d" && target_frame =="b") ||
             (source_frame == "e" && target_frame =="c") ||
             (source_frame == "f" && target_frame =="d") ||
             (source_frame == "g" && target_frame =="e") ||
             (source_frame == "h" && target_frame =="f") ||
             (source_frame == "i" && target_frame =="g")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI/4),cos(-M_PI/4)), tf::Vector3(-1, -1, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 3
    else if ((source_frame == "a" && target_frame =="d") ||
             (source_frame == "b" && target_frame =="e") ||
             (source_frame == "c" && target_frame =="f") ||
             (source_frame == "d" && target_frame =="g") ||
             (source_frame == "e" && target_frame =="h") ||
             (source_frame == "f" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI*3/8),cos(M_PI*3/8)), tf::Vector3(-1 - sqrt(2)/2, sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 3
    else if ((target_frame == "a" && source_frame =="d") ||
             (target_frame == "b" && source_frame =="e") ||
             (target_frame == "c" && source_frame =="f") ||
             (target_frame == "d" && source_frame =="g") ||
             (target_frame == "e" && source_frame =="h") ||
             (target_frame == "f" && source_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI*3/8),cos(-M_PI*3/8)), tf::Vector3(-1 - sqrt(2)/2, -sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 4
    else if ((source_frame == "a" && target_frame =="e") ||
             (source_frame == "b" && target_frame =="f") ||
             (source_frame == "c" && target_frame =="g") ||
             (source_frame == "d" && target_frame =="h") ||
             (source_frame == "e" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI/2),cos(M_PI/2)), tf::Vector3(-2, 0, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 4
    else if ((target_frame == "a" && source_frame =="e") ||
             (target_frame == "b" && source_frame =="f") ||
             (target_frame == "c" && source_frame =="g") ||
             (target_frame == "d" && source_frame =="h") ||
             (target_frame == "e" && source_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI/2),cos(-M_PI/2)), tf::Vector3(-2, 0, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 5
    else if ((source_frame == "a" && target_frame =="f") ||
             (source_frame == "b" && target_frame =="g") ||
             (source_frame == "c" && target_frame =="h") ||
             (source_frame == "d" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI*5/8),cos(M_PI*5/8)), tf::Vector3(-1-sqrt(2)/2, -sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 5
    else if ((target_frame == "a" && source_frame =="f") ||
             (target_frame == "b" && source_frame =="g") ||
             (target_frame == "c" && source_frame =="h") ||
             (target_frame == "d" && source_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI*5/8),cos(-M_PI*5/8)), tf::Vector3(-1-sqrt(2)/2, sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 6
    else if ((source_frame == "a" && target_frame =="g") ||
             (source_frame == "b" && target_frame =="h") ||
             (source_frame == "c" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI*3/4),cos(M_PI*3/4)), tf::Vector3(-1, -1, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 6
    else if ((target_frame == "a" && source_frame =="g") ||
             (target_frame == "b" && source_frame =="h") ||
             (target_frame == "c" && source_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI*3/4),cos(-M_PI*3/4)), tf::Vector3(-1, 1, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Chaining 7
    else if ((source_frame == "a" && target_frame =="h") ||
             (source_frame == "b" && target_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(M_PI*7/8),cos(M_PI*7/8)), tf::Vector3(sqrt(2)/2-1, -sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    // Inverse Chaining 7
    else if ((target_frame == "a" && source_frame =="h") ||
             (target_frame == "b" && source_frame =="i")
             )
    {
      tf::Transform expected(tf::Quaternion(0,0,sin(-M_PI*7/8),cos(-M_PI*7/8)), tf::Vector3(sqrt(2)/2-1, sqrt(2)/2, 0));
      CHECK_TRANSFORMS_NEAR(out_xfm, expected, epsilon);
    }
    else
    {
      EXPECT_FALSE("Ring_45 testing Shouldn't get here");
      printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    }

  }
}

TEST(tf, setTransformNoInsertOnSelfTransform)
{
  tf::Transformer mTR(true);
  StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0), "same_frame", "same_frame");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, setTransformNoInsertWithNan)
{
  tf::Transformer mTR(true);
  StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0), "same_frame", "other_frame");
  EXPECT_TRUE(mTR.setTransform(tranStamped));

  tranStamped.setOrigin(tf::Point(1.0,1.0,0.0/0.0));
  EXPECT_TRUE(std::isnan(tranStamped.getOrigin().z()));
  EXPECT_FALSE(mTR.setTransform(tranStamped));

}

TEST(tf, setTransformNoInsertWithNoFrameID)
{
  tf::Transformer mTR(true);
  StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0), "parent_frame", "");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, setTransformNoInsertWithNoParentID)
{
  tf::Transformer mTR(true);
  StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0), "", "my_frame");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, TransformTransformsCartesian)
{
  uint64_t runs = 400;
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

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10.0 + i), "child");

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
  
  
}

/** Make sure that the edge cases of transform the top of the tree to the top of the tree and 
 * the leaf of the tree can transform to the leaf of the tree without a lookup exception and accurately */
TEST(tf, TransformTransformToOwnFrame)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs), yawvalues(runs),  pitchvalues(runs),  rollvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yawvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    pitchvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    rollvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    tf::Quaternion qt;
    qt.setRPY(yawvalues[i],pitchvalues[i],rollvalues[i]);
    StampedTransform tranStamped(tf::Transform(qt, tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "parent", "child");
    mTR.setTransform(tranStamped);

  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "child");
    Stamped<Pose> inpose2 (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1


    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("parent",inpose2, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1
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
  
  
}

TEST(tf, TransformPointCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "my_parent", "child");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (tf::Vector3(x,y,z), ros::Time().fromNSec(10 + i), "child");

    try{
      Stamped<Point> outvec(tf::Vector3(0,0,0), ros::Time().fromNSec(10 + i), "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPoint("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), xvalues[i]+x, epsilon);
    EXPECT_NEAR(outvec.y(), yvalues[i]+y, epsilon);
    EXPECT_NEAR(outvec.z(), zvalues[i]+z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformVectorCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "my_parent", "child");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (tf::Vector3(x,y,z), ros::Time().fromNSec(10 + i), "child");

    try{
      Stamped<Vector3> outvec(tf::Vector3(0,0,0), ros::Time().fromNSec(10 + i), "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformVector("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), x, epsilon);
    EXPECT_NEAR(outvec.y(), y, epsilon);
    EXPECT_NEAR(outvec.z(), z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformQuaternionCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;


    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "my_parent", "child");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    tf::Quaternion qt;
    qt.setRPY(xvalues[i],yvalues[i],zvalues[i]);
    Stamped<tf::Quaternion> invec (qt, ros::Time().fromNSec(10 + i), "child");
    //    printf("%f, %f, %f\n", xvalues[i],yvalues[i], zvalues[i]);

    try{
      
      Stamped<tf::Quaternion> outvec(qt, ros::Time().fromNSec(10 + i), "child");

    mTR.transformQuaternion("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.angle(invec) , 0, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(data, Vector3Conversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    tf::Vector3 btv = tf::Vector3(xvalues[i], yvalues[i], zvalues[i]);
    tf::Vector3 btv_out = tf::Vector3(0,0,0);
    geometry_msgs::Vector3 msgv;
    vector3TFToMsg(btv, msgv);
    vector3MsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
  } 
  
}

TEST(data, Vector3StampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<tf::Vector3> btv = Stamped<tf::Vector3>(tf::Vector3(xvalues[i], yvalues[i], zvalues[i]), ros::Time().fromNSec(1), "no frame");
    Stamped<tf::Vector3> btv_out;
    geometry_msgs::Vector3Stamped msgv;
    vector3StampedTFToMsg(btv, msgv);
    vector3StampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, QuaternionConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    tf::Quaternion btv;
    btv.setRPY(xvalues[i], yvalues[i], zvalues[i]);
    tf::Quaternion btv_out = tf::Quaternion(0,0,0,1);
    geometry_msgs::Quaternion msgv;
    quaternionTFToMsg(btv, msgv);
    quaternionMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
  } 
  
}

TEST(data, QuaternionStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<tf::Quaternion> btv = Stamped<tf::Quaternion>(tf::Quaternion(), ros::Time().fromNSec(1), "no frame");
    btv.setRPY(xvalues[i], yvalues[i], zvalues[i]);
    Stamped<tf::Quaternion> btv_out;
    geometry_msgs::QuaternionStamped msgv;
    quaternionStampedTFToMsg(btv, msgv);
    quaternionStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, TransformConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    tf::Quaternion qt;
    qt.setRPY(xvalues2[i], yvalues2[i], zvalues2[i]);
    tf::Transform btv = tf::Transform(qt, tf::Vector3(xvalues[i], yvalues[i], zvalues[i]));
    tf::Transform btv_out;
    geometry_msgs::Transform msgv;
    transformTFToMsg(btv, msgv);
    transformMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
  } 
  
}

TEST(data, PoseStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    tf::Quaternion qt;
    qt.setRPY(xvalues2[i], yvalues2[i], zvalues2[i]);
    Stamped<Pose> btv = Stamped<Pose>(tf::Transform(qt, tf::Vector3(xvalues[i], yvalues[i], zvalues[i])), ros::Time().fromNSec(1), "no frame");
    Stamped<Pose> btv_out;
    geometry_msgs::PoseStamped msgv;
    poseStampedTFToMsg(btv, msgv);
    poseStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(tf, ListOneInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "child");

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
  
}

TEST(tf, ListTwoInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "child", "grandchild");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "grandchild");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


TEST(tf, ListOneForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, ListTwoForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "child", "grandchild");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("grandchild",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformThrougRoot)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100),  "my_parent", "childA");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100),  "my_parent", "childB");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000 + i*100), "childA");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformThroughNO_PARENT)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parentA", "childA");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parentB", "childB");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<tf::Transform> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10 + i), "childA");
    bool exception_thrown = false;

    try{
    Stamped<tf::Transform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }
  
}


TEST(tf, getParent)
{
  
  std::vector<std::string> children;
  std::vector<std::string> parents;

  children.push_back("a");
  parents.push_back("c");

  children.push_back("b");
  parents.push_back("c");

  children.push_back("c");
  parents.push_back("e");

  children.push_back("d");
  parents.push_back("e");

  children.push_back("e");
  parents.push_back("f");

  children.push_back("f");
  parents.push_back("j");

  // Issue #74.
  children.push_back("/k");
  parents.push_back("l");

  tf::Transformer mTR(true);

  for (uint64_t i = 0; i <  children.size(); i++)
    {
      StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10), parents[i], children[i]);
      mTR.setTransform(tranStamped);
    }

  //std::cout << mTR.allFramesAsString() << std::endl;

  std::string output;
  for  (uint64_t i = 0; i <  children.size(); i++)
    {
      EXPECT_TRUE(mTR.getParent(children[i], ros::Time().fromNSec(10), output));
      EXPECT_STREQ(parents[i].c_str(), output.c_str());
    }
  
  EXPECT_FALSE(mTR.getParent("j", ros::Time().fromNSec(10), output));

  EXPECT_FALSE(mTR.getParent("no_value", ros::Time().fromNSec(10), output));
  
}


TEST(tf, NO_PARENT_SET)
{
  double epsilon = 1e-6;
  
  std::vector<std::string> children;
  std::vector<std::string> parents;

  children.push_back("b");
  parents.push_back("a");
  children.push_back("a");
  parents.push_back("NO_PARENT");

  tf::Transformer mTR(true);

  for (uint64_t i = 0; i <  children.size(); i++)
    {
      StampedTransform tranStamped(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10),  parents[i], children[i]);
      mTR.setTransform(tranStamped);
    }

  //std::cout << mTR.allFramesAsString() << std::endl;


  Stamped<tf::Transform> inpose (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10), "a");
  Stamped<tf::Transform> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("a",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
}

TEST(tf, waitForTransform)
{
  EXPECT_TRUE(ros::ok());

  tf::Transformer mTR(true);

  

  // Check assertion of extra string
  std::string error_str;
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), ros::Duration().fromSec(1.0), ros::Duration().fromSec(0.01), &error_str));
  EXPECT_STREQ(tf2_ros::threading_error.c_str(), error_str.c_str());

  // check that it doesn't segfault if NULL
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), ros::Duration().fromSec(1.0), ros::Duration().fromSec(0.01)));

  

  //A seperate thread is required to use the blocking call for normal usage
  // This isn't actually using it, but it will not affect this direct usage case.  
  mTR.setUsingDedicatedThread(true);  
  // make sure timeout is resonably lengthed

  ros::Duration timeout = ros::Duration().fromSec(1.0);
  ros::Duration poll_freq = ros::Duration().fromSec(0.1);
  double eps = 0.2;

  // Default polling freq
  ros::Time start_time = ros::Time::now();
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), timeout));
  ros::Time stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)-timeout).toSec()) < eps);

  // 10Hz polling
  start_time = ros::Time::now();
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), timeout, poll_freq));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)-timeout).toSec()) < eps);
  

  //Now it should be able to transform
  mTR.setTransform( StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000000),  "parent", "me"));
  
  start_time = ros::Time::now();
  EXPECT_TRUE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000),timeout));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)).toSec()) < eps);


  start_time = ros::Time::now();
  EXPECT_TRUE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000),timeout, poll_freq));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)).toSec()) < eps);
}


TEST(tf, Exceptions)
{

 tf::Transformer mTR(true);

 
 Stamped<tf::Transform> outpose;

 //connectivity when no data
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(10000000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000000) , "me"), outpose);
   EXPECT_FALSE("ConnectivityException Not Thrown");   
 }
 catch ( tf::LookupException &ex)
 {
   EXPECT_TRUE("Lookupgh Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 
 mTR.setTransform( StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(100000), "parent", "me"));

 //Extrapolation not valid with one value
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 

 mTR.setTransform( StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(300000), "parent", "me"));

 //NO Extration when Interpolating
 //inverse list
 EXPECT_TRUE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }



 //forward list
 EXPECT_TRUE(mTR.canTransform("me", "parent", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("me",Stamped<Pose>(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(200000) , "parent"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  

 //Extrapolating backwards
 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(1000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose> (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 //forwards list
 EXPECT_FALSE(mTR.canTransform("me", "parent", ros::Time().fromNSec(1000)));
 try 
 {
   mTR.transformPose("me",Stamped<Pose> (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  


 // Test extrapolation inverse and forward linkages FORWARD

 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try 
 {
   mTR.transformPose("parent", Stamped<Pose> (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(350000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }

 //forward list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try 
 {
   mTR.transformPose("me", Stamped<Pose> (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(350000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  



}



TEST(tf, NoExtrapolationExceptionFromParent)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));
  


  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000), "parent", "a"));
  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000),  "parent", "a"));
  
  
  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000),  "parent", "b"));
  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000),  "parent", "b"));

  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent", "parent"));
  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent's parent", "parent's parent"));

  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000),  "parent's parent", "parent"));
  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(10000),  "parent's parent's parent", "parent's parent"));

  Stamped<Point> output;

  try
  {
    mTR.transformPoint( "b", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(2000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    EXPECT_FALSE("Shouldn't have gotten this exception");
  }



}



TEST(tf, ExtrapolationFromOneValue)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));
  


  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000),  "parent", "a"));

  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent", "parent"));


  Stamped<Point> output;

  bool excepted = false;
  //Past time
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  excepted = false;
  //Future one element
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(100000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  //Past multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(1), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  //Future case multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  mTR.setTransform(  StampedTransform (tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time().fromNSec(20000),  "parent", "a"));

  excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_FALSE(excepted);

}




TEST(tf, RepeatedTimes)
{
  Transformer mTR;
  tf::Quaternion qt1, qt2;
  qt1.setRPY(1,0,0);
  qt2.setRPY(1,1,0);
  mTR.setTransform(  StampedTransform (tf::Transform(qt1, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "parent", "b"));
  mTR.setTransform(  StampedTransform (tf::Transform(qt2, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "parent", "b"));

  tf::StampedTransform  output;
  try{
    mTR.lookupTransform("parent", "b" , ros::Time().fromNSec(4000), output);
    EXPECT_TRUE(!std::isnan(output.getOrigin().x()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().y()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().x()));
    EXPECT_TRUE(!std::isnan(output.getRotation().y()));
    EXPECT_TRUE(!std::isnan(output.getRotation().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().w()));
  }
  catch (...)
  {
    EXPECT_FALSE("Excetion improperly thrown");
  }
  

}

TEST(tf, frameExists)
{
  Transformer mTR;

  // test with fully qualified name
  EXPECT_FALSE(mTR.frameExists("/b"));;
  EXPECT_FALSE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));

  //test with resolveping
  EXPECT_FALSE(mTR.frameExists("b"));;
  EXPECT_FALSE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  tf::Quaternion qt1;
  qt1.setRPY(1,0,0);
  mTR.setTransform(  StampedTransform (tf::Transform(qt1, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "/parent", "/b"));



  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));


  //Test with resolveping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  tf::Quaternion qt2;
  qt2.setRPY(1,1,0);
  mTR.setTransform(  StampedTransform (tf::Transform(qt2, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "/frame", "/other"));


  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_TRUE(mTR.frameExists("/other"));
  EXPECT_TRUE(mTR.frameExists("/frame"));

  
  //Test with resolveping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_TRUE(mTR.frameExists("other"));
  EXPECT_TRUE(mTR.frameExists("frame"));

}


TEST(tf, canTransform)
{
  Transformer mTR;

  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time()));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time::now()));

  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    tf::Quaternion qt;
    qt.setRPY(1,0,0);
    mTR.setTransform(  StampedTransform (tf::Transform(qt, tf::Vector3(0,0,0)), ros::Time().fromSec(i),  "parent", "child"));
    mTR.setTransform(  StampedTransform (tf::Transform(qt, tf::Vector3(0,0,0)), ros::Time().fromSec(i),  "parent", "other_child"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);


  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", zero_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", old_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", valid_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", future_time));

  // Basic API Tests

  //Valid data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", valid_time));

  //zero data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", zero_time));

  //Old data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", old_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", old_time));

  //Future data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", future_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", future_time));

  //Same Frame should pass for all times
  EXPECT_TRUE(mTR.canTransform("child", "child", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", old_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", future_time));

  // Advanced API Tests

  // Source = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "other_child", valid_time, "child"));
  //Old data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", old_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", old_time, "other_child", valid_time, "child"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "other_child", valid_time, "child"));
  //future data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", future_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", future_time, "other_child", valid_time, "child"));

  //transforming through fixed into the past
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", old_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", old_time, "child"));
  //transforming through fixed into the future
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", future_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", future_time, "child"));

  // Target = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "parent"));
  //Old data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", old_time, "parent", valid_time, "parent"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //future data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", future_time, "parent", valid_time, "parent"));

  //transforming through fixed into the zero
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", zero_time, "parent"));
  //transforming through fixed into the past
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", old_time, "parent"));
  //transforming through fixed into the valid
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //transforming through fixed into the future
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", future_time, "parent"));

}

TEST(tf, lookupTransform)
{
  Transformer mTR;
  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    tf::Quaternion qt;
    qt.setRPY(1,0,0);
    mTR.setTransform(  StampedTransform (tf::Transform(qt, tf::Vector3(0,0,0)), ros::Time().fromSec(i),  "parent", "child"));
    mTR.setTransform(  StampedTransform (tf::Transform(qt, tf::Vector3(0,0,0)), ros::Time().fromSec(i),  "parent", "other_child"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);

  //output
  tf::StampedTransform output;

  // Basic API Tests

  try
  {
    //confirm zero length list disconnected will return true
    mTR.lookupTransform("some_frame1","some_frame1", zero_time, output);
    mTR.lookupTransform("some_frame2","some_frame2", old_time, output);
    mTR.lookupTransform("some_frame3","some_frame3", valid_time, output);
    mTR.lookupTransform("some_frame4","some_frame4", future_time, output);
    mTR.lookupTransform("child","child", future_time, output);
    mTR.lookupTransform("other_child","other_child", future_time, output);

    //Valid data should pass
    mTR.lookupTransform("child", "parent", valid_time, output);
    mTR.lookupTransform("child", "other_child", valid_time, output);
    
    //zero data should pass
    mTR.lookupTransform("child", "parent", zero_time, output);
    mTR.lookupTransform("child", "other_child", zero_time, output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception thrown");
  }
  try
  {
    //Old data should fail
    mTR.lookupTransform("child", "parent", old_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //Future data should fail
    mTR.lookupTransform("child", "parent", future_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
    
  try {
    //Same Frame should pass for all times
    mTR.lookupTransform("child", "child", zero_time, output);
    mTR.lookupTransform("child", "child", old_time, output);
    mTR.lookupTransform("child", "child", valid_time, output);
    mTR.lookupTransform("child", "child", future_time, output);
    
    // Advanced API Tests
    
    // Source = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", zero_time, "other_child", valid_time, "child", output);
    //Old data in fixed frame should pass
    mTR.lookupTransform("child", old_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", old_time, "other_child", valid_time, "child", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", valid_time, "other_child", valid_time, "child", output);
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", future_time, "other_child", valid_time, "child", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    // Target = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "parent", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
  //Old data in fixed frame should pass
  mTR.lookupTransform("child", old_time, "parent", valid_time, "parent", output);
      EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "parent", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the zero
    mTR.lookupTransform("child", valid_time, "parent", zero_time, "parent", output);
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "parent", output);
    //transforming through fixed into the valid
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }
  

  //zero time goes to latest known value for frames
  try
  {
  double epsilon = 1e-6;
    mTR.lookupTransform("a", "a", ros::Time(),output);
    EXPECT_NEAR(ros::Time().toSec(), output.stamp_.toSec(), epsilon);
    mTR.lookupTransform("child", "child", ros::Time().fromSec(15),output);
    EXPECT_NEAR(15.0, output.stamp_.toSec(), epsilon);
    mTR.lookupTransform("child", "child", ros::Time(),output);
    EXPECT_NEAR(19.0, output.stamp_.toSec(), epsilon);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }
  
}


TEST(tf, getFrameStrings)
{
  Transformer mTR;

  tf::Quaternion qt1, qt2;
  qt1.setRPY(1,0,0);
  qt2.setRPY(1,1,0);
  mTR.setTransform(  StampedTransform (tf::Transform(qt1, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "/parent", "/b"));
  std::vector <std::string> frames_string;
  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), (unsigned)2);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("parent").c_str());


  mTR.setTransform(  StampedTransform (tf::Transform(qt2, tf::Vector3(0,0,0)), ros::Time().fromNSec(4000),  "/frame", "/other"));
  
  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), (unsigned)4);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("parent").c_str());
  EXPECT_STREQ(frames_string[2].c_str(), std::string("other").c_str());
  EXPECT_STREQ(frames_string[3].c_str(), std::string("frame").c_str());

}

bool expectInvalidQuaternion(tf::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
    printf("this should have thrown\n");
    return false;
  }
  catch (tf::InvalidArgument &ex)  
  {
    return true;
  }
  catch  (...)
  {
    printf("A different type of exception was expected\n");
    return false;
  }
  return false;
}

bool expectValidQuaternion(tf::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
  }
  catch (tf::TransformException &ex)  
  {
    return false;
  }
  return true;
}

bool expectInvalidQuaternion(geometry_msgs::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
    printf("this should have thrown\n");
    return false;
  }
  catch (tf::InvalidArgument &ex)  
  {
    return true;
  }
  catch  (...)
  {
    printf("A different type of exception was expected\n");
    return false;
  }
  return false;
}

bool expectValidQuaternion(geometry_msgs::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
  }
  catch (tf::TransformException &ex)  
  {
    return false;
  }
  return true;
}


TEST(tf, assertQuaternionValid)
{
  tf::Quaternion q(1,0,0,0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setX(0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setY(1);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setZ(1);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setY(0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setW(1);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setZ(0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setZ(sqrt(2.0)/2.0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setW(sqrt(2.0)/2.0);
  EXPECT_TRUE(expectValidQuaternion(q));

  q.setZ(sqrt(2.0)/2.0 + 0.01);
  EXPECT_TRUE(expectInvalidQuaternion(q));

  q.setZ(sqrt(2.0)/2.0 - 0.01);
  EXPECT_TRUE(expectInvalidQuaternion(q));

  // check NaNs
  q.setValue(1,0,0,0);
  q.setX(0.0/0.0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setX(1.0);

  q.setY(NAN);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setY(0.0);

  q.setZ(0.0/0.0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setZ(0.0);

  q.setW(0.0/0.0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setW(0.0);

  /*    Waiting for gtest 1.1 or later
    EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  q.setX(0);
  EXPECT_THROW(tf::assertQuaternionValid(q), tf::InvalidArgument);
  q.setY(1);
  EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  */
}
TEST(tf, assertQuaternionMsgValid)
{
  geometry_msgs::Quaternion q;
  q.x = 1;//others zeroed to start

  EXPECT_TRUE(expectValidQuaternion(q));
  q.x = 0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.y = 1;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.z = 1;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.y = 0;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.w = 1;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.z = 0;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.z = sqrt(2.0)/2.0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.w = sqrt(2.0)/2.0;
  EXPECT_TRUE(expectValidQuaternion(q));

  q.z = sqrt(2.0)/2.0 + 0.01;
  EXPECT_TRUE(expectInvalidQuaternion(q));

  q.z = sqrt(2.0)/2.0 - 0.01;
  EXPECT_TRUE(expectInvalidQuaternion(q));

  // check NaNs
  q.x = 1.0; q.y = 0.0; q.z = 0.0; q.w = 0.0;
  q.x = 0.0/0.0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.x = 1.0;

  q.y = NAN;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.y = 0.0;

  q.z = 0.0/0.0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.z = 0.0;

  q.w = 0.0/0.0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.w = 0.0;

  /*    Waiting for gtest 1.1 or later
    EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  q.x = 0);
  EXPECT_THROW(tf::assertQuaternionValid(q), tf::InvalidArgument);
  q.y = 1);
  EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  */
}

TEST(data, StampedOperatorEqualEqual)
{
  tf::Pose pose0, pose1, pose0a;
  pose0.setIdentity();
  pose0a.setIdentity();
  pose1.setIdentity();
  pose1.setOrigin(tf::Vector3(1, 0, 0));
  tf::Stamped<tf::Pose> stamped_pose_reference(pose0a, ros::Time(), "frame_id");
  tf::Stamped<tf::Pose> stamped_pose0A(pose0, ros::Time(), "frame_id");
  EXPECT_TRUE(stamped_pose0A == stamped_pose_reference); // Equal
  tf::Stamped<tf::Pose> stamped_pose0B(pose0, ros::Time(), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose0B == stamped_pose_reference); // Different Frame id
  tf::Stamped<tf::Pose> stamped_pose0C(pose0, ros::Time(1.0), "frame_id");
  EXPECT_FALSE(stamped_pose0C == stamped_pose_reference); // Different Time
  tf::Stamped<tf::Pose> stamped_pose0D(pose0, ros::Time(1.0), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose0D == stamped_pose_reference); // Different frame id and time
  tf::Stamped<tf::Pose> stamped_pose0E(pose1, ros::Time(), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose0E == stamped_pose_reference); // Different pose, frame id
  tf::Stamped<tf::Pose> stamped_pose0F(pose1, ros::Time(1.0), "frame_id");
  EXPECT_FALSE(stamped_pose0F == stamped_pose_reference); // Different pose, time
  tf::Stamped<tf::Pose> stamped_pose0G(pose1, ros::Time(1.0), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose0G == stamped_pose_reference); // Different pose, frame id and time
  tf::Stamped<tf::Pose> stamped_pose0H(pose1, ros::Time(), "frame_id");
  EXPECT_FALSE(stamped_pose0H == stamped_pose_reference); // Different pose

}

TEST(data, StampedTransformOperatorEqualEqual)
{
  tf::Transform transform0, transform1, transform0a;
  transform0.setIdentity();
  transform0a.setIdentity();
  transform1.setIdentity();
  transform1.setOrigin(tf::Vector3(1, 0, 0));
  tf::StampedTransform stamped_transform_reference(transform0a, ros::Time(), "frame_id", "child_frame_id");
  tf::StampedTransform stamped_transform0A(transform0, ros::Time(), "frame_id", "child_frame_id");
  EXPECT_TRUE(stamped_transform0A == stamped_transform_reference); // Equal
  tf::StampedTransform stamped_transform0B(transform0, ros::Time(), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0B == stamped_transform_reference); // Different Frame id
  tf::StampedTransform stamped_transform0C(transform0, ros::Time(1.0), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0C == stamped_transform_reference); // Different Time
  tf::StampedTransform stamped_transform0D(transform0, ros::Time(1.0), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0D == stamped_transform_reference); // Different frame id and time
  tf::StampedTransform stamped_transform0E(transform1, ros::Time(), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0E == stamped_transform_reference); // Different transform, frame id
  tf::StampedTransform stamped_transform0F(transform1, ros::Time(1.0), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0F == stamped_transform_reference); // Different transform, time
  tf::StampedTransform stamped_transform0G(transform1, ros::Time(1.0), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0G == stamped_transform_reference); // Different transform, frame id and time
  tf::StampedTransform stamped_transform0H(transform1, ros::Time(), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0H == stamped_transform_reference); // Different transform


  //Different child_frame_id
  tf::StampedTransform stamped_transform1A(transform0, ros::Time(), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1A == stamped_transform_reference); // Equal
  tf::StampedTransform stamped_transform1B(transform0, ros::Time(), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1B == stamped_transform_reference); // Different Frame id
  tf::StampedTransform stamped_transform1C(transform0, ros::Time(1.0), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1C == stamped_transform_reference); // Different Time
  tf::StampedTransform stamped_transform1D(transform0, ros::Time(1.0), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1D == stamped_transform_reference); // Different frame id and time
  tf::StampedTransform stamped_transform1E(transform1, ros::Time(), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1E == stamped_transform_reference); // Different transform, frame id
  tf::StampedTransform stamped_transform1F(transform1, ros::Time(1.0), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1F == stamped_transform_reference); // Different transform, time
  tf::StampedTransform stamped_transform1G(transform1, ros::Time(1.0), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1G == stamped_transform_reference); // Different transform, frame id and time
  tf::StampedTransform stamped_transform1H(transform1, ros::Time(), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1H == stamped_transform_reference); // Different transform

}

TEST(data, StampedOperatorEqual)
{
 tf::Pose pose0, pose1, pose0a;
  pose0.setIdentity();
  pose1.setIdentity();
  pose1.setOrigin(tf::Vector3(1, 0, 0));
  tf::Stamped<tf::Pose> stamped_pose0(pose0, ros::Time(), "frame_id");
  tf::Stamped<tf::Pose> stamped_pose1(pose1, ros::Time(1.0), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose1 == stamped_pose0);
  stamped_pose1 = stamped_pose0;
  EXPECT_TRUE(stamped_pose1 == stamped_pose0);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); //needed for ros::TIme::now()
  ros::init(argc, argv, "tf_unittest");
  return RUN_ALL_TESTS();
}
