#include <gtest/gtest.h>
#include <tf/tf.h>
#include <sys/time.h>

#include "tf/LinearMath/Vector3.h"

using namespace tf;

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};



TEST(tf, SignFlipExtrapolate)
{
  double epsilon = 1e-3;

  double truex, truey, trueyaw1, trueyaw2;

  truex = 5.220;
  truey = 1.193;
  trueyaw1 = 2.094;
  trueyaw2 = 2.199;
  ros::Time ts0;
  ts0.fromSec(46.6);
  ros::Time ts1;
  ts1.fromSec(46.7);
  ros::Time ts2;
  ts2.fromSec(46.8);
  
  TransformStorage tout;
  double yaw, pitch, roll;

  TransformStorage t0(StampedTransform
                      (tf::Transform(tf::Quaternion(0.000, 0.000,  -0.8386707128751809, 0.5446388118427071),
                                   tf::Vector3(1.0330764266905630, 5.2545257423922198, -0.000)),
                       ts0, "odom", "other0"), 3);
  TransformStorage t1(StampedTransform
                      (tf::Transform(tf::Quaternion(0.000, 0.000,  0.8660255375641606, -0.4999997682866531),
                                   tf::Vector3(1.5766646418987809, 5.1177550046707436, -0.000)),
                       ts1, "odom", "other1"), 3);
  TransformStorage t2(StampedTransform
                      (tf::Transform(tf::Quaternion(0.000, 0.000, 0.8910066733792211, -0.4539902069358919),
                                   tf::Vector3(2.1029791754869160, 4.9249128183465967, -0.000)),
                       ts2, "odom", "other2"), 3);

  tf::TimeCache tc;
  tf::Transform res;

  tc.interpolate(t0, t1, ts1, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw1, epsilon);

  tc.interpolate(t0, t1, ts2, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw2, epsilon);

  tc.interpolate(t1, t2, ts2, tout);
  res = tout.inverse();
  res.getBasis().getEulerZYX(yaw,pitch,roll);

  EXPECT_NEAR(res.getOrigin().x(), truex, epsilon);
  EXPECT_NEAR(res.getOrigin().y(), truey, epsilon);
  EXPECT_NEAR(yaw, trueyaw2, epsilon);
}



/** @todo Make this actually Assert something */

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

