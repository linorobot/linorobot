#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <sys/time.h>

#include "tf/LinearMath/Vector3.h"

using namespace tf;


// The fixture for testing class Foo.
class TransformTwistLinearTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TransformTwistLinearTest() {
    double x = -.1;
    double y = 0;
    double z = 0;
    for (double t = 0.1; t <= 6; t += 0.1)
    {
      if      (t < 1) x += 1;
      else if (t < 2) y += 1;
      else if (t < 3) x -= 1;
      else if (t < 4) y -= 1;
      else if (t < 5) z += 1;
      else            z -= 1;
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z)), ros::Time(t), "foo", "bar"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,0,0)), ros::Time(t), "foo", "stationary_offset_child"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,1)), ros::Time(t), "stationary_offset_parent", "foo"));
    }

    // You can do set-up work for each test here.
    tw_.header.stamp = ros::Time();
    tw_.header.frame_id = "bar";
    tw_.twist.linear.x =  1;
    tw_.twist.linear.y =  2;
    tw_.twist.linear.z =  3;
    tw_.twist.angular.x =  0;
    tw_.twist.angular.y =  0;
    tw_.twist.angular.z =  0;
  
  }

  virtual ~TransformTwistLinearTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for LinearVelocity.

  tf::TransformListener tf_;

  geometry_msgs::TwistStamped tw_;

};

// The fixture for testing class Foo.
class TransformTwistAngularTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TransformTwistAngularTest() {
    double x = -.1;
    double y = 0;
    double z = 0;
    for (double t = 0.1; t < 6; t += 0.1)
    {
      if      (t < 1) x += .1;
      else if (t < 2) x -= .1;
      else if (t < 3) y += .1;
      else if (t < 4) y -= .1;
      else if (t < 5) z += .1;
      else            z -= .1;
      tf_.setTransform(StampedTransform(tf::Transform(tf::createQuaternionFromRPY(x, y, z), tf::Vector3(0,0,0)), ros::Time(t), "foo", "bar"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,0,0)), ros::Time(t), "foo", "stationary_offset_child"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,-1)), ros::Time(t), "stationary_offset_parent", "foo"));
    }

    // You can do set-up work for each test here.
    // You can do set-up work for each test here.
    tw_x.header.stamp = ros::Time();
    tw_x.header.frame_id = "bar";
    tw_x.twist.angular.x =  1;

    tw_y.header.stamp = ros::Time();
    tw_y.header.frame_id = "bar";
    tw_y.twist.angular.y =  1;

    tw_z.header.stamp = ros::Time();
    tw_z.header.frame_id = "bar";
    tw_z.twist.angular.z =  1;
  }

  virtual ~TransformTwistAngularTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for AngularVelocity.

  tf::TransformListener tf_;
  geometry_msgs::TwistStamped tw_x, tw_y, tw_z;
};

TEST_F(TransformTwistLinearTest, LinearVelocityToThreeFrames)
{
  geometry_msgs::TwistStamped tw;
  geometry_msgs::TwistStamped tw_in =tw_;

  std::vector<std::string> offset_frames;

  offset_frames.push_back("foo");
  offset_frames.push_back("stationary_offset_child");
  offset_frames.push_back("stationary_offset_parent");


  for (std::vector<std::string>::iterator it = offset_frames.begin(); it != offset_frames.end(); ++it)
  {
    try
    {
      tw_in.header.stamp = ros::Time(0.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(11.0, tw.twist.linear.x);
      EXPECT_FLOAT_EQ(2.0 , tw.twist.linear.y);
      EXPECT_FLOAT_EQ(3.0, tw.twist.linear.z);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.x);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.y);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.z);
  
      tw_in.header.stamp = ros::Time(1.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(1.0, tw.twist.linear.x);
      EXPECT_FLOAT_EQ(12.0, tw.twist.linear.y);
      EXPECT_FLOAT_EQ(3.0, tw.twist.linear.z);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.x);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.y);
      EXPECT_FLOAT_EQ(0.0, tw.twist.angular.z);

      tw_in.header.stamp = ros::Time(2.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(-9.0, tw.twist.linear.x);
      EXPECT_FLOAT_EQ(2.0, tw.twist.linear.y);
      EXPECT_FLOAT_EQ(3.0, tw.twist.linear.z);
      EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

      tw_in.header.stamp = ros::Time(3.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(tw.twist.linear.x, 1.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.y, -8.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.z, 3.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

      tw_in.header.stamp = ros::Time(4.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(tw.twist.linear.x, 1.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.y, 2.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.z, 13.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);

      tw_in.header.stamp = ros::Time(5.5);
      tf_.transformTwist(*it, tw_in, tw);
      EXPECT_FLOAT_EQ(tw.twist.linear.x, 1.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.y, 2.0);
      EXPECT_FLOAT_EQ(tw.twist.linear.z, -7.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(tw.twist.angular.z, 0.0);
    }
    catch(tf::TransformException &ex)
    {
      EXPECT_STREQ("", ex.what());
    }
  }
};

TEST_F(TransformTwistAngularTest, AngularVelocityAlone)
{
  double epsilon = 1e-14;
  geometry_msgs::TwistStamped tw;
  geometry_msgs::TwistStamped tw_in;

  try
  {
    //tf_.lookupVelocity("foo", "bar", ros::Time(0.5), ros::Duration(0.1), tw);
    tw_in =tw_x;
    tw_in.header.stamp = ros::Time(0.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 2.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);
  
    //tf_.lookupVelocity("foo", "bar", ros::Time(1.5), ros::Duration(0.1), tw);
    tw_in =tw_x;
    tw_in.header.stamp = ros::Time(1.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    //    tf_.lookupVelocity("foo", "bar", ros::Time(2.5), ros::Duration(0.1), tw);
    tw_in =tw_y;
    tw_in.header.stamp = ros::Time(2.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x,  0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 2.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    //tf_.lookupVelocity("foo", "bar", ros::Time(3.5), ros::Duration(0.1), tw);
    tw_in =tw_y;
    tw_in.header.stamp = ros::Time(3.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    //tf_.lookupVelocity("foo", "bar", ros::Time(4.5), ros::Duration(0.1), tw);
    tw_in =tw_z;
    tw_in.header.stamp = ros::Time(4.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 2.0, epsilon);

    //    tf_.lookupVelocity("foo", "bar", ros::Time(5.5), ros::Duration(0.1), tw);
    tw_in =tw_z;
    tw_in.header.stamp = ros::Time(5.5);
    tf_.transformTwist("foo", tw_in, tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
};
/*
TEST_F(TransformTwistAngularTest, AngularVelocityOffsetChildFrameInX)
{
  double epsilon = 1e-14;
  geometry_msgs::TwistStamped tw;
  try
  {
    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(0.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);
  
    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(1.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(2.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x,  0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(3.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(4.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 1.0, epsilon);

    tf_.lookupVelocity("stationary_offset_child", "bar", ros::Time(5.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, -1.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
};

TEST_F(TransformTwistAngularTest, AngularVelocityOffsetParentFrameInZ)
{
  double epsilon = 1e-14;
  geometry_msgs::TwistStamped tw;
  try
  {
    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(0.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);
  
    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(1.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(2.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(3.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, -1.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 0.0, epsilon);

    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(4.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, 1.0, epsilon);

    tf_.lookupVelocity("stationary_offset_parent", "bar", ros::Time(5.5), ros::Duration(0.1), tw);
    EXPECT_NEAR(tw.twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(tw.twist.angular.z, -1.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
};
*/

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testing_transform_twist");
  return RUN_ALL_TESTS();
}

