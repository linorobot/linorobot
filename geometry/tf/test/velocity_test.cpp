#include <gtest/gtest.h>
#include <tf/tf.h>
#include <sys/time.h>

#include "tf/LinearMath/Vector3.h"

using namespace tf;


// The fixture for testing class Foo.
class LinearVelocitySquareTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  LinearVelocitySquareTest() {
    double x = -.1;
    double y = 0;
    double z = 0;
    for (double t = 0.1; t <= 6; t += 0.1)
    {
      if      (t < 1) x += .1;
      else if (t < 2) y += .1;
      else if (t < 3) x -= .1;
      else if (t < 4) y -= .1;
      else if (t < 5) z += .1;
      else            z -= .1;
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(x, y, z)), ros::Time(t), "foo", "bar"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,0,0)), ros::Time(t), "foo", "stationary_offset_child"));
      tf_.setTransform(StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,1)), ros::Time(t), "stationary_offset_parent", "foo"));
    }

    // You can do set-up work for each test here.
  }

  virtual ~LinearVelocitySquareTest() {
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

  tf::Transformer tf_;

};

// The fixture for testing class Foo.
class AngularVelocitySquareTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  AngularVelocitySquareTest() {
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
  }

  virtual ~AngularVelocitySquareTest() {
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

  tf::Transformer tf_;

};

TEST_F(LinearVelocitySquareTest, LinearVelocityToThreeFrames)
{
  geometry_msgs::Twist twist;
  std::vector<std::string> offset_frames;

  offset_frames.push_back("foo");
  offset_frames.push_back("stationary_offset_child");
  offset_frames.push_back("stationary_offset_parent");


  for (std::vector<std::string>::iterator it = offset_frames.begin(); it != offset_frames.end(); ++it)
  {
    try
    {
      tf_.lookupTwist("bar", *it, ros::Time(0.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, 1.0);
      EXPECT_FLOAT_EQ(twist.linear.y, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.z, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);
  
      tf_.lookupTwist("bar", *it, ros::Time(1.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.y, 1.0);
      EXPECT_FLOAT_EQ(twist.linear.z, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);

      tf_.lookupTwist("bar", *it, ros::Time(2.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, -1.0);
      EXPECT_FLOAT_EQ(twist.linear.y, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.z, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);

      tf_.lookupTwist("bar", *it, ros::Time(3.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.y, -1.0);
      EXPECT_FLOAT_EQ(twist.linear.z, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);

      tf_.lookupTwist("bar", *it, ros::Time(4.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.y, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.z, 1.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);

      tf_.lookupTwist("bar", *it, ros::Time(5.5), ros::Duration(0.1), twist);
      EXPECT_FLOAT_EQ(twist.linear.x, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.y, 0.0);
      EXPECT_FLOAT_EQ(twist.linear.z, -1.0);
      EXPECT_FLOAT_EQ(twist.angular.x, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.y, 0.0);
      EXPECT_FLOAT_EQ(twist.angular.z, 0.0);
    }
    catch(tf::TransformException &ex)
    {
      EXPECT_STREQ("", ex.what());
    }
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityAlone)
{
  double epsilon = 1e-6;
  geometry_msgs::Twist twist;
  try
  {
    tf_.lookupTwist("bar", "foo", ros::Time(0.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);
  
    tf_.lookupTwist("bar", "foo", ros::Time(1.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "foo", ros::Time(2.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x,  0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "foo", ros::Time(3.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "foo", ros::Time(4.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 1.0, epsilon);

    tf_.lookupTwist("bar", "foo", ros::Time(5.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, -1.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityOffsetChildFrameInX)
{
  double epsilon = 1e-6;
  geometry_msgs::Twist twist;
  try
  {
    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(0.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);
  
    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(1.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(2.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x,  0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(3.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(4.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 1.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 1.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_child", ros::Time(5.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, -1.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, -1.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityOffsetParentFrameInZ)
{
  double epsilon = 1e-6;
  geometry_msgs::Twist twist;
  try
  {
    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(0.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, -1.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);
  
    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(1.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 1.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(2.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 1.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(3.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, -1.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 0.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(4.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, 1.0, epsilon);

    tf_.lookupTwist("bar", "stationary_offset_parent", ros::Time(5.5), ros::Duration(0.1), twist);
    EXPECT_NEAR(twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.angular.z, -1.0, epsilon);
  }
  catch(tf::TransformException &ex)
  {
    EXPECT_STREQ("", ex.what());
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

