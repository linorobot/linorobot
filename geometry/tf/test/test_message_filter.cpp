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

/** \author Josh Faust */

#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"

#include <gtest/gtest.h>

using namespace tf;

class Notification
{
public:
	Notification(int expected_count)
	: count_(0)
	, expected_count_(expected_count)
	, failure_count_(0)
	{
	}

	void notify(const geometry_msgs::PointStamped::ConstPtr& message)
	{
		++count_;
	}

	void failure(const geometry_msgs::PointStamped::ConstPtr& message, FilterFailureReason reason)
	{
	  ++failure_count_;
	}

	int count_;
	int expected_count_;
	int failure_count_;
};

TEST(MessageFilter, noTransforms)
{
  tf::TransformListener tf_client;
	Notification n(1);
	MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
	filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

	geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
	msg->header.stamp = ros::Time::now();
	msg->header.frame_id = "frame2";
	filter.add(msg);

	EXPECT_EQ(0, n.count_);
}

TEST(MessageFilter, noTransformsSameFrame)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "frame1";
  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, preexistingTransforms)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

	ros::Time stamp = ros::Time::now();
	tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
	tf_client.setTransform(transform);

	geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
	msg->header.stamp = stamp;
	msg->header.frame_id = "frame2";

	filter.add(msg);

	EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, postTransforms)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

	ros::Time stamp = ros::Time::now();

	geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

	EXPECT_EQ(0, n.count_);

	tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
	tf_client.setTransform(transform);

	ros::WallDuration(0.1).sleep();
	ros::spinOnce();

	EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, queueSize)
{
  tf::TransformListener tf_client;
  Notification n(10);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 10);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

	ros::Time stamp = ros::Time::now();

	for (int i = 0; i < 20; ++i)
	{
	  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
    msg->header.stamp = stamp;
    msg->header.frame_id = "frame2";

    filter.add(msg);
	}

	EXPECT_EQ(0, n.count_);
	EXPECT_EQ(10, n.failure_count_);

	tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
	tf_client.setTransform(transform);

	ros::WallDuration(0.1).sleep();
	ros::spinOnce();

	EXPECT_EQ(10, n.count_);
}

TEST(MessageFilter, setTargetFrame)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
	filter.setTargetFrame("frame1000");

	ros::Time stamp = ros::Time::now();
  tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1000", "frame2");
  tf_client.setTransform(transform);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

	filter.add(msg);


	EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, multipleTargetFrames)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  std::vector<std::string> target_frames;
  target_frames.push_back("frame1");
  target_frames.push_back("frame2");
	filter.setTargetFrames(target_frames);

	ros::Time stamp = ros::Time::now();
  tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame3");
  tf_client.setTransform(transform);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame3";
  filter.add(msg);

  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

	EXPECT_EQ(0, n.count_); // frame1->frame3 exists, frame2->frame3 does not (yet)

	//ros::Time::setNow(ros::Time::now() + ros::Duration(1.0));

	transform.child_frame_id_ = "frame2";
	tf_client.setTransform(transform);

	ros::WallDuration(0.1).sleep();
	ros::spinOnce();

	EXPECT_EQ(1, n.count_);  // frame2->frame3 now exists
}

TEST(MessageFilter, tolerance)
{
  ros::Duration offset(0.2);
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
  filter.setTolerance(offset);

	ros::Time stamp = ros::Time::now();
  tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

	EXPECT_EQ(0, n.count_); //No return due to lack of space for offset

	//ros::Time::setNow(ros::Time::now() + ros::Duration(0.1));

	transform.stamp_ += offset*1.1;
	tf_client.setTransform(transform);

	ros::WallDuration(0.1).sleep();
	ros::spinOnce();

	EXPECT_EQ(1, n.count_); // Now have data for the message published earlier

	msg->header.stamp = stamp + offset;
	filter.add(msg);

	EXPECT_EQ(1, n.count_); // Latest message is off the end of the offset
}

// TODO: re-enable once ROS 0.7.3 is out and the Timer issues have been fixed
#if 0
TEST(MessageFilter, maxRate)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1, ros::NodeHandle(), ros::Duration(1.0));
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  ros::Time stamp = ros::Time::now();
  tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  stamp += ros::Duration(0.1);
  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);

  transform.stamp_ = stamp;
  tf_client.setTransform(transform);

  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  EXPECT_EQ(0, n.count_);

  //ros::Time::setNow(ros::Time::now() + ros::Duration(1.0));
  tf_client.setTransform(transform);

  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  EXPECT_EQ(1, n.count_);
}
#endif

TEST(MessageFilter, outTheBackFailure)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  ros::Time stamp = ros::Time::now();
  tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  transform.stamp_ = stamp + ros::Duration(10000);
  tf_client.setTransform(transform);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}

TEST(MessageFilter, emptyFrameIDFailure)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(tf_client, "frame1", 1);
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.frame_id = "";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}

TEST(MessageFilter, removeCallback)
{
  // Callback queue in separate thread
  ros::CallbackQueue cbqueue;
  ros::AsyncSpinner spinner(1, &cbqueue);
  ros::NodeHandle threaded_nh;
  threaded_nh.setCallbackQueue(&cbqueue);

  // TF filters; no data needs to be published
  boost::scoped_ptr<tf::TransformListener> tf_listener;
  boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PointStamped> > tf_filter;

  spinner.start();
  for (int i = 0; i < 3; ++i) {
    tf_listener.reset(new tf::TransformListener());
    // Have callback fire at high rate to increase chances of race condition
    tf_filter.reset(
      new tf::MessageFilter<geometry_msgs::PointStamped>(*tf_listener,
                                                         "map", 5, threaded_nh,
                                                         ros::Duration(0.000001)));

    // Sleep and reset; sleeping increases chances of race condition
    ros::Duration(0.001).sleep();
    tf_filter.reset();
    tf_listener.reset();
  }
  spinner.stop();
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::Time::setNow(ros::Time());
	ros::init(argc, argv, "test_message_filter");
	ros::NodeHandle nh;

	int ret = RUN_ALL_TESTS();

	return ret;
}
