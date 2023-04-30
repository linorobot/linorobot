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

#include "tf/transform_broadcaster.h"
#include "ros/ros.h"

class testBroadcaster 
{
public:
  //constructor
  testBroadcaster() : count(0), count1(0){};
  //Clean up ros connections
  ~testBroadcaster() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;


  // A function to call to send data periodically
  void test () {
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,2,3)), ros::Time().fromSec(1), "frame2", "frame1"));

    if (count > 9000)
    {
      count = 0;
      std::cerr<<"Counter 0 rolledover at 9000"<< std::endl;
    }
    else
      count ++;
    //std::cerr<<count<<std::endl;
  }

  // A function to call to send data periodically
  void test_vector () {
    std::vector<tf::StampedTransform> vec;
    vec.push_back(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,2,3)), ros::Time().fromSec(1), "vframe2", "vframe1"));
    vec.push_back(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(1,2,3)), ros::Time().fromSec(1), "vframe1", "vframe0"));
    broadcaster.sendTransform(vec);

    if (count1 > 9000)
    {
      count1 = 0;
      std::cerr<<"Counter 1 rolledover at 9000"<< std::endl;
    }
    else
      count1 ++;
    //std::cerr<<count1<<std::endl;
  }
private:
  int count;
  int count1;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "testBroadcaster", ros::init_options::AnonymousName);

  //Construct/initialize the server
  testBroadcaster myTestBroadcaster;

  ros::NodeHandle node; 
  while(ros::ok())//Check if a Ctrl-C or other shutdown command has been recieved
  {
      //Send some data
      myTestBroadcaster.test();
      myTestBroadcaster.test_vector();
      usleep(1000);
  }

  return 0;
}

