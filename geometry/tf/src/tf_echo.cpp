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

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

#define _USE_MATH_DEFINES
class echoListener
{
public:

  tf::TransformListener tf;

  //constructor with name
  echoListener()
  {

  }

  ~echoListener()
  {

  }

private:

};


int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);

  // Allow 2 or 3 command line arguments
  if (argc < 3 || argc > 4)
  {
    printf("Usage: tf_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return -1;
  }

  ros::NodeHandle nh("~");

  double rate_hz;
  if (argc == 4)
  {
    // read rate from command line
    rate_hz = atof(argv[3]);
  }
  else
  {
    // read rate parameter
    nh.param("rate", rate_hz, 1.0);
  }
  if (rate_hz <= 0.0)
  {
    std::cerr << "Echo rate must be > 0.0\n";
    return -1;
  }
  ros::Rate rate(rate_hz);

  int precision(3);
  if (nh.getParam("precision", precision))
  {
    if (precision < 1)
    {
      std::cerr << "Precision must be > 0\n";
      return -1;
    }
    printf("Precision default value was overriden, new value: %d\n", precision);
  }

  //Instantiate a local listener
  echoListener echoListener;


  std::string source_frameid = std::string(argv[1]);
  std::string target_frameid = std::string(argv[2]);

  // Wait for up to one second for the first transforms to become avaiable. 
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(nh.ok())
    {
      try
      {
        tf::StampedTransform echo_transform;
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
        std::cout.precision(precision);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
        double yaw, pitch, roll;
        echo_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = echo_transform.getRotation();
        tf::Vector3 v = echo_transform.getOrigin();
        std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                  << q.getZ() << ", " << q.getW() << "]" << std::endl
                  << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                  << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;

        //print transform
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      rate.sleep();
    }

  return 0;
}

