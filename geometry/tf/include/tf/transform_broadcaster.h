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


/** \author Tully Foote */

#ifndef TF_TRANSFORMBROADCASTER_H
#define TF_TRANSFORMBROADCASTER_H

#include "tf/tf.h"
#include "tf/tfMessage.h"

#include <tf2_ros/transform_broadcaster.h>


namespace tf
{


/** \brief This class provides an easy way to publish coordinate frame transform information.  
 * It will handle all the messaging and stuffing of messages.  And the function prototypes lay out all the 
 * necessary data needed for each message.  */

class TransformBroadcaster{
public:
  /** \brief Constructor (needs a ros::Node reference) */
  TransformBroadcaster();

  /** \brief Send a StampedTransform 
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const StampedTransform & transform);

  /** \brief Send a vector of StampedTransforms 
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const std::vector<StampedTransform> & transforms);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const geometry_msgs::TransformStamped & transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);

private:

  tf2_ros::TransformBroadcaster tf2_broadcaster_;

};

}

#endif //TF_TRANSFORMBROADCASTER_H
