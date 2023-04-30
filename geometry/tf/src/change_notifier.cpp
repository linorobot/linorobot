/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "xmlrpcpp/XmlRpcValue.h"

class FramePair
{
public:
  FramePair(const std::string& source_frame, const std::string& target_frame, double translational_update_distance, double angular_update_distance) : 
    source_frame_(source_frame), 
    target_frame_(target_frame),
    translational_update_distance_(translational_update_distance),
    angular_update_distance_(angular_update_distance)
  {
    pose_in_ = tf::Stamped<tf::Pose>(tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), ros::Time(), source_frame_);
  }

public:
  std::string source_frame_;
  std::string target_frame_;

  tf::Stamped<tf::Pose> pose_in_;
  tf::Stamped<tf::Pose> pose_out_;
  tf::Stamped<tf::Pose> last_sent_pose_;
  
  double translational_update_distance_;
  double angular_update_distance_;
};

bool getFramePairs(const ros::NodeHandle& local_node, std::vector<FramePair>& frame_pairs, double default_translational_update_distance, double default_angular_update_distance)
{
  XmlRpc::XmlRpcValue frame_pairs_param;
  if (!local_node.getParam("frame_pairs", frame_pairs_param))
  {
    // No frame_pairs parameter provided. Default to base_link->map.
    frame_pairs.push_back(FramePair("base_link", "map", default_translational_update_distance, default_angular_update_distance));
    return true;
  }

  if (frame_pairs_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Expecting a list for frame_pairs parameter");
    return false;
  }
  for (int i = 0; i < frame_pairs_param.size(); i++)
  {
    XmlRpc::XmlRpcValue frame_pair_param = frame_pairs_param[i];
    if (frame_pair_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("frame_pairs must be specified as maps, but they are XmlRpcType: %d", frame_pair_param.getType());
      return false;
    }

    // Get the source_frame
    if (!frame_pair_param.hasMember("source_frame"))
    {
      ROS_ERROR("frame_pair does not specified source_frame");
      return false;
    }
    XmlRpc::XmlRpcValue source_frame_param = frame_pair_param["source_frame"];
    if (source_frame_param.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("source_frame must be a string, but it is XmlRpcType: %d", source_frame_param.getType());
      return false;
    }
    std::string source_frame = source_frame_param;

    // Get the target_frame
    if (!frame_pair_param.hasMember("target_frame"))
    {
      ROS_ERROR("frame_pair does not specified target_frame");
      return false;
    }
    XmlRpc::XmlRpcValue target_frame_param = frame_pair_param["target_frame"];
    if (target_frame_param.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("target_frame must be a string, but it is XmlRpcType: %d", target_frame_param.getType());
      return false;
    }
    std::string target_frame = target_frame_param;

    // Get the (optional) translational_update_distance
    double translational_update_distance = default_translational_update_distance;
    if (frame_pair_param.hasMember("translational_update_distance"))
    {
      XmlRpc::XmlRpcValue translational_update_distance_param = frame_pair_param["translational_update_distance"];
      if (translational_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          translational_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("translational_update_distance must be either an integer or a double, but it is XmlRpcType: %d", translational_update_distance_param.getType());
        return false;
      }
      translational_update_distance = translational_update_distance_param;
    }

    // Get the (optional) angular_update_distance
    double angular_update_distance = default_angular_update_distance;
    if (frame_pair_param.hasMember("angular_update_distance"))
    {
      XmlRpc::XmlRpcValue angular_update_distance_param = frame_pair_param["angular_update_distance"];
      if (angular_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          angular_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("angular_update_distance must be either an integer or a double, but it is XmlRpcType: %d", angular_update_distance_param.getType());
        return false;
      }
      angular_update_distance = angular_update_distance_param;
    }

    ROS_INFO("Notifying change on %s -> %s (translational update distance: %.4f, angular update distance: %.4f)", source_frame.c_str(), target_frame.c_str(), translational_update_distance, angular_update_distance);

    frame_pairs.push_back(FramePair(source_frame, target_frame, translational_update_distance, angular_update_distance));
  }

  return true;
}

/** This is a program to provide notifications of changes of state within tf 
 * It was written for providing an easy way to on demand update a web graphic of 
 * where the robot is located.  It's not designed or recommended for use in live 
 * operation for feedback.  */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "change_notifier", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  ros::NodeHandle local_node("~");

  double polling_frequency, translational_update_distance, angular_update_distance;
  local_node.param(std::string("polling_frequency"),             polling_frequency,             10.0);
  local_node.param(std::string("translational_update_distance"), translational_update_distance,  0.10);
  local_node.param(std::string("angular_update_distance"),       angular_update_distance,        0.10);

  std::vector<FramePair> frame_pairs;
  if (!getFramePairs(local_node, frame_pairs, translational_update_distance, angular_update_distance))
    return 1;

  tf::TransformListener tfl(node);

  // Advertise the service
  ros::Publisher pub = node.advertise<tf::tfMessage>("tf_changes", 1, true);

  while (node.ok())
  {
    try
    {
      tf::tfMessage msg;

      for (std::vector<FramePair>::iterator i = frame_pairs.begin(); i != frame_pairs.end(); i++)
      {
        FramePair& fp = *i;

        tfl.transformPose(fp.target_frame_, fp.pose_in_, fp.pose_out_);

        const tf::Vector3&    origin   = fp.pose_out_.getOrigin();
        const tf::Quaternion& rotation = fp.pose_out_.getRotation();

        if (origin.distance(fp.last_sent_pose_.getOrigin()) > fp.translational_update_distance_ ||
            rotation.angle(fp.last_sent_pose_.getRotation()) > fp.angular_update_distance_)
        {
          fp.last_sent_pose_ = fp.pose_out_;

          tf::StampedTransform stampedTf(tf::Transform(rotation, origin), fp.pose_out_.stamp_, "/" + fp.target_frame_, "/" + fp.source_frame_);
          geometry_msgs::TransformStamped msgtf;
          transformStampedTFToMsg(stampedTf, msgtf);
          msg.transforms.push_back(msgtf);
        }
      }

      if (msg.transforms.size() > 0)
        pub.publish(msg);
    }
    catch (tf::TransformException& ex)
    {
      ROS_DEBUG("Exception: %s\n", ex.what());
    }
    
    // Sleep until next polling
    if (polling_frequency > 0)
      ros::Duration().fromSec(1.0 / polling_frequency).sleep();
  }

  return 0;
}
