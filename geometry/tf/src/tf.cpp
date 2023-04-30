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

#include "tf/tf.h"
#include <sys/time.h>
#include "ros/assert.h"
#include "ros/ros.h"
#include <angles/angles.h>

using namespace tf;

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
// In C++0x this must be initialized here #5401
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;


enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

struct CanTransformAccum
{
  CompactFrameID gather(TimeCache* cache, ros::Time time, std::string* error_string)
  {
    return cache->getParent(time, error_string);
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
  }

  TransformStorage st;
};

struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCache* cache, ros::Time time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  ros::Time time;
  tf::Quaternion source_to_top_quat;
  tf::Vector3 source_to_top_vec;
  tf::Quaternion target_to_top_quat;
  tf::Vector3 target_to_top_vec;

  tf::Quaternion result_quat;
  tf::Vector3 result_vec;
};


std::string assert_resolved(const std::string& prefix, const std::string& frame_id)
{
  ROS_DEBUG("tf::assert_resolved just calls tf::resolve");
  return tf::resolve(prefix, frame_id);
}

std::string tf::resolve(const std::string& prefix, const std::string& frame_name)
{
  //  printf ("resolveping prefix:%s with frame_name:%s\n", prefix.c_str(), frame_name.c_str());
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      return strip_leading_slash(frame_name);
    }
  if (prefix.size() > 0)
  {
    if (prefix[0] == '/')
    {
      std::string composite = strip_leading_slash(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }
    else
    {
      std::string composite;
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }

  }
  else
 {
    std::string composite;
    composite.append(frame_name);
    return composite;
  }
}


std::string tf::strip_leading_slash(const std::string& frame_name)
{
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      std::string shorter = frame_name;
      shorter.erase(0,1);
      return shorter;
    }
  
  return frame_name;
}


Transformer::Transformer(bool interpolating,
                                ros::Duration cache_time):
  fall_back_to_wall_time_(false),
  tf2_buffer_ptr_(std::make_shared<tf2_ros::Buffer>(cache_time))
{

}

Transformer::~Transformer()
{

}


void Transformer::clear()
{
  tf2_buffer_ptr_->clear();
}


bool Transformer::setTransform(const StampedTransform& transform, const std::string& authority)
{
  geometry_msgs::TransformStamped msgtf;
  transformStampedTFToMsg(transform, msgtf);
  return tf2_buffer_ptr_->setTransform(msgtf, authority);
  
}


void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                     const ros::Time& time, StampedTransform& transform) const
{
  geometry_msgs::TransformStamped output = 
    tf2_buffer_ptr_->lookupTransform(strip_leading_slash(target_frame),
                                strip_leading_slash(source_frame), time);
  transformStampedMsgToTF(output, transform);
  return;
}


void Transformer::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame, StampedTransform& transform) const
{
  geometry_msgs::TransformStamped output = 
    tf2_buffer_ptr_->lookupTransform(strip_leading_slash(target_frame), target_time,
                                strip_leading_slash(source_frame), source_time,
                                strip_leading_slash(fixed_frame));
  transformStampedMsgToTF(output, transform);
}


void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame,
                              const ros::Time& time, const ros::Duration& averaging_interval, 
                              geometry_msgs::Twist& twist) const
{
  // ref point is origin of tracking_frame, ref_frame = obs_frame
  lookupTwist(tracking_frame, observation_frame, observation_frame, tf::Point(0,0,0), tracking_frame, time, averaging_interval, twist);
}



void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                 const tf::Point & reference_point, const std::string& reference_point_frame, 
                 const ros::Time& time, const ros::Duration& averaging_interval, 
                 geometry_msgs::Twist& twist) const
{
  
  ros::Time latest_time, target_time;
  getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); ///\TODO check time on reference point too

  if (ros::Time() == time)
    target_time = latest_time;
  else
    target_time = time;

  ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);
  
  ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
  StampedTransform start, end;
  lookupTransform(observation_frame, tracking_frame, start_time, start);
  lookupTransform(observation_frame, tracking_frame, end_time, end);


  tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
  tfScalar ang = quat_temp.getAngle();
  
  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


  tf::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(), 
                       (delta_y)/corrected_averaging_interval.toSec(),
                       (delta_z)/corrected_averaging_interval.toSec());
  tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


  // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)


  //correct for the position of the reference frame
  tf::StampedTransform inverse;
  lookupTransform(reference_frame,tracking_frame,  target_time, inverse);
  tf::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);


  //Rereference the twist about a new reference point
  // Start by computing the original reference point in the reference frame:
  tf::Stamped<tf::Point> rp_orig(tf::Point(0,0,0), target_time, tracking_frame);
  transformPoint(reference_frame, rp_orig, rp_orig);
  // convert the requrested reference point into the right frame
  tf::Stamped<tf::Point> rp_desired(reference_point, target_time, reference_point_frame);
  transformPoint(reference_frame, rp_desired, rp_desired);
  // compute the delta
  tf::Point delta = rp_desired - rp_orig;
  // Correct for the change in reference point 
  out_vel = out_vel + out_rot * delta;
  // out_rot unchanged   

  /*
    printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n", 
         out_rot.x(),out_rot.y(),out_rot.z(),
         out_vel.x(),out_vel.y(),out_vel.z());
  */   

  twist.linear.x =  out_vel.x();
  twist.linear.y =  out_vel.y();
  twist.linear.z =  out_vel.z();
  twist.angular.x =  out_rot.x();
  twist.angular.y =  out_rot.y();
  twist.angular.z =  out_rot.z();

}

bool Transformer::waitForTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  return tf2_buffer_ptr_->canTransform(strip_leading_slash(target_frame),
                                  strip_leading_slash(source_frame), time, timeout, error_msg);
}


bool Transformer::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const ros::Time& time, std::string* error_msg) const
{
  return tf2_buffer_ptr_->canTransform(strip_leading_slash(target_frame),
                                  strip_leading_slash(source_frame), time, error_msg);
}


bool Transformer::canTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                               const ros::Time& source_time, const std::string& fixed_frame,
                               std::string* error_msg) const
{
  return tf2_buffer_ptr_->canTransform(strip_leading_slash(target_frame), target_time,
                                  strip_leading_slash(source_frame), source_time,
                                  strip_leading_slash(fixed_frame), error_msg);
}

bool Transformer::waitForTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                                   const ros::Time& source_time, const std::string& fixed_frame,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  return tf2_buffer_ptr_->canTransform(strip_leading_slash(target_frame), target_time,
                                  strip_leading_slash(source_frame), source_time,
                                  strip_leading_slash(fixed_frame), timeout, error_msg);
}


bool Transformer::getParent(const std::string& frame_id, ros::Time time, std::string& parent) const
{
  return tf2_buffer_ptr_->_getParent(strip_leading_slash(frame_id), time, parent);
}


bool Transformer::frameExists(const std::string& frame_id_str) const
{
  return tf2_buffer_ptr_->_frameExists(strip_leading_slash(frame_id_str));
}

void Transformer::setExtrapolationLimit(const ros::Duration& distance)
{
  ROS_WARN("Transformer::setExtrapolationLimit is deprecated and does not do anything");
}


struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

int Transformer::getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const
{
  CompactFrameID target_id = tf2_buffer_ptr_->_lookupFrameNumber(strip_leading_slash(target_frame));
  CompactFrameID source_id = tf2_buffer_ptr_->_lookupFrameNumber(strip_leading_slash(source_frame));

  return tf2_buffer_ptr_->_getLatestCommonTime(source_id, target_id, time, error_string);
}


//@todo - Fix this to work with new data structures
void Transformer::chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
{
  tf2_buffer_ptr_->_chainAsVector(target_frame, target_time,
                             source_frame, source_time, 
                             fixed_frame, output);
}

std::string Transformer::allFramesAsString() const
{
  return tf2_buffer_ptr_->allFramesAsString();
}

std::string Transformer::allFramesAsDot(double current_time) const
{
  return tf2_buffer_ptr_->_allFramesAsDot(current_time);
}


bool Transformer::ok() const { return true; }

void Transformer::getFrameStrings(std::vector<std::string> & vec) const
{
  tf2_buffer_ptr_->_getFrameStrings(vec);
}


void Transformer::transformQuaternion(const std::string& target_frame, const Stamped<Quaternion>& stamped_in, Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);

  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}


void Transformer::transformVector(const std::string& target_frame,
                                  const Stamped<tf::Vector3>& stamped_in,
                                  Stamped<tf::Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}


void Transformer::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                                      const Stamped<Quaternion>& stamped_in,
                                      const std::string& fixed_frame,
                                      Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}


void Transformer::transformVector(const std::string& target_frame, const ros::Time& target_time,
                                  const Stamped<Vector3>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}


void Transformer::transformPoint(const std::string& target_frame, const ros::Time& target_time,
                                 const Stamped<Point>& stamped_in,
                                 const std::string& fixed_frame,
                                 Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}

void Transformer::transformPose(const std::string& target_frame, const ros::Time& target_time,
                                const Stamped<Pose>& stamped_in,
                                const std::string& fixed_frame,
                                Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
}

boost::signals2::connection Transformer::addTransformsChangedListener(boost::function<void(void)> callback)
{
  return tf2_buffer_ptr_->_addTransformsChangedListener(callback);
}

void Transformer::removeTransformsChangedListener(boost::signals2::connection c)
{
  tf2_buffer_ptr_->_removeTransformsChangedListener(c);
}
