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

#ifndef TF_TF_H
#define TF_TF_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>
#include <memory>

#include <tf/exceptions.h>
#include "tf/time_cache.h"
#include <boost/unordered_map.hpp>
#include <boost/signals2.hpp>
#include "geometry_msgs/TwistStamped.h"

#include <tf2_ros/buffer.h>
#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.
#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef tf_EXPORTS // we are building a shared lib/dll
    #define TF_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define TF_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define TF_DECL
#endif

namespace tf
{
/** \brief resolve tf names */
std::string resolve(const std::string& prefix, const std::string& frame_name);

/** strip a leading slash for */
std::string strip_leading_slash(const std::string& frame_name);

/** \deprecated This has been renamed to tf::resolve */
ROS_DEPRECATED static inline std::string remap(const std::string& prefix, const std::string& frame_name) { return tf::resolve(prefix, frame_name);}

enum ErrorValues { NO_ERROR = 0, LOOKUP_ERROR, CONNECTIVITY_ERROR, EXTRAPOLATION_ERROR};

/** \brief An internal representation of transform chains
 *
 * This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
typedef struct
{
  std::vector<TransformStorage > inverseTransforms;
  std::vector<TransformStorage > forwardTransforms;
} TransformLists;

/** \brief A Class which provides coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and lookup of
 * relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.
 * For example your typical robot would have a transform from global to real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to hand.
 * libTF is designed to take care of all the intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for generating the transform into that frame from it's parent and a reference to the parent frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the exception tf::LookupException
 */
class TF_DECL Transformer
{
public:
  /************* Constants ***********************/
  static const unsigned int MAX_GRAPH_DEPTH = 100UL;   //!< The maximum number of time to recurse before assuming the tree has a loop.
  static const double DEFAULT_CACHE_TIME;  //!< 10.0 is the default amount of time to cache data in seconds, set in cpp file. 
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL; //!< The default amount of time to extrapolate //deprecated since integration with tf2


  /** Constructor
   * \param interpolating Unused, legacy always true
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  Transformer(bool interpolating = true,
              ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME));
  virtual ~Transformer(void);

  /** \brief Clear all data */
  void clear();

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * returns true unless an error occured
   */
  bool setTransform(const StampedTransform& transform, const std::string & authority = "default_authority");

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   */
  void lookupTransform(const std::string& target_frame, const std::string& source_frame,
                       const ros::Time& time, StampedTransform& transform) const;
  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time. 
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   */
  void lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                       const std::string& source_frame, const ros::Time& source_time,
                       const std::string& fixed_frame, StampedTransform& transform) const;

  /** \brief Lookup the twist of the tracking_frame with respect to the observation frame in the reference_frame using the reference point
   * \param tracking_frame The frame to track
   * \param observation_frame The frame from which to measure the twist
   * \param reference_frame The reference frame in which to express the twist
   * \param reference_point The reference point with which to express the twist
   * \param reference_point_frame The frame_id in which the reference point is expressed
   * \param time The time at which to get the velocity
   * \param duration The period over which to average
   * \param twist The twist output
   * 
   * This will compute the average velocity on the interval 
   * (time - duration/2, time+duration/2). If that is too close to the most
   * recent reading, in which case it will shift the interval up to
   * duration/2 to prevent extrapolation.  Possible exceptions
   * tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   * 
   * New in geometry 1.1
   */

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                   const tf::Point & reference_point, const std::string& reference_point_frame, 
                   const ros::Time& time, const ros::Duration& averaging_interval, 
                   geometry_msgs::Twist& twist) const;

  /** \brief lookup the twist of the tracking frame with respect to the observational frame 
   * 
   * This is a simplified version of
   * lookupTwist with it assumed that the reference point is the
   * origin of the tracking frame, and the reference frame is the
   * observation frame.  
   * 
   * New in geometry 1.1
   */

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, 
                   const ros::Time& time, const ros::Duration& averaging_interval,
                   geometry_msgs::Twist& twist) const;

  /** \brief Block until a transform is possible or it times out
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool waitForTransform(const std::string& target_frame, const std::string& source_frame,
                        const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
                        std::string* error_msg = NULL) const;
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time,
                    std::string* error_msg = NULL) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame,
                    std::string* error_msg = NULL) const;

  /** \brief Block until a transform is possible or it times out
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool waitForTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame,
                    const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
                    std::string* error_msg = NULL) const;

  /**@brief Return the latest rostime which is common across the spanning set
   * zero if fails to cross */
  int getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const;

  /** \brief Transform a Stamped Quaternion into the target frame
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument. */
  void transformQuaternion(const std::string& target_frame, const Stamped<tf::Quaternion>& stamped_in, Stamped<tf::Quaternion>& stamped_out) const;
  /** \brief Transform a Stamped Vector3 into the target frame 
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformVector(const std::string& target_frame, const Stamped<tf::Vector3>& stamped_in, Stamped<tf::Vector3>& stamped_out) const;
  /** \brief Transform a Stamped Point into the target frame 
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformPoint(const std::string& target_frame, const Stamped<tf::Point>& stamped_in, Stamped<tf::Point>& stamped_out) const;
  /** \brief Transform a Stamped Pose into the target frame 
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformPose(const std::string& target_frame, const Stamped<tf::Pose>& stamped_in, Stamped<tf::Pose>& stamped_out) const;

  /** \brief Transform a Stamped Quaternion into the target frame 
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                           const Stamped<tf::Quaternion>& stamped_in,
                           const std::string& fixed_frame,
                           Stamped<tf::Quaternion>& stamped_out) const;
  /** \brief Transform a Stamped Vector3 into the target frame 
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformVector(const std::string& target_frame, const ros::Time& target_time,
                       const Stamped<tf::Vector3>& stamped_in,
                       const std::string& fixed_frame,
                       Stamped<tf::Vector3>& stamped_out) const;
  /** \brief Transform a Stamped Point into the target frame
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformPoint(const std::string& target_frame, const ros::Time& target_time,
                      const Stamped<tf::Point>& stamped_in,
                      const std::string& fixed_frame,
                      Stamped<tf::Point>& stamped_out) const;
  /** \brief Transform a Stamped Pose into the target frame
   * This can throw anything a lookupTransform can throw as well as tf::InvalidArgument.*/
  void transformPose(const std::string& target_frame, const ros::Time& target_time,
                     const Stamped<tf::Pose>& stamped_in,
                     const std::string& fixed_frame,
                     Stamped<tf::Pose>& stamped_out) const;

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException
   */
  //std::string chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string & fixed_frame) const;

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException
   */
  void chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string & fixed_frame, std::vector<std::string>& output) const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsString() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsDot(double current_time = 0) const;

  /** \brief A way to get a std::vector of available frame ids */
  void getFrameStrings(std::vector<std::string>& ids) const;

  /**@brief Fill the parent of a frame.
   * @param frame_id The frame id of the frame in question
   * @param parent The reference to the string to fill the parent
   * Returns true unless "NO_PARENT" */
  bool getParent(const std::string& frame_id, ros::Time time, std::string& parent) const;

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  bool frameExists(const std::string& frame_id_str) const;

  /**@brief Set the distance which tf is allow to extrapolate
   * \param distance How far to extrapolate before throwing an exception
   * default is zero */
  void setExtrapolationLimit(const ros::Duration& distance);

  /**@brief Get the duration over which this transformer will cache */
  ros::Duration getCacheLength() { return tf2_buffer_ptr_->getCacheLength();}

  /**
   * \brief Add a callback that happens when a new transform has arrived
   *
   * \param callback The callback, of the form void func();
   * \return A boost::signals2::connection object that can be used to remove this
   * listener
   */
  boost::signals2::connection addTransformsChangedListener(boost::function<void(void)> callback);
  void removeTransformsChangedListener(boost::signals2::connection c);

  /** 
   * \brief Get the tf_prefix this is running with
   */
  std::string getTFPrefix() const { return tf_prefix_;};

  //Declare that it is safe to call waitForTransform
  void setUsingDedicatedThread(bool value) { tf2_buffer_ptr_->setUsingDedicatedThread(value);};
  // Get the state of using_dedicated_thread_ from the buffer
  bool isUsingDedicatedThread() { return tf2_buffer_ptr_->isUsingDedicatedThread();};

  // Get a copy of the shared_ptr containing the tf2_ros::Buffer object
  std::shared_ptr<tf2_ros::Buffer> getTF2BufferPtr() { return tf2_buffer_ptr_;};

protected:

  

  /** \brief The internal storage class for ReferenceTransform.
   *
   * An instance of this class is created for each frame in the system.
   * This class natively handles the relationship between frames.
   *
   * The derived class Pose3DCache provides a buffered history of positions
   * with interpolation.
   *
   */


  /******************** Internal Storage ****************/


  /// transform prefix to apply as necessary
  std::string tf_prefix_;

  
 public:
  // A flag to allow falling back to wall time
  bool fall_back_to_wall_time_;
  
 protected:
  /** Hack method to work around #4150 */
  ros::Time now() const { 
    if (!fall_back_to_wall_time_) 
      return ros::Time::now() ; 
    else {
      ros::WallTime wt = ros::WallTime::now();
      return ros::Time(wt.sec, wt.nsec);
    };
  }

  // Allows broadcaster to check ok() before wait for transform
  // Always returns true in base class 
  virtual bool ok() const;
   
  /************************* Internal Functions ****************************/




protected:
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_ptr_;

};


/** \brief Throw InvalidArgument if quaternion is malformed */
inline void assertQuaternionValid(const tf::Quaternion & q)
{
  if(std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w()))
  {
    std::stringstream ss;
    ss << "Quaternion contains a NaN" << std::endl;
    throw tf::InvalidArgument(ss.str());
  }

  if(std::fabs(q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w() - 1) > 0.01)
  {
    std::stringstream ss;
    ss << "Quaternion malformed, magnitude: " << q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w() << " should be 1.0" <<std::endl;
    throw tf::InvalidArgument(ss.str());
  }  //  ROS_ASSERT(std::fabs(q.x()*q.x() + q.y()*q.y() + q.z*q.z() + q.w()*q.w() - 1 < 0.01));
}

/** \brief Throw InvalidArgument if quaternion is malformed */
inline void assertQuaternionValid(const geometry_msgs::Quaternion & q)
{
  if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w))
  {
    std::stringstream ss;
    ss << "Quaternion contains a NaN" << std::endl;
    throw tf::InvalidArgument(ss.str());
  }

  if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01)
  {
    std::stringstream ss;
    ss << "Quaternion malformed, magnitude: " << q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w << " should be 1.0" <<std::endl;
    throw tf::InvalidArgument(ss.str());
  }  //  ROS_ASSERT(std::fabs(q.x()*q.x() + q.y()*q.y() + q.z*q.z() + q.w()*q.w() - 1 < 0.01));
}
}
#endif //TF_TF_H
