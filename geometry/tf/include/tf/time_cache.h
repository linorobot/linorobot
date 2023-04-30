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

#ifndef TF_TIME_CACHE_H
#define TF_TIME_CACHE_H

#include <set>
#include <boost/thread/mutex.hpp>

#include "tf/transform_datatypes.h"
#include "tf/exceptions.h"

#include "tf/LinearMath/Transform.h"

#include <sstream>

namespace tf
{
enum ExtrapolationMode {  ONE_VALUE, INTERPOLATE, EXTRAPOLATE_BACK, EXTRAPOLATE_FORWARD };


typedef uint32_t CompactFrameID;
typedef std::pair<ros::Time, CompactFrameID> P_TimeAndFrameID;

/** \brief Storage for transforms and their parent */
class TransformStorage
{
public:
  TransformStorage();
  TransformStorage(const StampedTransform& data, CompactFrameID frame_id, CompactFrameID child_frame_id);

  TransformStorage(const TransformStorage& rhs)
  {
    *this = rhs;
  }

  TransformStorage& operator=(const TransformStorage& rhs)
  {
#if 01
    rotation_ = rhs.rotation_;
    translation_ = rhs.translation_;
    stamp_ = rhs.stamp_;
    frame_id_ = rhs.frame_id_;
    child_frame_id_ = rhs.child_frame_id_;
#endif
    return *this;
  }

  bool operator< (const TransformStorage &b) const
  {
    return this->stamp_ < b.stamp_;
  }


  tf::Quaternion rotation_;
  tf::Vector3 translation_;
  ros::Time stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
};




/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache(ros::Duration max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));

  bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0);
  bool insertData(const TransformStorage& new_data);
  void clearList();
  CompactFrameID getParent(ros::Time time, std::string* error_str);
  P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  unsigned int getListLength();
  ros::Time getLatestTimestamp();
  ros::Time getOldestTimestamp();


private:
  typedef std::set<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  ros::Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(const TransformStorage*& one, const TransformStorage*& two, ros::Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);


  void pruneList();



};


}
#endif // TF_TIME_CACHE_H
