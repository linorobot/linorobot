/*
 * Copyright (c) 2012, D. Kuhner, P. Ruchti, University of Freiburg
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
 *     * Neither the name of the University of Freiburg nor the names of its
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

#ifndef OCTOMAP_SERVER_TRACKINGOCTOMAPSERVER_H_
#define OCTOMAP_SERVER_TRACKINGOCTOMAPSERVER_H_

#include "octomap_server/OctomapServer.h"

namespace octomap_server {

class TrackingOctomapServer: public OctomapServer {
public:
  TrackingOctomapServer(const std::string& filename = "");
  virtual ~TrackingOctomapServer();

  void trackCallback(sensor_msgs::PointCloud2Ptr cloud);
  void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
  void trackChanges();

  bool listen_changes;
  bool track_changes;
  int min_change_pub;
  std::string change_id_frame;
  ros::Publisher pubFreeChangeSet;
  ros::Publisher pubChangeSet;
  ros::Subscriber subChangeSet;
  ros::Subscriber subFreeChanges;
};

} /* namespace octomap */
#endif /* TRACKINGOCTOMAPSERVER_H_ */
