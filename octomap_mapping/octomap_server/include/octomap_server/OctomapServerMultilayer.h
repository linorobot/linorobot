/*
 * Copyright (c) 2010-2013, A. Hornung, M. Philips
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
#ifndef OCTOMAP_SERVER_OCTOMAPSERVERMULTILAYER_H
#define OCTOMAP_SERVER_OCTOMAPSERVERMULTILAYER_H

#include <octomap_server/OctomapServer.h>

namespace octomap_server {
class OctomapServerMultilayer : public OctomapServer{

public:
  OctomapServerMultilayer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  virtual ~OctomapServerMultilayer();

protected:
  struct ProjectedMap{
    double minZ;
    double maxZ;
    double z; // for visualization
    std::string name;
    nav_msgs::OccupancyGrid map;
  };
  typedef std::vector<ProjectedMap> MultilevelGrid;

  /// hook that is called after traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  std::vector<ros::Publisher*> m_multiMapPub;
  ros::Subscriber m_attachedObjectsSub;

  std::vector<std::string> m_armLinks;
  std::vector<double> m_armLinkOffsets;

  MultilevelGrid m_multiGridmap;


};
}

#endif

