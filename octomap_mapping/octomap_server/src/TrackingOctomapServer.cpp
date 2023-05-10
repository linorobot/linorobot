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

#include <octomap_server/TrackingOctomapServer.h>
#include <string>

using namespace octomap;

namespace octomap_server {

TrackingOctomapServer::TrackingOctomapServer(const std::string& filename) :
	    OctomapServer()
{
  //read tree if necessary
  if (filename != "") {
    if (m_octree->readBinary(filename)) {
      ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
      m_treeDepth = m_octree->getTreeDepth();
      m_res = m_octree->getResolution();
      m_gridmap.info.resolution = m_res;

      publishAll();
    } else {
      ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
      exit(-1);
    }
  }

  ros::NodeHandle private_nh("~");

  std::string changeSetTopic = "changes";
  std::string changeIdFrame = "/talker/changes";

  private_nh.param("topic_changes", changeSetTopic, changeSetTopic);
  private_nh.param("change_id_frame", change_id_frame, changeIdFrame);
  private_nh.param("track_changes", track_changes, false);
  private_nh.param("listen_changes", listen_changes, false);
  private_nh.param("min_change_pub", min_change_pub, 0);

  if (track_changes && listen_changes) {
    ROS_WARN("OctoMapServer: It might not be useful to publish changes and at the same time listen to them."
        "Setting 'track_changes' to false!");
    track_changes = false;
  }

  if (track_changes) {
    ROS_INFO("starting server");
    pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>(
        changeSetTopic, 1);
    m_octree->enableChangeDetection(true);
  }

  if (listen_changes) {
    ROS_INFO("starting client");
    subChangeSet = private_nh.subscribe(changeSetTopic, 1,
                                        &TrackingOctomapServer::trackCallback, this);
  }
}

TrackingOctomapServer::~TrackingOctomapServer() {
}

void TrackingOctomapServer::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
  OctomapServer::insertScan(sensorOrigin, ground, nonground);

  if (track_changes) {
    trackChanges();
  }
}

void TrackingOctomapServer::trackChanges() {
  KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
  KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();

  pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();

  int c = 0;
  for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
    ++c;
    OcTreeNode* node = m_octree->search(iter->first);

    bool occupied = m_octree->isNodeOccupied(node);

    point3d center = m_octree->keyToCoord(iter->first);

    pcl::PointXYZI pnt;
    pnt.x = center(0);
    pnt.y = center(1);
    pnt.z = center(2);

    if (occupied) {
      pnt.intensity = 1000;
    }
    else {
      pnt.intensity = -1000;
    }

    changedCells.push_back(pnt);
  }

  if (c > min_change_pub)
  {
    sensor_msgs::PointCloud2 changed;
    pcl::toROSMsg(changedCells, changed);
    changed.header.frame_id = change_id_frame;
    changed.header.stamp = ros::Time().now();
    pubChangeSet.publish(changed);
    ROS_DEBUG("[server] sending %d changed entries", (int)changedCells.size());

    m_octree->resetChangeDetection();
    ROS_DEBUG("[server] octomap size after updating: %d", (int)m_octree->calcNumNodes());
  }
}

void TrackingOctomapServer::trackCallback(sensor_msgs::PointCloud2Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZI> cells;
  pcl::fromROSMsg(*cloud, cells);
  ROS_DEBUG("[client] size of newly occupied cloud: %i", (int)cells.points.size());

  for (size_t i = 0; i < cells.points.size(); i++) {
    pcl::PointXYZI& pnt = cells.points[i];
    m_octree->updateNode(m_octree->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
  }

  m_octree->updateInnerOccupancy();
  ROS_DEBUG("[client] octomap size after updating: %d", (int)m_octree->calcNumNodes());
}

} /* namespace octomap */
