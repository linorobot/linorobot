/*
 * Copyright (c) 2011-2013, A. Hornung, M. Philips
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

#include <octomap_server/OctomapServerMultilayer.h>

using namespace octomap;

namespace octomap_server{



OctomapServerMultilayer::OctomapServerMultilayer(ros::NodeHandle private_nh_)
: OctomapServer(private_nh_)
{

  // TODO: callback for arm_navigation attached objects was removed, is
  // there a replacement functionality?

  // TODO: param maps, limits
  // right now 0: base, 1: spine, 2: arms
  ProjectedMap m;
  m.name = "projected_base_map";
  m.minZ = 0.0;
  m.maxZ = 0.3;
  m.z = 0.0;
  m_multiGridmap.push_back(m);

  m.name = "projected_spine_map";
  m.minZ = 0.25;
  m.maxZ = 1.4;
  m.z = 0.6;
  m_multiGridmap.push_back(m);

  m.name = "projected_arm_map";
  m.minZ = 0.7;
  m.maxZ = 0.9;
  m.z = 0.8;
  m_multiGridmap.push_back(m);


  for (unsigned i = 0; i < m_multiGridmap.size(); ++i){
    ros::Publisher* pub = new ros::Publisher(m_nh.advertise<nav_msgs::OccupancyGrid>(m_multiGridmap.at(i).name, 5, m_latchedTopics));
    m_multiMapPub.push_back(pub);
  }

  // init arm links (could be params as well)
  m_armLinks.push_back("l_elbow_flex_link");
  m_armLinkOffsets.push_back(0.10);
  m_armLinks.push_back("l_gripper_l_finger_tip_link");
  m_armLinkOffsets.push_back(0.03);
  m_armLinks.push_back("l_gripper_r_finger_tip_link");
  m_armLinkOffsets.push_back(0.03);
  m_armLinks.push_back("l_upper_arm_roll_link");
  m_armLinkOffsets.push_back(0.16);
  m_armLinks.push_back("l_wrist_flex_link");
  m_armLinkOffsets.push_back(0.05);
  m_armLinks.push_back("r_elbow_flex_link");
  m_armLinkOffsets.push_back(0.10);
  m_armLinks.push_back("r_gripper_l_finger_tip_link");
  m_armLinkOffsets.push_back(0.03);
  m_armLinks.push_back("r_gripper_r_finger_tip_link");
  m_armLinkOffsets.push_back(0.03);
  m_armLinks.push_back("r_upper_arm_roll_link");
  m_armLinkOffsets.push_back(0.16);
  m_armLinks.push_back("r_wrist_flex_link");
  m_armLinkOffsets.push_back(0.05);


}

OctomapServerMultilayer::~OctomapServerMultilayer(){
  for (unsigned i = 0; i < m_multiMapPub.size(); ++i){
    delete m_multiMapPub[i];
  }

}

void OctomapServerMultilayer::handlePreNodeTraversal(const ros::Time& rostime){
  // multilayer server always publishes 2D maps:
  m_publish2DMap = true;
  nav_msgs::MapMetaData gridmapInfo = m_gridmap.info;

  OctomapServer::handlePreNodeTraversal(rostime);


  // recalculate height of arm layer (stub, TODO)
  geometry_msgs::PointStamped vin;
  vin.point.x = 0;
  vin.point.y = 0;
  vin.point.z = 0;
  vin.header.stamp = rostime;
  double link_padding = 0.03;

  double minArmHeight = 2.0;
  double maxArmHeight = 0.0;

  for (unsigned i = 0; i < m_armLinks.size(); ++i){
    vin.header.frame_id = m_armLinks[i];
    geometry_msgs::PointStamped vout;
    const bool found_trans =
        m_tfListener.waitForTransform("base_footprint", m_armLinks.at(i),
                                      ros::Time(0), ros::Duration(1.0));
    ROS_ASSERT_MSG(found_trans, "Timed out waiting for transform to %s",
                   m_armLinks[i].c_str());
    m_tfListener.transformPoint("base_footprint",vin,vout);
    maxArmHeight = std::max(maxArmHeight, vout.point.z + (m_armLinkOffsets.at(i) + link_padding));
    minArmHeight = std::min(minArmHeight, vout.point.z - (m_armLinkOffsets.at(i) + link_padding));
  }
  ROS_INFO("Arm layer interval adjusted to (%f,%f)", minArmHeight, maxArmHeight);
  m_multiGridmap.at(2).minZ = minArmHeight;
  m_multiGridmap.at(2).maxZ = maxArmHeight;
  m_multiGridmap.at(2).z = (maxArmHeight+minArmHeight)/2.0;





  // TODO: also clear multilevel maps in BBX region (see OctomapServer.cpp)?

  bool mapInfoChanged = mapChanged(gridmapInfo, m_gridmap.info);

  for (MultilevelGrid::iterator it = m_multiGridmap.begin(); it != m_multiGridmap.end(); ++it){
    it->map.header = m_gridmap.header;
    it->map.info = m_gridmap.info;
    it->map.info.origin.position.z = it->z;
    if (m_projectCompleteMap){
      ROS_INFO("Map resolution changed, rebuilding complete 2D maps");
      it->map.data.clear();
      // init to unknown:
      it->map.data.resize(it->map.info.width * it->map.info.height, -1);
    } else if (mapInfoChanged){
      adjustMapData(it->map, gridmapInfo);
    }
  }
}

void OctomapServerMultilayer::handlePostNodeTraversal(const ros::Time& rostime){

  // TODO: calc tall / short obs. cells for arm layer, => temp arm layer
//  std::vector<int> shortObsCells;
//  for(unsigned int i=0; i<arm_map.data.size(); i++){
//    if(temp_arm_map.data[i] == 0){
//      if(map.data[i] == -1)
//        arm_map.data[i] = -1;
//    }
//    else if(arm_map.data[i] == 0)
//      arm_map.data[i] = 0;
//    else if(double(arm_map.data[i])/temp_arm_map.data[i] > 0.8)
//      arm_map.data[i] = 101;
//    else{
//      arm_map.data[i] = 100;
//      shortObsCells.push_back(i);
//    }
//  }
//
//  std::vector<int> tallObsCells;
//  tallObsCells.reserve(shortObsCells.size());
//  int dxy[8] = {-arm_map.info.width-1, -arm_map.info.width, -arm_map.info.width+1, -1, 1, arm_map.info.width-1, arm_map.info.width, arm_map.info.width+1};
//  for(unsigned int i=0; i<shortObsCells.size(); i++){
//    for(int j=0; j<8; j++){
//      int temp = shortObsCells[i]+dxy[j];
//      if(temp<0 || temp>=arm_map.data.size())
//        continue;
//      if(arm_map.data[temp]==101){
//        tallObsCells.push_back(shortObsCells[i]);
//        break;
//      }
//    }
//  }
//  for(unsigned int i=0; i<tallObsCells.size(); i++)
//    arm_map.data[tallObsCells[i]] = 101;



  OctomapServer::handlePostNodeTraversal(rostime);

  for (unsigned i = 0; i < m_multiMapPub.size(); ++i){
    m_multiMapPub[i]->publish(m_multiGridmap.at(i).map);
  }

}
void OctomapServerMultilayer::update2DMap(const OcTreeT::iterator& it, bool occupied){
  double z = it.getZ();
  double s2 = it.getSize()/2.0;

  // create a mask on which maps to update:
  std::vector<bool> inMapLevel(m_multiGridmap.size(), false);
  for (unsigned i = 0; i < m_multiGridmap.size(); ++i){
    if (z+s2 >= m_multiGridmap[i].minZ && z-s2 <= m_multiGridmap[i].maxZ){
      inMapLevel[i] = true;
    }
  }

  if (it.getDepth() == m_maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      m_gridmap.data[idx] = 100;
    else if (m_gridmap.data[idx] == -1){
      m_gridmap.data[idx] = 0;
    }

    for (unsigned i = 0; i < inMapLevel.size(); ++i){
      if (inMapLevel[i]){
        if (occupied)
          m_multiGridmap[i].map.data[idx] = 100;
        else if (m_multiGridmap[i].map.data[idx] == -1)
          m_multiGridmap[i].map.data[idx] = 0;
      }
    }

  } else {
    int intSize = 1 << (m_treeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
        if (occupied)
          m_gridmap.data[idx] = 100;
        else if (m_gridmap.data[idx] == -1){
          m_gridmap.data[idx] = 0;
        }

        for (unsigned i = 0; i < inMapLevel.size(); ++i){
          if (inMapLevel[i]){
            if (occupied)
              m_multiGridmap[i].map.data[idx] = 100;
            else if (m_multiGridmap[i].map.data[idx] == -1)
              m_multiGridmap[i].map.data[idx] = 0;
          }
        }
      }
    }
  }


}

}



