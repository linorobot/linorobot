/*
 * Copyright (c) 2013, A. Hornung, University of Freiburg
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

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <octomap_msgs/GetOctomap.h>
using octomap_msgs::GetOctomap;

#define USAGE "\nUSAGE: octomap_server_static <mapfile.[bt|ot]>\n" \
		"  mapfile.bt: OctoMap filename to be loaded (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;

class OctomapServerStatic{
public:
  OctomapServerStatic(const std::string& filename)
    : m_octree(NULL), m_worldFrameId("/map")
  {

    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);


    // open file:
    if (filename.length() <= 3){
      ROS_ERROR("Octree file does not have .ot extension");
      exit(1);
    }

    std::string suffix = filename.substr(filename.length()-3, 3);

    // .bt files only as OcTree, all other classes need to be in .ot files:
    if (suffix == ".bt"){
      OcTree* octree = new OcTree(filename);

      m_octree = octree;
    } else if (suffix == ".ot"){
      AbstractOcTree* tree = AbstractOcTree::read(filename);
      if (!tree){
        ROS_ERROR("Could not read octree from file");
        exit(1);
      }

      m_octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);

    } else{
      ROS_ERROR("Octree file does not have .bt or .ot extension");
      exit(1);
    }

    if (!m_octree ){
      ROS_ERROR("Could not read right octree class in file");
      exit(1);
    }

    ROS_INFO("Read octree type \"%s\" from file %s", m_octree->getTreeType().c_str(), filename.c_str());
    ROS_INFO("Octree resultion: %f, size: %zu", m_octree->getResolution(), m_octree->size());


    m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServerStatic::octomapBinarySrv, this);
    m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServerStatic::octomapFullSrv, this);

  }

  ~OctomapServerStatic(){


  }

  bool octomapBinarySrv(GetOctomap::Request  &req,
                        GetOctomap::Response &res)
  {
    ROS_INFO("Sending binary map data on service request");
    res.map.header.frame_id = m_worldFrameId;
    res.map.header.stamp = ros::Time::now();
    if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
      return false;

    return true;
  }

  bool octomapFullSrv(GetOctomap::Request  &req,
                                     GetOctomap::Response &res)
  {
    ROS_INFO("Sending full map data on service request");
    res.map.header.frame_id = m_worldFrameId;
    res.map.header.stamp = ros::Time::now();


    if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
      return false;

    return true;
  }

private:
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService;
  ros::NodeHandle m_nh;
  std::string m_worldFrameId;
  AbstractOccupancyOcTree* m_octree;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server_static");
  std::string mapFilename("");

  if (argc == 2)
    mapFilename = std::string(argv[1]);
  else{
    ROS_ERROR("%s", USAGE);
    exit(1);
  }

  try{
    OctomapServerStatic ms(mapFilename);
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server_static exception: %s", e.what());
    exit(2);
  }

  exit(0);
}


