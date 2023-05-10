/**
* octomap_server_nodelet: A nodelet version of A. Hornung's octomap server
* @author Marcus Liebhardt
* License: BSD
*/

/*
 * Copyright (c) 2012, Marcus Liebhardt, Yujin Robot
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
 *     * Neither the name of Yujin Robot nor the names of its
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
#include <octomap_server/OctomapServer.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>


namespace octomap_server
{

class OctomapServerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_DEBUG("Initializing octomap server nodelet ...");
    ros::NodeHandle& nh = this->getNodeHandle();
    ros::NodeHandle& private_nh = this->getPrivateNodeHandle();
    server_.reset(new OctomapServer(private_nh, nh));

    std::string mapFilename("");
    if (private_nh.getParam("map_file", mapFilename)) {
      if (!server_->openFile(mapFilename)){
        NODELET_WARN("Could not open file %s", mapFilename.c_str());
      }
    }
  }
private:
  boost::shared_ptr<OctomapServer> server_;
};

} // namespace

PLUGINLIB_EXPORT_CLASS(octomap_server::OctomapServerNodelet, nodelet::Nodelet)
