^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package octomap_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.7 (2021-12-24)
------------------
* Address warnings on Noetic (`#81 <https://github.com/octomap/octomap_mapping/issues/81>`_)
* Contributors: Wolfgang Merkt

0.6.6 (2020-12-08)
------------------
* Update CI, package format, dependencies to address dependency issue on Debian Buster (`#79 <https://github.com/OctoMap/octomap_mapping/issues/79>`_)
* Contributors: Wolfgang Merkt

0.6.5 (2020-04-23)
------------------
* Add color server nodelet (`#68 <https://github.com/OctoMap/octomap_mapping/issues/68>`_, `#67 <https://github.com/OctoMap/octomap_mapping/issues/67>`_)
* Updated maintainer email
* Contributors: clunietp, Wolfgang Merkt

0.6.4 (2020-01-08)
------------------
* Add private node handle to fix nodelet support (`#61 <https://github.com/OctoMap/octomap_mapping/issues/61>`_), fixes `#39 <https://github.com/OctoMap/octomap_mapping/issues/39>`_
* Add octomap_server_color library by default (`#60 <https://github.com/OctoMap/octomap_mapping/issues/60>`_) - by Matthew Powelson
* Check if part of a voxel is in occupancy range (`#59 <https://github.com/OctoMap/octomap_mapping/issues/59>`_) - by Jasper v. B.
* Contributors: Matthew Powelson, Wolfgang Merkt, Jasper v. B.

0.6.3 (2019-01-28)
------------------
* Fix compilation on Debian Stretch
* Get rgb from point cloud iterator without byte shift
* Contributors: Kentaro Wada, Wolfgang Merkt

0.6.2 (2019-01-27)
------------------
* Update maintainer email (Wolfgang Merkt)
* Change catkin_package `DEPENDS` to `OCTOMAP` to avoid CMake warning
* Update maintainer email (Arming Hornung)
* Update to use non deprecated pluginlib macro
* Fixed memory leak of colors pointer if COLOR_OCTOMAP_SERVER defined
* Contributors: Armin Hornung, Mikael Arguedas, Ronky, Wolfgang Merkt

0.6.1 (2016-10-19)
------------------
* Fix for Colored Octomap: Use PCLPoint everywhere
  Fixes compiler error when enabling the define
  for color.
* Fixed maxRange bug in OctomapServer.cpp for clearing
* Adjust maintainer email
* Contributors: Armin Hornung, Brandon Kinman, Felix Endres

0.6.0 (2016-03-25)
------------------
* Add sensor model parameters to dynamic_reconfigure
* Load map file from rosparam
* Add x and y filter for pointcloud
* Preparations for ColorOctomapServer (compile option, from source)
* Fix iterator in OctomapServer
* TrackingOctomapServer: Publish node center rather than index, prevent from publishing empty cloud
* Contributors: Armin Hornung, Javier V. Gomez, JJeremie Deray, MasakiMurooka, Shohei Fujii, Wolfgang Merkt

0.5.3 (2014-01-09)
------------------
* Fixing PCL linking errors on build farm

0.5.2 (2014-01-09)
------------------
* Fixing PCL linking errors on build farm

0.5.1 (2013-11-25)
------------------
* Fix missing nodelet plugin from install

0.5.0 (2013-10-24)
------------------
* Small fix in octomap_server_static usage
* Catkinization, remove support for arm_navigation

0.4.9 (2013-06-27)
------------------
* cleanup of unused functions
* Parameters, reading .bt files in octomap_server_static
* added simple octomap_server_static node to serve OctoMaps from .ot files (no scan integration)
* Fix for incremental 2D projection map updates (thx to B.Coenen for the report)
* Publish free space as MarkerArray and CollisionMap (set parameter ~publish_free_space=True to enable). Thx to I. Wieser!
* renamed OctomapServer's NodeHandle constructor parameter to be more clear, added the same to OctomapServerMultilayer

0.4.8 (2013-01-08)
------------------
* Applied patch from issue `#7 <https://github.com/OctoMap/octomap_mapping/issues/7>`_: Nodelet version of octomap_server
  Modified to not change the global namespace
* fixes for cmake / catkin
* fixed octomap_server for OctoMap 1.5 (deprecations), adjusted to new msg format
* changed message format to contain only data, meta information stored in new message fields (untested for Groovy)

0.4.6 (2013-01-28)
------------------
* Added NodeHandle parameter to OctomapServerMultilayer constructor
* Commited patch `#7 <https://github.com/OctoMap/octomap_mapping/issues/7>`_, contributed by M. Liebhardt: Nodelet version of the octomap server
* octomap_server and octomap_saver now fully support both binary and full occupancy maps
* octomap_server can now open .ot files properly, updated octomap_ros to new-style stack.xml
* deprecated OctomapROS in octomap_ros => directly use octomap lib and conversions.h
* removed OctomapROS wrapper from octomap_server classes
* octomap_server manifest exports dynamic_reconfigure path in cppflags
* parameter in launch file adjusted

0.4.5 (2012-06-18)
------------------
* new parameter to enable incremental 2D mapping (experimental, default: false)
* bug fix for OctomapServer map projection
* Fixed OctomapServer not clearing obstacles in projected 2D map properly
* fixed map reset and incremental 2D updates
* added arm layer height lookup
* Fixed resolution change (dynamic_reconfigure) and dynamic map size w. incremental updates
* incremental update of projected 2D maps only in updated 3D region, map dynamically grows
* increased Electric compatibility of octomap_server
* OctomapServer keeps track of update region for downprojected 2D map

0.4.4 (2012-04-20)
------------------
* Turned octomap_msgs and octomap_ros into unary stacks, code in octomap_mapping adjusted

0.4.3 (2012-04-17)
------------------
* Merged rev 2477:2613 from trunk:
  - fixed ground filter
  - added missing license headers, improved code layout to ROS standard
  - adjusted to OctoMap 1.4 changes
  - collision map publisher & eraser script ported from branch
  - disabled lazy update temporarily (needs param)
  - dynamic reconfigure interface to limit query depth (and voxel resolution) on the fly

0.4.2 (2012-03-16)
------------------
* fixed ground filter (from trunk, electric)\nVersion increased to 0.4.2

0.4.1 (2012-02-21 16:50)
------------------------
* switched octomap_ros and octomap_server to pure CMake-style linking, version 0.4.1
* removed uneccesary FindEigen.cmake files

0.4.0 (2012-02-21 15:37)
------------------------
* removed eigen package from depends
* Transitioned octomap package to deprecated, now forwards flags with pkg-config to system dependency

0.3.8 (2012-04-26)
------------------
* increased octomap version to 1.4.2, stack version 0.3.8

0.3.7 (2012-02-22)
------------------
* removed temp. workaround for unstable (Eigen for PCL included), increased stack version to 0.3.7
* server/client architecture for octomap_server
* octomap_server: ground plane filter defaults to false, base_footprint frame now only required when filtering

0.3.6 (2012-01-09)
------------------
* changed to Eigen rosdep for electric and fuerte

0.3.5 (2011-10-30)
------------------
* added OctomapServerMultilayer as stub
* More refactoring of octomap_server, added hooks for node traversal
* OctomapServerCombined is now OctomapServer
* cleanup of octomap_server
* - adjusted octomap_mapping trunk to compile against ROS electric (only affects octomap_server).
  => use branch for diamondback!

0.3.4 (2011-10-12)
------------------
* publish empty map (+vis) after reset
* OctomapServerCombined: Drop old octree completely when resetting
* OctomapServerCombined: Parameter for latching topics, reset service
* added srv and service implementation to clear a bbx region in OctomapServerCombined
* OctomapServer:
  private -> protected
  added default constructor
* octomap_server:
  - fixed 2D map for larger volumes
  - now handles an initial file always as static, topics are published latched then
* removed debug PCD writing
* - ground filter now more reliable, filtering in base frame of robot instead of global frame.
  - more parameters for ground filter

0.3.3 (2011-08-17 07:41)
------------------------
* octomap package udpate to use new OctoMap 1.2 library only (no visualization). Removed dependency on Qt / QGLViewer.
* fixed ground plane appearing as occupied

0.3.2 (2011-08-09)
------------------
* merged in changes of octomap_mapping trunk (up to rev 1781):
  - octomap updated to 1.1.1 (testing), tarball URL on ros.org
  - ground plane extraction OctomapServerCombined, configurable using PCL
  - fixes and cleanup in OctomapServerCombined
* parameters for ground plane filtering
* Ground plane extraction improved
* Ground plane extraction (pcl) for testing
* - octomap: use OctoMap 1.1.1 (testing)
  - octomap_server: handle larger pruned nodes in 2D map projection
* refactoring & cleanup of OctomapServerCombined, ready for ground plane extraction
* merged back octomap_server from experimental branch:
  - proper class with more capabilities
  - now sends out map in various representations / visualizations
  - subscribes to PointCloud2 with tf::MessageFilter
  - uses octomap_ros wrapper / conversions
  - OctomapServerCombined (experimental): also builds downprojected 2D map
* added MoveMap.msg from octomap2, extended conversions.h
* templated octomapMsg conversion functions
* octomap_saver adjusted to moved locations
* Moved messages and conversions to octomap_ros from octomap_server
* Removed unnecessary exports in manifests
* - fixes in mainfest / stack.xml for ROS 1.3
  - doxygen properly configured with rosdoc
  - stack release 0.1.2 prep
* Preparations for .deb releases
* License in cpp files, restored compatibility with boxturtle
* Adjusted license to BSD, more parameters in octomap_server
* OctoMap server (copied from octomap repo, trunk)
* Initial checkin of octomap stack (nearly empty at the moment)

0.3.1 (2011-07-15)
------------------
* Patched for arm_navigation changes in "unstable"

0.3.0 (2011-06-28)
------------------
* merged back octomap_server from experimental branch:
  - proper class with more capabilities
  - now sends out map in various representations / visualizations
  - subscribes to PointCloud2 with tf::MessageFilter
  - uses octomap_ros wrapper / conversions
  - OctomapServerCombined (experimental): also builds downprojected 2D map
* added MoveMap.msg from octomap2, extended conversions.h
* templated octomapMsg conversion functions

0.2.0 (2011-03-16)
------------------
* updated stack.xml for cturtle only
* octomap_saver adjusted to moved locations
* Moved messages and conversions to octomap_ros from octomap_server
* Removed unnecessary exports in manifests

0.1.2 (2010-11-23)
------------------
* - fixes in mainfest / stack.xml for ROS 1.3
  - doxygen properly configured with rosdoc
  - stack release 0.1.2 prep

0.1.1 (2010-11-17)
------------------

0.1.0 (2010-11-16)
------------------
* Preparations for .deb releases
* License in cpp files, restored compatibility with boxturtle
* Adjusted license to BSD, more parameters in octomap_server
* OctoMap server (copied from octomap repo, trunk)
* Initial checkin of octomap stack (nearly empty at the moment)
