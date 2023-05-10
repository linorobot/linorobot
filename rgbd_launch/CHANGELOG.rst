^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rgbd_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2023-02-19)
------------------
* Capability: Add queue_size option `#52 <https://github.com/ros-drivers/rgbd_launch/issues/52>`_
* Maintenance: Synced with older master branch `#54 <https://github.com/ros-drivers/rgbd_launch/issues/54>`_, `#53 <https://github.com/ros-drivers/rgbd_launch/issues/53>`_

2.3.0 (2020-06-02)
------------------
* Noetic support `#49 <https://github.com/ros-drivers/rgbd_launch/issues/49>`_
* Contributors: Mike Ferguson, Isaac I.Y. Saito

2.2.2 (2016-09-13)
------------------
* [capability] add rgb prefix, rectify_ir to node name
* [maintenance] enable rostest upon build.
* [maintenance] Remove Indigo. Enable Kinetic from Travis conf. `#32 <https://github.com/ros-drivers/rgbd_launch/issues/32>`_
* Contributors: Yuki Furuta, Isaac I.Y. Saito

2.2.1 (2016-05-07)
------------------
* [feat] Depth registered filtered `#26 <https://github.com/ros-drivers/rgbd_launch/issues/26>`_
* [sys] Update config to using industrial_ci with Prerelease Test. `#24 <https://github.com/ros-drivers/rgbd_launch/issues/24>`_
* Contributors: Jonathan Bohren, Isaac I.Y. Saito

2.2.0 (2015-11-17)
------------------
* 1st release into ROS Jade
* [feat] Adjust to tf2 (`#18 <https://github.com/ros-drivers/rgbd_launch/issues/18>`_)
* [sys] travis enabled
* Contributors: Daiki Maekawa, Isaac I.Y. Saito

2.1.1 (2015-11-16)
------------------
* 1st ROS Jade release
* [feat] Add convert_metric nodes to depth_registered.launch.xml (`#13 <https://github.com/ros-drivers/rgbd_launch/issues/13>`_ from kbogert/hydro-devel)
* [feat] Add the metric nodes to output a depth image
* [fix] Merge pull request `#1 <https://github.com/ros-drivers/rgbd_launch/issues/1>`_ from piyushk/piyush/kbogert-depth-registered-metric
  fixed rect convert metric to conform to both s/w and h/w pipelines. fixe...
* Contributors: Kenneth Bogert, Piyush Khandelwal

2.1.0 (2014-05-05)
------------------
* Revert "Add machine parameter". closes `#5 <https://github.com/ros-drivers/rgbd_launch/issues/5>`_
* Contributors: Piyush Khandelwal

2.0.1 (2013-09-06)
------------------
* Merge pull request `#2 <https://github.com/ros-drivers/rgbd_launch/issues/2>`_ - added machine parameter to launch nodelet manager on a remote machine.
* Merge pull request `#1 <https://github.com/ros-drivers/rgbd_launch/issues/1>`_ - added debayer_processing argument

2.0.0 (2013-08-19)
------------------
* explicit s/w and h/w processing chains with flags to enable/disable
* marked and moved all launch files as internal
* added a script to serve as an upgrade notice for ROS Hydro. This notice should be removed for ROS Indigo
* fixed cloud processing when device registration is enabled
* added tf prefix resolution to kinect_frames
* Migrated from openni_launch v1.9.1 and removed all openni specific files

