^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf
^^^^^^^^^^^^^^^^^^^^^^^^

1.13.2 (2020-06-08)
-------------------
* fix shebang line for python3 (`#212 <https://github.com/ros/geometry/issues/212>`_)
* Contributors: Mikael Arguedas

1.13.1 (2020-05-15)
-------------------
* Fix ring45 test expectations (`#211 <https://github.com/ros/geometry/issues/211>`_)
  Copying https://github.com/ros/geometry2/commit/04625380bdff3f3e9e860fc0e85f71674ddd1587
* import setup from setuptools instead of distutils-core (`#209 <https://github.com/ros/geometry/issues/209>`_)
* Contributors: Alejandro Hernández Cordero, Shane Loretz

1.13.0 (2020-03-10)
-------------------

1.12.1 (2020-03-10)
-------------------
* Use process_time() for Python 3.8 compatibility (`#205 <https://github.com/ros/geometry/issues/205>`_)
* Bump CMake version to avoid CMP0048 warning (`#204 <https://github.com/ros/geometry/issues/204>`_)
* Add rostest include dirs (`#195 <https://github.com/ros/geometry/issues/195>`_)
* Remove trailing semicolons from tf sources (`#187 <https://github.com/ros/geometry/issues/187>`_)
  * [tf] Removed trailing semicolons after functions from all sources
  Used the -Wpedantic compiler flag to find all occurrences
* Allow to choose output precision in tf_echo (`#186 <https://github.com/ros/geometry/issues/186>`_)
  * Allow to choose output precision in tf_echo
* update how c++11 requirement is added (`#184 <https://github.com/ros/geometry/issues/184>`_)
* update install destination in CMakeLists.txt (`#183 <https://github.com/ros/geometry/issues/183>`_)
  * export binary to right locations
  * specify archive and runtime destinations, update whitespace (`#5 <https://github.com/ros/geometry/issues/5>`_)
* add visibility macro
* windows bring up, use ROS_DEPRECATED
* Remove `signals` from find_package(Boost COMPONENTS ...)
* fixing error of casting away constness in  method void tfSwapScalarEndian(const tfScalar& sourceVal, tfScalar& destVal) in line 626 of Vector3.h (`#179 <https://github.com/ros/geometry/issues/179>`_)
* Fix log output typo: message_notifier -> message_filter (`#177 <https://github.com/ros/geometry/issues/177>`_)
  Almost all the log outputs use message_filter, except one. The warning
  text still referred to message_notifier. This commit fixes that.
* Contributors: C-NR, James Xu, Maarten de Vries, Martin Günther, Shane Loretz, Victor Lamoine, Yoshua Nava

1.12.0 (2018-05-02)
-------------------
* Adapt to new xmlrpcpp header location (`#164 <https://github.com/ros/geometry/issues/164>`_)
* Maintain & expose tf2 Buffer in shared_ptr for tf (`#163 <https://github.com/ros/geometry/issues/163>`_)
  - Adds a tf2_ros::Buffer via a public accessor
  method to expose to customers of Transformer
  - Maintains the tf2_ros::Buffer in a shared_ptr
  to safely share access to the Buffer object
  - As this is targeting Melodic, adds c++11 compile
  flags to grant access to std::shared_ptr's
  - Reorders the include_directories in the CMakeLists
  to ensure the headers exposed in this package are
  read *before* the system installed catkin_INCLUDE_DIRS
  (otherwise changes to tf source headers are never detected
  during a catkin_make on a system with ros-*-geometry
  installed)
* Prevent rates that result in core dump (0.0) or no limit on update rate at all (<0.0) `#159 <https://github.com/ros/geometry/issues/159>`_ (`#160 <https://github.com/ros/geometry/issues/160>`_)
* Fix empty yaml parsing (`#153 <https://github.com/ros/geometry/issues/153>`_)
  Fixes `#152 <https://github.com/ros/geometry/issues/152>`_
  The empty yaml was coming through as a list not a dict so was breaking the expectations.
  I used the shorthand `or {}` since I know any valid data won't evaluate to zero. A more complete solution is described here: https://stackoverflow.com/a/35777649/604099
* Make python scripts Python3 compatible. (`#151 <https://github.com/ros/geometry/issues/151>`_)
  * Python 3 fixes
  * Prefer str.format over operator % and small python3 fixes.
* Contributors: Ian McMahon, Lucas Walter, Maarten de Vries, Tully Foote

1.11.9 (2017-07-14)
-------------------
* Replace legacy python code with appropriate calls to tf2_ros (`#149 <https://github.com/ros/geometry/issues/149>`_) (`#134 <https://github.com/ros/geometry/issues/134>`_)
* Replace deprecated Eigen module with Eigen3
* Update minimum version for run dependency on tf2_ros
* Add support for static_transforms in tf_monitor.
  Fixes `#136 <https://github.com/ros/geometry/issues/136>`_ with `#134 <https://github.com/ros/geometry/issues/134>`_ for tf_echo and view_frames.
* Pass through allFramesAsDot time argument optionally.
* remove vestigial includes. Fixes `#146 <https://github.com/ros/geometry/issues/146>`_ (`#147 <https://github.com/ros/geometry/issues/147>`_)
* Commented code caused error in documentation (`#142 <https://github.com/ros/geometry/issues/142>`_)
* [doc] Add migration notice in manifest. (`#129 <https://github.com/ros/geometry/issues/129>`_)
* Fix "stdlib.h: No such file or directory" errors in GCC-6
* Fix error for null conversion.
* Change version regex for graphviz in view_frames
* fix for issue in getAngleShortestPath(), closes `#102 <https://github.com/ros/geometry/issues/102>`_
* Contributors: AndyZe, Edward Venator, Hodorgasm, Isaac I.Y. Saito, Michael Korn, Mike Purvis, Tom Moore, Tully Foote, Timo Röhling

1.11.8 (2016-03-04)
-------------------
* Update assertQuaternionValid to check for NaNs
* Remove outdated manifest loading in python files
* update unit tests to catch https://github.com/ros/geometry_experimental/issues/102
* Contributors: Chris Mansley, Michael Hwang, Tully Foote

1.11.7 (2015-04-21)
-------------------
* add a unit test for pytf wait_for_transform
* removed msg serv installation from cmakelists
* generated autodoc
* Fixed Vector3 documentation
* display RPY in both radian and degree
* Fixed command line arguments
* using TimeStamp and FrameId in message filter
  this allows to use tf::MessageFilter with pcl::PointCloud<T>
  see `#55 <https://github.com/ros/geometry/issues/55>`_
* Added and optional third argument to specify publishing frequency
* Contributors: Adnan Munawar, Brice Rebsamen, Jackie Kay, Tully Foote, Ying Lu

1.11.6 (2015-03-25)
-------------------
* reenable python tests
* Broadcaster: Rospy fix `#84 <https://github.com/ros/geometry/issues/84>`_. Add sendTransformMessage.
* Contributors: Tully Foote, lsouchet

1.11.5 (2015-03-17)
-------------------
* Strip leading slash get parent `#79 <https://github.com/ros/geometry/issues/79>`_ 
* Make frameExists strip leading slash going into tf2.`#63 <https://github.com/ros/geometry/issues/63>`_
* Update broadcaster.py,  Added ability to use TransformStamped
* update view_frames to use AllFramesAsDot(rospy.time.now()) `#77 <https://github.com/ros/geometry/issues/77>`_
* Contributors: David Lu!!, Gaël Ecorchard, Kei Okada, Tully Foote

1.11.4 (2014-12-23)
-------------------
* Install static lib and remove test for Android
* Larger default queue size in broadcaster
  With queue_size=1 when two transforms are sent in quick succession,
  the second is often lost. The C++ code uses a default queue_size of
  100, so switch to that default here as well. If that is not appropriate,
  a queue_size constructor argument is provided.
* Update package.xml
* add rate parameter to tf_echo
* Added check for normalized quaternion in roswtf plugin
* Contributors: David Lu!!, Gary Servin, Kevin Hallenbeck, Stephan Wirth, contradict

1.11.3 (2014-05-07)
-------------------
* convert to boost signals2 following `ros/ros_comm#267 <https://github.com/ros/ros_comm/issues/267>`_ Fixes `#23 <https://github.com/ros/geometry/issues/23>`_. Requires `ros/geometry_experimental#61 <https://github.com/ros/geometry_experimental/issues/61>`_ as well.
* add rospy publisher queue_size argument
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* add queue_size to tf publisher
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Contributors: Lukas Bulwahn, Tully Foote

1.11.2 (2014-02-25)
-------------------
* fixing test linking
* Contributors: Tully Foote

1.11.1 (2014-02-23)
-------------------

1.11.0 (2014-02-14)
-------------------
* TF uses ros::MessageEvent to get connection information
* Contributors: Kevin Watts, Tully Foote

1.10.8 (2014-02-01)
-------------------
* Port groovy-devel patch to hydro-devel
* Added rosconsole as catkin dependency for catkin_package
* Add rosconsole as runtime dependency
* Contributors: Michael Ferguson, Mirza Shah

1.10.7 (2013-12-27)
-------------------
* fix bug in tf::Matrix3x3::getEulerYPR()
  Fixes a bug in tf::Matrix3x3::getEulerYPR() implementation's handling
  of gimbal lock cases (when the new x axis aligns with the old +/-z
  axis).
* add test that demonstrated bug in tf::Matrix3x3
  tf::Matrix3x3::getEulerYPR() has a bug which returns an incorrect rpy
  for certain corner case inputs.  This test demonstrates that bug.
* Fix const correctness of tf::Vector3 rotate() method
  The method does not modify the class thus should be const.
  This has already been fixed in Bullet itself.
* add automatic tf buffer cleaning on bag loop for python
  This logic was already implemented for c++
  but not for the python module.
* Contributors: Acorn Pooley, Timo Rohling, Tully Foote, v4hn

1.10.6 (2013-08-28)
-------------------
* switching to wrapper scripts which will provide a deprecation warning for `#3 <https://github.com/ros/geometry/issues/3>`_
* add missing roswtf dependency to really export the plugin (fix `#27 <https://github.com/ros/geometry/issues/27>`_)
* Update listener.py
  Fix the tf listener service exception in rospy. See:
  http://answers.ros.org/question/10777/service-exception-using-tf-listener-in-rospy/
* Fix MessageFilter race condition
  If MessageFilter does not explicitly stop its callback timer when it's
  being destroyed, there is a race condition when that timer is processed in
  a callback queue run by a different thread.  Specifically,
  maxRateTimerCallback() may be called after messages_mutex_ has been
  destroyed, causing a unrecoverable error.

1.10.5 (2013-07-19)
-------------------
* tf: export dependency on tf2_ros
  Fixes `#21 <https://github.com/ros/geometry/issues/21>`_
* added run dependency on graphviz
  closes `#19 <https://github.com/ros/geometry/issues/19>`_

1.10.4 (2013-07-11)
-------------------
* fixing erase syntax
* resolving https://github.com/ros/geometry/issues/18 using implementation added in tf2::BufferCore, adding dependency on next version of tf2 for this

1.10.3 (2013-07-09)
-------------------
* fixing unittest for new resolve syntax

1.10.2 (2013-07-09)
-------------------
* strip leading slashes in resolve, and also any time a method is passed from tf to tf2 assert the leading slash is stripped as well.  tf::resolve with two arguments will end up with foo/bar instead of /foo/bar.  Fixes https://github.com/ros/geometry_experimental/issues/12
* added two whitespaces to make message_filter compile with c++11
  more on this here: http://stackoverflow.com/questions/10329942/error-unable-to-find-string-literal-operator-slashes
* using CATKIN_ENABLE_TESTING to optionally configure tests in tf

1.10.1 (2013-07-05)
-------------------
* updating dependency requirement to tf2_ros 0.4.3
* removing unused functions
  removing unused private methods
  removing ``max_extrapolation_distance_``
  removing unused data storage _frameIDs frameIDS_reverse ``frame_authority_``
  removing cache_time from tf, passing through method to tf2 buffer_core
  removing unused variables ``frames_`` and ``frame_mutex_`` and ``interpolating_``
  removing unused mutex and transformchanged signaling
  commenting on deprecation of MAX_EXTRAPOLATION_DISTANCE

1.10.0 (2013-07-05)
-------------------
* adding versioned dependency on recent geometry_experimental changes
* fixing test dependencies
* fixing callbacks for message filters
* remove extra invalid comment
* dedicated thread logic all implemented
* removing commented out code
* mostly completed conversion of tf::TransformListener to use tf2 under the hood
* lookuptwist working
* tf::Transformer converted to use tf2::Buffer under the hood.  passing tf_unittest.cpp
* making tf exceptions typedefs of tf2 exceptions for compatability
* first stage of converting Transformer to Buffer
* switching to use tf2's TransformBroadcaster
* adding dependency on tf2_ros to start moving over contents
* fixing unit tests

1.9.31 (2013-04-18 18:16)
-------------------------

1.9.30 (2013-04-18 16:26)
-------------------------
* Adding correct install targets for tf scripts
* Removing scripts from setup.py install

1.9.29 (2013-01-13)
-------------------
* use CATKIN_DEVEL_PREFIX instead of obsolete CATKIN_BUILD_PREFIX

1.9.28 (2013-01-02)
-------------------

1.9.27 (2012-12-21)
-------------------
* set addditional python version
* added license headers to various files

1.9.26 (2012-12-14)
-------------------
* add missing dep to catkin

1.9.25 (2012-12-13)
-------------------
* add missing downstream depend
* update setup.py

1.9.24 (2012-12-11)
-------------------
* Version 1.9.24

1.9.23 (2012-11-22)
-------------------
* Releaseing version 1.9.23
* tf depended on angles but did not find_package it

1.9.22 (2012-11-04 09:14)
-------------------------

1.9.21 (2012-11-04 01:19)
-------------------------

1.9.20 (2012-11-02)
-------------------

1.9.19 (2012-10-31)
-------------------
* fix catkin function order
* Removed deprecated 'brief' attribute from <description> tags.

1.9.18 (2012-10-16)
-------------------
* tf: Fixed wrong install directory for python message files.
* tf: fixed bug where generated python message code was not being installed.
* tf: added setup.py file and changed CMakeLists.txt to install python files and bound library (_tf.so, also known as pytf_py in CMakeLists.txt) which must have been missed during the previous catkin-ization.

1.9.17 (2012-10-02)
-------------------
* fix several dependency issues

1.9.16 (2012-09-29)
-------------------
* adding geometry metapackage and updating to 1.9.16

1.9.15 (2012-09-30)
-------------------
* fix a few dependency/catkin problems
* remove old API files
* comply to the new catkin API

1.9.14 (2012-09-18)
-------------------
* patch from Tom Ruehr from tf sig
* patch from `#5401 <https://github.com/ros/geometry/issues/5401>`_ for c++0x support

1.9.13 (2012-09-17)
-------------------
* update manifests

1.9.12 (2012-09-16)
-------------------
* use the proper angles package

1.9.11 (2012-09-14 22:49)
-------------------------
* no need for angles anymore

1.9.10 (2012-09-14 22:30)
-------------------------
* no need for bullet anymore

1.9.9 (2012-09-11)
------------------
* update depends
* minor patches for new build system

1.9.8 (2012-09-03)
------------------
* fixes for groovy's catkin

1.9.7 (2012-08-10 12:19)
------------------------

1.9.6 (2012-08-02 19:59)
------------------------
* changing how we install bins

1.9.5 (2012-08-02 19:48)
------------------------
* fix the header to be compiled properly
* using PROGRAMS insteas of TARGETS

1.9.4 (2012-08-02 18:29)
------------------------

1.9.3 (2012-08-02 18:28)
------------------------
* forgot to install some things
* also using DEPENDS

1.9.2 (2012-08-01 21:05)
------------------------
* make sure the tf target depends on the messages (and clean some include_directories too)

1.9.1 (2012-08-01 19:16)
------------------------
* install manifest.xml

1.9.0 (2012-08-01 18:52)
------------------------
* catkin build system
* remove bullet dep
* fix bug `#5089 <https://github.com/ros/geometry/issues/5089>`_
* add link flag for OSX
* tf: MessageFilter: added public getter/setter for ``queue_size_``
* adding btQuaternion constructor for ease of use
* fixing method naming for camelCase and adding bt* Constructor methods
* tf.tfwtf now uses rosgraph.Master instead of roslib
* Added tf and angles to catkin
* cleanup up last errors
* ``SIMD_`` -> ``TFSIMD_`` defines to not conflict
* write in bullet assignment and return methods
* executable bit on conversion script
* changing defines from BT to TF
* removing BULLET_VERSION info
* changing all bt* to tf* in LinearMath to avoid collisions
* convert btScalar to tfScalar to avoid definition conflicts
* deleting GEN_clamp and GEN_clamped as they're unused and would conflict
* non conflicting minmax functions
* the migration script
* applied bullet_migration_sed.py to LinearMath include dir with namespaced rules and everything with Namespaced rules and all 152 tests pass
* removing all BT_USE_DOUBLE_PRECISION ifs and hardcoding them to the double case
* adding tf namespaces to moved files
* breaking bullet dependency
* removing redundant typedefs with new datatypes
* moving filenames to not collide in search and replaces
* changing include guards
* moving linear math into tf namespace
* copying in bullet datatypes
* switching to a recursive mutex and actually holding locks for the right amount of time.  ticket:5
* Giving error message when time cache is empty for lookup failures
* Moving ``lct_cache_`` to local variable from class member. As class member, using this variable makes lookupTransform not thread-safe
* velocity test precision a little lower requirements
* Fix to error message for earliest extrapolation time exception, ros-pkg5085
* Fixing epsilon to prevent test failures
* Reducing epsilon value for velocity tests
* add missing empty_listener.cpp file
* Not calling ros::Time::now() in tf.cpp, causes problems with pytf
* fix for ROS_BREAK include
* Adding faster point cloud transform, as specified in ros-pkg`#4958 <https://github.com/ros/geometry/issues/4958>`_
* Cache unittest reenabled
* Adding speed_test from tf2 to check lookupTransform/canTransform
* Josh's optimizations from tf2 merged into tf. Tests pass
* Benchmark test includes tests of lookupTransform
* Adding ros::Time::init to benchmark test
* Testing compound transforms with lookupTransform
* Adding helix test of time-varying transforms, with interpolation, to test lookupTransform
* Moving test executables to bin/. Cleanup in tf_unittest. Removed deprecated calls to bullet, added 'ring45' test from tf2 as lookupTransform test
* patch for `#4952 <https://github.com/ros/geometry/issues/4952>`_
* kevin's patch for #ros-pkg4882
* Fix for TransformListener hanging on simulation shutdown, `#4882 <https://github.com/ros/geometry/issues/4882>`_
* removing old srv export
* removing old srv includ path
* this should never have been passing in an error string here -- likely one of the reasons MessageFilter is so slow
* Adding to author list to create branch
* removing reset_time topic and catching negative time change to reset the tf buffer
* `#4277 <https://github.com/ros/geometry/issues/4277>`_ transformPointCloud
* revert patch that uses ros::ok in waitForTransform. ticket `#4235 <https://github.com/ros/geometry/issues/4235>`_
* make tf unittest a ros node
* fix lockup in waitForTransform. ticket 4235
* reverting r30406 and r30407, they are redundant with standardized functionality and break previous functionality
* sse detection `#4114 <https://github.com/ros/geometry/issues/4114>`_
* tf: change_notifier should sleep after an exception
* created common place for ROS Pose/Point/Quaternion to numpy arrays transformations
* added TransformBroadcaster.sendTransform for PoseStamped
* one more patch for `#4183 <https://github.com/ros/geometry/issues/4183>`_
* new unit test
* waitforTransform now polls on walltime to avoid ros::Time initialization issues.  basic unit test for waitForTransform in python.
* fix for stricter time
* fix ros::Time unit test problem with ROS 1.1.9
* `#4103 <https://github.com/ros/geometry/issues/4103>`_ method getTFPrefix() added, documented, tested
* moving patch to trunk from tag r30172
* Added Ubuntu platform tags
* Update MessageFilter to use traits and MessageEvent
* `#4039 <https://github.com/ros/geometry/issues/4039>`_, moved PoseMath from tf to tf_conversions
* `#4031 <https://github.com/ros/geometry/issues/4031>`_ add lookupTwist and lookupTwistFull
* fixing zero time edge case of lookupTwist, thanks james
* commenting debug statement
* Typo in comment
* documentation
* fixing up unit tests
* lookup twist for `#4010 <https://github.com/ros/geometry/issues/4010>`_
* commenting twist test while the code is being refactored
* removing transform twist as per api review in ticket `#4010 <https://github.com/ros/geometry/issues/4010>`_
* Added doctest for PoseMath creation from message
* Doc for PoseMath
* Double module tf
* Remove expect_exception
* comment for operator
* opeartor == for StampedTransform too `#3990 <https://github.com/ros/geometry/issues/3990>`_
* First cut at posemath
* adding operator== to Stamped<T> with unit tests
* adding methods for vectorized publishing of transforms `#3954 <https://github.com/ros/geometry/issues/3954>`_
* fix thread-safety of add()
* Re-add message filter test that was accidentally removed when the message notifier was deleted
* Fix message filter in the case where messages are arriving faster than the update timer is running (exacerbated by rosbag play --clock not actually broadcasting the clock at 100hz). (`#3810 <https://github.com/ros/geometry/issues/3810>`_)
* Tiny refactor for callerid->authority
* `#3942 <https://github.com/ros/geometry/issues/3942>`_ testcase
* Add doc for Transformer.clear
* Missing initializer from TransformListener
* New test test_cache_time
* fixing quaternion checking and adding unittests `#3758 <https://github.com/ros/geometry/issues/3758>`_
* review status `#3776 <https://github.com/ros/geometry/issues/3776>`_
* tf: change_notifier now supports multiple frames; publishes tfMessages
* passing basic tests for transformtwist
* adding transformTwist method
* all tests passing on lookupVelocity
* tests for values calculated by hand
* linear velocity to multiple other targets
* expanding to all three dimentions and asserting others are zero
* first cut velocity, basic test architecture layed out.
* searchparam when publishing
* noting deprecations better and changing frame_id to frame_name for unresolved
* removing /tf_message since it's been deprecated
* returning remap capability to remap `#3602 <https://github.com/ros/geometry/issues/3602>`_
* inlining helper function
* tf: changed manifest to have lower-case tf
* comment
* more documentation
* adding helper function for getting tf_prefix
* patches for tf_monitor to correctly display the chain, thanks for the help Sachin.
* asserting that incoming frameids are resolved, currently at debug level as this is not fully implemented in othe code.  This level will escalate slowly as compliance is increased `#3169 <https://github.com/ros/geometry/issues/3169>`_
* not using my own deprecated function
* more usage
* tf_echo usage expanded
* fixing typo in documentation
* removing include of message_notifier
* removing deprecated message_notifier `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated data type and constructor `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated sendTransform calls
* fixing test for usage of deprecated APIs `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated setTransform method `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated lookupTransform methods `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removed deprecated canTransform method `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated canTransform `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated transform_sender `#3046 <https://github.com/ros/geometry/issues/3046>`_
* removing deprecated transformStampedMsgToTF and transformStampedTFToMsg `#3046 <https://github.com/ros/geometry/issues/3046>`_
* fixing startup race condition `#3168 <https://github.com/ros/geometry/issues/3168>`_
* adding InvalidArgument exception for transformMethods, currently it only throws if w in quaternions are w <= 0 or w > 1 `#3236 <https://github.com/ros/geometry/issues/3236>`_
* reving for release
* commenting all velocity work for it's not ready to be released
* adding in deprecated call which I removed accidentally
* renaming tf::remap to tf::resolve as per `#3190 <https://github.com/ros/geometry/issues/3190>`_ with backwards compatability.  Also Standardizing to only do searchparam at startup `#3167 <https://github.com/ros/geometry/issues/3167>`_
* Switch MessageFilter back to using a Timer instead of a WallTimer, since the time-jumping is now fixed (`#2430 <https://github.com/ros/geometry/issues/2430>`_)
* adding createQuaternionFromRPY method to help deprecation transition `#2992 <https://github.com/ros/geometry/issues/2992>`_
* Added specific tes for quaternion types
* Switching refernece frame and moving frame ordering for lookup transform call to actually be correct
* adding test to the back
* fixing lookupVelocity special cases for zero time
* documention
  improvements
* Doc clarifications
* removing debugging
* lookupVelocity Python first cut
* transformVector3
* switching tf_prefix to searchParam so you can set it for a whole namespace `#2921 <https://github.com/ros/geometry/issues/2921>`_
* removing .py extension from script
* simpler topic name
* adding tf_remapping script to remap frame ids `#870 <https://github.com/ros/geometry/issues/870>`_
* fixing manifest loading to right package
* uncommenting lookup velocity and fixing implementation
* removing redundant angles package dependency `#3334 <https://github.com/ros/geometry/issues/3334>`_
* Patch from `#3337 <https://github.com/ros/geometry/issues/3337>`_
* fixing ~ usage
* commenting out lookupvelocity while it's still not working for release of patches
* angles needed for velocity lookup
* Switch from to_seconds to to_sec, `#3324 <https://github.com/ros/geometry/issues/3324>`_
* updating for 0.10 changes to python and hudson
* fixing deprecated to_seconds call in tfwtf
* merging 0.4x changes into trunk
* a first trial of lookupVelocity
* added createQuaternionMsgFromRollPitchYaw helper function
* removing wait_for_transform_death test from default, for it doesn't work under release
* switching to Release from Debug
* fixing usage message of static_transform_sender
* Warn about received messages with frame_ids that are not fully qualified, but resolve them locally
* moving deprecation note to top of summary
* * Remap target frames (`#3119 <https://github.com/ros/geometry/issues/3119>`_)
  * Throw out messages immediately if they have an empty frame_id
* fixing display of chain to show all links
* documentation for `#2072 <https://github.com/ros/geometry/issues/2072>`_
* fixing frequency output of tf_monitor
* making remapping on send more consistent
* removing unused variable
* Doxygen comments for the failure reasons
* Add a failure callback to tf::MessageFilter
* fixing `#2990 <https://github.com/ros/geometry/issues/2990>`_ deprecated ~ call
* update tf error strings. Still need review and user testing
* notifier should subscribe to tf and tf_message
* doc: updated setTransform to properly list child_frame_id
* Doc clearer on exceptions
* restoring caller_id to graph view in python
* Set daemon on listener thread
* better command line outputs
* Removed turtlesim reference from python broadcaster
* removing useages of deprecated bullet APIs
* Add rosdoc to manifest
* Fix build break
* New Sphinx docs
* changing display of legend to be above the tree
* make output consistent with view frames
* tweak output of tf_echo
* tweek output of tf_echo
* update output string
* update output of view frames
* make tf_echo wait for up to one second before throwing exceptions
* Fixes for pytf: exception distinction, waitForTransform, threaded listener
* Switch MessageFilter back to a WallTimer... shouldn't have been checked in with my last checkin
* Remove last remnants of Node use
* Fix compiler warnings
* removing last warnings relating to `#2477 <https://github.com/ros/geometry/issues/2477>`_
* tf monitor working, and a little bit cleaner display
* fixing useage of ~ params
* cleaning up tf_echo output
* fixing warning
* static_transform_publsher replacing transform_sender for backwards compatability, and fixing new StampedTransform
* update tf description
* remove extra / in method def. Ticket `#2778 <https://github.com/ros/geometry/issues/2778>`_
* fixed deprecation of Stamped<> 4 constructor vs 3 constructor.  and switched three usages `#2477 <https://github.com/ros/geometry/issues/2477>`_
* converting transformPointCloud to use new StampedTransform `#2477 <https://github.com/ros/geometry/issues/2477>`_
* fixing warnings related to `#2477 <https://github.com/ros/geometry/issues/2477>`_
* internally switching to StampedTransform for `#2477 <https://github.com/ros/geometry/issues/2477>`_ cleanup
* fixing usage of Stamped<Transform> to StampedTransform
* switching Stamped<btTransform> to StampedTransform, deprecating usage, and changing all APIs to the new one with backwards compatabilty `#2477 <https://github.com/ros/geometry/issues/2477>`_. It's working but lots of warnings left to fix
* removing warning
* fixing deprecated function call usage
* one less node API call
* one less node usage
* fixing urls for new server
* Rename tf message from \tf_message to \tf. Listener is backwards compatible, broadcaster is not. See ticket `#2381 <https://github.com/ros/geometry/issues/2381>`_
* migration part 1
