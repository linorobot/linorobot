# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import rospy
import tf
import time
import bullet
import math

try:

    rospy.init_node("test_node")
    tfl = tf.TransformListener()
    time.sleep(1)

    # View all frames
    print("All frames are:\n", tfl.all_frames_as_string())

    # dot based introspection
    print("All frame graph is:\n", tfl.all_frames_as_dot())

    
    # test transforming pose
    po = tf.PoseStamped()
    po.frame_id = "base_link"
    print("calling transform pose")
    po2 = tfl.transform_pose("/map", po)

    print("po2.pose.this", po2.pose.this)
    print("po.pose.this", po.pose.this)

    # test transforming point
    po = tf.PointStamped()
    po.frame_id = "base_link"
    po3 = tfl.transform_point("/map", po)
    print("po3")
    print(po3.this)

    # test transforming vector
    po = tf.VectorStamped()
    po.frame_id = "base_link"
    po4 = tfl.transform_vector("/map", po)
    print(po4.this)
    
    # test transforming quaternion
    po = tf.QuaternionStamped()
    po.frame_id = "base_link"
    po5 = tfl.transform_quaternion("/map", po)
    print("po5",  po5.this)
    
    tr = tf.TransformStamped()

    lps = tf.PoseStamped()
    lps.pose.setIdentity()
    print("setting stamp")
    mytime = rospy.Time(10,20)
    lps.stamp = mytime
    print(mytime)
    print("getting stamp")
    output = lps.stamp
    print(output)
    print(lps.pose)
    print("setting pose.positon to 1,2,3")
    lps.pose.setOrigin( bullet.Vector3(1,2,3))
    print(lps.pose.getOrigin())
    print(lps.pose)

    transform_stamped = tf.TransformStamped()
    print("getting stamp")
    print(transform_stamped.stamp)
#    mytime = rospy.Time().now()
    mytime = rospy.Time(10,20)
    transform_stamped.stamp = mytime
    print(mytime)
    print("getting stamp", transform_stamped.stamp)
    print("transform:", transform_stamped.transform)
    transform_stamped.transform.setIdentity()
    print("after setIdentity()", transform_stamped.transform)
    #    transform_stamped.transform.basis.setEulerZYX(0,0,0)
    quat = bullet.Quaternion(math.pi/2,0,0)
    print("quaternion ", quat)
    transform_stamped.transform.setRotation(quat)
    print("setting rotation to PI/2",transform_stamped.transform)


    pointstamped = tf.PointStamped()
    print("getting stamp")
    print(pointstamped.stamp)
#    mytime = rospy.Time().now()
    mytime = rospy.Time(10,20)
    pointstamped.stamp = mytime
    print(mytime)
    print("getting stamp")
    output = pointstamped.stamp
    print(output)
    print(pointstamped.point)
    print(transform_stamped.transform * pointstamped.point)

    pose_only = bullet.Transform(transform_stamped.transform)
    print("destructing pose_only", pose_only.this    )
    pose_only = []

    
    print("Creating copy")
    po2_copy = tf.PoseStamped(po2)
    print("po2_copy.pose", po2_copy.pose.this)
    print("po2.pose", po2.pose.this)

    print("Creating copy2")
    po2_copy2 = tf.PoseStamped(po2)
    print("po2_copy2.pose", po2_copy2.pose.this)


    print("destructing po2  po2.pose is", po2.pose.this)
    po2 = []


    print("destructing po2_copy po2_copy.pose is", po2_copy.pose.this)
    po2_copy = []    

    

    


    print("done")

except ValueError as e:
    print("Exception {} Improperly thrown: {}".format(type(e), e))


