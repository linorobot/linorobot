# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

from __future__ import print_function

import time

from roswtf.rules import warning_rule, error_rule
import rosgraph
import rospy

import tf.msg

import math


# global list of messages received
_msgs = []

################################################################################
# RULES

def rostime_delta(ctx):
    deltas = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            d = t.header.stamp - stamp
            secs = d.to_sec()
            if abs(secs) > 1.:
                if callerid in deltas:
                    if abs(secs) > abs(deltas[callerid]):
                        deltas[callerid] = secs
                else:
                    deltas[callerid]  = secs

    errors = []
    for k, v in deltas.items():
        errors.append("receiving transform from [{}] that differed from ROS time by {}s".format(k, v))
    return errors

def reparenting(ctx):
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            if frame_id in parent_id_map and parent_id_map[frame_id] != parent_id:
                msg = "reparenting of [{}] to [{}] by [{}]".format(frame_id, parent_id, callerid)
                parent_id_map[frame_id] = parent_id
                if msg not in errors:
                    errors.append(msg)
            else:
                parent_id_map[frame_id] = parent_id
    return errors

def cycle_detection(ctx):
    max_depth = 100
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            parent_id_map[frame_id] = parent_id

    for frame in parent_id_map:
        frame_list = []
        current_frame = frame
        count = 0
        while count < max_depth + 2:
            count = count + 1
            frame_list.append(current_frame)
            try:
                current_frame = parent_id_map[current_frame]
            except KeyError:
                break
            if current_frame == frame:
                errors.append("Frame {} is in a loop. It's loop has elements:\n{}".format(frame, " -> ".join(frame_list)))
                break

    return errors

def multiple_authority(ctx):
    errors = []
    authority_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id 
            if frame_id in authority_map and authority_map[frame_id] != callerid:
                msg = "node [{}] publishing transform [{}] with parent [{}] already published by node [{}]".format(callerid, frame_id, parent_id, authority_map[frame_id])
                authority_map[frame_id] = callerid
                if msg not in errors:
                    errors.append(msg)
            else:
                authority_map[frame_id] = callerid
    return errors

def no_msgs(ctx):
    return not _msgs

def not_normalized(ctx):
    errors = []
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            q = t.transform.rotation
            length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if math.fabs(length - 1) > 1e-6:
                errors.append("rotation from [{}] to [{}] is not unit length, {}".format(t.header.frame_id, t.child_frame_id, length))
    return errors

################################################################################
# roswtf PLUGIN

# tf_warnings and tf_errors declare the rules that we actually check

tf_warnings = [
  (no_msgs, "No tf messages"),
  (rostime_delta, "Received out-of-date/future transforms:"),  
  (not_normalized, "Received non-normalized rotation in transforms:"),
]
tf_errors = [
  (reparenting, "TF re-parenting contention:"),
  (cycle_detection, "TF cycle detection::"),
  (multiple_authority, "TF multiple authority contention:"),
]

# rospy subscriber callback for /tf
def _tf_handle(msg):
    _msgs.append((msg, rospy.get_rostime(), msg._connection_header['callerid']))

# @return bool: True if /tf has a publisher
def is_tf_active():
    master = rosgraph.Master('/tfwtf')
    if master is not None:
        try:
            val = master.getPublishedTopics('/')
            if filter(lambda x: x[0] == '/tf', val):
                return True
        except:
            pass
    return False

# roswtf entry point for online checks
def roswtf_plugin_online(ctx):
    # don't run plugin if tf isn't active as these checks take awhile
    if not is_tf_active():
        return

    print("running tf checks, this will take a second...")
    sub1 = rospy.Subscriber('/tf', tf.msg.tfMessage, _tf_handle)
    time.sleep(1.0)
    sub1.unregister()
    print("... tf checks complete")

    for r in tf_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in tf_errors:
        error_rule(r, r[0](ctx), ctx)

# currently no static checks for tf
#def roswtf_plugin_static(ctx):
#    for r in tf_warnings:
#        warning_rule(r, r[0](ctx), ctx)
#    for r in tf_errors:
#        error_rule(r, r[0](ctx), ctx)
