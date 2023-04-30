# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

from __future__ import print_function

import rostest
import rospy
import numpy
import unittest
import sys
import time
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO


import tf.transformations
import geometry_msgs.msg

from tf.msg import tfMessage

import tf

iterations = 10000

t = tf.Transformer()
def mkm():
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = "PARENT"
    m.child_frame_id = "THISFRAME"
    m.transform.translation.y = 5.0
    m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    return m

tm = tfMessage([mkm() for i in range(20)])

def deserel_to_string(o):
    s = StringIO()
    o.serialize(s)
    return s.getvalue()

mstr = deserel_to_string(tm)

class Timer:
    def __init__(self, func):
        self.func = func
    def mean(self, iterations = 1000000):
        started = time.time()
        for i in range(iterations):
            self.func()
        took = time.time() - started
        return took / iterations
        
import tf.msg
import tf.cMsg
for t in [tf.msg.tfMessage, tf.cMsg.tfMessage]:
    m2 = t()
    m2.deserialize(mstr)
    for m in m2.transforms:
        print(type(m), sys.getrefcount(m))
    assert deserel_to_string(m2) == mstr, "deserel screwed up for type %s" % repr(t)

    m2 = t()
    print("deserialize only {} us each".format(1e6 * Timer(lambda: m2.deserialize(mstr)).mean())

sys.exit(0)

started = time.time()
for i in range(iterations):
    for m in tm.transforms:
        t.setTransform(m)
took = time.time() - started
print("setTransform only {} took {} us each".format(iterations, took, (1e6 * took / iterations)))

started = time.time()
for i in range(iterations):
    m2 = tfMessage()
    m2.deserialize(mstr)
    for m in m2.transforms:
        t.setTransform(m)
took = time.time() - started
print("deserialize+setTransform {} took {} us each".format(iterations, took, (1e6 * took / iterations)))

from tf import TransformListener
