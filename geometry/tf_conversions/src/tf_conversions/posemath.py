# Copyright (c) 2010, Willow Garage, Inc.
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

from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import tf
import rospy
import numpy

from PyKDL import *

def fromTf(tf):
    """
    :param tf: :class:`tf.Transformer` transform
    :type tf: tuple (translation, quaternion)
    :return: New :class:`PyKDL.Frame` object

    Convert a pose returned by :meth:`tf.Transformer.lookupTransform` to a :class:`PyKDL.Frame`.

    .. doctest::

        >>> import rospy
        >>> import tf
        >>> import geometry_msgs.msg
        >>> t = tf.Transformer(True, rospy.Duration(10.0))
        >>> m = geometry_msgs.msg.TransformStamped()
        >>> m.header.frame_id = 'THISFRAME'
        >>> m.child_frame_id = 'CHILD'
        >>> m.transform.translation.x = 668.5
        >>> m.transform.rotation.w = 1.0
        >>> t.setTransform(m)
        >>> t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0))
        ((668.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        >>> import tf_conversions.posemath as pm
        >>> p = pm.fromTf(t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0)))
        >>> print(pm.toMsg(p * p))
        position: 
          x: 1337.0
          y: 0.0
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0

    """

    position, quaternion = tf
    x, y, z = position
    Qx, Qy, Qz, Qw = quaternion
    return Frame(Rotation.Quaternion(Qx, Qy, Qz, Qw), 
                 Vector(x, y, z))

def toTf(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a tuple (position, quaternion) for the pose.
    """

    return ((f.p[0], f.p[1], f.p[2]), f.M.GetQuaternion())

# to and from pose message
def fromMsg(p):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    """
    return Frame(Rotation.Quaternion(p.orientation.x,
                                     p.orientation.y,
                                     p.orientation.z,
                                     p.orientation.w),  
                 Vector(p.position.x, p.position.y, p.position.z))

def toMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS Pose message for the Frame f.

    """
    p = Pose()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = f.M.GetQuaternion()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    return p


# to and from matrix
def fromMatrix(m):
    """
    :param m: input 4x4 matrix
    :type m: :func:`numpy.array`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a 4x4 numpy array to a :class:`PyKDL.Frame`.

    """
    return Frame(Rotation(m[0,0], m[0,1], m[0,2],
                          m[1,0], m[1,1], m[1,2],
                          m[2,0], m[2,1], m[2,2]),  
                 Vector(m[0,3], m[1, 3], m[2, 3]))
    
def toMatrix(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a numpy 4x4 array for the Frame F.
    """
    return numpy.array([[f.M[0,0], f.M[0,1], f.M[0,2], f.p[0]],
                        [f.M[1,0], f.M[1,1], f.M[1,2], f.p[1]],
                        [f.M[2,0], f.M[2,1], f.M[2,2], f.p[2]],
                        [0,0,0,1]])


# from camera parameters
def fromCameraParams(cv, rvec, tvec):
    """
    :param cv: OpenCV module
    :param rvec: A Rodrigues rotation vector - see :func:`Rodrigues2`
    :type rvec: 3x1 :class:`CvMat`
    :param tvec: A translation vector 
    :type tvec: 3x1 :class:`CvMat`
    :return: New :class:`PyKDL.Frame` object
    
    For use with :func:`FindExtrinsicCameraParams2`::

        import cv
        import tf_conversions.posemath as pm
        ...
        rvec = cv.CreateMat(3, 1, cv.CV_32FC1)
        tvec = cv.CreateMat(3, 1, cv.CV_32FC1)
        cv.FindExtrinsicCameraParams2(model, corners, intrinsic_matrix, kc, rvec, tvec)
        pose = pm.fromCameraParams(cv, rvec, tvec)

    """
    m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                      [ 0, 0, 0, tvec[1,0] ], 
                      [ 0, 0, 0, tvec[2,0] ], 
                      [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
    cv.Rodrigues2(rvec, m[:3,:3])
    return fromMatrix(m)


