# Copyright (c) 2008, Willow Garage, Inc.
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

import rospy
import numpy
import yaml

import geometry_msgs.msg
import sensor_msgs.msg

import tf2_ros
from . import transformations

def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

def strip_leading_slash(s):
    return s[1:] if s.startswith("/") else s

## Proxy Transformer class to call TF2 methods
class Transformer(object):

    def __init__(self, interpolate=True, cache_time=None):
        self._buffer = tf2_ros.Buffer(cache_time, debug=False)
        self._using_dedicated_thread = False

    def allFramesAsDot(self, current_time=None):
        if current_time:
            return self._buffer._allFramesAsDot(current_time)
        return self._buffer._allFramesAsDot()

    def allFramesAsString(self):
        return self._buffer.all_frames_as_string()

    def setTransform(self, transform, authority="default_authority"):
        self._buffer.set_transform(transform, authority)

    def canTransform(self, target_frame, source_frame, time):
        return self._buffer.can_transform(strip_leading_slash(target_frame), strip_leading_slash(source_frame), time)

    def canTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self._buffer.can_transform_full(strip_leading_slash(target_frame), target_time, strip_leading_slash(source_frame), source_time, strip_leading_slash(fixed_frame))

    def waitForTransform(self, target_frame, source_frame, time, timeout, polling_sleep_duration=None):
        if not self._using_dedicated_thread:
            raise tf2_ros.TransformException("cannot wait for transform without a dedicated thread that listens to incoming TF messages")
        can_transform, error_msg = self._buffer.can_transform(strip_leading_slash(target_frame), strip_leading_slash(source_frame), time, timeout, return_debug_tuple=True)
        if not can_transform:
            raise tf2_ros.TransformException(error_msg or "no such transformation: \"{}\" -> \"{}\"".format(source_frame, target_frame))

    def waitForTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout, polling_sleep_duration=None):
        if not self._using_dedicated_thread:
            raise tf2_ros.TransformException("cannot wait for transform without a dedicated thread that listens to incoming TF messages")
        can_transform, error_msg = self._buffer.can_transform_full(strip_leading_slash(target_frame), target_time, strip_leading_slash(source_frame), source_time, strip_leading_slash(fixed_frame), timeout, return_debug_tuple=True)
        if not can_transform:
            raise tf2_ros.TransformException(error_msg or "no such transformation: \"{}\" -> \"{}\"".format(source_frame, target_frame))

    def chain(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self._buffer._chain( target_frame, target_time, source_frame, source_time, fixed_frame)

    def clear(self):
        self._buffer.clear()

    def frameExists(self, frame_id):
        """ Not a recommended API, only here for backwards compatibility """
        return frame_id in self.getFrameStrings()

    def getFrameStrings(self):
        """ Not a recommended API, only here for backwards compatibility """
        data = yaml.load(self._buffer.all_frames_as_yaml()) or {}
        return [p for p, _ in data.items()]

    def getLatestCommonTime(self, source_frame, dest_frame):
        return self._buffer.get_latest_common_time(strip_leading_slash(source_frame), strip_leading_slash(dest_frame))

    def lookupTransform(self, target_frame, source_frame, time):
        msg = self._buffer.lookup_transform(strip_leading_slash(target_frame), strip_leading_slash(source_frame), time)
        t = msg.transform.translation
        r = msg.transform.rotation
        return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

    def lookupTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        msg = self._buffer.lookup_transform_full(strip_leading_slash(target_frame), target_time, strip_leading_slash(source_frame), source_time, strip_leading_slash(fixed_frame))
        t = msg.transform.translation
        r = msg.transform.rotation
        return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

    def lookupTwist(self, tracking_frame, observation_frame, time, averaging_interval):
        return self.lookupTwistFull(tracking_frame, observation_frame, observation_frame, (0, 0, 0), tracking_frame, time, averaging_interval)

    def lookupTwistFull(self, tracking_frame, observation_frame, reference_frame, ref_point, reference_point_frame, time, averaging_interval):
        latest_time = self.getLatestCommonTime(observation_frame, tracking_frame)
        target_time = time or latest_time
        end_time = min(target_time + rospy.Duration(0.5 * averaging_interval.to_sec()), latest_time)
        start_time = max(rospy.Time(0.0001) + averaging_interval, end_time) - averaging_interval
        delta_t = (end_time - start_time).to_sec()

        start_tr, start_rt = self.lookupTransform(observation_frame, tracking_frame, start_time)
        end_tr, end_rt = self.lookupTransform(observation_frame, tracking_frame, end_time)
        dR = numpy.dot(numpy.linalg.inv(transformations.quaternion_matrix(start_rt)), transformations.quaternion_matrix(end_rt))
        ang, o, _ = transformations.rotation_from_matrix(dR)
        delta_x, delta_y, delta_z = end_tr[0] - start_tr[0], end_tr[1] - start_tr[1], end_tr[2] - start_tr[2]

        # Compute twist in observation_frame w.r.t. tracking_frame
        vel0 = delta_x / delta_t, delta_y / delta_t, delta_z / delta_t
        rot0 = o[0] * ang / delta_t, o[1] * ang / delta_t, o[2] * ang / delta_t

        # Shift to reference_frame
        inverse_tr, inverse_rt = self.lookupTransform(reference_frame, tracking_frame, target_time)
        iR = transformations.quaternion_matrix(inverse_rt)[:3, :3]
        rot = numpy.dot(iR, rot0)
        vel = numpy.dot(iR, vel0) + numpy.cross(inverse_tr, rot)

        # Correct for reference point
        rp_orig = numpy.array((inverse_tr[0], inverse_tr[1], inverse_tr[2], 1))
        rp_tr, rp_rt = self.lookupTransform(reference_frame, reference_point_frame, target_time)
        T = numpy.dot(transformations.translation_matrix(rp_tr), transformations.quaternion_matrix(rp_rt))
        rp_desired = numpy.dot(T, (rp_orig[0], rp_orig[1], rp_orig[2], 1))
        delta = rp_desired - rp_orig
        vel += numpy.dot(rot, delta[:3])

        return (vel[0], vel[1], vel[2]), (rot[0], rot[1], rot[2])

    def setUsingDedicatedThread(self, value):
        self._using_dedicated_thread = value

    def getTFPrefix(self):
        # The tf2 resolver does not support TF prefixes, so we return the empty prefix here
        return ""


## Extends tf's Transformer, adding transform methods for ROS message
## types PointStamped, QuaternionStamped and PoseStamped.
class TransformerROS(Transformer):
    """
    TransformerROS extends the base class :class:`tf.Transformer`,
    adding methods for handling ROS messages. 
    """

    ## Looks up the transform for ROS message header hdr to frame
    ## target_frame, and returns the transform as a Numpy 4x4 matrix.
    # @param target_frame The target frame
    # @param hdr          A ROS message header object

    def asMatrix(self, target_frame, hdr):
        """
        :param target_frame: the tf target frame, a string
        :param hdr: a message header
        :return: a :class:`numpy.matrix` 4x4 representation of the transform
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
        
        Uses :meth:`lookupTransform` to look up the transform for ROS message header hdr to frame
        target_frame, and returns the transform as a :class:`numpy.matrix`
        4x4.
        """
        translation,rotation = self.lookupTransform(target_frame, hdr.frame_id, hdr.stamp)
        return self.fromTranslationRotation(translation, rotation)

    ## Returns a Numpy 4x4 matrix for a transform.
    # @param translation  translation as (x,y,z)
    # @param rotation     rotation as (x,y,z,w)

    def fromTranslationRotation(self, translation, rotation):
        """
        :param translation: translation expressed as a tuple (x,y,z)
        :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
        :return: a :class:`numpy.matrix` 4x4 representation of the transform
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
        
        Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
        """

        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    ## Transforms a geometry_msgs PointStamped message to frame target_frame, returns the resulting PointStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PointStamped object

    def transformPoint(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.PointStamped message
        :return: new geometry_msgs.msg.PointStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PointStamped message to frame target_frame, returns a new PointStamped message.
        """

        mat44 = self.asMatrix(target_frame, ps.header)
        xyz = tuple(numpy.dot(mat44, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3]
        r = geometry_msgs.msg.PointStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.point = geometry_msgs.msg.Point(*xyz)
        return r

    ## Transforms a geometry_msgs Vector3Stamped message to frame target_frame, returns the resulting Vector3Stamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.Vector3Stamped object

    def transformVector3(self, target_frame, v3s):
        """
        :param target_frame: the tf target frame, a string
        :param v3s: the geometry_msgs.msg.Vector3Stamped message
        :return: new geometry_msgs.msg.Vector3Stamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs Vector3Stamped message to frame target_frame, returns a new Vector3Stamped message.
        """

        mat44 = self.asMatrix(target_frame, v3s.header)
        mat44[0,3] = 0.0
        mat44[1,3] = 0.0
        mat44[2,3] = 0.0
        xyz = tuple(numpy.dot(mat44, numpy.array([v3s.vector.x, v3s.vector.y, v3s.vector.z, 1.0])))[:3]
        r = geometry_msgs.msg.Vector3Stamped()
        r.header.stamp = v3s.header.stamp
        r.header.frame_id = target_frame
        r.vector = geometry_msgs.msg.Vector3(*xyz)
        return r

    ## Transforms a geometry_msgs QuaternionStamped message to frame target_frame, returns the resulting QuaternionStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.QuaternionStamped object

    def transformQuaternion(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.QuaternionStamped message
        :return: new geometry_msgs.msg.QuaternionStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs QuaternionStamped message to frame target_frame, returns a new QuaternionStamped message.
        """

        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given quat as a 4x4
        pose44 = xyzw_to_mat44(ps.quaternion)

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # quat is orientation of txpose
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value QuaternionStamped
        r = geometry_msgs.msg.QuaternionStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.quaternion = geometry_msgs.msg.Quaternion(*quat)
        return r

    ## Transforms a geometry_msgs PoseStamped message to frame target_frame, returns the resulting PoseStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PoseStamped object

    def transformPose(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.PoseStamped message
        :return: new geometry_msgs.msg.PoseStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given pose as a 4x4
        pose44 = numpy.dot(xyz_to_mat44(ps.pose.position), xyzw_to_mat44(ps.pose.orientation))

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # xyz and quat are txpose's position and orientation
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value PoseStamped
        r = geometry_msgs.msg.PoseStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))
        return r

    def transformPointCloud(self, target_frame, point_cloud):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the sensor_msgs.msg.PointCloud message
        :return: new sensor_msgs.msg.PointCloud message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
        r = sensor_msgs.msg.PointCloud()
        r.header.stamp = point_cloud.header.stamp
        r.header.frame_id = target_frame
        r.channels = point_cloud.channels

        mat44 = self.asMatrix(target_frame, point_cloud.header)
        def xf(p):
            xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
            return geometry_msgs.msg.Point(*xyz)
        r.points = [xf(p) for p in point_cloud.points]
        return r


class TransformListener(TransformerROS):

    """
    TransformListener is a subclass of :class:`tf.TransformerROS` that
    subscribes to the ``"/tf"`` message topic, and calls :meth:`tf.Transformer.setTransform`
    with each incoming transformation message.

    In this way a TransformListener object automatically
    stays up to to date with all current transforms.  Typical usage might be::

        import tf
        from geometry_msgs.msg import PointStamped

        class MyNode:

            def __init__(self):

                self.tl = tf.TransformListener()
                rospy.Subscriber("/sometopic", PointStamped, self.some_message_handler)
                ...
            
            def some_message_handler(self, point_stamped):

                # want to work on the point in the "world" frame
                point_in_world = self.tl.transformPoint("world", point_stamped)
                ...
        
    """
    def __init__(self, *args, **kwargs):
        TransformerROS.__init__(self, *args, **kwargs)
        self._listener = tf2_ros.TransformListener(self._buffer)
        self.setUsingDedicatedThread(True)
