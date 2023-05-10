tf (Python)
===========

.. exception:: Exception

    base class for tf exceptions.  Because :exc:`tf.Exception` is the
    base class for other exceptions, you can catch all tf exceptions
    by writing::

        try:
            # do some tf work
        except tf.Exception:
            print "some tf exception happened"
        

.. exception:: ConnectivityException

   subclass of :exc:`Exception`.
   Raised when that the fixed_frame tree is not connected between the frames requested.

.. exception:: LookupException

   subclass of :exc:`Exception`.
   Raised when a tf method has attempted to access a frame, but
   the frame is not in the graph.
   The most common reason for this is that the frame is not
   being published, or a parent frame was not set correctly 
   causing the tree to be broken.  

.. exception:: ExtrapolationException

   subclass of :exc:`Exception`
   Raised when a tf method would have required extrapolation beyond current limits.


Transformer
-----------

.. class:: tf.Transformer(interpolating, cache_time = rospy.Duration(10))

   :param interpolating: Whether to interpolate transformations.
   :param cache_time: how long tf should retain transformation information in the past.

   The Transformer object is the core of tf. It maintains a
   time-varying graph of transforms, and permits asynchronous graph
   modification and queries:

   .. doctest::

       >>> import rospy
       >>> import tf
       >>> import geometry_msgs.msg
       >>> t = tf.Transformer(True, rospy.Duration(10.0))
       >>> t.getFrameStrings()
       []
       >>> m = geometry_msgs.msg.TransformStamped()
       >>> m.header.frame_id = 'THISFRAME'
       >>> m.child_frame_id = 'CHILD'
       >>> m.transform.translation.x = 2.71828183
       >>> m.transform.rotation.w = 1.0
       >>> t.setTransform(m)
       >>> t.getFrameStrings()
       ['/CHILD', '/THISFRAME']
       >>> t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0))
       ((2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
       >>> t.lookupTransform('CHILD', 'THISFRAME', rospy.Time(0))
       ((-2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


   The transformer refers to frames using strings, and represents
   transformations using translation (x, y, z) and quaternions (x, y,
   z, w) expressed as Python a :class:`tuple`.
   Transformer also does not mandate any particular
   linear algebra library.
   Transformer does not handle ROS messages directly; the only ROS type it
   uses is `rospy.Time() <http://www.ros.org/doc/api/rospy/html/rospy.rostime-module.html>`_.
   
   To use tf with ROS messages, see :class:`TransformerROS` and :class:`TransformListener`.

    .. method::  allFramesAsDot() -> string

        Returns a string representing all frames, intended for use with `Graphviz <http://www.graphviz.org>`_.

    .. method::  allFramesAsString() -> string

        Returns a human-readable string representing all frames

    .. method::  setTransform(transform, authority = "")

        :param transform: transform object, see below
        :param authority: string giving authority for this transformation.

        Adds a new transform to the Transformer graph. transform is an object with the following structure::

            header
                stamp             time stamp, rospy.Time
                frame_id          string, parent frame
            child_frame_id        string, child frame
            transform
                translation
                    x             float
                    y             float
                    z             float
                rotation
                    x             float
                    y             float
                    z             float
                    w             float

        These members exactly match those of a ROS TransformStamped message.

    .. method::  waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration = rospy.Duration(0.01))
        
        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :param time: time of the transformation, use ``rospy.Time()`` to indicate present time.
        :param timeout: how long this call should block while waiting for the transform, as a :class:`rospy.Duration`
        :raises: :exc:`tf.Exception`

        Waits for the given transformation to become available.
        If the timeout occurs before the transformation becomes available, raises :exc:`tf.Exception`.

    .. method::  waitForTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame)
        
        :param target_frame: transformation target frame in tf, string
        :param target_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param source_frame: transformation source frame in tf, string
        :param source_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param fixed_frame: reference frame common to both target_frame and source_frame.
        :param timeout: how long this call should block while waiting for the transform, as a :class:`rospy.Duration`
        :raises: :exc:`tf.Exception`

        Extended version of :meth:`waitForTransform`.

    .. method::  canTransform(target_frame, source_frame, time) -> bool

        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :param time: time of the transformation, use ``rospy.Time()`` to indicate present time.

        Returns True if the Transformer can determine the transform from source_frame to target_frame at time.

    .. method::  canTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame) -> bool

        Extended version of :meth:`canTransform`.

    .. method::  chain(target_frame, target_time, source_frame, source_time, fixed_frame) -> list

        :param target_frame: transformation target frame in tf, string
        :param target_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param source_frame: transformation source frame in tf, string
        :param source_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param fixed_frame: reference frame common to both target_frame and source_frame.
        :returns: list of tf frames
        :raises: :exc:`tf.ConnectivityException`, :exc:`tf.LookupException`

        returns the chain of frames connecting source_frame to target_frame.

    .. method::  clear() -> None

        Clear all transformations.

    .. method::  frameExists(frame_id) -> Bool

        :param frame_id: a tf frame, string

        returns True if frame frame_id exists in the Transformer.

    .. method::  getFrameStrings -> list

        returns all frame names in the Transformer as a list.

    .. method::  getLatestCommonTime(source_frame, target_frame) -> time

        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :returns: a :class:`rospy.Time` for the most recent time at which the transform is available
        :raises: :exc:`tf.Exception`

        Determines the most recent time for which Transformer can compute the transform between the two given frames. 
        Raises :exc:`tf.Exception` if transformation is not possible.

    .. method::  lookupTransform(target_frame, source_frame, time) -> (position, quaternion)

        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :param time: time of the transformation, use ``rospy.Time()`` to indicate most recent common time.
        :returns: position as a translation (x, y, z) and orientation as a quaternion (x, y, z, w)
        :raises: :exc:`tf.ConnectivityException`, :exc:`tf.LookupException`, or :exc:`tf.ExtrapolationException`

        Returns the transform from source_frame to target_frame at time. Raises one of the exceptions if the transformation is not possible.

        Note that a time of zero means latest common time, so::

            t.lookupTransform("a", "b", rospy.Time())

        is equivalent to::

            t.lookupTransform("a", "b", t.getLatestCommonTime("a", "b"))


    .. method::  lookupTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame) -> position, quaternion

        :param target_frame: transformation target frame in tf, string
        :param target_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param source_frame: transformation source frame in tf, string
        :param source_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param fixed_frame: reference frame common to both target_frame and source_frame.
        :raises: :exc:`tf.ConnectivityException`, :exc:`tf.LookupException`, or :exc:`tf.ExtrapolationException`

        Extended version of :meth:`lookupTransform`.

    .. method:: lookupTwist(tracking_frame, observation_frame, time, averaging_interval) -> linear, angular

        :param tracking_frame: The frame to track
        :type tracking_frame: str
        :param observation_frame: The frame from which to measure the twist
        :type observation_frame: str
        :param time: The time at which to get the velocity
        :type time: :class:`rospy.Time`
        :param duration: The period over which to average
        :type duration: :class:`rospy.Duration`
        :returns: a tuple with linear velocity as (x, y, z) and angular velocity as (x, y, z)
        :raises: :exc:`tf.ConnectivityException`, :exc:`tf.LookupException`, or :exc:`tf.ExtrapolationException`

        Simplified version of :meth:`tf.lookupTwistFull`.

        Return the linear and angular velocity of the moving_frame in the reference_frame.  tf considers 
        :math:`time - duration / 2` to :math:`time + duration / 2` as the initial interval, and will shift by
        up to :math:`duration / 2` to avoid no data.

        .. versionadded:: 1.1

    .. method:: lookupTwistFull(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame, time, averaging_interval) -> linear, angular

        :param tracking_frame: The frame to track
        :type tracking_frame: str
        :param observation_frame: The frame from which to measure the twist
        :type observation_frame: str
        :param reference_frame: The reference frame in which to express the twist
        :type reference_frame: str
        :param reference_point: The reference point with which to express the twist
        :type reference_point: x, y, z
        :param reference_point_frame: The frame_id in which the reference point is expressed
        :type reference_point_frame: str
        :param time: The time at which to get the velocity
        :type time: :class:`rospy.Time`
        :param duration: The period over which to average
        :type duration: :class:`rospy.Duration`
        :returns: a tuple with linear velocity as (x, y, z) and angular velocity as (x, y, z)
        :raises: :exc:`tf.ConnectivityException`, :exc:`tf.LookupException`, or :exc:`tf.ExtrapolationException`

        Return the linear and angular velocity of the moving_frame in the reference_frame.  tf considers 
        :math:`time - duration / 2` to :math:`time + duration / 2` as the initial interval, and will shift by
        up to :math:`duration / 2` to avoid no data.

        .. versionadded:: 1.1

    .. method:: getTFPrefix() -> str

        :returns: the TF Prefix that the transformer is running with
      
        Returns the tf_prefix this transformer is running with.

TransformerROS
--------------

.. autoclass:: tf.TransformerROS
    :members:

TransformListener
-----------------

.. autoclass:: tf.TransformListener

TransformBroadcaster
--------------------

.. autoclass:: tf.TransformBroadcaster
    :members:
