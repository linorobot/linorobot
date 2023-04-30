tf_conversions
==============

.. toctree::
    :maxdepth: 2

PoseMath
--------

PoseMath is a utility that makes it easy to work with :class:`PyKDL.Frame`'s. 
It can work with poses from a variety of sources: :meth:`tf.Transformer.lookupTransform`, 
:mod:`opencv` and ROS messages.  It has utility functions to convert between these
types and the :class:`PyKDL.Frame` pose representation.

.. doctest::
    :options: -ELLIPSIS, +NORMALIZE_WHITESPACE

    >>> from geometry_msgs.msg import Pose
    >>> import tf_conversions.posemath as pm
    >>> import PyKDL
    >>>
    >>> msg = Pose()
    >>> msg.position.x = 7.0
    >>> msg.orientation.w = 1.0
    >>> 
    >>> frame = PyKDL.Frame(PyKDL.Rotation.RPY(2, 0, 1), PyKDL.Vector(1,2,4))
    >>>
    >>> res = pm.toTf(pm.fromMsg(msg) * frame)
    >>> print res
    ((8.0, 2.0, 4.0), (0.73846026260412856, 0.40342268011133486, 0.25903472399992566, 0.4741598817790379))



.. automodule:: tf_conversions.posemath
    :members: fromTf, fromMsg, toMsg, fromMatrix, toMatrix, fromCameraParams

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

