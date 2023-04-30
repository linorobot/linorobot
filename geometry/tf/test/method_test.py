from __future__ import print_function

import rospy
import tf
import time
import bullet
import math


try:
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
    print("setting rotation to PI/2\n After setRotation")
    print(transform_stamped.transform)

    other_transform = bullet.Transform()
    other_transform.setIdentity()
    transform_stamped.transform = other_transform
    print("After assignment of Identity")
    print(transform_stamped.transform)

    other_transform = bullet.Transform()
    other_transform.setIdentity()
    other_transform.setRotation(quat)
    transform_stamped.transform = other_transform
    print("After assignment of Rotation Transformation")
    print(transform_stamped.transform)
    
except ValueError as e:
    print("Exception {} Improperly thrown: {}".format(type(e), e))
