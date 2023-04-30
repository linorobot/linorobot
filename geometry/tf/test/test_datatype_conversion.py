#!/usr/bin/env python

import sys
import unittest
import tf
import geometry_msgs.msg
import rospy

## A sample python unit test
class PoseConversions(unittest.TestCase):
    def setUp(self):
        #Setup the pose tests
        self.tfpose_stamped = tf.PoseStamped()
        self.tfpose_stamped.pose.setIdentity()
        self.tfpose_stamped.frame_id = "frame1"
        self.tfpose_stamped.stamp = roslib.rostime.Time(10,0)

        self.msgpose_stamped = geometry_msgs.msg.PoseStamped()
        self.msgpose_stamped.pose.position.x = 0
        self.msgpose_stamped.pose.position.y = 0
        self.msgpose_stamped.pose.position.z = 0
        self.msgpose_stamped.pose.orientation.x = 0
        self.msgpose_stamped.pose.orientation.y = 0
        self.msgpose_stamped.pose.orientation.z = 0
        self.msgpose_stamped.pose.orientation.w = 1
        self.msgpose_stamped.header.frame_id = "frame1"
        self.msgpose_stamped.header.stamp = roslib.rostime.Time(10,0)

        ## Setup the point tests
        self.tfpoint_stamped = tf.PointStamped()
        self.tfpoint_stamped.point.x = 0
        self.tfpoint_stamped.point.y = 0
        self.tfpoint_stamped.point.z = 0
        self.tfpoint_stamped.frame_id = "frame1"
        self.tfpoint_stamped.stamp = roslib.rostime.Time(10,0)

        self.msgpoint_stamped = geometry_msgs.msg.PointStamped()
        self.msgpoint_stamped.point.x = 0
        self.msgpoint_stamped.point.y = 0
        self.msgpoint_stamped.point.z = 0
        self.msgpoint_stamped.header.frame_id = "frame1"
        self.msgpoint_stamped.header.stamp = roslib.rostime.Time(10,0)

        ## Setup the vector tests
        self.tfvector_stamped = tf.VectorStamped()
        self.tfvector_stamped.vector.x = 0
        self.tfvector_stamped.vector.y = 0
        self.tfvector_stamped.vector.z = 0
        self.tfvector_stamped.frame_id = "frame1"
        self.tfvector_stamped.stamp = roslib.rostime.Time(10, 0)

        self.msgvector_stamped = geometry_msgs.msg.Vector3Stamped()
        self.msgvector_stamped.vector.x = 0
        self.msgvector_stamped.vector.y = 0
        self.msgvector_stamped.vector.z = 0
        self.msgvector_stamped.header.frame_id = "frame1"
        self.msgvector_stamped.header.stamp = roslib.rostime.Time(10,0)

        ## Setup the quaternion tests
        self.tfquaternion_stamped = tf.QuaternionStamped()
        self.tfquaternion_stamped.quaternion.x = 0
        self.tfquaternion_stamped.quaternion.y = 0
        self.tfquaternion_stamped.quaternion.z = 0
        self.tfquaternion_stamped.quaternion.w = 1
        self.tfquaternion_stamped.frame_id = "frame1"
        self.tfquaternion_stamped.stamp = roslib.rostime.Time(10, 0)

        self.msgquaternion_stamped = geometry_msgs.msg.QuaternionStamped()
        self.msgquaternion_stamped.quaternion.x = 0
        self.msgquaternion_stamped.quaternion.y = 0
        self.msgquaternion_stamped.quaternion.z = 0
        self.msgquaternion_stamped.quaternion.w = 1
        self.msgquaternion_stamped.header.frame_id = "frame1"
        self.msgquaternion_stamped.header.stamp = roslib.rostime.Time(10,0)
        
    # Test Pose conversions
    def test_msg_operator_equals_pose(self):
        self.assertEquals(self.msgpose_stamped, self.msgpose_stamped, "pose msg test correctness")

    def test_bt_operator_equals_pose(self):
        self.assertEquals(self.tfpose_stamped, self.tfpose_stamped, "pose bt test correctness")

    def test_msg_operator_on_converted(self):
        self.assertEquals(tf.pose_stamped_bt_to_msg(self.tfpose_stamped), tf.pose_stamped_bt_to_msg(self.tfpose_stamped), "pose msg test correctness after conversion")

    def test_bt_operator_on_converted(self):
        self.assertEquals(tf.pose_stamped_msg_to_bt(self.msgpose_stamped), tf.pose_stamped_msg_to_bt(self.msgpose_stamped), "pose bt test correctness after conversion")

    def test_to_msg_pose(self):
        self.assertEquals(tf.pose_bt_to_msg(self.tfpose_stamped.pose), self.msgpose_stamped.pose, "pose tf to msg incorrect")
    def test_to_tf_pose(self):
        self.assertEquals(tf.pose_msg_to_bt(self.msgpose_stamped.pose), self.tfpose_stamped.pose, "pose stamped msg to tf incorrect")

    def test_stamped_to_msg_pose(self):
        self.assertEquals(tf.pose_stamped_bt_to_msg(self.tfpose_stamped), self.msgpose_stamped, "pose stamped tf to msg incorrect")
    def test_stamped_to_tf_pose(self):
        self.assertEquals(tf.pose_stamped_msg_to_bt(self.msgpose_stamped), self.tfpose_stamped, "pose stamped msg to tf incorrect")
        

        
    # Test Point conversions
    def test_bt_operator_equal_point(self):
        self.assertEquals(self.tfpoint_stamped, self.tfpoint_stamped, "point tf test correctness")

    def test_msg_operator_equal_point(self):
        self.assertEquals(self.msgpoint_stamped, self.msgpoint_stamped, "point msg test correctness")

    def test_msg_operator_equal_point_converted(self):
        self.assertEquals(tf.point_stamped_bt_to_msg(self.tfpoint_stamped), tf.point_stamped_bt_to_msg(self.tfpoint_stamped), "point msg test correctness after conversion")

    def test_bt_operator_equal_point_converted(self):
        self.assertEquals(tf.point_stamped_msg_to_bt(self.msgpoint_stamped), tf.point_stamped_msg_to_bt(self.msgpoint_stamped), "point bt test correctness after conversion")

    def test_to_msg_point(self):
        self.assertEquals(tf.point_bt_to_msg(self.tfpoint_stamped.point), self.msgpoint_stamped.point, "point tf to msg incorrect")
    def test_to_tf_point(self):
        self.assertEquals(tf.point_msg_to_bt(self.msgpoint_stamped.point), self.tfpoint_stamped.point, "point stamped msg to tf incorrect")

    def test_stamped_to_msg_point(self):
        self.assertEquals(tf.point_stamped_bt_to_msg(self.tfpoint_stamped), self.msgpoint_stamped, "point stamped tf to msg incorrect")
    def test_stamped_to_tf_point(self):
        self.assertEquals(tf.point_stamped_msg_to_bt(self.msgpoint_stamped), self.tfpoint_stamped, "point stamped msg to tf incorrect")
        
    # Test Vector conversions
    def test_msg_operator_equal_vector(self):
        self.assertEquals(self.tfvector_stamped, self.tfvector_stamped, "vector bt test correctness")

    def test_msg_operator_equal_vector(self):
        self.assertEquals(self.msgvector_stamped, self.msgvector_stamped, "vector msg test correctness")

    def test_msg_operator_equal_vector_converted(self):
        self.assertEquals(tf.vector_stamped_bt_to_msg(self.tfvector_stamped), tf.vector_stamped_bt_to_msg(self.tfvector_stamped), "vector msg test correctness after conversion")

    def test_bt_operator_equal_vector_converted(self):
        self.assertEquals(tf.vector_stamped_msg_to_bt(self.msgvector_stamped), tf.vector_stamped_msg_to_bt(self.msgvector_stamped), "vector bt test correctness after conversion")

    def test_to_msg_vector(self):
        self.assertEquals(tf.vector_bt_to_msg(self.tfvector_stamped.vector), self.msgvector_stamped.vector, "vector tf to msg incorrect")
    def test_to_tf_vector(self):
        self.assertEquals(tf.vector_msg_to_bt(self.msgvector_stamped.vector), self.tfvector_stamped.vector, "vector stamped msg to tf incorrect")

    def test_stamped_to_msg_vector(self):
        self.assertEquals(tf.vector_stamped_bt_to_msg(self.tfvector_stamped), self.msgvector_stamped, "vector stamped tf to msg incorrect")
    def test_stamped_to_tf_vector(self):
        self.assertEquals(tf.vector_stamped_msg_to_bt(self.msgvector_stamped), self.tfvector_stamped, "vector stamped msg to tf incorrect")

    # Test Quaternion conversions
    def test_bt_operator_equal_quaternion(self):
        self.assertEquals(self.tfquaternion_stamped, self.tfquaternion_stamped, "quaternion bt test correctness")

    def test_msg_operator_equal_quaternion(self):
        self.assertEquals(self.msgquaternion_stamped, self.msgquaternion_stamped, "quaternion msg test correctness")

    def test_msg_operator_equal_quaternion_converted(self):
        self.assertEquals(tf.quaternion_stamped_bt_to_msg(self.tfquaternion_stamped), tf.quaternion_stamped_bt_to_msg(self.tfquaternion_stamped), "quaternion msg test correctness after conversion")

    def test_bt_operator_equal_quaternion_converted(self):
        self.assertEquals(tf.quaternion_stamped_msg_to_bt(self.msgquaternion_stamped), tf.quaternion_stamped_msg_to_bt(self.msgquaternion_stamped), "quaternion bt test correctness after conversion")

    def test_to_msg_quaternion(self):
        self.assertEquals(tf.quaternion_bt_to_msg(self.tfquaternion_stamped.quaternion), self.msgquaternion_stamped.quaternion, "quaternion tf to msg incorrect")
    def test_to_tf_quaternion(self):
        self.assertEquals(tf.quaternion_msg_to_bt(self.msgquaternion_stamped.quaternion), self.tfquaternion_stamped.quaternion, "quaternion stamped msg to tf incorrect")

    def test_stamped_to_msg_quaternion(self):
        self.assertEquals(tf.quaternion_stamped_bt_to_msg(self.tfquaternion_stamped), self.msgquaternion_stamped, "quaternion stamped tf to msg incorrect")
    def test_stamped_to_tf_quaternion(self):
        self.assertEquals(tf.quaternion_stamped_msg_to_bt(self.msgquaternion_stamped), self.tfquaternion_stamped, "quaternion stamped msg to tf incorrect")
if __name__ == '__main__':
    import rostest
    rostest.unitrun('tf', 'test_tf_data_conversions', PoseConversions)
                            
