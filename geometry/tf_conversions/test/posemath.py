import rostest
import rospy
import numpy
import unittest
import sys

from tf import Transformer
import tf_conversions.posemath as pm

from geometry_msgs.msg import TransformStamped
from PyKDL import Frame

class TestPoseMath(unittest.TestCase):

    def setUp(self):
        pass

    def test_fromTf(self):
        transformer = Transformer(True, rospy.Duration(10.0))
        m = TransformStamped()
        m.header.frame_id = 'wim'
        m.child_frame_id = 'james'
        m.transform.translation.x = 2.71828183
        m.transform.rotation.w = 1.0
        transformer.setTransform(m)
        b = pm.fromTf(transformer.lookupTransform('wim', 'james', rospy.Time(0)))
        
    def test_roundtrip(self):
        c = Frame()

        d = pm.fromMsg(pm.toMsg(c))
        self.assertEqual(repr(c), repr(d))

        d = pm.fromMatrix(pm.toMatrix(c))
        self.assertEqual(repr(c), repr(d))

        d = pm.fromTf(pm.toTf(c))
        self.assertEqual(repr(c), repr(d))

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1].startswith('--gtest_output'):
        rostest.unitrun('tf', 'directed', TestPoseMath)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestPoseMath(sys.argv[1]))
        unittest.TextTestRunner(verbosity=2).run(suite)
