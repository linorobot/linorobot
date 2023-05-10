import rostest
import rospy
import unittest
import time

import tf.transformations
import geometry_msgs.msg
import sensor_msgs.msg

import tf


def process_time():
    try:
        # Available in Python >= 3.3 required in Python >= 3.8
        return time.process_time()
    except AttributeError:
        # Python < 3.3 compatibility
        return time.clock()


class Mock:
  pass

def setT(t, parent, frame, ti, x):
  m = Mock()
  m.parent_id = parent
  m.header = Mock()
  m.header.stamp = ti
  m.header.frame_id = frame
  m.transform = Mock()
  m.transform.translation = Mock()
  m.transform.translation.x = x
  m.transform.translation.y = 0
  m.transform.translation.z = 0
  m.transform.rotation = Mock()
  m.transform.rotation.x = 0
  m.transform.rotation.y = 0
  m.transform.rotation.z = 0
  m.transform.rotation.w = 1
  t.setTransform(m)

class TestPython(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        super(TestPython, cls).setUpClass()
        rospy.rostime.set_rostime_initialized(True)

    def setUp(self):
        pass

    def common(self, t):
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = "PARENT"
        m.child_frame_id = "THISFRAME"
        m.transform.translation.y = 5.0
        m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        t.setTransform(m)
        afs = t.allFramesAsString()
        self.assert_(len(afs) != 0)
        self.assert_("PARENT" in afs)
        self.assert_("THISFRAME" in afs)

        # Test getFrameStrings
        frames = t.getFrameStrings()
        self.assert_("THISFRAME" in frames)
        self.assert_("PARENT" not in frames)

        self.assert_(t.frameExists("THISFRAME"))
        self.assert_(not t.frameExists("PARENT"))


        self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == 0)
        for ti in [3, 5, 10, 11, 19, 20, 21]:
            m.header.stamp.secs = ti
            t.setTransform(m)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == ti)

        # Verify that getLatestCommonTime with nonexistent frames raise exception
        self.assertRaises(tf.Exception, lambda: t.getLatestCommonTime("MANDALAY", "JUPITER"))
        self.assertRaises(tf.LookupException, lambda: t.lookupTransform("MANDALAY", "JUPITER", rospy.Time()))

        # Ask for transform for valid frames, but more than 10 seconds in the past.  Should raise ExtrapolationException
        self.assertRaises(tf.ExtrapolationException, lambda: t.lookupTransform("THISFRAME", "PARENT", rospy.Time(2)))

        #### print t.lookupVelocity("THISFRAME", "PARENT", rospy.Time(15), rospy.Duration(5))

    def test_smoke(self):
        t = tf.Transformer()
        self.common(t)

    def test_chain(self):
        t = tf.Transformer()
        self.common(t)
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = "A"
        m.child_frame_id = "B"
        m.transform.translation.y = 5.0
        m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        t.setTransform(m)

        m.header.frame_id = "B"
        m.child_frame_id = "C"
        t.setTransform(m)

        m.header.frame_id = "B"
        m.child_frame_id = "C"
        t.setTransform(m)

        chain = t.chain("A", rospy.Time(0), "C", rospy.Time(0), "B")
        print("Chain is {}".format(chain))
        self.assert_("C" in chain)
        self.assert_("B" in chain)


    def test_wait_for_transform(self):

        def elapsed_time_within_epsilon(t, delta, epsilon):
            self.assertLess( t - epsilon,   delta)
            self.assertGreater( delta, t + epsilon)

        t = tf.Transformer()
        self.common(t)

        timeout = rospy.Duration(1)
        epsilon = 0.1

        # Check for dedicated thread exception, existing frames
        self.assertRaises(tf.Exception, lambda: t.waitForTransform("PARENT", "THISFRAME", rospy.Time(), timeout))
        # Check for dedicated thread exception, non-existing frames
        self.assertRaises(tf.Exception, lambda: t.waitForTransform("MANDALAY", "JUPITER", rospy.Time(), timeout))
        t.setUsingDedicatedThread(True)

        # This will no longer thorw
        self.assertEqual(t.waitForTransform("PARENT", "THISFRAME", rospy.Time(), timeout), None)
        self.assertEqual(t.waitForTransform("PARENT", "THISFRAME", rospy.Time(15), timeout), None)

        # Verify exception still thrown with unavailable time near timeout
        start = process_time()
        self.assertRaises(tf.Exception, lambda: t.waitForTransform("PARENT", "THISFRAME", rospy.Time(25), timeout))
        elapsed_time_within_epsilon(process_time() - start, timeout.to_sec(), epsilon)

        # Verify exception stil thrown with non-existing frames near timeout
        start = process_time()
        self.assertRaises(tf.Exception, lambda: t.waitForTransform("MANDALAY", "JUPITER", rospy.Time(), timeout))
        elapsed_time_within_epsilon(process_time() - start, timeout.to_sec(), epsilon)

    def test_cache_time(self):
        # Vary cache_time and confirm its effect on ExtrapolationException from lookupTransform().

        for cache_time in range(2, 98):
            t = tf.Transformer(True, rospy.Duration(cache_time))
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "PARENT"
            m.child_frame_id = "THISFRAME"
            m.transform.translation.y = 5.0
            m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
            t.setTransform(m)
            afs = t.allFramesAsString()
            self.assert_(len(afs) != 0)
            self.assert_("PARENT" in afs)
            self.assert_("THISFRAME" in afs)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == 0)

            # Set transforms for time 0..100 inclusive
            for ti in range(101):
                m.header.stamp = rospy.Time(ti)
                t.setTransform(m)
                self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == ti)
            self.assertEqual(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec(), 100)

            # (avoid time of 0 because that means 'latest')

            for ti in range(1, 100 - cache_time):
                self.assertRaises(tf.ExtrapolationException, lambda: t.lookupTransform("THISFRAME", "PARENT", rospy.Time(ti)))
            for ti in range(100 - cache_time, 100):
                t.lookupTransform("THISFRAME", "PARENT", rospy.Time(ti))

    def test_subclass(self):
        class TransformerSubclass(tf.Transformer):
            def extra(self):
              return 77
        t = TransformerSubclass(True, rospy.Duration.from_sec(10.0))
        self.assert_(t.extra() == 77)
        self.common(t)
        self.assert_(t.extra() == 77)

    def test_twist(self):
        t = tf.Transformer()

        vel = 3
        for ti in range(5):
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "PARENT"
            m.header.stamp = rospy.Time(ti)
            m.child_frame_id = "THISFRAME"
            m.transform.translation.x = ti * vel
            m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
            t.setTransform(m)

        tw0 = t.lookupTwist("THISFRAME", "PARENT", rospy.Time(0.0), rospy.Duration(4.001))
        self.assertAlmostEqual(tw0[0][0], vel, 2)
        tw1 = t.lookupTwistFull("THISFRAME", "PARENT", "PARENT", (0, 0, 0), "THISFRAME", rospy.Time(0.0), rospy.Duration(4.001))
        self.assertEqual(tw0, tw1)

    def test_transformer_ros(self):
        tr = tf.TransformerROS()

        m = geometry_msgs.msg.TransformStamped()
        m.header.stamp = rospy.Time().from_sec(3.0)
        m.header.frame_id = "PARENT"
        m.child_frame_id = "THISFRAME"
        m.transform.translation.y = 5.0
        m.transform.rotation.x = 0.04997917
        m.transform.rotation.y = 0
        m.transform.rotation.z = 0
        m.transform.rotation.w = 0.99875026
        tr.setTransform(m)
        m.header.stamp = rospy.Time().from_sec(5.0)
        tr.setTransform(m)

        # Smoke the various transform* methods

        types = [ "Point", "Pose", "Quaternion", "Vector3" ]
        for t in types:
            msg = getattr(geometry_msgs.msg, "{}Stamped".format(t))()
            msg.header.frame_id = "THISFRAME"
            msg_t = getattr(tr, "transform{}".format(t))("PARENT", msg)
            self.assertEqual(msg_t.header.frame_id, "PARENT")

        # PointCloud is a bit different, so smoke is different

        msg = sensor_msgs.msg.PointCloud()
        msg.header.frame_id = "THISFRAME"
        msg.points = [geometry_msgs.msg.Point32(1,2,3)]
        xmsg = tr.transformPointCloud("PARENT", msg)
        self.assertEqual(xmsg.header.frame_id, "PARENT")
        self.assertEqual(len(msg.points), len(xmsg.points))
        self.assertNotEqual(msg.points[0], xmsg.points[0])

        """
        Two fixed quaternions, a small twist around X concatenated.

        >>> t.quaternion_from_euler(0.1, 0, 0)
        array([ 0.04997917,  0.        ,  0.        ,  0.99875026])
        >>> t.quaternion_from_euler(0.2, 0, 0)
        array([ 0.09983342,  0.        ,  0.        ,  0.99500417])
        """

        # Specific test for quaternion types

        msg = geometry_msgs.msg.QuaternionStamped()
        q = [ 0.04997917,  0.        ,  0.        ,  0.99875026 ]
        msg.quaternion.x = q[0]
        msg.quaternion.y = q[1]
        msg.quaternion.z = q[2]
        msg.quaternion.w = q[3]
        msg.header.stamp = rospy.Time().from_sec(3.0)
        msg.header.frame_id = "THISFRAME"
        msg_t = tr.transformQuaternion("PARENT", msg)
        self.assertEqual(msg_t.header.frame_id, "PARENT")
        for a,v in zip("xyzw", [ 0.09983342,  0.        ,  0.        ,  0.99500417]):
            self.assertAlmostEqual(v,
                                   getattr(msg_t.quaternion, a),
                                   4)

    def test_transformer_wait_for_transform_dedicated_thread(self):
        tr = tf.Transformer()
        try:
          tr.waitForTransform("PARENT", "THISFRAME", rospy.Time().from_sec(4.0), rospy.Duration(3.0))
          self.assertFalse("This should throw")
        except tf.Exception as ex:
          print("successfully caught")
          pass

    def test_transformer_wait_for_transform(self):
        tr = tf.Transformer()
        tr.setUsingDedicatedThread(1)

        try:
          tr.waitForTransform("PARENT", "THISFRAME", rospy.Time().from_sec(4.0), rospy.Duration(3.0))
          self.assertFalse("This should throw")
        except tf.Exception as ex:
          pass

        m = geometry_msgs.msg.TransformStamped()
        m.header.stamp = rospy.Time().from_sec(3.0)
        m.header.frame_id = "PARENT"
        m.child_frame_id = "THISFRAME"
        m.transform.translation.y = 5.0
        m.transform.rotation.x = 0.04997917
        m.transform.rotation.y = 0
        m.transform.rotation.z = 0
        m.transform.rotation.w = 0.99875026
        tr.setTransform(m)
        m.header.stamp = rospy.Time().from_sec(5.0)
        tr.setTransform(m)

        try:
          tr.waitForTransform("PARENT", "THISFRAME", rospy.Time().from_sec(4.0), rospy.Duration(3.0))
        except tf.Exception as ex:
          self.assertFalse("This should not throw")


    def test_getTFPrefix(self):
        t = tf.Transformer()
        self.assertEqual(t.getTFPrefix(), "")

    def disabled_random(self):
        import networkx as nx
        for (r,h) in [ (2,2), (2,5), (3,5) ]:
            G = nx.balanced_tree(r, h)
            t = tf.Transformer(True, rospy.Duration(10.0))

            for n in G.nodes():
                if n != 0:
                    # n has parent p
                    p = min(G.neighbors(n))
                    setT(t, str(p), str(n), rospy.Time(0), 1)
            for n in G.nodes():
                ((x,_,_), _) = t.lookupTransform("0", str(n), rospy.Time(0))
                self.assert_(x == nx.shortest_path_length(G, 0, n))
            for i in G.nodes():
                for j in G.nodes():
                    ((x,_,_), _) = t.lookupTransform(str(i), str(j), rospy.Time())
                    self.assert_(abs(x) == abs(nx.shortest_path_length(G, 0, i) - nx.shortest_path_length(G, 0, j)))


if __name__ == '__main__':
    rostest.unitrun('tf', 'directed', TestPython)
