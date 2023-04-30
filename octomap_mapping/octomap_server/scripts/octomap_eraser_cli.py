#!/usr/bin/env python

"""Clear a region specified by a global axis-aligned bounding box in stored
OctoMap.

"""

import sys
from time import sleep

import roslib
roslib.load_manifest('octomap_server')
from geometry_msgs.msg import Point
import octomap_msgs.srv
import rospy


SRV_NAME = '/octomap_server/clear_bbx'
SRV_INTERFACE = octomap_msgs.srv.BoundingBoxQuery


if __name__ == '__main__':
    min = Point(*[float(x) for x in sys.argv[1:4]])
    max = Point(*[float(x) for x in sys.argv[4:7]])

    rospy.init_node('octomap_eraser_cli', anonymous=True)
    sleep(1)
    service = rospy.ServiceProxy(SRV_NAME, SRV_INTERFACE)
    rospy.loginfo("Connected to %s service." % SRV_NAME)
    
    service(min, max)
