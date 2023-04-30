#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'geometry_msgs', 'sensor_msgs', 'std_msgs'],
    scripts=['scripts/groovy_compatibility/tf_remap',
             'scripts/groovy_compatibility/view_frames']
)

setup(**d)
