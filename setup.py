#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rospy_websocker_client'],
    package_dir={'' : 'src'},
)

setup(**d)
