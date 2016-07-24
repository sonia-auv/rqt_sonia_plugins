#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_dvl', 'rqt_dvl.ros_comm', 'rqt_dvl.widgets'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_dvl']
)

setup(**d)
