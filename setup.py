#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_mapping_client', 'rqt_mapping_client.ros_comm', 'rqt_mapping_client.widgets'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_mapping_client']
)
