cmake_minimum_required(VERSION 2.8.3)
project(rqt_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  cv_bridge
  std_msgs
)
catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/rqt_vision
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
