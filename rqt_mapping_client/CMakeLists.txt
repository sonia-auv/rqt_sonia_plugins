# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     25/07/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the GPLv3 license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)

project(rqt_mapping_client)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(SRC_DIR "src")

if(APPLE)
  set(CMAKE_MACOSX_RPATH 1)
endif()

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    rqt_gui
    rqt_gui_cpp
    image_transport
    sensor_msgs
    geometry_msgs
    cv_bridge
    sonia_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_package(
    INCLUDE_DIRS ${SRC_DIR}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge
    geometry_msgs sonia_msgs
)
catkin_python_setup()

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/*.cc" "${SRC_DIR}/*.cpp")
file(GLOB_RECURSE HDR_FILES "${SRC_DIR}/*.h")

list(REMOVE_ITEM SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/${PROJECT_NAME}/main.cc)

set(UIS_FILES
  resource/main_window.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(MOCS_FILES ${HDR_FILES})
  qt5_wrap_ui(UIS_H_FILES ${UIS_FILES})
else()
  qt4_wrap_cpp(MOCS_FILES ${HDR_FILES})
  qt4_wrap_ui(UIS_H_FILES ${UIS_FILES})
endif()

include_directories(
    ${SRC_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS})

#===============================================================================
# C R E A T E   E X E C U T A B L E

set(DEPENDENCIES ${catkin_LIBRARIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  set(DEPENDENCIES ${DEPENDENCIES} Qt5::Widgets)
else()
  set(DEPENDENCIES ${DEPENDENCIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

set(COMPILE_FILES ${SRC_FILES} ${MOCS_FILES} ${UIS_H_FILES})

add_library(${PROJECT_NAME} ${COMPILE_FILES})
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
add_dependencies(${PROJECT_NAME} sonia_msgs_generate_messages_cpp)


#===============================================================================
# I N S T A L L A T I O N

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_mapping_client
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_mapping_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#============================================================================
# U N I T   T E S T S

catkin_add_gtest(ros_communicator_test test/ros_communicator_test.cc
    ${COMPILE_FILES})
target_link_libraries(ros_communicator_test ${DEPENDENCIES})
