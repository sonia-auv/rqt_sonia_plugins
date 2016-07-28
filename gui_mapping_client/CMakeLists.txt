# \file	CMakeLists.txt
# \author	Thibaut Mattio <thibaut.mattio@gmail.com>
# \date	08/05/2015
#
# \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
#
# \section LICENSE
#
# This file is part of S.O.N.I.A. software.
#
# S.O.N.I.A. software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# S.O.N.I.A. software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)

project(gui_mapping_client)

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
    roscpp
    image_transport
    sensor_msgs
    cv_bridge
    std_msgs
    sonia_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_package(
    INCLUDE_DIRS ${SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS roscpp image_transport std_msgs sensor_msgs cv_bridge
    sonia_msgs
)
#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE SRC_FILES "${SRC_DIR}/*.cc" "${SRC_DIR}/*.cpp")
file(GLOB_RECURSE HDR_FILES "${SRC_DIR}/*.h")

list(REMOVE_ITEM SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_DIR}/${PROJECT_NAME}/main.cc)

set(UIS_FILES
    resource/main_window.ui
    resource/focused_frame.ui
    )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(MOCS_FILES ${HDR_FILES})
  qt5_wrap_ui(UIS_H_FILES ${UIS_FILES})
  qt5_add_resources(QRC_FILES resource/resources.rqc)
else()
  qt4_wrap_cpp(MOCS_FILES ${HDR_FILES})
  qt4_wrap_ui(UIS_H_FILES ${UIS_FILES})
  qt4_add_resources(QRC_FILES resource/resources.rqc)
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

set(COMPILE_FILES ${SRC_FILES} ${MOCS_FILES} ${UIS_H_FILES} ${QRC_FILES})

add_executable(${PROJECT_NAME}_node ${COMPILE_FILES})
target_link_libraries(${PROJECT_NAME}_node ${DEPENDENCIES})
add_dependencies(${PROJECT_NAME}_node sonia_msgs_generate_messages_cpp)
