# \file	CMakeLists.txt
# \author	Alexi Demers <alexidemers@gmail.com>
# \date	16/06/2015
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

cmake_minimum_required( VERSION 2.8.3 )
project( can_client )

IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall " )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s" )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

set(can_client_SRC_DIR "src")
set(can_client_UIS_DIR "ui")

#===============================================================================
# C A T K I N   P A C K A G E S

find_package( catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    message_generation
    image_transport
    cv_bridge
    sonia_msgs
)

find_package( catkin REQUIRED provider_can )

file(GLOB QWT_SEARCH_PATHS "/opt/qwt-*" "/usr/include/qwt-*")
find_path(QWT_INCLUDE_DIR
        NAMES qwt.h
        NAMES qwt_global.h
        NAMES qwt_plot.h
        PATHS ${QWT_SEARCH_PATHS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include( ${QT_USE_FILE} )

catkin_package(
    INCLUDE_DIRS ${can_client_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    cv_bridge
    provider_can
    sonia_msgs
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE can_client_SRC_FILES "${can_client_SRC_DIR}/*.cpp" )
file(GLOB_RECURSE can_client_HDR_FILES "${can_client_SRC_DIR}/*.h")
file(GLOB_RECURSE can_client_UIS "${can_client_UIS_DIR}/*.ui" )



include_directories(
    ${can_client_SRC_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIR}
)
link_directories(${QWT_INCLUDE_DIR}/lib)

qt4_wrap_cpp( can_client_QT_MOC ${can_client_HDR_FILES} )
qt4_wrap_ui( can_client_QT_UIS ${can_client_UIS} )

#===============================================================================
# C R E A T E   E X E C U T A B L E

add_executable( ${PROJECT_NAME}_node
    ${can_client_SRC_FILES}
    ${can_client_HDR_FILES}
    ${can_client_QT_MOC}
    ${can_client_QT_UIS}
)

target_link_libraries( ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${QT4_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
  qwt
  )

add_dependencies(${PROJECT_NAME}_node  sonia_msgs_generate_messages_cpp)

