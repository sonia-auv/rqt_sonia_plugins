# \file	CMakeLists.txt
# \author	Thibaut Mattio <thibaut.mattio@gmail.com>
# \date	08/05/2015
# \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>

#===============================================================================
# C M A K E   C O N F I G U R A T I O N

cmake_minimum_required( VERSION 2.8.3 )

project( vision_client )

IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3" )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#===============================================================================
# C A T K I N   P A C K A G E S

find_package( catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    message_generation
    image_transport
    cv_bridge
)
find_package( catkin REQUIRED vision_server )

catkin_package( CATKIN_DEPENDS
    std_msgs
    vision_server
    image_transport
)

#===============================================================================
# OTHER PACKAGES

find_package( Qt4 REQUIRED COMPONENTS
    QtCore
    QtGui
)
include( ${QT_USE_FILE} )

#===============================================================================
# I N C L U D E   F I L E S

set( LOCAL_SRC_DIR "src")
set( LOCAL_UI_DIR "ui")

file( GLOB_RECURSE HDR "${LOCAL_SRC_DIR}/*.h" )
file( GLOB_RECURSE SRC "${LOCAL_SRC_DIR}/*.cpp" )
file( GLOB_RECURSE UI "${LOCAL_UI_DIR}/*.ui" )

set( INCLUDE_DIRS
    ${LOCAL_SRC_DIR}
    "${LOCAL_SRC_DIR}/widgets"
    "${LOCAL_SRC_DIR}/gui"
    "${LOCAL_SRC_DIR}/ros"
)

include_directories(
    ${INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

# Generating MOC and headers for UI files.
# For some reason, commands are case sensitive, so just keep it like it is...
QT4_WRAP_CPP( HDR_MOC ${HDR} )
QT4_WRAP_UI( UI_HDR ${UI} )

add_executable( vision_client_node
    ${SRC}
    ${HDR}
    ${HDR_MOC}
    ${UI_HDR}
)

target_link_libraries( vision_client_node
  ${catkin_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${QT4_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)
