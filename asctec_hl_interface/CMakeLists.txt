cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(ASCTEC_HL_INTERFACE_SRC
    src/comm.cpp
    src/hl_interface.cpp
    src/ssdk_interface.cpp
    src/ekf_interface.cpp
    )

rosbuild_add_boost_directories()

rosbuild_add_executable(hl_node ${ASCTEC_HL_INTERFACE_SRC} src/node.cpp)
rosbuild_link_boost(hl_node thread system signals)

rosbuild_add_executable(ctrl_test src/ctrl_test.cpp)
rosbuild_add_executable(waypoint_server src/waypoint_server.cpp)
rosbuild_add_executable(waypoint_client src/waypoint_client.cpp)

rosbuild_add_executable(command_converter src/body_command_converter.cpp)
