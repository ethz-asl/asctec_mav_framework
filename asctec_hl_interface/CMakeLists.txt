cmake_minimum_required(VERSION 2.8.3)
project(asctec_hl_interface)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    sensor_msgs 
    tf 
    dynamic_reconfigure 
    asctec_hl_comm 
    actionlib 
    diagnostic_updater 
    sensor_fusion_comm
)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals system thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

generate_dynamic_reconfigure_options(cfg/HLInterface.cfg cfg/SSDK.cfg)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS roscpp sensor_msgs tf dynamic_reconfigure asctec_hl_comm actionlib diagnostic_updater sensor_fusion_comm
    INCLUDE_DIRS 
    LIBRARIES 
)

set(ASCTEC_HL_INTERFACE_SRC
    src/comm.cpp
    src/hl_interface.cpp
    src/ssdk_interface.cpp
    src/ekf_interface.cpp
    )

add_executable(hl_node ${ASCTEC_HL_INTERFACE_SRC} src/node.cpp)
add_dependencies(hl_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(hl_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ctrl_test src/ctrl_test.cpp)
add_dependencies(ctrl_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(ctrl_test ${catkin_LIBRARIES})

add_executable(waypoint_server src/waypoint_server.cpp)
add_dependencies(waypoint_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(waypoint_client src/waypoint_client.cpp)
add_dependencies(waypoint_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})


