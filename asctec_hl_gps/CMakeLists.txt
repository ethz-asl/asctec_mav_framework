cmake_minimum_required(VERSION 2.8.3)
project(asctec_hl_gps)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs asctec_hl_comm message_filters std_srvs)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

add_definitions(--std=c++11)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS roscpp sensor_msgs asctec_hl_comm message_filters std_srvs 
    INCLUDE_DIRS
    LIBRARIES # TODO
)

add_executable(gps_conversion_node src/gps_conversion.cpp src/gps_conversion_node.cpp)
add_dependencies(gps_conversion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_conversion_node ${catkin_LIBRARIES})

add_executable(set_gps_reference_node src/set_gps_reference_node.cpp)
add_dependencies(set_gps_reference_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_gps_reference_node ${catkin_LIBRARIES})


