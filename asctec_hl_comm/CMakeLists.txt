cmake_minimum_required(VERSION 2.8.3)
project(asctec_hl_comm)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs geometry_msgs sensor_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

add_message_files(
  FILES
  DoubleArrayStamped.msg  
  GpsCustomCartesian.msg  
  GpsCustom.msg  
  mav_ctrl.msg  
  mav_ekf.msg  
  mav_imu.msg  
  mav_rcdata.msg  
  mav_state.msg  
  mav_status.msg  
  PositionWithCovarianceStamped.msg
  MotorSpeed.msg
)

add_service_files(
  FILES
  mav_ctrl_motors.srv
  MavCtrlSrv.srv
)

add_action_files(
  FILES
  Waypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs actionlib_msgs
)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES 
)
