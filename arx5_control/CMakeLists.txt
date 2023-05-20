cmake_minimum_required(VERSION 3.0.2)
project(arx5_control)

add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy
  sensor_msgs
  arx5_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  arx5_grasp.srv
  det.srv
  MarkerDetResult.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sensor_msgs arx5_msgs
)

catkin_install_python(PROGRAMS
  scripts/arx_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(actuator_control
   SHARED
   src/ros_interface/ros_interface.cpp
   src/libcan/CANAdapter.cpp
   src/libcan/CANFrameParser.cpp
   src/libcan/SocketCAN.cpp
)

AUX_SOURCE_DIRECTORY(src/libcan ACTUATOR_LIBCAN_SOURCES)
AUX_SOURCE_DIRECTORY(src/ros_interface ACTUATOR_ROS_SOURCES)
AUX_SOURCE_DIRECTORY(src/hardware ACTUATOR_HARDWARE_SOURCES)
AUX_SOURCE_DIRECTORY(src ACTUATOR_SOURCES)

add_executable(arx5_control ${ACTUATOR_LIBCAN_SOURCES} ${ACTUATOR_ROS_SOURCES} ${ACTUATOR_HARDWARE_SOURCES} ${ACTUATOR_SOURCES})
add_dependencies(arx5_control arx5_msgs_generate_messages_cpp)

target_link_libraries(arx5_control ${catkin_LIBRARIES})
