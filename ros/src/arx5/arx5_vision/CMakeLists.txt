cmake_minimum_required(VERSION 3.0.2)
project(arx5_vision)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge roscpp rospy
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/arx5_vision_.py
  scripts/publish.py
  scripts/Best_Contour.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
