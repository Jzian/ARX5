cmake_minimum_required(VERSION 3.1.3)
project(arx5_moveit)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/follow_joint_trajectory_server.py
  scripts/arx5_moveit_follow.py
  scripts/arx5_keyboard.py
  scripts/calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
