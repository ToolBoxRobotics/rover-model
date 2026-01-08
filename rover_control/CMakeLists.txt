cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  # moveit_commander is a python lib, often not found via cmake, but good to list if available
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/kinematics.py
  scripts/arm_bridge.py
  scripts/autonomous_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch, config, and webui folders
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY webui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webui
)
