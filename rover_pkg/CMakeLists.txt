cmake_minimum_required(VERSION 3.0.2)
project(rover_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

# Install Python scripts so they are executable via rosrun
catkin_install_python(PROGRAMS
  scripts/health_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install firmware folder (for reference/uploading)
install(DIRECTORY firmware/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/firmware
)
