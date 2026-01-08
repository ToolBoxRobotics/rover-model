cmake_minimum_required(VERSION 3.0.2)
project(rover_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()

# Install URDF folder
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
