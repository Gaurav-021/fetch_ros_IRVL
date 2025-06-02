cmake_minimum_required(VERSION 3.8)
project(fetch_description)

# Required for install and package registration
find_package(ament_cmake REQUIRED)

# Install all resources: urdf, meshes, rviz, etc.
install(DIRECTORY
  urdf
  meshes
  robots
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()