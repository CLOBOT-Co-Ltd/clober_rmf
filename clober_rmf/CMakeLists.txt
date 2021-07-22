
cmake_minimum_required(VERSION 3.5)

project(clober_rmf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()