
cmake_minimum_required(VERSION 3.5)

project(clober_rmf)

# find_package(ament_cmake REQUIRED)

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)

  install(
    DIRECTORY launch/ maps/
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    PROGRAMS 
      scripts/test.py
      scripts/send_negotiation_notice.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # install(DIRECTORY
  #   launch/
  #   maps/
  #   scripts/
  #   DESTINATION share/${PROJECT_NAME}
  # )

  ament_package()
else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
