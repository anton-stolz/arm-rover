cmake_minimum_required(VERSION 3.8)
project(my_fast_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/point_to_pose.py
  scripts/bump_mapper.py
  scripts/sequential_handover.py
  scripts/amcl_global_initializer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
