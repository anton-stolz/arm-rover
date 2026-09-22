cmake_minimum_required(VERSION 3.8)
project(my_fast_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(imu_component SHARED src/imu_component.cpp)
target_include_directories(imu_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(imu_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_component rclcpp rclcpp_components sensor_msgs nav_msgs)
rclcpp_components_register_node(imu_component PLUGIN "my_fast_controller::ImuNode" EXECUTABLE imu_node)

add_library(bumper_component SHARED src/bumper_component.cpp)
target_include_directories(bumper_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(bumper_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(bumper_component rclcpp rclcpp_components geometry_msgs std_msgs)
rclcpp_components_register_node(bumper_component PLUGIN "my_fast_controller::BumperNode" EXECUTABLE bumper_node)

add_library(diff_drive_component SHARED src/diff_drive_component.cpp)
target_include_directories(diff_drive_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(diff_drive_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(diff_drive_component rclcpp rclcpp_components geometry_msgs nav_msgs std_msgs tf2 tf2_ros)
rclcpp_components_register_node(diff_drive_component PLUGIN "my_fast_controller::DiffDriveNode" EXECUTABLE diff_drive_node)

add_library(bump_mapper_component SHARED src/bump_mapper_component.cpp)
target_include_directories(bump_mapper_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(bump_mapper_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(bump_mapper_component rclcpp rclcpp_components sensor_msgs std_msgs tf2 tf2_ros)
rclcpp_components_register_node(bump_mapper_component PLUGIN "my_fast_controller::BumpMapperNode" EXECUTABLE bump_mapper_node)

add_library(point_to_pose_component SHARED src/point_to_pose_component.cpp)
target_include_directories(point_to_pose_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(point_to_pose_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(point_to_pose_component rclcpp rclcpp_components rclcpp_action geometry_msgs nav2_msgs tf2 tf2_ros)
rclcpp_components_register_node(point_to_pose_component PLUGIN "my_fast_controller::PointToPoseNode" EXECUTABLE point_to_pose_node)

add_library(pose_preserver_component SHARED src/pose_preserver_component.cpp)
target_include_directories(pose_preserver_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(pose_preserver_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pose_preserver_component rclcpp rclcpp_components tf2 tf2_ros)
rclcpp_components_register_node(pose_preserver_component PLUGIN "my_fast_controller::PosePreserverNode" EXECUTABLE pose_preserver_node)

ament_package()

install(TARGETS
  imu_component
  bumper_component
  diff_drive_component
  bump_mapper_component
  point_to_pose_component
  pose_preserver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
