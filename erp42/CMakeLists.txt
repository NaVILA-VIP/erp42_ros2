cmake_minimum_required(VERSION 3.8)
project(erp42)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(erp42_odometry SHARED src/odometry.cpp)
target_compile_features(erp42_odometry PUBLIC cxx_std_17)
ament_target_dependencies(erp42_odometry gazebo_dev gazebo_ros geometry_msgs nav_msgs rclcpp tf2_ros)
install(TARGETS erp42_odometry LIBRARY DESTINATION lib)
install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
