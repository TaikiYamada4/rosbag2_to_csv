cmake_minimum_required(VERSION 3.14)
project(rosbag2_to_csv)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(rosbag2_cpp REQUIRED)
#find_package(rosbag2_storage REQUIRED)

# Define include directories
include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME}_base SHARED
  src/rosbag2_to_csv_base.cpp
  src/type_specific/pose_with_covariance_stamped_msg_to_csv.cpp
  src/type_specific/twist_with_covariance_stamped_msg_to_csv.cpp
  src/type_specific/imu_msg_to_csv.cpp
  src/type_specific/odometry_msg_to_csv.cpp
  src/type_specific/float32_stamped_msg_to_csv.cpp
)

ament_auto_add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_base
)

ament_auto_package()