cmake_minimum_required(VERSION 3.8)
project(slider_state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python build
find_package(rclpy REQUIRED)


install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/imu_pub.py 
  scripts/body_pose_estimator.py
  DESTINATION lib/${PROJECT_NAME} 
)
ament_package()