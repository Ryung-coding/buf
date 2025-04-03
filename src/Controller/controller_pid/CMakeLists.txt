cmake_minimum_required(VERSION 3.10)
project(controller_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(sbus_interfaces REQUIRED)
find_package(imu_interfaces REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(controller_worker_pid src/controller_worker.cpp)

# Link libraries
ament_target_dependencies(controller_worker_pid
  rclcpp
  rclcpp_components
  controller_interfaces
  sbus_interfaces
  imu_interfaces
  mocap_interfaces
  watchdog_interfaces
)

install(TARGETS
  controller_worker_pid
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_package()