cmake_minimum_required(VERSION 3.10)
project(dynamixel_worker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(dynamixel_worker src/dynamixel_worker.cpp)

# Link libraries
ament_target_dependencies(dynamixel_worker
  dynamixel_sdk
  rclcpp
  rclcpp_components
  dynamixel_interfaces
  watchdog_interfaces
)

install(TARGETS
  dynamixel_worker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_package()