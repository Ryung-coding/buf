cmake_minimum_required(VERSION 3.10)
project(watchdog_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(watchdog_worker src/watchdog_worker.cpp)

# Link libraries
ament_target_dependencies(watchdog_worker
  rclcpp
  rclcpp_components
  watchdog_interfaces
)

install(TARGETS
  watchdog_worker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_package()