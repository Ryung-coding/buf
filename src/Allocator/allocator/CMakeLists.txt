cmake_minimum_required(VERSION 3.10)
project(allocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(allocator_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories for allocator_worker target
include_directories(include)

# Add executables
add_executable(allocator_worker src/allocator_worker.cpp)

# Include Eigen directories
target_include_directories(allocator_worker PRIVATE ${EIGEN3_INCLUDE_DIR})

# Link libraries
ament_target_dependencies(allocator_worker
  rclcpp
  rclcpp_components
  controller_interfaces
  allocator_interfaces
  watchdog_interfaces
)

install(TARGETS
  allocator_worker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_package()