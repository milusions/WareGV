cmake_minimum_required(VERSION 3.8)
project(waregv_hardware)

# Set C++ standard to C++17 for ros2_control compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ros2_control & C++ Dependencies
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
include_directories(include)

# -----------------------------------------------------------------------------
# Build Shared C++ Hardware Interface Library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/sts3215_driver.cpp
  src/sts3215_servo_control_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export pluginlib XML so ros2_control discovery works
pluginlib_export_plugin_description_file(
  hardware_interface
  waregv_hardware.xml
)

# -----------------------------------------------------------------------------
# Python Package & Executables Setup
# -----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS 
  waregv_hardware/imu_chasis.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Install C++ Libraries, Headers, Launch & Directories
# -----------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES waregv_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch waregv_hardware 
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------------------------------------------------------
# Export Configuration
# -----------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()