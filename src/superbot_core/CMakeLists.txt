cmake_minimum_required(VERSION 3.16)
project(superbot_core LANGUAGES C CXX)

# Default standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# Tambahkan dependency ROS2 untuk hardware_interface, pluginlib, dll
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Build shared library untuk hardware interface
add_library(
  superbot_hardware
  SHARED
  hardware/superbot_system.cpp
)

target_compile_features(superbot_hardware PUBLIC cxx_std_17)

target_include_directories(superbot_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/superbot_hardware>
)

ament_target_dependencies(
  superbot_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Jika kamu pakai serial (misal komunikasi USB), tambahkan ini:
target_link_libraries(superbot_hardware PUBLIC serial)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface superbot_hardware.xml)

# Install header
install(
  DIRECTORY hardware/include/
  DESTINATION include/superbot_hardware
)

# Install assets penting
install(
  DIRECTORY config description launch maps ros2_control worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install shared lib
install(TARGETS superbot_hardware
  EXPORT export_superbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Optional testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Exports
ament_export_targets(export_superbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
