cmake_minimum_required(VERSION 3.16)
project(superbot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs  # ✅ Tambahkan ini
)

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Find other dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  superbot_hardware
  SHARED
  hardware/superbot_system.cpp
)

# Set C++17 standard
target_compile_features(superbot_hardware PUBLIC cxx_std_17)

# Include directories
target_include_directories(superbot_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/superbot_hardware>
)

ament_target_dependencies(
  superbot_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link dependencies
target_link_libraries(superbot_hardware PUBLIC serial)

# Define visibility macros (for DLLs)
target_compile_definitions(${PROJECT_NAME} PRIVATE "superbot_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface superbot_hardware.xml)

# INSTALL
# Install header files
install(
  DIRECTORY hardware/include/
  DESTINATION include/superbot_hardware
)

# Install other directories directly (without description and bringup)
install(
  DIRECTORY config launch rviz urdf ros2_control
  DESTINATION share/superbot_hardware
)

# Install the library
install(TARGETS superbot_hardware
  EXPORT export_superbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_superbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
