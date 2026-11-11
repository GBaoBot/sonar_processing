cmake_minimum_required(VERSION 3.8)
project(sonar_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Find ROS 2 Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)

set(PACKAGE_DEPENDENCIES
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "sensor_msgs"
)

foreach(PKGDEP IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKGDEP} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# =============================================================================
# Component Library
# =============================================================================
add_library(${PROJECT_NAME}_component SHARED
  src/ca_cfar.cpp
  src/os_cfar.cpp
  src/gaussian_filter.cpp
  src/sonar_processing_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_component ${PACKAGE_DEPENDENCIES})

# =============================================================================
# Register Component
# =============================================================================
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "sonar_processing::SonarProcessingNode"
  EXECUTABLE sonar_processing_node
)

# =============================================================================
# Installation
# =============================================================================
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =============================================================================
# Export
# =============================================================================
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
