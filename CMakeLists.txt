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
  "pluginlib"
  "tf2_ros"
)

foreach(PKGDEP IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKGDEP} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# =============================================================================
# Core: the sonar maths, plain OpenCV (no rclcpp, no pluginlib). Link this
# directly to unit-test or reuse an algorithm.
# =============================================================================
add_library(${PROJECT_NAME}_core SHARED
  src/core/gaussian_filter.cpp
  src/core/cfar/os_cfar.cpp
  src/core/cfar/ca_cfar.cpp
  src/core/morphology_filter.cpp
  src/core/persistence_filter.cpp
  src/core/scan_builder.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_core sensor_msgs)   # ScanBuilder fills a LaserScan

# =============================================================================
# Stage plugins: thin wrappers (params -> algorithm). Loaded only through pluginlib,
# nothing links this library.
# =============================================================================
add_library(${PROJECT_NAME}_stage_plugins SHARED
  src/stage_plugins/gaussian.cpp
  src/stage_plugins/cfar/os_cfar.cpp
  src/stage_plugins/cfar/ca_cfar.cpp
  src/stage_plugins/morphology.cpp
  src/stage_plugins/persistence.cpp
  src/stage_plugins/to_laserscan.cpp
)
target_link_libraries(${PROJECT_NAME}_stage_plugins ${PROJECT_NAME}_core ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_stage_plugins
  rclcpp rclcpp_lifecycle sensor_msgs pluginlib tf2_ros)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin_description.xml)

# =============================================================================
# Component Library
# =============================================================================
add_library(${PROJECT_NAME}_component SHARED
  src/sonar_processing_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_component ${PACKAGE_DEPENDENCIES})

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "sonar_processing::SonarProcessingNode"
  EXECUTABLE sonar_processing_node
)

# =============================================================================
# Installation
# =============================================================================
install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_stage_plugins
    ${PROJECT_NAME}_component
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
ament_export_include_directories(include)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
