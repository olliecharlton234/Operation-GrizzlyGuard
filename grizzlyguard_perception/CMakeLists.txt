cmake_minimum_required(VERSION 3.8)
project(grizzlyguard_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# Pure image-processing library (thermal_detector.hpp/.cpp).
#
# This has NO ROS dependency at all -- it only needs OpenCV -- so it can be
# linked into the live ROS node, the standalone demo executable, and the
# gtest unit tests below, keeping the perception logic testable without ROS
# or the simulator.
# ---------------------------------------------------------------------------
add_library(thermal_detector
  src/thermal_detector.cpp
)
target_include_directories(thermal_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(thermal_detector ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# ROS 2 node: subscribes to a camera topic, runs the pipeline, publishes the
# GrizzlyGuard perception topics.
# ---------------------------------------------------------------------------
add_executable(thermal_perception_node
  src/thermal_perception_node.cpp
  src/thermal_perception_node_main.cpp
)
target_link_libraries(thermal_perception_node thermal_detector)
ament_target_dependencies(thermal_perception_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# ---------------------------------------------------------------------------
# Standalone demo / testing-fallback executable. No ROS runtime required:
# it generates synthetic frames and runs them through the same
# thermal_detector functions used by the ROS node above.
# ---------------------------------------------------------------------------
add_executable(thermal_demo
  src/thermal_demo_main.cpp
)
target_link_libraries(thermal_demo thermal_detector)

install(TARGETS
  thermal_perception_node
  thermal_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS thermal_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

  # Unit tests for the pure image-processing pipeline (no ROS required).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_thermal_detector test/test_thermal_detector.cpp)
  target_link_libraries(test_thermal_detector thermal_detector)
endif()

ament_package()
