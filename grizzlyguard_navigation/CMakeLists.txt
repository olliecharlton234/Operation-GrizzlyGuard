cmake_minimum_required(VERSION 3.8)
project(grizzlyguard_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Layout: everything lives in src/
#   src/patrol_node.py  src/patrol_path.py  src/route_loader.py
#   src/patrol_routes.yaml
#   src/altitude_controller.cpp   (optional)

# ---------------------------------------------------------------- C++ nodes
# Only .cpp files go in add_executable(). Built only if the file exists.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/altitude_controller.cpp)
  add_executable(altitude_controller src/altitude_controller.cpp)
  target_compile_features(altitude_controller PUBLIC cxx_std_17)
  ament_target_dependencies(altitude_controller rclcpp geometry_msgs nav_msgs std_msgs)
  install(TARGETS altitude_controller DESTINATION lib/${PROJECT_NAME})
endif()

# ------------------------------------------------------------- Python nodes
# Python is NOT compiled - install(PROGRAMS) just copies it so `ros2 run`
# can find it. (A .py in add_executable() gives "Cannot determine link language".)
install(PROGRAMS
  src/patrol_node.py
  src/patrol_path.py
  src/route_loader.py
  src/world_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------- route config
# patrol_node.py looks for share/grizzlyguard_navigation/config/patrol_routes.yaml
install(FILES src/patrol_routes.yaml DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
