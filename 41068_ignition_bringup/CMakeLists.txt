cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

# pose_bridge: a resident set_pose client for the bear gait demo.
# Shelling out to `ign service` costs ~330 ms per call, which caps the
# animation near 1.4 Hz and looks visibly jerky. Going through the transport
# library directly costs ~0.5 ms, so the gait can run at 30 Hz and look smooth.
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_executable(pose_bridge src/pose_bridge.cc)
target_link_libraries(pose_bridge
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

install(TARGETS pose_bridge DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch models urdf_husky worlds urdf_parrot
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dynamic_world_demo.py scripts/basic_autonomy_demo.py
           scripts/bear_gait_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
