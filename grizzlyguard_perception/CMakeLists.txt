cmake_minimum_required(VERSION 3.8)
project(grizzlyguard_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---------------------------------------------------------------------------
# ONNX Runtime (C++). There is no apt/rosdep package, so it is downloaded once with
#   scripts/install_onnxruntime.sh        (installs to ~/onnxruntime)
# or pointed to with  -DONNXRUNTIME_ROOT=/path  or the ONNXRUNTIME_ROOT environment variable.
# OpenCV's cv::dnn is NOT used: OpenCV 4.5/4.6 (Humble) cannot run YOLOv8-family ONNX heads.
# ---------------------------------------------------------------------------
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime install prefix")
set(_ort_hints
  ${ONNXRUNTIME_ROOT}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime
  $ENV{HOME}/onnxruntime)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${_ort_hints} PATH_SUFFIXES include include/onnxruntime)
find_library(ONNXRUNTIME_LIBRARY onnxruntime HINTS ${_ort_hints} PATH_SUFFIXES lib lib64)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found. Install it once with:\n"
    "  bash src/<repo>/grizzlyguard_perception/scripts/install_onnxruntime.sh\n"
    "or pass -DONNXRUNTIME_ROOT=/path/to/onnxruntime (folder containing include/ and lib/).")
endif()
get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
add_library(onnxruntime_imported SHARED IMPORTED)
set_target_properties(onnxruntime_imported PROPERTIES
  IMPORTED_LOCATION ${ONNXRUNTIME_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES ${ONNXRUNTIME_INCLUDE_DIR})

# ROS-independent detector core (preprocessing, decoding, NMS, ONNX Runtime session).
add_library(yolo_detector STATIC src/yolo_detector.cpp)
set_target_properties(yolo_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(yolo_detector PUBLIC ${OpenCV_LIBS} onnxruntime_imported)

# Live ROS 2 node.
add_executable(bear_yolo_node src/bear_yolo_node.cpp)
target_link_libraries(bear_yolo_node yolo_detector)
ament_target_dependencies(bear_yolo_node
  rclcpp std_msgs sensor_msgs vision_msgs cv_bridge ament_index_cpp)

# Headless offline check on image files (no ROS needed at runtime).
add_executable(yolo_image_demo src/yolo_image_demo.cpp)
target_link_libraries(yolo_image_demo yolo_detector)

# Find libonnxruntime.so at runtime without LD_LIBRARY_PATH.
set_target_properties(bear_yolo_node yolo_image_demo PROPERTIES
  BUILD_RPATH ${ONNXRUNTIME_LIB_DIR}
  INSTALL_RPATH ${ONNXRUNTIME_LIB_DIR})

install(TARGETS bear_yolo_node yolo_image_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Formatting-only linters, not part of the functional tests for this package.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolo_detector test/test_yolo_detector.cpp)
  target_link_libraries(test_yolo_detector yolo_detector)
  set_target_properties(test_yolo_detector PROPERTIES BUILD_RPATH ${ONNXRUNTIME_LIB_DIR})
endif()

ament_package()