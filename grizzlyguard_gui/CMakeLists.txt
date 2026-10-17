cmake_minimum_required(VERSION 3.19)
project(bear2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

# ROS2 (for the Gazebo simulation link). Requires the ROS2 environment to be
# sourced before running cmake, e.g.:
#   source /opt/ros/humble/setup.bash   (or whichever distro you're using)
# This is plain CMake (not ament_cmake/colcon) - if this ends up needing to
# be a proper ROS2 package built via colcon instead, ask and this file can
# be converted to use ament_cmake + a package.xml.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Gamepad/controller support uses SDL2 (Qt6 has no built-in gamepad module).
# Install it first:
#   Ubuntu/Debian: sudo apt install libsdl2-dev
#   macOS (brew):  brew install sdl2
#   Windows:       via vcpkg: vcpkg install sdl2
find_package(SDL2 REQUIRED)

add_executable(BearDroneGUI
    src/main.cpp
    src/mainwindow.h
    src/mainwindow.cpp
    src/cameraviewwidget.h
    src/cameraviewwidget.cpp
    src/mapwidget.h
    src/mapwidget.cpp
    src/trafficlightwidget.h
    src/trafficlightwidget.cpp
    src/deterrentindicatorwidget.h
    src/deterrentindicatorwidget.cpp
    src/telemetrypanel.h
    src/telemetrypanel.cpp
    src/manualcontrolwidget.h
    src/manualcontrolwidget.cpp
    src/gamepadcontroller.h
    src/gamepadcontroller.cpp
    src/dronebackend.h
    src/dronebackend.cpp
    src/simulationworker.h
    src/simulationworker.cpp
    src/perimeteralertdialog.cpp
    src/perimeteralertdialog.h
)

target_link_libraries(BearDroneGUI PRIVATE Qt6::Widgets)

# rclcpp/sensor_msgs are found via plain find_package above (not ament), so
# link them directly. NOTE: the exact CMake target name for message
# packages like sensor_msgs varies slightly between ROS2 distros (Humble/
# Iron/Jazzy) - if this line fails to configure, run
# `cmake --find-package -DNAME=sensor_msgs -DCOMPILER_ID=GNU -DLANGUAGE=CXX
# -DMODE=EXIST` or just check `ros2 pkg xml sensor_msgs` / the distro's
# docs for the correct target name and swap it in below.
target_link_libraries(BearDroneGUI PRIVATE rclcpp::rclcpp)
if(TARGET sensor_msgs::sensor_msgs)
    target_link_libraries(BearDroneGUI PRIVATE sensor_msgs::sensor_msgs)
else()
    target_include_directories(BearDroneGUI PRIVATE ${sensor_msgs_INCLUDE_DIRS})
endif()

# SDL2's CMake package sometimes exposes SDL2::SDL2, sometimes only the
# SDL2_INCLUDE_DIRS/SDL2_LIBRARIES variables depending on platform/version -
# this handles both.
if(TARGET SDL2::SDL2)
    target_link_libraries(BearDroneGUI PRIVATE SDL2::SDL2)
else()
    target_include_directories(BearDroneGUI PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(BearDroneGUI PRIVATE ${SDL2_LIBRARIES})
endif()
