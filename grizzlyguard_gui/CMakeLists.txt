cmake_minimum_required(VERSION 3.19)
project(grizzlyguard_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)       # makes this a real ROS 2 package for colcon
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)          # /parrot1/odometry subscription (step 3)
find_package(SDL2 REQUIRED)              # gamepad: sudo apt install libsdl2-dev
find_package(visualization_msgs REQUIRED)

# ---- Executable ----
add_executable(BearDroneGUI
    src/main.cpp
    resources.qrc
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
    src/perimeteralertdialog.h
    src/perimeteralertdialog.cpp
)

target_include_directories(BearDroneGUI PRIVATE src ${SDL2_INCLUDE_DIRS})

# Plain (non-keyword) signature on purpose: Humble's ament_target_dependencies
# uses the plain form, and CMake refuses to mix plain and PRIVATE/PUBLIC.
target_link_libraries(BearDroneGUI Qt6::Widgets ${SDL2_LIBRARIES})
ament_target_dependencies(BearDroneGUI rclcpp sensor_msgs geometry_msgs std_msgs nav_msgs)

if(TARGET visualization_msgs::visualization_msgs)
    target_link_libraries(BearDroneGUI PRIVATE visualization_msgs::visualization_msgs)
else()
    target_include_directories(BearDroneGUI PRIVATE ${visualization_msgs_INCLUDE_DIRS})
    target_link_libraries(BearDroneGUI PRIVATE ${visualization_msgs_LIBRARIES})
endif()

# std_msgs (manual-mode flag and integration messages) is linked the same way.
if(TARGET std_msgs::std_msgs)
    target_link_libraries(BearDroneGUI PRIVATE std_msgs::std_msgs)
else()
    target_include_directories(BearDroneGUI PRIVATE ${std_msgs_INCLUDE_DIRS})
    target_link_libraries(BearDroneGUI PRIVATE ${std_msgs_LIBRARIES})
endif()

ament_package()
