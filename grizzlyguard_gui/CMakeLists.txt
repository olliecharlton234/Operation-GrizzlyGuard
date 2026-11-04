cmake_minimum_required(VERSION 3.19)
project(grizzlyguard_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Plain CMake project (not colcon). Source ROS 2 before configuring.

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SDL2 REQUIRED)

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

target_include_directories(BearDroneGUI PRIVATE src)

# Every call uses PRIVATE - CMake won't mix plain and keyword forms.
target_link_libraries(BearDroneGUI PRIVATE Qt6::Widgets)
target_link_libraries(BearDroneGUI PRIVATE rclcpp::rclcpp)

foreach(msg_pkg sensor_msgs geometry_msgs std_msgs nav_msgs visualization_msgs)
    if(TARGET ${msg_pkg}::${msg_pkg})
        target_link_libraries(BearDroneGUI PRIVATE ${msg_pkg}::${msg_pkg})
    else()
        target_include_directories(BearDroneGUI PRIVATE ${${msg_pkg}_INCLUDE_DIRS})
        target_link_libraries(BearDroneGUI PRIVATE ${${msg_pkg}_LIBRARIES})
    endif()
endforeach()

if(TARGET SDL2::SDL2)
    target_link_libraries(BearDroneGUI PRIVATE SDL2::SDL2)
else()
    target_include_directories(BearDroneGUI PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(BearDroneGUI PRIVATE ${SDL2_LIBRARIES})
endif()
