// ============================================================
// MAIN.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include <QApplication>
#include "mainwindow.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    // Must happen before any rclcpp::Node is created (SimulationWorker
    // creates one once its thread starts) - and before QApplication so
    // Qt-specific argv flags don't confuse ROS2's arg parsing, or vice versa.
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    app.setApplicationName("Bear Drone Monitor");

    MainWindow window;
    window.resize(1280, 800);
    window.show();

    int result = app.exec();

    rclcpp::shutdown();
    return result;
}
