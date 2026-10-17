#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>
#include <QImage>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// ============================================================================
// This is now a real rclcpp::Node (not just a placeholder), since the
// simulation is Gazebo + ROS2. It runs on its own QThread (see
// DroneBackend) and integrates with Qt's event loop via a QTimer calling
// rclcpp::spin_some() periodically - a common pattern for combining ROS2
// with a Qt GUI without needing a second raw thread just for ROS2 spinning.
//
// WHAT'S REAL vs. STILL A PLACEHOLDER:
//   - The camera subscription (topic name, message type, image conversion)
//     is real, since "Gazebo publishes camera images as sensor_msgs/Image"
//     is a known, standard thing - just confirm/adjust kCameraTopic below
//     to match your actual camera sensor's topic name.
//   - Telemetry (position/battery/bear detection) and outgoing commands
//     (move/takeoff/land/etc.) are STILL fake/placeholder, since those
//     depend on decisions not yet made (see SIMULATION_INTERFACE.md) - the
//     drone control stack, bear detection message format, and whether
//     position is local X/Y or GPS lat/lon.
//
// THREADING: everything in this class runs on the worker thread. Never
// touch a QWidget from here - see the note in dronebackend.h.
// ============================================================================
class SimulationWorker : public QObject
{
    Q_OBJECT

public:
    explicit SimulationWorker(QObject *parent = nullptr);
    ~SimulationWorker() override;

public slots:
    void initialize(); // runs once the worker thread starts

    // --- INCOMING commands from the GUI. Still placeholders - see
    // SIMULATION_INTERFACE.md for what's needed to make these real. ---
    void onMoveRequested(double forward, double right, double up, double yawRate);
    void onTakeoffRequested();
    void onLandRequested();
    void onReturnToHomeRequested();
    void onEmergencyStopRequested(); // cancels current action, lands immediately - NOT an instant motor cut
    void onManualModeChanged(bool active);

signals:
    void initialized();
    void frameReady(const QImage &frame);           // REAL - from the ROS2 camera subscription
    void videoConnectionChanged(bool connected);      // REAL - true while a publisher is present on the camera topic
    void dronePositionChanged(double latitude, double longitude); // still placeholder/demo
    void batteryChanged(int percent);                 // still placeholder/demo
    void bearDetected(const QString &bearId, double latitude, double longitude); // still placeholder/demo
    void bearLost(const QString &bearId);
    void droneNameChanged(const QString &name);
    void deterrentDeployedChanged(bool deployed);     // still placeholder/demo

private slots:
    void spinRos();   // pumps ROS2 callbacks - called periodically by a QTimer
    void demoTick();  // DEMO ONLY - telemetry placeholder, delete once real data exists

private:
    void onImageMessage(const sensor_msgs::msg::Image::SharedPtr msg);
    QImage convertRosImageToQImage(const sensor_msgs::msg::Image::SharedPtr &msg) const;

    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSub;
    class QTimer *m_rosSpinTimer = nullptr;
    bool m_wasVideoConnected = false;

    // --- DEMO ONLY (telemetry placeholder) ---
    class QTimer *m_demoTimer = nullptr;
    double m_demoLat = -33.8688;
    double m_demoLon = 151.2093;
    int m_demoTickCount = 0;
};

#endif // SIMULATIONWORKER_H
