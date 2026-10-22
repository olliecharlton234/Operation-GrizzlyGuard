// ============================================================
// SIMULATIONWORKER.H
// Public interface and member declarations
// ============================================================

#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>
#include <QImage>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

/**
 * @brief ROS 2 transport worker running in DroneBackend's QThread.
 *
 * Implemented: camera frames, manual velocity, manual-mode flag.
 * Awaiting team interfaces: world-frame pose, bear tracking, perimeter,
 * deterrent and higher-level flight commands. No demo telemetry is emitted.
 */
class SimulationWorker : public QObject
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit SimulationWorker(QObject *parent = nullptr);
    ~SimulationWorker() override;

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    void initialize(); // runs once the worker thread starts

    // Movement and manual mode are functional; higher-level commands await integration.
    void onMoveRequested(double forward, double right, double up, double yawRate);
    void onTakeoffRequested();
    void onLandRequested();
    void onReturnToHomeRequested();
    void onEmergencyStopRequested(); // currently sends zero velocity only
    void onManualModeChanged(bool active);

    // -------------------------------------------------------
    // SIGNALS
    // -------------------------------------------------------
signals:
    void initialized();
    void frameReady(const QImage &frame);           // ROS 2 camera subscription
    void videoConnectionChanged(bool connected);      // REAL - true while a publisher is present on the camera topic
    void dronePositionChanged(double x, double y); // reserved for calibrated world-frame data
    void bearDetected(const QString &bearId, double x, double y); // reserved for perception integration
    void bearLost(const QString &bearId);
    void droneNameChanged(const QString &name);
    void deterrentDeployedChanged(bool deployed);     // emitted only when real data is integrated

    // -------------------------------------------------------
    // PRIVATE SLOTS
    // -------------------------------------------------------
private slots:
    void spinRos();   // pumps ROS2 callbacks - called periodically by a QTimer

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    void onImageMessage(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishVelocity(double forward, double right, double up, double yawRate);
    void publishManualMode(bool manual);
    QImage convertRosImageToQImage(const sensor_msgs::msg::Image::SharedPtr &msg) const;

    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub; // drives the drone in Gazebo
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_manualModePub;   // tells the automatic code to stand down
    bool m_manualMode = false; // true while the operator is flying manually
    class QTimer *m_rosSpinTimer = nullptr;
    bool m_wasVideoConnected = false;

};

#endif // SIMULATIONWORKER_H
