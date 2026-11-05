// ============================================================
// SIMULATIONWORKER.H
// Public interface and member declarations
// ============================================================

#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>
#include <QImage>
#include <QString>

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/empty.hpp>

class QTimer;

/**
 * @brief ROS 2 transport worker running in DroneBackend's QThread.
 *
 * ROS 2 transport for camera, manual control, world pose, bear tracking,
 * perimeter status/alerts, deterrent status, drone status and flight
 * command requests.
 *
 * Takeoff / Land in MANUAL mode are flown here: the GUI owns /cmd_vel while
 * the operator is flying, so it climbs/descends itself using the z from
 * /parrot1/world_pose. Outside manual mode they are only forwarded to
 * patrol_node as /drone/takeoff and /drone/land.
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

    void onMoveRequested(double forward, double right, double up, double yawRate);
    void onTakeoffRequested();
    void onLandRequested();
    void onReturnToHomeRequested();
    void onEmergencyStopRequested(); // publishes the emergency-stop request and zero velocity
    void onManualModeChanged(bool active);

    // -------------------------------------------------------
    // SIGNALS
    // -------------------------------------------------------
signals:
    void initialized();
    void frameReady(const QImage &frame);           // ROS 2 camera subscription
    void videoConnectionChanged(bool connected);      // true while a publisher is present on the camera topic
    void dronePositionChanged(double x, double y); // Gazebo world-frame position
    void bearDetected(const QString &bearId, double x, double y);
    void bearLost(const QString &bearId);
    void droneNameChanged(const QString &name);
    void deterrentDeployedChanged(bool deployed);
    void perimeterStatusChanged(int status);
    void perimeterAlertReceived(const QString &message);
    void droneStatusChanged(const QString &status);

    // -------------------------------------------------------
    // PRIVATE SLOTS
    // -------------------------------------------------------
private slots:
    void spinRos();       // pumps ROS2 callbacks - called periodically by a QTimer
    void verticalTick();  // manual-mode takeoff/land controller, 20 Hz while active

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    void onImageMessage(const sensor_msgs::msg::Image::SharedPtr msg);
    void onWorldPoseMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onBearMarkersMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void publishVelocity(double forward, double right, double up, double yawRate);
    void publishTwist(double forward, double right, double verticalSpeed, double yawRate);
    void publishManualMode(bool manual);
    void startManualVertical(double targetZ, const QString &label);
    void stopManualVertical();
    QImage convertRosImageToQImage(const sensor_msgs::msg::Image::SharedPtr &msg) const;

    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_worldPoseSub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_bearDetectionsSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_bearLostSub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_perimeterStatusSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_perimeterAlertSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_deterrentStatusSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_droneStatusSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub; // drives the drone in Gazebo
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_manualModePub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_takeoffPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_landPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_returnHomePub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_emergencyStopPub;
    bool m_manualMode = false; // true while the operator is flying manually
    QTimer *m_rosSpinTimer = nullptr;
    bool m_wasVideoConnected = false;

    // --- Manual-mode takeoff / land ---
    QTimer *m_verticalTimer = nullptr;
    double m_currentZ = 0.0;          // from /parrot1/world_pose
    bool m_haveZ = false;
    double m_verticalTarget = NAN;    // NAN = no takeoff/land in progress
    QString m_verticalLabel;
    // Latest operator input (-1..1), so WASD/yaw keep working during a climb.
    double m_inForward = 0.0, m_inRight = 0.0, m_inYaw = 0.0;
};

#endif // SIMULATIONWORKER_H