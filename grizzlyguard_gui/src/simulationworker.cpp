// ============================================================
// SIMULATIONWORKER.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "simulationworker.h"

#include <QTimer>
#include <QDebug>
#include <QThread>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
// CONFIRM/ADJUST these to match your actual Gazebo camera sensor's topic
// name - this is a common default for a camera plugin, not guaranteed to
// match your specific model/world file.
constexpr const char *kCameraTopic = "/parrot1/camera/image";
constexpr const char *kCmdVelTopic = "/cmd_vel";   // CHECK THIS: the drone may use a different topic
constexpr double kMaxLinearSpeed = 0.3;   // m/s at full key press or full stick
constexpr double kMaxAngularSpeed = 1.0;  // rad/s at full key press or full stick
// The automatic-movement code subscribes to this. true = operator is flying
// manually, so it must stop sending commands; false = it may resume.
constexpr const char *kManualModeTopic = "/manual_mode";
constexpr const char *kWorldPoseTopic = "/parrot1/world_pose";
constexpr const char *kBearDetectionsTopic = "/bears/detections";
constexpr const char *kBearLostTopic = "/bears/lost";
constexpr const char *kPerimeterStatusTopic = "/perimeter/status";
constexpr const char *kPerimeterAlertTopic = "/perimeter/alert";
constexpr const char *kDeterrentStatusTopic = "/deterrent/status";
constexpr const char *kDroneStatusTopic = "/drone/status";
constexpr const char *kTakeoffTopic = "/drone/takeoff";
constexpr const char *kLandTopic = "/drone/land";
constexpr const char *kReturnHomeTopic = "/drone/return_to_home";
constexpr const char *kEmergencyStopTopic = "/drone/emergency_stop";
constexpr int kRosSpinIntervalMs = 10; // ~100Hz - plenty for camera + telemetry callbacks

// --- Manual-mode takeoff / land (world-frame z from /parrot1/world_pose) ---
// Keep kManualTakeoffAltitude equal to patrol_node's cruise_altitude so the
// drone doesn't change height when manual mode is released.
constexpr double kManualTakeoffAltitude = 2.0; // m
constexpr double kManualLandAltitude = 0.3;    // m, same as patrol_node land_altitude
constexpr double kVerticalGain = 0.8;          // (m/s) per m of error
constexpr double kVerticalMaxSpeed = 0.5;      // m/s
constexpr double kVerticalTolerance = 0.15;    // m, "reached" band
constexpr int kVerticalIntervalMs = 50;        // 20 Hz
}

// ============================================================
// SIMULATION WORKER
// ============================================================
SimulationWorker::SimulationWorker(QObject *parent)
    : QObject(parent)
{
    // Don't touch rclcpp here - the node is created in initialize(), after
    // this object has actually moved onto the worker thread.
}

// ============================================================
// ~SIMULATION WORKER
// ============================================================
SimulationWorker::~SimulationWorker()
{
    // m_node and m_imageSub clean up via shared_ptr automatically.
}

// ============================================================
// INITIALIZE
// ============================================================
void SimulationWorker::initialize()
{
    qDebug() << "[SimulationWorker] Initializing on thread:" << QThread::currentThread();

    // rclcpp::init(argc, argv) must already have been called once in
    // main.cpp before this runs - see main.cpp.
    m_node = std::make_shared<rclcpp::Node>("bear_drone_gui_node");

    // --- REAL: camera subscription ---
    m_imageSub = m_node->create_subscription<sensor_msgs::msg::Image>(
        kCameraTopic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImageMessage(msg); });

    m_cmdVelPub = m_node->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

    // --- Integration subscriptions ---
    m_worldPoseSub = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        kWorldPoseTopic, 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onWorldPoseMessage(msg); });
    m_bearDetectionsSub = m_node->create_subscription<visualization_msgs::msg::MarkerArray>(
        kBearDetectionsTopic, 10,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { onBearMarkersMessage(msg); });
    m_bearLostSub = m_node->create_subscription<std_msgs::msg::String>(
        kBearLostTopic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit bearLost(QString::fromStdString(msg->data));
        });
    m_perimeterStatusSub = m_node->create_subscription<std_msgs::msg::UInt8>(
        kPerimeterStatusTopic, 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            emit perimeterStatusChanged(static_cast<int>(msg->data));
        });
    m_perimeterAlertSub = m_node->create_subscription<std_msgs::msg::String>(
        kPerimeterAlertTopic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit perimeterAlertReceived(QString::fromStdString(msg->data));
        });
    m_deterrentStatusSub = m_node->create_subscription<std_msgs::msg::Bool>(
        kDeterrentStatusTopic, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            emit deterrentDeployedChanged(msg->data);
        });
    m_droneStatusSub = m_node->create_subscription<std_msgs::msg::String>(
        kDroneStatusTopic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            emit droneStatusChanged(QString::fromStdString(msg->data));
        });

    // --- Integration command publishers ---
    m_takeoffPub = m_node->create_publisher<std_msgs::msg::Empty>(kTakeoffTopic, 10);
    m_landPub = m_node->create_publisher<std_msgs::msg::Empty>(kLandTopic, 10);
    m_returnHomePub = m_node->create_publisher<std_msgs::msg::Empty>(kReturnHomeTopic, 10);
    m_emergencyStopPub = m_node->create_publisher<std_msgs::msg::Empty>(kEmergencyStopTopic, 10);

    // --- REAL: manual/automatic hand-off flag. transient_local means a node that
    // starts later still receives the latest value straight away. ---
    m_manualModePub = m_node->create_publisher<std_msgs::msg::Bool>(
        kManualModeTopic, rclcpp::QoS(1).transient_local().reliable());
    publishManualMode(false); // GUI starts in automatic mode

    // Pump ROS2 callbacks periodically, cooperatively with Qt's event loop
    // on this thread - avoids needing a second raw thread just for spinning.
    m_rosSpinTimer = new QTimer(this);
    connect(m_rosSpinTimer, &QTimer::timeout, this, &SimulationWorker::spinRos);
    m_rosSpinTimer->start(kRosSpinIntervalMs);

    // Manual takeoff/land controller; only runs while a climb/descent is active.
    m_verticalTimer = new QTimer(this);
    m_verticalTimer->setInterval(kVerticalIntervalMs);
    connect(m_verticalTimer, &QTimer::timeout, this, &SimulationWorker::verticalTick);

    emit droneNameChanged("DRN-01");
    emit initialized();
}

// ============================================================
// SPIN ROS
// ============================================================
void SimulationWorker::spinRos()
{
    rclcpp::spin_some(m_node);

    // Report whether anyone is actually publishing on the camera topic -
    // this is ROS2's equivalent of "connected", since topics don't have a
    // handshake the way a TCP socket does.
    bool isConnected = m_imageSub->get_publisher_count() > 0;
    if (isConnected != m_wasVideoConnected) {
        m_wasVideoConnected = isConnected;
        emit videoConnectionChanged(isConnected);
    }
}

// ============================================================
// ON IMAGE MESSAGE
// ============================================================
void SimulationWorker::onImageMessage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // NOTE: this callback runs during rclcpp::spin_some() above, on this
    // worker thread - safe to emit Qt signals from here.
    QImage frame = convertRosImageToQImage(msg);
    if (!frame.isNull())
        emit frameReady(frame);
}

// ============================================================
// ON WORLD POSE MESSAGE
// ============================================================
void SimulationWorker::onWorldPoseMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // This topic contract requires positions expressed in the Gazebo world frame.
    // Do not feed /parrot1/odom directly into the map: its frame is parrot1_odom.
    m_currentZ = msg->pose.position.z;
    m_haveZ = true;
    emit dronePositionChanged(msg->pose.position.x, msg->pose.position.y);
}

// ============================================================
// ON BEAR MARKERS MESSAGE
// ============================================================
void SimulationWorker::onBearMarkersMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (const auto &marker : msg->markers) {
        const QString id = QString::fromStdString(marker.ns) + ":" + QString::number(marker.id);
        if (marker.action == visualization_msgs::msg::Marker::DELETE) {
            emit bearLost(id);
        } else if (marker.action == visualization_msgs::msg::Marker::ADD ||
                   marker.action == visualization_msgs::msg::Marker::MODIFY) {
            emit bearDetected(id, marker.pose.position.x, marker.pose.position.y);
        }
    }
}

// ============================================================
// CONVERT ROS IMAGE TO QIMAGE
// ============================================================
QImage SimulationWorker::convertRosImageToQImage(const sensor_msgs::msg::Image::SharedPtr &msg) const
{
    // Handles the most common encodings a Gazebo camera plugin publishes.
    // If your camera uses a different encoding, add a case here - the
    // `msg->encoding` string tells you which one it actually is (log it
    // once if unsure: qDebug() << QString::fromStdString(msg->encoding)).
    const QString encoding = QString::fromStdString(msg->encoding);
    const uchar *data = msg->data.data();
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const int stride = static_cast<int>(msg->step);


    if (encoding == "32FC1") {
        if (width <= 0 || height <= 0 ||
            static_cast<size_t>(msg->step) <
                static_cast<size_t>(width) * sizeof(float) ||
            msg->data.size() <
                static_cast<size_t>(msg->step) * msg->height) {
            qWarning() << "[SimulationWorker] Invalid depth image";
            return QImage();
        }

        QImage image(width, height, QImage::Format_Grayscale8);
        if (image.isNull())
            return QImage();

        constexpr float maxDepthMetres = 10.0f;

        for (int y = 0; y < height; ++y) {
            const uchar *row =
                data + static_cast<size_t>(y) * msg->step;
            uchar *output = image.scanLine(y);

            for (int x = 0; x < width; ++x) {
                const uchar *pixel =
                    row + static_cast<size_t>(x) * sizeof(float);

                quint32 bits = msg->is_bigendian
                    ? qFromBigEndian<quint32>(pixel)
                    : qFromLittleEndian<quint32>(pixel);

                float depth;
                std::memcpy(&depth, &bits, sizeof(depth));

                if (!std::isfinite(depth) || depth <= 0.0f) {
                    output[x] = 0;
                } else {
                    const float scaled =
                        qBound(0.0f, depth / maxDepthMetres, 1.0f);
                    output[x] =
                        static_cast<uchar>(255.0f * (1.0f - scaled));
                }
            }
        }

        return image;
    }


    QImage::Format format;
    if (encoding == "rgb8")
        format = QImage::Format_RGB888;
    else if (encoding == "bgr8")
        format = QImage::Format_BGR888;
    else if (encoding == "mono8")
        format = QImage::Format_Grayscale8;
    else if (encoding == "rgba8")
        format = QImage::Format_RGBA8888;
    else {
        qWarning() << "[SimulationWorker] Unsupported image encoding:" << encoding
                   << "- add a case in convertRosImageToQImage()";
        return QImage();
    }

    // .copy() is essential here - `data` points into msg's buffer, which
    // goes out of scope (msg is a local shared_ptr in this call chain)
    // once this function returns, so QImage needs its own deep copy.
    return QImage(data, width, height, stride, format).copy();
}


// ============================================================
// ON MOVE REQUESTED
// ============================================================
void SimulationWorker::onMoveRequested(double forward, double right, double up, double yawRate)
{
    // Drive input only counts in manual mode. Outside it, the automatic code
    // is in charge and the GUI must not fight it (this also covers the gamepad,
    // which can send input on any page).
    if (!m_manualMode) {
        qDebug() << "[SimulationWorker] move ignored: not in manual mode";
        return;
    }

    m_inForward = forward;
    m_inRight = right;
    m_inYaw = yawRate;

    if (!std::isnan(m_verticalTarget)) {
        if (up != 0.0) {
            // Operator pressed Up/Down: they take over altitude again.
            stopManualVertical();
            emit droneStatusChanged("MANUAL: " + m_verticalLabel + " cancelled by operator");
        } else {
            // Takeoff/land in progress: verticalTick() publishes the
            // combined command (operator x/y/yaw + controller z).
            return;
        }
    }

    qDebug() << "[SimulationWorker] move" << forward << right << up << yawRate;
    publishVelocity(forward, right, up, yawRate);
}

// ============================================================
// PUBLISH VELOCITY
// ============================================================
void SimulationWorker::publishVelocity(double forward, double right, double up, double yawRate)
{
    publishTwist(forward, right, up * kMaxLinearSpeed, yawRate);
}

// ============================================================
// PUBLISH TWIST
// ============================================================
void SimulationWorker::publishTwist(double forward, double right, double verticalSpeed, double yawRate)
{
    if (!m_cmdVelPub)
        return; // ROS node not up yet

    // GUI inputs run from -1 to 1 (yaw positive = turn right). ROS uses +y LEFT
    // and positive angular z = turn LEFT, so right and yaw are flipped here.
    // verticalSpeed is already in m/s.
    geometry_msgs::msg::Twist msg;
    msg.linear.x = forward * kMaxLinearSpeed;
    msg.linear.y = -right * kMaxLinearSpeed;
    msg.linear.z = verticalSpeed;
    msg.angular.z = -yawRate * kMaxAngularSpeed;
    m_cmdVelPub->publish(msg);
}

// ============================================================
// PUBLISH MANUAL MODE
// ============================================================
void SimulationWorker::publishManualMode(bool manual)
{
    if (!m_manualModePub)
        return; // ROS node not up yet

    std_msgs::msg::Bool msg;
    msg.data = manual;
    m_manualModePub->publish(msg);
}

// ============================================================
// START / STOP MANUAL VERTICAL (takeoff / land in manual mode)
// ============================================================
void SimulationWorker::startManualVertical(double targetZ, const QString &label)
{
    if (!m_haveZ) {
        qWarning() << "[SimulationWorker] manual" << label
                   << "needs altitude from" << kWorldPoseTopic << "- is world_pose_publisher running?";
        emit droneStatusChanged("MANUAL: " + label + " FAILED (no altitude data)");
        return;
    }
    m_verticalTarget = targetZ;
    m_verticalLabel = label;
    emit droneStatusChanged(QString("MANUAL: %1 to %2 m").arg(label).arg(targetZ, 0, 'f', 1));
    m_verticalTimer->start();
}

void SimulationWorker::stopManualVertical()
{
    if (m_verticalTimer)
        m_verticalTimer->stop();
    m_verticalTarget = NAN;
}

// ============================================================
// VERTICAL TICK
// ============================================================
void SimulationWorker::verticalTick()
{
    if (!m_manualMode || std::isnan(m_verticalTarget)) {
        stopManualVertical();
        return;
    }

    const double err = m_verticalTarget - m_currentZ;
    if (std::abs(err) < kVerticalTolerance) {
        publishTwist(m_inForward, m_inRight, 0.0, m_inYaw); // stop climbing/sinking
        const QString done = (m_verticalLabel == "landing") ? "landed" : "takeoff complete";
        emit droneStatusChanged(QString("MANUAL: %1 (%2 m)").arg(done).arg(m_currentZ, 0, 'f', 1));
        stopManualVertical();
        return;
    }

    const double vz = std::clamp(kVerticalGain * err, -kVerticalMaxSpeed, kVerticalMaxSpeed);
    publishTwist(m_inForward, m_inRight, vz, m_inYaw);
}


// ============================================================
// ON TAKEOFF REQUESTED
// ============================================================
void SimulationWorker::onTakeoffRequested()
{
    qDebug() << "[SimulationWorker] publishing takeoff request";
    // Always tell patrol_node (in manual mode it just records it).
    if (m_takeoffPub) m_takeoffPub->publish(std_msgs::msg::Empty{});
    if (m_manualMode)
        startManualVertical(kManualTakeoffAltitude, "takeoff");
}

// ============================================================
// ON LAND REQUESTED
// ============================================================
void SimulationWorker::onLandRequested()
{
    qDebug() << "[SimulationWorker] publishing land request";
    if (m_landPub) m_landPub->publish(std_msgs::msg::Empty{});
    if (m_manualMode)
        startManualVertical(kManualLandAltitude, "landing");
}

// ============================================================
// ON RETURN TO HOME REQUESTED
// ============================================================
void SimulationWorker::onReturnToHomeRequested()
{
    qDebug() << "[SimulationWorker] publishing return-to-home request";
    if (m_returnHomePub) m_returnHomePub->publish(std_msgs::msg::Empty{});
}

// ============================================================
// ON EMERGENCY STOP REQUESTED
// ============================================================
void SimulationWorker::onEmergencyStopRequested()
{
    qDebug() << "[SimulationWorker] publishing emergency-stop request and zero velocity";
    stopManualVertical();
    if (m_emergencyStopPub) m_emergencyStopPub->publish(std_msgs::msg::Empty{});
    publishVelocity(0.0, 0.0, 0.0, 0.0);
}

// ============================================================
// ON MANUAL MODE CHANGED
// ============================================================
void SimulationWorker::onManualModeChanged(bool active)
{
    qDebug() << "[SimulationWorker] manual mode active:" << active;
    m_manualMode = active;
    stopManualVertical();
    m_inForward = m_inRight = m_inYaw = 0.0;
    if (active) {
        // Tell the automatic movement code to go quiet, then cancel whatever
        // it last commanded so the drone does not keep flying the old command.
        publishManualMode(true);
        publishVelocity(0.0, 0.0, 0.0, 0.0);
    } else {
        // Stop the drone first, then hand control back to the automatic code.
        publishVelocity(0.0, 0.0, 0.0, 0.0);
        publishManualMode(false);
    }
}