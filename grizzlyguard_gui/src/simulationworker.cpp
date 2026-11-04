// ============================================================
// SIMULATIONWORKER.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "simulationworker.h"

#include <QTimer>
#include <QDebug>
#include <QThread>
#include <QtEndian>
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
constexpr int kRosSpinIntervalMs = 10; // ~100Hz - plenty for camera + telemetry callbacks
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

    // subscription for 

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
    qDebug() << "[SimulationWorker] move" << forward << right << up << yawRate;
    publishVelocity(forward, right, up, yawRate);
}

// ============================================================
// PUBLISH VELOCITY
// ============================================================
void SimulationWorker::publishVelocity(double forward, double right, double up, double yawRate)
{
    if (!m_cmdVelPub)
        return; // ROS node not up yet

    // GUI inputs run from -1 to 1 (yaw positive = turn right). ROS uses +y LEFT
    // and positive angular z = turn LEFT, so right and yaw are flipped here.
    geometry_msgs::msg::Twist msg;
    msg.linear.x = forward * kMaxLinearSpeed;
    msg.linear.y = -right * kMaxLinearSpeed;
    msg.linear.z = up * kMaxLinearSpeed;
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
// ON TAKEOFF REQUESTED
// ============================================================
void SimulationWorker::onTakeoffRequested()
{
    qDebug() << "[SimulationWorker] takeoff";
    // TODO: depends on drone control stack - see SIMULATION_INTERFACE.md
}

// ============================================================
// ON LAND REQUESTED
// ============================================================
void SimulationWorker::onLandRequested()
{
    qDebug() << "[SimulationWorker] land";
    // TODO: depends on drone control stack
}

// ============================================================
// ON RETURN TO HOME REQUESTED
// ============================================================
void SimulationWorker::onReturnToHomeRequested()
{
    qDebug() << "[SimulationWorker] return to home";
    // TODO: depends on drone control stack
}

// ============================================================
// ON EMERGENCY STOP REQUESTED
// ============================================================
void SimulationWorker::onEmergencyStopRequested()
{
    qDebug() << "[SimulationWorker] Emergency landing NOT IMPLEMENTED; sending zero velocity only";
    // IMPORTANT: cancel current action then land in place - NOT an instant
    // motor cut, and NOT return-to-home-then-land. See SIMULATION_INTERFACE.md.
    publishVelocity(0.0, 0.0, 0.0, 0.0);
    // No landing request is sent until the simulation supports one.
}

// ============================================================
// ON MANUAL MODE CHANGED
// ============================================================
void SimulationWorker::onManualModeChanged(bool active)
{
    qDebug() << "[SimulationWorker] manual mode active:" << active;
    m_manualMode = active;
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
    // Autonomous controller must subscribe to /manual_mode to honor this flag.
}
