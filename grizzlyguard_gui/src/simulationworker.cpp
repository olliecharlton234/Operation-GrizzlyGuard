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
constexpr int kRosSpinIntervalMs = 10; // ~100Hz - plenty for camera + telemetry callbacks
}

SimulationWorker::SimulationWorker(QObject *parent)
    : QObject(parent)
{
    // Don't touch rclcpp here - the node is created in initialize(), after
    // this object has actually moved onto the worker thread.
}

SimulationWorker::~SimulationWorker()
{
    // m_node and m_imageSub clean up via shared_ptr automatically.
}

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

    // Pump ROS2 callbacks periodically, cooperatively with Qt's event loop
    // on this thread - avoids needing a second raw thread just for spinning.
    m_rosSpinTimer = new QTimer(this);
    connect(m_rosSpinTimer, &QTimer::timeout, this, &SimulationWorker::spinRos);
    m_rosSpinTimer->start(kRosSpinIntervalMs);

    // --- STILL PLACEHOLDER: telemetry demo data (see SIMULATION_INTERFACE.md) ---
    m_demoTimer = new QTimer(this);
    connect(m_demoTimer, &QTimer::timeout, this, &SimulationWorker::demoTick);
    m_demoTimer->start(1000);

    emit droneNameChanged("DRN-01");
    emit initialized();
}

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

void SimulationWorker::onImageMessage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // NOTE: this callback runs during rclcpp::spin_some() above, on this
    // worker thread - safe to emit Qt signals from here.
    QImage frame = convertRosImageToQImage(msg);
    if (!frame.isNull())
        emit frameReady(frame);
}

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

// ============================================================================
// INCOMING commands from the GUI - STILL PLACEHOLDERS. Replace with real
// publishers/service calls once the drone control stack is decided (see
// SIMULATION_INTERFACE.md).
// ============================================================================

void SimulationWorker::onMoveRequested(double forward, double right, double up, double yawRate)
{
    qDebug() << "[SimulationWorker] move" << forward << right << up << yawRate;
    // TODO: publish geometry_msgs/Twist on cmd_vel (or whatever your stack uses)
}

void SimulationWorker::onTakeoffRequested()
{
    qDebug() << "[SimulationWorker] takeoff";
    // TODO: depends on drone control stack - see SIMULATION_INTERFACE.md
}

void SimulationWorker::onLandRequested()
{
    qDebug() << "[SimulationWorker] land";
    // TODO: depends on drone control stack
}

void SimulationWorker::onReturnToHomeRequested()
{
    qDebug() << "[SimulationWorker] return to home";
    // TODO: depends on drone control stack
}

void SimulationWorker::onEmergencyStopRequested()
{
    qDebug() << "[SimulationWorker] EMERGENCY STOP - cancelling current action and landing";
    // IMPORTANT: cancel current action then land in place - NOT an instant
    // motor cut, and NOT return-to-home-then-land. See SIMULATION_INTERFACE.md.
    onLandRequested(); // placeholder: reuses the same land path as the Land button
}

void SimulationWorker::onManualModeChanged(bool active)
{
    qDebug() << "[SimulationWorker] manual mode active:" << active;
    // TODO: switch simulated drone between autonomous/guided and manual control
}

// ============================================================================
// DEMO DATA ONLY (telemetry placeholder) - delete this function, m_demoTimer,
// and the four m_demo* fields once real telemetry subscriptions exist.
// The camera feed above is already real and independent of this.
// ============================================================================
void SimulationWorker::demoTick()
{
    m_demoLat += 0.0002;
    emit dronePositionChanged(m_demoLat, m_demoLon);

    emit bearDetected("bear-1", m_demoLat + 0.0015, m_demoLon + 0.0008);

    ++m_demoTickCount;
    if (m_demoTickCount % 10 == 0)
        emit deterrentDeployedChanged(m_demoTickCount % 20 == 0);
}
