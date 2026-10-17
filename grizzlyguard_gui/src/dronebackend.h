#ifndef DRONEBACKEND_H
#define DRONEBACKEND_H

#include <QObject>
#include <QImage>
#include <QString>

class QThread;
class SimulationWorker;

// ============================================================================
// ADAPTER LAYER - the ONLY class MainWindow/widgets talk to. It owns a
// background QThread running SimulationWorker, which is a real rclcpp::Node
// (Gazebo + ROS2) - see simulationworker.h for what's real (camera) vs.
// still placeholder (telemetry/commands, pending decisions documented in
// SIMULATION_INTERFACE.md).
//
// WHY A SEPARATE OBJECT AT ALL, IF IT JUST FORWARDS?
//   - Keeps MainWindow ignorant of threading/ROS2 details - it just calls
//     m_backend->sendMove(...) and connects to m_backend's signals.
//   - If the simulation's interface changes, only this file's forwarding
//     connections (and simulationworker.cpp) need updating, not MainWindow.
//
// THREADING: DroneBackend itself lives on the GUI/main thread. Qt signal-
// slot connections between objects on different threads are automatically
// delivered safely (queued) by Qt's event loop - no manual locking needed.
// ============================================================================
class DroneBackend : public QObject
{
    Q_OBJECT

public:
    explicit DroneBackend(QObject *parent = nullptr);
    ~DroneBackend() override;

    // Starts the simulation thread (ROS2 node init + spinning). The GUI
    // should show "Waiting for simulation to start..." until
    // connectionStatusChanged(true) fires.
    void start();

public slots:
    void sendMove(double forward, double right, double up, double yawRate);
    void sendTakeoff();
    void sendLand();
    void sendReturnToHome();
    void sendEmergencyStop(); // stops current action and commands an immediate landing - NOT an instant motor cut
    void setManualModeActive(bool active);

signals:
    void frameReady(const QImage &frame);
    void dronePositionChanged(double latitude, double longitude);
    void batteryChanged(int percent);
    void bearDetected(const QString &bearId, double latitude, double longitude);
    void bearLost(const QString &bearId);
    void droneNameChanged(const QString &name);
    void deterrentDeployedChanged(bool deployed);

    // True once SimulationWorker::initialized() fires (the ROS2 node is up).
    void connectionStatusChanged(bool connected);

    // Separate from the above - true while a publisher exists on the
    // camera topic. Kept separate because the camera sensor and the rest
    // of the simulation may come online at different times.
    void videoConnectionChanged(bool connected);

private:
    QThread *m_thread;
    SimulationWorker *m_worker; // lives on m_thread, not the GUI thread
};



#endif // DRONEBACKEND_H
