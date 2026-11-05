// ============================================================
// DRONEBACKEND.H
// Public interface and member declarations
// ============================================================

#ifndef DRONEBACKEND_H
#define DRONEBACKEND_H

#include <QObject>
#include <QImage>
#include <QString>

class QThread;
class SimulationWorker;

class DroneBackend : public QObject
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit DroneBackend(QObject *parent = nullptr);
    ~DroneBackend() override;

    // Starts the simulation thread (ROS2 node init + spinning). The GUI
    // should show "Waiting for simulation to start..." until
    // connectionStatusChanged(true) fires.
    void start();

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    void sendMove(double forward, double right, double up, double yawRate);
    void sendTakeoff();
    void sendLand();
    void sendReturnToHome();
    void sendEmergencyStop(); // zero-velocity command only, landing not integrated yet
    void setManualModeActive(bool active);

    // -------------------------------------------------------
    // SIGNALS
    // -------------------------------------------------------
signals:
    void frameReady(const QImage &frame);
    void dronePositionChanged(double x, double y);
    void bearDetected(const QString &bearId, double x, double y);
    void bearLost(const QString &bearId);
    void droneNameChanged(const QString &name);
    void deterrentDeployedChanged(bool deployed);
    void perimeterStatusChanged(int status);
    void perimeterAlertReceived(const QString &message);
    void droneStatusChanged(const QString &status);

    // True once SimulationWorker::initialized() fires (the ROS2 node is up).
    void connectionStatusChanged(bool connected);

    // Separate from the above - true while a publisher exists on the
    // camera topic. Kept separate because the camera sensor and the rest
    // of the simulation may come online at different times.
    void videoConnectionChanged(bool connected);

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QThread *m_thread;
    SimulationWorker *m_worker; // lives on m_thread, not the GUI thread
};



#endif // DRONEBACKEND_H
