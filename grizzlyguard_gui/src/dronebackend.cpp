#include "dronebackend.h"
#include "simulationworker.h"

#include <QThread>
#include <QDebug>

DroneBackend::DroneBackend(QObject *parent)
    : QObject(parent)
{
    m_thread = new QThread(this);
    m_worker = new SimulationWorker(); // no parent - moveToThread requires a parentless object

    m_worker->moveToThread(m_thread);

    connect(m_thread, &QThread::started, m_worker, &SimulationWorker::initialize);
    connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);

    connect(m_worker, &SimulationWorker::initialized, this, [this]() {
        emit connectionStatusChanged(true);
    });
    connect(m_worker, &SimulationWorker::frameReady, this, &DroneBackend::frameReady);
    connect(m_worker, &SimulationWorker::videoConnectionChanged, this, &DroneBackend::videoConnectionChanged);
    connect(m_worker, &SimulationWorker::dronePositionChanged, this, &DroneBackend::dronePositionChanged);
    connect(m_worker, &SimulationWorker::batteryChanged, this, &DroneBackend::batteryChanged);
    connect(m_worker, &SimulationWorker::bearDetected, this, &DroneBackend::bearDetected);
    connect(m_worker, &SimulationWorker::bearLost, this, &DroneBackend::bearLost);
    connect(m_worker, &SimulationWorker::droneNameChanged, this, &DroneBackend::droneNameChanged);
    connect(m_worker, &SimulationWorker::deterrentDeployedChanged, this, &DroneBackend::deterrentDeployedChanged);
}

DroneBackend::~DroneBackend()
{
    m_thread->quit();
    m_thread->wait();
}

void DroneBackend::start()
{
    emit connectionStatusChanged(false);
    m_thread->start();
}

void DroneBackend::sendMove(double forward, double right, double up, double yawRate)
{
    QMetaObject::invokeMethod(m_worker, "onMoveRequested", Qt::QueuedConnection,
                              Q_ARG(double, forward), Q_ARG(double, right),
                              Q_ARG(double, up), Q_ARG(double, yawRate));
}

void DroneBackend::sendTakeoff()
{
    QMetaObject::invokeMethod(m_worker, "onTakeoffRequested", Qt::QueuedConnection);
}

void DroneBackend::sendLand()
{
    QMetaObject::invokeMethod(m_worker, "onLandRequested", Qt::QueuedConnection);
}

void DroneBackend::sendReturnToHome()
{
    QMetaObject::invokeMethod(m_worker, "onReturnToHomeRequested", Qt::QueuedConnection);
}

void DroneBackend::sendEmergencyStop()
{
    QMetaObject::invokeMethod(m_worker, "onEmergencyStopRequested", Qt::QueuedConnection);
}

void DroneBackend::setManualModeActive(bool active)
{
    QMetaObject::invokeMethod(m_worker, "onManualModeChanged", Qt::QueuedConnection,
                              Q_ARG(bool, active));
}